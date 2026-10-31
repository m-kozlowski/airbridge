#include "sd_storage.h"

#include <Arduino.h>
#include <algorithm>
#include <errno.h>
#include <string.h>

#include "board.h"
#include "debug_log.h"
#include "usb_storage.h"

#if AB_STORAGE_HAS_SDCARD
#include <limits>
#include <unistd.h>
#include <FS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#if AB_STORAGE_SPI_SD_ENABLED
#include "sd_spi_card.h"
#include "sd_spi_hooks.h"
#else
#include <SD_MMC.h>
#include <driver/sdmmc_host.h>
#include <driver/sdmmc_defs.h>
#include <sdmmc_cmd.h>
#endif
#include <esp_vfs_fat.h>
#include <ff.h>
#include <diskio_impl.h>
#if !AB_STORAGE_SPI_SD_ENABLED
#include <diskio_sdmmc.h>
#endif
#include <esp_heap_caps.h>
#include "memory_manager.h"
#include "uart_arbiter.h"
#include "airsense_state.h"
#include "airbridge_ota.h"
#include "resmed_ota.h"
#endif

namespace SdStorage {

static Status status = {
    AB_STORAGE_HAS_SDCARD != 0,
    false,
    0,
    0,
    0,
    "",
};
static portMUX_TYPE status_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t file_mutation_revision[size_t(FileScope::Count)] = {};

namespace {
bool paths_overlap(const char *left, const char *right) {
    if (!left || !*left || !right || !*right) return false;
    size_t a = strlen(left), b = strlen(right);
    while (a > 1 && left[a - 1] == '/') --a;
    while (b > 1 && right[b - 1] == '/') --b;
    const size_t common = std::min(a, b);
    if (strncasecmp(left, right, common)) return false;
    return a == b || common == 1 || (a < b ? right[a] : left[b]) == '/';
}

uint32_t file_scope_mask(const char *path) {
    constexpr uint32_t all = (1u << unsigned(FileScope::Count)) - 1;
    if (!path || path[0] != '/') return all;
    constexpr uint32_t catalog = 1u << unsigned(FileScope::Catalog);
    constexpr uint32_t recording = 1u << unsigned(FileScope::Recording);
    constexpr uint32_t reports = 1u << unsigned(FileScope::Reports);
    constexpr uint32_t exports = 1u << unsigned(FileScope::Export);
    constexpr uint32_t firmware = 1u << unsigned(FileScope::Firmware);
    constexpr uint32_t sources = recording | reports | exports;
    static constexpr struct { const char *path; uint32_t scopes; } roots[] = {
        {"/DATALOG", sources},
        {"/STR.edf", sources}, {"/STR.edf.part", sources}, {"/STR.edf.bak", sources},
        {"/Identification.tgt", sources}, {"/Identification.crc", sources},
        {"/Identification.tgt.part", sources}, {"/Identification.crc.part", sources},
        {"/Identification.tgt.bak", sources}, {"/Identification.crc.bak", sources},
        {"/airbridge/catalog.bin", catalog | sources},
        {"/airbridge/catalog.bin.part", catalog | sources},
        {"/airbridge/catalog.bin.bak", catalog | sources},
        {"/airbridge/pending", recording},
        {"/airbridge/report", reports},
        {"/airbridge/resmed-firmware", firmware},
    };
    uint32_t mask = 1u << unsigned(FileScope::All);
    for (const auto &root : roots)
        if (paths_overlap(path, root.path)) mask |= root.scopes;
    // Receipt directories are account-specific; the roots above cover ancestors.
    if (!strncasecmp(path, "/airbridge/smb_", 15) ||
        !strncasecmp(path, "/airbridge/shq_", 15)) mask |= exports;
    return mask;
}
}  // namespace

#if AB_STORAGE_HAS_SDCARD
namespace {
bool init_started = false;
bool formatting = false;
#if AB_STORAGE_SPI_SD_ENABLED
SdSpi::Card storage_card;
uint32_t spi_mount_clock_khz = 0;
char spi_mount_error[96] = {};
bool spi_power_cycle_required = false;
SpiError pending_spi;
uint32_t spi_started_ms = 0;
uint8_t spi_command = 0xff;
uint32_t spi_argument = 0;
bool spi_active = false;
#else
auto &storage_card = SD_MMC;
TaskHandle_t mount_caller = nullptr;
bool mount_driver_called = false;
esp_err_t mount_result = ESP_OK;
sdmmc_card_t *mount_card_info = nullptr;
#endif

// Caller holds status_mux. Only copy initialization results, never query the bus.
void clear_bus_status() {
    status.bus_active = false;
    status.bus_width = 0;
    status.bus_clock_khz = 0;
    status.card_type = CardType::Unknown;
#if AB_STORAGE_SPI_SD_ENABLED
    status.bus = Bus::Spi;
    status.bus_clock_limit_khz = AB_SPI_SD_FREQ_HZ / 1000;
#else
    status.bus = Bus::Sdmmc;
    status.bus_clock_limit_khz = AB_SDMMC_FREQ_KHZ;
#endif
}

#if AB_STORAGE_SPI_SD_ENABLED
void set_spi_bus_status(uint32_t clock_khz, sdcard_type_t type) {
    clear_bus_status();
    status.bus_active = type != CARD_NONE;
    status.bus_width = status.bus_active ? 1 : 0;
    status.bus_clock_khz = clock_khz;
    status.card_type = type == CARD_SDHC ? CardType::Sdhc : type == CARD_SD ? CardType::Sd :
        type == CARD_MMC ? CardType::Mmc : CardType::Unknown;
}
#else
void set_bus_status(const sdmmc_card_t *card) {
    clear_bus_status();
    status.bus_active = card != nullptr;
    // IDF narrows log_bus_width during initialization; get_bus_width is the slot limit.
    status.bus_width = card && (card->log_bus_width == 0 ||
        card->log_bus_width == 2 || card->log_bus_width == 3)
        ? (1u << card->log_bus_width) : 0;
    status.bus_clock_khz = card && card->real_freq_khz > 0 ? card->real_freq_khz : 0;
    status.bus_clock_limit_khz = AB_SDMMC_FREQ_KHZ;
    status.card_type = !card ? CardType::Unknown : card->is_mmc ? CardType::Mmc :
        card->is_sdio ? CardType::Sdio :
        (card->ocr & SD_OCR_SDHC_CAP) ? CardType::Sdhc : CardType::Sd;
}
#endif

struct Request {
    enum Kind { Run, Acquire, TryAcquire, Release, Close, Begin, End, Mount, UsbIo } kind;
    bool (*operation)(fs::FS &, void *);
    void *context;
    uint32_t generation;
    SemaphoreHandle_t done;
    bool success;
    TaskHandle_t caller;
    uint32_t file_id;
    Access access;
    const char *path;
    const char *destination;
    Session::AdmissionNotify admission_notify;
    void *admission_context;
    Session::IoObserver observer;
    void *observer_context;
    bool waiting_for_reader;
};

void mount_card();
void advance_usb();

QueueHandle_t requests = nullptr;
QueueHandle_t interactive_requests = nullptr;
QueueHandle_t background_requests = nullptr;
unsigned interactive_streak = 0;
uint32_t generation = 1;
struct Lease {
    uint32_t generation = 0;
    Access access = Access::Background;
    char read_root[256] = {};
};
// Slot zero is the sole background/exclusive owner; the other slots are readers.
Lease sessions[3];
uint32_t active_sessions = 0;
uint32_t exclusive_waiter = 0;
// A yielded owner retries within 1 s. Reserve its next turn without keeping an
// abandoned request alive forever; its resumed session gets to finish once.
constexpr uint32_t BACKGROUND_RESUME_MS = 1500;
constexpr uint32_t BACKGROUND_TURN_MS = 100;
struct BackgroundWaiter {
    uint32_t token = 0;
    uint32_t requested_ms = 0;
    Session::AdmissionNotify notify = nullptr;
    void *context = nullptr;
};
// Only the export and report workers request background admission. Other
// session copies borrow their owner's lease instead of joining this queue.
BackgroundWaiter background_waiters[2];
uint32_t first_background_waiter = 0;
uint32_t first_background_waiter_at = 0;
uint32_t background_turn_at = 0;
bool background_resume_pending = false;
uint32_t background_resume_at = 0;
uint32_t protected_background = 0;
bool reader_blocked_write = false;
uint32_t recorder_waiting = 0;
TaskHandle_t maintenance_waiter = nullptr;
bool maintenance_granted = false;
TaskHandle_t worker = nullptr;
TaskHandle_t direct_owner = nullptr;
#if AB_USB_MSC_ENABLED
#if AB_STORAGE_SPI_SD_ENABLED
uint32_t usb_sectors = 0;
#else
sdmmc_card_t usb_card = {};
#endif
bool usb_host_started = false;
bool usb_io_pending = false;
struct {
    Request request;
    bool write;
    uint32_t sector;
    uint8_t *buffer;
    size_t size;
    void (*done)(bool);
} usb_io = {};
#endif

void wake_worker() {
    if (worker) xTaskNotifyGive(worker);
}

#if !AB_STORAGE_SPI_SD_ENABLED
void configure_host(sdmmc_host_t &host, sdmmc_slot_config_t &slot) {
    host.flags = AB_SDMMC_WIDTH == 1 ? SDMMC_HOST_FLAG_1BIT : SDMMC_HOST_FLAG_4BIT;
    host.max_freq_khz = AB_SDMMC_FREQ_KHZ;
    slot.width = AB_SDMMC_WIDTH;
    slot.clk = static_cast<gpio_num_t>(AB_SDMMC_CLK_GPIO);
    slot.cmd = static_cast<gpio_num_t>(AB_SDMMC_CMD_GPIO);
    slot.d0 = static_cast<gpio_num_t>(AB_SDMMC_D0_GPIO);
    slot.d1 = static_cast<gpio_num_t>(AB_SDMMC_WIDTH == 1 ? -1 : AB_SDMMC_D1_GPIO);
    slot.d2 = static_cast<gpio_num_t>(AB_SDMMC_WIDTH == 1 ? -1 : AB_SDMMC_D2_GPIO);
    slot.d3 = static_cast<gpio_num_t>(AB_SDMMC_WIDTH == 1 ? -1 : AB_SDMMC_D3_GPIO);
}
#endif

struct OpenFile {
    fs::File file;
    uint32_t id = 0;
    uint32_t owner = 0;
    void close() { file.close(); id = owner = 0; }
};
// Background parsing needs two inputs; each browser producer needs one.
// Four readers + writer leave three of the mount's eight handles for callbacks.
OpenFile readers[4];
OpenFile writer;
char writer_path[256] = {};
uint32_t next_file = 0;

void close_files(uint32_t owner = 0) {
    for (OpenFile &reader : readers)
        if (!owner || reader.owner == owner) reader.close();
    if (!owner || writer.owner == owner) writer.close();
}

void revoke_sessions() {
    close_files();
    for (Lease &session : sessions)
        __atomic_store_n(&session.generation, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&active_sessions, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&exclusive_waiter, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&protected_background, 0, __ATOMIC_RELEASE);
    __atomic_store_n(&reader_blocked_write, false, __ATOMIC_RELEASE);
    for (auto &waiting : background_waiters) waiting = {};
    __atomic_store_n(&first_background_waiter, 0, __ATOMIC_RELEASE);
    background_resume_pending = false;
}

void refresh_background_waiters(uint32_t remove = 0) {
    size_t next = 0;
    for (const auto &waiting : background_waiters) {
        if (waiting.token && waiting.token != remove &&
            uint32_t(millis() - waiting.requested_ms) < BACKGROUND_RESUME_MS)
            background_waiters[next++] = waiting;
    }
    while (next < 2) background_waiters[next++] = {};
    __atomic_store_n(&first_background_waiter_at, background_waiters[0].requested_ms, __ATOMIC_RELEASE);
    __atomic_store_n(&first_background_waiter, background_waiters[0].token, __ATOMIC_RELEASE);
}

bool request_background_turn(Request &request) {
    for (auto &waiting : background_waiters) {
        if (waiting.token && waiting.token != request.generation) continue;
        if (!waiting.token) {
            if (++generation == 0) ++generation;
            waiting.token = request.generation = generation;
        }
        waiting.requested_ms = millis();
        waiting.notify = request.admission_notify;
        waiting.context = request.admission_context;
        refresh_background_waiters();
        return background_waiters[0].token == request.generation;
    }
    return false;
}

OpenFile *find_file(uint32_t id) {
    for (OpenFile &reader : readers)
        if (reader.id == id) return &reader;
    return writer.id == id ? &writer : nullptr;
}

uint32_t retain_file(OpenFile &slot, const fs::File &file, uint32_t owner) {
    if (++next_file == 0) ++next_file;
    slot.id = next_file;
    slot.file = file;
    slot.owner = owner;
    return slot.id;
}

bool background_allowed() {
    return local_access_allowed() && !__atomic_load_n(&recorder_waiting, __ATOMIC_ACQUIRE) &&
        AirSenseState::local_background_allowed();
}

bool firmware_allowed() {
    const auto state = Arbiter::get_state();
    return local_access_allowed() && !__atomic_load_n(&recorder_waiting, __ATOMIC_ACQUIRE) &&
        ResmedOta::is_active() && (state == SYS_IDLE || state == SYS_OTA_AIRSENSE);
}

bool background_allowed(uint32_t expected) {
    if (!expected) return false;
    for (const Lease &session : sessions)
        if (expected == __atomic_load_n(&session.generation, __ATOMIC_ACQUIRE))
            return session.access == Access::Firmware ? firmware_allowed() : background_allowed();
    return false;
}

void notify_background_waiter() {
    if (!background_waiters[0].token) return;
    refresh_background_waiters();
    if (!mounted() || !background_allowed() || maintenance_pending() ||
        direct_owner || exclusive_waiter || sessions[0].generation) return;
    auto &waiting = background_waiters[0];
    const auto notify = waiting.notify;
    waiting.notify = nullptr;
    if (notify) notify(waiting.context);
}

bool mutation_blocked(Request &request) {
    bool blocked = false;
    if (request.path && background_allowed(request.generation)) {
        for (size_t i = 1; i < 3; ++i) {
            const Lease &session = sessions[i];
            if (session.generation && (paths_overlap(request.path, session.read_root) ||
                                      paths_overlap(request.destination, session.read_root))) {
                blocked = true;
                break;
            }
        }
    }
    __atomic_store_n(&request.waiting_for_reader, blocked, __ATOMIC_RELEASE);
    if (blocked) __atomic_store_n(&protected_background, request.generation, __ATOMIC_RELEASE);
    return blocked;
}

bool read_blocked(const char *root) {
    return writer.id && paths_overlap(root, writer_path);
}

void process_request(Request &value) {
    Request *request = &value;
    request->success = false;
    if (request->kind == Request::Mount) {
        if (local_access_allowed() && !direct_owner && !active_sessions) mount_card();
        request->success = mounted();
#if AB_USB_MSC_ENABLED
    } else if (request->kind == Request::UsbIo) {
        // Already-admitted writes finish even if the host ejects/disconnects meanwhile.
#if AB_STORAGE_SPI_SD_ENABLED
        const bool valid = usb_host_started && usb_io.size && !(usb_io.size % 512) &&
            uint64_t(usb_io.sector) + usb_io.size / 512 <= usb_sectors;
        const esp_err_t result = !valid ? ESP_ERR_INVALID_STATE : usb_io.write
            ? storage_card.write_sectors(usb_io.buffer, usb_io.sector, usb_io.size / 512)
            : storage_card.read_sectors(usb_io.buffer, usb_io.sector, usb_io.size / 512);
#else
        const bool valid = usb_host_started && usb_io.size && !(usb_io.size % 512) &&
            uint64_t(usb_io.sector) + usb_io.size / 512 <= usb_card.csd.capacity;
        const esp_err_t result = !valid ? ESP_ERR_INVALID_STATE : usb_io.write
            ? sdmmc_write_sectors(&usb_card, usb_io.buffer, usb_io.sector, usb_io.size / 512)
            : sdmmc_read_sectors(&usb_card, usb_io.buffer, usb_io.sector, usb_io.size / 512);
#endif
        if (result != ESP_OK)
            Log::logf(CAT_STORAGE, LOG_WARN, "USB %s sector=%lu bytes=%u failed: %s\n",
                      usb_io.write ? "write" : "read", (unsigned long)usb_io.sector,
                      unsigned(usb_io.size), esp_err_to_name(result));
        const auto done = usb_io.done;
        __atomic_store_n(&usb_io_pending, false, __ATOMIC_RELEASE);
        done(result == ESP_OK);
#endif
    } else if (request->kind == Request::Begin) {
        refresh_background_waiters();
        if (!mounted() || !(request->access == Access::Firmware ? firmware_allowed() : background_allowed())) return;
        const bool background_turn = request->access == Access::Background &&
            request_background_turn(*request);
        if (maintenance_pending()) return;
        if (request->access == Access::Exclusive || request->access == Access::Firmware) {
            if (!exclusive_waiter) {
                if (protected_background || (background_resume_pending &&
                    uint32_t(millis() - background_resume_at) < BACKGROUND_RESUME_MS)) return;
                background_resume_pending = false;
                if (++generation == 0) ++generation;
                __atomic_store_n(&exclusive_waiter, generation, __ATOMIC_RELEASE);
                request->generation = generation;
            }
            if (exclusive_waiter != request->generation || active_sessions || direct_owner) return;
        } else if (direct_owner || exclusive_waiter) {
            if (exclusive_waiter && request->access == Access::Background) {
                background_resume_pending = true;
            }
            return;
        }

        Lease *slot = nullptr;
        if (request->access == Access::ReadOnly) {
            Request *pending = nullptr;
            if (protected_background || (background_resume_pending &&
                uint32_t(millis() - background_resume_at) < BACKGROUND_RESUME_MS) ||
                xQueuePeek(background_requests, &pending, 0) == pdTRUE) return;
            if (read_blocked(request->path)) return;
            for (size_t i = 1; i < 3; ++i)
                if (!sessions[i].generation) { slot = &sessions[i]; break; }
        } else if (request->access == Access::Background) {
            if (!background_turn || sessions[0].generation) return;
            slot = &sessions[0];
            refresh_background_waiters(request->generation);
        } else if (!sessions[0].generation) slot = &sessions[0];
        if (!slot) return;
        if (request->path && strlen(request->path) >= sizeof(slot->read_root)) return;
        if (request->access != Access::Exclusive && request->access != Access::Firmware) {
            if (++generation == 0) ++generation;
            request->generation = generation;
        }
        snprintf(slot->read_root, sizeof(slot->read_root), "%s", request->path ? request->path : "");
        slot->access = request->access;
        __atomic_store_n(&slot->generation, request->generation, __ATOMIC_RELEASE);
        __atomic_add_fetch(&active_sessions, 1, __ATOMIC_RELEASE);
        if (request->access == Access::Background) {
            __atomic_store_n(&background_turn_at, millis(), __ATOMIC_RELEASE);
            if (background_resume_pending &&
                uint32_t(millis() - background_resume_at) < BACKGROUND_RESUME_MS)
                __atomic_store_n(&protected_background, request->generation, __ATOMIC_RELEASE);
            background_resume_pending = false;
        }
        request->success = true;
    } else if (request->kind == Request::End) {
        refresh_background_waiters(request->generation);
        for (Lease &session : sessions) {
            if (!request->generation || session.generation != request->generation) continue;
            if (&session == &sessions[0] && request->generation != exclusive_waiter && exclusive_waiter)
                background_resume_pending = true;
            close_files(request->generation);
            __atomic_store_n(&session.generation, 0, __ATOMIC_RELEASE);
            __atomic_sub_fetch(&active_sessions, 1, __ATOMIC_RELEASE);
        }
        if (protected_background == request->generation)
            __atomic_store_n(&protected_background, 0, __ATOMIC_RELEASE);
        if (exclusive_waiter == request->generation) {
            __atomic_store_n(&exclusive_waiter, 0, __ATOMIC_RELEASE);
            background_resume_at = millis();
        }
        request->success = true;
    } else if (request->kind == Request::Acquire || request->kind == Request::TryAcquire) {
        if (local_access_allowed() && mounted() && !direct_owner &&
            (request->kind == Request::Acquire || !active_sessions)) {
            revoke_sessions();
            __atomic_store_n(&direct_owner, request->caller, __ATOMIC_RELEASE);
            request->success = true;
        }
    } else if (request->kind == Request::Close) {
        OpenFile *file = find_file(request->file_id);
        if (file) file->close();
        request->success = true;
    } else if (request->kind == Request::Release) {
        if (direct_owner == request->caller) {
            __atomic_store_n(&direct_owner, nullptr, __ATOMIC_RELEASE);
            request->success = true;
        }
    } else if (!direct_owner && background_allowed(request->generation) && mounted()) {
        request->success = request->operation(storage_card, request->context);
    }
}

Request *next_request() {
    Request *request = nullptr;
    if (xQueueReceive(requests, &request, 0) == pdTRUE) return request;
    Request *background = nullptr;
    const bool background_pending = xQueuePeek(background_requests, &background, 0) == pdTRUE;
    const bool blocked = background_pending && mutation_blocked(*background);
    __atomic_store_n(&reader_blocked_write, blocked, __ATOMIC_RELEASE);
    const bool background_ready = background_pending && !blocked;
    if (interactive_streak < 3 || !background_ready) {
        if (xQueueReceive(interactive_requests, &request, 0) == pdTRUE) {
            if (interactive_streak < 3) ++interactive_streak;
            return request;
        }
    }
    if (background_ready && xQueueReceive(background_requests, &request, 0) == pdTRUE) {
        interactive_streak = 0;
        return request;
    }
    return nullptr;
}

void io_task(void *) {
    while (true) {
        advance_usb();
        notify_background_waiter();
        Request *request = next_request();
        if (!request) { ulTaskNotifyTake(pdTRUE, portMAX_DELAY); continue; }
        process_request(*request);
        if (request->done) xSemaphoreGive(request->done);
    }
}

bool init_worker() {
    if (worker) return true;
    if (!requests) requests = xQueueCreate(4, sizeof(Request *));
    if (!interactive_requests) interactive_requests = xQueueCreate(4, sizeof(Request *));
    if (!background_requests) background_requests = xQueueCreate(4, sizeof(Request *));
    if (!requests || !interactive_requests || !background_requests) return false;
    BaseType_t created = pdFAIL;
    if (aircannect::Memory::psram_available())
        created = xTaskCreatePinnedToCoreWithCaps(io_task, "sd_io", 4096,
            nullptr, 1, &worker, 0, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (created != pdPASS)
        created = xTaskCreatePinnedToCore(io_task, "sd_io", 4096,
            nullptr, 1, &worker, 0);
    if (created != pdPASS) worker = nullptr;
    return worker != nullptr;
}

bool direct_request(Request &request) {
    // No executor means no auxiliary handles; keep the same ownership token.
    if (request.kind == Request::Acquire || request.kind == Request::TryAcquire) {
        if (!local_access_allowed() || !mounted()) return false;
        TaskHandle_t expected = nullptr;
        return __atomic_compare_exchange_n(&direct_owner, &expected, request.caller,
            false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
    }
    if (request.kind == Request::Release) {
        TaskHandle_t expected = request.caller;
        return __atomic_compare_exchange_n(&direct_owner, &expected, nullptr,
            false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
    }
    return false;
}

bool dispatch(Request &request, bool control) {
    request.caller = xTaskGetCurrentTaskHandle();
    if (!worker) return direct_request(request);
    if (request.caller == worker) return false;
    StaticSemaphore_t done_state;
    request.done = xSemaphoreCreateBinaryStatic(&done_state);
    request.success = false;
    Request *pointer = &request;
    QueueHandle_t queue = control ? requests : request.access == Access::Background
        ? background_requests : interactive_requests;
    const bool queued = xQueueSend(queue, &pointer, control ? portMAX_DELAY : 0) == pdTRUE;
    if (queued) {
        wake_worker();
        // Keep the queued request and its stack context alive. Observation does
        // not cancel an in-flight filesystem call or replay a pending mutation.
        while (xSemaphoreTake(request.done, request.observer
                   ? pdMS_TO_TICKS(250) : portMAX_DELAY) != pdTRUE) {
            request.observer(request.observer_context,
                __atomic_load_n(&request.waiting_for_reader, __ATOMIC_ACQUIRE)
                    ? IoEvent::WaitingForReader : IoEvent::Waiting, request.path);
        }
    }
    vSemaphoreDelete(request.done);
    if (queued && request.success && request.kind == Request::Run && request.observer)
        request.observer(request.observer_context, IoEvent::Complete, request.path);
    return queued && request.success;
}

void close_handle(uint32_t handle, Session::IoObserver observer = nullptr, void *context = nullptr) {
    if (!handle) return;
    Request request = {};
    request.kind = Request::Close;
    request.file_id = handle;
    request.observer = observer;
    request.observer_context = context;
    (void)dispatch(request, true);
}
}  // namespace

#if !AB_STORAGE_SPI_SD_ENABLED
extern "C" esp_err_t __real_esp_vfs_fat_sdmmc_mount(
    const char *, const sdmmc_host_t *, const sdmmc_slot_config_t *,
    const esp_vfs_fat_mount_config_t *, sdmmc_card_t **);

// SD_MMC.begin() discards the result and hides the card. Capture both only for
// the owning attempt; the card is read only after begin succeeds, then forgotten.
extern "C" esp_err_t __wrap_esp_vfs_fat_sdmmc_mount(
    const char *path, const sdmmc_host_t *host, const sdmmc_slot_config_t *slot,
    const esp_vfs_fat_mount_config_t *config, sdmmc_card_t **card) {
    const esp_err_t result = __real_esp_vfs_fat_sdmmc_mount(path, host, slot, config, card);
    const TaskHandle_t caller = __atomic_load_n(&mount_caller, __ATOMIC_ACQUIRE);
    if (caller && xTaskGetCurrentTaskHandle() == caller) {
        mount_driver_called = true;
        mount_result = result;
        mount_card_info = result == ESP_OK && card ? *card : nullptr;
    }
    return result;
}
#else
// The storage lease serializes disk operations; only completed faults are published.
void spi_io_begin(uint8_t drive, uint32_t sector, uint32_t count, bool write) {
    portENTER_CRITICAL(&status_mux);
    spi_active = !status.spi_error.valid;
    portEXIT_CRITICAL(&status_mux);
    if (!spi_active) return;
    pending_spi = {};
    pending_spi.drive = drive;
    pending_spi.sector = sector;
    pending_spi.count = count;
    pending_spi.write = write;
    spi_command = 0xff;
    spi_argument = 0;
    spi_started_ms = millis();
}

void spi_io_command(uint8_t command, uint32_t argument) {
    if (!spi_active) return;
    spi_command = command;
    spi_argument = argument;
}

void spi_io_fault(const char *phase, int response) {
    if (!spi_active || pending_spi.valid) return;
    const int saved_errno = errno;
    pending_spi.valid = true;
    pending_spi.occurred_ms = millis();
    pending_spi.command = spi_command;
    pending_spi.argument = spi_argument;
    pending_spi.response = response;
    snprintf(pending_spi.phase, sizeof(pending_spi.phase), "%s", phase);
    errno = saved_errno;
}

void spi_io_end(int disk_result) {
    if (!spi_active) return;
    if (disk_result != 0) spi_io_fault("disk_result", disk_result);
    if (pending_spi.valid) {
        pending_spi.disk_result = disk_result;
        pending_spi.elapsed_ms = millis() - spi_started_ms;
        portENTER_CRITICAL(&status_mux);
        status.spi_error = pending_spi;
        portEXIT_CRITICAL(&status_mux);
    }
    spi_active = false;
}

void spi_init_event(uint32_t requested_hz, uint32_t attempt_hz,
                    uint32_t actual_hz, bool selected, const char *reason,
                    bool power_cycle_required) {
    const int saved_errno = errno;
    spi_power_cycle_required |= power_cycle_required;
    spi_mount_clock_khz = selected ? actual_hz / 1000 : 0;
    snprintf(spi_mount_error, sizeof(spi_mount_error), "%s", selected ? "" : reason);
    Log::logf(CAT_STORAGE, selected ? LOG_INFO : LOG_WARN,
        "SPI SD init requested_hz=%lu attempt_hz=%lu actual_hz=%lu result=%s reason=%s\n",
        (unsigned long)requested_hz, (unsigned long)attempt_hz, (unsigned long)actual_hz,
        selected ? "selected" : "failed", reason);
    errno = saved_errno;
}
#endif

bool write_exact(fs::File &file, const uint8_t *data, size_t size) {
    return file && file.write(data, size) == size;
}

static void mount_error(const char *message) {
    portENTER_CRITICAL(&status_mux);
    strncpy(status.error, message, sizeof(status.error) - 1);
    status.error[sizeof(status.error) - 1] = 0;
    portEXIT_CRITICAL(&status_mux);
}
#endif

bool factory_format() {
#if AB_STORAGE_HAS_SDCARD
    portENTER_CRITICAL(&status_mux);
    const bool allowed = !init_started && !formatting && !status.mounted &&
        !__atomic_load_n(&direct_owner, __ATOMIC_ACQUIRE) &&
        !__atomic_load_n(&active_sessions, __ATOMIC_ACQUIRE);
    if (allowed) formatting = true;
    portEXIT_CRITICAL(&status_mux);
    if (!allowed) {
        mount_error("format requires pre-init boot");
        Log::logf(CAT_STORAGE, LOG_WARN, "format requires pre-init boot\n");
        return false;
    }

    constexpr size_t work_bytes = 4096;
    static_assert(FF_MAX_SS <= work_bytes, "SD format work buffer too small");
#if !AB_STORAGE_SPI_SD_ENABLED
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    configure_host(host, slot);

    sdmmc_card_t card = {};
#endif
    uint32_t sectors = 0;
    BYTE pdrv = FF_DRV_NOT_USED;
    PARTITION previous_partition = {};
    void *work = nullptr;
    bool host_started = false;
    bool disk_registered = false;
    const char *step = "host state";
    int result = ESP_OK;
    const bool formatted = [&]() {
#if !AB_STORAGE_SPI_SD_ENABLED
        sdmmc_host_state_t host_state = {};
        result = sdmmc_host_get_state(&host_state);
        if (result != ESP_OK) return false;
        if (host_state.host_initialized) {
            result = ESP_ERR_INVALID_STATE;
            return false;
        }
#endif

        // DMA-capable internal memory avoids card-sized allocation and bounce I/O.
        step = "buffer";
        work = heap_caps_malloc(work_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!work) { result = ESP_ERR_NO_MEM; return false; }
#if AB_STORAGE_SPI_SD_ENABLED
        step = "SPI init";
        if (spi_power_cycle_required) { result = ESP_ERR_INVALID_STATE; return false; }
        if (!storage_card.begin_raw()) { result = ESP_FAIL; return false; }
        host_started = true;
        pdrv = storage_card.drive();
        step = "card type";
        if (storage_card.cardType() != CARD_SD && storage_card.cardType() != CARD_SDHC) {
            result = ESP_ERR_NOT_SUPPORTED;
            return false;
        }
        step = "card geometry";
        sectors = storage_card.numSectors();
        if (storage_card.sectorSize() != 512 || sectors < 2) {
            result = ESP_ERR_NOT_SUPPORTED;
            return false;
        }
#else
        step = "drive";
        result = ff_diskio_get_drive(&pdrv);
        if (result != ESP_OK) return false;
        step = "host init";
        result = sdmmc_host_init();
        if (result != ESP_OK) return false;
        host_started = true;
        step = "slot init";
        result = sdmmc_host_init_slot(host.slot, &slot);
        if (result != ESP_OK) return false;
        step = "card init";
        result = sdmmc_card_init(&host, &card);
        if (result != ESP_OK) return false;
        step = "card type";
        // MMC can trigger a whole-volume trim in the SDK diskio formatter path.
        if (card.is_mmc || card.is_sdio) {
            result = ESP_ERR_NOT_SUPPORTED;
            return false;
        }
        step = "card geometry";
        if (card.csd.sector_size != 512 || card.csd.capacity < 2) {
            result = ESP_ERR_NOT_SUPPORTED;
            return false;
        }

        ff_diskio_register_sdmmc(pdrv, &card);
        sectors = card.csd.capacity;
#endif
        disk_registered = true;
        previous_partition = VolToPart[pdrv];
        VolToPart[pdrv] = {pdrv, 1};
        const char drive[] = {static_cast<char>('0' + pdrv), ':', 0};
        const LBA_t partitions[] = {100, 0, 0, 0};
        step = "partition";
        result = f_fdisk(pdrv, partitions, work);
        if (result != FR_OK) return false;
        // f_fdisk writes only the MBR; invalidate old GPT headers without a full erase.
        memset(work, 0, 512);
        step = "GPT headers";
#if AB_STORAGE_SPI_SD_ENABLED
        result = storage_card.write_sectors(static_cast<const uint8_t *>(work), 1, 1);
        if (result != ESP_OK) return false;
        result = storage_card.write_sectors(static_cast<const uint8_t *>(work), sectors - 1, 1);
#else
        result = sdmmc_write_sectors(&card, work, 1, 1);
        if (result != ESP_OK) return false;
        result = sdmmc_write_sectors(&card, work, sectors - 1, 1);
#endif
        if (result != ESP_OK) return false;
        // Force partition 1 of the new table, never an old detected volume or SFD.
        const MKFS_PARM options = {FM_FAT32, 2, 0, 0, 0};
        step = "FAT32";
        result = f_mkfs(drive, &options, work, work_bytes);
        return result == FR_OK;
    }();

    if (disk_registered) {
        VolToPart[pdrv] = previous_partition;
#if !AB_STORAGE_SPI_SD_ENABLED
        ff_diskio_unregister(pdrv);
#endif
    }
    bool success = formatted;
    if (host_started) {
#if AB_STORAGE_SPI_SD_ENABLED
        const esp_err_t shutdown = storage_card.end_raw();
#else
        const esp_err_t shutdown = sdmmc_host_deinit();
#endif
        if (shutdown != ESP_OK) {
            Log::logf(CAT_STORAGE, LOG_ERROR, "format host shutdown failed (%d)\n", shutdown);
            if (success) { step = "host shutdown"; result = shutdown; }
            success = false;
        }
    }
    heap_caps_free(work);

    char error[sizeof(status.format_error)] = {};
    if (!success) snprintf(error, sizeof(error), "format %s failed (%d)", step, result);
    mount_error(error);
    portENTER_CRITICAL(&status_mux);
    status.format_result = success ? FormatResult::Succeeded : FormatResult::Failed;
    memcpy(status.format_error, error, sizeof(status.format_error));
    formatting = false;
    portEXIT_CRITICAL(&status_mux);
    Log::logf(CAT_STORAGE, success ? LOG_INFO : LOG_ERROR,
              success ? "new SD partition table and empty FAT32 created\n" : "%s\n", error);
    return success;
#else
    return false;
#endif
}

void init() {
#if AB_STORAGE_HAS_SDCARD
    portENTER_CRITICAL(&status_mux);
    const bool busy = formatting;
    if (!busy) init_started = true;
    if (!status.bus_active) clear_bus_status();
    portEXIT_CRITICAL(&status_mux);
    if (busy) {
        Log::logf(CAT_STORAGE, LOG_WARN, "mount deferred during factory format\n");
        return;
    }
    if (mounted() || !local_access_allowed()) return;
    if (!init_worker()) {
        Log::logf(CAT_STORAGE, LOG_WARN, "auxiliary I/O unavailable; recorder only\n");
        mount_card();
        return;
    }
#if AB_USB_MSC_ENABLED
    portENTER_CRITICAL(&status_mux);
    status.usb_supported = true;
    portEXIT_CRITICAL(&status_mux);
#endif
    Request request = {};
    request.kind = Request::Mount;
    dispatch(request, true);
#endif
}

#if AB_STORAGE_HAS_SDCARD
namespace {
void mount_card() {
    if (mounted()) return;

    portENTER_CRITICAL(&status_mux);
    clear_bus_status();
    portEXIT_CRITICAL(&status_mux);
#if AB_STORAGE_SPI_SD_ENABLED
    // The driver object is freed after failed mount; uncertainty survives it.
    if (spi_power_cycle_required) {
        mount_error("mount failed: SPI card power cycle required");
        return;
    }
    spi_mount_clock_khz = 0;
    spi_mount_error[0] = 0;
    const bool success = storage_card.begin();
    if (!success || storage_card.cardType() == CARD_NONE) {
        storage_card.end();
        char error[sizeof(status.error)];
        snprintf(error, sizeof(error), "mount failed: SPI %.70s",
                 spi_mount_error[0] ? spi_mount_error : "pin/bus/card setup");
        mount_error(error);
        Log::logf(CAT_STORAGE, LOG_ERROR, "%s\n", error);
        return;
    }
    const sdcard_type_t type = storage_card.cardType();
#else
    mount_card_info = nullptr;

    bool pins_ok;
    if (AB_SDMMC_WIDTH == 1) {
        pins_ok = SD_MMC.setPins(AB_SDMMC_CLK_GPIO, AB_SDMMC_CMD_GPIO,
                                 AB_SDMMC_D0_GPIO);
    } else {
        pins_ok = SD_MMC.setPins(AB_SDMMC_CLK_GPIO, AB_SDMMC_CMD_GPIO,
                                 AB_SDMMC_D0_GPIO, AB_SDMMC_D1_GPIO,
                                 AB_SDMMC_D2_GPIO, AB_SDMMC_D3_GPIO);
    }
    if (!pins_ok) {
        mount_error("pin configuration failed");
        Log::logf(CAT_STORAGE, LOG_ERROR, "pin configuration failed\n");
        return;
    }

    const bool one_bit = AB_SDMMC_WIDTH == 1;
    mount_driver_called = false;
    mount_result = ESP_OK;
    __atomic_store_n(&mount_caller, xTaskGetCurrentTaskHandle(), __ATOMIC_RELEASE);
    const bool success = SD_MMC.begin("/sdcard", one_bit, false, AB_SDMMC_FREQ_KHZ, 8);
    __atomic_store_n(&mount_caller, nullptr, __ATOMIC_RELEASE);
    if (!success || SD_MMC.cardType() == CARD_NONE) {
        mount_card_info = nullptr;
        if (success) SD_MMC.end();
        char error[sizeof(status.error)];
        if (mount_driver_called && mount_result != ESP_OK) {
            snprintf(error, sizeof(error), "mount failed: %s (0x%x)",
                     esp_err_to_name(mount_result), static_cast<unsigned>(mount_result));
        } else if (success) {
            snprintf(error, sizeof(error), "no card");
        } else {
            snprintf(error, sizeof(error), "mount failed: SDMMC %s",
                     mount_driver_called ? "pin registration" : "pin/bus setup");
        }
        mount_error(error);
        Log::logf(CAT_STORAGE, success && mount_result == ESP_OK ? LOG_WARN : LOG_ERROR,
                  "%s\n", error);
        return;
    }
#endif

    const uint64_t card_bytes = storage_card.cardSize();
    const uint64_t total_bytes = storage_card.totalBytes();
    const uint64_t used_bytes = storage_card.usedBytes();
    portENTER_CRITICAL(&status_mux);
    status.card_bytes = card_bytes;
    status.total_bytes = total_bytes;
    status.used_bytes = used_bytes;
    status.error[0] = 0;
#if AB_STORAGE_SPI_SD_ENABLED
    set_spi_bus_status(spi_mount_clock_khz, type);
#else
    set_bus_status(mount_card_info);
#endif
    // Also covers a USB round trip missed by consumers observing mounted=true.
    for (auto &revision : file_mutation_revision) ++revision;
    __atomic_store_n(&status.mounted, true, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&status_mux);
#if !AB_STORAGE_SPI_SD_ENABLED
    mount_card_info = nullptr;
#endif
    Status published;
    get_status(published);
    char width[12] = "unavailable", frequency[24] = "unavailable";
    if (published.bus_width) snprintf(width, sizeof(width), "%u", published.bus_width);
    if (published.bus_clock_khz)
        snprintf(frequency, sizeof(frequency), "%ukHz", published.bus_clock_khz);
    Log::logf(CAT_STORAGE, LOG_INFO,
              "mounted bus=%s width=%s freq=%s limit=%ukHz type=%s size=%lluMiB\n",
              bus_name(published.bus), width, frequency, published.bus_clock_limit_khz,
              card_type_name(published.card_type),
              static_cast<unsigned long long>(card_bytes / (1024 * 1024)));
}

void advance_usb() {
#if AB_USB_MSC_ENABLED
    bool usb_failed = false;
    const Mode mode = __atomic_load_n(&status.mode, __ATOMIC_ACQUIRE);
    if (mode == Mode::ToUsb) {
        revoke_sessions();
        if (__atomic_load_n(&direct_owner, __ATOMIC_ACQUIRE)) return;
        // Covers an update/reboot admitted just before the handoff request.
        if (OtaManager::busy()) {
            mount_error("update or reboot active");
            __atomic_store_n(&status.mode, Mode::Local, __ATOMIC_RELEASE);
            return;
        }
        portENTER_CRITICAL(&status_mux);
        clear_bus_status();
        __atomic_store_n(&status.mounted, false, __ATOMIC_RELEASE);
        portEXIT_CRITICAL(&status_mux);
        storage_card.end();
#if AB_STORAGE_SPI_SD_ENABLED
        esp_err_t result = spi_power_cycle_required ? ESP_ERR_INVALID_STATE :
            storage_card.begin_raw() ? ESP_OK : ESP_FAIL;
        usb_host_started = result == ESP_OK;
        usb_sectors = result == ESP_OK ? storage_card.numSectors() : 0;
        if (result == ESP_OK && (storage_card.sectorSize() != 512 || !usb_sectors))
            result = ESP_ERR_NOT_SUPPORTED;
        const uint32_t clock_khz = result == ESP_OK ? storage_card.frequency_khz() : 0;
        const auto type = result == ESP_OK ? storage_card.cardType() : CARD_NONE;
#else
        sdmmc_host_t host = SDMMC_HOST_DEFAULT();
        sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
        configure_host(host, slot);
        esp_err_t result = sdmmc_host_init();
        usb_host_started = result == ESP_OK;
        if (result == ESP_OK) result = sdmmc_host_init_slot(host.slot, &slot);
        if (result == ESP_OK) result = sdmmc_card_init(&host, &usb_card);
        if (result == ESP_OK && (usb_card.csd.sector_size != 512 || !usb_card.csd.capacity))
            result = ESP_ERR_NOT_SUPPORTED;
#endif
        if (result == ESP_OK) {
            portENTER_CRITICAL(&status_mux);
#if AB_STORAGE_SPI_SD_ENABLED
            set_spi_bus_status(clock_khz, type);
#else
            set_bus_status(&usb_card);
#endif
            __atomic_store_n(&status.mode, Mode::Usb, __ATOMIC_RELEASE);
            portEXIT_CRITICAL(&status_mux);
#if AB_STORAGE_SPI_SD_ENABLED
            result = UsbStorage::expose(usb_sectors);
#else
            result = UsbStorage::expose(usb_card.csd.capacity);
#endif
        }
        if (result == ESP_OK) {
            Log::logf(CAT_STORAGE, LOG_INFO, "SD shared over USB; local recording paused\n");
            if (__atomic_load_n(&status.mode, __ATOMIC_ACQUIRE) == Mode::Usb) return;
        } else {
            Log::logf(CAT_STORAGE, LOG_ERROR, "USB handoff failed: %s\n", esp_err_to_name(result));
            usb_failed = true;
            __atomic_store_n(&status.mode, Mode::ToLocal, __ATOMIC_RELEASE);
        }
    }
    if (__atomic_load_n(&status.mode, __ATOMIC_ACQUIRE) == Mode::ToLocal) {
        portENTER_CRITICAL(&status_mux);
        const bool failed = status.error[0] != 0;
        portEXIT_CRITICAL(&status_mux);
        // Keep admission closed after failure until request_usb(false) retries.
        if (failed) return;
        if (__atomic_load_n(&usb_io_pending, __ATOMIC_ACQUIRE)) return;
        portENTER_CRITICAL(&status_mux);
        clear_bus_status();
        portEXIT_CRITICAL(&status_mux);
        const esp_err_t stopped = UsbStorage::stop();
        if (stopped != ESP_OK) {
            mount_error("USB shutdown failed");
            Log::logf(CAT_STORAGE, LOG_ERROR, "USB shutdown failed: %s\n", esp_err_to_name(stopped));
            return;
        }
        if (usb_host_started) {
#if AB_STORAGE_SPI_SD_ENABLED
            const esp_err_t result = storage_card.end_raw();
#else
            const esp_err_t result = sdmmc_host_deinit();
#endif
            if (result != ESP_OK) {
                mount_error("USB card shutdown failed");
                Log::logf(CAT_STORAGE, LOG_ERROR, "USB card shutdown failed: %s\n", esp_err_to_name(result));
                return;
            }
            usb_host_started = false;
#if AB_STORAGE_SPI_SD_ENABLED
            usb_sectors = 0;
#endif
        }
        mount_card();
        if (!mounted()) {
            Log::logf(CAT_STORAGE, LOG_ERROR, "SD remount after USB failed\n");
            return;
        }
        if (usb_failed) mount_error("USB handoff failed");
        __atomic_store_n(&status.mode, Mode::Local, __ATOMIC_RELEASE);
        Log::logf(CAT_STORAGE, LOG_INFO, "SD returned from USB\n");
    }
#endif
}
}  // namespace
#endif

bool local_access_allowed() {
    return __atomic_load_n(&status.mode, __ATOMIC_ACQUIRE) == Mode::Local;
}

bool request_usb(bool enabled, const char **error) {
    const char *rejected = nullptr;
#if AB_USB_MSC_ENABLED
    if (enabled && OtaManager::busy()) rejected = "update or reboot active";
    portENTER_CRITICAL(&status_mux);
    if (!rejected && !status.usb_supported) rejected = "USB storage unavailable";
    if (!rejected) {
        if (enabled && status.mode == Mode::Local) {
            if (!status.mounted) rejected = "SD card unavailable";
            else {
                status.error[0] = 0;
                __atomic_store_n(&status.mode, Mode::ToUsb, __ATOMIC_RELEASE);
            }
        } else if (!enabled && status.mode == Mode::Usb) {
            __atomic_store_n(&status.mode, Mode::ToLocal, __ATOMIC_RELEASE);
        } else if (!enabled && status.mode == Mode::ToLocal && status.error[0]) {
            status.error[0] = 0;
        } else if ((enabled && status.mode != Mode::Usb) ||
                   (!enabled && status.mode != Mode::Local)) rejected = "USB handoff in progress";
    }
    portEXIT_CRITICAL(&status_mux);
    if (!rejected) wake_worker();
#else
    (void)enabled;
    rejected = "USB storage unsupported";
#endif
    if (error) *error = rejected;
    return !rejected;
}

void usb_ejected() {
#if AB_USB_MSC_ENABLED
    portENTER_CRITICAL(&status_mux);
    if (status.mode == Mode::Usb)
        __atomic_store_n(&status.mode, Mode::ToLocal, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&status_mux);
    wake_worker();
#endif
}

bool usb_transfer(bool write, uint32_t sector, uint8_t *buffer, size_t size,
                  void (*done)(bool)) {
#if AB_USB_MSC_ENABLED
    portENTER_CRITICAL(&status_mux);
    const bool admitted = status.mode == Mode::Usb && worker && !usb_io_pending;
    if (admitted) __atomic_store_n(&usb_io_pending, true, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&status_mux);
    if (!admitted) return false;
    usb_io.write = write;
    usb_io.sector = sector;
    usb_io.buffer = buffer;
    usb_io.size = size;
    usb_io.done = done;
    usb_io.request.kind = Request::UsbIo;
    Request *pointer = &usb_io.request;
    if (xQueueSend(requests, &pointer, 0) == pdTRUE) { wake_worker(); return true; }
    __atomic_store_n(&usb_io_pending, false, __ATOMIC_RELEASE);
    wake_worker();
    return false;
#else
    (void)write; (void)sector; (void)buffer; (void)size; (void)done;
    return false;
#endif
}

bool mounted() {
    return __atomic_load_n(&status.mounted, __ATOMIC_ACQUIRE);
}

fs::FS *filesystem() {
#if AB_STORAGE_HAS_SDCARD
    return mounted() ? &storage_card : nullptr;
#else
    return nullptr;
#endif
}

void refresh_usage() {
#if AB_STORAGE_HAS_SDCARD
    if (!mounted()) return;
    const uint64_t total_bytes = storage_card.totalBytes();
    const uint64_t used_bytes = storage_card.usedBytes();
    portENTER_CRITICAL(&status_mux);
    status.total_bytes = total_bytes;
    status.used_bytes = used_bytes;
    portEXIT_CRITICAL(&status_mux);
#endif
}

void get_status(Status &out) {
    portENTER_CRITICAL(&status_mux);
    out = status;
    portEXIT_CRITICAL(&status_mux);
}

const char *state_name(const Status &value) {
    if (!value.supported) return "unsupported";
    switch (value.mode) {
    case Mode::ToUsb: return "usb_starting";
    case Mode::Usb: return "usb";
    case Mode::ToLocal: return "usb_stopping";
    default: return value.mounted ? "mounted" : "unavailable";
    }
}

const char *format_result_name(FormatResult result) {
    switch (result) {
    case FormatResult::Succeeded: return "succeeded";
    case FormatResult::Failed: return "failed";
    default: return "none";
    }
}

const char *card_type_name(CardType type) {
    switch (type) {
    case CardType::Sd: return "SD";
    case CardType::Sdhc: return "SDHC";
    case CardType::Mmc: return "MMC";
    case CardType::Sdio: return "SDIO";
    default: return "unknown";
    }
}

const char *bus_name(Bus bus) {
    switch (bus) {
    case Bus::Sdmmc: return "SDMMC";
    case Bus::Spi: return "SPI";
    default: return "none";
    }
}

uint32_t files_revision(FileScope scope) {
    portENTER_CRITICAL(&status_mux);
    const uint32_t revision = file_mutation_revision[size_t(scope)];
    portEXIT_CRITICAL(&status_mux);
    return revision;
}

void notify_files_changed(const char *path, const char *destination) {
    const uint32_t mask = file_scope_mask(path) |
        (destination ? file_scope_mask(destination) : 0);
    portENTER_CRITICAL(&status_mux);
    for (size_t i = 0; i < size_t(FileScope::Count); ++i)
        if (mask & (1u << i)) ++file_mutation_revision[i];
    portEXIT_CRITICAL(&status_mux);
}

bool acquire() {
#if AB_STORAGE_HAS_SDCARD
    __atomic_add_fetch(&recorder_waiting, 1, __ATOMIC_ACQ_REL);
    Request request = {};
    request.kind = Request::Acquire;
    const bool taken = dispatch(request, true);
    cancel_acquire();
    __atomic_sub_fetch(&recorder_waiting, 1, __ATOMIC_ACQ_REL);
    return taken;
#else
    return false;
#endif
}

bool try_acquire(bool reserve_next) {
#if AB_STORAGE_HAS_SDCARD
    const TaskHandle_t caller = xTaskGetCurrentTaskHandle();
    const TaskHandle_t waiting = __atomic_load_n(&maintenance_waiter, __ATOMIC_ACQUIRE);
    if (waiting && waiting != caller) return false;
    if (reserve_next) {
        TaskHandle_t expected = nullptr;
        if (!__atomic_compare_exchange_n(&maintenance_waiter, &expected, caller,
            false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE) && expected != caller) return false;
    }
    Request request = {};
    request.kind = Request::TryAcquire;
    const bool taken = dispatch(request, false);
    if (taken && __atomic_load_n(&maintenance_waiter, __ATOMIC_ACQUIRE) == caller)
        __atomic_store_n(&maintenance_granted, true, __ATOMIC_RELEASE);
    if (!local_access_allowed() || !mounted()) cancel_acquire();
    return taken;
#else
    (void)reserve_next;
    return false;
#endif
}

void finish_maintenance() {
#if AB_STORAGE_HAS_SDCARD
    if (__atomic_load_n(&maintenance_granted, __ATOMIC_ACQUIRE)) cancel_acquire();
#endif
}

void cancel_acquire() {
#if AB_STORAGE_HAS_SDCARD
    TaskHandle_t expected = xTaskGetCurrentTaskHandle();
    if (__atomic_load_n(&maintenance_waiter, __ATOMIC_ACQUIRE) != expected) return;
    __atomic_store_n(&maintenance_granted, false, __ATOMIC_RELEASE);
    if (__atomic_compare_exchange_n(&maintenance_waiter, &expected, nullptr,
        false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) wake_worker();
#endif
}

bool maintenance_pending() {
#if AB_STORAGE_HAS_SDCARD
    return __atomic_load_n(&maintenance_waiter, __ATOMIC_ACQUIRE) != nullptr;
#else
    return false;
#endif
}

bool waiting_for_readers() {
#if AB_STORAGE_HAS_SDCARD
    const bool readers = __atomic_load_n(&sessions[1].generation, __ATOMIC_ACQUIRE) ||
        __atomic_load_n(&sessions[2].generation, __ATOMIC_ACQUIRE);
    return readers && (maintenance_pending() ||
        __atomic_load_n(&exclusive_waiter, __ATOMIC_ACQUIRE) ||
        __atomic_load_n(&reader_blocked_write, __ATOMIC_ACQUIRE));
#else
    return false;
#endif
}

bool maintenance_ready() {
#if AB_STORAGE_HAS_SDCARD
    const TaskHandle_t waiting = __atomic_load_n(&maintenance_waiter, __ATOMIC_ACQUIRE);
    return !waiting || (waiting == xTaskGetCurrentTaskHandle() &&
        !__atomic_load_n(&active_sessions, __ATOMIC_ACQUIRE) &&
        !__atomic_load_n(&direct_owner, __ATOMIC_ACQUIRE));
#else
    return false;
#endif
}

void release() {
#if AB_STORAGE_HAS_SDCARD
    Request request = {};
    request.kind = Request::Release;
    if (!dispatch(request, true))
        Log::logf(CAT_STORAGE, LOG_ERROR, "release by non-owner\n");
#endif
}

bool Session::begin(Access access, const char *read_root) {
#if AB_STORAGE_HAS_SDCARD
    if (valid()) return access == access_;
    if (generation_ && access != access_) end();
    Request request = {};
    request.kind = Request::Begin;
    request.access = access;
    request.path = read_root;
    request.generation = generation_;
    request.admission_notify = admission_notify_;
    request.admission_context = admission_context_;
    const bool admitted = dispatch(request, false);
    generation_ = request.generation;
    access_ = access;
    return admitted;
#else
    (void)access; (void)read_root;
    return false;
#endif
}

void Session::end() {
#if AB_STORAGE_HAS_SDCARD
    if (!generation_) return;
    Request request = {};
    request.kind = Request::End;
    request.generation = generation_;
    if (valid()) {
        request.observer = io_observer_;
        request.observer_context = io_context_;
    }
    (void)dispatch(request, true);
    generation_ = 0;
#endif
}

bool Session::valid() const {
#if AB_STORAGE_HAS_SDCARD
    return worker && mounted() && background_allowed(generation_);
#else
    return false;
#endif
}

bool Session::yield_requested() const {
#if AB_STORAGE_HAS_SDCARD
    const bool peer_waiting = __atomic_load_n(&first_background_waiter, __ATOMIC_ACQUIRE) &&
        uint32_t(millis() - __atomic_load_n(&first_background_waiter_at, __ATOMIC_ACQUIRE)) < BACKGROUND_RESUME_MS;
    const bool turn_complete = uint32_t(millis() - __atomic_load_n(&background_turn_at, __ATOMIC_ACQUIRE)) >= BACKGROUND_TURN_MS;
    return access_ == Access::Background && valid() && (maintenance_pending() ||
        (peer_waiting && turn_complete) ||
        (__atomic_load_n(&exclusive_waiter, __ATOMIC_ACQUIRE) &&
         generation_ != __atomic_load_n(&protected_background, __ATOMIC_ACQUIRE)));
#else
    return false;
#endif
}

bool Session::protect_read(const char *path) {
#if AB_STORAGE_HAS_SDCARD
    if (access_ != Access::ReadOnly || (path && strlen(path) >= sizeof(sessions[1].read_root))) return false;
    return run([&](fs::FS &) {
        if (read_blocked(path)) return false;
        for (const auto &reader : readers)
            if (reader.id && reader.owner == generation_) return false;
        for (size_t i = 1; i < 3; ++i) {
            auto &session = sessions[i];
            if (session.generation != generation_) continue;
            snprintf(session.read_root, sizeof(session.read_root), "%s", path ? path : "");
            return true;
        }
        return false;
    });
#else
    (void)path;
    return false;
#endif
}

bool Session::run(bool (*operation)(fs::FS &, void *), void *context) const {
#if AB_STORAGE_HAS_SDCARD
    if (!operation || !valid() || xTaskGetCurrentTaskHandle() == worker) return false;
    Request request = {};
    request.kind = Request::Run;
    request.operation = operation;
    request.context = context;
    request.generation = generation_;
    request.access = access_;
    request.observer = io_observer_;
    request.observer_context = io_context_;
    return dispatch(request, false);
#else
    return false;
#endif
}

bool Session::mutate(const char *path, const char *destination,
                     bool (*operation)(fs::FS &, void *), void *context) const {
#if AB_STORAGE_HAS_SDCARD
    if (!path || !*path || access_ == Access::ReadOnly || !operation || !valid() ||
        xTaskGetCurrentTaskHandle() == worker) return false;
    Request request = {};
    request.kind = Request::Run;
    request.operation = operation;
    request.context = context;
    request.generation = generation_;
    request.access = access_;
    request.path = path;
    request.destination = destination;
    request.observer = io_observer_;
    request.observer_context = io_context_;
    return dispatch(request, false);
#else
    return false;
#endif
}

bool valid_path(const char *path) {
    if (!path || path[0] != '/' || strlen(path) >= 256) return false;
    if (!path[1]) return true;
    const char *segment = path + 1;
    for (const char *p = segment;; p++) {
        if (*p && (uint8_t(*p) < 32 || *p == '\\' || *p == 127)) return false;
        if (!*p || *p == '/') {
            const size_t size = p - segment;
            if (!size || (size == 1 && segment[0] == '.') ||
                (size == 2 && segment[0] == '.' && segment[1] == '.')) return false;
            if (!*p) return true;
            segment = p + 1;
        }
    }
}

bool valid_write_path(const char *path) {
    if (!valid_path(path) || !path[1]) return false;
    for (const char *p = path + 1; *p; ++p) {
        if (strchr("\\:*?\"<>|", *p)) return false;
        if ((!p[1] || p[1] == '/') && (*p == '.' || *p == ' ')) return false;
    }
    return true;
}

const char *Session::publish_file(const char *temporary, const char *path,
                                  bool overwrite, const char *backup) const {
#if AB_STORAGE_HAS_SDCARD
    const char *error = "publish_failed";
    const bool published = mutate(temporary, path, [&](fs::FS &fs) {
        const bool exists = fs.exists(path);
        if (exists) {
            fs::File file = fs.open(path);
            if (!file || file.isDirectory()) { error = "destination_not_file"; return false; }
            file.close();
            if (!overwrite) { error = "file_exists"; return false; }
            if (fs.exists(backup)) { error = "backup_exists"; return false; }
            if (!fs.rename(path, backup)) return false;
        }
        if (!fs.rename(temporary, path)) {
            if (exists && !fs.rename(backup, path)) {
                error = "replace_restore_failed";
                notify_files_changed(path);
            }
            return false;
        }
        if (exists) fs.remove(backup);
        notify_files_changed(path);
        return true;
    });
    return published ? nullptr : error;
#else
    (void)temporary; (void)path; (void)overwrite; (void)backup;
    return "unsupported";
#endif
}

bool Session::ensure_parent(const char *path) const {
#if AB_STORAGE_HAS_SDCARD
    char parent[256];
    if (!path || path[0] != '/' || strlen(path) >= sizeof(parent)) return false;
    strcpy(parent, path);
    char *end = strrchr(parent, '/');
    if (!end[1]) return false;
    if (end == parent) return valid();
    *end = 0;
    for (char *p = parent + 1;; ++p) {
        if (*p && *p != '/') continue;
        const char saved = *p;
        *p = 0;
        const bool ok = mutate(parent, [&](fs::FS &fs) {
            if (fs.exists(parent)) {
                fs::File directory = fs.open(parent);
                return directory && directory.isDirectory();
            }
            if (!fs.mkdir(parent)) return false;
            notify_files_changed(parent);
            return true;
        });
        *p = saved;
        if (!ok) return false;
        if (!saved) break;
    }
    return valid();
#else
    (void)path;
    return false;
#endif
}

bool Reader::open(const Session &session, const char *path, bool directory) {
    close();
#if AB_STORAGE_HAS_SDCARD
    if (!path) return false;
    const bool opened = session.run([&](fs::FS &fs) {
        OpenFile *slot = nullptr;
        unsigned held = 0;
        for (OpenFile &reader : readers) {
            if (reader.owner == session.generation_) ++held;
            if (!slot && !reader.id) slot = &reader;
        }
        if (held >= (session.access_ == Access::ReadOnly ? 1u : 2u)) return false;
        if (!slot) return false;
        fs::File file = fs.open(path, FILE_READ);
        if (!file || file.isDirectory() != directory) return false;
        size_ = file.size();
        modified_ = file.getLastWrite();
        handle_ = retain_file(*slot, file, session.generation_);
        physical_offset_ = 0;
        return true;
    });
    if (!opened) return false;
    session_ = session;
    return true;
#else
    return false;
#endif
}

bool Reader::next(Entry &entry, bool &end) {
    end = false;
#if AB_STORAGE_HAS_SDCARD
    if (!handle_) return false;
    return session_.run([&](fs::FS &) {
        OpenFile *reader = find_file(handle_);
        if (!reader || !reader->file.isDirectory()) return false;
        fs::File file = reader->file.openNextFile();
        if (!file) { end = true; return true; }
        const char *name = file.name();
        if (!name || strlen(name) >= sizeof(entry.name)) return false;
        strcpy(entry.name, name);
        entry.directory = file.isDirectory();
        entry.size = file.size();
        entry.modified = file.getLastWrite();
        return true;
    });
#else
    return false;
#endif
}

bool Reader::seek(uint64_t offset) {
#if AB_STORAGE_HAS_SDCARD
    if (!handle_ || offset > size_ || offset > UINT32_MAX) return false;
    const bool sought = session_.run([&](fs::FS &) {
        OpenFile *reader = find_file(handle_);
        if (!reader || !reader->file || reader->file.isDirectory()) return false;
        physical_offset_ = UINT64_MAX;
        if (!reader->file.seek(static_cast<uint32_t>(offset))) return false;
        physical_offset_ = offset;
        return true;
    });
    if (!sought) return false;
    offset_ = offset;
    return true;
#else
    return false;
#endif
}

bool Reader::read_at(uint64_t offset, uint8_t *data, size_t length) {
#if AB_STORAGE_HAS_SDCARD
    if (!handle_ || (!data && length) || length > READ_CHUNK_BYTES ||
        offset > UINT32_MAX || offset > size_ || length > size_ - offset) return false;
    bool positioned = false;
    size_t count = 0;
    const bool complete = session_.run([&](fs::FS &) {
        OpenFile *reader = find_file(handle_);
        if (!reader || !reader->file || reader->file.isDirectory()) return false;
        if (physical_offset_ != offset) {
            physical_offset_ = UINT64_MAX;
            if (!reader->file.seek(static_cast<uint32_t>(offset))) return false;
            physical_offset_ = offset;
        }
        positioned = true;
        if (length) count = reader->file.read(data, length);
        physical_offset_ = count == length ? offset + count : UINT64_MAX;
        return count == length;
    });
    if (positioned) offset_ = offset + count;
    return complete && session_.valid();
#else
    return false;
#endif
}

size_t Reader::read(uint8_t *data, size_t length) {
    size_t total = 0;
#if AB_STORAGE_HAS_SDCARD
    if (!*this || !data) return 0;
    while (total < length && offset_ < size_) {
        const size_t count = std::min<size_t>(READ_CHUNK_BYTES,
            std::min<uint64_t>(length - total, size_ - offset_));
        const bool read = session_.run([&](fs::FS &) {
            OpenFile *reader = find_file(handle_);
            if (!reader) return false;
            const size_t consumed = reader->file.read(data + total, count);
            if (physical_offset_ != UINT64_MAX)
                physical_offset_ = consumed == count ? physical_offset_ + consumed : UINT64_MAX;
            return consumed == count;
        });
        if (!read) break;
        total += count;
        offset_ += count;
    }
#endif
    return total;
}

void Reader::close() {
#if AB_STORAGE_HAS_SDCARD
    close_handle(handle_, session_.valid() ? session_.io_observer_ : nullptr, session_.io_context_);
#endif
    handle_ = 0;
    size_ = offset_ = 0;
    physical_offset_ = UINT64_MAX;
}

bool Writer::open(const Session &session, const char *path, uint64_t append_at,
                  bool truncate_tail, Mode mode) {
    close();
#if AB_STORAGE_HAS_SDCARD
    if (!path || strlen(path) >= sizeof(writer_path) || append_at > UINT32_MAX ||
        (truncate_tail && path[0] != '/') ||
        !session.mutate(path, [&](fs::FS &fs) {
        if (writer.id) return false;

        if (truncate_tail) {
            fs::File prefix = fs.open(path, FILE_READ);
            if (!prefix || prefix.isDirectory() || prefix.size() < append_at) {
                prefix.close();
                return false;
            }
            const bool has_tail = prefix.size() > append_at;
            prefix.close();
            if (has_tail) {
                // Arduino File has no truncate API; use the mounted FAT VFS
                // only here on the executor, after rejecting short files.
                const char *mount = fs.mountpoint();
                char native_path[sizeof(writer_path) + 32];
                if (!mount || !*mount ||
                    append_at > uint64_t(std::numeric_limits<off_t>::max())) return false;
                const int length = snprintf(native_path, sizeof(native_path), "%s%s", mount, path);
                if (length < 0 || size_t(length) >= sizeof(native_path) ||
                    ::truncate(native_path, static_cast<off_t>(append_at)) != 0) return false;
            }
        }

        fs::File file = mode == Mode::Readback
            ? fs.open(path, append_at || truncate_tail ? "r+" : "w+")
            : fs.open(path, append_at || truncate_tail ? FILE_APPEND : FILE_WRITE);
        if (!file || file.isDirectory() || ((append_at || truncate_tail) && file.size() != append_at) ||
            (mode == Mode::Readback && append_at && !file.seek(static_cast<uint32_t>(append_at)))) {
            file.close();
            return false;
        }
        handle_ = retain_file(writer, file, session.generation_);
        strcpy(writer_path, path);
        return true;
    })) return false;
    session_ = session;
    size_ = append_at;
    mode_ = mode;
    return true;
#else
    return false;
#endif
}

bool Writer::write(const uint8_t *data, size_t length) {
#if AB_STORAGE_HAS_SDCARD
    if (!handle_ || (!data && length) || length > UINT32_MAX - size_) return false;
    for (size_t at = 0; at < length;) {
        const size_t take = std::min(length - at, WRITE_CHUNK_BYTES);
        if (!session_.run([&](fs::FS &) {
            OpenFile *file = find_file(handle_);
            return file && write_exact(file->file, data + at, take);
        })) {
            close();
            return false;
        }
        at += take;
        size_ += take;
    }
    return session_.valid();
#else
    return false;
#endif
}

bool Writer::read_at(uint64_t offset, uint8_t *data, size_t length) {
#if AB_STORAGE_HAS_SDCARD
    if (!handle_ || mode_ != Mode::Readback || (!data && length) ||
        length > READ_CHUNK_BYTES || offset > size_ || length > size_ - offset) return false;
    bool restored = true;
    const bool complete = session_.run([&](fs::FS &) {
        OpenFile *file = find_file(handle_);
        if (!file || !file->file || file->file.isDirectory()) return false;
        const bool read = file->file.seek(static_cast<uint32_t>(offset)) &&
            (!length || file->file.read(data, length) == length);
        restored = file->file.seek(static_cast<uint32_t>(size_));
        if (!restored) file->close();
        return read && restored;
    });
    if (!restored) {
        handle_ = 0;
        size_ = 0;
        mode_ = Mode::Sequential;
    }
    return complete && session_.valid();
#else
    return false;
#endif
}

bool Writer::flush(uint64_t expected_size) {
#if AB_STORAGE_HAS_SDCARD
    return handle_ && size_ == expected_size && session_.run([&](fs::FS &) {
        OpenFile *file = find_file(handle_);
        if (!file) return false;
        file->file.flush();
        return file->file.size() == expected_size;
    });
#else
    return false;
#endif
}

bool Writer::finish(uint64_t expected_size) {
    const bool complete = flush(expected_size);
    close();
    return complete;
}

void Writer::close() {
#if AB_STORAGE_HAS_SDCARD
    close_handle(handle_, session_.valid() ? session_.io_observer_ : nullptr, session_.io_context_);
#endif
    handle_ = 0;
    size_ = 0;
    mode_ = Mode::Sequential;
}

}  // namespace SdStorage
