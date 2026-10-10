#include "sd_storage.h"

#include <Arduino.h>
#include <algorithm>
#include <string.h>

#include "board.h"
#include "debug_log.h"
#include "usb_storage.h"

#if AB_STORAGE_HAS_SDCARD
#include <SD_MMC.h>
#include <FS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <driver/sdmmc_host.h>
#include <sdmmc_cmd.h>
#include <ff.h>
#include <diskio_impl.h>
#include <diskio_sdmmc.h>
#include <esp_heap_caps.h>
#include "memory_manager.h"
#include "uart_arbiter.h"
#include "airsense_state.h"
#include "airbridge_ota.h"
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
static uint32_t file_mutation_revision = 0;

#if AB_STORAGE_HAS_SDCARD
namespace {
bool init_started = false;
bool formatting = false;

struct Request {
    enum Kind { Run, Acquire, TryAcquire, Release, Close, Begin, End, Mount, Wake, UsbIo } kind;
    bool (*operation)(fs::FS &, void *);
    void *context;
    uint32_t generation;
    SemaphoreHandle_t done;
    bool success;
    TaskHandle_t caller;
    uint32_t reader_id;
};

void mount_card();
void advance_usb();

QueueHandle_t requests = nullptr;
uint32_t generation = 1;
uint32_t active_session = 0;
uint32_t recorder_waiting = 0;
TaskHandle_t worker = nullptr;
TaskHandle_t direct_owner = nullptr;
Request wake_request = {};
#if AB_USB_MSC_ENABLED
sdmmc_card_t usb_card = {};
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
    if (!requests) return;
    Request *pointer = &wake_request;
    // A full queue already wakes the worker, which advances handoff after each request.
    xQueueSend(requests, &pointer, 0);
}

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

struct OpenFile {
    fs::File file;
    uint32_t id = 0;
};
OpenFile readers[2];
uint32_t next_reader = 0;

void close_readers() {
    for (OpenFile &reader : readers) {
        reader.file.close();
        reader.id = 0;
    }
}

OpenFile *find_reader(uint32_t id) {
    for (OpenFile &reader : readers)
        if (reader.id == id) return &reader;
    return nullptr;
}

bool background_allowed() {
    return local_access_allowed() && !__atomic_load_n(&recorder_waiting, __ATOMIC_ACQUIRE) &&
        AirSenseState::local_background_allowed();
}

bool background_allowed(uint32_t expected) {
    return expected && expected == __atomic_load_n(&active_session, __ATOMIC_ACQUIRE) &&
        background_allowed();
}

void process_request(Request &value) {
    Request *request = &value;
    request->success = false;
    if (request->kind == Request::Wake) {
        return;
    } else if (request->kind == Request::Mount) {
        if (local_access_allowed() && !direct_owner && !active_session) mount_card();
        request->success = mounted();
#if AB_USB_MSC_ENABLED
    } else if (request->kind == Request::UsbIo) {
        // Already-admitted writes finish even if the host ejects/disconnects meanwhile.
        const bool valid = usb_host_started && usb_io.size && !(usb_io.size % 512) &&
            uint64_t(usb_io.sector) + usb_io.size / 512 <= usb_card.csd.capacity;
        const esp_err_t result = !valid ? ESP_ERR_INVALID_STATE : usb_io.write
            ? sdmmc_write_sectors(&usb_card, usb_io.buffer, usb_io.sector, usb_io.size / 512)
            : sdmmc_read_sectors(&usb_card, usb_io.buffer, usb_io.sector, usb_io.size / 512);
        if (result != ESP_OK)
            Log::logf(CAT_STORAGE, LOG_WARN, "USB %s sector=%lu bytes=%u failed: %s\n",
                      usb_io.write ? "write" : "read", (unsigned long)usb_io.sector,
                      unsigned(usb_io.size), esp_err_to_name(result));
        const auto done = usb_io.done;
        __atomic_store_n(&usb_io_pending, false, __ATOMIC_RELEASE);
        done(result == ESP_OK);
#endif
    } else if (request->kind == Request::Begin) {
        if (!direct_owner && !active_session && mounted() && background_allowed()) {
            if (++generation == 0) ++generation;
            __atomic_store_n(&active_session, generation, __ATOMIC_RELEASE);
            request->generation = generation;
            request->success = true;
        }
    } else if (request->kind == Request::End) {
        if (active_session == request->generation) {
            close_readers();
            __atomic_store_n(&active_session, 0, __ATOMIC_RELEASE);
        }
        request->success = true;
    } else if (request->kind == Request::Acquire || request->kind == Request::TryAcquire) {
        if (local_access_allowed() && mounted() && !direct_owner &&
            (request->kind == Request::Acquire || !active_session)) {
            close_readers();
            __atomic_store_n(&active_session, 0, __ATOMIC_RELEASE);
            __atomic_store_n(&direct_owner, request->caller, __ATOMIC_RELEASE);
            request->success = true;
        }
    } else if (request->kind == Request::Close) {
        OpenFile *reader = find_reader(request->reader_id);
        if (reader) { reader->file.close(); reader->id = 0; }
        request->success = true;
    } else if (request->kind == Request::Release) {
        if (direct_owner == request->caller) {
            __atomic_store_n(&direct_owner, nullptr, __ATOMIC_RELEASE);
            request->success = true;
        }
    } else if (!direct_owner && background_allowed(request->generation) && mounted()) {
        request->success = request->operation(SD_MMC, request->context);
    }
}

void io_task(void *) {
    Request *request;
    while (true) {
        if (xQueueReceive(requests, &request, portMAX_DELAY) != pdTRUE) continue;
        process_request(*request);
        if (request->done) xSemaphoreGive(request->done);
        advance_usb();
    }
}

bool init_worker() {
    if (worker) return true;
    if (!requests) requests = xQueueCreate(4, sizeof(Request *));
    if (!requests) return false;
    wake_request.kind = Request::Wake;
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
    // No executor means no auxiliary readers; keep the same ownership token.
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
    const bool queued = xQueueSend(requests, &pointer, control ? portMAX_DELAY : 0) == pdTRUE;
    if (queued) xSemaphoreTake(request.done, portMAX_DELAY);
    vSemaphoreDelete(request.done);
    return queued && request.success;
}
}  // namespace

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
        !__atomic_load_n(&active_session, __ATOMIC_ACQUIRE);
    if (allowed) formatting = true;
    portEXIT_CRITICAL(&status_mux);
    if (!allowed) {
        mount_error("format requires pre-init boot");
        Log::logf(CAT_STORAGE, LOG_WARN, "format requires pre-init boot\n");
        return false;
    }

    constexpr size_t work_bytes = 4096;
    static_assert(FF_MAX_SS <= work_bytes, "SD format work buffer too small");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    configure_host(host, slot);

    sdmmc_card_t card = {};
    BYTE pdrv = FF_DRV_NOT_USED;
    PARTITION previous_partition = {};
    void *work = nullptr;
    bool host_started = false;
    bool disk_registered = false;
    const char *step = "host state";
    int result = ESP_OK;
    const bool formatted = [&]() {
        sdmmc_host_state_t host_state = {};
        result = sdmmc_host_get_state(&host_state);
        if (result != ESP_OK) return false;
        if (host_state.host_initialized) {
            result = ESP_ERR_INVALID_STATE;
            return false;
        }

        // DMA-capable internal memory avoids card-sized allocation and bounce I/O.
        step = "buffer";
        work = heap_caps_malloc(work_bytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!work) { result = ESP_ERR_NO_MEM; return false; }
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
        disk_registered = true;
        previous_partition = VolToPart[pdrv];
        VolToPart[pdrv] = {pdrv, 1};
        const char drive[] = {static_cast<char>('0' + pdrv), ':', 0};
        const LBA_t partitions[] = {100, 0, 0, 0};
        step = "partition";
        result = f_fdisk(pdrv, partitions, work);
        if (result != FR_OK) return false;
        // f_fdisk writes only the MBR; invalidate old GPT headers without a full erase.
        memset(work, 0, card.csd.sector_size);
        step = "GPT headers";
        result = sdmmc_write_sectors(&card, work, 1, 1);
        if (result != ESP_OK) return false;
        result = sdmmc_write_sectors(&card, work, card.csd.capacity - 1, 1);
        if (result != ESP_OK) return false;
        // Force partition 1 of the new table, never an old detected volume or SFD.
        const MKFS_PARM options = {FM_FAT32, 2, 0, 0, 0};
        step = "FAT32";
        result = f_mkfs(drive, &options, work, work_bytes);
        return result == FR_OK;
    }();

    if (disk_registered) {
        VolToPart[pdrv] = previous_partition;
        ff_diskio_unregister(pdrv);
    }
    bool success = formatted;
    if (host_started) {
        const esp_err_t shutdown = sdmmc_host_deinit();
        if (shutdown != ESP_OK) {
            Log::logf(CAT_STORAGE, LOG_ERROR, "format host shutdown failed (%d)\n", shutdown);
            if (success) { step = "host shutdown"; result = shutdown; }
            success = false;
        }
    }
    heap_caps_free(work);

    char error[sizeof(status.error)] = {};
    if (!success) snprintf(error, sizeof(error), "format %s failed (%d)", step, result);
    mount_error(error);
    portENTER_CRITICAL(&status_mux);
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
    if (!SD_MMC.begin("/sdcard", one_bit, false, AB_SDMMC_FREQ_KHZ, 8)) {
        mount_error("mount failed");
        Log::logf(CAT_STORAGE, LOG_ERROR, "mount failed\n");
        return;
    }
    if (SD_MMC.cardType() == CARD_NONE) {
        SD_MMC.end();
        mount_error("no card");
        Log::logf(CAT_STORAGE, LOG_WARN, "no card detected\n");
        return;
    }

    const uint64_t card_bytes = SD_MMC.cardSize();
    const uint64_t total_bytes = SD_MMC.totalBytes();
    const uint64_t used_bytes = SD_MMC.usedBytes();
    portENTER_CRITICAL(&status_mux);
    status.card_bytes = card_bytes;
    status.total_bytes = total_bytes;
    status.used_bytes = used_bytes;
    status.error[0] = 0;
    __atomic_store_n(&status.mounted, true, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&status_mux);
    Log::logf(CAT_STORAGE, LOG_INFO,
              "mounted width=%u freq=%ukHz size=%lluMB\n",
              AB_SDMMC_WIDTH, AB_SDMMC_FREQ_KHZ,
              static_cast<unsigned long long>(card_bytes / (1024 * 1024)));
}

void advance_usb() {
#if AB_USB_MSC_ENABLED
    bool usb_failed = false;
    const Mode mode = __atomic_load_n(&status.mode, __ATOMIC_ACQUIRE);
    if (mode == Mode::ToUsb) {
        close_readers();
        __atomic_store_n(&active_session, 0, __ATOMIC_RELEASE);
        if (__atomic_load_n(&direct_owner, __ATOMIC_ACQUIRE)) return;
        // Covers an update/reboot admitted just before the handoff request.
        if (OtaManager::busy()) {
            mount_error("update or reboot active");
            __atomic_store_n(&status.mode, Mode::Local, __ATOMIC_RELEASE);
            return;
        }
        __atomic_store_n(&status.mounted, false, __ATOMIC_RELEASE);
        SD_MMC.end();
        sdmmc_host_t host = SDMMC_HOST_DEFAULT();
        sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
        configure_host(host, slot);
        esp_err_t result = sdmmc_host_init();
        usb_host_started = result == ESP_OK;
        if (result == ESP_OK) result = sdmmc_host_init_slot(host.slot, &slot);
        if (result == ESP_OK) result = sdmmc_card_init(&host, &usb_card);
        if (result == ESP_OK && (usb_card.csd.sector_size != 512 || !usb_card.csd.capacity))
            result = ESP_ERR_NOT_SUPPORTED;
        if (result == ESP_OK) {
            __atomic_store_n(&status.mode, Mode::Usb, __ATOMIC_RELEASE);
            result = UsbStorage::expose(usb_card.csd.capacity);
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
        const esp_err_t stopped = UsbStorage::stop();
        if (stopped != ESP_OK) {
            mount_error("USB shutdown failed");
            Log::logf(CAT_STORAGE, LOG_ERROR, "USB shutdown failed: %s\n", esp_err_to_name(stopped));
            return;
        }
        if (usb_host_started) {
            const esp_err_t result = sdmmc_host_deinit();
            if (result != ESP_OK) {
                mount_error("USB card shutdown failed");
                Log::logf(CAT_STORAGE, LOG_ERROR, "USB card shutdown failed: %s\n", esp_err_to_name(result));
                return;
            }
            usb_host_started = false;
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
    if (xQueueSend(requests, &pointer, 0) == pdTRUE) return true;
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
    return mounted() ? &SD_MMC : nullptr;
#else
    return nullptr;
#endif
}

void refresh_usage() {
#if AB_STORAGE_HAS_SDCARD
    if (!mounted()) return;
    const uint64_t total_bytes = SD_MMC.totalBytes();
    const uint64_t used_bytes = SD_MMC.usedBytes();
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

uint32_t files_revision() {
    portENTER_CRITICAL(&status_mux);
    const uint32_t revision = file_mutation_revision;
    portEXIT_CRITICAL(&status_mux);
    return revision;
}

void notify_files_changed() {
    portENTER_CRITICAL(&status_mux);
    ++file_mutation_revision;
    portEXIT_CRITICAL(&status_mux);
}

bool acquire() {
#if AB_STORAGE_HAS_SDCARD
    __atomic_add_fetch(&recorder_waiting, 1, __ATOMIC_ACQ_REL);
    Request request = {};
    request.kind = Request::Acquire;
    const bool taken = dispatch(request, true);
    __atomic_sub_fetch(&recorder_waiting, 1, __ATOMIC_ACQ_REL);
    return taken;
#else
    return false;
#endif
}

bool try_acquire() {
#if AB_STORAGE_HAS_SDCARD
    Request request = {};
    request.kind = Request::TryAcquire;
    return dispatch(request, false);
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

bool Session::begin() {
#if AB_STORAGE_HAS_SDCARD
    Request request = {};
    request.kind = Request::Begin;
    if (!dispatch(request, false)) return false;
    generation_ = request.generation;
    return true;
#else
    return false;
#endif
}

void Session::end() {
#if AB_STORAGE_HAS_SDCARD
    if (!generation_) return;
    Request request = {};
    request.kind = Request::End;
    request.generation = generation_;
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

bool Session::run(bool (*operation)(fs::FS &, void *), void *context) const {
#if AB_STORAGE_HAS_SDCARD
    if (!operation || !valid() || xTaskGetCurrentTaskHandle() == worker) return false;
    Request request = {};
    request.kind = Request::Run;
    request.operation = operation;
    request.context = context;
    request.generation = generation_;
    return dispatch(request, false);
#else
    return false;
#endif
}

bool Reader::open(const Session &session, const char *path, bool directory) {
    close();
#if AB_STORAGE_HAS_SDCARD
    if (!path) return false;
    const bool opened = session.run([&](fs::FS &fs) {
        OpenFile *slot = nullptr;
        for (OpenFile &reader : readers) if (!reader.id) { slot = &reader; break; }
        if (!slot) return false;
        fs::File file = fs.open(path, FILE_READ);
        if (!file || file.isDirectory() != directory) return false;
        size_ = file.size();
        modified_ = file.getLastWrite();
        if (++next_reader == 0) ++next_reader;
        slot->id = handle_ = next_reader;
        slot->file = file;
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
        OpenFile *reader = find_reader(handle_);
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
        OpenFile *reader = find_reader(handle_);
        return reader && reader->file && !reader->file.isDirectory() &&
            reader->file.seek(static_cast<uint32_t>(offset));
    });
    if (!sought) return false;
    offset_ = offset;
    return true;
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
            OpenFile *reader = find_reader(handle_);
            return reader && reader->file.read(data + total, count) == count;
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
    if (handle_) {
        Request request = {};
        request.kind = Request::Close;
        request.reader_id = handle_;
        (void)dispatch(request, true);
    }
#endif
    handle_ = 0;
    size_ = offset_ = 0;
}

}  // namespace SdStorage
