#include "airbridge_ota.h"
#include "sd_storage.h"

#include "app_config.h"
#include "build_info.h"
#include "debug_log.h"
#include "export_sync.h"
#include "ota_release_manifest.h"
#include "ota_url_client.h"
#include "resmed_ota.h"
#include "oxi_arbiter.h"
#include "oxi_ble.h"
#include "uart_arbiter.h"
#include "airsense_state.h"
#include "wifi_setup.h"

#include <atomic>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <esp_ota_ops.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

namespace OtaManager {
namespace {

static constexpr size_t MANIFEST_MAX_BYTES = 4 * 1024;
static constexpr uint32_t INITIAL_CHECK_DELAY_MS = 60 * 1000;
static constexpr uint32_t CHECK_INTERVAL_MS = 24UL * 60 * 60 * 1000;
static constexpr uint32_t RETRY_INTERVAL_MS = 60UL * 60 * 1000;
static constexpr uint32_t REBOOT_DELAY_MS = 1500;
static constexpr uint32_t TASK_STACK_BYTES = 8192;
static constexpr uint32_t BLE_PAUSE_TIMEOUT_MS = 5000;

enum Operation : uint8_t {
    OP_NONE,
    OP_CHECK,
    OP_INSTALL,
    OP_MANUAL,
    OP_RESMED,
};

struct RuntimeStatus {
    bool initialized = false;
    bool enabled = false;
    State result = State::Idle;
    bool reboot_pending = false;
    bool (*reboot_prepare)() = nullptr;
    size_t bytes = 0;
    size_t total_size = 0;
    uint32_t last_check_ms = 0;
    uint32_t next_check_ms = 0;
    uint32_t reboot_at_ms = 0;
    uint32_t resmed_claimed_at_ms = 0;
    bool resmed_started = false;
    bool manual_image_started = false;
    Operation operation = OP_NONE;
    char update_version[OtaRelease::VERSION_MAX] = {};
    char error[64] = {};
};

static RuntimeStatus runtime;
static std::atomic<uint32_t> status_revision{0};
static const char *last_blocked = nullptr;
static SemaphoreHandle_t mutex = nullptr;
static char work_url[OtaRelease::URL_MAX] = {};
static OtaRelease::Artifact available_artifact;
static OtaImage::Writer image_writer;

bool lock(TickType_t timeout = portMAX_DELAY) {
    return !mutex || xSemaphoreTakeRecursive(mutex, timeout) == pdTRUE;
}

void unlock(bool changed = false) {
    if (changed) status_revision.fetch_add(1);
    if (mutex) xSemaphoreGiveRecursive(mutex);
}

uint8_t progress_percent(size_t bytes, size_t total) {
    return total ? (uint8_t)min((size_t)100, bytes * 100 / total) : 0;
}

void set_error_locked(const char *error) {
    runtime.result = State::Error;
    snprintf(runtime.error, sizeof(runtime.error), "%s", error ? error : "error");
}

bool deadline_due(uint32_t now, uint32_t deadline) {
    return deadline && (int32_t)(now - deadline) >= 0;
}

const char *reboot_image_blocked() {
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *boot = esp_ota_get_boot_partition();
    if (!running) return "running_image_unavailable";
    if (!boot) return "boot_image_unavailable";
    return running->address != boot->address ? "different_boot_image_selected" : nullptr;
}

bool operation_allowed() {
    if (!lock(pdMS_TO_TICKS(50))) return false;
    bool allowed = runtime.operation == OP_CHECK;
    unlock();
    return allowed;
}

void finish_check(const OtaRelease::Manifest *manifest,
                  const OtaRelease::Artifact *artifact,
                  bool update_available,
                  const char *error) {
    if (!lock()) return;
    if (runtime.operation != OP_CHECK) {
        unlock();
        return;
    }
    runtime.result = update_available ? State::Available : State::Current;
    runtime.last_check_ms = millis();
    runtime.next_check_ms = runtime.last_check_ms +
        (error ? RETRY_INTERVAL_MS : CHECK_INTERVAL_MS);
    runtime.operation = OP_NONE;
    available_artifact = {};
    runtime.update_version[0] = '\0';
    runtime.error[0] = '\0';
    if (manifest) {
        snprintf(runtime.update_version, sizeof(runtime.update_version), "%s",
                 manifest->version);
    }
    if (artifact && update_available && !error) available_artifact = *artifact;
    if (error) set_error_locked(error);
    unlock(true);
    if (error) {
        Log::logf(CAT_OTA, LOG_WARN, "Release check failed: %s\n", error);
    } else {
        Log::logf(CAT_OTA, update_available ? LOG_INFO : LOG_DEBUG, "Release check: latest=%s available=%d\n",
                  manifest ? manifest->version : "?", update_available);
    }
}

void check_task(void *) {
    char url[sizeof(work_url)] = {};
    if (lock()) {
        snprintf(url, sizeof(url), "%s", work_url);
        unlock();
    }

    uint8_t *buffer = nullptr;
    size_t length = 0;
    OtaUrl::Error transport_error;
    if (!OxiBle::release_memory(BLE_PAUSE_TIMEOUT_MS)) {
        finish_check(nullptr, nullptr, false, "ble_memory_busy");
        vTaskDelete(nullptr);
        return;
    }

    bool fetched = OtaUrl::fetch(url, buffer, MANIFEST_MAX_BYTES, length,
                                transport_error,
                                [](void *) { return operation_allowed(); }, nullptr);
    OtaRelease::Manifest manifest;
    char manifest_error[OtaRelease::ERROR_MAX] = {};
    bool parsed = fetched && OtaRelease::parse_manifest(
        (char *)buffer, length, AB_OTA_RELEASE_TARGET,
        manifest, manifest_error);
    heap_caps_free(buffer);

    // TLS and the manifest buffer are gone before BLE allocates again.
    bool restored = OxiBle::restore_memory(BLE_PAUSE_TIMEOUT_MS);
    if (!fetched || !restored) {
        finish_check(nullptr, nullptr, false,
                     !restored ? "ble_restore_failed" :
                     transport_error.code[0] ? transport_error.code
                                             : "manifest_fetch_failed");
        vTaskDelete(nullptr);
        return;
    }

    if (!parsed) {
        finish_check(nullptr, nullptr, false, manifest_error);
        vTaskDelete(nullptr);
        return;
    }

    bool newer = false;
    if (!OtaRelease::is_newer(airbridge_version(), manifest.version, newer)) {
        finish_check(&manifest, nullptr, false,
                     "current_version_invalid");
        vTaskDelete(nullptr);
        return;
    }

    OtaRelease::Artifact resolved = manifest.artifact;
    if (!OtaRelease::resolve_artifact_url(
            url, manifest.artifact.url, resolved.url, sizeof(resolved.url))) {
        finish_check(&manifest, nullptr, false,
                     "artifact_url_invalid");
        vTaskDelete(nullptr);
        return;
    }

    const esp_partition_t *partition = esp_ota_get_next_update_partition(nullptr);
    bool installable = newer && partition && resolved.image_size <= partition->size;
    finish_check(&manifest, &resolved, newer,
                 newer && !installable ? "artifact_too_large" : nullptr);
    vTaskDelete(nullptr);
}

void abort_install(const char *error) {
    abort_image();

    if (lock()) {
        runtime.operation = OP_NONE;
        runtime.reboot_pending = false;
        runtime.reboot_at_ms = 0;
        set_error_locked(error);
        unlock(true);
    }
    Arbiter::set_state(SYS_IDLE);
    Log::logf(CAT_OTA, LOG_ERROR, "Release install failed: %s\n",
              error ? error : "install_failed");
}

bool install_continue(void *) {
    if (!lock(pdMS_TO_TICKS(50))) return false;
    bool allowed = runtime.operation == OP_INSTALL &&
                   Arbiter::get_state() == SYS_OTA_ESP;
    unlock();
    return allowed;
}

bool install_write(void *, size_t offset, const uint8_t *data, size_t len) {
    if (!lock(pdMS_TO_TICKS(100))) return false;
    bool expected = runtime.operation == OP_INSTALL && offset == runtime.bytes &&
                    runtime.bytes <= runtime.total_size &&
                    len <= runtime.total_size - runtime.bytes;
    unlock();
    if (!expected) return false;

    if (!write_image(offset, data, len)) return false;

    if (lock()) {
        uint8_t before = progress_percent(runtime.bytes, runtime.total_size);
        runtime.bytes += len;
        unlock(before != progress_percent(runtime.bytes, runtime.total_size));
    }
    return true;
}

void install_task(void *) {
    OtaRelease::Artifact artifact;
    if (lock()) {
        artifact = available_artifact;
        unlock();
    }

    // Let the main loop suspend live streams after entering the OTA state.
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!install_continue(nullptr)) {
        abort_install("install_cancelled");
        vTaskDelete(nullptr);
        return;
    }

    if (!begin_image(artifact.size, artifact.image_size,
                     artifact.zlib ? OtaImage::Encoding::Zlib : OtaImage::Encoding::Plain)) {
        abort_install(image_status().error);
        vTaskDelete(nullptr);
        return;
    }

    OtaUrl::Error transport_error;
    if (!OtaUrl::stream(artifact.url, artifact.size, install_write,
                        install_continue, nullptr, transport_error)) {
        abort_install(image_status().error ? image_status().error :
                      transport_error.code[0] ? transport_error.code
                                              : "firmware_fetch_failed");
        vTaskDelete(nullptr);
        return;
    }

    if (!finish_image()) {
        abort_install(image_status().error);
        vTaskDelete(nullptr);
        return;
    }

    char partition_name[17] = {};
    snprintf(partition_name, sizeof(partition_name), "%s",
             image_status().partition);
    if (lock()) {
        runtime.operation = OP_NONE;
        runtime.reboot_pending = true;
        runtime.reboot_at_ms = millis() + REBOOT_DELAY_MS;
        runtime.error[0] = '\0';
        unlock(true);
    }
    Log::logf(CAT_OTA, LOG_INFO,
              "Release %s installed to '%s', rebooting\n",
              runtime.update_version, partition_name);
    vTaskDelete(nullptr);
}

bool start_worker(TaskFunction_t function, const char *name) {
    BaseType_t result = xTaskCreatePinnedToCore(
        function, name, TASK_STACK_BYTES, nullptr, 1, nullptr, 0);
    return result == pdPASS;
}

bool background_work_idle() {
    return SdStorage::local_access_allowed() && !OxiArbiter::is_feeding() && !ExportSync::busy();
}

const char *start_blocked() {
    if (!runtime.initialized) return "ota_unavailable";
    if (runtime.operation != OP_NONE || runtime.reboot_pending) return "ota_busy";
    if (WiFi.status() != WL_CONNECTED) return "network_unavailable";
    if (!AirSenseState::system_idle()) return "device_not_idle";
    if (ResmedOta::is_active()) return "resmed_ota_active";
    if (!background_work_idle()) return "background_work_active";
    return nullptr;
}

struct BlockedInputs {
    uint32_t network;
    bool initialized, ota_busy, device_idle, resmed_active, background_idle;

    bool operator==(const BlockedInputs &other) const {
        return network == other.network && initialized == other.initialized &&
            ota_busy == other.ota_busy && device_idle == other.device_idle &&
            resmed_active == other.resmed_active && background_idle == other.background_idle;
    }
};

// Called under the OTA mutex. Admission requests still use start_blocked directly.
const char *cached_start_blocked() {
    static BlockedInputs previous = {};
    static bool observed = false;
    const BlockedInputs inputs = {WiFiSetup::revision(), runtime.initialized,
        runtime.operation != OP_NONE || runtime.reboot_pending,
        AirSenseState::system_idle(), ResmedOta::is_active(), background_work_idle()};
    if (!observed || !(inputs == previous)) {
        const char *blocked = start_blocked();
        if (blocked != last_blocked && runtime.enabled &&
            runtime.operation == OP_NONE && !runtime.reboot_pending)
            status_revision.fetch_add(1);
        last_blocked = blocked;
        previous = inputs;
        observed = true;
    }
    return last_blocked;
}

}  // namespace

void init() {
    auto &cfg = Config::get();
    esp_ota_mark_app_valid_cancel_rollback();
    if (!mutex) mutex = xSemaphoreCreateRecursiveMutex();
    if (!mutex || !lock()) return;
    runtime = {};
    runtime.initialized = cfg.wifi_mode != WIFI_MODE_OFF;
    runtime.enabled = runtime.initialized && cfg.update_url.length() > 0;
    runtime.next_check_ms = millis() + INITIAL_CHECK_DELAY_MS;
    unlock(true);

    if (!runtime.initialized) return;
    Log::logf(CAT_OTA, LOG_DEBUG,
              "Ready, release target=%s\n", AB_OTA_RELEASE_TARGET);
}

bool request_reboot(bool (*prepare)()) {
    if ((prepare && !mutex) || !lock()) {
        if (prepare) Log::logf(CAT_OTA, LOG_WARN, "Prepared reboot refused: ota_unavailable\n");
        return false;
    }
    const char *rejected = nullptr;
    if (!SdStorage::local_access_allowed()) rejected = "usb_storage_active";
    if (!rejected && prepare) {
        if (runtime.operation != OP_NONE || ResmedOta::is_active()) rejected = "ota_busy";
        else if (runtime.reboot_pending) rejected = "reboot_pending";
        else rejected = reboot_image_blocked();
    }
    if (rejected) {
        unlock();
        Log::logf(CAT_OTA, LOG_WARN, "Prepared reboot refused: %s\n", rejected);
        return false;
    }
    const bool changed = !runtime.reboot_pending;
    if (changed) {
        runtime.reboot_pending = true;
        runtime.reboot_at_ms = millis() + REBOOT_DELAY_MS;
        runtime.reboot_prepare = prepare;
    }
    unlock(changed);
    return true;
}

void handle() {
    bool initialized = false;
    bool reboot = false;
    bool (*prepare)() = nullptr;
    bool auto_check = false;
    const char *blocked = "ota_unavailable";
    if (lock(pdMS_TO_TICKS(10))) {
        initialized = runtime.initialized;
        reboot = runtime.reboot_pending &&
                 deadline_due(millis(), runtime.reboot_at_ms);
        if (reboot) prepare = runtime.reboot_prepare;
        auto_check = runtime.enabled && runtime.operation == OP_NONE &&
                     !runtime.reboot_pending &&
                     deadline_due(millis(), runtime.next_check_ms);
        if (runtime.operation == OP_RESMED) {
            if (ResmedOta::is_active()) runtime.resmed_started = true;
            bool release_claim =
                (runtime.resmed_started && !ResmedOta::is_active()) ||
                (!runtime.resmed_started &&
                 millis() - runtime.resmed_claimed_at_ms >= 5000);
            if (release_claim) {
                runtime.operation = OP_NONE;
                if (!runtime.resmed_started)
                    Log::logf(CAT_OTA, LOG_WARN, "ResMed OTA claim expired before start\n");
                runtime.resmed_started = false;
                status_revision.fetch_add(1);
            }
        }
        blocked = cached_start_blocked();
        unlock();
    }
    if (reboot) {
        // Keep reboot_pending reserved while preparation runs outside the lock.
        const char *image_blocked = prepare ? reboot_image_blocked() : nullptr;
        if (prepare && (image_blocked || !prepare())) {
            if (lock()) {
                runtime.reboot_pending = false;
                runtime.reboot_at_ms = 0;
                runtime.reboot_prepare = nullptr;
                unlock(true);
            }
            Log::logf(CAT_OTA, LOG_ERROR,
                      "Prepared reboot cancelled: %s\n",
                      image_blocked ? image_blocked : "preparation_failed");
            return;
        }
        delay(50);
        ESP.restart();
        return;
    }
    if (!initialized) return;
    if (auto_check && !blocked) {
        if (!request_check() && lock(pdMS_TO_TICKS(10))) {
            if (deadline_due(millis(), runtime.next_check_ms))
                runtime.next_check_ms = millis() + RETRY_INTERVAL_MS;
            unlock();
        }
    }
}

bool request_check(const char **error) {
    auto &cfg = Config::get();
    if (error) *error = nullptr;
    if (!mutex || !lock()) {
        if (error) *error = "ota_unavailable";
        return false;
    }
    const char *rejected = start_blocked();
    if (!runtime.enabled) rejected = "update_check_disabled";
    else if (cfg.update_url.length() >= sizeof(work_url) ||
             !OtaUrl::supported(cfg.update_url.c_str()))
        rejected = "update_url_invalid";
    if (!rejected) {
        snprintf(work_url, sizeof(work_url), "%s", cfg.update_url.c_str());
        runtime.operation = OP_CHECK;
        runtime.error[0] = '\0';
        unlock(true);
        if (start_worker(check_task, "ota_check")) return true;
        if (lock()) {
            runtime.operation = OP_NONE;
            set_error_locked("update_task_alloc_failed");
            unlock(true);
        }
        if (error) *error = "update_task_alloc_failed";
        return false;
    }
    unlock();
    if (error) *error = rejected;
    return false;
}

bool request_install(const char **error) {
    if (error) *error = nullptr;
    if (!mutex || !lock()) {
        if (error) *error = "ota_unavailable";
        return false;
    }
    const char *rejected = start_blocked();
    if (!rejected && (runtime.result != State::Available || !available_artifact.size))
        rejected = "update_not_available";
    if (!rejected) {
        runtime.operation = OP_INSTALL;
        runtime.bytes = 0;
        runtime.total_size = available_artifact.size;
        runtime.error[0] = '\0';
        Arbiter::set_state(SYS_OTA_ESP);
        unlock(true);
        if (start_worker(install_task, "ota_install")) return true;
        if (lock()) {
            runtime.operation = OP_NONE;
            set_error_locked("install_task_alloc_failed");
            unlock(true);
        }
        Arbiter::set_state(SYS_IDLE);
        if (error) *error = "install_task_alloc_failed";
        return false;
    }
    unlock();
    if (error) *error = rejected;
    return false;
}

void config_changed() {
    auto &cfg = Config::get();
    if (!lock()) return;
    if (runtime.operation == OP_CHECK) runtime.operation = OP_NONE;
    runtime.enabled = runtime.initialized && cfg.update_url.length() > 0;
    runtime.result = State::Idle;
    runtime.update_version[0] = '\0';
    runtime.error[0] = '\0';
    available_artifact = {};
    runtime.next_check_ms = millis() + INITIAL_CHECK_DELAY_MS;
    unlock(true);
}

uint32_t revision() { return status_revision.load(); }

const char *state_name(State state) {
    switch (state) {
    case State::Disabled: return "disabled";
    case State::Idle: return "idle";
    case State::Checking: return "checking";
    case State::Current: return "current";
    case State::Available: return "available";
    case State::Installing: return "installing";
    case State::Rebooting: return "rebooting";
    case State::Busy: return "busy";
    case State::Error: return "error";
    }
    return "error";
}

bool get_status(Status &status) {
    memset(&status, 0, sizeof(status));
    if (!lock(pdMS_TO_TICKS(50))) return false;
    status.blocked = cached_start_blocked();
    status.revision = status_revision.load();
    status.state = runtime.reboot_pending ? State::Rebooting :
        runtime.operation == OP_CHECK ? State::Checking :
        runtime.operation == OP_INSTALL ? State::Installing :
        runtime.operation != OP_NONE ? State::Busy :
        !runtime.enabled ? State::Disabled : runtime.result;
    status.bytes = runtime.bytes;
    status.total_size = runtime.total_size;
    status.progress = progress_percent(runtime.bytes, runtime.total_size);
    status.last_check_age_ms = runtime.last_check_ms
        ? millis() - runtime.last_check_ms : 0;
    snprintf(status.update_version, sizeof(status.update_version), "%s",
             runtime.update_version);
    if (status.state == State::Error)
        snprintf(status.error, sizeof(status.error), "%s", runtime.error);
    unlock();
    return true;
}

bool begin_manual_upload() {
    if (!lock()) return false;
    bool allowed = runtime.initialized && runtime.operation == OP_NONE &&
                   !runtime.reboot_pending &&
                   AirSenseState::system_idle() &&
                   !ResmedOta::is_active() && background_work_idle();
    if (allowed) {
        runtime.operation = OP_MANUAL;
        runtime.manual_image_started = false;
        runtime.bytes = 0;
        runtime.total_size = 0;
    }
    unlock(allowed);
    return allowed;
}

void end_manual_upload(bool success, const char *error) {
    if (!lock()) return;
    bool ended = runtime.operation == OP_MANUAL;
    const bool image_started = ended && runtime.manual_image_started;
    const OtaImage::Status image = image_started ? image_writer.status() : OtaImage::Status{};
    if (ended) runtime.operation = OP_NONE;
    unlock(ended);
    if (ended && !success) {
        if (image_started)
            Log::logf(CAT_OTA, LOG_ERROR,
                "HTTP upload failed stage=%s bytes=%u wire=%u: %s\n",
                image.error_stage ? image.error_stage : "transport",
                unsigned(image.bytes), unsigned(image.wire_bytes),
                image.error ? image.error : error ? error : "upload_failed");
        else
            Log::logf(CAT_OTA, LOG_ERROR, "HTTP upload failed: %s\n", error ? error : "upload_failed");
    }
}

bool begin_image(size_t wire_size, size_t image_size, OtaImage::Encoding encoding) {
    if (!lock()) return false;
    const bool owned = runtime.operation == OP_MANUAL || runtime.operation == OP_INSTALL;
    if (runtime.operation == OP_MANUAL) runtime.manual_image_started = true;
    unlock();
    return owned && image_writer.begin(wire_size, image_size, encoding);
}

bool write_image(size_t index, const uint8_t *data, size_t len) {
    return image_writer.write(index, data, len);
}

bool finish_image() { return image_writer.finish(); }
void abort_image() { image_writer.abort(); }
const OtaImage::Status &image_status() { return image_writer.status(); }

bool begin_resmed_flash() {
    if (!lock()) return false;
    bool allowed = runtime.initialized && runtime.operation == OP_NONE &&
                   !runtime.reboot_pending &&
                   AirSenseState::system_idle() &&
                   !ResmedOta::is_active() && background_work_idle();
    if (allowed) {
        runtime.operation = OP_RESMED;
        runtime.resmed_claimed_at_ms = millis();
        runtime.resmed_started = false;
    }
    unlock(allowed);
    return allowed;
}

void cancel_resmed_flash_claim() {
    if (!lock()) return;
    bool cancelled = runtime.operation == OP_RESMED && !runtime.resmed_started;
    if (cancelled)
        runtime.operation = OP_NONE;
    unlock(cancelled);
}

bool busy() {
    if (!lock(pdMS_TO_TICKS(20))) return true;
    bool result = runtime.operation != OP_NONE || runtime.reboot_pending;
    unlock();
    return result;
}

}  // namespace OtaManager
