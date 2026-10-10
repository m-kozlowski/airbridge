#include "factory_reset.h"

#include "airbridge_ota.h"
#include "airsense_state.h"
#include "board.h"
#include "debug_log.h"
#include "resmed_ota.h"
#if AB_STORAGE_HAS_SDCARD
#include "sd_storage.h"
#endif

#include <nvs.h>
#include <nvs_flash.h>

namespace FactoryReset {
namespace {

constexpr const char *MARKER_NAMESPACE = "factory_reset";
constexpr const char *MARKER_KEY = "pending";

const char *blocked() {
    if (!AirSenseState::local_background_allowed()) return "device_not_idle";
    if (ResmedOta::is_active()) return "resmed_ota_active";
    return nullptr;
}

bool prepare_scope(Scope scope) {
    const bool storage_only = scope == Scope::StorageOnly;
    const char *name = storage_only ? "SD format" : "Factory reset";
    const log_cat_t category = storage_only ? CAT_STORAGE : CAT_CONFIG;
    const char *error = blocked();
    if (error) {
        Log::logf(category, LOG_WARN, "%s cancelled: %s\n", name, error);
        return false;
    }

    nvs_handle_t handle;
    esp_err_t cleanup = ESP_OK;
    esp_err_t result = nvs_open(MARKER_NAMESPACE, NVS_READWRITE, &handle);
    if (result == ESP_OK) {
        result = nvs_set_u8(handle, MARKER_KEY, static_cast<uint8_t>(scope));
        if (result == ESP_OK) result = nvs_commit(handle);
        if (result != ESP_OK) {
            // NVS writes may persist before commit; disarm a failed request.
            cleanup = nvs_erase_key(handle, MARKER_KEY);
            if (cleanup == ESP_OK) cleanup = nvs_commit(handle);
        }
        nvs_close(handle);
    }
    if (result != ESP_OK) {
        Log::logf(category, LOG_ERROR, "%s marker save failed: %s\n",
                  name, esp_err_to_name(result));
        if (cleanup != ESP_OK && cleanup != ESP_ERR_NVS_NOT_FOUND) {
            Log::logf(category, LOG_ERROR,
                      "%s fatal: marker cleanup failed: %s; reset may remain armed. "
                      "Restarting before normal work resumes\n", name, esp_err_to_name(cleanup));
            Log::poll();
            ESP.restart();
            for (;;) delay(1000);
        }
        return false;
    }
    Log::logf(category, LOG_WARN, "%s armed for next boot\n", name);
    return true;
}

bool prepare() { return prepare_scope(Scope::All); }
bool prepare_storage_format() { return prepare_scope(Scope::StorageOnly); }

}  // namespace

bool request(const char **error, Scope scope) {
#if !AB_STORAGE_HAS_SDCARD
    if (scope == Scope::StorageOnly) {
        if (error) *error = "storage_unsupported";
        return false;
    }
#endif
    const char *rejected = blocked();
    if (!rejected && !OtaManager::request_reboot(
            scope == Scope::StorageOnly ? prepare_storage_format : prepare))
        rejected = "prepared_reboot_unavailable";
    if (error) *error = rejected;
    return !rejected;
}

bool run_pending_on_boot() {
    nvs_handle_t handle;
    esp_err_t result = nvs_open(MARKER_NAMESPACE, NVS_READONLY, &handle);
    if (result == ESP_ERR_NVS_NOT_FOUND) return true;

    uint8_t marker = 0;
    if (result == ESP_OK) {
        result = nvs_get_u8(handle, MARKER_KEY, &marker);
        nvs_close(handle);
    }
    if (result == ESP_ERR_NVS_NOT_FOUND) return true;
    if (result != ESP_OK) {
        Log::logf(CAT_CONFIG, LOG_ERROR,
                  "Factory reset fatal: marker read failed: %s; pending intent unknown, boot halted\n",
                  esp_err_to_name(result));
        return false;
    }

    const bool storage_only = marker == static_cast<uint8_t>(Scope::StorageOnly);
    const bool recognized = storage_only || marker == static_cast<uint8_t>(Scope::All);
    const char *name = storage_only ? "SD format" : "Factory reset";
    const log_cat_t category = storage_only ? CAT_STORAGE : CAT_CONFIG;
    result = nvs_open(MARKER_NAMESPACE, NVS_READWRITE, &handle);
    if (result == ESP_OK) {
        result = nvs_erase_key(handle, MARKER_KEY);
        if (result == ESP_OK) result = nvs_commit(handle);
        nvs_close(handle);
    }
    if (result != ESP_OK) {
        Log::logf(category, LOG_ERROR, "%s marker consume failed: %s\n",
                  name, esp_err_to_name(result));
        if (recognized) {
            Log::logf(category, LOG_ERROR,
                      "%s fatal: armed intent could not be consumed; "
                      "boot halted before config/storage owners\n", name);
            return false;
        }
        return true;
    }
    if (!recognized) {
        Log::logf(CAT_CONFIG, LOG_WARN, "Unknown factory reset marker ignored\n");
        return true;
    }

#if AB_STORAGE_HAS_SDCARD
    const bool sd_formatted = SdStorage::factory_format();
    if (!sd_formatted) {
        if (storage_only) {
            Log::logf(CAT_STORAGE, LOG_ERROR,
                      "SD format failed; NVS settings preserved. "
                      "SD data may be lost; explicit retry required\n");
            return true;
        }
        Log::logf(CAT_CONFIG, LOG_WARN,
                  "Factory reset: SD format failed; continuing with NVS erase\n");
    }
#else
    const bool sd_formatted = true;
#endif

    if (storage_only) {
#if AB_STORAGE_HAS_SDCARD
        Log::logf(CAT_STORAGE, LOG_WARN, "SD format complete; configuration preserved\n");
#else
        Log::logf(CAT_STORAGE, LOG_WARN, "SD format unavailable; configuration preserved\n");
#endif
        return true;
    }

    result = nvs_flash_erase();
    if (result != ESP_OK) {
        Log::logf(CAT_CONFIG, LOG_ERROR,
                  "Factory reset failed: NVS erase: %s; data may be lost. "
                  "Restarting; explicit retry required\n",
                  esp_err_to_name(result));
    } else if (sd_formatted) {
        Log::logf(CAT_CONFIG, LOG_WARN, "Factory reset complete; restarting\n");
    } else {
        Log::logf(CAT_CONFIG, LOG_WARN,
                  "Factory reset: NVS cleared; SD format not completed; restarting\n");
    }
    // Erase deinitializes NVS even on failure. Never start config owners here.
    Log::poll();
    ESP.restart();
    // Do not allow old configuration owners to run even if restart returns.
    for (;;) delay(1000);
}

}  // namespace FactoryReset
