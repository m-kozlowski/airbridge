#include "file_upload.h"
#include "resmed_firmware_repository.h"
#include "airbridge_ota.h"
#include "resmed_ota.h"
#include "memory_manager.h"
#include "debug_log.h"
#include "board.h"
#include "sd_storage.h"

#include <Arduino.h>
#include <algorithm>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <utility>
#if AB_STORAGE_HAS_SDCARD
#include <FS.h>
#endif

namespace FileUpload {
namespace {
SemaphoreHandle_t mutex = nullptr;
TaskHandle_t task = nullptr;
Status status;
uint32_t next_id = 0;
uint8_t *ring = nullptr;
size_t capacity = 0, produced = 0, consumed = 0;
bool active = false, final_received = false, sealed = false, claimed = false;
Drained drain_notify;
uint32_t drain_end = 0;
const esp_partition_t *partition = nullptr;
uint32_t last_input = 0;
constexpr uint32_t INPUT_TIMEOUT_MS = 30000;

void changed() { ++status.revision; }

void reject(const char *error) {
    if (!status.error[0]) snprintf(status.error, sizeof(status.error), "%s", error);
    status.cancel_allowed = false;
}

bool interrupted() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    const bool result = status.error[0] != 0;
    xSemaphoreGive(mutex);
    return result;
}

#if AB_STORAGE_HAS_SDCARD
bool file_read(void *context, uint64_t offset, uint8_t *out, size_t size) {
    return !interrupted() && static_cast<SdStorage::Reader *>(context)->read_at(offset, out, size);
}

#endif

void run_upload() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    const Destination destination = status.destination;
#if AB_STORAGE_HAS_SDCARD
    const bool explicit_bootloader = status.bootloader;
    const bool overwrite = status.overwrite;
#endif
    const uint32_t id = status.id;
    const uint32_t expected_size = status.size;
    char path[256];
    snprintf(path, sizeof(path), "%s", status.path);
    xSemaphoreGive(mutex);
    const char *destination_name = destination == Destination::Sd ? "sd" : "staging";
    Log::logf(CAT_OTA, LOG_INFO,
        "File upload started id=%lu dest=%s path=%s bytes=%u\n",
        static_cast<unsigned long>(id), destination_name, path, unsigned(expected_size));
    const char *failure = nullptr;
    uint32_t written = 0;
    size_t erased = 0;
    ResmedFirmwareImage::Result image;
#if AB_STORAGE_HAS_SDCARD
    bool committed = false;
    SdStorage::Session session;
    SdStorage::Writer output;
    SdStorage::Reader input;
    char temporary[256] = {};
    char backup[64] = {};
    bool private_created = false;
    if (destination == Destination::Sd) {
        snprintf(temporary, sizeof(temporary), "/.upload-%lu.part", static_cast<unsigned long>(id));
        snprintf(backup, sizeof(backup), "/.upload-%lu.bak", static_cast<unsigned long>(id));
        const uint32_t started = millis();
        bool admitted = session.begin(SdStorage::Access::Exclusive);
        while (!admitted && !interrupted() && SdStorage::local_access_allowed() &&
               uint32_t(millis() - started) < 2000) {
            vTaskDelay(pdMS_TO_TICKS(10));
            admitted = session.begin(SdStorage::Access::Exclusive);
        }
        if (!admitted) failure = "storage_busy";
        else if (!session.ensure_parent(path)) failure = "directory_failed";
        else if (!session.run([&](fs::FS &fs) {
            if (fs.exists(path)) {
                fs::File current = fs.open(path);
                if (!overwrite || !current || current.isDirectory()) return false;
            }
            return !fs.exists(temporary) && !fs.exists(backup);
        })) failure = "file_exists";
        else if (!(private_created = output.open(session, temporary))) failure = "open_failed";
    }
#else
    if (destination == Destination::Sd) failure = "unsupported";
#endif
    if (destination == Destination::Staging) {
        partition = ResmedOta::get_staging_partition();
        if (!partition) failure = "staging_partition_missing";
    }

    while (!failure) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        if (status.error[0]) { xSemaphoreGive(mutex); break; }
        const size_t offset = consumed % capacity;
        const size_t size = std::min({produced - consumed, capacity - offset, WRITE_CHUNK_BYTES});
        const bool finished = final_received && !size;
        if (!size && !finished && uint32_t(millis() - last_input) >= INPUT_TIMEOUT_MS)
            reject("upload_timeout");
        xSemaphoreGive(mutex);
        if (finished) break;
        if (!size) { ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)); continue; }
        if (destination == Destination::Staging && !written && ring[offset] == 0xe9) {
            failure = "esp_image_not_resmed"; break;
        }
        if (destination == Destination::Staging) {
            if (written > partition->size || size > partition->size - written) {
                failure = "staging_partition_too_small";
                break;
            }
            while (erased < written + size) {
                if (erased > partition->size || partition->size - erased < 4096 ||
                    esp_partition_erase_range(partition, erased, 4096) != ESP_OK) {
                    failure = "partition_erase_failed";
                    break;
                }
                erased += 4096;
            }
            if (!failure && esp_partition_write(partition, written, ring + offset, size) != ESP_OK)
                failure = "partition_write_failed";
        }
#if AB_STORAGE_HAS_SDCARD
        else if (!output.write(ring + offset, size)) failure = "sd_write_failed";
#endif
        if (failure) break;
        written += size;
        xSemaphoreTake(mutex, portMAX_DELAY);
        consumed += size;
        status.written = written;
        changed();
        Drained notify;
        char error[sizeof(status.error)];
        snprintf(error, sizeof(error), "%s", status.error);
        const bool drained_success = !status.error[0] && status.written >= drain_end;
        if (drain_notify && (status.error[0] ||
            (drained_success && !(final_received && drain_end == status.received))))
            notify.swap(drain_notify);
        xSemaphoreGive(mutex);
        if (notify) notify(drained_success, error);
        vTaskDelay(1);
    }

    if (!failure && !interrupted()) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        status.state = State::Finalizing;
        changed();
        xSemaphoreGive(mutex);
#if AB_STORAGE_HAS_SDCARD
        if (destination == Destination::Sd && !output.finish(written)) failure = "sd_finish_failed";
        if (!failure && explicit_bootloader) {
            if (!input.open(session, temporary)) failure = "image_read_failed";
            else {
                ResmedFirmwareImage::Reader reader{&input, written, file_read};
                ResmedFirmwareImage::inspect(reader, image, {}, ring, capacity);
                if (written != 0x4000 || image.status != ResmedFirmwareImage::Status::Ok ||
                    !image.bid_supported || !image.bid_matches || !image.crc_ok())
                    failure = "invalid_bootloader";
                else if (!ResmedFirmwareRepository::bootloader_path(image.bid, path, sizeof(path)) ||
                         !session.ensure_parent(path)) failure = "bootloader_store_failed";
            }
            input.close();
        }
#endif
        xSemaphoreTake(mutex, portMAX_DELAY);
        if (!failure && !status.error[0]) {
            sealed = true;
            status.cancel_allowed = false;
            changed();
        }
        xSemaphoreGive(mutex);
#if AB_STORAGE_HAS_SDCARD
        if (!failure && !interrupted() && destination == Destination::Sd) {
            failure = session.publish_file(temporary, path, overwrite, backup);
            committed = !failure;
        }
#endif
    }
#if AB_STORAGE_HAS_SDCARD
    input.close();
    output.close();
    if (private_created && !committed)
        session.mutate(temporary, [&](fs::FS &fs) { return !fs.exists(temporary) || fs.remove(temporary); });
    session.end();
#endif
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (failure) reject(failure);
    const bool success = !status.error[0];
    aircannect::Memory::free(ring);
    ring = nullptr;
    capacity = produced = consumed = 0;
    status.image = image;
    status.source_ready = success;
    status.cancel_allowed = success && destination == Destination::Staging;
    if (!success || destination != Destination::Staging) {
        if (destination == Destination::Staging)
            OtaManager::end_manual_upload(success, success ? nullptr : status.error);
        partition = nullptr;
    }
    status.state = success ? State::Ready : !strcmp(status.error, "cancelled") ? State::Cancelled : State::Failed;
    snprintf(status.path, sizeof(status.path), "%s", path);
    active = false;
    task = nullptr;
    status.complete = true;
    changed();
    Drained notify;
    notify.swap(drain_notify);
    const bool drained_success = success && written >= drain_end;
    const uint32_t received = status.received;
    char error[sizeof(status.error)];
    snprintf(error, sizeof(error), "%s", status.error);
    xSemaphoreGive(mutex);
    const bool cancelled = !strcmp(error, "cancelled");
    Log::logf(CAT_OTA, success || cancelled ? LOG_INFO : LOG_ERROR,
        "File upload %s id=%lu dest=%s path=%s bytes=%u received=%u%s%s\n",
        success ? "complete" : cancelled ? "cancelled" : "failed",
        static_cast<unsigned long>(id), destination_name, path, unsigned(written),
        unsigned(received), success ? "" : ": ", error);
    if (notify) notify(drained_success, error);
}

void worker(void *) {
    run_upload();
    vTaskDeleteWithCaps(nullptr);
}
}

void init() {
    if (mutex) return;
    mutex = xSemaphoreCreateMutex();
}

Result begin_upload(Destination destination, const char *filename, size_t size, uint32_t &id,
                    bool bootloader, bool overwrite) {
    id = 0;
    if (!mutex) return Result::Unavailable;
    if (destination != Destination::Sd && destination != Destination::Staging) return Result::Unsupported;
    char path[ResmedFirmwareRepository::MAX_PATH];
    if (destination == Destination::Sd) {
        if (!SdStorage::valid_write_path(filename) ||
            !strncasecmp(filename, "/.upload-", 9)) return Result::InvalidName;
        snprintf(path, sizeof(path), "%s", filename);
    } else {
        if (!ResmedFirmwareRepository::valid_filename(filename)) return Result::InvalidName;
        const int length = snprintf(path, sizeof(path), "%s", filename);
        if (length < 0 || size_t(length) >= sizeof(path)) return Result::InvalidName;
    }
    if (size > (destination == Destination::Sd ? MAX_FILE_BYTES : MAX_IMAGE_BYTES)) return Result::InvalidSize;
    if (overwrite && destination != Destination::Sd) return Result::Unsupported;
    if (bootloader && (destination != Destination::Sd || (size && size != 0x4000)))
        return Result::InvalidSize;
    if (destination == Destination::Sd) {
#if AB_STORAGE_HAS_SDCARD
        if (!SdStorage::mounted() || !SdStorage::local_access_allowed()) return Result::Unavailable;
#else
        return Result::Unsupported;
#endif
    }
    if (xSemaphoreTake(mutex, 0) != pdTRUE) return Result::Busy;
    if (active || (status.source_ready && status.destination == Destination::Staging) ||
        claimed || next_id == UINT32_MAX) {
        xSemaphoreGive(mutex);
        return Result::Busy;
    }
    const char *error = nullptr;
    if (destination == Destination::Staging && !OtaManager::begin_manual_upload(&error)) {
        xSemaphoreGive(mutex);
        return Result::Busy;
    }
    capacity = std::min(size ? size : MAX_IMAGE_BYTES, MAX_CHUNK_BYTES);
    ring = static_cast<uint8_t *>(aircannect::Memory::alloc_large(capacity, false));
    if (!ring) {
        capacity = std::min(capacity, INITIAL_CHUNK_BYTES);
        ring = static_cast<uint8_t *>(aircannect::Memory::alloc_large(capacity, true));
    }
    if (!ring) {
        if (destination == Destination::Staging) OtaManager::end_manual_upload(false, "upload_no_memory");
        capacity = 0;
        xSemaphoreGive(mutex);
        return Result::NoMemory;
    }
    const uint32_t revision = status.revision;
    status = {};
    status.revision = revision + 1;
    status.id = id = ++next_id;
    status.destination = destination;
    status.state = State::Receiving;
    status.cancel_allowed = true;
    status.size = size;
    status.chunk_bytes = capacity;
    status.bootloader = bootloader;
    status.overwrite = overwrite;
    status.repository = destination == Destination::Sd &&
        (bootloader || ResmedFirmwareRepository::valid_path(path));
    snprintf(status.path, sizeof(status.path), "%s", path);
    active = true;
    final_received = sealed = claimed = false;
    partition = nullptr;
    produced = consumed = 0;
    last_input = millis();
    // Partition erase/write disables the flash/PSRAM cache: staging must keep
    // its executing stack internal even when the copied input ring is PSRAM.
    const unsigned stack_caps = destination == Destination::Staging
        ? MALLOC_CAP_INTERNAL : MALLOC_CAP_SPIRAM;
    int created = xTaskCreatePinnedToCoreWithCaps(worker, "fw_upload", 6144,
        nullptr, 1, &task, 0, stack_caps | MALLOC_CAP_8BIT);
    if (created != pdPASS && destination != Destination::Staging)
        created = xTaskCreatePinnedToCoreWithCaps(worker, "fw_upload", 6144,
            nullptr, 1, &task, 0, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (created != pdPASS) {
        aircannect::Memory::free(ring);
        ring = nullptr;
        capacity = 0;
        active = false;
        status.complete = true;
        status.state = State::Failed;
        reject("worker_no_memory");
        if (destination == Destination::Staging) OtaManager::end_manual_upload(false, status.error);
        id = 0;
        xSemaphoreGive(mutex);
        return Result::NoMemory;
    }
    xSemaphoreGive(mutex);
    return Result::Accepted;
}

Result submit_chunk(uint32_t id, size_t index, const uint8_t *data, size_t length, bool final) {
    if (!mutex) return Result::Unavailable;
    xSemaphoreTake(mutex, portMAX_DELAY);
    Result result = Result::Accepted;
    const size_t limit = status.bootloader ? 0x4000 :
        status.destination == Destination::Sd ? MAX_FILE_BYTES : MAX_IMAGE_BYTES;
    if (!id || status.id != id) result = Result::UnknownId;
    else if (!active || sealed || status.error[0]) result = Result::Closed;
    else if (final_received || index != status.received || length > status.chunk_bytes ||
             index > limit || length > limit - index ||
             (length && !data) || (status.size && (index > status.size || length > status.size - index)) ||
             (final && !index && !length && status.destination != Destination::Sd) ||
             (final && status.size && index + length != status.size)) {
        result = Result::InvalidChunk;
        reject("invalid_chunk");
    } else if (length > capacity - (produced - consumed)) {
        result = Result::QueueFull;
        reject("queue_full");
    } else {
        const size_t offset = produced % capacity;
        const size_t first = std::min(length, capacity - offset);
        if (first) memcpy(ring + offset, data, first);
        if (length > first) memcpy(ring, data + first, length - first);
        produced += length;
        status.received += length;
        final_received = final;
        if (final) status.size = status.received;
        last_input = millis();
    }
    if (status.id == id && active) changed();
    if (task) xTaskNotifyGive(task);
    xSemaphoreGive(mutex);
    return result;
}

namespace {
bool cancel_upload(uint32_t id, bool discard_ready) {
    if (!mutex) return false;
    xSemaphoreTake(mutex, portMAX_DELAY);
    const bool accepted = id && status.id == id && status.cancel_allowed && !claimed &&
        ((active && !sealed) || (discard_ready && status.source_ready && status.destination == Destination::Staging));
    if (accepted) {
        if (active) reject("cancelled");
        else {
            status.source_ready = false;
            status.state = State::Cancelled;
            reject("cancelled");
            partition = nullptr;
            OtaManager::end_manual_upload(false, "cancelled");
        }
        changed();
    }
    if (accepted && task) xTaskNotifyGive(task);
    xSemaphoreGive(mutex);
    return accepted;
}
}

bool cancel(uint32_t id) { return cancel_upload(id, true); }
bool cancel_input(uint32_t id) { return cancel_upload(id, false); }

void get_status(Status &out) {
    out = {};
    if (!mutex) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    out = status;
    xSemaphoreGive(mutex);
}

bool copy_status(uint32_t id, Status &out) {
    get_status(out);
    return id && out.id == id;
}

Result when_drained(uint32_t id, size_t end, Drained notify) {
    if (!mutex) return Result::Unavailable;
    xSemaphoreTake(mutex, portMAX_DELAY);
    Result result = Result::Accepted;
    bool immediate = false, success = false;
    char error[sizeof(status.error)];
    snprintf(error, sizeof(error), "%s", status.error);
    if (!id || status.id != id) result = Result::UnknownId;
    else if ((!end && !final_received) || end > status.received || !notify) result = Result::InvalidChunk;
    else if (drain_notify) result = Result::Busy;
    else if (status.error[0] || status.complete ||
             (status.written >= end && !(final_received && end == status.received))) {
        immediate = true;
        success = !status.error[0] && status.written >= end;
    } else {
        drain_end = end;
        drain_notify = std::move(notify);
    }
    xSemaphoreGive(mutex);
    if (immediate) notify(success, error);
    return result;
}

bool copy_source(uint32_t id, Source &out) {
    out = {};
    if (!mutex) return false;
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (id && status.id == id && status.source_ready && !claimed && status.destination == Destination::Staging)
        out = {partition, status.size, status.id};
    xSemaphoreGive(mutex);
    return out.partition != nullptr;
}

bool claim_source(uint32_t id, Source &out, const char **error) {
    out = {};
    if (error) *error = "stale_upload";
    if (!mutex) return false;
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (id && status.id == id && status.source_ready && !claimed && status.destination == Destination::Staging &&
        OtaManager::begin_resmed_flash(error, true)) {
        claimed = true;
        status.cancel_allowed = false;
        out = {partition, status.size, status.id};
        changed();
    }
    xSemaphoreGive(mutex);
    return out.partition != nullptr;
}

bool release_source(uint32_t token) {
    if (!mutex) return false;
    xSemaphoreTake(mutex, portMAX_DELAY);
    const bool accepted = token && status.id == token && status.source_ready && status.destination == Destination::Staging;
    if (accepted) {
        if (!claimed) OtaManager::end_manual_upload(true);
        claimed = false;
        partition = nullptr;
        status.source_ready = false;
        status.cancel_allowed = false;
        changed();
    }
    xSemaphoreGive(mutex);
    return accepted;
}

const char *result_name(Result result) {
    switch (result) {
    case Result::Accepted: return "accepted";
    case Result::Busy: return "busy";
    case Result::Unsupported: return "unsupported";
    case Result::Unavailable: return "unavailable";
    case Result::InvalidName: return "invalid_name";
    case Result::InvalidSize: return "invalid_size";
    case Result::UnknownId: return "unknown_id";
    case Result::InvalidChunk: return "invalid_chunk";
    case Result::QueueFull: return "queue_full";
    case Result::NoMemory: return "no_memory";
    case Result::Closed: return "closed";
    }
    return "unavailable";
}

}  // namespace FileUpload
