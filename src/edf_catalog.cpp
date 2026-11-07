#include "edf_catalog.h"

#include "board.h"

#if AB_STORAGE_HAS_SDCARD

#include <Arduino.h>
#include <FS.h>
#include <algorithm>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <stdio.h>
#include <string.h>

#include "crc.h"
#include "debug_log.h"
#include "memory_manager.h"
#include "sd_storage.h"

namespace EdfCatalog {
namespace Metadata {
void catalog_reloaded(uint32_t count, uint32_t generation, bool (*read)(uint32_t, Entry &));
void catalog_committed(uint32_t index, const Entry &entry, uint32_t count, uint32_t generation);
}
namespace {

constexpr const char *CATALOG_DIR = "/airbridge";
constexpr const char *CATALOG_PATH = "/airbridge/catalog.bin";
constexpr const char *CATALOG_PART = "/airbridge/catalog.bin.part";
constexpr const char *CATALOG_BACKUP = "/airbridge/catalog.bin.bak";
constexpr uint16_t CATALOG_VERSION = 1;
constexpr uint16_t HEADER_SIZE = 24;
constexpr uint16_t ENTRY_SIZE = 32;
constexpr uint32_t MAX_ENTRIES = 4096;

struct Header {
    uint32_t count;
    uint32_t generation;
    uint32_t entries_crc;
};

static Status status = {true};
static uint32_t status_revision = 0;
static portMUX_TYPE status_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t changed_indexes[32] = {};
static uint32_t change_count = 0;
static SemaphoreHandle_t mutex = nullptr;
static fs::FS *storage = nullptr;
// Keep the 32-byte wire representation: 4096 entries occupy at most 128 KiB.
static uint8_t *entries = nullptr;
static uint32_t validated_files_revision = 0;
static bool initialized = false;

static void set_error(const char *message) {
    portENTER_CRITICAL(&status_mux);
    status.ready = false;
    strncpy(status.error, message ? message : "catalog error",
            sizeof(status.error) - 1);
    status.error[sizeof(status.error) - 1] = 0;
    __atomic_add_fetch(&status_revision, 1, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&status_mux);
    Log::logf(CAT_EDF, LOG_ERROR, "catalog: %s\n",
              message ? message : "catalog error");
}

static bool digits_only(const char *text, size_t size) {
    if (!text) return false;
    for (size_t i = 0; i < size; i++)
        if (text[i] < '0' || text[i] > '9') return false;
    return true;
}

static bool valid_digits(const char *text, size_t size) {
    return digits_only(text, size) && text[size] == 0;
}

static bool valid_entry(const Entry &entry) {
    return valid_digits(entry.therapy_day, 8) &&
           strlen(entry.file_prefix) == 15 &&
           digits_only(entry.file_prefix, 8) &&
           entry.file_prefix[8] == '_' &&
           valid_digits(entry.file_prefix + 9, 6);
}

static void encode_header(const Header &header, uint8_t *out) {
    memcpy(out, "ABEC", 4);
    SdStorage::put_le16(out + 4, CATALOG_VERSION);
    SdStorage::put_le16(out + 6, HEADER_SIZE);
    SdStorage::put_le16(out + 8, ENTRY_SIZE);
    SdStorage::put_le16(out + 10, 0);
    SdStorage::put_le32(out + 12, header.count);
    SdStorage::put_le32(out + 16, header.generation);
    SdStorage::put_le32(out + HEADER_SIZE - 4, header.entries_crc);
}

static bool decode_header(const uint8_t *data, Header &header) {
    if (memcmp(data, "ABEC", 4) != 0 ||
        SdStorage::get_le16(data + 4) != CATALOG_VERSION ||
        SdStorage::get_le16(data + 6) != HEADER_SIZE ||
        SdStorage::get_le16(data + 8) != ENTRY_SIZE) {
        return false;
    }
    header.count = SdStorage::get_le32(data + 12);
    header.generation = SdStorage::get_le32(data + 16);
    header.entries_crc = SdStorage::get_le32(data + HEADER_SIZE - 4);
    return header.count <= MAX_ENTRIES;
}

static void encode_entry(const Entry &entry, uint8_t *out) {
    memset(out, 0, ENTRY_SIZE);
    memcpy(out, entry.therapy_day, 8);
    memcpy(out + 8, entry.file_prefix, 15);
    out[23] = entry.flags;
    SdStorage::put_le32(out + 24, entry.finalized_epoch);
    SdStorage::put_le32(out + 28, entry.str_digest);
}

static bool decode_entry(const uint8_t *data, Entry &entry) {
    memset(&entry, 0, sizeof(entry));
    memcpy(entry.therapy_day, data, 8);
    memcpy(entry.file_prefix, data + 8, 15);
    entry.flags = data[23];
    entry.finalized_epoch = SdStorage::get_le32(data + 24);
    entry.str_digest = SdStorage::get_le32(data + 28);
    return valid_entry(entry);
}

// The caller holds the catalog mutex; this accessor never loads or touches SD.
static bool metadata_entry(uint32_t index, Entry &entry) {
    return index < status.entries && decode_entry(entries + size_t(index) * ENTRY_SIZE, entry);
}

static bool read_header(fs::File &file, Header &header) {
    uint8_t bytes[HEADER_SIZE];
    if (!file || file.size() < HEADER_SIZE ||
        file.read(bytes, sizeof(bytes)) != sizeof(bytes) ||
        !decode_header(bytes, header) ||
        file.size() != HEADER_SIZE + header.count * ENTRY_SIZE) {
        return false;
    }

    return true;
}

static bool read_valid_entries(fs::File &file, const Header &header, uint8_t *destination = nullptr) {
    const size_t length = size_t(header.count) * ENTRY_SIZE;
    const size_t block_size = std::min(length, SdStorage::READ_CHUNK_BYTES);
    auto *scratch = !destination && block_size ? static_cast<uint8_t *>(
        aircannect::Memory::alloc_large(block_size)) : nullptr;
    if (!destination && block_size && !scratch) return false;
    uint32_t crc = crc32_ieee_initial();
    bool valid = true;
    for (size_t at = 0; valid && at < length; at += block_size) {
        const size_t count = std::min(block_size, length - at);
        uint8_t *block = destination ? destination + at : scratch;
        valid = file.read(block, count) == count;
        if (!valid) break;
        crc = crc32_ieee_update(crc, block, count);
        for (size_t i = 0; valid && i < count; i += ENTRY_SIZE) {
            Entry decoded;
            valid = decode_entry(block + i, decoded);
        }
    }
    aircannect::Memory::free(scratch);
    return valid && crc32_ieee_finish(crc) == header.entries_crc;
}

static bool read_valid_header(fs::File &file, Header &header) {
    return read_header(file, header) && read_valid_entries(file, header);
}

// Called only under the catalog mutex and the caller's SD ownership.
static bool load_catalog() {
    const uint32_t revision = SdStorage::files_revision(SdStorage::FileScope::Catalog);
    if (status.ready && validated_files_revision == revision) return true;
    storage = SdStorage::filesystem();
    if (!storage) return false;
    fs::File file = storage->open(CATALOG_PATH, FILE_READ);
    Header header;
    if (!read_header(file, header)) {
        set_error("validation failed");
        return false;
    }
    const size_t bytes = size_t(header.count) * ENTRY_SIZE;
    auto *loaded = bytes ? static_cast<uint8_t *>(
        aircannect::Memory::alloc_large(bytes, bytes <= 4096)) : nullptr;
    if (bytes && !loaded) {
        set_error("catalog allocation failed");
        return false;
    }
    if (!read_valid_entries(file, header, loaded) ||
        revision != SdStorage::files_revision(SdStorage::FileScope::Catalog)) {
        aircannect::Memory::free(loaded);
        set_error("validation failed");
        return false;
    }
    aircannect::Memory::free(entries);
    entries = loaded;
    portENTER_CRITICAL(&status_mux);
    status.ready = false;
    initialized = true;
    validated_files_revision = revision;
    change_count = 0;
    status.entries = header.count;
    status.generation = header.generation;
    status.error[0] = 0;
    portEXIT_CRITICAL(&status_mux);
    Metadata::catalog_reloaded(status.entries, status.generation, metadata_entry);
    portENTER_CRITICAL(&status_mux);
    status.ready = true;
    __atomic_add_fetch(&status_revision, 1, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&status_mux);
    return true;
}

static bool write_empty_catalog() {
    Header header = {0, 1, crc32_ieee(nullptr, 0)};
    uint8_t bytes[HEADER_SIZE];
    encode_header(header, bytes);
    fs::File file = storage->open(CATALOG_PATH, FILE_WRITE);
    if (!file || !SdStorage::write_exact(file, bytes, sizeof(bytes))) {
        if (file) file.close();
        return false;
    }
    file.flush();
    file.close();
    portENTER_CRITICAL(&status_mux);
    status.entries = 0;
    status.generation = header.generation;
    portEXIT_CRITICAL(&status_mux);
    return true;
}

static bool publish_catalog() {
    if (storage->exists(CATALOG_BACKUP) &&
        !storage->remove(CATALOG_BACKUP)) {
        return false;
    }
    if (storage->exists(CATALOG_PATH) &&
        !storage->rename(CATALOG_PATH, CATALOG_BACKUP)) {
        return false;
    }
    if (!storage->rename(CATALOG_PART, CATALOG_PATH)) {
        (void)storage->rename(CATALOG_BACKUP, CATALOG_PATH);
        return false;
    }
    if (storage->exists(CATALOG_BACKUP)) storage->remove(CATALOG_BACKUP);
    return true;
}

static void recover_catalog() {
    if (!storage->exists(CATALOG_PATH) && storage->exists(CATALOG_BACKUP)) {
        if (!storage->rename(CATALOG_BACKUP, CATALOG_PATH)) {
            set_error("backup recovery failed");
            return;
        }
    }
}

}  // namespace

void init() {
    if (status.ready && validated_files_revision == SdStorage::files_revision(SdStorage::FileScope::Catalog)) return;
    portENTER_CRITICAL(&status_mux);
    status.error[0] = 0;
    __atomic_add_fetch(&status_revision, 1, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&status_mux);
    storage = SdStorage::filesystem();
    if (!storage) return;
    if (!mutex) mutex = xSemaphoreCreateMutex();
    if (!mutex) {
        set_error("mutex allocation failed");
        return;
    }
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(2000)) != pdTRUE) return;
    if (!storage->exists(CATALOG_DIR) && !storage->mkdir(CATALOG_DIR)) {
        set_error("directory creation failed");
        xSemaphoreGive(mutex);
        return;
    }
    recover_catalog();
    if (status.error[0]) { xSemaphoreGive(mutex); return; }
    if (!initialized && !storage->exists(CATALOG_PATH) && !write_empty_catalog()) {
        set_error("initialization failed");
        xSemaphoreGive(mutex);
        return;
    }
    if (!load_catalog()) {
        xSemaphoreGive(mutex);
        return;
    }
    // Only discard recovery copies after the published catalog is validated.
    if (storage->exists(CATALOG_BACKUP)) storage->remove(CATALOG_BACKUP);
    if (storage->exists(CATALOG_PART)) storage->remove(CATALOG_PART);
    xSemaphoreGive(mutex);
    Log::logf(CAT_EDF, LOG_INFO,
              "catalog ready entries=%u generation=%u\n",
              status.entries, status.generation);
}

bool commit(const Entry &entry) {
    if (!mutex || !valid_entry(entry) ||
        xSemaphoreTake(mutex, pdMS_TO_TICKS(2000)) != pdTRUE) {
        return false;
    }
    if (!load_catalog()) { xSemaphoreGive(mutex); return false; }
    uint32_t changed_index = status.entries;
    for (uint32_t i = 0; i < status.entries; ++i) {
        if (!memcmp(entries + i * ENTRY_SIZE + 8, entry.file_prefix, 15)) {
            changed_index = i;
            break;
        }
    }
    Header new_header = {
        status.entries + (changed_index == status.entries ? 1u : 0u),
        status.generation + 1,
        0,
    };
    const size_t size = size_t(new_header.count) * ENTRY_SIZE;
    auto *next = new_header.count <= MAX_ENTRIES ? static_cast<uint8_t *>(
        aircannect::Memory::alloc_large(size, size <= 4096)) : nullptr;
    if (!next) { xSemaphoreGive(mutex); return false; }
    if (status.entries) memcpy(next, entries, size_t(status.entries) * ENTRY_SIZE);
    encode_entry(entry, next + changed_index * ENTRY_SIZE);
    new_header.entries_crc = crc32_ieee(next, size);
    uint8_t header_bytes[HEADER_SIZE];
    encode_header(new_header, header_bytes);

    storage->remove(CATALOG_PART);
    fs::File output = storage->open(CATALOG_PART, FILE_WRITE);
    bool success = output && SdStorage::write_exact(output, header_bytes, sizeof(header_bytes));
    for (size_t at = 0; success && at < size;) {
        const size_t count = std::min(size - at, SdStorage::WRITE_CHUNK_BYTES);
        success = SdStorage::write_exact(output, next + at, count);
        at += count;
    }
    if (output) {
        output.flush();
        output.close();
    }
    if (success) {
        fs::File check = storage->open(CATALOG_PART, FILE_READ);
        Header verified;
        success = read_valid_header(check, verified) &&
            verified.count == new_header.count && verified.generation == new_header.generation &&
            verified.entries_crc == new_header.entries_crc;
    }
    if (success && !publish_catalog()) {
        success = false;
        set_error("publication failed");
    }
    if (!success) storage->remove(CATALOG_PART);

    if (success) {
        aircannect::Memory::free(entries);
        entries = next;
        next = nullptr;
        portENTER_CRITICAL(&status_mux);
        status.entries = new_header.count;
        status.generation = new_header.generation;
        changed_indexes[status.generation % 32] = changed_index;
        if (change_count < 32) ++change_count;
        __atomic_add_fetch(&status_revision, 1, __ATOMIC_RELEASE);
        portEXIT_CRITICAL(&status_mux);
        Metadata::catalog_committed(changed_index, entry, status.entries, status.generation);
        Log::logf(CAT_EDF, LOG_DEBUG,
                  "catalog commit %s entries=%u generation=%u\n",
                  entry.file_prefix, status.entries, status.generation);
    }
    aircannect::Memory::free(next);
    xSemaphoreGive(mutex);
    return success;
}

bool read(uint32_t index, Entry &entry) {
    if (!mutex ||
        xSemaphoreTake(mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return false;
    }
    const bool success = load_catalog() && index < status.entries &&
        decode_entry(entries + size_t(index) * ENTRY_SIZE, entry);
    xSemaphoreGive(mutex);
    return success;
}

bool find(const char *file_prefix, Entry &entry) {
    if (!mutex || !file_prefix ||
        strlen(file_prefix) != 15 ||
        xSemaphoreTake(mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return false;
    }

    bool found = false;
    if (load_catalog()) {
        for (uint32_t i = 0; i < status.entries; i++) {
            const auto *bytes = entries + size_t(i) * ENTRY_SIZE;
            if (!memcmp(bytes + 8, file_prefix, 15)) {
                found = decode_entry(bytes, entry);
                break;
            }
        }
    }
    xSemaphoreGive(mutex);
    return found;
}

bool snapshot_prefixes(char *out, size_t out_size, uint32_t &count) {
    count = 0;
    if (!mutex ||
        xSemaphoreTake(mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return false;
    }

    const bool success = load_catalog() && status.entries <= out_size / 16 &&
        (!status.entries || out);
    for (uint32_t i = 0; success && i < status.entries; i++) {
        memcpy(out + size_t(i) * 16, entries + size_t(i) * ENTRY_SIZE + 8, 15);
        out[size_t(i) * 16 + 15] = 0;
    }
    if (success) count = status.entries;
    xSemaphoreGive(mutex);
    return success;
}

bool snapshot_day(const char *day, Entry *&out, uint32_t &count) {
    out = nullptr;
    count = 0;
    if (!mutex || !day || !valid_digits(day, 8) ||
        xSemaphoreTake(mutex, pdMS_TO_TICKS(1000)) != pdTRUE) return false;

    bool success = load_catalog();
    uint32_t capacity = 0;
    for (uint32_t i = 0; success && i < status.entries; i++) {
        const auto *bytes = entries + size_t(i) * ENTRY_SIZE;
        if (memcmp(bytes, day, 8)) continue;
        Entry entry;
        success = decode_entry(bytes, entry);
        if (!success) break;
        if (count == capacity) {
            const uint32_t next = capacity ? capacity * 2 : 4;
            // A typical day needs only a few entries; large days require PSRAM.
            void *grown = aircannect::Memory::realloc_large(
                out, next * sizeof(Entry), next <= 32);
            if (!grown) { success = false; break; }
            out = static_cast<Entry *>(grown);
            capacity = next;
        }
        out[count++] = entry;
    }
    xSemaphoreGive(mutex);
    if (!success) {
        aircannect::Memory::free(out);
        out = nullptr;
        count = 0;
    }
    return success;
}

void get_status(Status &out) {
    portENTER_CRITICAL(&status_mux);
    out = status;
    out.ready &= validated_files_revision == SdStorage::files_revision(SdStorage::FileScope::Catalog);
    portEXIT_CRITICAL(&status_mux);
}

namespace Metadata {
bool rebuild_projection() {
    if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return false;
    const bool ready = status.ready && validated_files_revision ==
        SdStorage::files_revision(SdStorage::FileScope::Catalog);
    if (ready) catalog_reloaded(status.entries, status.generation, metadata_entry);
    xSemaphoreGive(mutex);
    return ready;
}
}

uint32_t revision() {
    return __atomic_load_n(&status_revision, __ATOMIC_ACQUIRE);
}

bool changes_since(uint32_t generation, Changes &out) {
    portENTER_CRITICAL(&status_mux);
    out.catalog = status;
    out.catalog.ready &= validated_files_revision == SdStorage::files_revision(SdStorage::FileScope::Catalog);
    const uint32_t distance = status.generation - generation;
    const bool complete = out.catalog.ready && distance <= change_count;
    out.count = complete ? distance : 0;
    for (uint32_t i = 0; i < out.count; ++i)
        out.indexes[i] = changed_indexes[(generation + i + 1) % 32];
    portEXIT_CRITICAL(&status_mux);
    return complete;
}

}  // namespace EdfCatalog

#else

namespace EdfCatalog {

void init() {}
uint32_t revision() { return 0; }
bool commit(const Entry &) { return false; }
bool changes_since(uint32_t, Changes &out) { out = {}; return false; }
bool read(uint32_t, Entry &) { return false; }
bool find(const char *, Entry &) { return false; }
bool snapshot_prefixes(char *, size_t, uint32_t &count) {
    count = 0;
    return false;
}
bool snapshot_day(const char *, Entry *&out, uint32_t &count) {
    out = nullptr;
    count = 0;
    return false;
}

void get_status(Status &out) {
    out = {};
    out.supported = false;
}

}  // namespace EdfCatalog

#endif
