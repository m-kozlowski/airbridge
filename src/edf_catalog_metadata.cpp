#include "edf_catalog.h"
#include "board.h"

#if AB_STORAGE_HAS_SDCARD

#include <FS.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "air10_clock.h"
#include "crc.h"
#include "memory_manager.h"
#include "report_edf.h"
#include "sd_storage.h"

namespace EdfCatalog {
namespace Metadata { bool rebuild_projection(); }
namespace {

constexpr uint32_t MAX_SESSIONS = 4096;
constexpr uint8_t ALL_FILES = (1u << SESSION_FILE_COUNT) - 1;
constexpr size_t STR_READ_BYTES = 4096;

struct Row {
    SessionMetadata value;
    uint8_t scanned = 0;
    bool numeric_start = false;
    uint32_t retry_at = 0;
};

struct LivePublication {
    SessionMetadata value;
    uint32_t revision = 0, files_revision = 0;
    bool present = false;
};

static portMUX_TYPE live_mux = portMUX_INITIALIZER_UNLOCKED;
static LivePublication live;
static uint32_t reload_revision = 0;
static SemaphoreHandle_t metadata_mutex = nullptr;
static Row *rows = nullptr;
static uint32_t row_count = 0, catalog_generation = 0;
static uint32_t files_revision = 0;
static bool history_ready = false;

static StrMetadata str_metadata;
static uint32_t str_files_revision = 0;
static bool str_known = false;
static uint32_t str_retry_at = 0;
// Only the idle worker advances the STR read between SD turns.
static uint32_t str_crc = 0;
static uint8_t *str_buffer = nullptr;
static uint64_t str_offset = 0;
static StrMetadata pending_str;

static LivePublication live_snapshot() {
    portENTER_CRITICAL(&live_mux);
    const auto result = live;
    portEXIT_CRITICAL(&live_mux);
    return result;
}

static bool valid_day(const char *day) {
    if (!day || strnlen(day, 9) != 8) return false;
    const char native[] = {day[6], day[7], day[4], day[5],
                           day[0], day[1], day[2], day[3], 0};
    Air10Clock::Calendar calendar;
    return Air10Clock::parse_calendar(native, "000000", calendar) && calendar.year >= 1970;
}

static bool parse_prefix(const char *prefix, int64_t &seconds) {
    if (strnlen(prefix, 16) != 15 || prefix[8] != '_') return false;
    const char date[] = {prefix[6], prefix[7], prefix[4], prefix[5],
                         prefix[0], prefix[1], prefix[2], prefix[3], 0};
    Air10Clock::Calendar calendar;
    if (!Air10Clock::parse_calendar(date, prefix + 9, calendar)) return false;
    seconds = Air10Clock::civil_seconds(calendar);
    return true;
}

static void reset_str_read() {
    aircannect::Memory::free(str_buffer);
    str_buffer = nullptr;
    str_offset = 0;
    pending_str = {};
}

static bool resize_rows(uint32_t count) {
    if (count > MAX_SESSIONS) return false;
    if (count != row_count) {
        const size_t bytes = size_t(count) * sizeof(Row);
        auto *next = bytes ? static_cast<Row *>(
            aircannect::Memory::alloc_large(bytes, bytes <= 4096)) : nullptr;
        if (bytes && !next) return false;
        if (bytes) memset(next, 0, bytes);
        if (rows && next) memcpy(next, rows, size_t(std::min(row_count, count)) * sizeof(Row));
        aircannect::Memory::free(rows);
        rows = next;
        row_count = count;
    }
    return true;
}

static void set_entry(Row &row, const Entry &entry, bool reset) {
    if (reset || strcmp(row.value.file_prefix, entry.file_prefix) ||
        strcmp(row.value.therapy_day, entry.therapy_day)) {
        row = {};
        memcpy(row.value.therapy_day, entry.therapy_day, sizeof(entry.therapy_day));
        memcpy(row.value.file_prefix, entry.file_prefix, sizeof(entry.file_prefix));
        parse_prefix(entry.file_prefix, row.value.start_seconds);
    }
    row.value.closed = (entry.flags & ENTRY_LIVE_COMPLETE) && !row.value.part_mask;
}

struct FileRead { fs::File &file; bool failed = false; };

static bool read_fixed(void *context, uint64_t offset, uint8_t *out, size_t size) {
    auto &input = *static_cast<FileRead *>(context);
    const bool ok = offset <= UINT32_MAX && input.file.seek(static_cast<uint32_t>(offset)) &&
                    input.file.read(out, size) == size;
    input.failed |= !ok;
    return ok;
}

static Row inspect_file(fs::FS &storage, Row row) {
    uint8_t slot = 0;
    while (slot < SESSION_FILE_COUNT && (row.scanned & (1u << slot))) ++slot;
    if (slot == SESSION_FILE_COUNT) return row;
    const uint8_t bit = 1u << slot;
    char path[96];
    snprintf(path, sizeof(path), "/DATALOG/%s/%s_%s.edf", row.value.therapy_day,
             row.value.file_prefix, SESSION_SUFFIXES[slot]);
    bool partial = false;
    if (!storage.exists(path)) {
        snprintf(path, sizeof(path), "/DATALOG/%s/%s_%s.edf.part", row.value.therapy_day,
                 row.value.file_prefix, SESSION_SUFFIXES[slot]);
        partial = true;
    }
    if (!storage.exists(path)) { row.scanned |= bit; return row; }
    fs::File file = storage.open(path, FILE_READ);
    if (!file) { row.retry_at = millis() + 1000; return row; }
    if (file.isDirectory() || file.size() > UINT32_MAX) { row.scanned |= bit; return row; }
    const Row before = row;
    row.value.files_mask |= bit;
    if (partial) {
        row.value.part_mask |= bit;
        row.value.closed = false;
    }
    row.value.file_sizes[slot] = file.size();
    if (slot < 3) {
        ReportEdf::Layout layout;
        FileRead input{file};
        const ReportEdf::Reader reader = {read_fixed, &input, file.size()};
        if (ReportEdf::read_layout(reader, layout)) {
            if (!row.numeric_start || layout.start_seconds < row.value.start_seconds)
                row.value.start_seconds = layout.start_seconds;
            row.numeric_start = true;
            if (!partial) {
                const size_t bytes = layout.signal_count * sizeof(ReportEdf::Signal);
                auto *signals = static_cast<ReportEdf::Signal *>(
                    aircannect::Memory::alloc_large(bytes, bytes <= 4096));
                if (!signals) { Row retry = before; retry.retry_at = millis() + 1000; return retry; }
                const bool complete = ReportEdf::read_signals(reader, layout, signals, layout.signal_count);
                aircannect::Memory::free(signals);
                const double duration = layout.records * layout.record_seconds;
                if (complete && std::isfinite(duration) && duration >= 0 && duration <= INT32_MAX)
                    row.value.end_seconds = std::max(row.value.end_seconds,
                        layout.start_seconds + static_cast<int64_t>(duration));
            }
        }
        if (input.failed) { Row retry = before; retry.retry_at = millis() + 1000; return retry; }
    }
    row.scanned |= bit;
    row.retry_at = 0;
    return row;
}

// One open file and at most 4 KiB of source bytes per worker turn.
enum class StrRead { Pending, Ready, Failed };

static StrRead read_str_turn(fs::FS &storage, StrMetadata &result) {
    if (!storage.exists("/STR.edf")) {
        reset_str_read();
        result = {};
        return StrRead::Ready;
    }
    fs::File file = storage.open("/STR.edf", FILE_READ);
    if (!file || file.isDirectory()) return StrRead::Failed;
    if (!str_buffer) {
        pending_str.present = true;
        pending_str.size = file.size();
        pending_str.modified = file.getLastWrite();
        str_crc = crc32_ieee_initial();
        if (pending_str.size) {
            str_buffer = static_cast<uint8_t *>(aircannect::Memory::alloc_large(
                std::min<uint64_t>(STR_READ_BYTES, pending_str.size)));
            if (!str_buffer) return StrRead::Failed;
        }
    }
    if (file.size() != pending_str.size || file.getLastWrite() != pending_str.modified) {
        reset_str_read();
        return StrRead::Pending;
    }
    const size_t count = std::min<uint64_t>(STR_READ_BYTES, pending_str.size - str_offset);
    if (count) {
        if (str_offset > UINT32_MAX || !file.seek(static_cast<uint32_t>(str_offset)) ||
            file.read(str_buffer, count) != count) return StrRead::Failed;
        str_crc = crc32_ieee_update(str_crc, str_buffer, count);
        str_offset += count;
    }
    if (str_offset != pending_str.size) return StrRead::Pending;
    result = pending_str;
    snprintf(result.etag, sizeof(result.etag), "\"%08lx\"",
             static_cast<unsigned long>(crc32_ieee_finish(str_crc)));
    reset_str_read();
    return StrRead::Ready;
}

}  // namespace

namespace Metadata {
// Catalog calls these while holding its mutex, after publishing validated RAM.
// The supplied accessor cannot perform I/O or acquire the catalog mutex again.
void catalog_reloaded(uint32_t count, uint32_t generation, bool (*read_entry)(uint32_t, Entry &)) {
    if (!metadata_mutex) metadata_mutex = xSemaphoreCreateMutex();
    if (!metadata_mutex || xSemaphoreTake(metadata_mutex, portMAX_DELAY) != pdTRUE) return;
    ++reload_revision;
    history_ready = resize_rows(count);
    for (uint32_t i = 0; history_ready && i < count; ++i) {
        Entry entry;
        history_ready = read_entry(i, entry);
        if (history_ready) set_entry(rows[i], entry, true);
    }
    files_revision = SdStorage::files_revision(SdStorage::FileScope::SessionFiles);
    catalog_generation = generation;
    portENTER_CRITICAL(&live_mux);
    if (live.present && (live.value.closed || live.files_revision != files_revision)) {
        live.present = false;
        live.value = {};
        ++live.revision;
    }
    portEXIT_CRITICAL(&live_mux);
    xSemaphoreGive(metadata_mutex);
}

void catalog_committed(uint32_t index, const Entry &entry, uint32_t count, uint32_t generation) {
    if (!metadata_mutex || xSemaphoreTake(metadata_mutex, portMAX_DELAY) != pdTRUE) return;
    if (history_ready && files_revision == SdStorage::files_revision(SdStorage::FileScope::SessionFiles)) {
        history_ready = resize_rows(count) && index < row_count;
        if (history_ready) set_entry(rows[index], entry, false);
    } else history_ready = false;
    catalog_generation = generation;
    xSemaphoreGive(metadata_mutex);
}
}

void publish_session(const SessionMetadata &session) {
    const uint32_t files = SdStorage::files_revision(SdStorage::FileScope::SessionFiles);
    // Preserve a closed segment before a noon rollover replaces the live value.
    // Readers hold this mutex only for a bounded RAM copy, never for network I/O.
    if (session.closed && metadata_mutex && xSemaphoreTake(metadata_mutex, portMAX_DELAY) == pdTRUE) {
        if (history_ready && files_revision == files) {
            for (uint32_t i = 0; i < row_count; ++i) {
                if (strcmp(rows[i].value.file_prefix, session.file_prefix)) continue;
                rows[i].value = session;
                rows[i].scanned = ALL_FILES;
                rows[i].numeric_start = true;
                rows[i].retry_at = 0;
                break;
            }
        }
        xSemaphoreGive(metadata_mutex);
    }
    portENTER_CRITICAL(&live_mux);
    live.value = session;
    live.value.therapy_day[8] = 0;
    live.value.file_prefix[15] = 0;
    live.files_revision = files;
    live.present = true;
    ++live.revision;
    portEXIT_CRITICAL(&live_mux);
}

void refresh_metadata() {
    Status catalog;
    get_status(catalog);
    if (!catalog.ready || !SdStorage::mounted() || !SdStorage::local_access_allowed()) return;

    bool rebuild = !metadata_mutex;
    if (metadata_mutex && xSemaphoreTake(metadata_mutex, 0) == pdTRUE) {
        rebuild = !history_ready || catalog_generation != catalog.generation ||
            files_revision != SdStorage::files_revision(SdStorage::FileScope::SessionFiles);
        xSemaphoreGive(metadata_mutex);
    } else if (metadata_mutex) return;
    // Retry an optional allocation failure without rereading the persisted catalog.
    if (rebuild && !Metadata::rebuild_projection()) return;
    if (!metadata_mutex || !SdStorage::try_acquire_idle()) return;

    const auto work = [&]() {
        if (!SdStorage::local_access_allowed()) return;
        const uint32_t files = SdStorage::files_revision(SdStorage::FileScope::SessionFiles);
        const uint32_t str_files = SdStorage::files_revision(SdStorage::FileScope::Str);
        if (xSemaphoreTake(metadata_mutex, 0) != pdTRUE) return;
        if (!history_ready || files_revision != files) { xSemaphoreGive(metadata_mutex); return; }
        const uint32_t reload = reload_revision;
        if (str_files_revision != str_files) {
            str_known = false;
            str_retry_at = 0;
            str_files_revision = str_files;
            reset_str_read();
        }
        const bool read_str = !str_known && (!str_retry_at || int32_t(millis() - str_retry_at) >= 0);
        uint32_t selected = row_count;
        if (!read_str) {
            for (uint32_t i = row_count; i; --i) {
                if (rows[i - 1].scanned != ALL_FILES &&
                    (!rows[i - 1].retry_at || int32_t(millis() - rows[i - 1].retry_at) >= 0)) {
                    selected = i - 1;
                    break;
                }
            }
        }
        Row selected_row;
        if (selected < row_count) selected_row = rows[selected];
        xSemaphoreGive(metadata_mutex);

        fs::FS *storage = SdStorage::filesystem();
        if (!storage) return;
        if (read_str) {
            StrMetadata result;
            const auto step = read_str_turn(*storage, result);
            if (step == StrRead::Failed) {
                reset_str_read();
                str_retry_at = millis() + 1000;
            }
            if (str_files != SdStorage::files_revision(SdStorage::FileScope::Str)) {
                reset_str_read();
                return;
            }
            if (step == StrRead::Ready && xSemaphoreTake(metadata_mutex, portMAX_DELAY) == pdTRUE) {
                str_metadata = result;
                str_known = true;
                str_retry_at = 0;
                xSemaphoreGive(metadata_mutex);
            }
        } else if (selected < row_count) {
            const Row result = inspect_file(*storage, selected_row);
            if (xSemaphoreTake(metadata_mutex, portMAX_DELAY) == pdTRUE) {
                if (reload == reload_revision && selected < row_count &&
                    files == SdStorage::files_revision(SdStorage::FileScope::SessionFiles))
                    rows[selected] = result;
                xSemaphoreGive(metadata_mutex);
            }
        }
    };
    work();
    SdStorage::release();
}

void release_page(Page &page) {
    aircannect::Memory::free(page.sessions);
    page = {};
}

PageResult copy_page(const char *since, Page &out, const char *cursor, uint32_t limit,
                     bool descending) {
    if (out.sessions) return PageResult::Busy;
    out = {};
    if (since && !valid_day(since)) return PageResult::InvalidDay;
    if (!limit || limit > MAX_PAGE_SIZE) return PageResult::InvalidLimit;
    Row after;
    if (cursor) {
        if (strnlen(cursor, sizeof(out.next_cursor)) != sizeof(out.next_cursor) - 1 || cursor[8] != '/')
            return PageResult::InvalidCursor;
        memcpy(after.value.therapy_day, cursor, 8);
        memcpy(after.value.file_prefix, cursor + 9, 15);
        if (!valid_day(after.value.therapy_day) ||
            !parse_prefix(after.value.file_prefix, after.value.start_seconds))
            return PageResult::InvalidCursor;
    }
    if (!metadata_mutex || !SdStorage::mounted() || !SdStorage::local_access_allowed())
        return PageResult::Unavailable;
    if (xSemaphoreTake(metadata_mutex, 0) != pdTRUE) return PageResult::Busy;
    const auto copy = [&]() -> PageResult {
        Status catalog;
        get_status(catalog);
        const uint32_t files = SdStorage::files_revision(SdStorage::FileScope::SessionFiles);
        const auto publication = live_snapshot();
        if (!catalog.ready || !history_ready || catalog_generation != catalog.generation ||
            files_revision != files || !str_known ||
            str_files_revision != SdStorage::files_revision(SdStorage::FileScope::Str))
            return PageResult::Unavailable;
        const bool live_selected = publication.present && publication.files_revision == files;
        Row live_row;
        live_row.value = publication.value;
        live_row.scanned = ALL_FILES;
        const Row *selected[MAX_PAGE_SIZE];
        uint32_t count = 0, matches = 0;
        const auto before = [descending](const Row *a, const Row *b) {
            const int day = strcmp(a->value.therapy_day, b->value.therapy_day);
            const int key = day ? day : strcmp(a->value.file_prefix, b->value.file_prefix);
            return descending ? key > 0 : key < 0;
        };
        // Retain only the requested page, regardless of the catalog's stored order.
        const auto consider = [&](const Row &row) {
            if ((since && strcmp(row.value.therapy_day, since) < 0) ||
                (cursor && !before(&after, &row))) return;
            ++matches;
            if (count < limit) {
                selected[count++] = &row;
                std::push_heap(selected, selected + count, before);
            } else if (before(&row, selected[0])) {
                std::pop_heap(selected, selected + count, before);
                selected[count - 1] = &row;
                std::push_heap(selected, selected + count, before);
            }
        };
        bool merged = false;
        for (uint32_t i = 0; i < row_count; ++i) {
            const bool current = live_selected && !strcmp(rows[i].value.file_prefix, publication.value.file_prefix);
            merged |= current;
            consider(current ? live_row : rows[i]);
        }
        if (live_selected && !merged) consider(live_row);
        std::sort_heap(selected, selected + count, before);
        for (uint32_t i = 0; i < count; ++i)
            if (selected[i]->scanned != ALL_FILES) return PageResult::Unavailable;
        const size_t bytes = size_t(count) * sizeof(SessionMetadata);
        out.sessions = bytes ? static_cast<SessionMetadata *>(
            aircannect::Memory::alloc_large(bytes, bytes <= 4096)) : nullptr;
        if (bytes && !out.sessions) return PageResult::Unavailable;
        for (uint32_t i = 0; i < count; ++i) out.sessions[out.count++] = selected[i]->value;
        if (matches > count) {
            const auto &last = out.sessions[count - 1];
            snprintf(out.next_cursor, sizeof(out.next_cursor), "%s/%s", last.therapy_day, last.file_prefix);
        }
        out.str = str_metadata;
        const auto latest = live_snapshot();
        Status current;
        get_status(current);
        if (!current.ready || current.generation != catalog_generation ||
            latest.revision != publication.revision ||
            files != SdStorage::files_revision(SdStorage::FileScope::SessionFiles) ||
            str_files_revision != SdStorage::files_revision(SdStorage::FileScope::Str) ||
            !SdStorage::mounted() || !SdStorage::local_access_allowed()) {
            release_page(out);
            return PageResult::Unavailable;
        }
        return PageResult::Ready;
    };
    const auto result = copy();
    xSemaphoreGive(metadata_mutex);
    return result;
}

}  // namespace EdfCatalog

#else

namespace EdfCatalog {
void publish_session(const SessionMetadata &) {}
void refresh_metadata() {}
void release_page(Page &page) { page = {}; }
PageResult copy_page(const char *, Page &out, const char *, uint32_t, bool) {
    out = {};
    return PageResult::Unavailable;
}
}

#endif
