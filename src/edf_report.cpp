#include "edf_report.h"
#include "board.h"
#include "sd_storage.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <utility>

namespace EdfReport {

bool Request::operator==(const Request &other) const {
    return action == other.action && view == other.view && day == other.day &&
        period == other.period && from_ms == other.from_ms && to_ms == other.to_ms &&
        px == other.px && revision == other.revision &&
        !memcmp(excluded, other.excluded, sizeof(excluded));
}

bool valid(const Request &request) {
    if (request.action > Action::Series || request.view > View::Period ||
        request.day == UINT16_MAX) return false;
    if (request.view == View::Period && request.period != 1 && request.period != 7 &&
        request.period != 30 && request.period != 90 && request.period != 180 &&
        request.period != 365) return false;
    for (uint8_t excluded : request.excluded)
        if (excluded && (request.action != Action::Series || !request.revision)) return false;
    return request.action != Action::Series ||
        (request.from_ms > 0 && request.to_ms > request.from_ms &&
         request.to_ms - request.from_ms <= 366ll * 86400000 &&
         request.px && request.px <= 1600);
}

const char *state_name(State state) {
    switch (state) {
        case State::Ready: return "ready";
        case State::WaitingStorage: return "waiting_storage";
        case State::Building: return "building";
        case State::Blocked: return "blocked";
        default: return "error";
    }
}

size_t Result::read(size_t offset, char *out, size_t capacity) const {
    if (!data_ || offset >= length_ || !out) return 0;
    const size_t count = std::min(capacity, length_ - offset);
    memcpy(out, data_ + offset, count);
    return count;
}

Result::~Result() { reset(); }
Result::Result(Result &&other) noexcept
    : id_(std::exchange(other.id_, 0)), data_(std::exchange(other.data_, nullptr)),
      length_(std::exchange(other.length_, 0)) {}

}  // namespace EdfReport

#if AB_STORAGE_HAS_SDCARD
#include "air10_clock.h"
#include "airsense_state.h"
#include "edf_catalog.h"
#include "debug_log.h"
#include "memory_manager.h"
#include "large_text_buffer.h"
#include "hex_util.h"
#include "report_edf.h"
#include <Arduino.h>
#include <FS.h>
#include <esp_heap_caps.h>
#include <freertos/semphr.h>
#include <cstdarg>
#include <cstdio>
#include <new>

namespace EdfReport {
namespace {
constexpr uint32_t STATS_CACHE_VERSION = 6;
constexpr uint32_t SERIES_CACHE_VERSION = 5;
constexpr size_t MAX_BINS = 16384;
constexpr size_t MAX_EVENTS = 4096;
constexpr size_t MAX_DISK_EVENTS = 65536;
constexpr size_t MAX_BODY = 512 * 1024;
constexpr size_t INTERNAL_BODY = 16 * 1024;
constexpr size_t INTERNAL_WORK = 8 * 1024;
constexpr uint64_t MAX_SD_CACHE = 128ull * 1024 * 1024;
constexpr uint32_t RETAIN_MS = 30000;
constexpr size_t SLOT_COUNT = 2;
constexpr size_t CHANNEL_COUNT = 9;
constexpr int64_t LEVEL_MS[] = {10000, 60000, 600000, 3600000};
constexpr size_t LEVEL_COUNT = sizeof(LEVEL_MS) / sizeof(LEVEL_MS[0]);
constexpr const char *CACHE_DIR = "/airbridge/report";
constexpr const char *SUFFIXES[] = {"BRP", "PLD", "SAD", "EVE", "CSL"};

struct ChannelSpec {
    const char *id, *label, *unit, *signal;
    uint8_t file;
};
constexpr ChannelSpec CHANNELS[] = {
    {"Press", "Pressure", "cmH2O", "Press.2s", 1},
    {"EprPress", "Expiratory pressure", "cmH2O", "EprPress.2s", 1},
    {"MaskPress", "Mask pressure", "cmH2O", "MaskPress.2s", 1},
    {"Leak", "Leak", "L/min", "Leak.2s", 1},
    {"MinVent", "Minute ventilation", "L/min", "MinVent.2s", 1},
    {"RespRate", "Respiratory rate", "bpm", "RespRate.2s", 1},
    {"TidVol", "Tidal volume", "mL", "TidVol.2s", 1},
    {"SpO2", "SpO2", "%", "SpO2.1s", 2},
    {"Pulse", "Pulse", "bpm", "Pulse.1s", 2},
};

// Sidecars are identity inputs, not recomputed source-data checksums.
struct SourceKey {
    uint64_t sizes[5] = {};
    uint8_t sidecars[5][8] = {};
    uint8_t present = 0, checksums = 0;

    bool operator==(const SourceKey &other) const {
        return present == other.present && checksums == other.checksums &&
            !memcmp(sizes, other.sizes, sizeof(sizes)) &&
            !memcmp(sidecars, other.sidecars, sizeof(sidecars));
    }
};
struct IndexedSession {
    EdfCatalog::Entry entry = {};
    SourceKey key;
    uint16_t day = 0;
    int64_t start_ms = 0, end_ms = 0;
    int64_t event_from_ms = 0, event_to_ms = 0;
    bool bounds_known = false;
};
struct Event {
    int64_t time_ms = 0, duration_ms = 0;
    ReportEdf::EventKind kind = ReportEdf::EventKind::Unknown;
};
struct CachedChannel {
    ReportEdf::Signal signal;
    uint64_t count = 0, missing = 0;
    double sum = 0, coverage_seconds = 0;
    uint32_t bins = 0;
};
struct CacheHeader {
    uint32_t version = STATS_CACHE_VERSION, header_bytes = sizeof(CacheHeader), bytes = 0;
    char magic[8] = {'A','B','R','P','T','0','1',0};
    SourceKey key;
    int64_t start_ms = 0, end_ms = 0;
    int64_t chart_start_ms = 0, chart_end_ms = 0;
    int64_t event_from_ms = 0, event_to_ms = 0;
    double used_seconds = 0;
    uint64_t counts[5] = {};
    CachedChannel channels[CHANNEL_COUNT];
    uint32_t events = 0;
    uint16_t day = 0;
    bool recorded = false, partial = false, events_truncated = false, events_known = false;
    bool usage_known = false;
};
struct SessionData {
    CacheHeader header;
    uint64_t *bins = nullptr;
    Event *events = nullptr;
    ~SessionData() {
        aircannect::Memory::free(bins);
        aircannect::Memory::free(events);
    }
};

struct Payload : aircannect::LargeTextBuffer {
    Payload() : LargeTextBuffer(MAX_BODY, INTERNAL_BODY) {}
    bool format_error = false;
    bool good() const { return !format_error && !overflowed(); }
    bool print(const char *format, ...) {
        char text[256];
        va_list args;
        va_start(args, format);
        const int count = vsnprintf(text, sizeof(text), format, args);
        va_end(args);
        if (count < 0 || size_t(count) >= sizeof(text)) { format_error = true; return false; }
        return append(text, size_t(count));
    }
};

enum class JobState : uint8_t { Free, Queued, Running, Complete };
struct Job {
    Request request;
    JobState state = JobState::Free;
    uint32_t id = 0, completed_ms = 0, data_revision = 0;
    uint16_t readers = 0;
    uint16_t consumers = 1, deliveries = 0;
    bool delivered = false, cancelled = false, shared = false;
    int code = 0;
    const char *error = nullptr;
    mutable const char *issue = nullptr;
    mutable char issue_path[96] = {};
    Payload body;
};

bool note_issue(const Job &job, const char *reason, const char *path = "") {
    if (!job.issue) {
        job.issue = reason;
        snprintf(job.issue_path, sizeof(job.issue_path), "%s", path);
    }
    return false;
}

void log_result(const Job &job) {
    const bool expected = job.code == 409 || job.code == 410;
    const char *reason = expected ? job.error : job.issue ? job.issue : job.error;
    Log::logf(CAT_REPORT, expected ? LOG_DEBUG : job.code >= 500 ? LOG_ERROR :
        job.issue ? LOG_WARN : LOG_DEBUG,
        "EDF job=%lu %s day=%u code=%d %s path=%s\n",
        (unsigned long)job.id, job.request.action == Action::Days ? "days" :
        job.request.action == Action::Summary ? "summary" : "series",
        unsigned(job.request.day), job.code, reason ? reason : "complete",
        expected || !job.issue_path[0] ? "-" : job.issue_path);
}

struct JobTiming {
    bool enabled;
    uint32_t id, started_us;
    Action action;
    uint32_t index_us = 0, numeric_us = 0, events_us = 0, encode_us = 0;

    explicit JobTiming(const Job *job)
        : enabled(job && Log::get_cat_level(CAT_REPORT) >= LOG_DEBUG),
          id(job ? job->id : 0), started_us(stamp()),
          action(job ? job->request.action : Action::Days) {}

    uint32_t stamp() const { return enabled ? micros() : 0; }

    void log(int code, size_t bytes) const {
        if (!enabled) return;
        Log::logf(CAT_REPORT, LOG_DEBUG,
            "report job=%lu action=%s code=%d bytes=%u ms total=%lu "
            "idx=%lu num=%lu evt=%lu enc=%lu\n",
            static_cast<unsigned long>(id), action == Action::Days ? "days" :
                action == Action::Summary ? "summary" : "series", code,
            static_cast<unsigned>(bytes), static_cast<unsigned long>((stamp() - started_us) / 1000),
            static_cast<unsigned long>(index_us / 1000), static_cast<unsigned long>(numeric_us / 1000),
            static_cast<unsigned long>(events_us / 1000), static_cast<unsigned long>(encode_us / 1000));
    }
};

Job jobs[SLOT_COUNT];
SemaphoreHandle_t mutex = nullptr, wake = nullptr;
TaskHandle_t worker_task = nullptr;
portMUX_TYPE status_mux = portMUX_INITIALIZER_UNLOCKED;
Status status;
uint32_t next_id = 1, data_revision = 1;
IndexedSession *index = nullptr;
size_t index_count = 0;
uint32_t catalog_generation = 0, file_revision = 0;
uint32_t invalidation_generation = 0, indexed_invalidation = 0;
bool index_ready = false;
bool cache_usage_known = false;
uint64_t cache_bytes = 0;
enum class IndexFailure : uint8_t { Error, WaitingCatalog, Preempted };
IndexFailure index_failure = IndexFailure::Error;
const char *index_error = nullptr;
size_t index_error_entry = SIZE_MAX;
bool prune_cache(const SdStorage::Session &session, uint64_t needed = 0,
                 const char *keep = nullptr);

bool retire_cache(const SdStorage::Session &session, const IndexedSession &entry) {
    bool removed = false;
    const bool success = session.run([&](fs::FS &fs) {
        for (const char *suffix : {"stats", "series", "stats.part", "series.part"}) {
            char path[96];
            snprintf(path, sizeof(path), "%s/%s.%s", CACHE_DIR, entry.entry.file_prefix, suffix);
            if (fs.exists(path)) {
                if (!fs.remove(path)) return false;
                removed = true;
            }
        }
        return true;
    });
    if (removed) cache_usage_known = false;
    return success;
}

void publish(State state, const char *error = "") {
    portENTER_CRITICAL(&status_mux);
    if (status.state != state || strcmp(status.error, error)) {
        status.state = state;
        snprintf(status.error, sizeof(status.error), "%s", error);
        ++status.revision;
    }
    portEXIT_CRITICAL(&status_mux);
}

bool reusable(const Job &job) {
    return job.state == JobState::Free || (job.state == JobState::Complete &&
        !job.readers && (job.cancelled || uint32_t(millis() - job.completed_ms) >=
            RETAIN_MS));
}
void reset_job(Job &job) {
    job.~Job();
    new (&job) Job;
}
bool aborted(const Job *job, const SdStorage::Session &session) {
    return (job && __atomic_load_n(&job->cancelled, __ATOMIC_ACQUIRE)) ||
        !AirSenseState::local_background_allowed() || !session.valid();
}

void source_path(const IndexedSession &entry, uint8_t file, const char *ext,
                 char (&out)[96]) {
    snprintf(out, sizeof(out), "/DATALOG/%s/%s_%s.%s", entry.entry.therapy_day,
             entry.entry.file_prefix, SUFFIXES[file], ext);
}
void cache_path(const IndexedSession &entry, const char *ext, char (&out)[96]) {
    snprintf(out, sizeof(out), "%s/%s.%s", CACHE_DIR, entry.entry.file_prefix, ext);
}

bool catalog_day(const char *text, uint16_t &out) {
    if (strlen(text) != 8) return false;
    int parts[3] = {};
    for (size_t i = 0; i < 8; ++i) {
        if (text[i] < '0' || text[i] > '9') return false;
        const size_t part = i < 4 ? 0 : i < 6 ? 1 : 2;
        parts[part] = parts[part] * 10 + text[i] - '0';
    }
    char dac[9];
    snprintf(dac, sizeof(dac), "%02d%02d%04d", parts[2], parts[1], parts[0]);
    Air10Clock::Calendar calendar;
    if (!Air10Clock::parse_calendar(dac, "120000", calendar)) return false;
    const int32_t day = Air10Clock::civil_epoch_day(parts[0], parts[1], parts[2]);
    if (day <= 0 || day >= UINT16_MAX) return false;
    out = static_cast<uint16_t>(day);
    return true;
}

bool fingerprint(const SdStorage::Session &session, IndexedSession &entry) {
    entry.key = {};
    return session.run([&](fs::FS &fs) {
        for (uint8_t i = 0; i < 5; ++i) {
            char path[96];
            source_path(entry, i, "edf", path);
            fs::File file = fs.open(path, FILE_READ);
            if (file && !file.isDirectory()) {
                entry.key.sizes[i] = file.size();
                entry.key.present |= 1u << i;
            }
            file.close();
            source_path(entry, i, "crc", path);
            file = fs.open(path, FILE_READ);
            if (file && file.size() == 8 &&
                file.read(entry.key.sidecars[i], 8) == 8) entry.key.checksums |= 1u << i;
            file.close();
        }
        return true;
    });
}

bool refresh_index(const SdStorage::Session &session) {
    index_failure = IndexFailure::Error;
    index_error = "catalog_status";
    index_error_entry = SIZE_MAX;
    EdfCatalog::Changes changes;
    bool incremental = false;
    if (!session.run([&](fs::FS &) {
        incremental = EdfCatalog::changes_since(catalog_generation, changes);
        return changes.catalog.ready;
    })) {
        if (!session.valid() || !AirSenseState::local_background_allowed()) index_failure = IndexFailure::Preempted;
        else if (!changes.catalog.ready) index_failure = IndexFailure::WaitingCatalog;
        return false;
    }
    const uint32_t files = SdStorage::files_revision();
    const uint32_t invalidation = __atomic_load_n(&invalidation_generation, __ATOMIC_ACQUIRE);
    const bool invalidated = invalidation != indexed_invalidation;
    if (index_ready && !invalidated && changes.catalog.generation == catalog_generation && files == file_revision)
        return true;
    const bool full = !index_ready || invalidated || !incremental || files != file_revision ||
        changes.catalog.entries < index_count;
    const size_t old_count = index_count;
    index_error = "session_limit";
    if (changes.catalog.entries > MAX_SESSION_COUNT) return false;
    if (changes.catalog.entries != index_count) {
        const size_t bytes = changes.catalog.entries * sizeof(IndexedSession);
        if (!bytes) {
            aircannect::Memory::free(index);
            index = nullptr;
        } else {
            void *grown = aircannect::Memory::realloc_large(index, bytes, bytes <= INTERNAL_WORK);
            if (!grown) { index_error = "index_alloc"; return false; }
            index = static_cast<IndexedSession *>(grown);
        }
        if (changes.catalog.entries > index_count)
            for (size_t i = index_count; i < changes.catalog.entries; ++i)
                new (&index[i]) IndexedSession;
        index_count = changes.catalog.entries;
    }
    bool detail_changed = !index_ready || changes.catalog.entries < old_count;
    const size_t count = full ? index_count : changes.count;
    for (size_t n = 0; n < count; ++n) {
        if (aborted(nullptr, session)) {
            index_failure = IndexFailure::Preempted;
            index_ready = false;
            return false;
        }
        const size_t i = full ? n : changes.indexes[n];
        index_error_entry = i;
        index_error = "catalog_index";
        if (i >= index_count) return false;
        IndexedSession next;
        index_error = "catalog_read";
        if (!session.run([&](fs::FS &) { return EdfCatalog::read(i, next.entry); })) return false;
        index_error = "catalog_day";
        if (!catalog_day(next.entry.therapy_day, next.day)) return false;
        // STR/Identification readiness is not a detail-data revision.
        const bool changed = strcmp(index[i].entry.file_prefix, next.entry.file_prefix) ||
            strcmp(index[i].entry.therapy_day, next.entry.therapy_day) ||
            ((index[i].entry.flags ^ next.entry.flags) & EdfCatalog::ENTRY_LIVE_COMPLETE);
        if (full || changed) {
            index_error = "source_fingerprint";
            if (!fingerprint(session, next)) { index_ready = false; return false; }
            const bool key_changed = !(index[i].key == next.key);
            index_error = "cache_retire";
            if (index[i].entry.file_prefix[0] && index[i].key.present && (changed || key_changed) &&
                !retire_cache(session, index[i])) { index_ready = false; return false; }
            detail_changed |= changed || key_changed;
            if (!changed && !key_changed) {
                next.start_ms = index[i].start_ms;
                next.end_ms = index[i].end_ms;
                next.event_from_ms = index[i].event_from_ms;
                next.event_to_ms = index[i].event_to_ms;
                next.bounds_known = index[i].bounds_known;
            }
        } else {
            next.key = index[i].key;
            next.start_ms = index[i].start_ms;
            next.end_ms = index[i].end_ms;
            next.event_from_ms = index[i].event_from_ms;
            next.event_to_ms = index[i].event_to_ms;
            next.bounds_known = index[i].bounds_known;
        }
        index[i] = next;
    }
    catalog_generation = changes.catalog.generation;
    file_revision = files;
    indexed_invalidation = invalidation;
    index_ready = true;
    if (full) {
        cache_usage_known = false;
        if (!prune_cache(session)) { index_ready = false; index_error = "cache_prune"; return false; }
    }
    if (detail_changed && !invalidated) __atomic_add_fetch(&data_revision, 1, __ATOMIC_RELEASE);
    portENTER_CRITICAL(&status_mux);
    status.catalog_generation = catalog_generation;
    status.files_revision = file_revision;
    status.sessions = index_count;
    status.data_revision = __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
    if (detail_changed || invalidated) ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    return true;
}

struct Input {
    SdStorage::Reader file;
    ReportEdf::Reader reader;
    ReportEdf::Header *header = nullptr;
    uint8_t *record = nullptr;
    size_t record_capacity = 0;
    uint64_t record_offset = UINT64_MAX;
    char path[96] = {};
    const SdStorage::Session &session;
    const Job &job;
    Input(const SdStorage::Session &session, const Job &job) : session(session), job(job) {
        void *memory = aircannect::Memory::alloc_large(sizeof(ReportEdf::Header),
                                                      sizeof(ReportEdf::Header) <= INTERNAL_WORK);
        if (memory) header = new (memory) ReportEdf::Header;
    }
    ~Input() {
        aircannect::Memory::free(header);
        aircannect::Memory::free(record);
    }
    bool open(const SdStorage::Session &session, const IndexedSession &entry, uint8_t kind) {
        if (!header) return note_issue(job, "header_alloc");
        record_offset = UINT64_MAX;
        *header = {};
        source_path(entry, kind, "edf", path);
        if (!file.open(session, path)) return note_issue(job, "source_open", path);
        reader = {[](void *ctx, uint64_t offset, uint8_t *out, size_t count) {
            auto &input = *static_cast<Input *>(ctx);
            if (aborted(&input.job, input.session)) return false;
            auto &file = input.file;
            const auto &header = *input.header;
            if (input.record && header.record_bytes && offset >= header.header_bytes &&
                offset + count <= header.header_bytes + uint64_t(header.records) * header.record_bytes) {
                const uint64_t base = header.header_bytes +
                    ((offset - header.header_bytes) / header.record_bytes) * header.record_bytes;
                if (offset + count <= base + header.record_bytes) {
                    if (base != input.record_offset) {
                        if (!file.seek(base)) return note_issue(input.job, "source_seek", input.path);
                        for (size_t at = 0; at < header.record_bytes;) {
                            const size_t take = std::min<size_t>(header.record_bytes - at,
                                                                SdStorage::READ_CHUNK_BYTES);
                            if (file.read(input.record + at, take) != take)
                                return note_issue(input.job, "source_read", input.path);
                            at += take;
                        }
                        input.record_offset = base;
                    }
                    memcpy(out, input.record + (offset - base), count);
                    return true;
                }
            }
            if (!file.seek(offset)) return note_issue(input.job, "source_seek", input.path);
            size_t done = 0;
            while (done < count) {
                const size_t take = std::min(count - done, SdStorage::READ_CHUNK_BYTES);
                if (file.read(out + done, take) != take)
                    return note_issue(input.job, "source_read", input.path);
                done += take;
            }
            return true;
        }, this, file.size()};
        if (!ReportEdf::read_header(reader, *header)) return note_issue(job, "source_header", path);
        if (header->record_bytes > record_capacity) {
            // One actual record, at most the reader's validated 64 KiB limit.
            void *grown = aircannect::Memory::realloc_large(record, header->record_bytes,
                                                           header->record_bytes <= INTERNAL_WORK);
            if (grown) {
                record = static_cast<uint8_t *>(grown);
                record_capacity = header->record_bytes;
            }
        }
        if (record_capacity < header->record_bytes) return note_issue(job, "record_alloc", path);
        return true;
    }
};

size_t total_bins(const CacheHeader &header) {
    size_t count = 0;
    for (const auto &channel : header.channels) count += channel.bins;
    return count;
}
bool allocate_data(SessionData &out) {
    const size_t count = total_bins(out.header);
    if (count > MAX_BINS || out.header.events > MAX_DISK_EVENTS) return false;
    if (count) {
        const size_t bytes = count * sizeof(uint64_t);
        out.bins = static_cast<uint64_t *>(aircannect::Memory::alloc_large(bytes, bytes <= INTERNAL_WORK));
        if (!out.bins) return false;
        memset(out.bins, 0, bytes);
    }
    if (out.header.events) {
        const size_t bytes = std::min<size_t>(out.header.events, MAX_EVENTS) * sizeof(Event);
        out.events = static_cast<Event *>(aircannect::Memory::alloc_large(bytes, bytes <= INTERNAL_WORK));
        if (!out.events) return false;
    }
    return true;
}

bool sane_header(const CacheHeader &h, const IndexedSession &entry, uint64_t bytes) {
    if (h.version != STATS_CACHE_VERSION || h.header_bytes != sizeof(h) ||
        memcmp(h.magic, "ABRPT01", 8) || !(h.key == entry.key) || h.day != entry.day ||
        h.events > MAX_DISK_EVENTS || total_bins(h) > MAX_BINS ||
        h.start_ms < 0 || h.end_ms < h.start_ms ||
        h.chart_start_ms < 0 || h.chart_end_ms < h.chart_start_ms ||
        h.event_from_ms < 0 || h.event_to_ms < h.event_from_ms ||
        !std::isfinite(h.used_seconds) || h.used_seconds < 0 || h.used_seconds > 86400 ||
        bytes != sizeof(h) + total_bins(h) * sizeof(uint64_t) + h.events * sizeof(Event) ||
        h.bytes != bytes) return false;
    for (const auto &channel : h.channels) {
        if (!channel.bins) continue;
        if (!memchr(channel.signal.label, 0, sizeof(channel.signal.label)) ||
            !memchr(channel.signal.unit, 0, sizeof(channel.signal.unit)) ||
            ReportEdf::histogram_bins(channel.signal) != channel.bins ||
            !std::isfinite(channel.sum) || !std::isfinite(channel.coverage_seconds) ||
            channel.coverage_seconds < 0 || channel.coverage_seconds > 86400)
            return false;
    }
    return true;
}

bool load_cache(const SdStorage::Session &session, const IndexedSession &entry,
                SessionData &out) {
    char path[96];
    cache_path(entry, "stats", path);
    SdStorage::Reader reader;
    if (!reader.open(session, path) || reader.read(reinterpret_cast<uint8_t *>(&out.header),
        sizeof(out.header)) != sizeof(out.header) || !sane_header(out.header, entry, reader.size()) ||
        !allocate_data(out)) return false;
    const size_t bin_bytes = total_bins(out.header) * sizeof(uint64_t);
    const size_t event_bytes = std::min<size_t>(out.header.events, MAX_EVENTS) * sizeof(Event);
    for (auto range : {std::make_pair(reinterpret_cast<uint8_t *>(out.bins), bin_bytes),
                       std::make_pair(reinterpret_cast<uint8_t *>(out.events), event_bytes)}) {
        for (size_t offset = 0; offset < range.second;) {
            const size_t take = std::min(range.second - offset, SdStorage::READ_CHUNK_BYTES);
            if (reader.read(range.first + offset, take) != take) return false;
            offset += take;
        }
    }
    return true;
}

bool start_part(const SdStorage::Session &, const char *, const uint8_t *, size_t);
bool append_part(const SdStorage::Session &, const char *, const uint8_t *, size_t);
bool finish_part(const SdStorage::Session &, const char *, const char *, uint64_t);

bool read_event(Input &input, uint32_t record, Event &event) {
    ReportEdf::Annotation annotation;
    if (!ReportEdf::read_annotation(input.reader, *input.header, record, annotation))
        return note_issue(input.job, "annotation_record", input.path);
    event.time_ms = llround((input.header->start_seconds + annotation.onset_seconds) * 1000);
    event.duration_ms = annotation.has_duration ? llround(annotation.duration_seconds * 1000) : 0;
    event.kind = annotation.kind;
    return true;
}

struct PartGuard {
    const SdStorage::Session &session;
    const char *path;
    bool complete = false;
    ~PartGuard() {
        if (complete) return;
        cache_usage_known = false;
        (void)session.run([&](fs::FS &fs) { return !fs.exists(path) || fs.remove(path); });
    }
};

bool store_cache(const SdStorage::Session &session, const Job &job, const IndexedSession &entry,
                 SessionData &data) {
    if (data.header.events > MAX_DISK_EVENTS) return false;
    char path[96], part[96];
    cache_path(entry, "stats", path);
    cache_path(entry, "stats.part", part);
    data.header.bytes = sizeof(data.header) + total_bins(data.header) * sizeof(uint64_t) +
        data.header.events * sizeof(Event);
    if (!prune_cache(session, data.header.bytes, path)) return false;
    PartGuard part_guard{session, part};
    if (!start_part(session, part, reinterpret_cast<const uint8_t *>(&data.header), sizeof(data.header)) ||
        !append_part(session, part, reinterpret_cast<const uint8_t *>(data.bins),
                     total_bins(data.header) * sizeof(uint64_t))) return false;
    if (data.header.events <= MAX_EVENTS) {
        if (!append_part(session, part, reinterpret_cast<const uint8_t *>(data.events),
                         data.header.events * sizeof(Event))) return false;
    } else {
        // Persist all events with a bounded block, not just the in-RAM prefix.
        Input input(session, job);
        Event block[32];
        uint32_t events = 0;
        size_t used = 0;
        for (uint8_t kind : {uint8_t(3), uint8_t(4)}) {
            if (!(entry.key.present & (1u << kind))) continue;
            if (!input.open(session, entry, kind)) return false;
            for (uint32_t i = 0; i < input.header->records; ++i) {
                Event event;
                if (!read_event(input, i, event)) return false;
                if (event.kind == ReportEdf::EventKind::Unknown) continue;
                block[used++] = event;
                ++events;
                if (used == 32) {
                    if (!append_part(session, part, reinterpret_cast<uint8_t *>(block), sizeof(block))) return false;
                    used = 0;
                }
            }
            input.file.close();
        }
        if (events != data.header.events || !append_part(session, part,
            reinterpret_cast<uint8_t *>(block), used * sizeof(Event))) return false;
    }
    part_guard.complete = finish_part(session, part, path, data.header.bytes);
    return part_guard.complete;
}

ReportEdf::Histogram histogram(const CachedChannel &channel, uint64_t *bins) {
    ReportEdf::Histogram out;
    out.signal = channel.signal;
    out.bins = bins;
    out.bin_count = channel.bins;
    out.count = channel.count;
    out.missing = channel.missing;
    out.sum = channel.sum;
    return out;
}

bool prepare_channels(Input &input, uint8_t file, SessionData &data) {
    for (size_t c = 0; c < CHANNEL_COUNT; ++c) {
        if (CHANNELS[c].file != file) continue;
        const int signal = ReportEdf::find_signal(*input.header, CHANNELS[c].signal);
        if (signal < 0) continue;
        auto &channel = data.header.channels[c];
        channel.signal = input.header->signals[signal];
        channel.bins = ReportEdf::histogram_bins(channel.signal);
        // Merge normalized physical scales while preserving exact digital bins.
        double scale = 1;
        if (c == 3 && !strcmp(channel.signal.unit, "L/s")) scale = 60;
        if (c == 6 && !strcmp(channel.signal.unit, "L")) scale = 1000;
        channel.signal.physical_min *= scale;
        channel.signal.physical_max *= scale;
        snprintf(channel.signal.unit, sizeof(channel.signal.unit), "%s", CHANNELS[c].unit);
        if (!channel.bins || total_bins(data.header) > MAX_BINS) return false;
    }
    return true;
}

bool scan_channels(const SdStorage::Session &session, const Job &job,
                   Input &input, uint8_t file, SessionData &data) {
    size_t offset = 0;
    ReportEdf::Sample samples[32];
    for (size_t c = 0; c < CHANNEL_COUNT; ++c) {
        auto &channel = data.header.channels[c];
        if (CHANNELS[c].file == file && channel.bins) {
            const int signal = ReportEdf::find_signal(*input.header, CHANNELS[c].signal);
            auto hist = histogram(channel, data.bins + offset);
            const uint64_t count = ReportEdf::sample_count(*input.header, signal);
            for (uint64_t first = 0; first < count;) {
                if (aborted(&job, session)) return false;
                const size_t take = std::min<uint64_t>(32, count - first);
                if (!ReportEdf::read_samples(input.reader, *input.header, signal,
                                             first, take, samples)) return false;
                for (size_t i = 0; i < take; ++i)
                    if (!ReportEdf::add_sample(hist, samples[i].raw, samples[i].valid)) return false;
                first += take;
            }
            channel.count = hist.count;
            channel.missing = hist.missing;
            channel.sum = hist.sum;
            channel.coverage_seconds = hist.count * input.header->record_seconds /
                channel.signal.samples_per_record;
        }
        offset += channel.bins;
    }
    return true;
}

int count_index(ReportEdf::EventKind kind) {
    switch (kind) {
        case ReportEdf::EventKind::OA: return 0;
        case ReportEdf::EventKind::CA: return 1;
        case ReportEdf::EventKind::UA: return 2;
        case ReportEdf::EventKind::H: return 3;
        case ReportEdf::EventKind::RERA: return 4;
        default: return -1;
    }
}
const char *event_name(ReportEdf::EventKind kind) {
    switch (kind) {
        case ReportEdf::EventKind::OA: return "oa";
        case ReportEdf::EventKind::CA: return "ca";
        case ReportEdf::EventKind::UA: return "ua";
        case ReportEdf::EventKind::H: return "h";
        case ReportEdf::EventKind::RERA: return "rera";
        case ReportEdf::EventKind::CsrStart: return "csr_start";
        case ReportEdf::EventKind::CsrEnd: return "csr_end";
        default: return "unknown";
    }
}

bool numeric_extent(const ReportEdf::Header &header, int64_t &start, int64_t &end) {
    start = header.start_seconds * 1000;
    const double duration = header.records * header.record_seconds;
    if (duration < 0 || duration > 86400 || start < 0) return false;
    end = start + llround(duration * 1000);
    return true;
}

bool build_session(const SdStorage::Session &session, const Job &job,
                   const IndexedSession &entry, SessionData &out) {
    out.header.key = entry.key;
    out.header.day = entry.day;
    out.header.partial = entry.key.present != 31 || entry.key.checksums != 31;
    Input input(session, job);
    uint8_t readable_numeric = 0;
    // BRP contributes its header extent only. Its samples and overview cache
    // remain lazy, independent of the PLD/SAD summary statistics.
    for (uint8_t kind : {uint8_t(0), uint8_t(1), uint8_t(2)}) {
        if (!(entry.key.present & (1u << kind))) continue;
        if (!input.open(session, entry, kind)) {
            if (aborted(&job, session)) return false;
            out.header.partial = true;
            input.file.close();
            continue;
        }
        if (kind && !prepare_channels(input, kind, out)) return false;
        readable_numeric |= 1u << kind;
        bool chart_present = !kind && ReportEdf::find_signal(*input.header, "Flow.40ms") >= 0;
        for (size_t c = 0; c < CHANNEL_COUNT; ++c)
            chart_present |= CHANNELS[c].file == kind && out.header.channels[c].bins;
        int64_t start = 0, end = 0;
        if (chart_present && numeric_extent(*input.header, start, end) && end > start) {
            if (!out.header.chart_end_ms) out.header.chart_start_ms = start;
            else out.header.chart_start_ms = std::min(out.header.chart_start_ms, start);
            out.header.chart_end_ms = std::max(out.header.chart_end_ms, end);
        }
        if (kind == 1) {
            out.header.usage_known = input.header->records == 0;
            int coverage = -1;
            for (const char *name : {"Press.2s", "EprPress.2s", "MaskPress.2s"}) {
                coverage = ReportEdf::find_signal(*input.header, name);
                if (coverage >= 0) break;
            }
            if (coverage >= 0) {
                ReportEdf::RecordedBounds bounds;
                if (!ReportEdf::recorded_bounds(input.reader, *input.header, coverage, bounds))
                    return false;
                out.header.recorded = bounds.present;
                out.header.usage_known |= bounds.present;
                if (bounds.present) {
                    out.header.start_ms = llround(bounds.begin_seconds * 1000);
                    out.header.end_ms = llround(bounds.end_seconds * 1000);
                    out.header.used_seconds = bounds.end_seconds - bounds.begin_seconds;
                }
            }
        }
        input.file.close();
    }
    // Cap event retention, but count every supported event independently.
    uint32_t event_capacity = 0;
    for (uint8_t kind : {uint8_t(3), uint8_t(4)}) {
        if (!(entry.key.present & (1u << kind))) continue;
        if (!input.open(session, entry, kind)) {
            if (aborted(&job, session)) return false;
            out.header.partial = true;
            input.file.close();
            continue;
        }
        if (kind == 3) out.header.events_known = true;
        event_capacity += std::min<uint32_t>(input.header->records, MAX_EVENTS - event_capacity);
        input.file.close();
    }
    out.header.events = event_capacity;
    if (!allocate_data(out)) return false;
    for (uint8_t kind : {uint8_t(1), uint8_t(2)}) {
        if (!(readable_numeric & (1u << kind))) continue;
        if (!input.open(session, entry, kind) || !scan_channels(session, job, input, kind, out))
            return false;
        input.file.close();
    }
    uint32_t events = 0;
    for (uint8_t kind : {uint8_t(3), uint8_t(4)}) {
        if (!(entry.key.present & (1u << kind))) continue;
        if (!input.open(session, entry, kind)) {
            if (aborted(&job, session)) return false;
            if (kind == 3) out.header.events_known = false;
            out.header.partial = true;
            input.file.close();
            continue;
        }
        for (uint32_t i = 0; i < input.header->records; ++i) {
            if (aborted(&job, session)) return false;
            Event event;
            if (!read_event(input, i, event)) {
                if (aborted(&job, session)) return false;
                if (kind == 3) out.header.events_known = false;
                out.header.partial = true;
                break;
            }
            if (event.kind == ReportEdf::EventKind::Unknown) continue;
            const int64_t begin = event.time_ms - event.duration_ms;
            if (!events) {
                out.header.event_from_ms = begin;
                out.header.event_to_ms = event.time_ms;
            } else {
                out.header.event_from_ms = std::min(out.header.event_from_ms, begin);
                out.header.event_to_ms = std::max(out.header.event_to_ms, event.time_ms);
            }
            const int count = count_index(event.kind);
            if (kind == 3 && count >= 0 && out.header.recorded &&
                event.time_ms >= out.header.start_ms && event.time_ms < out.header.end_ms)
                ++out.header.counts[count];
            if (events < event_capacity) out.events[events] = event;
            else out.header.events_truncated = true;
            ++events;
        }
        input.file.close();
    }
    out.header.events = events;
    if (events) std::sort(out.events, out.events + std::min<uint32_t>(events, event_capacity), [](const Event &a, const Event &b) {
        return a.time_ms < b.time_ms;
    });
    return true;
}

bool session_data(const SdStorage::Session &session, const Job &job,
                  const IndexedSession &entry, SessionData &out) {
    if (load_cache(session, entry, out)) {
        if (out.header.partial) note_issue(job, "partial_source", entry.entry.file_prefix);
        return true;
    }
    out.~SessionData();
    new (&out) SessionData;
    if (!build_session(session, job, entry, out))
        return note_issue(job, "session_build", entry.entry.file_prefix);
    if (out.header.partial) note_issue(job, "partial_source", entry.entry.file_prefix);
    // The cache is derived and replaceable; failed persistence does not discard
    // useful parsed data or modify any source EDF/sidecar.
    if (!store_cache(session, job, entry, out) && !aborted(&job, session))
        note_issue(job, "cache_write", entry.entry.file_prefix);
    return !aborted(&job, session);
}

uint16_t last_day(const Request &request) {
    if (request.day) return request.day;
    uint16_t day = 0;
    for (size_t i = 0; i < index_count; ++i)
        if ((index[i].entry.flags & EdfCatalog::ENTRY_LIVE_COMPLETE) &&
            index[i].key.present) day = std::max(day, index[i].day);
    return day;
}
uint16_t first_day(const Request &request, uint16_t last) {
    const uint16_t days = request.view == View::Day ? 1 : request.period;
    return last >= days ? last - days + 1 : 0;
}
bool selected(const IndexedSession &entry, uint16_t first, uint16_t last) {
    return entry.day >= first && entry.day <= last &&
        (entry.entry.flags & EdfCatalog::ENTRY_LIVE_COMPLETE);
}

void start_json(Payload &body, uint32_t publication, uint32_t dataset) {
    body.print("{\"source\":\"edf\",\"revision\":%u,\"data_revision\":%u,",
               publication, dataset);
}
void append_event(Payload &body, const Event &event, bool &comma, const char *type = nullptr) {
    body.print("%s{\"time_ms\":%lld,\"duration_ms\":%lld,\"type\":\"%s\"}",
               comma ? "," : "", static_cast<long long>(event.time_ms),
               static_cast<long long>(event.duration_ms), type ? type : event_name(event.kind));
    comma = true;
}

bool days_json(Job &job) {
    const size_t bytes = index_count * sizeof(uint16_t);
    auto *days = bytes ? static_cast<uint16_t *>(aircannect::Memory::alloc_large(bytes,
                                                       bytes <= INTERNAL_WORK)) : nullptr;
    if (bytes && !days) return false;
    size_t count = 0;
    bool partial = false;
    for (size_t i = 0; i < index_count; ++i) {
        if (!(index[i].entry.flags & EdfCatalog::ENTRY_LIVE_COMPLETE)) continue;
        if (index[i].key.present & 7) days[count++] = index[i].day;
        if (index[i].key.present != 31 || index[i].key.checksums != 31) partial = true;
    }
    if (count) std::sort(days, days + count, std::greater<uint16_t>());
    start_json(job.body, revision() + 1, job.data_revision);
    job.body.print("\"partial\":%s,\"days\":[", partial ? "true" : "false");
    for (size_t n = 0; n < count;) {
        const uint16_t day = days[n];
        size_t end = n + 1;
        while (end < count && days[end] == day) ++end;
        job.body.print("%s{\"day\":%u,\"session_count\":%u}", n ? "," : "", day,
                       static_cast<unsigned>(end - n));
        n = end;
    }
    job.body.print("]}");
    aircannect::Memory::free(days);
    return job.body.good();
}

struct Aggregate {
    ReportEdf::Histogram channels[CHANNEL_COUNT];
    double coverage[CHANNEL_COUNT] = {};
    bool incompatible[CHANNEL_COUNT] = {};
    ~Aggregate() {
        for (auto &channel : channels) aircannect::Memory::free(channel.bins);
    }
    bool merge(const SessionData &data, bool &partial) {
        size_t offset = 0;
        for (size_t c = 0; c < CHANNEL_COUNT; ++c) {
            const auto &cached = data.header.channels[c];
            if (cached.bins && !incompatible[c]) {
                auto next = histogram(cached, data.bins + offset);
                if (!channels[c].bins) {
                    const size_t bytes = cached.bins * sizeof(uint64_t);
                    auto *bins = static_cast<uint64_t *>(aircannect::Memory::alloc_large(bytes,
                                                                   bytes <= INTERNAL_WORK));
                    if (!bins || !ReportEdf::init_histogram(cached.signal, bins,
                                                           cached.bins, channels[c])) {
                        aircannect::Memory::free(bins);
                        return false;
                    }
                }
                if (!ReportEdf::merge_histogram(channels[c], next)) {
                    incompatible[c] = true;
                    partial = true;
                } else coverage[c] += cached.coverage_seconds;
            }
            offset += cached.bins;
        }
        return true;
    }
};

bool summary_json(const SdStorage::Session &session, Job &job) {
    Aggregate aggregate;
    Payload events;
    const uint16_t last = last_day(job.request), first = first_day(job.request, last);
    bool partial = false, event_comma = false, session_comma = false, events_truncated = false;
    bool usage_known = true, events_known = true;
    uint32_t sessions = 0;
    uint64_t counts[5] = {};
    double seconds = 0;
    start_json(job.body, revision() + 1, job.data_revision);
    job.body.print("\"view\":\"%s\",\"day\":%u,\"first_day\":%u,\"last_day\":%u,\"sessions\":[",
                   job.request.view == View::Day ? "day" : "period", last, first, last);
    for (size_t i = 0; i < index_count; ++i) {
        auto &entry = index[i];
        if (!selected(entry, first, last)) continue;
        if (aborted(&job, session)) return false;
        if (!(entry.key.present & 7)) {
            note_issue(job, "numeric_files_missing", entry.entry.file_prefix);
            partial = true;
            usage_known = events_known = false;
            continue;
        }
        SessionData data;
        if (!session_data(session, job, entry, data)) {
            if (aborted(&job, session)) return false;
            partial = true;
            usage_known = events_known = false;
            continue;
        }
        partial |= data.header.partial;
        usage_known &= data.header.usage_known;
        events_known &= data.header.usage_known;
        entry.bounds_known = data.header.chart_end_ms > data.header.chart_start_ms;
        entry.start_ms = data.header.chart_start_ms;
        entry.end_ms = data.header.chart_end_ms;
        entry.event_from_ms = data.header.event_from_ms;
        entry.event_to_ms = data.header.event_to_ms;
        events_truncated |= data.header.events_truncated;
        if (!aggregate.merge(data, partial)) return false;
        if (data.header.recorded) {
            seconds += data.header.used_seconds;
            events_known &= data.header.events_known;
            for (size_t c = 0; c < 5; ++c) counts[c] += data.header.counts[c];
        }
        job.body.print("%s{\"id\":%u,\"day\":%u,\"prefix\":\"%s\",\"start_ms\":",
                       session_comma ? "," : "", static_cast<unsigned>(i), entry.day, entry.entry.file_prefix);
        if (entry.bounds_known)
            job.body.print("%lld,\"end_ms\":%lld}", static_cast<long long>(entry.start_ms),
                           static_cast<long long>(entry.end_ms));
        else job.body.print("null,\"end_ms\":null}");
        session_comma = true;
        ++sessions;
        for (uint32_t e = 0; e < std::min<size_t>(data.header.events, MAX_EVENTS); ++e) {
            // Keep the response bounded even for year periods, without losing
            // event counts used by the independently accumulated AHI.
            if (events.length() > MAX_BODY / 2 - 256) { events_truncated = true; break; }
            append_event(events, data.events[e], event_comma);
        }
    }
    job.body.print("],\"present\":%s,\"partial\":%s,\"used_seconds\":",
                   sessions ? "true" : "false", partial ? "true" : "false");
    if (usage_known && seconds > 0) job.body.print("%.10g", seconds);
    else job.body.print("null");
    job.body.print(",\"session_count\":%u,\"ahi\":", sessions);
    if (seconds > 0 && events_known) job.body.print("%.10g", (counts[0] + counts[1] + counts[2] + counts[3]) * 3600.0 / seconds);
    else job.body.print("null");
    job.body.print(",\"events_known\":%s,", events_known && seconds > 0 ? "true" : "false");
    if (events_known && seconds > 0) job.body.print("\"counts\":{\"oa\":%llu,\"ca\":%llu,\"ua\":%llu,\"h\":%llu,\"rera\":%llu},",
                   static_cast<unsigned long long>(counts[0]), static_cast<unsigned long long>(counts[1]),
                   static_cast<unsigned long long>(counts[2]), static_cast<unsigned long long>(counts[3]),
                   static_cast<unsigned long long>(counts[4]));
    else job.body.print("\"counts\":{\"oa\":null,\"ca\":null,\"ua\":null,\"h\":null,\"rera\":null},");
    job.body.print("\"events_truncated\":%s,\"events\":[", events_truncated ? "true" : "false");
    if (events.length()) job.body.append(events.c_str(), events.length());
    job.body.print("],\"channels\":[");
    for (size_t c = 0; c < CHANNEL_COUNT; ++c) {
        ReportEdf::Statistics stats;
        const bool present = !aggregate.incompatible[c] && aggregate.channels[c].bins &&
            ReportEdf::summarize(aggregate.channels[c], stats) && stats.present;
        job.body.print("%s{\"id\":\"%s\",\"label\":\"%s\",\"unit\":\"%s\",\"count\":%llu,",
                       c ? "," : "", CHANNELS[c].id, CHANNELS[c].label, CHANNELS[c].unit,
                       static_cast<unsigned long long>(present ? stats.count : 0));
        job.body.print("\"coverage_seconds\":%.10g,", present ? aggregate.coverage[c] : 0);
        if (present) job.body.print("\"mean\":%.10g,\"median\":%.10g,\"p95\":%.10g,\"max\":%.10g}",
                                   stats.mean, stats.median, stats.p95, stats.max);
        else job.body.print("\"mean\":null,\"median\":null,\"p95\":null,\"max\":null}");
    }
    job.body.print("]}");
    return job.body.good() && events.good();
}

struct Cell {
    float low = INFINITY, high = -INFINITY;
    uint32_t covered_ms = 0;
};
struct ReducedLevel {
    int64_t start_ms = 0;
    uint32_t count = 0;
    uint64_t offset = 0;
};
struct ReducedChannel {
    int64_t end_ms = 0;
    ReducedLevel levels[LEVEL_COUNT];
};
struct ReducedHeader {
    char magic[8] = {'A','B','S','E','R','0','1',0};
    uint32_t version = SERIES_CACHE_VERSION, header_bytes = sizeof(ReducedHeader);
    uint64_t bytes = 0;
    SourceKey key;
    ReducedChannel channels[CHANNEL_COUNT + 1];
};

uint8_t chart_file(size_t c) { return c ? CHANNELS[c - 1].file : 0; }
const char *chart_signal(size_t c) { return c ? CHANNELS[c - 1].signal : "Flow.40ms"; }
double chart_scale(size_t c, const ReportEdf::Signal &signal) {
    if ((c == 0 || c == 4) && !strcmp(signal.unit, "L/s")) return 60;
    if (c == 7 && !strcmp(signal.unit, "L")) return 1000;
    return 1;
}

bool start_part(const SdStorage::Session &session, const char *path,
                const uint8_t *header, size_t bytes) {
    return session.run([&](fs::FS &fs) {
        if ((!fs.exists("/airbridge") && !fs.mkdir("/airbridge")) ||
            (!fs.exists(CACHE_DIR) && !fs.mkdir(CACHE_DIR))) return false;
        if (fs.exists(path) && !fs.remove(path)) return false;
        fs::File file = fs.open(path, FILE_WRITE);
        const bool success = file && SdStorage::write_exact(file, header, bytes);
        file.close();
        return success;
    });
}
bool append_part(const SdStorage::Session &session, const char *path,
                 const uint8_t *data, size_t bytes) {
    for (size_t at = 0; at < bytes;) {
        const size_t take = std::min(bytes - at, SdStorage::READ_CHUNK_BYTES);
        if (!session.run([&](fs::FS &fs) {
            fs::File file = fs.open(path, FILE_APPEND);
            const bool success = file && SdStorage::write_exact(file, data + at, take);
            file.close();
            return success;
        })) return false;
        at += take;
    }
    return true;
}
bool finish_part(const SdStorage::Session &session, const char *part,
                 const char *path, uint64_t bytes) {
    uint64_t old_bytes = 0;
    const bool success = session.run([&](fs::FS &fs) {
        fs::File file = fs.open(part, FILE_APPEND);
        if (!file || file.size() != bytes) return false;
        file.flush();
        file.close();
        fs::File old = fs.open(path, FILE_READ);
        if (old) old_bytes = old.size();
        old.close();
        return (!fs.exists(path) || fs.remove(path)) && fs.rename(part, path);
    });
    if (success && cache_usage_known) cache_bytes = cache_bytes - std::min(cache_bytes, old_bytes) + bytes;
    else cache_usage_known = false;
    return success;
}

bool cache_name(const char *input, char (&path)[96], const char *&base) {
    base = strrchr(input, '/');
    base = base ? base + 1 : input;
    if (strlen(base) < 21 || strlen(base) > 32) return false;
    for (size_t n = 0; n < 15; ++n)
        if (n == 8 ? base[n] != '_' : base[n] < '0' || base[n] > '9') return false;
    if (strcmp(base + 15, ".stats") && strcmp(base + 15, ".series") &&
        strcmp(base + 15, ".stats.part") && strcmp(base + 15, ".series.part")) return false;
    snprintf(path, sizeof(path), "%s/%s", CACHE_DIR, base);
    return true;
}

bool prune_cache(const SdStorage::Session &session, uint64_t needed, const char *keep) {
    if (needed > MAX_SD_CACHE) return false;
    if (cache_usage_known && cache_bytes + needed <= MAX_SD_CACHE) return true;
    while (true) {
        SdStorage::Reader directory;
        if (!directory.open(session, CACHE_DIR, true)) {
            bool missing = false;
            if (!session.run([&](fs::FS &fs) { missing = !fs.exists(CACHE_DIR); return true; })) return false;
            if (missing) { cache_bytes = 0; cache_usage_known = true; return true; }
            return false;
        }
        uint64_t total = 0, oldest_bytes = 0;
        char oldest[96] = {};
        bool end = false;
        SdStorage::Reader::Entry file;
        while (!end) {
            if (!directory.next(file, end)) return false;
            if (end || file.directory) continue;
            char path[96];
            const char *base = nullptr;
            if (!cache_name(file.name, path, base)) continue;
            bool live = false;
            for (size_t i = 0; i < index_count; ++i)
                if (index[i].key.present && !strncmp(base, index[i].entry.file_prefix, 15)) {
                    live = true;
                    break;
                }
            if (!live || strstr(base, ".part")) {
                if (!session.run([&](fs::FS &fs) { return fs.remove(path); })) return false;
                continue;
            }
            total += file.size;
            if ((!keep || strcmp(path, keep)) && (!oldest[0] || strcmp(path, oldest) < 0)) {
                snprintf(oldest, sizeof(oldest), "%s", path);
                oldest_bytes = file.size;
            }
        }
        directory.close();
        cache_bytes = total;
        cache_usage_known = true;
        if (total + needed <= MAX_SD_CACHE) return true;
        if (!oldest[0] || !session.run([&](fs::FS &fs) { return fs.remove(oldest); })) return false;
        cache_bytes -= oldest_bytes;
        // Retire oldest cache entries only. Missing cache is rebuilt from EDF.
        if (cache_bytes + needed <= MAX_SD_CACHE) return true;
    }
}

bool reduced_header(const SdStorage::Session &session, const IndexedSession &entry,
                    ReducedHeader &out) {
    char path[96];
    cache_path(entry, "series", path);
    SdStorage::Reader reader;
    if (!reader.open(session, path) || reader.read(reinterpret_cast<uint8_t *>(&out), sizeof(out)) != sizeof(out) ||
        memcmp(out.magic, "ABSER01", 8) || out.version != SERIES_CACHE_VERSION ||
        out.header_bytes != sizeof(out) || !(out.key == entry.key) || out.bytes != reader.size()) return false;
    uint64_t offset = sizeof(out);
    for (const auto &channel : out.channels) {
        if (channel.end_ms < 0) return false;
        for (size_t n = 0; n < LEVEL_COUNT; ++n) {
            const auto &level = channel.levels[n];
            if (level.offset != offset || level.count > uint64_t(86400000 / LEVEL_MS[n] + 2) ||
                level.start_ms < 0 || level.start_ms % LEVEL_MS[n]) return false;
            offset += uint64_t(level.count) * sizeof(Cell);
        }
    }
    return offset == out.bytes;
}

void cell_sample(Cell *cells, uint32_t count, int64_t start_ms, int64_t step_ms,
                 int64_t begin, int64_t end, double value, bool valid) {
    if (!valid || end <= start_ms) return;
    int64_t cell = std::max<int64_t>(0, (begin - start_ms) / step_ms);
    while (cell < count && start_ms + cell * step_ms < end) {
        const int64_t left = std::max(begin, start_ms + cell * step_ms);
        const int64_t right = std::min(end, start_ms + (cell + 1) * step_ms);
        if (right > left) {
            cells[cell].low = std::min(cells[cell].low, static_cast<float>(value));
            cells[cell].high = std::max(cells[cell].high, static_cast<float>(value));
            cells[cell].covered_ms += right - left;
        }
        ++cell;
    }
}

bool build_reduced(const SdStorage::Session &session, const Job &job,
                   const IndexedSession &entry, ReducedHeader &out) {
    out = {};
    out.key = entry.key;
    Input input(session, job);
    for (uint8_t kind = 0; kind < 3; ++kind) {
        if (!(entry.key.present & (1u << kind))) continue;
        if (!input.open(session, entry, kind)) return false;
        int64_t start = 0, end = 0;
        if (!numeric_extent(*input.header, start, end)) return false;
        for (size_t c = 0; c <= CHANNEL_COUNT; ++c) {
            if (chart_file(c) != kind || ReportEdf::find_signal(*input.header, chart_signal(c)) < 0)
                continue;
            auto &channel = out.channels[c];
            channel.end_ms = end;
            for (size_t n = 0; n < LEVEL_COUNT; ++n) {
                auto &level = channel.levels[n];
                level.start_ms = start / LEVEL_MS[n] * LEVEL_MS[n];
                level.count = end > start ? (end - level.start_ms + LEVEL_MS[n] - 1) / LEVEL_MS[n] : 0;
            }
        }
        input.file.close();
    }
    uint64_t offset = sizeof(out);
    for (auto &channel : out.channels) {
        for (auto &level : channel.levels) {
            level.offset = offset;
            offset += uint64_t(level.count) * sizeof(Cell);
        }
    }
    out.bytes = offset;
    char path[96], part[96];
    cache_path(entry, "series", path);
    cache_path(entry, "series.part", part);
    if (!prune_cache(session, out.bytes, path)) return false;
    PartGuard part_guard{session, part};
    if (!start_part(session, part, reinterpret_cast<const uint8_t *>(&out), sizeof(out))) return false;
    for (size_t c = 0; c <= CHANNEL_COUNT; ++c) {
        const auto &channel = out.channels[c];
        size_t count = 0;
        for (const auto &level : channel.levels) count += level.count;
        const size_t bytes = count * sizeof(Cell);
        if (!count) continue;
        auto *cells = static_cast<Cell *>(aircannect::Memory::alloc_large(bytes, bytes <= INTERNAL_WORK));
        if (!cells) return false;
        for (size_t i = 0; i < count; ++i) new (&cells[i]) Cell;
        bool success = input.open(session, entry, chart_file(c));
        const int signal = success ? ReportEdf::find_signal(*input.header, chart_signal(c)) : -1;
        success &= signal >= 0;
        if (success) {
            const auto &spec = input.header->signals[signal];
            const double step = input.header->record_seconds * 1000 / spec.samples_per_record;
            const int64_t origin = input.header->start_seconds * 1000;
            const uint64_t samples = ReportEdf::sample_count(*input.header, signal);
            const double scale = chart_scale(c, spec);
            ReportEdf::Sample values[32];
            for (uint64_t first = 0; success && first < samples;) {
                const size_t take = std::min<uint64_t>(32, samples - first);
                success = !aborted(&job, session) && ReportEdf::read_samples(input.reader,
                    *input.header, signal, first, take, values);
                for (size_t i = 0; success && i < take; ++i) {
                    const int64_t begin = origin + llround((first + i) * step);
                    const int64_t end = origin + llround((first + i + 1) * step);
                    const double value = values[i].value * scale;
                    size_t cell_offset = 0;
                    for (size_t n = 0; n < LEVEL_COUNT; ++n) {
                        const auto &level = channel.levels[n];
                        cell_sample(cells + cell_offset, level.count, level.start_ms, LEVEL_MS[n],
                                    begin, end, value, values[i].valid);
                        cell_offset += level.count;
                    }
                }
                first += take;
            }
        }
        input.file.close();
        success = success && append_part(session, part, reinterpret_cast<uint8_t *>(cells), bytes);
        aircannect::Memory::free(cells);
        if (!success) return false;
    }
    part_guard.complete = finish_part(session, part, path, out.bytes);
    return part_guard.complete;
}

struct Bucket {
    float low = INFINITY, high = -INFINITY;
    uint64_t covered_ms = 0;
};
struct Series {
    Bucket *buckets = nullptr;
    uint16_t px = 0;
    int64_t from_ms = 0, to_ms = 0, step_ms = 0, level_ms = 0;
    size_t level_index = 0;
    bool partial = false;
    explicit Series(const Request &request) {
        from_ms = request.from_ms;
        to_ms = request.to_ms;
        step_ms = (to_ms - from_ms + request.px - 1) / request.px;
        if (step_ms >= 10000) {
            for (size_t n = 0; n < LEVEL_COUNT; ++n)
                if (LEVEL_MS[n] <= step_ms) level_index = n;
            level_ms = LEVEL_MS[level_index];
            from_ms = from_ms / level_ms * level_ms;
            step_ms = ((request.to_ms - from_ms + request.px - 1) / request.px + level_ms - 1) /
                level_ms * level_ms;
            px = (request.to_ms - from_ms + step_ms - 1) / step_ms;
            to_ms = from_ms + px * step_ms;
        } else px = (to_ms - from_ms + step_ms - 1) / step_ms;
        const size_t count = px * (CHANNEL_COUNT + 1), bytes = count * sizeof(Bucket);
        buckets = static_cast<Bucket *>(aircannect::Memory::alloc_large(bytes, bytes <= INTERNAL_WORK));
        if (buckets) for (size_t i = 0; i < count; ++i) new (&buckets[i]) Bucket;
    }
    ~Series() { aircannect::Memory::free(buckets); }
    void add(size_t c, int64_t begin, int64_t end, float low, float high, uint64_t covered) {
        if (!covered || end <= from_ms || begin >= to_ms) return;
        const int64_t cell = std::max<int64_t>(0, (begin - from_ms) / step_ms);
        if (cell >= px) return;
        Bucket &bucket = buckets[c * px + cell];
        bucket.low = std::min(bucket.low, low);
        bucket.high = std::max(bucket.high, high);
        bucket.covered_ms += covered;
    }
};

void series_channel_json(Payload &out, const Series &series, size_t channel) {
    auto format_bucket = [&](size_t at, char *text) -> size_t {
        const auto &bucket = series.buckets[channel * series.px + at];
        if (!bucket.covered_ms || !std::isfinite(bucket.low) || !std::isfinite(bucket.high))
            return 0;
        const uint64_t expected = std::min<int64_t>(series.step_ms, series.to_ms -
            (series.from_ms + int64_t(at) * series.step_ms));
        return snprintf(text, 64, "%.7g,%.7g,%llu", bucket.low, bucket.high,
                        static_cast<unsigned long long>(std::min(bucket.covered_ms, expected)));
    };
    auto visit_runs = [&](auto &&visit) {
        char value[64], next[64];
        const auto *buckets = series.buckets + channel * series.px;
        size_t begin = 0, length = format_bucket(0, value);
        for (size_t at = 1; at <= series.px; ++at) {
            if (at < series.px) {
                const auto &bucket = buckets[at], &previous = buckets[at - 1];
                const uint64_t expected = std::min<int64_t>(series.step_ms, series.to_ms -
                    (series.from_ms + int64_t(at) * series.step_ms));
                // The preceding bucket cannot be the clipped tail. Bitwise
                // float equality also distinguishes +0 from -0.
                if (std::min(bucket.covered_ms, expected) ==
                    std::min(previous.covered_ms, static_cast<uint64_t>(series.step_ms)) &&
                    !memcmp(&bucket.low, &previous.low, sizeof(bucket.low)) &&
                    !memcmp(&bucket.high, &previous.high, sizeof(bucket.high)))
                    continue;
            }
            const size_t next_length = at < series.px ? format_bucket(at, next) : 0;
            // Compare wire values, including tail-clipped coverage and the
            // existing seven-significant-digit representation, not raw floats.
            if (at < series.px && length == next_length && !memcmp(value, next, length))
                continue;
            visit(at - begin, value, length);
            begin = at;
            length = next_length;
            if (length) memcpy(value, next, length + 1);
        }
    };

    size_t dense_bytes = sizeof("\"buckets\":[]") - 1 + series.px - 1;
    size_t run_bytes = sizeof("\"runs\":[]") - 1, run_count = 0;
    visit_runs([&](size_t count, const char *, size_t length) {
        dense_bytes += count * (length ? length + 2 : 4);
        run_bytes += snprintf(nullptr, 0, "%zu", count) + 2 +
            (length ? length + 1 : 0) + (run_count++ ? 1 : 0);
    });
    const bool compact = run_bytes < dense_bytes;
    out.print("\"%s\":[", compact ? "runs" : "buckets");
    bool comma = false;
    visit_runs([&](size_t count, const char *value, size_t length) {
        if (compact) {
            if (length) out.print("%s[%zu,%s]", comma ? "," : "", count, value);
            else out.print("%s[%zu]", comma ? "," : "", count);
            comma = true;
        } else {
            for (size_t i = 0; i < count; ++i) {
                if (length) out.print("%s[%s]", comma ? "," : "", value);
                else out.print("%snull", comma ? "," : "");
                comma = true;
            }
        }
    });
    out.print("]");
}

bool cached_series(const SdStorage::Session &session, const Job &job,
                   const IndexedSession &entry, Series &series) {
    ReducedHeader header;
    if (!reduced_header(session, entry, header) && !build_reduced(session, job, entry, header))
        return note_issue(job, "series_cache_build", entry.entry.file_prefix);
    char path[96];
    cache_path(entry, "series", path);
    SdStorage::Reader reader;
    if (!reader.open(session, path)) return false;
    Cell cells[64];
    for (size_t c = 0; c <= CHANNEL_COUNT; ++c) {
        const auto &channel = header.channels[c];
        const auto &level = channel.levels[series.level_index];
        const uint32_t count = level.count;
        const int64_t start = level.start_ms;
        const uint64_t offset = level.offset;
        if (!count || channel.end_ms <= series.from_ms || start >= series.to_ms) continue;
        const uint32_t first = std::max<int64_t>(0, (series.from_ms - start) / series.level_ms);
        const uint32_t last = std::min<int64_t>(count, (series.to_ms - start) / series.level_ms);
        if (!reader.seek(offset + first * sizeof(Cell))) return false;
        for (uint32_t n = first; n < last;) {
            const size_t take = std::min<uint32_t>(64, last - n);
            if (aborted(&job, session) || reader.read(reinterpret_cast<uint8_t *>(cells),
                take * sizeof(Cell)) != take * sizeof(Cell)) return false;
            for (size_t i = 0; i < take; ++i) {
                const int64_t begin = start + (n + i) * series.level_ms;
                if (cells[i].covered_ms > series.level_ms ||
                    (cells[i].covered_ms && (!std::isfinite(cells[i].low) ||
                     !std::isfinite(cells[i].high) || cells[i].low > cells[i].high))) return false;
                series.add(c, begin, begin + series.level_ms, cells[i].low,
                           cells[i].high, cells[i].covered_ms);
            }
            n += take;
        }
    }
    return true;
}

bool window_series(const SdStorage::Session &session, const Job &job,
                   const IndexedSession &entry, Series &series) {
    Input input(session, job);
    ReportEdf::Sample values[32];
    for (size_t c = 0; c <= CHANNEL_COUNT; ++c) {
        if (!(entry.key.present & (1u << chart_file(c)))) { series.partial = true; continue; }
        if (!input.open(session, entry, chart_file(c))) return false;
        const int signal = ReportEdf::find_signal(*input.header, chart_signal(c));
        if (signal < 0) { input.file.close(); continue; }
        const auto &spec = input.header->signals[signal];
        const double step = input.header->record_seconds * 1000 / spec.samples_per_record;
        const int64_t origin = input.header->start_seconds * 1000;
        const uint64_t count = ReportEdf::sample_count(*input.header, signal);
        const uint64_t first = std::min<uint64_t>(count,
            std::max<double>(0, std::floor((series.from_ms - origin) / step)));
        const uint64_t last = std::min<uint64_t>(count,
            std::max<double>(0, std::ceil((series.to_ms - origin) / step)));
        const double scale = chart_scale(c, spec);
        for (uint64_t n = first; n < last;) {
            const size_t take = std::min<uint64_t>(32, last - n);
            if (aborted(&job, session) || !ReportEdf::read_samples(input.reader, *input.header,
                signal, n, take, values)) return false;
            for (size_t i = 0; i < take; ++i) {
                if (!values[i].valid) continue;
                int64_t begin = std::max<int64_t>(series.from_ms, origin + llround((n + i) * step));
                const int64_t end = std::min<int64_t>(series.to_ms, origin + llround((n + i + 1) * step));
                while (begin < end) {
                    const int64_t cell = (begin - series.from_ms) / series.step_ms;
                    const int64_t right = std::min(end, series.from_ms + (cell + 1) * series.step_ms);
                    const float value = values[i].value * scale;
                    series.add(c, begin, right, value, value, right - begin);
                    begin = right;
                }
            }
            n += take;
        }
        input.file.close();
    }
    return true;
}

bool cached_summary_header(const SdStorage::Session &session, const IndexedSession &entry,
                           CacheHeader &header) {
    char path[96];
    cache_path(entry, "stats", path);
    SdStorage::Reader reader;
    return reader.open(session, path) && reader.read(reinterpret_cast<uint8_t *>(&header),
        sizeof(header)) == sizeof(header) && sane_header(header, entry, reader.size());
}
bool build_summary_header(const SdStorage::Session &session, const Job &job,
                          const IndexedSession &entry, CacheHeader &header) {
    SessionData data;
    if (!session_data(session, job, entry, data)) return false;
    header = data.header;
    return true;
}

bool window_events(const SdStorage::Session &session, const Job &job,
                   const IndexedSession &entry, const CacheHeader &header,
                   int64_t from, int64_t to, Payload &out, bool &comma, bool &truncated) {
    if (truncated) return true;
    auto append = [&](const Event &event, const char *type = nullptr) {
        const bool intersects = type ?
            event.time_ms > from && event.time_ms - event.duration_ms < to :
            event.duration_ms > 0 ?
            event.time_ms >= from && event.time_ms - event.duration_ms < to :
            event.time_ms >= from && event.time_ms < to;
        if (event.kind == ReportEdf::EventKind::Unknown || !intersects)
            return;
        if (out.length() > MAX_BODY / 4 - 256) { truncated = true; return; }
        append_event(out, event, comma, type);
    };
    // Pair before viewport filtering, retaining only one pending start per
    // session. A repeated start leaves the preceding boundary unmatched.
    Event csr_start;
    bool csr_active = false;
    auto consume = [&](const Event &event) {
        if (event.kind == ReportEdf::EventKind::CsrStart) {
            if (csr_active) append(csr_start);
            csr_start = event;
            csr_active = true;
        } else if (event.kind == ReportEdf::EventKind::CsrEnd) {
            if (csr_active && event.time_ms > csr_start.time_ms) {
                Event interval = event;
                interval.duration_ms = event.time_ms - csr_start.time_ms;
                append(interval, "csr");
            } else {
                if (csr_active) append(csr_start);
                append(event);
            }
            csr_active = false;
        } else {
            append(event);
        }
    };
    char path[96];
    cache_path(entry, "stats", path);
    SdStorage::Reader reader;
    if (reader.open(session, path) && reader.size() == header.bytes &&
        reader.seek(sizeof(header) + total_bins(header) * sizeof(uint64_t))) {
        Event block[32];
        for (uint32_t n = 0; n < header.events;) {
            const size_t take = std::min<uint32_t>(32, header.events - n);
            if (aborted(&job, session) || reader.read(reinterpret_cast<uint8_t *>(block),
                take * sizeof(Event)) != take * sizeof(Event)) return false;
            for (size_t i = 0; i < take; ++i) consume(block[i]);
            if (truncated) return out.good();
            n += take;
        }
        if (csr_active) append(csr_start);
        return out.good();
    }
    reader.close();
    // Failed derived-cache persistence is not permission to truncate a late
    // viewport. Read the small annotation sources, never unrelated histograms.
    Input input(session, job);
    for (uint8_t kind : {uint8_t(3), uint8_t(4)}) {
        if (!(entry.key.present & (1u << kind))) continue;
        if (!input.open(session, entry, kind)) return false;
        for (uint32_t i = 0; i < input.header->records; ++i) {
            Event event;
            if (!read_event(input, i, event)) return false;
            consume(event);
            if (truncated) return out.good();
        }
        input.file.close();
    }
    if (csr_active) append(csr_start);
    return out.good();
}

bool series_json(const SdStorage::Session &session, Job &job, JobTiming &timing) {
    uint32_t started = timing.stamp();
    Series series(job.request);
    timing.numeric_us += timing.stamp() - started;
    if (!series.buckets) return false;
    Payload events;
    bool event_comma = false, events_truncated = false;
    const uint16_t last = last_day(job.request), first = first_day(job.request, last);
    for (size_t i = 0; i < index_count; ++i) {
        auto &entry = index[i];
        if (job.request.excludes(i) || !selected(entry, first, last)) continue;
        const int64_t noon = int64_t(entry.day) * 86400000 + 43200000;
        if (noon + 86400000 <= series.from_ms || noon >= series.to_ms ||
            (entry.bounds_known && (entry.end_ms <= series.from_ms || entry.start_ms >= series.to_ms) &&
             (entry.event_to_ms < series.from_ms || entry.event_from_ms >= series.to_ms))) continue;
        if (aborted(&job, session)) return false;
        if (!(entry.key.present & 7)) { series.partial = true; continue; }
        started = timing.stamp();
        CacheHeader header;
        if (!cached_summary_header(session, entry, header) && !build_summary_header(session, job, entry, header)) {
            timing.numeric_us += timing.stamp() - started;
            if (aborted(&job, session)) return false;
            series.partial = true;
            continue;
        }
        series.partial |= header.partial;
        entry.bounds_known = header.chart_end_ms > header.chart_start_ms;
        entry.start_ms = header.chart_start_ms;
        entry.end_ms = header.chart_end_ms;
        entry.event_from_ms = header.event_from_ms;
        entry.event_to_ms = header.event_to_ms;
        // Non-overlapping sessions need neither BRP cache construction nor a
        // source window. Their summaries remain reusable for later requests.
        const bool numeric_overlap = !entry.bounds_known ||
            (entry.end_ms > series.from_ms && entry.start_ms < series.to_ms);
        const bool success = !numeric_overlap || (series.level_ms ? cached_series(session, job, entry, series)
                                                                  : window_series(session, job, entry, series));
        timing.numeric_us += timing.stamp() - started;
        if (!success) {
            if (aborted(&job, session)) return false;
            series.partial = true;
        }
        started = timing.stamp();
        if (!window_events(session, job, entry, header, series.from_ms, series.to_ms,
                           events, event_comma, events_truncated)) series.partial = true;
        timing.events_us += timing.stamp() - started;
    }
    started = timing.stamp();
    start_json(job.body, revision() + 1, job.data_revision);
    job.body.print("\"excluded\":\"");
    size_t mask_bytes = sizeof(job.request.excluded);
    while (mask_bytes && !job.request.excluded[mask_bytes - 1]) --mask_bytes;
    for (size_t i = 0; i < mask_bytes; ++i) {
        const char pair[] = {
            aircannect::hex_digit(job.request.excluded[i] >> 4, aircannect::HexCase::Lower),
            aircannect::hex_digit(job.request.excluded[i], aircannect::HexCase::Lower)};
        job.body.append(pair, sizeof(pair));
    }
    job.body.print("\",\"day\":%u,\"from_ms\":%lld,\"to_ms\":%lld,\"bucket_ms\":%lld,\"px\":%u,",
                   last, static_cast<long long>(series.from_ms), static_cast<long long>(series.to_ms),
                   static_cast<long long>(series.step_ms), series.px);
    job.body.print("\"requested_from_ms\":%lld,\"requested_to_ms\":%lld,\"partial\":%s,\"channels\":[",
                   static_cast<long long>(job.request.from_ms), static_cast<long long>(job.request.to_ms),
                   series.partial ? "true" : "false");
    for (size_t c = 0; c <= CHANNEL_COUNT; ++c) {
        job.body.print("%s{\"id\":\"%s\",\"label\":\"%s\",\"unit\":\"%s\",", c ? "," : "",
                       c ? CHANNELS[c - 1].id : "Flow", c ? CHANNELS[c - 1].label : "Flow",
                       c ? CHANNELS[c - 1].unit : "L/min");
        series_channel_json(job.body, series, c);
        job.body.print("}");
    }
    job.body.print("],\"events_truncated\":%s,\"events\":[", events_truncated ? "true" : "false");
    if (events.length()) job.body.append(events.c_str(), events.length());
    job.body.print("]}");
    timing.encode_us += timing.stamp() - started;
    return job.body.good() && events.good();
}

void error_body(Job &job, int code, const char *error) {
    job.body.~Payload();
    new (&job.body) Payload;
    job.code = code;
    job.error = error;
    job.body.print("{\"source\":\"edf\",\"error\":\"%s\"}", error);
}

bool dirty() {
    EdfCatalog::Status catalog;
    EdfCatalog::get_status(catalog);
    return !index_ready || __atomic_load_n(&invalidation_generation, __ATOMIC_ACQUIRE) != indexed_invalidation ||
        catalog.generation != catalog_generation ||
        SdStorage::files_revision() != file_revision;
}

void defer_job(Job *job) {
    if (!job) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (job->cancelled) reset_job(*job);
    else job->state = JobState::Queued;
    xSemaphoreGive(mutex);
}

bool work_once() {
    static const char *reported_index_error = nullptr;
    Job *job = nullptr;
    xSemaphoreTake(mutex, portMAX_DELAY);
    for (auto &candidate : jobs) {
        if (reusable(candidate)) reset_job(candidate);
        if (candidate.state == JobState::Queued &&
            (!job || int32_t(candidate.id - job->id) < 0)) job = &candidate;
    }
    if (job) job->state = JobState::Running;
    xSemaphoreGive(mutex);
    if (!AirSenseState::local_background_allowed()) {
        defer_job(job); publish(State::Blocked, "not_idle"); return false;
    }
    if (!SdStorage::local_access_allowed()) { defer_job(job); publish(State::Blocked, "usb_storage"); return false; }
    if (!SdStorage::mounted()) { defer_job(job); publish(State::Blocked, "no_sd"); return false; }
    if (!job && !dirty()) { publish(State::Ready); return false; }
    SdStorage::Session session;
    if (!session.begin()) { defer_job(job); publish(State::WaitingStorage, "storage_busy"); return false; }
    JobTiming timing(job);
    publish(State::Building);
    const uint32_t index_started = timing.stamp();
    const bool indexed = refresh_index(session);
    timing.index_us = timing.stamp() - index_started;
    if (!indexed) {
        const bool preempted = !session.valid() || !AirSenseState::local_background_allowed();
        session.end();
        if (preempted || index_failure == IndexFailure::Preempted || index_failure == IndexFailure::WaitingCatalog) {
            defer_job(job);
            publish(preempted ? State::Blocked : State::WaitingStorage,
                    preempted ? "not_idle" : "catalog_not_ready");
        } else {
            Log::logf(CAT_REPORT, reported_index_error == index_error ? LOG_DEBUG : LOG_WARN,
                "EDF index failed: %s entry=%ld\n", index_error,
                index_error_entry == SIZE_MAX ? -1L : long(index_error_entry));
            reported_index_error = index_error;
            if (job) {
                xSemaphoreTake(mutex, portMAX_DELAY);
                job->data_revision = __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
                error_body(*job, job->cancelled ? 410 : 503, job->cancelled ? "cancelled" : "index_unavailable");
                job->state = JobState::Complete;
                job->completed_ms = millis();
                const int code = job->code;
                const size_t bytes = job->body.length();
                xSemaphoreGive(mutex);
                portENTER_CRITICAL(&status_mux);
                ++status.revision;
                portEXIT_CRITICAL(&status_mux);
                timing.log(code, bytes);
            }
            publish(State::Error, "index_unavailable");
            return job != nullptr;
        }
        return false;
    }
    if (reported_index_error) {
        Log::logf(CAT_REPORT, LOG_INFO, "EDF index recovered\n");
        reported_index_error = nullptr;
    }
    if (!job) {
        session.end();
        publish(State::Ready);
        return false;
    }
    xSemaphoreTake(mutex, portMAX_DELAY);
    job->state = JobState::Running;
    job->data_revision = __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
    job->code = 0;
    job->error = job->issue = nullptr;
    job->issue_path[0] = 0;
    xSemaphoreGive(mutex);
    bool success = false;
    if (__atomic_load_n(&job->cancelled, __ATOMIC_ACQUIRE)) error_body(*job, 410, "cancelled");
    else if (job->request.revision && job->request.revision != job->data_revision)
        error_body(*job, 409, "stale_revision");
    else {
        success = job->request.action == Action::Days ? days_json(*job) :
            job->request.action == Action::Summary ? summary_json(session, *job) : series_json(session, *job, timing);
        if (success) job->code = 200;
    }
    const bool interrupted = !AirSenseState::local_background_allowed() || !session.valid();
    EdfCatalog::Status catalog;
    EdfCatalog::get_status(catalog);
    const bool changed = catalog.generation != catalog_generation || SdStorage::files_revision() != file_revision ||
        job->data_revision != __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
    session.end();
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (job->cancelled) error_body(*job, 410, "cancelled");
    else if (interrupted || changed) {
        job->body.~Payload();
        new (&job->body) Payload;
        job->state = JobState::Queued;
        job->code = 0;
        xSemaphoreGive(mutex);
        publish(interrupted ? State::Blocked : State::Building, interrupted ? "not_idle" : "");
        return false;
    } else if (!success && !job->code) error_body(*job, 503, "report_build_failed");
    job->state = JobState::Complete;
    job->completed_ms = millis();
    const int code = job->code;
    const size_t bytes = job->body.length();
    log_result(*job);
    xSemaphoreGive(mutex);
    portENTER_CRITICAL(&status_mux);
    ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    publish(State::Ready);
    timing.log(code, bytes);
    return true;
}

void worker(void *) {
    while (true) {
        if (work_once()) continue;
        Status publication;
        get_status(publication);
        xSemaphoreTake(wake, publication.state == State::WaitingStorage || publication.state == State::Blocked
            ? pdMS_TO_TICKS(1000) : portMAX_DELAY);
    }
}

}  // namespace

void init() {
    if (mutex) return;
    mutex = xSemaphoreCreateMutex();
    wake = xSemaphoreCreateBinary();
    portENTER_CRITICAL(&status_mux);
    status.supported = true;
    status.data_revision = data_revision;
    ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    if (!mutex || !wake) {
        Log::logf(CAT_REPORT, LOG_ERROR, "EDF worker synchronization allocation failed\n");
        publish(State::Error, "worker_unavailable");
        return;
    }
    BaseType_t created = pdFAIL;
    if (aircannect::Memory::psram_available())
        created = xTaskCreatePinnedToCoreWithCaps(worker, "edf_report", 8192, nullptr, 1,
            &worker_task, 0, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (created != pdPASS)
        created = xTaskCreatePinnedToCoreWithCaps(worker, "edf_report", 8192, nullptr, 1,
            &worker_task, 0, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (created != pdPASS) {
        Log::logf(CAT_REPORT, LOG_ERROR, "EDF worker task allocation failed\n");
        worker_task = nullptr;
        publish(State::Error, "worker_unavailable");
    }
    tick();
}

void tick() {
    static uint32_t observed_catalog = 0, observed_files = 0;
    const bool local = SdStorage::local_access_allowed();
    const bool mounted = SdStorage::mounted();
    const bool available = local && mounted && worker_task && AirSenseState::local_background_allowed();
    const uint32_t catalog = EdfCatalog::revision(), files = SdStorage::files_revision();
    bool changed = false, invalidated = false;
    portENTER_CRITICAL(&status_mux);
    // A deliberate USB unmount pauses reports, not a request to rebuild metadata.
    if (local && status.mounted != mounted) {
        status.mounted = mounted;
        invalidated = true;
        changed = true;
    }
    if (status.available != available) { status.available = available; changed = true; }
    if (observed_files != files) { observed_files = files; invalidated = true; changed = true; }
    if (invalidated) {
        status.data_revision = __atomic_add_fetch(&data_revision, 1, __ATOMIC_RELEASE);
        __atomic_add_fetch(&invalidation_generation, 1, __ATOMIC_RELEASE);
    }
    if (changed) ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    if (!local) publish(State::Blocked, "usb_storage");
    else if (!mounted) publish(State::Blocked, "no_sd");
    else if (!available && worker_task) publish(State::Blocked, "not_idle");
    if (wake && (changed || observed_catalog != catalog)) {
        observed_catalog = catalog;
        xSemaphoreGive(wake);
    }
    if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return;
    for (auto &job : jobs) if (reusable(job)) reset_job(job);
    xSemaphoreGive(mutex);
}

void get_status(Status &out) {
    portENTER_CRITICAL(&status_mux);
    out = status;
    portEXIT_CRITICAL(&status_mux);
}
uint32_t revision() {
    portENTER_CRITICAL(&status_mux);
    const uint32_t out = status.revision;
    portEXIT_CRITICAL(&status_mux);
    return out;
}

bool submit(const Request &request, uint32_t &id, const char **error) {
    auto reject = [&](const char *reason) { if (error) *error = reason; return false; };
    if (!valid(request)) return reject("request_invalid");
    Status publication;
    get_status(publication);
    if (!worker_task) return reject("worker_unavailable");
    if (!publication.mounted) return reject("no_sd");
    if (!publication.available) return reject("not_idle");
    if (request.revision && request.revision != publication.data_revision) return reject("stale_revision");
    if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return reject("busy");
    EdfCatalog::Status catalog;
    EdfCatalog::get_status(catalog);
    const bool current_index = catalog.ready &&
        catalog.generation == publication.catalog_generation &&
        SdStorage::files_revision() == publication.files_revision;
    Job *slot = nullptr, *delivered = nullptr;
    for (auto &job : jobs) {
        if (current_index && !job.cancelled && job.state == JobState::Complete && job.code == 200 &&
            job.data_revision == publication.data_revision && job.request == request &&
            uint32_t(millis() - job.completed_ms) < RETAIN_MS) {
            if (job.consumers == UINT16_MAX) { xSemaphoreGive(mutex); return reject("busy"); }
            ++job.consumers;
            job.shared = true;
            job.completed_ms = millis();
            id = job.id;
            xSemaphoreGive(mutex);
            portENTER_CRITICAL(&status_mux);
            ++status.revision;
            portEXIT_CRITICAL(&status_mux);
            return true;
        }
        if (!job.cancelled && (job.state == JobState::Queued || job.state == JobState::Running) &&
            job.request == request) {
            if (job.consumers == UINT16_MAX) { xSemaphoreGive(mutex); return reject("busy"); }
            ++job.consumers;
            job.shared = true;
            id = job.id;
            xSemaphoreGive(mutex);
            return true;
        }
        if (reusable(job)) slot = &job;
        if (job.state == JobState::Complete && !job.readers && job.data_revision != publication.data_revision)
            slot = &job;
        if (job.state == JobState::Complete && !job.readers && job.delivered &&
            job.deliveries >= job.consumers && (!delivered ||
            int32_t(job.id - delivered->id) < 0)) delivered = &job;
    }
    // Retention is opportunistic: routine days->summary->series and repeated
    // zooms can reclaim delivered/unpinned slots without a browser delay timer.
    if (!slot) slot = delivered;
    if (!slot) { xSemaphoreGive(mutex); return reject("busy"); }
    reset_job(*slot);
    slot->request = request;
    slot->id = next_id++;
    if (!next_id) next_id = 1;
    slot->state = JobState::Queued;
    id = slot->id;
    xSemaphoreGive(mutex);
    portENTER_CRITICAL(&status_mux);
    ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    xSemaphoreGive(wake);
    return true;
}

void cancel(uint32_t id) {
    if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return;
    for (auto &job : jobs) {
        if (job.id != id || job.shared || job.readers) continue;
        __atomic_store_n(&job.cancelled, true, __ATOMIC_RELEASE);
        if (job.state == JobState::Queued || job.state == JobState::Complete) reset_job(job);
        break;
    }
    xSemaphoreGive(mutex);
    if (wake) xSemaphoreGive(wake);
}

int poll(uint32_t id, Result &out) {
    out.reset();
    if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return 503;
    int code = 410;
    for (auto &job : jobs) {
        if (job.id != id || job.state == JobState::Free) continue;
        if (job.state != JobState::Complete) { code = 202; break; }
        Status publication;
        get_status(publication);
        if (job.code == 200 && (!SdStorage::local_access_allowed() || !SdStorage::mounted() || !publication.mounted ||
            job.data_revision != publication.data_revision ||
            SdStorage::files_revision() != publication.files_revision)) { code = 409; break; }
        EdfCatalog::Status catalog;
        EdfCatalog::get_status(catalog);
        if (job.code == 200 && catalog.generation != publication.catalog_generation) { code = 202; break; }
        if (!job.body.length() || job.readers == UINT16_MAX) { code = 503; break; }
        code = job.code;
        ++job.readers;
        if (job.deliveries < job.consumers) ++job.deliveries;
        job.delivered = true;
        out.id_ = job.id;
        out.data_ = job.body.c_str();
        out.length_ = job.body.length();
        break;
    }
    xSemaphoreGive(mutex);
    return code;
}

void Result::reset() {
    if (!id_) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    for (auto &job : jobs) if (job.id == id_) {
        if (job.readers) --job.readers;
        if (reusable(job)) reset_job(job);
        break;
    }
    xSemaphoreGive(mutex);
    id_ = 0;
    data_ = nullptr;
    length_ = 0;
}

}  // namespace EdfReport
#else
namespace EdfReport {
void init() {}
void tick() {}
void get_status(Status &out) {
    out = {};
    snprintf(out.error, sizeof(out.error), "%s", "unsupported");
}
uint32_t revision() { return 0; }
bool submit(const Request &, uint32_t &, const char **error) {
    if (error) *error = "unsupported";
    return false;
}
void cancel(uint32_t) {}
int poll(uint32_t, Result &out) { out.reset(); return 422; }
void Result::reset() { id_ = 0; data_ = nullptr; length_ = 0; }
}  // namespace EdfReport
#endif
