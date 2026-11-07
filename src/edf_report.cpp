#include "edf_report.h"
#include "board.h"
#include "sd_storage.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <functional>
#include <memory>
#include <utility>

namespace EdfReport {

bool Request::operator==(const Request &other) const {
    return action == other.action && view == other.view && day == other.day &&
        period == other.period && revision == other.revision;
}

bool valid(const Request &request) {
    if (request.action > Action::Reindex || uint8_t(request.action) == 2 || request.view > View::Period ||
        request.day == UINT16_MAX) return false;
    if (request.action == Action::History && request.view != View::Day) return false;
    if (request.action == Action::Reindex && (request.view != View::Day || request.revision)) return false;
    if (request.view == View::Period && request.period != 1 && request.period != 7 &&
        request.period != 30 && request.period != 90 && request.period != 180 &&
        request.period != 365) return false;
    return true;
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
    : id_(std::exchange(other.id_, 0)), readers_(std::exchange(other.readers_, nullptr)),
      data_(std::exchange(other.data_, nullptr)),
      length_(std::exchange(other.length_, 0)) {}

}  // namespace EdfReport

#if AB_STORAGE_HAS_SDCARD
#include "air10_clock.h"
#include "air10_edf.h"
#include "air10_str_timeline.h"
#include "airsense_state.h"
#include "edf_catalog.h"
#include "debug_log.h"
#include "memory_manager.h"
#include "large_text_buffer.h"
#include "json_util.h"
#include "job_progress_watch.h"
#include "hex_util.h"
#include "report_edf.h"
#include "report_artifact_builder.h"
#include "settings_defs.h"
#include <Arduino.h>
#include <FS.h>
#include <esp_heap_caps.h>
#include <esp_random.h>
#include <freertos/semphr.h>
#include <cstdarg>
#include <cstdio>
#include <new>

namespace EdfReport {
namespace {
constexpr uint32_t STATS_CACHE_VERSION = 7;
constexpr size_t MAX_BINS = 16384;
constexpr size_t MAX_EVENTS = 4096;
constexpr size_t MAX_DISK_EVENTS = 65536;
constexpr size_t MAX_BODY = 512 * 1024;
constexpr size_t INTERNAL_BODY = 16 * 1024;
constexpr size_t INTERNAL_WORK = 8 * 1024;
constexpr uint32_t RETAIN_MS = 6 * 60 * 60 * 1000;
constexpr uint32_t DELIVERY_MS = 30000;
constexpr uint32_t FOREGROUND_QUIET_MS = 1000;
constexpr uint8_t FOREGROUND_BURST = 3;
constexpr uint32_t FOREGROUND_TURN_MS = 5000;
constexpr uint32_t BACKGROUND_TURN_MS = 100;
constexpr uint32_t DAYS_PROTECTED_TURN_MS = 1000;
constexpr size_t SLOT_COUNT = 2;
constexpr size_t RETAINED_SLOTS = 32;
constexpr double LEAK_THRESHOLD = 24;
constexpr const char *CACHE_DIR = "/airbridge/report";
constexpr const char *SUFFIXES[] = {"BRP", "PLD", "SAD", "EVE", "CSL"};
char dataset_id[65] = {};
uint32_t rebuild_generation = 0;

struct ChannelSpec {
    const char *id, *label, *unit, *signal;
    uint8_t file;
    ReportArtifact::SignalId artifact_id;
};
using ReportArtifact::SignalId;
constexpr ChannelSpec CHANNELS[] = {
    {"Press", "Pressure", "cmH2O", "Press.2s", 1, SignalId::Press},
    {"EprPress", "Expiratory pressure", "cmH2O", "EprPress.2s", 1, SignalId::EprPress},
    {"MaskPress", "Mask pressure", "cmH2O", "MaskPress.2s", 1, SignalId::MaskPress},
    {"Leak", "Leak", "L/min", "Leak.2s", 1, SignalId::Leak},
    {"MinVent", "Minute ventilation", "L/min", "MinVent.2s", 1, SignalId::MinVent},
    {"RespRate", "Respiratory rate", "bpm", "RespRate.2s", 1, SignalId::RespRate},
    {"TidVol", "Tidal volume", "mL", "TidVol.2s", 1, SignalId::TidVol},
    {"SpO2", "SpO2", "%", "SpO2.1s", 2, SignalId::SpO2},
    {"Pulse", "Pulse", "bpm", "Pulse.1s", 2, SignalId::Pulse},
    {"FlowLim", "Flow limitation", "", "FlowLim.2s", 1, SignalId::FlowLim},
    {"Snore", "Snore", "", "Snore.2s", 1, SignalId::Snore},
    {"TgtVent", "Target ventilation", "L/min", "TgtVent.2s", 1, SignalId::TgtVent},
    {"Ti", "Inspiratory time", "s", "Ti.2s", 1, SignalId::Ti},
    {"IERatio", "I:E", "%", "IERatio.2s", 1, SignalId::IERatio},
    {"AlvMinVent", "Alveolar ventilation", "L/min", "AlvMinVent.2s", 1, SignalId::AlvMinVent},
};
constexpr size_t CHANNEL_COUNT = sizeof(CHANNELS) / sizeof(CHANNELS[0]);

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
    double csr_seconds = 0, leak_high_seconds = 0;
    bool csr_known = false;
};
struct ReportWork {
    static void *operator new(size_t size, const std::nothrow_t &) noexcept {
        return aircannect::Memory::alloc_large(size, size <= INTERNAL_WORK);
    }
    static void operator delete(void *memory) noexcept { aircannect::Memory::free(memory); }
};
struct SessionData : ReportWork {
    CacheHeader header;
    uint64_t *bins = nullptr;
    Event *events = nullptr;
    ~SessionData() {
        aircannect::Memory::free(bins);
        aircannect::Memory::free(events);
    }
};

struct DailyValues {
    uint64_t count[4] = {}, events = 0;
    double sum[4] = {}, seconds = 0;
    bool present = false, ready = true, usage = true, event_count = true;

    double value(size_t metric) const {
        if (!present || !ready) return NAN;
        if (!metric) return usage && seconds > 0 ? seconds : NAN;
        if (metric == 1) return usage && event_count && seconds > 0 ? events * 3600.0 / seconds : NAN;
        return count[metric - 2] ? sum[metric - 2] / count[metric - 2] : NAN;
    }
};
struct CachedDay {
    DailyValues values;
    uint32_t updated = 0;
    uint16_t day = 0;
    bool has_values = false, artifacts_checked = false;
    uint8_t artifact_key[32] = {};
    uint8_t cleanup_key[32] = {};
    bool cleanup_checked = false;
};
// Two annual selections; lazy growth, no per-day SD files or internal bulk fallback.
constexpr size_t MAX_CACHED_DAYS = 2 * 366;
CachedDay *daily_cache = nullptr;
size_t daily_capacity = 0;
uint32_t daily_update = 0;

void invalidate_day(uint16_t day = 0) {
    if (!day) {
        aircannect::Memory::free(daily_cache);
        daily_cache = nullptr;
        daily_capacity = 0;
        return;
    }
    for (size_t i = 0; i < daily_capacity; ++i)
        if (daily_cache[i].day == day) daily_cache[i].day = 0;
}
CachedDay *cached_day_entry(uint16_t day, bool create) {
    if (!day) return nullptr;
    size_t slot = daily_capacity;
    for (size_t i = 0; i < daily_capacity; ++i) {
        if (daily_cache[i].day == day) {
            daily_cache[i].updated = ++daily_update;
            return &daily_cache[i];
        }
        if (!daily_cache[i].day && slot == daily_capacity) slot = i;
    }
    if (!create) return nullptr;
    if (slot == daily_capacity && daily_capacity < MAX_CACHED_DAYS) {
        const size_t capacity = std::min(MAX_CACHED_DAYS, std::max(size_t(16), daily_capacity * 2));
        auto *grown = static_cast<CachedDay *>(aircannect::Memory::realloc_large(
            daily_cache, capacity * sizeof(CachedDay), false));
        if (grown) {
            daily_cache = grown;
            for (size_t i = daily_capacity; i < capacity; ++i) new (&daily_cache[i]) CachedDay;
            daily_capacity = capacity;
        }
    }
    if (!daily_capacity) return nullptr; // Cache allocation failure does not fail a report.
    if (slot == daily_capacity) {
        slot = 0;
        for (size_t i = 1; i < daily_capacity; ++i)
            if (int32_t(daily_cache[i].updated - daily_cache[slot].updated) < 0) slot = i;
    }
    daily_cache[slot] = {};
    daily_cache[slot].day = day;
    daily_cache[slot].updated = ++daily_update;
    return &daily_cache[slot];
}
bool cached_day(uint16_t day, DailyValues &out, bool build) {
    const auto *cached = cached_day_entry(day, false);
    if (!cached || !cached->has_values || (build && !cached->values.ready)) return false;
    out = cached->values;
    return true;
}
void remember_day(uint16_t day, const DailyValues &values) {
    if (auto *cached = cached_day_entry(day, true)) {
        cached->values = values;
        cached->has_values = true;
    }
}

struct Payload : aircannect::LargeTextBuffer {
    Payload() : LargeTextBuffer(MAX_BODY, INTERNAL_BODY) {}
    bool format_error = false;
    bool good() const { return !format_error && !overflowed(); }
    size_t write(uint8_t byte) { return write(&byte, 1); }
    size_t write(const uint8_t *bytes, size_t length) {
        return append(reinterpret_cast<const char *>(bytes), length) ? length : 0;
    }
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
struct SessionBuild;
struct SummaryWork;
struct HistoryWork;
struct PrepareWork;
struct Job;
struct RetainedIssue {
    const char *reason = nullptr;
    char path[96] = {};
    void capture(const Job &job);
    void restore(const Job &job) const;
};
struct Job {
    ~Job();
    Request request;
    JobState state = JobState::Free;
    uint32_t id = 0, completed_ms = 0, data_revision = 0, cache_generation = 0;
    uint32_t artifact_revision = 0, files_revision = 0;
    std::atomic<uint16_t> readers{0};
    uint16_t consumers = 1, deliveries = 0;
    bool delivered = false, cancelled = false, shared = false;
    bool background = false, recovered_index = false;
    mutable bool storage_yielded = false;
    uint32_t foreground_epoch = 0;
    uint32_t days_epoch = 0;
    uint32_t turn_started_ms = 0, turn_budget_ms = 0;
    bool protected_turn = false, backfill_waiting = false;
    mutable aircannect::JobProgressPoint turn_progress;
    mutable bool turn_progress_set = false;
    int code = 0;
    const char *error = nullptr;
    mutable const char *issue = nullptr;
    mutable char issue_path[96] = {};
    mutable RetainedIssue accepted_issue;
    mutable uint64_t source_bytes = 0;
    mutable uint64_t source_us = 0, tiles_us = 0;
    mutable uint64_t sleep_us = 0, observe_us = 0;
    mutable uint64_t cleanup_us = 0, headroom_us = 0;
    mutable uint32_t headroom_calls = 0;
    mutable uint32_t source_reads = 0;
    mutable uint32_t max_idle_ms = 0;
    uint32_t admitted_ms = 0, attempts = 0, yields = 0;
    uint64_t active_us = 0, index_us = 0;
    // Histogram and resumed source work are subsets of their phases. Resume
    // includes useful build_session work, not just reopening a paused input.
    uint64_t prepare_numeric_us = 0, prepare_events_us = 0, prepare_histograms_us = 0;
    uint64_t prepare_history_us = 0, prepare_summary_us = 0, prepare_publish_us = 0, prepare_resume_us = 0;
    bool diagnostics = false;
    mutable aircannect::JobProgressWatch progress;
    const char *wait_reason = "queued";
    bool indexing = false;
    uint32_t listed_days = 0;
    Payload body;
    mutable std::unique_ptr<SessionBuild> build;
    std::unique_ptr<SummaryWork> summary;
    std::unique_ptr<HistoryWork> history;
    std::unique_ptr<PrepareWork> preparation;
};

void observe_report_jobs(bool force = false, bool retained = false,
                         const Job *io_job = nullptr, const SdStorage::IoEvent *io_event = nullptr);
aircannect::JobProgressPoint report_progress_point(const Job &job);

bool note_issue(const Job &job, const char *reason, const char *path = "") {
    if (!job.issue) {
        job.issue = reason;
        snprintf(job.issue_path, sizeof(job.issue_path), "%s", path);
    }
    return false;
}

void RetainedIssue::capture(const Job &job) {
    if (!reason && job.issue) {
        reason = job.issue;
        snprintf(path, sizeof(path), "%s", job.issue_path);
    }
}
void RetainedIssue::restore(const Job &job) const {
    if (reason) note_issue(job, reason, path);
}

// A transferable source keeps only its own accepted degradation, never an
// earlier session's diagnostic or an error from an interrupted attempt.
struct SourceIssueScope {
    const Job &job;
    RetainedIssue previous;
    SourceIssueScope(const Job &job, const RetainedIssue &source) : job(job) {
        previous.capture(job);
        job.issue = nullptr;
        job.issue_path[0] = 0;
        source.restore(job);
    }
    ~SourceIssueScope() {
        if (previous.reason) {
            job.issue = nullptr;
            previous.restore(job);
        }
    }
};

const char *action_name(Action action) {
    return action == Action::Days ? "days" : action == Action::Summary ? "summary" :
        action == Action::History ? "history" : action == Action::Prepare ? "prepare" : "reindex";
}

void log_result(const Job &job) {
    const bool expected = job.code == 409 || job.code == 410;
    const char *reason = expected ? job.error : job.issue ? job.issue : job.error;
    Log::logf(CAT_REPORT, expected ? LOG_DEBUG : job.code >= 500 ? LOG_ERROR :
        job.issue ? LOG_WARN : LOG_DEBUG,
        "EDF job=%lu %s day=%u code=%d %s path=%s\n",
        (unsigned long)job.id, action_name(job.request.action),
        unsigned(job.request.day), job.code, reason ? reason : "complete",
        expected || !job.issue_path[0] ? "-" : job.issue_path);
}

struct JobTiming {
    bool enabled;
    uint32_t id, started_us;
    Action action;
    uint32_t index_us = 0;
    Job *job;

    explicit JobTiming(Job *job)
        : enabled(job && Log::get_cat_level(CAT_REPORT) >= LOG_DEBUG),
          id(job ? job->id : 0), started_us(stamp()),
          action(job ? job->request.action : Action::Days), job(job) {
        if (job) {
            job->diagnostics = enabled;
            ++job->attempts;
        }
    }

    uint32_t stamp() const { return enabled ? micros() : 0; }

    void finish(bool yielded = false) {
        if (!job) return;
        if (enabled) {
            job->active_us += uint32_t(stamp() - started_us);
            job->index_us += index_us;
        }
        if (yielded) ++job->yields;
        started_us = stamp();
        index_us = 0;
    }

    void log() {
        if (!enabled) return;
        finish();
        Log::logf(CAT_REPORT, LOG_DEBUG,
            "report job=%lu ms elapsed=%lu active=%llu attempts=%lu yields=%lu gap_ms=%lu\n",
            static_cast<unsigned long>(id), static_cast<unsigned long>(millis() - job->admitted_ms),
            static_cast<unsigned long long>(job->active_us / 1000),
            static_cast<unsigned long>(job->attempts), static_cast<unsigned long>(job->yields),
            static_cast<unsigned long>(job->max_idle_ms));
        Log::logf(CAT_REPORT, LOG_DEBUG, "report job=%lu ms idx=%llu src=%llu tiles=%llu histograms=%llu\n",
            static_cast<unsigned long>(id), static_cast<unsigned long long>(job->index_us / 1000),
            static_cast<unsigned long long>(job->source_us / 1000),
            static_cast<unsigned long long>(job->tiles_us / 1000),
            static_cast<unsigned long long>(job->prepare_histograms_us / 1000));
        Log::logf(CAT_REPORT, LOG_DEBUG,
            "report job=%lu source reads=%lu bytes=%llu headroom_ms=%llu calls=%lu\n",
            static_cast<unsigned long>(id), static_cast<unsigned long>(job->source_reads),
            static_cast<unsigned long long>(job->source_bytes),
            static_cast<unsigned long long>(job->headroom_us / 1000),
            static_cast<unsigned long>(job->headroom_calls));
        if (action == Action::Prepare) {
            Log::logf(CAT_REPORT, LOG_DEBUG, "report job=%lu ms numeric=%llu events=%llu cleanup=%llu obs=%llu\n",
                static_cast<unsigned long>(id), static_cast<unsigned long long>(job->prepare_numeric_us / 1000),
                static_cast<unsigned long long>(job->prepare_events_us / 1000),
                static_cast<unsigned long long>(job->cleanup_us / 1000),
                static_cast<unsigned long long>(job->observe_us / 1000));
            Log::logf(CAT_REPORT, LOG_DEBUG, "report job=%lu ms history=%llu summary=%llu pub=%llu sleep=%llu\n",
                static_cast<unsigned long>(id), static_cast<unsigned long long>(job->prepare_history_us / 1000),
                static_cast<unsigned long long>(job->prepare_summary_us / 1000),
                static_cast<unsigned long long>(job->prepare_publish_us / 1000),
                static_cast<unsigned long long>(job->sleep_us / 1000));
        }
    }
};

struct JobPool {
    Job fallback[SLOT_COUNT + 1];
    Job *data = fallback;
    size_t count = SLOT_COUNT + 1;
    // status_mux protects published lifetimes, active state transitions and
    // joins. Remove the ID before destroying buffers under the work mutex.
    uint32_t retained_ids[RETAINED_SLOTS] = {};
    bool delivery_waiting = false;
    Job *begin() { return data; }
    Job *end() { return data + count; }
} jobs;
SemaphoreHandle_t mutex = nullptr, wake = nullptr;
TaskHandle_t worker_task = nullptr;
portMUX_TYPE status_mux = portMUX_INITIALIZER_UNLOCKED;
Status status;
uint32_t next_id = 1, data_revision = 1;
uint32_t foreground_epoch = 0;
uint32_t days_epoch = 0;
uint8_t foreground_turns = 0;
uint8_t days_turns = 0;
bool backfill_due = false;
uint32_t backfill_work_ms = 0;
enum class MaintenancePhase : uint8_t { Index, Publish, IndexGc, ArtifactGc, Count };
struct MaintenanceProgress {
    aircannect::JobProgressWatch watch;
    MaintenancePhase phase = MaintenancePhase::Index;
    uint64_t cursor = 0, steps = 0;
    const char *wait_reason = "working";
    bool active = false, complete = false, observing = false;
} maintenance_progress[size_t(MaintenancePhase::Count)];
MaintenanceProgress *maintenance_io = nullptr;
uint32_t artifact_reads = 0;
uint32_t foreground_at = 0;
bool foreground_seen = false;
uint32_t backfill_revision = 0;
uint16_t backfill_before = UINT16_MAX;
uint8_t backfill_period = 0;
constexpr uint16_t PERIODS[] = {1, 7, 30, 90, 180, 365};
uint32_t published_index_data = 0, published_index_artifacts = UINT32_MAX;
bool index_commit_pending = false;
ReportArtifact::Document retired_index;
uint32_t retired_index_files = 0;
struct IndexCleanup {
    uint32_t files = UINT32_MAX;
    uint16_t shard = 0;
    uint8_t present[32] = {};
    bool discovered = false;
} index_cleanup;

struct CleanupAnchor {
    uint16_t day = 0;
    uint8_t digest[32] = {};
    uint32_t bytes = 0;
};
struct CleanupLegacy {
    uint16_t day = 0;
    uint32_t bytes = 0;
    char name[33] = {};
};
struct ArtifactCleanup {
    uint32_t files = UINT32_MAX, data = 0, artifacts = UINT32_MAX;
    size_t next = 0;
    bool discovered = false, complete = false;
    aircannect::LargeTextBuffer days{MAX_SESSION_COUNT * sizeof(uint16_t), 0};
    aircannect::LargeTextBuffer anchors{2 * MAX_BODY, 0};
    aircannect::LargeTextBuffer legacy{MAX_SESSION_COUNT * sizeof(CleanupLegacy), 0};
} artifact_cleanup;

struct CachedPublication {
    ReportArtifact::Document document;
    uint16_t day = 0, period = 0;
    ReportArtifact::Lookup state = ReportArtifact::Lookup::Failed;
};
CachedPublication *publications = nullptr;
size_t publication_count = 0, publication_capacity = 0;
uint32_t publication_files_revision = 0;

uint32_t background_retry = 0;
bool backfill_enabled = true;
IndexedSession *index = nullptr;
size_t index_count = 0;
uint32_t catalog_generation = 0, file_revision = 0;
uint32_t invalidation_generation = 0, indexed_invalidation = 0;
bool index_ready = false;
struct IndexScan {
    bool active = false, full = false;
    bool detail_changed = false, summary_changed = false;
    uint32_t catalog = 0, files = 0, invalidation = 0;
    uint32_t started_ms = 0, resumes = 0;
    size_t next = 0;
} index_scan;
bool cache_usage_known = false;
uint64_t cache_bytes = 0;
uint32_t cache_generation = 0;
enum class IndexFailure : uint8_t { Error, WaitingCatalog, Preempted };
IndexFailure index_failure = IndexFailure::Error;
const char *index_error = nullptr;
size_t index_error_entry = SIZE_MAX;
bool migrate_cache(const SdStorage::Session &session, const Job *job);
bool artifact_headroom(const SdStorage::Session &session, uint16_t day, const Job &job);
void invalidate_str();
bool dataset(const SdStorage::Session &session);

void invalidate_publications() {
    aircannect::Memory::free(publications);
    publications = nullptr;
    publication_count = publication_capacity = 0;
    publication_files_revision = SdStorage::files_revision(SdStorage::FileScope::Reports);
}

CachedPublication *publication_entry(uint16_t day, uint16_t period) {
    if (publication_files_revision != SdStorage::files_revision(SdStorage::FileScope::Reports)) invalidate_publications();
    // Descending days followed by periods: normal cold discovery appends, and
    // index serialization finds retained metadata without another linear scan.
    const uint32_t key = period ? 65536u + period : 65535u - day;
    size_t slot = 0, end = publication_count;
    while (slot < end) {
        const size_t mid = slot + (end - slot) / 2;
        const auto &entry = publications[mid];
        const uint32_t candidate = entry.period ? 65536u + entry.period : 65535u - entry.day;
        if (candidate < key) slot = mid + 1;
        else end = mid;
    }
    if (slot < publication_count && publications[slot].period == period &&
        (period || publications[slot].day == day)) {
        auto &entry = publications[slot];
        // Only the selected end date is retained for each of the six periods.
        if (entry.day != day) entry = {};
        entry.day = day;
        entry.period = period;
        return &entry;
    }
    if (publication_count == publication_capacity) {
        const size_t capacity = std::min(index_count + sizeof(PERIODS) / sizeof(PERIODS[0]),
            std::max(size_t(16), publication_capacity * 2));
        if (capacity <= publication_capacity) return nullptr;
        const size_t bytes = capacity * sizeof(CachedPublication);
        auto *grown = static_cast<CachedPublication *>(aircannect::Memory::realloc_large(
            publications, bytes, bytes <= INTERNAL_WORK));
        if (!grown) return nullptr;
        publications = grown;
        publication_capacity = capacity;
    }
    memmove(publications + slot + 1, publications + slot,
        (publication_count - slot) * sizeof(CachedPublication));
    auto *entry = new (&publications[slot]) CachedPublication;
    ++publication_count;
    entry->day = day;
    entry->period = period;
    return entry;
}

void remember_publication(uint16_t day, uint16_t period, const ReportArtifact::Document &document) {
    if (auto *entry = publication_entry(day, period)) {
        entry->document = document;
        entry->state = ReportArtifact::Lookup::Ready;
    }
    if (auto *entry = cached_day_entry(day, false)) entry->cleanup_checked = false;
}

ReportArtifact::Lookup lookup_publication(const SdStorage::Session &session,
    uint16_t day, uint16_t period, const char *prefix, ReportArtifact::Document &document) {
    if (!session.valid()) return ReportArtifact::Lookup::Failed;
    auto *entry = publication_entry(day, period);
    if (entry && entry->state != ReportArtifact::Lookup::Failed) {
        document = entry->document;
        return entry->state;
    }
    const auto state = ReportArtifact::document_lookup(session, prefix, nullptr, document);
    if (state == ReportArtifact::Lookup::Ready) {
        Payload verified;
        // Validate old publications once, including fallback to an older slot.
        // A failed read may be transient; never memoize it as missing.
        if (!ReportArtifact::document_read(session, prefix, document, verified, nullptr))
            return ReportArtifact::Lookup::Failed;
    }
    if (entry && session.valid()) {
        entry->document = document;
        entry->state = state;
    }
    return state;
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
            (job.code == 200 && !job.issue ? RETAIN_MS : DELIVERY_MS)));
}
void reset_job(Job &job) {
    portENTER_CRITICAL(&status_mux);
    jobs.retained_ids[&job - jobs.data] = 0;
    portEXIT_CRITICAL(&status_mux);
    job.~Job();
    new (&job) Job;
}

// Caller holds mutex; publish only fully admitted slots, before returning IDs.
void admit_job(Job &job) {
    job.admitted_ms = millis();
    portENTER_CRITICAL(&status_mux);
    jobs.retained_ids[&job - jobs.data] = job.id;
    if (!job.background && job.request.action == Action::Days)
        __atomic_add_fetch(&days_epoch, 1, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&status_mux);
}

// Caller holds mutex. Undelivered results get the existing delivery grace;
// active HTTP leases are never candidates, even under memory pressure.
bool evictable(const Job &job) {
    return job.state == JobState::Complete && !job.readers &&
        (job.deliveries >= job.consumers || uint32_t(millis() - job.completed_ms) >= DELIVERY_MS);
}
bool reclaim_result() {
    // Called by Memory in any allocating task, including error serialization
    // with this mutex already held. Do not wait or touch SD here.
    if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return false;
    Job *oldest = nullptr;
    for (auto &job : jobs) {
        if (evictable(job) && job.body.capacity() &&
            (!oldest || int32_t(job.completed_ms - oldest->completed_ms) < 0)) oldest = &job;
    }
    if (oldest) reset_job(*oldest);
    xSemaphoreGive(mutex);
    return oldest != nullptr;
}
bool aborted(const Job *job, const SdStorage::Session &session, bool cooperative = true) {
    // Owner-only work batches: SD operations remain individually bounded, and
    // CPU codec/histogram loops also yield without a delay per sample or tile.
    static uint32_t batch_us = 0;
    const uint32_t now = micros();
    if (uint32_t(now - batch_us) >= 4000) {
        vTaskDelay(1);
        batch_us = micros();
        if (job && job->diagnostics) job->sleep_us += uint32_t(batch_us - now);
    }
    const uint32_t observed = job && job->diagnostics ? micros() : 0;
    observe_report_jobs();
    if (job && job->diagnostics) job->observe_us += uint32_t(micros() - observed);
    if (cooperative && session.yield_requested()) {
        if (job) job->storage_yielded = true;
        return true;
    }
    const bool backfill_limit = cooperative && job && !job->background &&
        job->request.action != Action::Days && job->backfill_waiting &&
        uint32_t(millis() - job->turn_started_ms) >= FOREGROUND_TURN_MS;
    const bool turn_limit = cooperative && job && job->protected_turn &&
        uint32_t(millis() - job->turn_started_ms) >= job->turn_budget_ms;
    bool advanced = false;
    if (cooperative && job) {
        // Seed at the first real checkpoint, not the transient indexing flag
        // on every resume. Reopening alone cannot spend the useful turn.
        if (!job->turn_progress_set) {
            job->turn_progress = report_progress_point(*job);
            job->turn_progress_set = true;
        } else if (backfill_limit || turn_limit) {
            const auto point = report_progress_point(*job);
            advanced = point.step > job->turn_progress.step ||
                (point.step == job->turn_progress.step && point.units > job->turn_progress.units);
        }
    }
    const bool backfill_turn = backfill_limit && advanced;
    if (backfill_turn) backfill_due = true;
    if (cooperative && job &&
        (backfill_turn || (turn_limit && advanced) ||
         (!job->protected_turn &&
          ((job->background && job->foreground_epoch != __atomic_load_n(&foreground_epoch, __ATOMIC_ACQUIRE)) ||
           (job->request.action != Action::Days && job->days_epoch != __atomic_load_n(&days_epoch, __ATOMIC_ACQUIRE)))))) {
        job->storage_yielded = true;
        return true;
    }
    return (job && __atomic_load_n(&job->cancelled, __ATOMIC_ACQUIRE)) ||
        !AirSenseState::local_background_allowed() || !session.valid();
}

void source_path(const IndexedSession &entry, uint8_t file, const char *ext,
                 char (&out)[96]) {
    snprintf(out, sizeof(out), "/DATALOG/%s/%s_%s.%s", entry.entry.therapy_day,
             entry.entry.file_prefix, SUFFIXES[file], ext);
}
void cache_path(const IndexedSession &entry, const char *ext, char (&out)[96]) {
    if (rebuild_generation) snprintf(out, sizeof(out), "%s/%s/%s.g%lu.%s", CACHE_DIR,
        entry.entry.therapy_day, entry.entry.file_prefix, static_cast<unsigned long>(rebuild_generation), ext);
    else snprintf(out, sizeof(out), "%s/%s/%s.%s", CACHE_DIR,
        entry.entry.therapy_day, entry.entry.file_prefix, ext);
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

bool refresh_index(const SdStorage::Session &session, const Job *job = nullptr) {
    index_failure = IndexFailure::Error;
    index_error = "catalog_status";
    index_error_entry = SIZE_MAX;
    EdfCatalog::Changes changes;
    bool incremental = false;
    if (!session.run([&](fs::FS &) {
        incremental = EdfCatalog::changes_since(catalog_generation, changes);
        return changes.catalog.ready;
    })) {
        if (aborted(job, session)) index_failure = IndexFailure::Preempted;
        else if (!changes.catalog.ready) index_failure = IndexFailure::WaitingCatalog;
        return false;
    }
    const uint32_t files = SdStorage::files_revision(SdStorage::FileScope::Reports);
    const uint32_t invalidation = __atomic_load_n(&invalidation_generation, __ATOMIC_ACQUIRE);
    const bool invalidated = invalidation != indexed_invalidation;
    if (index_ready && !invalidated && changes.catalog.generation == catalog_generation && files == file_revision)
        return true;
    const bool resume = index_scan.active && index_scan.catalog == changes.catalog.generation &&
        index_scan.files == files && index_scan.invalidation == invalidation &&
        changes.catalog.entries == index_count;
    if (!resume) {
        // A changed source invalidates the partially updated in-memory index.
        // Mere SD/therapy preemption retains both the cursor and change flags.
        const bool full = index_scan.active || !index_ready || invalidated || !incremental ||
            files != file_revision || changes.catalog.entries < index_count;
        if (full) { invalidate_day(); invalidate_str(); dataset_id[0] = 0; }
        if (!dataset(session)) { index_error = "dataset_identity"; return false; }
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
        index_scan = {true, full, !index_ready || changes.catalog.entries < old_count, false,
                      changes.catalog.generation, files, invalidation, millis(), 0, 0};
        index_ready = false;
    } else ++index_scan.resumes;
    const size_t count = index_scan.full ? index_count : changes.count;
    for (; index_scan.next < count; ++index_scan.next) {
        if (aborted(job, session)) {
            index_failure = IndexFailure::Preempted;
            return false;
        }
        const size_t i = index_scan.full ? index_scan.next : changes.indexes[index_scan.next];
        index_error_entry = i;
        index_error = "catalog_index";
        if (i >= index_count) return false;
        IndexedSession next;
        index_error = "catalog_read";
        if (!session.run([&](fs::FS &) { return EdfCatalog::read(i, next.entry); })) return false;
        index_error = "catalog_day";
        if (!catalog_day(next.entry.therapy_day, next.day)) return false;
        // STR/Identification readiness is not a detail-data revision.
        index_scan.summary_changed |= index[i].entry.str_digest != next.entry.str_digest ||
            ((index[i].entry.flags ^ next.entry.flags) & EdfCatalog::ENTRY_STR_READY);
        const bool changed = strcmp(index[i].entry.file_prefix, next.entry.file_prefix) ||
            strcmp(index[i].entry.therapy_day, next.entry.therapy_day) ||
            ((index[i].entry.flags ^ next.entry.flags) & EdfCatalog::ENTRY_LIVE_COMPLETE);
        if (index_scan.full || changed) {
            index_error = "source_fingerprint";
            if (!fingerprint(session, next)) return false;
            const bool key_changed = !(index[i].key == next.key);
            if (changed || key_changed) {
                if (index[i].day) invalidate_day(index[i].day);
                invalidate_day(next.day);
            }
            // A changed fingerprint is not proof that the sources can rebuild
            // the old cache. Its header rejects mismatches; retain the bytes
            // until a validated replacement is available.
            index_scan.detail_changed |= changed || key_changed;
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
    if (index_scan.full) {
        cache_usage_known = false;
        if (!migrate_cache(session, job)) {
            index_error = "cache_migrate";
            if (aborted(job, session)) index_failure = IndexFailure::Preempted;
            return false;
        }
    }
    EdfCatalog::Status current;
    EdfCatalog::get_status(current);
    if (aborted(job, session) || !current.ready || current.generation != index_scan.catalog ||
        SdStorage::files_revision(SdStorage::FileScope::Reports) != index_scan.files ||
        __atomic_load_n(&invalidation_generation, __ATOMIC_ACQUIRE) != index_scan.invalidation) {
        index_failure = IndexFailure::Preempted;
        return false;
    }
    const bool detail_changed = index_scan.detail_changed, summary_changed = index_scan.summary_changed;
    if (summary_changed) invalidate_str();
    catalog_generation = changes.catalog.generation;
    file_revision = files;
    indexed_invalidation = invalidation;
    index_ready = true;
    if ((detail_changed || summary_changed) && !invalidated) __atomic_add_fetch(&data_revision, 1, __ATOMIC_RELEASE);
    portENTER_CRITICAL(&status_mux);
    status.catalog_generation = catalog_generation;
    status.files_revision = file_revision;
    status.sessions = index_count;
    status.data_revision = __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
    if (detail_changed || summary_changed || invalidated) ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    Log::logf(CAT_REPORT, LOG_DEBUG, "EDF index ready entries=%u checked=%u resumes=%lu elapsed=%lums\n",
        unsigned(index_count), unsigned(count), static_cast<unsigned long>(index_scan.resumes),
        static_cast<unsigned long>(millis() - index_scan.started_ms));
    index_scan = {};
    return true;
}

struct Input {
    SdStorage::Reader file;
    ReportEdf::Reader reader;
    ReportEdf::Header *header = nullptr;
    uint8_t *record = nullptr;
    size_t record_capacity = 0;
    uint64_t record_offset = UINT64_MAX;
    size_t record_loaded = 0;
    bool header_ready = false, reading_header = false;
    char path[96] = {};
    SdStorage::Session session;
    const Job *job;
    ReportArtifact::Producer *artifact = nullptr;
    Input(const SdStorage::Session &session, const Job &job) : session(session), job(&job) {
        void *memory = aircannect::Memory::alloc_large(sizeof(ReportEdf::Header),
                                                      sizeof(ReportEdf::Header) <= INTERNAL_WORK);
        if (memory) header = new (memory) ReportEdf::Header;
    }
    ~Input() {
        aircannect::Memory::free(header);
        aircannect::Memory::free(record);
    }
    size_t read_source(uint64_t offset, uint8_t *out, size_t bytes) {
        const uint32_t started = job->diagnostics ? micros() : 0;
        const size_t count = file.read_at(offset, out, bytes) ? bytes : 0;
        if (job->diagnostics) {
            job->source_us += uint32_t(micros() - started);
            job->source_bytes += count;
            ++job->source_reads;
        }
        return count;
    }
    bool open(const SdStorage::Session &session, const IndexedSession &entry, uint8_t kind) {
        char source[96];
        source_path(entry, kind, "edf", source);
        this->session = session;
        if (header_ready && !strcmp(path, source))
            return file || file.open(session, path);
        if (!open_file(source)) return false;
        // A bounded descriptor read is the smallest resumable metadata step.
        // Recording/USB revocation still interrupts each underlying read.
        reading_header = true;
        const bool parsed = ReportEdf::read_header(reader, *header);
        reading_header = false;
        if (!parsed) return note_issue(*job, "source_header", path);
        header_ready = prepare_record();
        return header_ready;
    }
    bool open_file(const char *source) {
        if (!header) return note_issue(*job, "header_alloc");
        record_offset = UINT64_MAX;
        record_loaded = 0;
        header_ready = false;
        *header = {};
        snprintf(path, sizeof(path), "%s", source);
        if (!file.open(session, path)) return note_issue(*job, "source_open", path);
        reader = {[](void *ctx, uint64_t offset, uint8_t *out, size_t count) {
            auto &input = *static_cast<Input *>(ctx);
            if (aborted(input.job, input.session, !input.reading_header)) return false;
            const auto &header = *input.header;
            if (input.record && header.record_bytes && offset >= header.header_bytes &&
                offset + count <= header.header_bytes + uint64_t(header.records) * header.record_bytes) {
                const uint64_t base = header.header_bytes +
                    ((offset - header.header_bytes) / header.record_bytes) * header.record_bytes;
                if (offset + count <= base + header.record_bytes) {
                    if (!input.load_record((base - header.header_bytes) / header.record_bytes)) return false;
                    memcpy(out, input.record + (offset - base), count);
                    return true;
                }
            }
            size_t done = 0;
            while (done < count) {
                const size_t take = std::min(count - done, SdStorage::READ_CHUNK_BYTES);
                if (input.read_source(offset + done, out + done, take) != take)
                    return note_issue(*input.job, "source_read", input.path);
                done += take;
            }
            return true;
        }, this, file.size()};
        return true;
    }
    bool prepare_record() {
        if (header->record_bytes > record_capacity) {
            // One actual record, at most the reader's validated 64 KiB limit.
            void *grown = aircannect::Memory::realloc_large(record, header->record_bytes,
                                                           header->record_bytes <= INTERNAL_WORK);
            if (grown) {
                record = static_cast<uint8_t *>(grown);
                record_capacity = header->record_bytes;
            }
        }
        if (record_capacity < header->record_bytes) return note_issue(*job, "record_alloc", path);
        return true;
    }

    bool load_record(uint32_t index) {
        if (aborted(job, session) || index >= header->records || !record ||
            record_capacity < header->record_bytes) return false;
        const uint64_t base = header->header_bytes + uint64_t(index) * header->record_bytes;
        if (base != record_offset) {
            record_offset = base;
            record_loaded = 0;
        }
        if (record_loaded == header->record_bytes) return true;
        while (record_loaded < header->record_bytes) {
            if (aborted(job, session)) return false;
            const size_t take = std::min<size_t>(header->record_bytes - record_loaded, SdStorage::READ_CHUNK_BYTES);
            if (read_source(base + record_loaded, record + record_loaded, take) != take)
                return note_issue(*job, "source_read", path);
            record_loaded += take;
        }
        return true;
    }

    template <typename Sink>
    bool scan_records(const int *signals, size_t count, uint32_t &next, uint32_t end, Sink consume) {
        if (next > end || end > header->records) return false;
        bool present = false;
        for (size_t c = 0; c < count; ++c) present |= signals[c] >= 0;
        if (!present) return true;
        for (; next < end; ++next) {
            const uint32_t n = next;
            if (!load_record(n)) return false;
            if (artifact) {
                const uint32_t started = job->diagnostics ? micros() : 0;
                if (!artifact->record(*header, record, header->record_bytes)) {
                    note_issue(*job, artifact->error(), path);
                    return false;
                }
                if (job->diagnostics) job->tiles_us += uint32_t(micros() - started);
            }
            for (size_t c = 0; c < count; ++c) {
                if (signals[c] < 0) continue;
                ReportEdf::RecordSamples samples;
                // Commit all histogram channels with the producer's whole record.
                // No SD access or cancellation point splits this bounded update.
                if (!ReportEdf::record_samples(*header, record, header->record_bytes, signals[c], samples) ||
                    !consume(c, uint64_t(n) * samples.signal->samples_per_record, samples)) return false;
            }
        }
        return true;
    }

};

double chart_scale(size_t c, const ReportEdf::Signal &signal);

bool artifact_identity(const IndexedSession &entry, uint8_t kind, char (&prefix)[96], uint8_t (&key)[32]) {
    snprintf(prefix, sizeof(prefix), "%s/%s/%s_%s", CACHE_DIR, entry.entry.therapy_day,
             entry.entry.file_prefix, SUFFIXES[kind]);
    if (rebuild_generation) snprintf(prefix + strlen(prefix), sizeof(prefix) - strlen(prefix),
        ".g%lu", static_cast<unsigned long>(rebuild_generation));
    uint8_t identity[20] = {};
    SdStorage::put_le16(identity, ReportArtifact::VERSION);
    identity[2] = kind;
    identity[3] = (entry.key.checksums >> kind) & 1;
    SdStorage::put_le32(identity + 4, uint32_t(entry.key.sizes[kind]));
    SdStorage::put_le32(identity + 8, uint32_t(entry.key.sizes[kind] >> 32));
    memcpy(identity + 12, entry.key.sidecars[kind], 8);
    return ReportArtifact::fingerprint(identity, sizeof(identity), key);
}

struct ArtifactBuild {
    Input &input;
    std::unique_ptr<ReportArtifact::Producer> &producer;
    bool failed = false;

    ArtifactBuild(Input &input, const IndexedSession &entry, uint8_t kind, bool enabled,
                  std::unique_ptr<ReportArtifact::Producer> &producer) : input(input), producer(producer) {
        if (!enabled) return;
        if (producer) {
            if (producer->resume(input.session)) input.artifact = producer.get();
            else { failed = true; note_issue(*input.job, producer->error(), input.path); }
            return;
        }
        char prefix[96];
        uint8_t key[32];
        if (!artifact_identity(entry, kind, prefix, key)) {
            failed = true; note_issue(*input.job, "artifact_digest"); return;
        }
        ReportArtifact::Publication existing;
        const auto lookup = ReportArtifact::lookup(input.session, prefix, key, existing);
        if (lookup == ReportArtifact::Lookup::Ready) return;
        if (lookup != ReportArtifact::Lookup::Missing) {
            failed = true;
            note_issue(*input.job, lookup == ReportArtifact::Lookup::Invalid ? "artifact_invalid" : "artifact_lookup", prefix);
            return;
        }
        ReportArtifact::Selection selection[ReportArtifact::MAX_TRACKS];
        size_t count = 0;
        for (size_t c = 0; c <= CHANNEL_COUNT; ++c) {
            if ((c ? CHANNELS[c - 1].file : 0) != kind) continue;
            const int signal = ReportEdf::find_signal(*input.header, c ? CHANNELS[c - 1].signal : "Flow.40ms");
            if (signal < 0) continue;
            selection[count++] = {uint16_t(c ? CHANNELS[c - 1].artifact_id : SignalId::Flow),
                uint16_t(signal), chart_scale(c, input.header->signals[signal]),
                c ? CHANNELS[c - 1].id : "Flow", c ? CHANNELS[c - 1].unit : "L/min"};
        }
        if (!count) return;
        producer.reset(new (std::nothrow) ReportArtifact::Producer(input.session, [](void *context) {
            const auto &input = *static_cast<Input *>(context);
            return aborted(input.job, input.session);
        }, &input));
        if (!producer || !producer->begin(prefix, key, *input.header, selection, count)) {
            failed = true;
            note_issue(*input.job, producer ? producer->error() : "artifact_alloc", prefix);
            producer.reset();
            return;
        }
        input.artifact = producer.get();
    }
    ~ArtifactBuild() { input.artifact = nullptr; }
    bool needed() const { return producer != nullptr; }
    bool finish() {
        input.artifact = nullptr;
        ReportArtifact::Publication publication;
        const uint32_t started = input.job->diagnostics ? micros() : 0;
        const bool complete = !producer || producer->finish(publication);
        if (!complete && !aborted(input.job, input.session))
            note_issue(*input.job, producer->error(), input.path);
        if (input.job->diagnostics) input.job->tiles_us += uint32_t(micros() - started);
        if (complete) {
            Log::ScopedMicros measured(input.job->cleanup_us, input.job->diagnostics);
            producer.reset();
        }
        return complete;
    }
};

struct StrLayoutCache {
    ReportEdf::Layout layout;
    ReportEdf::Signal *signals = nullptr;
    uint64_t bytes = 0;

    void reset() {
        aircannect::Memory::free(signals);
        signals = nullptr;
        layout = {};
        bytes = 0;
    }
    bool load(const ReportEdf::Reader &reader) {
        if (signals && bytes == reader.size) return true;
        reset();
        if (!ReportEdf::read_layout(reader, layout) || layout.record_seconds != 86400 ||
            layout.records > Air10StrTimeline::RECORD_LIMIT) return false;
        const size_t size = layout.signal_count * sizeof(*signals);
        signals = static_cast<ReportEdf::Signal *>(aircannect::Memory::alloc_large(size, size <= INTERNAL_WORK));
        if (!signals || !ReportEdf::read_signals(reader, layout, signals, layout.signal_count, true) ||
            layout.record_bytes > 4096) { reset(); return false; }
        for (size_t i = 0; i < layout.signal_count; ++i) {
            const auto &date = signals[i];
            if (strcmp(date.label, "Date")) continue;
            if (date.samples_per_record == 1 && date.digital_min == 0 &&
                date.physical_min == 0 && date.physical_max == date.digital_max) {
                bytes = reader.size;
                return true;
            }
            break;
        }
        reset();
        return false;
    }
} str_layout;

void invalidate_str() { str_layout.reset(); }

struct HistoricalStr {
    Input input;
    const ReportEdf::Signal *signals = nullptr;
    bool present = false;

    HistoricalStr(const SdStorage::Session &session, const Job &job) : input(session, job) {}

    const ReportEdf::Signal *find(const char *label) const {
        if (!signals || !input.header) return nullptr;
        for (size_t i = 0; i < input.header->signal_count; ++i)
            if (!strcmp(signals[i].label, label)) return &signals[i];
        return nullptr;
    }
    double value(const char *label, double scale = 1) const {
        const auto *signal = find(label);
        if (!present || !signal || signal->samples_per_record != 1) return NAN;
        const uint8_t *sample = input.record + signal->record_offset;
        const uint16_t raw = sample[0] | uint16_t(sample[1]) << 8;
        if (raw == UINT16_MAX) return NAN;
        const int32_t digital = signal->digital_min < 0 ? int16_t(raw) : int32_t(raw);
        return signal->valid(digital) ? signal->physical(digital) * scale : NAN;
    }
    bool load(uint16_t day) {
        if (!input.open_file("/STR.edf") || !str_layout.load(input.reader)) return false;
        auto &header = *input.header;
        static_cast<ReportEdf::Layout &>(header) = str_layout.layout;
        signals = str_layout.signals;
        if (!input.prepare_record()) return false;
        const auto *date = find("Date");
        const int64_t guessed = int64_t(day) - header.start_seconds / 86400;
        auto read = [&](uint32_t record) {
            uint8_t bytes[2];
            return input.reader.read(input.reader.context,
                header.header_bytes + uint64_t(record) * header.record_bytes + date->record_offset,
                bytes, sizeof(bytes));
        };
        // The current timeline is continuous. Older compact files are located
        // by their Date samples, never by substituting a neighbouring record.
        if (guessed >= 0 && guessed < header.records) {
            if (!read(uint32_t(guessed))) return false;
            present = true;
            if (value("Date") == day) {
                present = std::isfinite(value("Mode")) || std::isfinite(value("Duration"));
                input.file.close(); return true;
            }
        }
        for (uint32_t record = 0; record < header.records; ++record) {
            if (!read(record)) return false;
            present = true;
            if (value("Date") == day) {
                present = std::isfinite(value("Mode")) || std::isfinite(value("Duration"));
                input.file.close(); return true;
            }
        }
        present = false;
        input.file.close();
        return true;
    }
    void settings_json(Payload &body, uint16_t day) const {
        body.print(",\"settings\":{\"day\":%u,\"available\":%s,\"fields\":[", day, present ? "true" : "false");
        bool comma = false;
        auto append = [&](const char *tag) {
            const auto *def = var_lookup(tag);
            if (!def) return;
            const auto &schema = Air10Edf::str_schema();
            for (size_t i = 0; i < schema.signal_count; ++i) {
                if (strcmp(Air10Edf::str_signal_tag(i), tag)) continue;
                const auto &spec = schema.signals[i];
                const auto *signal = find(spec.label);
                const double saved = value(spec.label);
                if (!std::isfinite(saved) || !signal || strcmp(signal->unit, spec.dimension)) return;
                char text[48];
                const bool option = def->type == SET_ENUM && saved >= 0 && saved <= INT32_MAX &&
                    saved == std::floor(saved) && setting_enum_label(def, int(saved), text, sizeof(text));
                if (!option)
                    snprintf(text, sizeof(text), "%.8g", saved);
                // Text and labels come only from the static Air10 catalog or
                // numeric formatting, never live metadata or file strings.
                body.print("%s{\"tag\":\"%s\",\"label\":\"%s\",\"value\":\"%s\",\"unit\":\"%s\"",
                           comma ? "," : "", tag, def->label, text, spec.dimension);
                if (option) body.print(",\"option\":%d", int(saved));
                body.print("}");
                comma = true;
                return;
            }
        };
        append("MOP");
        const double mode = value("Mode");
        if (std::isfinite(mode) && mode >= 0 && mode < MODE_COUNT && mode == std::floor(mode)) {
            for (const char *tag : MODE_LAYOUT[int(mode)]) { if (!tag) break; append(tag); }
            for (const char *tag : COMFORT_LAYOUT[int(mode)]) { if (!tag) break; append(tag); }
        }
        for (const char *tag : EPR_VARS) { if (!tag) break; append(tag); }
        append("SST");
        for (const char *tag : ACCESSORY_VARS) { if (!tag) break; append(tag); }
        body.print("]}");
    }
    double statistic(size_t channel, const char *suffix) const {
        const char *base = channel == 0 ? "TgtIPAP" : channel == 1 ? "TgtEPAP" : CHANNELS[channel].id;
        char label[32];
        snprintf(label, sizeof(label), "%s.%s", base, suffix);
        const auto *signal = find(label);
        if (!signal) return NAN;
        double scale = 1;
        if (channel == 3 && !strcmp(signal->unit, "L/s")) scale = 60;
        else if (channel == 6 && !strcmp(signal->unit, "L")) scale = 1000;
        else if (channel == 12 && !strcmp(signal->unit, "seconds")) scale = 1;
        else if (strcmp(signal->unit, CHANNELS[channel].unit)) return NAN;
        return value(label, scale);
    }
};

size_t total_bins(const CacheHeader &header) {
    size_t count = 0;
    for (const auto &channel : header.channels) count += channel.bins;
    return count;
}
bool allocate_data(SessionData &out, bool events = true) {
    const size_t count = total_bins(out.header);
    if (count > MAX_BINS || out.header.events > MAX_DISK_EVENTS) return false;
    uint64_t *bins = nullptr;
    Event *event_data = nullptr;
    if (count) {
        const size_t bytes = count * sizeof(uint64_t);
        bins = static_cast<uint64_t *>(aircannect::Memory::alloc_large(bytes, bytes <= INTERNAL_WORK));
        if (!bins) return false;
        memset(bins, 0, bytes);
    }
    if (events && out.header.events) {
        const size_t bytes = std::min<size_t>(out.header.events, MAX_EVENTS) * sizeof(Event);
        event_data = static_cast<Event *>(aircannect::Memory::alloc_large(bytes, bytes <= INTERNAL_WORK));
        if (!event_data) { aircannect::Memory::free(bins); return false; }
    }
    aircannect::Memory::free(out.bins);
    aircannect::Memory::free(out.events);
    out.bins = bins;
    out.events = event_data;
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
        !std::isfinite(h.csr_seconds) || h.csr_seconds < 0 || h.csr_seconds > h.used_seconds ||
        !std::isfinite(h.leak_high_seconds) || h.leak_high_seconds < 0 || h.leak_high_seconds > 86400 ||
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
        !allocate_data(out, false)) return false;
    const size_t bin_bytes = total_bins(out.header) * sizeof(uint64_t);
    for (size_t offset = 0; offset < bin_bytes;) {
        const size_t take = std::min(bin_bytes - offset, SdStorage::READ_CHUNK_BYTES);
        if (reader.read(reinterpret_cast<uint8_t *>(out.bins) + offset, take) != take) return false;
        offset += take;
    }
    return true;
}

bool read_event(Input &input, uint32_t record, Event &event) {
    ReportEdf::Annotation annotation;
    if (!ReportEdf::read_annotation(input.reader, *input.header, record, annotation))
        return note_issue(*input.job, "annotation_record", input.path);
    event.time_ms = llround((input.header->start_seconds + annotation.onset_seconds) * 1000);
    event.duration_ms = annotation.has_duration ? llround(annotation.duration_seconds * 1000) : 0;
    event.kind = annotation.kind;
    return true;
}

struct PartGuard {
    const SdStorage::Session &session;
    const char *path;
    SdStorage::Writer writer;
    bool complete = false;
    ~PartGuard() {
        writer.close();
        if (complete) return;
        cache_usage_known = false;
        (void)session.mutate(path, [&](fs::FS &fs) { return !fs.exists(path) || fs.remove(path); });
    }
};

bool start_part(PartGuard &, const uint8_t *, size_t);
bool finish_part(PartGuard &, const char *, uint64_t);

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

bool scan_channels(Input &input, uint8_t file, SessionData &data, uint32_t &next) {
    int signals[CHANNEL_COUNT];
    size_t offsets[CHANNEL_COUNT], offset = 0;
    for (size_t c = 0; c < CHANNEL_COUNT; ++c) {
        offsets[c] = offset;
        offset += data.header.channels[c].bins;
        signals[c] = -1;
        if (CHANNELS[c].file != file || !data.header.channels[c].bins) continue;
        signals[c] = ReportEdf::find_signal(*input.header, CHANNELS[c].signal);
        if (signals[c] < 0) return false;
    }
    if (!input.scan_records(signals, CHANNEL_COUNT, next, input.header->records,
        [&](size_t c, uint64_t, const ReportEdf::RecordSamples &samples) {
            auto &channel = data.header.channels[c];
            auto hist = histogram(channel, data.bins + offsets[c]);
            const uint32_t count = samples.signal->samples_per_record;
            for (uint32_t slot = 0; slot < count; ++slot) {
                const auto sample = samples[slot];
                if (!ReportEdf::add_sample(hist, sample.raw, sample.valid)) return false;
                if (c == 3 && sample.valid && channel.signal.physical(sample.raw) > LEAK_THRESHOLD)
                    data.header.leak_high_seconds += input.header->record_seconds / count;
            }
            channel.count = hist.count;
            channel.missing = hist.missing;
            channel.sum = hist.sum;
            return true;
        })) return false;
    for (size_t c = 0; c < CHANNEL_COUNT; ++c) {
        if (signals[c] >= 0) {
            auto &channel = data.header.channels[c];
            channel.coverage_seconds = channel.count * input.header->record_seconds /
                channel.signal.samples_per_record;
        }
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

struct CsrPairing {
    Event start;
    bool active = false, unmatched = false;

    template <typename Sink>
    void consume(const Event &event, Sink emit) {
        if (event.kind == ReportEdf::EventKind::CsrStart) {
            if (active) { emit(start, false); unmatched = true; }
            start = event;
            active = true;
        } else if (event.kind == ReportEdf::EventKind::CsrEnd) {
            if (active && event.time_ms > start.time_ms) {
                Event interval = event;
                interval.duration_ms = event.time_ms - start.time_ms;
                emit(interval, true);
            } else {
                if (active) emit(start, false);
                emit(event, false);
                unmatched = true;
            }
            active = false;
        } else emit(event, false);
    }
};

struct CacheStore : ReportWork {
    uint64_t written = 0;
    size_t bins = 0, used = 0;
    uint32_t events = 0, record = 0;
    uint8_t kind = 3;
    Event block[32];
};

struct SessionBuild : SessionData {
    Input input;
    std::unique_ptr<ReportArtifact::Producer> producer;
    std::unique_ptr<CacheStore> cache_store;
    CsrPairing csr;
    RetainedIssue accepted_issue;
    ReportEdf::RecordScanPosition bounds_position;
    char prefix[32] = {};
    uint32_t rebuild = 0, next_record = 0, event_count = 0;
    uint8_t header_kind = 0, numeric_kind = 0, event_kind = 3, readable_numeric = 0;
    bool allocated = false, finished = false, cache_stored = false, artifacts;

    SessionBuild(const SdStorage::Session &session, const Job &job,
                 const IndexedSession &entry, bool artifacts)
        : input(session, job), rebuild(rebuild_generation), artifacts(artifacts) {
        snprintf(prefix, sizeof(prefix), "%s", entry.entry.file_prefix);
        header.key = entry.key;
        header.day = entry.day;
        header.partial = entry.key.present != 31 || entry.key.checksums != 31;
    }
    bool matches_source(const IndexedSession &entry) const {
        return header.key == entry.key && header.day == entry.day &&
            !strcmp(prefix, entry.entry.file_prefix) && rebuild == rebuild_generation;
    }
    bool matches(const IndexedSession &entry, bool charts) const {
        return matches_source(entry) && artifacts == charts;
    }
    void pause() {
        if (producer) producer->pause();
        input.file.close();
        input.artifact = nullptr;
    }
};

bool store_cache(const SdStorage::Session &session, const Job &job, const IndexedSession &entry,
                 SessionBuild &data) {
    if (data.header.events > MAX_DISK_EVENTS) return false;
    char path[96], part[96];
    cache_path(entry, "stats", path);
    cache_path(entry, "stats.part", part);
    data.header.bytes = sizeof(data.header) + total_bins(data.header) * sizeof(uint64_t) +
        data.header.events * sizeof(Event);
    if (!data.cache_store) {
        // Statistics have one private prefix, independent of chart production.
        // Only the worker transfers its cursor, with all handles already closed.
        xSemaphoreTake(mutex, portMAX_DELAY);
        for (auto &other : jobs) {
            if (&other == &job || other.state != JobState::Queued || !other.build ||
                !other.build->finished || other.build->cache_stored || !other.build->matches_source(entry)) continue;
            if (other.build->cache_store) {
                data.cache_store = std::move(other.build->cache_store);
                break;
            }
        }
        xSemaphoreGive(mutex);
        if (!data.cache_store) {
            // Another selector may have completed the transferred prefix.
            // Use the existing cache validator; keep our accepted RAM counters.
            std::unique_ptr<SessionData> published(new (std::nothrow) SessionData);
            if (!published) return false;
            if (load_cache(session, entry, *published)) return true;
            data.cache_store.reset(new (std::nothrow) CacheStore);
        }
    }
    if (!data.cache_store) return false;
    auto &work = *data.cache_store;
    PartGuard part_guard{session, part};
    part_guard.complete = work.written != 0;
    if (!work.written) {
        if (!artifact_headroom(session, entry.day, job) ||
            !start_part(part_guard, reinterpret_cast<const uint8_t *>(&data.header), sizeof(data.header))) return false;
        work.written = sizeof(data.header);
    } else if (!part_guard.writer.open(session, part, work.written, true)) return false;
    // Keep the committed private prefix across turns; stale/revoked writes may
    // leave a tail, which append admission truncates to this RAM cursor.
    part_guard.complete = true;
    const size_t bin_bytes = total_bins(data.header) * sizeof(uint64_t);
    while (work.bins < bin_bytes) {
        if (aborted(&job, session)) return false;
        const size_t take = std::min(bin_bytes - work.bins, SdStorage::WRITE_CHUNK_BYTES);
        if (!part_guard.writer.write(reinterpret_cast<const uint8_t *>(data.bins) + work.bins, take)) return false;
        work.bins += take;
        work.written += take;
    }
    if (data.header.events <= MAX_EVENTS) {
        while (work.events < data.header.events) {
            if (aborted(&job, session)) return false;
            const size_t count = std::min<size_t>(32, data.header.events - work.events);
            const size_t bytes = count * sizeof(Event);
            if (!part_guard.writer.write(reinterpret_cast<const uint8_t *>(data.events + work.events), bytes)) return false;
            work.events += count;
            work.written += bytes;
        }
    } else {
        // The bounded RAM prefix cannot hold all annotations. Retain the replay
        // cursor and one output block, never restart it on a short SD grant.
        auto &input = data.input;
        input.session = session;
        for (; work.kind < 5; ++work.kind, work.record = 0) {
            if (!(entry.key.present & (1u << work.kind))) continue;
            if (!input.open(session, entry, work.kind)) return false;
            while (work.record < input.header->records || work.used == 32) {
                if (aborted(&job, session)) return false;
                if (work.used == 32) {
                    if (!part_guard.writer.write(reinterpret_cast<uint8_t *>(work.block), sizeof(work.block))) return false;
                    work.written += sizeof(work.block);
                    work.used = 0;
                }
                if (work.record == input.header->records) break;
                Event event;
                if (!read_event(input, work.record, event)) return false;
                ++work.record;
                if (event.kind == ReportEdf::EventKind::Unknown) continue;
                work.block[work.used++] = event;
                ++work.events;
            }
            input.file.close();
        }
        if (work.events != data.header.events) return false;
        if (work.used) {
            const size_t bytes = work.used * sizeof(Event);
            if (!part_guard.writer.write(reinterpret_cast<uint8_t *>(work.block), bytes)) return false;
            work.written += bytes;
            work.used = 0;
        }
    }
    if (work.written != data.header.bytes || !finish_part(part_guard, path, data.header.bytes)) return false;
    invalidate_day(entry.day);
    __atomic_add_fetch(&cache_generation, 1, __ATOMIC_RELEASE);
    return true;
}

bool build_session(const SdStorage::Session &session, const Job &job,
                   const IndexedSession &entry, SessionBuild &work) {
    SourceIssueScope issues(job, work.accepted_issue);
    if (work.finished) return true;
    auto &out = static_cast<SessionData &>(work);
    auto &input = work.input;
    input.session = session;
    const bool artifacts = work.artifacts;
    // Gather descriptors before allocating the bounded exact histograms.
    for (; work.header_kind < 3; ++work.header_kind) {
        const uint8_t kind = work.header_kind;
        if (!(entry.key.present & (1u << kind))) continue;
        if (!input.open(session, entry, kind)) {
            if (aborted(&job, session)) return false;
            out.header.partial = true;
            work.accepted_issue.capture(job);
            input.file.close();
            continue;
        }
        if (kind && !prepare_channels(input, kind, out)) return false;
        work.readable_numeric |= 1u << kind;
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
                if (!ReportEdf::recorded_bounds(input.reader, *input.header, coverage, bounds, &work.bounds_position))
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
    for (; work.header_kind < 5; ++work.header_kind) {
        const uint8_t kind = work.header_kind;
        if (!(entry.key.present & (1u << kind))) continue;
        if (!input.open(session, entry, kind)) {
            if (aborted(&job, session)) return false;
            out.header.partial = true;
            work.accepted_issue.capture(job);
            input.file.close();
            continue;
        }
        if (kind == 3) out.header.events_known = true;
        out.header.events += std::min<uint32_t>(input.header->records, MAX_EVENTS - out.header.events);
        input.file.close();
    }
    if (!work.allocated) {
        if (!allocate_data(out)) return false;
        work.allocated = true;
    }
    for (; work.numeric_kind < 3; ++work.numeric_kind, work.next_record = 0) {
        const uint8_t kind = work.numeric_kind;
        if (!(work.readable_numeric & (1u << kind))) continue;
        if (!kind && !artifacts) continue;
        if (!input.open(session, entry, kind)) return false;
        ArtifactBuild artifact(input, entry, kind, artifacts, work.producer);
        if (artifact.failed) return false;
        if (kind) {
            if (!scan_channels(input, kind, out, work.next_record)) return false;
        } else if (artifact.needed()) {
            const int flow = ReportEdf::find_signal(*input.header, "Flow.40ms");
            if (!input.scan_records(&flow, 1, work.next_record, input.header->records,
                [](size_t, uint64_t, const ReportEdf::RecordSamples &) { return true; })) return false;
        }
        input.file.close();
        if (!artifact.finish()) return false;
    }
    const uint32_t event_capacity = out.header.events;
    auto &events = work.event_count;
    auto &csr = work.csr;
    for (; work.event_kind < 5; ++work.event_kind, work.next_record = 0) {
        const uint8_t kind = work.event_kind;
        if (!(entry.key.present & (1u << kind))) continue;
        if (!input.open(session, entry, kind)) {
            if (aborted(&job, session)) return false;
            if (kind == 3) out.header.events_known = false;
            out.header.partial = true;
            work.accepted_issue.capture(job);
            input.file.close();
            continue;
        }
        if (kind == 4) out.header.csr_known = true;
        for (; work.next_record < input.header->records; ++work.next_record) {
            if (aborted(&job, session)) return false;
            Event event;
            if (!read_event(input, work.next_record, event)) {
                if (aborted(&job, session)) return false;
                if (kind == 3) out.header.events_known = false;
                if (kind == 4) out.header.csr_known = false;
                out.header.partial = true;
                work.accepted_issue.capture(job);
                break;
            }
            if (event.kind == ReportEdf::EventKind::Unknown) continue;
            if (kind == 4) csr.consume(event, [&](const Event &interval, bool paired) {
                if (!paired || !out.header.recorded) return;
                const int64_t begin = std::max(out.header.start_ms, interval.time_ms - interval.duration_ms);
                const int64_t end = std::min(out.header.end_ms, interval.time_ms);
                if (end > begin) out.header.csr_seconds += (end - begin) / 1000.0;
            });
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
    out.header.csr_known &= !csr.active && !csr.unmatched && out.header.usage_known;
    if (events) std::sort(out.events, out.events + std::min<uint32_t>(events, event_capacity), [](const Event &a, const Event &b) {
        return a.time_ms < b.time_ms;
    });
    work.finished = true;
    return true;
}

bool session_data(const SdStorage::Session &session, const Job &job,
                  const IndexedSession &entry, SessionData &out, bool artifacts = false) {
    if (!artifacts && !(job.build && job.build->matches(entry, artifacts)) && load_cache(session, entry, out)) {
        if (out.header.partial) note_issue(job, "partial_source", entry.entry.file_prefix);
        job.accepted_issue.capture(job);
        return true;
    }
    if (job.build && !job.build->matches(entry, artifacts)) job.build.reset();
    if (!job.build) {
        // One source's private spool has one owner even when a foreground
        // request overtakes a queued backfill with a different selector.
        xSemaphoreTake(mutex, portMAX_DELAY);
        for (auto &other : jobs) {
            if (&other != &job && other.state == JobState::Queued && other.build &&
                other.build->matches(entry, artifacts)) {
                job.build = std::move(other.build);
                job.build->input.job = &job;
                break;
            }
        }
        xSemaphoreGive(mutex);
    }
    if (!job.build) job.build.reset(new (std::nothrow) SessionBuild(session, job, entry, artifacts));
    if (!job.build) return note_issue(job, "session_alloc");
    if (!build_session(session, job, entry, *job.build)) {
        job.build->pause();
        return note_issue(job, "session_build", entry.entry.file_prefix);
    }
    auto &built = *job.build;
    if (built.header.partial) note_issue(job, "partial_source", entry.entry.file_prefix);
    // The cache is derived and replaceable; failed persistence does not discard
    // useful parsed data or modify any source EDF/sidecar.
    if (!built.header.partial && !built.cache_stored) {
        built.cache_stored = store_cache(session, job, entry, built);
        if (!built.cache_stored && !aborted(&job, session))
            note_issue(job, "cache_write", entry.entry.file_prefix);
    }
    if (aborted(&job, session)) { built.pause(); return false; }
    job.accepted_issue.capture(job);
    out.header = built.header;
    std::swap(out.bins, built.bins);
    std::swap(out.events, built.events);
    job.build.reset();
    return true;
}

uint16_t last_day(const Request &request) {
    if (request.day) return request.day;
    uint16_t day = 0;
    for (size_t i = 0; i < index_count; ++i)
        if (index[i].entry.flags & EdfCatalog::ENTRY_LIVE_COMPLETE)
            day = std::max(day, index[i].day);
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

bool summary_json(const SdStorage::Session &session, Job &job);
bool history_json(const SdStorage::Session &session, Job &job, bool build);
bool window_events(const SdStorage::Session &session, const Job &job,
                   const IndexedSession &entry, const CacheHeader &header,
                   int64_t from, int64_t to, Payload &out, bool &comma, bool &truncated,
                   uint32_t &next, CsrPairing &csr);

bool dataset(const SdStorage::Session &session) {
    if (*dataset_id) return true;
    constexpr const char *prefix = "/airbridge/report/dataset";
    ReportArtifact::Document document;
    Payload bytes;
    const auto found = ReportArtifact::document_lookup(session, prefix, nullptr, document);
    if (found == ReportArtifact::Lookup::Missing) {
        uint8_t id[32], key[32] = {};
        esp_fill_random(id, sizeof(id));
        if (!ReportArtifact::document_publish(session, prefix, key, id, sizeof(id), document)) return false;
    } else if (found != ReportArtifact::Lookup::Ready) return false;
    if (!ReportArtifact::document_read(session, prefix, document, bytes, nullptr) || bytes.length() != 32) return false;
    ReportArtifact::Document rebuild;
    const auto generation = ReportArtifact::document_lookup(session, "/airbridge/report/rebuild", nullptr, rebuild);
    rebuild_generation = 0;
    if (generation == ReportArtifact::Lookup::Ready) {
        Payload value;
        if (!ReportArtifact::document_read(session, "/airbridge/report/rebuild", rebuild, value, nullptr) || value.length() != 4) return false;
        rebuild_generation = SdStorage::get_le32(reinterpret_cast<const uint8_t *>(value.c_str()));
    } else if (generation != ReportArtifact::Lookup::Missing) return false;
    return aircannect::hex_encode(reinterpret_cast<const uint8_t *>(bytes.c_str()), 32,
        dataset_id, sizeof(dataset_id), aircannect::HexCase::Lower);
}

void publication_prefix(uint16_t day, char (&path)[96], uint16_t period = 0) {
    const time_t civil = int64_t(day) * 86400;
    tm date = {};
    gmtime_r(&civil, &date);
    char directory[9];
    strftime(directory, sizeof(directory), "%Y%m%d", &date);
    if (period) snprintf(path, sizeof(path), "%s/%s/period-%u", CACHE_DIR, directory, unsigned(period));
    else snprintf(path, sizeof(path), "%s/%s/day", CACHE_DIR, directory);
}

void period_prefix(const Request &request, char (&path)[96]) {
    publication_prefix(last_day(request), path, request.period);
}

bool publication_key(const Request &request, uint8_t (&key)[32]) {
    Payload identity;
    const uint16_t last = last_day(request);
    const uint16_t first = request.view == View::Day ? (last > 13 ? last - 13 : 0) : first_day(request, last);
    identity.print("publication-1:%s:%u:%u:%u:%lu", dataset_id, first, last, request.period,
                   static_cast<unsigned long>(rebuild_generation));
    for (size_t i = 0; i < index_count; ++i) {
        const auto &entry = index[i];
        if (!selected(entry, first, last)) continue;
        identity.print("/%u:%s:%s:%u:%u:%u:%u", unsigned(i), entry.entry.therapy_day,
            entry.entry.file_prefix, entry.entry.flags, entry.entry.str_digest,
            entry.key.present, entry.key.checksums);
        for (size_t k = 0; k < 5; ++k) {
            identity.print(":%llu:", static_cast<unsigned long long>(entry.key.sizes[k]));
            identity.append(reinterpret_cast<const char *>(entry.key.sidecars[k]), 8);
        }
    }
    return identity.good() && ReportArtifact::fingerprint(
        reinterpret_cast<const uint8_t *>(identity.c_str()), identity.length(), key);
}

bool artifact_headroom(const SdStorage::Session &session, uint16_t day, const Job &job) {
    Log::ScopedMicros measured(job.headroom_us, job.diagnostics);
    if (job.diagnostics) ++job.headroom_calls;
    SdStorage::Status storage;
    if (!session.run([](fs::FS &) { SdStorage::refresh_usage(); return true; })) return false;
    SdStorage::get_status(storage);
    uint64_t needed = 128 * 1024;
    for (size_t i = 0; i < index_count; ++i) if (selected(index[i], day, day))
        for (size_t kind = 0; kind < 3; ++kind) needed += index[i].key.sizes[kind] * 3;
    // Preserve room for another complete 24h numeric recording in the largest
    // supported schema, plus STR. Artifacts are optional, source EDF is not.
    const uint64_t reserve = 1440ull * (Air10Edf::record_size(Air10Edf::brp_schema(true)) +
        Air10Edf::record_size(Air10Edf::pld_schema()) + Air10Edf::record_size(Air10Edf::sad_schema())) +
        366ull * Air10Edf::record_size(Air10Edf::str_schema()) + 65536;
    return storage.total_bytes > storage.used_bytes && storage.total_bytes - storage.used_bytes >= needed + reserve;
}

bool prepare_period(const SdStorage::Session &session, Job &job) {
    if (!dataset(session)) return note_issue(job, "dataset_identity");
    char prefix[96]; uint8_t key[32];
    period_prefix(job.request, prefix);
    if (!publication_key(job.request, key)) return note_issue(job, "publication_key");
    ReportArtifact::Document document;
    if (ReportArtifact::document_lookup(session, prefix, key, document) == ReportArtifact::Lookup::Ready)
        return ReportArtifact::document_read(session, prefix, document, job.body, key);
    if (!summary_json(session, job) || job.issue || aborted(&job, session, false)) return false;
    EdfCatalog::Status current;
    EdfCatalog::get_status(current);
    if (current.generation != catalog_generation || SdStorage::files_revision(SdStorage::FileScope::Reports) != file_revision) return false;
    if (!ReportArtifact::document_publish(session, prefix, key,
        reinterpret_cast<const uint8_t *>(job.body.c_str()), job.body.length(), document))
        return note_issue(job, "publication_write", prefix);
    remember_publication(last_day(job.request), job.request.period, document);
    portENTER_CRITICAL(&status_mux);
    ++status.artifact_revision;
    ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    return true;
}

bool publish_histograms(const SdStorage::Session &session, const IndexedSession &entry,
                        const SessionData &data, ReportArtifact::Document &document) {
    Payload bytes;
    uint8_t header[16] = {};
    memcpy(header, "ABHIST01", 8);
    SdStorage::put_le16(header + 8, 1);
    SdStorage::put_le16(header + 10, CHANNEL_COUNT);
    const uint32_t length = sizeof(header) + CHANNEL_COUNT * 72 + total_bins(data.header) * 8;
    SdStorage::put_le32(header + 12, length);
    if (!bytes.reserve(length) || !bytes.append(reinterpret_cast<const char *>(header), sizeof(header))) return false;
    const auto u64 = [](uint8_t *out, uint64_t value) {
        SdStorage::put_le32(out, uint32_t(value));
        SdStorage::put_le32(out + 4, uint32_t(value >> 32));
    };
    const auto real = [&](uint8_t *out, double value) {
        static_assert(sizeof(value) == sizeof(uint64_t), "binary64 histogram values");
        uint64_t bits; memcpy(&bits, &value, sizeof(bits)); u64(out, bits);
    };
    size_t offset = 0;
    for (size_t c = 0; c < CHANNEL_COUNT; ++c) {
        const auto &channel = data.header.channels[c];
        uint8_t descriptor[72] = {};
        SdStorage::put_le16(descriptor, uint16_t(CHANNELS[c].artifact_id));
        SdStorage::put_le32(descriptor + 4, channel.bins);
        SdStorage::put_le32(descriptor + 8, uint32_t(channel.signal.digital_min));
        SdStorage::put_le32(descriptor + 12, uint32_t(channel.signal.digital_max));
        real(descriptor + 16, channel.signal.physical_min);
        real(descriptor + 24, channel.signal.physical_max);
        u64(descriptor + 32, channel.count); u64(descriptor + 40, channel.missing);
        real(descriptor + 48, channel.sum); real(descriptor + 56, channel.coverage_seconds);
        SdStorage::put_le32(descriptor + 64, channel.signal.samples_per_record);
        if (!bytes.append(reinterpret_cast<const char *>(descriptor), sizeof(descriptor))) return false;
        for (uint32_t bin = 0; bin < channel.bins; ++bin) {
            uint8_t value[8]; u64(value, data.bins[offset++]);
            if (!bytes.append(reinterpret_cast<const char *>(value), sizeof(value))) return false;
        }
    }
    char prefix[96]; uint8_t key[32];
    cache_path(entry, "histograms", prefix);
    return bytes.length() == length && ReportArtifact::fingerprint(
        reinterpret_cast<const uint8_t *>(bytes.c_str()), bytes.length(), key) &&
        ReportArtifact::document_publish(session, prefix, key,
            reinterpret_cast<const uint8_t *>(bytes.c_str()), bytes.length(), document);
}

struct PrepareWork : ReportWork {
    enum Phase { Numeric, Events, History, Summary, Publish } phase = Numeric;
    size_t next = 0;
    bool comma = false, loaded = false, numeric_missing = false, event_comma = false, events_ready = false;
    uint32_t next_event = 0;
    CsrPairing csr;
    Payload envelope, events;
    std::unique_ptr<SessionData> data;
    ReportArtifact::Document histogram;
};

bool prepare_json(const SdStorage::Session &session, Job &job) {
    if (job.request.view == View::Period) return prepare_period(session, job);
    const uint16_t day = last_day(job.request);
    if (!dataset(session)) return note_issue(job, "dataset_identity");
    char manifest[96], identity[65]; uint8_t manifest_key[32];
    publication_prefix(day, manifest);
    if (!publication_key(job.request, manifest_key)) return note_issue(job, "publication_key");
    ReportArtifact::Document saved;
    if (!job.preparation) {
        const auto known = ReportArtifact::document_lookup(session, manifest, manifest_key, saved);
        if (known == ReportArtifact::Lookup::Ready) {
            // Revalidate references only after a source/storage change or eviction.
            // scan_index clears this owner cache when files_revision changes.
            auto *cached = cached_day_entry(day, true);
            const bool checked = cached && cached->artifacts_checked &&
                !memcmp(cached->artifact_key, manifest_key, sizeof(manifest_key));
            for (size_t i = 0; !checked && i < index_count; ++i) {
                const auto &entry = index[i];
                if (!selected(entry, day, day)) continue;
                for (uint8_t kind = 0; kind < 3; ++kind) {
                    if (!(entry.key.present & (1u << kind))) continue;
                    char prefix[96]; uint8_t key[32];
                    ReportArtifact::Publication publication;
                    if (!artifact_identity(entry, kind, prefix, key) ||
                        ReportArtifact::lookup(session, prefix, key, publication) != ReportArtifact::Lookup::Ready)
                        return note_issue(job, "artifact_lookup", entry.entry.file_prefix);
                }
            }
            if (cached) {
                memcpy(cached->artifact_key, manifest_key, sizeof(manifest_key));
                cached->artifacts_checked = true;
            }
            if (ReportArtifact::document_read(session, manifest, saved, job.body, manifest_key)) return true;
            return note_issue(job, "publication_read", manifest);
        }
        if (known == ReportArtifact::Lookup::Failed || known == ReportArtifact::Lookup::Invalid)
            return note_issue(job, "publication_read", manifest);
        if (!artifact_headroom(session, day, job)) return note_issue(job, "recording_headroom");
        job.preparation.reset(new (std::nothrow) PrepareWork);
        if (!job.preparation) return note_issue(job, "artifact_alloc");
        job.body.clear();
        aircannect::hex_encode(manifest_key, 32, identity, sizeof(identity), aircannect::HexCase::Lower);
        start_json(job.body, revision() + 1, job.data_revision);
        job.body.print("\"dataset\":\"%s\",\"identity\":\"%s\",\"day\":%u,\"artifacts\":[", dataset_id, identity, day);
    }
    auto &work = *job.preparation;
    auto &comma = work.comma;
    if (work.phase == PrepareWork::Numeric) {
        Log::ScopedMicros measured(job.prepare_numeric_us, job.diagnostics);
        for (size_t &i = work.next; i < index_count; ++i) {
            const auto &entry = index[i];
            if (!selected(entry, day, day)) continue;
            if (aborted(&job, session)) return false;
            if (!work.loaded) {
                work.numeric_missing = false;
                for (uint8_t kind = 0; kind < 3; ++kind) {
                    if (!(entry.key.present & (1u << kind))) continue;
                    char prefix[96]; uint8_t key[32];
                    ReportArtifact::Publication publication;
                    if (!artifact_identity(entry, kind, prefix, key)) return note_issue(job, "artifact_digest");
                    const auto found = ReportArtifact::lookup(session, prefix, key, publication);
                    if (found == ReportArtifact::Lookup::Missing) work.numeric_missing = true;
                    else if (found != ReportArtifact::Lookup::Ready) return note_issue(job, "artifact_lookup", prefix);
                }
                work.loaded = true;
            }
            if (work.numeric_missing) {
                std::unique_ptr<SessionData> data(new (std::nothrow) SessionData);
                if (!data) return note_issue(job, "artifact_alloc");
                if (!session_data(session, job, entry, *data, true)) return false;
            }
            Payload references;
            bool reference_comma = comma;
            for (uint8_t kind = 0; kind < 3; ++kind) {
                if (!(entry.key.present & (1u << kind))) continue;
                char prefix[96], digest[65]; uint8_t key[32];
                ReportArtifact::Publication publication;
                if (!artifact_identity(entry, kind, prefix, key) ||
                    ReportArtifact::lookup(session, prefix, key, publication) != ReportArtifact::Lookup::Ready)
                    return note_issue(job, "artifact_unavailable", entry.entry.file_prefix);
                aircannect::hex_encode(publication.digest, 32, digest, sizeof(digest), aircannect::HexCase::Lower);
                references.print("%s{\"session\":%u,\"day\":\"%s\",\"kind\":\"%s\",\"digest\":\"%s\",\"bytes\":%u}",
                               reference_comma ? "," : "", unsigned(i), entry.entry.therapy_day, SUFFIXES[kind], digest, publication.bytes);
                reference_comma = true;
            }
            if (!references.good() || !job.body.append(references.c_str(), references.length())) return false;
            comma = reference_comma;
            work.loaded = false;
        }
        job.body.print("],\"event_groups\":[");
        comma = false;
        work.loaded = false;
        work.next = 0;
        work.phase = PrepareWork::Events;
    }
    if (work.phase == PrepareWork::Events) {
        Log::ScopedMicros measured(job.prepare_events_us, job.diagnostics);
        for (size_t &i = work.next; i < index_count; ++i) {
            const auto &entry = index[i];
            if (!selected(entry, day, day)) continue;
            if (!work.data) work.data.reset(new (std::nothrow) SessionData);
            if (!work.data) return note_issue(job, "session_alloc");
            auto &data = *work.data;
            if (!work.loaded) {
                if (!session_data(session, job, entry, data)) return false;
                work.loaded = true;
            }
            auto &histogram = work.histogram;
            if (!histogram.bytes) {
                Log::ScopedMicros histogram_time(job.prepare_histograms_us, job.diagnostics);
                ReportArtifact::Document published;
                if (!publish_histograms(session, entry, data, published))
                    return note_issue(job, "histogram_publish");
                histogram = published;
            }
            auto &events = work.events;
            if (!work.events_ready) {
                bool truncated = false;
                if (!events.length()) events.print("{\"events\":[");
                if (!window_events(session, job, entry, data.header, 0, INT64_MAX,
                    events, work.event_comma, truncated, work.next_event, work.csr) || truncated)
                    return note_issue(job, "event_limit");
                events.print("]}");
                work.events_ready = true;
            }
            char prefix[96], digest[65];
            cache_path(entry, "events", prefix);
            uint8_t event_key[32];
            ReportArtifact::Document publication;
            if (!events.good() || !ReportArtifact::fingerprint(reinterpret_cast<const uint8_t *>(events.c_str()), events.length(), event_key) ||
                !ReportArtifact::document_publish(session, prefix, event_key,
                    reinterpret_cast<const uint8_t *>(events.c_str()), events.length(), publication)) return note_issue(job, "events_publish");
            aircannect::hex_encode(publication.digest, 32, digest, sizeof(digest), aircannect::HexCase::Lower);
            job.body.print("%s{\"session\":%u,\"day\":\"%s\",\"digest\":\"%s\",\"bytes\":%u,",
                comma ? "," : "", unsigned(i), entry.entry.therapy_day, digest, publication.bytes);
            aircannect::hex_encode(histogram.digest, 32, digest, sizeof(digest), aircannect::HexCase::Lower);
            job.body.print("\"histogram\":{\"day\":\"%s\",\"digest\":\"%s\",\"bytes\":%u}}",
                entry.entry.therapy_day, digest, histogram.bytes);
            comma = true;
            work.data.reset();
            work.loaded = work.event_comma = work.events_ready = false;
            work.histogram = {};
            work.next_event = 0;
            work.csr = {};
            work.events.~Payload();
            new (&work.events) Payload;
        }
        job.body.print("],\"summary\":");
        work.envelope.swap(job.body);
        work.phase = PrepareWork::History;
    }
    // History uses the same exact session statistics, never browser aggregates.
    if (work.phase == PrepareWork::History) {
        Log::ScopedMicros measured(job.prepare_history_us, job.diagnostics);
        if (!history_json(session, job, true)) return false;
        job.body.clear();
        work.phase = PrepareWork::Summary;
    }
    if (work.phase == PrepareWork::Summary) {
        Log::ScopedMicros measured(job.prepare_summary_us, job.diagnostics);
        if (!summary_json(session, job)) return false;
        work.envelope.append(job.body.c_str(), job.body.length());
        work.envelope.print("}");
        job.body.swap(work.envelope);
        work.phase = PrepareWork::Publish;
    }
    Log::ScopedMicros publication_time(job.prepare_publish_us, job.diagnostics);
    if (!job.body.good() || aborted(&job, session, false)) return false;
    if (job.issue) {
        ReportArtifact::Document previous;
        const bool prior = ReportArtifact::document_lookup(session, manifest, nullptr, previous) == ReportArtifact::Lookup::Ready;
        if (strcmp(job.issue, "partial_source") || prior) return false;
    }
    // A catalog/STR update during production must not become the current head.
    EdfCatalog::Status current;
    EdfCatalog::get_status(current);
    if (current.generation != catalog_generation || SdStorage::files_revision(SdStorage::FileScope::Reports) != file_revision) return false;
    if (!ReportArtifact::document_publish(session, manifest, manifest_key,
        reinterpret_cast<const uint8_t *>(job.body.c_str()), job.body.length(), saved))
        return note_issue(job, "publication_write", manifest);
    remember_publication(day, 0, saved);
    if (auto *cached = cached_day_entry(day, true)) {
        memcpy(cached->artifact_key, manifest_key, sizeof(manifest_key));
        cached->artifacts_checked = true;
    }
    __atomic_add_fetch(&cache_generation, 1, __ATOMIC_RELEASE);
    portENTER_CRITICAL(&status_mux);
    ++status.artifact_revision;
    ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    return true;
}
void append_event(Payload &body, const Event &event, bool &comma, const char *type = nullptr) {
    body.print("%s{\"time_ms\":%lld,\"duration_ms\":%lld,\"type\":\"%s\"}",
               comma ? "," : "", static_cast<long long>(event.time_ms),
               static_cast<long long>(event.duration_ms), type ? type : event_name(event.kind));
    comma = true;
}

bool days_json(const SdStorage::Session &session, Job &job) {
    if (!dataset(session)) return note_issue(job, "dataset_identity");
    const size_t bytes = index_count * sizeof(uint16_t);
    auto *days = bytes ? static_cast<uint16_t *>(aircannect::Memory::alloc_large(bytes,
                                                       bytes <= INTERNAL_WORK)) : nullptr;
    if (bytes && !days) return false;
    size_t count = 0;
    bool partial = false;
    for (size_t i = 0; i < index_count; ++i) {
        if (!(index[i].entry.flags & EdfCatalog::ENTRY_LIVE_COMPLETE)) continue;
        days[count++] = index[i].day;
        if (index[i].key.present != 31 || index[i].key.checksums != 31) partial = true;
    }
    if (count) std::sort(days, days + count, std::greater<uint16_t>());
    start_json(job.body, revision() + 1, job.data_revision);
    job.body.print("\"dataset\":\"%s\",\"partial\":%s,\"days\":[", dataset_id, partial ? "true" : "false");
    for (size_t n = 0; n < count;) {
        const uint16_t day = days[n];
        size_t end = n + 1;
        while (end < count && days[end] == day) ++end;
        job.body.print("%s{\"day\":%u,\"session_count\":%u", n ? "," : "", day,
                       static_cast<unsigned>(end - n));
        char prefix[96]; uint8_t key[32];
        publication_prefix(day, prefix);
        Request request; request.day = day;
        ReportArtifact::Document document;
        if (publication_key(request, key) &&
            lookup_publication(session, day, 0, prefix, document) == ReportArtifact::Lookup::Ready) {
            char digest[65], identity[65];
            aircannect::hex_encode(document.digest, 32, digest, sizeof(digest), aircannect::HexCase::Lower);
            aircannect::hex_encode(document.key, 32, identity, sizeof(identity), aircannect::HexCase::Lower);
            job.body.print(",\"publication\":{\"day\":\"%.8s\",\"digest\":\"%s\",\"bytes\":%u},\"identity\":\"%s\",\"stale\":%s",
                prefix + strlen(CACHE_DIR) + 1, digest, document.bytes, identity, memcmp(key, document.key, 32) ? "true" : "false");
        }
        job.body.print("}");
        n = end;
        job.listed_days = n;
        observe_report_jobs();
    }
    job.body.print("],\"periods\":[");
    bool period_comma = false;
    for (uint16_t period : PERIODS) {
        Request request; request.view = View::Period; request.period = period;
        if (!count) break;
        char prefix[96]; uint8_t key[32];
        period_prefix(request, prefix);
        ReportArtifact::Document document;
        if (!publication_key(request, key) ||
            lookup_publication(session, last_day(request), period, prefix, document) != ReportArtifact::Lookup::Ready) continue;
        char digest[65];
        aircannect::hex_encode(document.digest, 32, digest, sizeof(digest), aircannect::HexCase::Lower);
        job.body.print("%s{\"period\":%u,\"day\":\"%.8s\",\"digest\":\"%s\",\"bytes\":%u,\"stale\":%s}",
            period_comma ? "," : "", period, prefix + strlen(CACHE_DIR) + 1,
            digest, document.bytes, memcmp(key, document.key, 32) ? "true" : "false");
        period_comma = true;
    }
    job.body.print("]}");
    aircannect::Memory::free(days);
    return job.body.good();
}

uint32_t foreground_wait() {
    portENTER_CRITICAL(&status_mux);
    const uint32_t elapsed = millis() - foreground_at;
    const uint32_t wait = foreground_seen && elapsed < FOREGROUND_QUIET_MS ? FOREGROUND_QUIET_MS - elapsed : 0;
    portEXIT_CRITICAL(&status_mux);
    return wait;
}

void foreground_activity() {
    portENTER_CRITICAL(&status_mux);
    foreground_at = millis();
    foreground_seen = true;
    portEXIT_CRITICAL(&status_mux);
}

bool join_active(const Request &request, uint32_t &id, bool &full) {
    uint32_t accepted = 0;
    bool promote = false;
    full = false;
    portENTER_CRITICAL(&status_mux);
    for (size_t i = 0; i < jobs.count; ++i) {
        // Unpublished slots may be allocating or destroying their Job.
        if (!jobs.retained_ids[i]) continue;
        auto &job = jobs.data[i];
        if ((job.state != JobState::Queued && job.state != JobState::Running) ||
            job.cancelled || !(job.request == request)) continue;
        if (job.consumers == UINT16_MAX) { full = true; break; }
        ++job.consumers;
        job.shared = true;
        promote = job.state == JobState::Queued && job.background;
        if (promote) {
            job.background = false;
            __atomic_add_fetch(&foreground_epoch, 1, __ATOMIC_RELEASE);
        }
        accepted = job.id;
        break;
    }
    portEXIT_CRITICAL(&status_mux);
    if (!accepted) return false;
    id = accepted;
    foreground_activity();
    if (promote) xSemaphoreGive(wake);
    return true;
}

bool cleanup_cancelled(void *context) {
    return __atomic_load_n(&artifact_reads, __ATOMIC_ACQUIRE) || foreground_wait() ||
        aborted(nullptr, *static_cast<const SdStorage::Session *>(context));
}

bool discard_index(const SdStorage::Session &session, const uint8_t digest[32]) {
    // Keep admitted artifact graphs alive. The SD mutation also serializes an
    // unlink with any overlapping stable read already admitted by storage.
    return !__atomic_load_n(&artifact_reads, __ATOMIC_ACQUIRE) && ReportArtifact::index_discard(session, digest,
        [](void *) { return __atomic_load_n(&artifact_reads, __ATOMIC_ACQUIRE) != 0; });
}

bool retire_index(const SdStorage::Session &session) {
    if (!retired_index.bytes) return true;
    if (retired_index_files != SdStorage::files_revision(SdStorage::FileScope::Reports)) {
        retired_index = {};
        return true; // External edits invalidate provenance; leave it for discovery.
    }
    if (!discard_index(session, retired_index.digest)) return false;
    retired_index = {};
    return true;
}

ReportArtifact::Lookup index_shape(const aircannect::LargeTextBuffer &body) {
    // Old root documents have no kind tag. Recognize only the exact compact
    // schema emitted by days_json; unknown/future documents are not garbage.
    aircannect::JsonAllocator allocator(false);
    JsonDocument parsed(&allocator);
    const auto error = deserializeJson(parsed, body.c_str(), body.length());
    if (error == DeserializationError::NoMemory) return ReportArtifact::Lookup::Failed;
    const auto root = parsed.as<JsonObjectConst>();
    if (error || root.size() != 7 || root["source"] != "edf" ||
        !root["revision"].is<uint32_t>() || !root["data_revision"].is<uint32_t>() ||
        !aircannect::sha256_text_valid(root["dataset"].as<const char *>()) ||
        !root["partial"].is<bool>() || !root["days"].is<JsonArrayConst>() ||
        !root["periods"].is<JsonArrayConst>() || measureJson(parsed) != body.length())
        return ReportArtifact::Lookup::Invalid;
    const auto publication = [](JsonObjectConst value) {
        const char *day = value["day"];
        return day && strlen(day) == 8 && strspn(day, "0123456789") == 8 &&
            aircannect::sha256_text_valid(value["digest"].as<const char *>()) &&
            value["bytes"].is<uint32_t>() && value["bytes"].as<uint32_t>() > 0;
    };
    for (JsonObjectConst day : root["days"].as<JsonArrayConst>()) {
        if (!day["day"].is<uint16_t>() || !day["session_count"].is<uint32_t>() ||
            (day.size() != 2 && day.size() != 5)) return ReportArtifact::Lookup::Invalid;
        if (day.size() == 5 && (day["publication"].as<JsonObjectConst>().size() != 3 ||
            !publication(day["publication"]) || !day["stale"].is<bool>() ||
            !aircannect::sha256_text_valid(day["identity"].as<const char *>())))
            return ReportArtifact::Lookup::Invalid;
    }
    if (root["periods"].size() > sizeof(PERIODS) / sizeof(PERIODS[0])) return ReportArtifact::Lookup::Invalid;
    for (JsonObjectConst period : root["periods"].as<JsonArrayConst>()) {
        const uint16_t length = period["period"];
        if (period.size() != 5 || !publication(period) || !period["stale"].is<bool>() ||
            std::find(std::begin(PERIODS), std::end(PERIODS), length) == std::end(PERIODS))
            return ReportArtifact::Lookup::Invalid;
    }
    return ReportArtifact::Lookup::Ready;
}

bool index_cleanup_pending() {
    return index_cleanup.files != SdStorage::files_revision(SdStorage::FileScope::Reports) ||
        !index_cleanup.discovered || index_cleanup.shard < 256;
}

bool cleanup_indexes(const SdStorage::Session &session) {
    auto *context = const_cast<SdStorage::Session *>(&session);
    if (cleanup_cancelled(context)) return false;
    if (index_cleanup.files != SdStorage::files_revision(SdStorage::FileScope::Reports)) {
        index_cleanup = {};
        index_cleanup.files = SdStorage::files_revision(SdStorage::FileScope::Reports);
    }
    SdStorage::Reader directory;
    SdStorage::Reader::Entry entry;
    bool end = false;
    if (!index_cleanup.discovered) {
        memset(index_cleanup.present, 0, sizeof(index_cleanup.present));
        if (!directory.open(session, CACHE_DIR, true)) {
            if (!session.run([](fs::FS &fs) { return !fs.exists(CACHE_DIR); })) return false;
            end = true;
        }
        while (!end) {
            if (cleanup_cancelled(context) || !directory.next(entry, end)) return false;
            if (end) break;
            const char *name = strrchr(entry.name, '/');
            name = name ? name + 1 : entry.name;
            if (!entry.directory || strlen(name) != 2 || strspn(name, "0123456789abcdef") != 2) continue;
            uint32_t shard;
            if (aircannect::parse_hex(name, 2, shard)) index_cleanup.present[shard / 8] |= 1u << (shard % 8);
        }
        directory.close();
        index_cleanup.discovered = true;
    }
    while (index_cleanup.shard < 256 && !(index_cleanup.present[index_cleanup.shard / 8] & (1u << (index_cleanup.shard % 8))))
        ++index_cleanup.shard;
    if (index_cleanup.shard == 256) return true;
    char path[96];
    snprintf(path, sizeof(path), "%s/%02x", CACHE_DIR, unsigned(index_cleanup.shard));
    if (!directory.open(session, path, true)) return false;
    end = false;
    while (!end) {
        if (cleanup_cancelled(context) || !directory.next(entry, end)) return false;
        if (end) break;
        const char *name = strrchr(entry.name, '/');
        name = name ? name + 1 : entry.name;
        if (entry.directory || strlen(name) != 68 || strcmp(name + 64, ".abp") ||
            strspn(name, "0123456789abcdef") != 64 || !entry.size || entry.size > MAX_BODY) continue;
        uint8_t digest[32]; size_t decoded = 0;
        if (!aircannect::hex_decode(name, 64, digest, sizeof(digest), decoded) || digest[0] != index_cleanup.shard) continue;
        aircannect::LargeTextBuffer body(MAX_BODY, 0);
        auto state = ReportArtifact::index_candidate(session, digest, entry.size, body, cleanup_cancelled, context);
        if (state == ReportArtifact::Lookup::Ready) {
            // The only four-byte root document in v1 is the rebuild counter.
            // Both live counter heads remain protected by index_candidate.
            const uint32_t generation = body.length() == 4 ?
                SdStorage::get_le32(reinterpret_cast<const uint8_t *>(body.c_str())) : 0;
            if (!generation || generation >= rebuild_generation) state = index_shape(body);
        }
        if (state == ReportArtifact::Lookup::Failed) return false;
        if (state != ReportArtifact::Lookup::Ready) continue;
        if (cleanup_cancelled(context) || !discard_index(session, digest)) return false;
    }
    ++index_cleanup.shard;
    return true;
}

bool publish_index(const SdStorage::Session &session) {
    if (retired_index.bytes && __atomic_load_n(&artifact_reads, __ATOMIC_ACQUIRE)) {
        // Readers delay collection, not publication. Rediscover the displaced
        // body in GC rather than retaining an unbounded list in RAM.
        index_cleanup.files = UINT32_MAX;
    } else if (!retire_index(session)) return false;
    const uint32_t artifacts = status.artifact_revision;
    const uint32_t data = __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
    if (published_index_data == data && published_index_artifacts == artifacts) {
        index_commit_pending = false;
        return true;
    }
    index_commit_pending = true;
    Job snapshot;
    snapshot.data_revision = data;
    if (!days_json(session, snapshot)) return false;
    ReportArtifact::Document document;
    uint8_t key[32];
    if (!ReportArtifact::fingerprint(reinterpret_cast<const uint8_t *>(snapshot.body.c_str()), snapshot.body.length(), key) ||
        !ReportArtifact::index_publish(session, key,
            reinterpret_cast<const uint8_t *>(snapshot.body.c_str()), snapshot.body.length(), document, retired_index)) {
        index_cleanup.files = UINT32_MAX; // A body may have committed without its head.
        return false;
    }
    published_index_data = data;
    published_index_artifacts = artifacts;
    retired_index_files = SdStorage::files_revision(SdStorage::FileScope::Reports);
    index_commit_pending = false;
    retire_index(session);
    return true;
}

struct Aggregate {
    ReportEdf::Histogram *channels = nullptr;
    double coverage[CHANNEL_COUNT] = {};
    bool incompatible[CHANNEL_COUNT] = {};
    Aggregate() {
        const size_t bytes = CHANNEL_COUNT * sizeof(*channels);
        channels = static_cast<ReportEdf::Histogram *>(aircannect::Memory::alloc_large(bytes, bytes <= INTERNAL_WORK));
        if (channels) for (size_t c = 0; c < CHANNEL_COUNT; ++c) new (&channels[c]) ReportEdf::Histogram;
    }
    ~Aggregate() {
        if (channels) for (size_t c = 0; c < CHANNEL_COUNT; ++c) aircannect::Memory::free(channels[c].bins);
        aircannect::Memory::free(channels);
    }
    bool merge(const SessionData &data, bool &partial) {
        // Allocate before adding any samples: a retry must merge this session
        // once even if a later channel cannot be allocated on the first pass.
        for (size_t c = 0; c < CHANNEL_COUNT; ++c) {
            const auto &cached = data.header.channels[c];
            if (!cached.bins || incompatible[c] || channels[c].bins) continue;
            const size_t bytes = cached.bins * sizeof(uint64_t);
            auto *bins = static_cast<uint64_t *>(aircannect::Memory::alloc_large(bytes,
                                                           bytes <= INTERNAL_WORK));
            if (!bins || !ReportEdf::init_histogram(cached.signal, bins,
                                                   cached.bins, channels[c])) {
                aircannect::Memory::free(bins);
                return false;
            }
        }
        size_t offset = 0;
        for (size_t c = 0; c < CHANNEL_COUNT; ++c) {
            const auto &cached = data.header.channels[c];
            if (cached.bins && !incompatible[c]) {
                auto next = histogram(cached, data.bins + offset);
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

bool history_json(const SdStorage::Session &session, Job &job, bool build);

struct SummaryWork : ReportWork {
    Aggregate aggregate;
    size_t next = 0;
    bool partial = false, session_comma = false;
    bool usage_known = true, events_known = true, csr_known = true;
    bool totals_written = false, complete = false;
    uint32_t sessions = 0;
    uint64_t counts[5] = {};
    double seconds = 0, csr_seconds = 0, leak_high_seconds = 0;
};

bool summary_json(const SdStorage::Session &session, Job &job) {
    const uint16_t last = last_day(job.request), first = first_day(job.request, last);
    if (!job.summary) {
        job.summary.reset(new (std::nothrow) SummaryWork);
        if (!job.summary) return note_issue(job, "aggregate_alloc");
        start_json(job.body, revision() + 1, job.data_revision);
        job.body.print("\"view\":\"%s\",\"day\":%u,\"first_day\":%u,\"last_day\":%u,\"sessions\":[",
                       job.request.view == View::Day ? "day" : "period", last, first, last);
    }
    auto &work = *job.summary;
    if (work.complete) return true;
    auto &aggregate = work.aggregate;
    if (!aggregate.channels) {
        job.summary.reset();
        job.body.clear();
        return note_issue(job, "aggregate_alloc");
    }
    auto &partial = work.partial;
    auto &session_comma = work.session_comma;
    auto &usage_known = work.usage_known;
    auto &events_known = work.events_known;
    auto &csr_known = work.csr_known;
    auto &sessions = work.sessions;
    auto &counts = work.counts;
    auto &seconds = work.seconds;
    auto &csr_seconds = work.csr_seconds;
    auto &leak_high_seconds = work.leak_high_seconds;
    for (size_t &i = work.next; i < index_count; ++i) {
        auto &entry = index[i];
        if (!selected(entry, first, last)) continue;
        if (aborted(&job, session)) return false;
        if (!(entry.key.present & 7)) {
            note_issue(job, "numeric_files_missing", entry.entry.file_prefix);
            job.accepted_issue.capture(job);
            partial = true;
            usage_known = events_known = csr_known = false;
            continue;
        }
        std::unique_ptr<SessionData> storage(new (std::nothrow) SessionData);
        if (!storage) return note_issue(job, "session_alloc");
        auto &data = *storage;
        if (!session_data(session, job, entry, data)) {
            if (aborted(&job, session)) return false;
            job.accepted_issue.capture(job);
            partial = true;
            usage_known = events_known = csr_known = false;
            continue;
        }
        if (!aggregate.merge(data, partial)) return false;
        partial |= data.header.partial;
        usage_known &= data.header.usage_known;
        events_known &= data.header.usage_known;
        csr_known &= data.header.csr_known;
        csr_seconds += data.header.csr_seconds;
        leak_high_seconds += data.header.leak_high_seconds;
        entry.bounds_known = data.header.chart_end_ms > data.header.chart_start_ms;
        entry.start_ms = data.header.chart_start_ms;
        entry.end_ms = data.header.chart_end_ms;
        entry.event_from_ms = data.header.event_from_ms;
        entry.event_to_ms = data.header.event_to_ms;
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
    }
    if (!work.totals_written) {
        HistoricalStr str(session, job);
        bool str_expected = false;
        if (job.request.view == View::Day)
            for (size_t i = 0; i < index_count; ++i)
                str_expected |= index[i].day == last && (index[i].entry.flags & EdfCatalog::ENTRY_STR_READY);
        if (str_expected && !str.load(last)) {
            if (aborted(&job, session)) return false;
            str.present = false;
            partial = true;
            note_issue(job, "str_read", "/STR.edf");
            job.accepted_issue.capture(job);
        }
        job.body.print("],\"present\":%s,\"partial\":%s,\"used_seconds\":",
                       sessions || str.present ? "true" : "false", partial ? "true" : "false");
        if (usage_known && seconds > 0) job.body.print("%.10g", seconds);
        else job.body.print("null");
        job.body.print(",\"rdi\":");
        if (seconds > 0 && events_known) job.body.print("%.10g", (counts[0] + counts[1] + counts[2] + counts[3] + counts[4]) * 3600.0 / seconds);
        else job.body.print("null");
        job.body.print(",\"csr_seconds\":");
        if (csr_known && seconds > 0) job.body.print("%.10g", csr_seconds);
        else job.body.print("null");
        job.body.print(",\"leak_threshold\":%.10g,\"leak_high_seconds\":", LEAK_THRESHOLD);
        if (!aggregate.incompatible[3] && aggregate.coverage[3] > 0) job.body.print("%.10g", leak_high_seconds);
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
        job.body.print("\"channels\":[");
        for (size_t c = 0; c < CHANNEL_COUNT; ++c) {
            ReportEdf::Statistics stats;
            const bool present = !aggregate.incompatible[c] && aggregate.channels[c].bins &&
                ReportEdf::summarize(aggregate.channels[c], stats) && stats.present;
            job.body.print("%s{\"id\":\"%s\",\"label\":\"%s\",\"unit\":\"%s\",\"count\":%llu,",
                           c ? "," : "", CHANNELS[c].id, CHANNELS[c].label, CHANNELS[c].unit,
                           static_cast<unsigned long long>(present ? stats.count : 0));
            job.body.print("\"coverage_seconds\":%.10g,", present ? aggregate.coverage[c] : 0);
            const double values[] = {present ? stats.min : NAN, present ? stats.mean : NAN,
                present ? stats.median : str.statistic(c, "50"),
                present ? stats.p95 : str.statistic(c, "95"), present ? stats.max : str.statistic(c, "Max")};
            constexpr const char *keys[] = {"min", "mean", "median", "p95", "max"};
            bool str_value = false;
            for (size_t n = 0; n < 5; ++n) {
                job.body.print("\"%s\":", keys[n]);
                if (std::isfinite(values[n])) { job.body.print("%.10g,", values[n]); str_value |= !present; }
                else job.body.print("null,");
            }
            job.body.print("\"source\":\"%s\"}", present ? "edf" : str_value ? "str" : "");
        }
        job.body.print("]");
        if (job.request.view == View::Day) str.settings_json(job.body, last);
        work.totals_written = true;
    }
    if (job.request.view == View::Day && !history_json(session, job, false)) return false;
    job.body.print("}");
    work.complete = job.body.good();
    return work.complete;
}

double chart_scale(size_t c, const ReportEdf::Signal &signal) {
    if ((c == 0 || c == 4) && !strcmp(signal.unit, "L/s")) return 60;
    if (c == 7 && !strcmp(signal.unit, "L")) return 1000;
    return 1;
}

bool start_part(PartGuard &part, const uint8_t *header, size_t bytes) {
    if (!part.session.mutate(part.path, [&](fs::FS &fs) {
        if ((!fs.exists("/airbridge") && !fs.mkdir("/airbridge")) ||
            (!fs.exists(CACHE_DIR) && !fs.mkdir(CACHE_DIR))) return false;
        char directory[96];
        snprintf(directory, sizeof(directory), "%s", part.path);
        *strrchr(directory, '/') = 0;
        if (!fs.exists(directory) && !fs.mkdir(directory)) return false;
        return !fs.exists(part.path) || fs.remove(part.path);
    })) return false;
    return part.writer.open(part.session, part.path) && part.writer.write(header, bytes);
}

bool finish_part(PartGuard &part, const char *path, uint64_t bytes) {
    if (!part.writer.finish(bytes)) return false;
    uint64_t old_bytes = 0;
    const bool success = part.session.mutate(part.path, path, [&](fs::FS &fs) {
        fs::File old = fs.open(path, FILE_READ);
        if (old) old_bytes = old.size();
        old.close();
        return (!fs.exists(path) || fs.remove(path)) && fs.rename(part.path, path);
    });
    if (success && cache_usage_known) cache_bytes = cache_bytes - std::min(cache_bytes, old_bytes) + bytes;
    else cache_usage_known = false;
    return success;
}

bool cache_name(const char *input, const char *&base) {
    base = strrchr(input, '/');
    base = base ? base + 1 : input;
    if (strlen(base) < 21 || strlen(base) > 32) return false;
    for (size_t n = 0; n < 15; ++n)
        if (n == 8 ? base[n] != '_' : base[n] < '0' || base[n] > '9') return false;
    if (strcmp(base + 15, ".stats") && strcmp(base + 15, ".series") &&
        strcmp(base + 15, ".stats.part") && strcmp(base + 15, ".series.part")) return false;
    return true;
}

bool migrate_cache(const SdStorage::Session &session, const Job *job) {
    if (rebuild_generation) return true;
    SdStorage::Reader directory;
    if (!directory.open(session, CACHE_DIR, true))
        return session.run([&](fs::FS &fs) { return !fs.exists(CACHE_DIR); });
    bool end = false;
    SdStorage::Reader::Entry file;
    while (!end) {
        if (aborted(job, session)) return false;
        if (!directory.next(file, end)) return false;
        const char *base = nullptr;
        if (end || file.directory || !cache_name(file.name, base) || strstr(base, ".part")) continue;
        for (size_t i = 0; i < index_count; ++i) {
            const auto &entry = index[i];
            if (strncmp(base, entry.entry.file_prefix, 15)) continue;
            char old[96], path[96], parent[96];
            snprintf(old, sizeof(old), "%s/%s", CACHE_DIR, base);
            cache_path(entry, base + 16, path);
            snprintf(parent, sizeof(parent), "%s/%s", CACHE_DIR, entry.entry.therapy_day);
            // A rename is restart-safe. Never overwrite an independently existing copy.
            if (!session.mutate(old, path, [&](fs::FS &fs) {
                return fs.exists(path) || ((!fs.exists(parent) ? fs.mkdir(parent) : true) && fs.rename(old, path));
            })) return false;
            break;
        }
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
    std::unique_ptr<SessionData> data(new (std::nothrow) SessionData);
    if (!data) return note_issue(job, "session_alloc");
    if (!session_data(session, job, entry, *data)) return false;
    header = data->header;
    return true;
}

struct HistoryWork : ReportWork {
    static constexpr size_t DAYS = 14;
    DailyValues days[DAYS];
    bool cached[DAYS] = {}, complete = true;
    size_t next = 0;
    CacheHeader header;
};

aircannect::JobProgressPoint report_progress_point(const Job &job) {
    static_assert(MAX_SESSION_COUNT <= UINT16_MAX, "report progress session cursor must fit 16 bits");
    static_assert(PrepareWork::Publish + 2 <= UINT8_MAX, "report progress phase must fit 8 bits");
    const uint64_t dataset = uint64_t(job.data_revision) << 32;
    if (job.indexing) return {dataset, index_scan.next};
    uint64_t phase = 1, item = 0, part = 0, units = 0;
    bool history = job.request.action == Action::History;
    bool summary = job.request.action == Action::Summary ||
        (job.request.action == Action::Prepare && job.request.view == View::Period);
    if (job.preparation) {
        const auto &work = *job.preparation;
        phase += work.phase;
        item = work.next;
        history = work.phase == PrepareWork::History;
        summary = work.phase == PrepareWork::Summary;
        if (work.phase == PrepareWork::Publish) phase = 6;
        if (work.phase == PrepareWork::Events && work.loaded) {
            part = 40 + (work.histogram.bytes != 0);
            units = work.next_event;
        }
    }
    if (summary && job.summary) {
        item = job.summary->next;
        if (job.summary->totals_written) {
            ++phase;
            history = true;
        }
    }
    if (history && job.history) item = job.history->next;
    if (job.build && part < 40) {
        const auto &work = *job.build;
        const auto &input = work.input;
        const uint64_t source = input.record_offset == UINT64_MAX ? 0 :
            input.record_offset - input.header->header_bytes + input.record_loaded;
        if (work.header_kind < 5) {
            part = 1 + work.header_kind;
            units = source;
            if (work.header_kind == 1 && input.header) {
                const auto &position = work.bounds_position;
                const uint64_t offset = input.header->header_bytes +
                    uint64_t(position.record) * input.header->record_bytes;
                units = uint64_t(position.ordinal) * input.header->record_bytes +
                    (input.record_offset == offset ? input.record_loaded : 0);
            }
        } else if (work.numeric_kind < 3) {
            part = 10 + work.numeric_kind;
            // Count partial reads as well as spool/output work. The source
            // offset belongs to the loaded record, not the next loop index.
            units = source + (work.producer ? work.producer->progress() : 0);
        } else {
            part = 20 + work.event_kind;
            units = work.next_record;
        }
        if (work.finished) {
            part = 30 + (work.cache_store ? work.cache_store->kind - 3 : 0);
            units = work.cache_store ? work.cache_store->written + work.cache_store->record : 0;
        }
    }
    if (job.request.action == Action::Days) units = job.listed_days;
    // Dataset revision (32), phase (8), session cursor (16), source stage (8).
    // Phases/stages reach 6/41; cursors include the catalog end sentinel.
    // Only an accepted dataset replacement advances the epoch, not a retry.
    return {dataset | (phase << 24) | (item << 8) | part, units};
}

void report_progress(const Job &job, aircannect::JobProgressPoint point, bool paused) {
    using Event = aircannect::JobProgressWatch::Event;
    const auto event = job.progress.observe(millis(), point, paused);
    if (job.diagnostics) job.max_idle_ms = std::max(job.max_idle_ms,
        std::max(job.progress.idle_ms(), job.progress.notice_ms()));
    if (event == Event::None) return;
    Log::logf(CAT_REPORT, event == Event::Resumed ? LOG_INFO : LOG_WARN,
        "EDF job=%lu %s %s day=%u wait=%s no_progress=%lus attempts=%lu\n",
        static_cast<unsigned long>(job.id), event == Event::Resumed ? "resumed" :
        event == Event::Stalled ? "stalled" : "still stalled", action_name(job.request.action), unsigned(job.request.day),
        job.wait_reason, static_cast<unsigned long>(job.progress.notice_ms() / 1000),
        static_cast<unsigned long>(job.progress.attempts()));
}

uint64_t maintenance_cursor(MaintenancePhase phase) {
    switch (phase) {
        case MaintenancePhase::Index: return index_scan.next;
        case MaintenancePhase::IndexGc: return uint64_t(index_cleanup.shard) * 2 + index_cleanup.discovered;
        case MaintenancePhase::ArtifactGc: return artifact_cleanup.next * 2 + artifact_cleanup.discovered;
        default: return 0;
    }
}

bool maintenance_suspended(MaintenancePhase phase) {
    if (phase != MaintenancePhase::IndexGc && phase != MaintenancePhase::ArtifactGc) return false;
    if (__atomic_load_n(&artifact_reads, __ATOMIC_ACQUIRE) || foreground_wait()) return true;
    // GC is idle work: queued/running reports intentionally own this worker.
    // Artifact GC also preserves retained result pins until their release.
    if (xSemaphoreTake(mutex, 0) != pdTRUE) return true;
    bool pinned = false;
    for (const auto &job : jobs)
        if (job.state == JobState::Queued || job.state == JobState::Running ||
            (phase == MaintenancePhase::ArtifactGc && job.state != JobState::Free && !evictable(job))) pinned = true;
    xSemaphoreGive(mutex);
    return pinned;
}

void observe_maintenance_progress(MaintenanceProgress &work, bool retained = false, bool complete = false,
                                  const SdStorage::IoEvent *io_event = nullptr) {
    if (!work.active) return;
    const bool readers = io_event ? *io_event == SdStorage::IoEvent::WaitingForReader :
        SdStorage::waiting_for_readers() || maintenance_suspended(work.phase);
    const bool paused = !AirSenseState::local_background_allowed() || !SdStorage::local_access_allowed() ||
        readers;
    const uint64_t cursor = maintenance_cursor(work.phase);
    if (!retained && cursor != work.cursor) { work.cursor = cursor; work.steps = 0; }
    using Event = aircannect::JobProgressWatch::Event;
    const auto event = retained ? work.watch.check(millis(), paused) :
        work.watch.observe(millis(), complete ? aircannect::JobProgressPoint{UINT64_MAX, 0} :
            aircannect::JobProgressPoint{work.cursor, work.steps}, paused);
    if (event != Event::None) {
        const char *phase = work.phase == MaintenancePhase::Index ? "index" :
            work.phase == MaintenancePhase::Publish ? "index_publish" :
            work.phase == MaintenancePhase::IndexGc ? "index_gc" : "artifact_gc";
        Log::logf(CAT_REPORT, event == Event::Resumed ? LOG_INFO : LOG_WARN,
            "EDF maintenance %s phase=%s wait=%s no_progress=%lus attempts=%lu\n",
            event == Event::Resumed ? "resumed" : event == Event::Stalled ? "stalled" : "still stalled", phase,
            work.wait_reason, static_cast<unsigned long>(work.watch.notice_ms() / 1000),
            static_cast<unsigned long>(work.watch.attempts()));
    }
    if (complete) work.active = false;
}

void observe_report_jobs(bool force, bool retained, const Job *io_job, const SdStorage::IoEvent *io_event) {
    static uint32_t observed_ms = 0;
    const uint32_t now = millis();
    if (!force && uint32_t(now - observed_ms) < 250) return;
    observed_ms = now;
    // The exact SD wait belongs to this caller; other queued work uses the
    // admission hint without masking an unrelated operation's stalled I/O.
    for (auto &work : maintenance_progress)
        observe_maintenance_progress(work, true, false, !io_job && &work == maintenance_io ? io_event : nullptr);
    const bool paused = !AirSenseState::local_background_allowed() || !SdStorage::local_access_allowed();
    const bool readers = SdStorage::waiting_for_readers();
    if (xSemaphoreTake(mutex, retained ? 0 : portMAX_DELAY) != pdTRUE) return;
    for (const auto &job : jobs) {
        if (job.cancelled || (job.state != JobState::Queued && job.state != JobState::Running)) continue;
        const bool reader_wait = io_event && &job == io_job ? *io_event == SdStorage::IoEvent::WaitingForReader : readers;
        report_progress(job, !retained && job.state == JobState::Running ? report_progress_point(job) :
            aircannect::JobProgressPoint{}, paused || reader_wait);
    }
    xSemaphoreGive(mutex);
}

void observe_report_io(void *context, SdStorage::IoEvent event, const char *path) {
    auto *job = static_cast<Job *>(context);
    auto *work = job ? nullptr : maintenance_io;
    if (event == SdStorage::IoEvent::Complete) {
        if (work && work->observing) {
            ++work->steps;
            observe_maintenance_progress(*work);
        }
        return;
    }
    const char *previous = job ? job->wait_reason : work ? work->wait_reason : nullptr;
    const char *reason = event == SdStorage::IoEvent::WaitingForReader ? "sd_reader_wait" :
        path ? "sd_write_wait" : "sd_io_wait";
    if (job) job->wait_reason = reason;
    else if (work) work->wait_reason = reason;
    // Close/destruction callbacks can observe a half-reset input or producer.
    // Only age retained high-water marks while a filesystem request is pending.
    observe_report_jobs(true, true, job, &event);
    if (job) job->wait_reason = previous;
    else if (work) work->wait_reason = previous;
}

bool history_json(const SdStorage::Session &session, Job &job, bool build) {
    constexpr size_t DAYS = HistoryWork::DAYS, METRICS = 6;
    constexpr size_t channels[] = {0, 3, 6, 4};
    constexpr const char *names[] = {"usage", "ahi", "pressure", "leak", "vt", "mv"};
    const uint16_t last = last_day(job.request), first = last > DAYS - 1 ? last - DAYS + 1 : 0;
    if (!job.history) {
        job.history.reset(new (std::nothrow) HistoryWork);
        if (!job.history) return false;
        for (size_t d = 0; d < DAYS; ++d) {
            job.history->cached[d] = cached_day(first + d, job.history->days[d], build);
            job.history->complete &= job.history->days[d].ready;
        }
    }
    auto &work = job.history;
    auto &complete = work->complete;
    for (size_t &i = work->next; i < index_count; ++i) {
        const auto &entry = index[i];
        if (!selected(entry, first, last) || entry.day - first >= DAYS) continue;
        if (work->cached[entry.day - first]) continue;
        if (aborted(&job, session)) return false;
        auto &day = work->days[entry.day - first];
        day.present = true;
        auto &header = work->header;
        if (!cached_summary_header(session, entry, header) &&
            (!build || !build_summary_header(session, job, entry, header))) {
            if (aborted(&job, session)) return false;
            job.accepted_issue.capture(job);
            day.ready = complete = false;
            continue;
        }
        day.usage &= header.usage_known;
        day.event_count &= header.events_known || !header.recorded;
        day.seconds += header.used_seconds;
        for (size_t c = 0; c < 4; ++c) {
            day.events += header.counts[c];
            day.count[c] += header.channels[channels[c]].count;
            day.sum[c] += header.channels[channels[c]].sum;
        }
    }
    if (aborted(&job, session, false)) return false;
    for (size_t d = 0; d < DAYS; ++d)
        if (!work->cached[d]) remember_day(first + d, work->days[d]);
    job.body.print(",\"history\":{\"first_day\":%u,\"last_day\":%u,\"complete\":%s,\"metrics\":[",
                   first, last, complete ? "true" : "false");
    for (size_t m = 0; m < METRICS; ++m) {
        double sum = 0;
        unsigned count = 0;
        for (size_t d = DAYS - 8; d < DAYS - 1; ++d) {
            const double value = work->days[d].value(m);
            if (std::isfinite(value)) { sum += value; ++count; }
        }
        const double current = work->days[DAYS - 1].value(m);
        job.body.print("%s{\"id\":\"%s\",\"baseline_count\":%u,\"baseline\":", m ? "," : "", names[m], count);
        if (count) job.body.print("%.10g", sum / count); else job.body.print("null");
        job.body.print(",\"delta\":");
        if (count && std::isfinite(current)) job.body.print("%.10g", current - sum / count);
        else job.body.print("null");
        job.body.print(",\"points\":[");
        for (size_t d = 0; d < DAYS; ++d) {
            const double value = work->days[d].value(m);
            if (d) job.body.print(",");
            if (std::isfinite(value)) job.body.print("%.10g", value); else job.body.print("null");
        }
        job.body.print("]}");
    }
    job.body.print("]}");
    job.history.reset();
    return job.body.good();
}

bool window_events(const SdStorage::Session &session, const Job &job,
                   const IndexedSession &entry, const CacheHeader &header,
                   int64_t from, int64_t to, Payload &out, bool &comma, bool &truncated,
                   uint32_t &next, CsrPairing &csr) {
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
    auto consume = [&](const Event &event) {
        csr.consume(event, [&](const Event &value, bool paired) { append(value, paired ? "csr" : nullptr); });
    };
    char path[96];
    cache_path(entry, "stats", path);
    SdStorage::Reader reader;
    if (reader.open(session, path) && reader.size() == header.bytes &&
        reader.seek(sizeof(header) + total_bins(header) * sizeof(uint64_t) + uint64_t(next) * sizeof(Event))) {
        Event block[32];
        for (uint32_t &n = next; n < header.events;) {
            const size_t take = std::min<uint32_t>(32, header.events - n);
            if (aborted(&job, session) || reader.read(reinterpret_cast<uint8_t *>(block),
                take * sizeof(Event)) != take * sizeof(Event)) return false;
            for (size_t i = 0; i < take; ++i) consume(block[i]);
            if (truncated) return out.good();
            n += take;
        }
        if (csr.active) append(csr.start);
        return out.good();
    }
    reader.close();
    // Without a usable cache, replay the small annotation fallback as one
    // bounded output. The caller never appends it to an earlier partial pass.
    next = 0;
    csr = {};
    comma = false;
    out.clear();
    out.print("{\"events\":[");
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
    if (csr.active) append(csr.start);
    return out.good();
}

Job::~Job() = default;

void clear_assembly(Job &job) {
    job.preparation.reset();
    job.summary.reset();
    job.history.reset();
    job.accepted_issue = {};
}

void error_body(Job &job, int code, const char *error) {
    job.body.~Payload();
    new (&job.body) Payload;
    job.code = code;
    job.error = error;
    job.body.print("{\"source\":\"edf\",\"error\":\"%s\"}", error);
}

// A collection proof lives for one SD lease. Only small publication documents
// are traversed; numeric payloads are never rehashed to decide reachability.
struct DayCleanup {
    struct File {
        char name[128] = {};
        uint32_t bytes = 0;
        bool keep = false, discard = false, leaf = false, legacy = false;
        uint16_t period = 0;
        bool replacement = false;
    };
    const SdStorage::Session &session;
    uint16_t day;
    uint32_t epoch = __atomic_load_n(&foreground_epoch, __ATOMIC_ACQUIRE);
    char directory[96] = {};
    File *files = nullptr;
    size_t count = 0, capacity = 0;
    uint32_t removed = 0;
    uint64_t removed_bytes = 0;
    uint8_t empty_shards[32] = {};

    DayCleanup(const SdStorage::Session &session, uint16_t day) : session(session), day(day) {
        publication_prefix(day, directory);
        *strrchr(directory, '/') = 0;
    }
    ~DayCleanup() { aircannect::Memory::free(files); }
    File *append() {
        if (count == capacity) {
            const size_t next = capacity ? capacity * 2 : 32;
            if (next > SIZE_MAX / sizeof(File)) return nullptr;
            auto *grown = static_cast<File *>(aircannect::Memory::realloc_large(files, next * sizeof(File), false));
            if (!grown) return nullptr;
            files = grown; capacity = next;
        }
        return new (files + count++) File;
    }
    bool active() const {
        return !cleanup_cancelled(const_cast<SdStorage::Session *>(&session)) &&
            epoch == __atomic_load_n(&foreground_epoch, __ATOMIC_ACQUIRE);
    }
    bool inventory(const char *relative = "") {
        char path[192];
        snprintf(path, sizeof(path), "%s%s%s", directory, *relative ? "/" : "", relative);
        SdStorage::Reader reader;
        if (!reader.open(session, path, true)) return false;
        SdStorage::Reader::Entry entry;
        bool end = false;
        bool empty = true;
        uint8_t shards[32] = {};
        while (!end) {
            if (!active() || !reader.next(entry, end)) return false;
            if (end) break;
            empty = false;
            const char *name = strrchr(entry.name, '/');
            name = name ? name + 1 : entry.name;
            if (entry.directory) {
                uint32_t shard;
                if (!*relative && strlen(name) == 2 && strspn(name, "0123456789abcdef") == 2 &&
                    aircannect::parse_hex(name, 2, shard)) shards[shard / 8] |= 1u << (shard % 8);
                continue;
            }
            if (entry.size > UINT32_MAX || strlen(name) + strlen(relative) + 2 > sizeof(File::name)) continue;
            auto *added = append();
            if (!added) return false;
            auto &file = *added;
            snprintf(file.name, sizeof(file.name), "%s%s%s", relative, *relative ? "/" : "", name);
            file.bytes = entry.size;
        }
        reader.close();
        uint32_t empty_shard;
        if (empty && *relative && aircannect::parse_hex(relative, 2, empty_shard))
            empty_shards[empty_shard / 8] |= 1u << (empty_shard % 8);
        for (unsigned i = 0; i < 256; ++i) if (shards[i / 8] & (1u << (i % 8))) {
            char shard[3]; snprintf(shard, sizeof(shard), "%02x", i);
            if (!inventory(shard)) return false;
        }
        return true;
    }
    File *find(const char *name) {
        size_t lo = 0, hi = count;
        while (lo < hi) {
            const size_t mid = lo + (hi - lo) / 2;
            if (strcmp(files[mid].name, name) < 0) lo = mid + 1; else hi = mid;
        }
        return lo < count && !strcmp(files[lo].name, name) ? files + lo : nullptr;
    }
    bool content(const uint8_t digest[32], uint32_t bytes, bool document, bool keep) {
        char text[65], name[80];
        aircannect::hex_encode(digest, 32, text, sizeof(text), aircannect::HexCase::Lower);
        snprintf(name, sizeof(name), "%.*s%s%s.%s", document ? 2 : 0, text,
            document ? "/" : "", text, document ? "abp" : "abt");
        auto *file = find(name);
        if (!file || file->bytes != bytes) return !keep;
        file->keep |= keep;
        file->discard |= !keep;
        return true;
    }
    bool reference(JsonObjectConst value, bool document, bool keep) {
        const char *date = value["day"], *text = value["digest"];
        uint16_t parsed;
        uint8_t digest[32]; size_t decoded;
        return date && catalog_day(date, parsed) && parsed == day &&
            aircannect::sha256_text_valid(text) && value["bytes"].is<uint32_t>() &&
            aircannect::hex_decode(text, 64, digest, sizeof(digest), decoded) &&
            content(digest, value["bytes"], document, keep);
    }
    bool read(const File &file, uint8_t *out, size_t length) {
        char path[224]; snprintf(path, sizeof(path), "%s/%s", file.legacy ? CACHE_DIR : directory, file.name);
        SdStorage::Reader reader;
        return active() && length <= file.bytes && reader.open(session, path) &&
            reader.size() == file.bytes && reader.read(out, length) == length;
    }
    const IndexedSession *source(const char *name, uint16_t source_day = 0) const {
        if (strlen(name) < 15 || (name[15] && name[15] != '.' && name[15] != '_')) return nullptr;
        if (!source_day) source_day = day;
        for (size_t i = 0; i < index_count; ++i)
            if (selected(index[i], source_day, source_day) && !strncmp(index[i].entry.file_prefix, name, 15)) return index + i;
        return nullptr;
    }
    // Recognize only filenames produced by the known cache/producer versions.
    // An unknown session or future suffix is preserved, not guessed away.
    bool obsolete(const File &file) const {
        const auto *entry = source(file.name);
        if (!entry) return false;
        const char *tail = file.name + 15;
        int kind = -1;
        if (*tail == '_') {
            for (int k = 0; k < 3; ++k) if (!strncmp(tail + 1, SUFFIXES[k], 3) && tail[4] == '.') kind = k;
            if (kind < 0 || !(entry->key.present & (1u << kind))) return false;
            tail += 4;
        }
        uint32_t generation = 0;
        if (!strncmp(tail, ".g", 2)) {
            const char *digits = tail + 2, *end = digits;
            while (*end >= '0' && *end <= '9') ++end;
            if (end == digits || *end != '.' || end - digits > 10) return false;
            uint64_t value = 0;
            for (const char *p = digits; p != end; ++p) value = value * 10 + unsigned(*p - '0');
            if (value > UINT32_MAX) return false;
            generation = value; tail = end;
        }
        if (kind < 0) {
            if (!strcmp(tail, ".series") || !strcmp(tail, ".series.part") || !strcmp(tail, ".stats.part")) return true;
            if (!strcmp(tail, ".stats") || !strcmp(tail, ".events.head0") || !strcmp(tail, ".events.head1") ||
                !strcmp(tail, ".histograms.head0") || !strcmp(tail, ".histograms.head1")) return file.legacy || generation != rebuild_generation;
            return !strcmp(tail, ".events.document.part") || !strcmp(tail, ".histograms.document.part");
        }
        if (!strcmp(tail, ".checkpoint.head0") || !strcmp(tail, ".checkpoint.head1") ||
            !strcmp(tail, ".checkpoint.document.part") || !strcmp(tail, ".samples.part") ||
            !strcmp(tail, ".tiles.part") || !strcmp(tail, ".resume.part")) return true;
        if (strlen(tail) != 69 || tail[0] != '.' || strcmp(tail + 65, ".ref") ||
            strspn(tail + 1, "0123456789abcdef") != 64) return false;
        char prefix[96], text[65]; uint8_t key[32];
        if (!artifact_identity(*entry, kind, prefix, key)) return false;
        aircannect::hex_encode(key, 32, text, sizeof(text), aircannect::HexCase::Lower);
        return generation != rebuild_generation || strncmp(text, tail + 1, 64);
    }
    bool covered(JsonObjectConst summary, bool period = false) const {
        if (!summary["sessions"].is<JsonArrayConst>() || !summary["partial"].is<bool>() || summary["partial"].as<bool>()) return false;
        for (JsonObjectConst item : summary["sessions"].as<JsonArrayConst>()) {
            const char *prefix = item["prefix"];
            const uint16_t source_day = period ? item["day"].as<uint16_t>() : day;
            if (!prefix || strlen(prefix) != 15 || !source_day) return false;
            const auto *entry = source(prefix, source_day);
            if (!entry || !(entry->key.present & 7)) return false;
        }
        return true;
    }
    bool covered_manifest(JsonObjectConst manifest) const {
        const auto summary = manifest["summary"].as<JsonObjectConst>();
        if (!covered(summary)) return false;
        for (JsonObjectConst artifact : manifest["artifacts"].as<JsonArrayConst>()) {
            const IndexedSession *entry = nullptr;
            for (JsonObjectConst item : summary["sessions"].as<JsonArrayConst>())
                if (item["id"] == artifact["session"]) entry = source(item["prefix"]);
            int kind = -1;
            for (int k = 0; k < 3; ++k) if (artifact["kind"] == SUFFIXES[k]) kind = k;
            if (!entry || kind < 0 || !(entry->key.present & (1u << kind))) return false;
        }
        return true;
    }
    ReportArtifact::Lookup collect() {
        using ReportArtifact::Lookup;
        char prefix[96]; uint8_t key[32];
        publication_prefix(day, prefix);
        Request request; request.day = day;
        ReportArtifact::Document current;
        if (!publication_key(request, key) ||
            ReportArtifact::document_lookup(session, prefix, key, current) != Lookup::Ready) return Lookup::Invalid;
        // A complete replacement must already exist. Cleanup never builds one,
        // scans EDF, or removes the only surviving legacy cache on speculation.
        std::unique_ptr<CacheHeader> header(new (std::nothrow) CacheHeader);
        if (!header) return Lookup::Failed;
        size_t sources = 0;
        for (size_t i = 0; i < index_count; ++i) if (selected(index[i], day, day)) {
            if (!active()) return Lookup::Failed;
            ++sources;
            if (!(index[i].key.present & 7) || !cached_summary_header(session, index[i], *header) || header->partial)
                return active() ? Lookup::Invalid : Lookup::Failed;
            for (uint8_t kind = 0; kind < 3; ++kind) if (index[i].key.present & (1u << kind)) {
                if (!active()) return Lookup::Failed;
                char name[96]; uint8_t identity[32]; ReportArtifact::Publication publication;
                if (!artifact_identity(index[i], kind, name, identity) ||
                    ReportArtifact::lookup(session, name, identity, publication) != Lookup::Ready)
                    return active() ? Lookup::Invalid : Lookup::Failed;
            }
        }
        if (!sources) return Lookup::Invalid;
        header.reset();
        if (!inventory()) return Lookup::Failed;
        for (size_t at = 0; at < artifact_cleanup.legacy.length(); at += sizeof(CleanupLegacy)) {
            CleanupLegacy legacy; memcpy(&legacy, artifact_cleanup.legacy.c_str() + at, sizeof(legacy));
            if (legacy.day != day) continue;
            auto *file = append();
            if (!file) return Lookup::Failed;
            strcpy(file->name, legacy.name); file->bytes = legacy.bytes; file->legacy = true;
        }
        std::sort(files, files + count, [](const File &a, const File &b) { return strcmp(a.name, b.name) < 0; });
        // Heads and references are parsed once before any deletion. Retained
        // manifests may still reference bodies whose named heads are obsolete.
        for (size_t i = 0; i < count; ++i) {
            if (!active()) return Lookup::Failed;
            auto &file = files[i];
            file.discard = obsolete(file);
            file.discard |= !strcmp(file.name, "day.document.part") ||
                !strcmp(file.name, "day.refused.head0") || !strcmp(file.name, "day.refused.head1") ||
                !strcmp(file.name, "day.refused.document.part");
            const size_t n = strlen(file.name);
            if (file.discard && n > 6 && (!strcmp(file.name + n - 6, ".stats") || !strcmp(file.name + n - 7, ".series"))) {
                uint8_t header[20];
                if (file.bytes < sizeof(header)) return Lookup::Invalid;
                if (!read(file, header, sizeof(header))) return active() ? Lookup::Invalid : Lookup::Failed;
                // Native v7 cache layouts are the only legacy formats this
                // owner understands. Future/foreign cache files are untouched.
                const bool stats = !strcmp(file.name + n - 6, ".stats");
                file.discard = stats ? SdStorage::get_le32(header) == STATS_CACHE_VERSION && !memcmp(header + 12, "ABRPT01", 8) :
                    SdStorage::get_le32(header + 8) == 7 && !memcmp(header, "ABSER01", 8);
            }
            if (n > 6 && (!strcmp(file.name + n - 6, ".head0") || !strcmp(file.name + n - 6, ".head1"))) {
                char name[224]; snprintf(name, sizeof(name), "%s/%.*s", directory, int(n - 6), file.name);
                ReportArtifact::Document document;
                const auto state = ReportArtifact::document_slot(session, name, file.name[n - 1] - '0', document);
                if (state != Lookup::Ready) return state == Lookup::Failed ? state : Lookup::Invalid;
                if (!content(document.digest, document.bytes, true, !file.discard)) return Lookup::Invalid;
                if (!strncmp(file.name, "period-", 7)) {
                    unsigned period = 0; int end = 0;
                    if (sscanf(file.name, "period-%u%n", &period, &end) == 1 && size_t(end) == n - 6) {
                        Request request; request.view = View::Period; request.day = day; request.period = period;
                        uint8_t current_key[32];
                        if (period <= UINT16_MAX && valid(request) && publication_key(request, current_key) &&
                            !memcmp(document.key, current_key, 32)) {
                            char digest[65], body[80];
                            aircannect::hex_encode(document.digest, 32, digest, sizeof(digest), aircannect::HexCase::Lower);
                            snprintf(body, sizeof(body), "%.2s/%s.abp", digest, digest);
                            if (auto *replacement = find(body)) { replacement->period = period; replacement->replacement = true; }
                        }
                    }
                }
                if (strstr(file.name, ".refused.head")) {
                    char digest[65], body[80];
                    aircannect::hex_encode(document.digest, 32, digest, sizeof(digest), aircannect::HexCase::Lower);
                    snprintf(body, sizeof(body), "%.2s/%s.abp", digest, digest);
                    if (auto *leaf = find(body)) leaf->leaf = true;
                }
            } else if (n > 69 && !strcmp(file.name + n - 4, ".ref") && file.name[n - 69] == '.') {
                uint8_t identity[32]; size_t decoded;
                if (!aircannect::hex_decode(file.name + n - 68, 64, identity, sizeof(identity), decoded)) return Lookup::Invalid;
                char name[224]; snprintf(name, sizeof(name), "%s/%.*s", directory, int(n - 69), file.name);
                ReportArtifact::Publication publication;
                const auto state = ReportArtifact::lookup(session, name, identity, publication);
                if (state != Lookup::Ready) return state == Lookup::Failed ? state : Lookup::Invalid;
                if (!content(publication.digest, publication.bytes, false, !file.discard)) return Lookup::Invalid;
            }
        }
        for (size_t at = 0; at < artifact_cleanup.anchors.length(); at += sizeof(CleanupAnchor)) {
            CleanupAnchor anchor; memcpy(&anchor, artifact_cleanup.anchors.c_str() + at, sizeof(anchor));
            if (anchor.day == day && !content(anchor.digest, anchor.bytes, true, true)) return Lookup::Invalid;
        }
        bool current_checked = false;
        for (size_t i = 0; i < count; ++i) {
            if (!active()) return Lookup::Failed;
            auto &file = files[i];
            if (file.leaf) { file.discard = true; continue; }
            if (strlen(file.name) == 68 && !strcmp(file.name + 64, ".abt") &&
                strspn(file.name, "0123456789abcdef") == 64 && file.bytes >= ReportArtifact::HEADER_BYTES) {
                uint8_t header[ReportArtifact::HEADER_BYTES];
                if (!read(file, header, sizeof(header))) return Lookup::Failed;
                if (!memcmp(header, "ABTILE01", 8) && SdStorage::get_le16(header + 8) == ReportArtifact::VERSION &&
                    SdStorage::get_le32(header + 12) == file.bytes) file.discard = true;
                continue;
            }
            if (strlen(file.name) != 71 || file.name[2] != '/' || strcmp(file.name + 67, ".abp") ||
                strspn(file.name + 3, "0123456789abcdef") != 64 || strncmp(file.name, file.name + 3, 2)) continue;
            uint8_t first[16];
            memset(first, 0, sizeof(first));
            if (file.bytes < 11) return Lookup::Invalid;
            if (!read(file, first, std::min<size_t>(file.bytes, sizeof(first)))) return Lookup::Failed;
            if (!memcmp(first, "ABHIST01", 8) && SdStorage::get_le16(first + 8) == 1 &&
                SdStorage::get_le32(first + 12) == file.bytes) { file.discard = true; continue; }
            if (!memcmp(first, "ABCHK001", 8) && file.bytes == 48) {
                // Unheaded checkpoint bodies have no live resume point.
                file.discard = true; continue;
            }
            if (memcmp(first, "{\"source\":\"edf\",", 16) && memcmp(first, "{\"events\":[", 11)) {
                uint8_t reason[32] = {};
                if (file.bytes >= sizeof(reason) || !read(file, reason, file.bytes)) return Lookup::Invalid;
                bool refused = false;
                for (const char *known : {"partial_source", "source_header", "artifact_lookup", "numeric_files_missing"})
                    refused |= file.bytes == strlen(known) && !memcmp(reason, known, file.bytes);
                if (!refused) return Lookup::Invalid;
                file.discard = true; continue;
            }
            if (file.bytes > MAX_BODY) return Lookup::Invalid;
            ReportArtifact::Document document;
            size_t decoded;
            if (!aircannect::hex_decode(file.name + 3, 64, document.digest, 32, decoded)) return Lookup::Invalid;
            document.bytes = file.bytes;
            aircannect::LargeTextBuffer body(MAX_BODY, 0);
            if (!ReportArtifact::document_read(session, prefix, document, body, nullptr, false,
                cleanup_cancelled, const_cast<SdStorage::Session *>(&session))) return Lookup::Failed;
            aircannect::JsonAllocator allocator(false);
            JsonDocument json(&allocator);
            const auto error = deserializeJson(json, body.c_str(), body.length());
            if (error) return error == DeserializationError::NoMemory ? Lookup::Failed : Lookup::Invalid;
            const auto root = json.as<JsonObjectConst>();
            if (root.size() == 1 && root["events"].is<JsonArrayConst>()) { file.discard = true; continue; }
            if (root["source"] != "edf" || !root["day"].is<uint16_t>() || root["day"].as<uint16_t>() != day) return Lookup::Invalid;
            if (!root["artifacts"].is<JsonArrayConst>()) {
                if (root.size() == 20 && root["view"] == "period" && root["first_day"].is<uint16_t>() &&
                    root["last_day"].as<uint16_t>() == day && root["first_day"].as<uint16_t>() <= day) {
                    const unsigned period = day - root["first_day"].as<uint16_t>() + 1;
                    const bool complete = covered(root, true);
                    file.replacement &= complete && file.period == period;
                    file.period = period;
                    file.discard = complete;
                } else return Lookup::Invalid;
                continue;
            }
            if (root.size() != 9 || !root["event_groups"].is<JsonArrayConst>() ||
                !root["summary"].is<JsonObjectConst>() || root["dataset"] != dataset_id) return Lookup::Invalid;
            const bool replacement = !memcmp(document.digest, current.digest, 32);
            if (replacement && (!covered_manifest(root) || root["summary"]["sessions"].size() != sources)) return Lookup::Invalid;
            current_checked |= replacement;
            file.keep |= replacement || !covered_manifest(root);
            file.discard = true;
            for (JsonObjectConst item : root["artifacts"].as<JsonArrayConst>())
                if (!reference(item, false, file.keep)) return Lookup::Invalid;
            for (JsonObjectConst item : root["event_groups"].as<JsonArrayConst>())
                if (!reference(item, true, file.keep) || !reference(item["histogram"], true, file.keep)) return Lookup::Invalid;
        }
        if (!current_checked) return Lookup::Invalid;
        for (size_t i = 0; i < count; ++i) if (files[i].period && files[i].discard) {
            bool replacement = false;
            for (size_t n = 0; n < count; ++n)
                replacement |= files[n].period == files[i].period && files[n].replacement;
            files[i].discard = replacement;
        }
        // Remove obsolete roots first. A reset at any following unlink leaves
        // both retained publication graphs intact; the next pass rediscovers.
        for (bool bodies : {false, true}) for (size_t i = 0; i < count; ++i) {
            const auto &file = files[i];
            const char *ext = strrchr(file.name, '.');
            const bool body = ext && (!strcmp(ext, ".abp") || !strcmp(ext, ".abt"));
            if (!file.discard || file.keep || body != bodies) continue;
            if (!active()) return Lookup::Failed;
            char path[224]; snprintf(path, sizeof(path), "%s/%s", file.legacy ? CACHE_DIR : directory, file.name);
            // A downloaded manifest can refer to a different leaf. Collection
            // protects the graph, unlike unrelated per-file build mutations.
            const bool ok = session.mutate(CACHE_DIR, [&](fs::FS &fs) {
                if (__atomic_load_n(&artifact_reads, __ATOMIC_ACQUIRE)) return false;
                return !fs.exists(path) || fs.remove(path);
            });
            if (!ok) return Lookup::Failed;
            ++removed; removed_bytes += file.bytes;
            uint32_t shard;
            if (file.name[2] == '/' && aircannect::parse_hex(file.name, 2, shard)) empty_shards[shard / 8] |= 1u << (shard % 8);
        }
        for (unsigned shard = 0; shard < 256; ++shard) if (empty_shards[shard / 8] & (1u << (shard % 8))) {
            if (!active()) return Lookup::Failed;
            bool retained = false;
            for (size_t i = 0; i < count; ++i) {
                uint32_t prefix;
                if (files[i].name[2] == '/' && aircannect::parse_hex(files[i].name, 2, prefix) && prefix == shard)
                    retained |= files[i].keep || !files[i].discard;
            }
            if (retained) continue;
            char path[100]; snprintf(path, sizeof(path), "%s/%02x", directory, shard);
            // rmdir succeeds only if empty; unknown files/subdirectories survive.
            if (!session.mutate(CACHE_DIR, [&](fs::FS &fs) {
                if (__atomic_load_n(&artifact_reads, __ATOMIC_ACQUIRE)) return false;
                fs.rmdir(path);
                return true;
            })) return Lookup::Failed;
        }
        return Lookup::Ready;
    }
};

bool artifact_cleanup_pending() {
    return !artifact_cleanup.complete || artifact_cleanup.files != SdStorage::files_revision(SdStorage::FileScope::Reports) ||
        artifact_cleanup.data != __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE) ||
        artifact_cleanup.artifacts != status.artifact_revision;
}

bool cleanup_artifacts(const SdStorage::Session &session) {
    auto *context = const_cast<SdStorage::Session *>(&session);
    if (cleanup_cancelled(context)) return false;
    // Result bodies can contain selectors into older graphs. Never invalidate
    // a pinned or not-yet-delivered response while collecting its dependencies.
    bool busy = false;
    xSemaphoreTake(mutex, portMAX_DELAY);
    for (auto &job : jobs) {
        if (job.state == JobState::Free) continue;
        if (!evictable(job)) busy = true;
    }
    if (!busy) for (auto &job : jobs) if (job.state != JobState::Free) reset_job(job);
    xSemaphoreGive(mutex);
    if (busy) return false;
    const uint32_t data = __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
    if (artifact_cleanup.files != SdStorage::files_revision(SdStorage::FileScope::Reports) || artifact_cleanup.data != data ||
        artifact_cleanup.artifacts != status.artifact_revision) {
        if (artifact_cleanup.files != SdStorage::files_revision(SdStorage::FileScope::Reports))
            for (size_t i = 0; i < daily_capacity; ++i) daily_cache[i].cleanup_checked = false;
        artifact_cleanup.~ArtifactCleanup();
        new (&artifact_cleanup) ArtifactCleanup;
        artifact_cleanup.files = SdStorage::files_revision(SdStorage::FileScope::Reports);
        artifact_cleanup.data = data;
        artifact_cleanup.artifacts = status.artifact_revision;
    }
    if (!artifact_cleanup.discovered) {
        artifact_cleanup.days.clear(); artifact_cleanup.anchors.clear(); artifact_cleanup.legacy.clear();
        // Read the two small root documents once per sweep, not once per day.
        for (unsigned slot = 0; slot < 2; ++slot) {
            ReportArtifact::Document document;
            const auto found = ReportArtifact::document_slot(session, "/airbridge/report/index", slot, document);
            if (found == ReportArtifact::Lookup::Missing) continue;
            aircannect::LargeTextBuffer body(MAX_BODY, 0);
            if (found != ReportArtifact::Lookup::Ready ||
                !ReportArtifact::document_read(session, "/airbridge/report/index", document, body, nullptr, false,
                    cleanup_cancelled, context) ||
                index_shape(body) != ReportArtifact::Lookup::Ready) return false;
            aircannect::JsonAllocator allocator(false); JsonDocument json(&allocator);
            if (deserializeJson(json, body.c_str(), body.length())) return false;
            for (const char *name : {"days", "periods"}) for (JsonObjectConst item : json[name].as<JsonArrayConst>()) {
                const auto ref = !strcmp(name, "days") ? item["publication"].as<JsonObjectConst>() : item;
                if (ref.isNull()) continue;
                // Older empty-card backfill published the zero-day sentinel.
                // That directory is outside GC's valid date range and remains
                // untouched; its references must not block other days forever.
                if (ref["day"] == "19700101") continue;
                CleanupAnchor anchor; size_t decoded;
                if (!catalog_day(ref["day"], anchor.day) ||
                    !aircannect::hex_decode(ref["digest"], 64, anchor.digest, 32, decoded)) return false;
                anchor.bytes = ref["bytes"];
                if (!artifact_cleanup.anchors.append(reinterpret_cast<const char *>(&anchor), sizeof(anchor))) return false;
            }
        }
        SdStorage::Reader directory;
        SdStorage::Reader::Entry entry; bool end = false;
        if (!directory.open(session, CACHE_DIR, true)) {
            if (!session.run([](fs::FS &fs) { return !fs.exists(CACHE_DIR); })) return false;
            end = true;
        }
        while (!end) {
            if (cleanup_cancelled(context) || !directory.next(entry, end)) return false;
            if (end) break;
            const char *name = strrchr(entry.name, '/'); name = name ? name + 1 : entry.name;
            uint16_t day;
            if (entry.directory && catalog_day(name, day) &&
                !artifact_cleanup.days.append(reinterpret_cast<const char *>(&day), sizeof(day))) return false;
            const char *base;
            if (!entry.directory && entry.size <= UINT32_MAX && cache_name(name, base)) {
                for (size_t i = 0; i < index_count; ++i) if (!strncmp(base, index[i].entry.file_prefix, 15)) {
                    CleanupLegacy legacy;
                    legacy.day = index[i].day; legacy.bytes = entry.size;
                    strcpy(legacy.name, base);
                    if (!artifact_cleanup.legacy.append(reinterpret_cast<const char *>(&legacy), sizeof(legacy))) return false;
                    break;
                }
            }
        }
        artifact_cleanup.discovered = true;
    }
    if (artifact_cleanup.next < artifact_cleanup.days.length()) {
        uint16_t day; memcpy(&day, artifact_cleanup.days.c_str() + artifact_cleanup.next, sizeof(day));
        // A new night's publication must not rescan every old day. Source
        // identity, root dependencies and owner publication invalidate this
        // proof; external edits/remount clear it for every cached day above.
        Request request; request.day = day;
        uint8_t key[32];
        aircannect::LargeTextBuffer identity(MAX_BODY, INTERNAL_WORK);
        if (!publication_key(request, key) || !identity.append(reinterpret_cast<const char *>(key), sizeof(key))) return false;
        for (size_t at = 0; at < artifact_cleanup.anchors.length(); at += sizeof(CleanupAnchor)) {
            CleanupAnchor anchor; memcpy(&anchor, artifact_cleanup.anchors.c_str() + at, sizeof(anchor));
            if (anchor.day != day) continue;
            bool duplicate = false;
            for (size_t n = sizeof(key); n < identity.length(); n += sizeof(anchor.digest))
                duplicate |= !memcmp(identity.c_str() + n, anchor.digest, sizeof(anchor.digest));
            if (!duplicate && !identity.append(reinterpret_cast<const char *>(anchor.digest), sizeof(anchor.digest))) return false;
        }
        if (!ReportArtifact::fingerprint(reinterpret_cast<const uint8_t *>(identity.c_str()), identity.length(), key)) return false;
        const auto *cached = cached_day_entry(day, false);
        const bool checked = cached && cached->cleanup_checked && !memcmp(cached->cleanup_key, key, sizeof(key));
        DayCleanup cleanup(session, day);
        const uint32_t started = millis();
        if (!checked) Log::logf(CAT_REPORT, LOG_DEBUG, "Report cleanup scanning day=%.8s\n",
            cleanup.directory + strlen(CACHE_DIR) + 1);
        const auto result = checked ? ReportArtifact::Lookup::Ready : cleanup.collect();
        if (cleanup.removed) {
            cache_usage_known = false;
            Log::logf(CAT_REPORT, LOG_INFO, "Report cleanup day=%.8s removed=%lu bytes=%llu\n",
                cleanup.directory + strlen(CACHE_DIR) + 1, static_cast<unsigned long>(cleanup.removed),
                static_cast<unsigned long long>(cleanup.removed_bytes));
        }
        if (result == ReportArtifact::Lookup::Failed) {
            Log::logf(CAT_REPORT, LOG_DEBUG, "Report cleanup deferred day=%.8s elapsed=%lums foreground=%u\n",
                cleanup.directory + strlen(CACHE_DIR) + 1, static_cast<unsigned long>(millis() - started),
                unsigned(foreground_wait() || __atomic_load_n(&artifact_reads, __ATOMIC_ACQUIRE)));
            return false;
        }
        if (auto *entry = cached_day_entry(day, true)) {
            memcpy(entry->cleanup_key, key, sizeof(key));
            entry->cleanup_checked = true;
        }
        if (result == ReportArtifact::Lookup::Invalid)
            Log::logf(CAT_REPORT, LOG_DEBUG, "Report cleanup preserved day=%.8s; replacement or references not verified\n",
                cleanup.directory + strlen(CACHE_DIR) + 1);
        artifact_cleanup.next += sizeof(day);
    }
    if (artifact_cleanup.next == artifact_cleanup.days.length()) {
        if (cleanup_cancelled(context)) return false;
        aircannect::LargeTextBuffer days(MAX_SESSION_COUNT * sizeof(uint16_t), 0), anchors(2 * MAX_BODY, 0);
        aircannect::LargeTextBuffer legacy(MAX_SESSION_COUNT * sizeof(CleanupLegacy), 0);
        days.swap(artifact_cleanup.days); anchors.swap(artifact_cleanup.anchors); legacy.swap(artifact_cleanup.legacy);
        artifact_cleanup.complete = true;
        if (session.valid()) session.run([](fs::FS &) { SdStorage::refresh_usage(); return true; });
    }
    return true;
}

bool dirty() {
    EdfCatalog::Status catalog;
    EdfCatalog::get_status(catalog);
    return !index_ready || __atomic_load_n(&invalidation_generation, __ATOMIC_ACQUIRE) != indexed_invalidation ||
        catalog.generation != catalog_generation ||
        SdStorage::files_revision(SdStorage::FileScope::Reports) != file_revision;
}

bool reindex(const SdStorage::Session &session, Job &job) {
    if (!dataset(session) || rebuild_generation == UINT32_MAX) return note_issue(job, "rebuild_generation");
    uint8_t value[4], key[32] = {};
    SdStorage::put_le32(value, rebuild_generation + 1);
    ReportArtifact::Document document;
    if (!ReportArtifact::document_publish(session, "/airbridge/report/rebuild", key, value, sizeof(value), document))
        return note_issue(job, "rebuild_publish");
    ++rebuild_generation;
    index_cleanup.files = UINT32_MAX;
    invalidate_day(); invalidate_str();
    backfill_revision = 0; backfill_before = UINT16_MAX; backfill_period = 0;
    indexed_invalidation = __atomic_add_fetch(&invalidation_generation, 1, __ATOMIC_RELEASE);
    job.data_revision = __atomic_add_fetch(&data_revision, 1, __ATOMIC_RELEASE);
    published_index_data = 0;
    portENTER_CRITICAL(&status_mux);
    status.data_revision = job.data_revision;
    ++status.artifact_revision;
    ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    job.body.print("{\"source\":\"edf\",\"rebuilding\":true,\"data_revision\":%lu}",
        static_cast<unsigned long>(job.data_revision));
    Log::logf(CAT_REPORT, LOG_INFO, "Report rebuild queued generation=%lu; previous publications preserved\n",
        static_cast<unsigned long>(rebuild_generation));
    return job.body.good();
}

void defer_job(Job *job, const char *reason) {
    if (!job) return;
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (job->cancelled) reset_job(*job);
    else {
        job->wait_reason = reason;
        portENTER_CRITICAL(&status_mux);
        job->state = JobState::Queued;
        portEXIT_CRITICAL(&status_mux);
    }
    xSemaphoreGive(mutex);
    observe_report_jobs(true);
}

bool enqueue_backfill(const SdStorage::Session &session) {
    if (!backfill_enabled) return false;
    if (background_retry && int32_t(millis() - background_retry) < 0) return false;
    if (background_retry) { background_retry = 0; backfill_before = UINT16_MAX; backfill_period = 0; }
    const uint32_t current = __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
    if (backfill_revision != current) { backfill_revision = current; backfill_before = UINT16_MAX; backfill_period = 0; }
    uint16_t day = 0;
    for (size_t i = 0; i < index_count; ++i)
        if (selected(index[i], 1, backfill_before - (backfill_before != 0))) day = std::max(day, index[i].day);
    if (!day) {
        backfill_before = 0;
        if (!last_day(Request{})) backfill_period = sizeof(PERIODS) / sizeof(PERIODS[0]);
    }
    if (!day && backfill_period >= sizeof(PERIODS) / sizeof(PERIODS[0])) return false;
    if (!dataset(session)) return false;
    Request request; request.action = Action::Prepare; request.day = day;
    char prefix[96]; uint8_t key[32];
    if (day) publication_prefix(day, prefix);
    else {
        request.view = View::Period; request.period = PERIODS[backfill_period];
        period_prefix(request, prefix);
    }
    if (!publication_key(request, key)) return false;
    ReportArtifact::Document document;
    const bool ready = lookup_publication(session, last_day(request), day ? 0 : request.period,
        prefix, document) == ReportArtifact::Lookup::Ready && !memcmp(key, document.key, sizeof(key));
    bool refused = false;
    if (!ready) {
        strcat(prefix, ".refused");
        refused = ReportArtifact::document_lookup(session, prefix, key, document) == ReportArtifact::Lookup::Ready;
    }
    if (ready || refused) {
        if (day) backfill_before = day; else ++backfill_period;
        return true;
    }
    xSemaphoreTake(mutex, portMAX_DELAY);
    Job *slot = nullptr;
    size_t active = 0;
    bool pending = false;
    for (auto &candidate : jobs) {
        if (candidate.state == JobState::Queued || candidate.state == JobState::Running) {
            ++active;
            pending |= candidate.request == request;
        }
        if (!slot && (reusable(candidate) || evictable(candidate))) slot = &candidate;
    }
    if (pending || active >= SLOT_COUNT) slot = nullptr;
    if (slot) {
        reset_job(*slot);
        slot->request = request;
        slot->id = next_id++;
        if (!next_id) next_id = 1;
        slot->state = JobState::Queued;
        slot->progress.check(millis());
        slot->background = true;
        slot->foreground_epoch = __atomic_load_n(&foreground_epoch, __ATOMIC_ACQUIRE);
        slot->consumers = 0;
        admit_job(*slot);
    }
    xSemaphoreGive(mutex);
    return slot != nullptr;
}

void finish_maintenance_progress() {
    for (auto &work : maintenance_progress) {
        if (!work.active) continue;
        const bool complete = work.complete || (work.phase == MaintenancePhase::Index ? !dirty() :
            work.phase == MaintenancePhase::Publish ? !index_commit_pending && !retired_index.bytes &&
                published_index_data == __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE) &&
                published_index_artifacts == status.artifact_revision :
            work.phase == MaintenancePhase::IndexGc ? !index_cleanup_pending() : !artifact_cleanup_pending());
        if (complete) observe_maintenance_progress(work, false, true);
    }
}

void maintenance_wait(const char *reason) {
    for (auto &work : maintenance_progress)
        if (work.active && !work.complete) work.wait_reason = reason;
}

bool maintenance_work(SdStorage::Session &session, MaintenancePhase phase,
                      bool (*operation)(const SdStorage::Session &)) {
    auto &work = maintenance_progress[size_t(phase)];
    finish_maintenance_progress();
    // Retain each phase across interleaved retries. A pinned collector must not
    // pause index publication or inherit its progress and stall warnings.
    if (!work.active) {
        work = {};
        work.phase = phase;
        work.active = true;
    }
    // Count completed steps within this retained phase/cursor, not attempts.
    // Replaying a failed publication's prefix cannot advance its high-water mark.
    work.cursor = maintenance_cursor(phase);
    work.steps = 0;
    work.wait_reason = "working";
    work.watch.attempt();
    observe_maintenance_progress(work, true);
    maintenance_io = &work;
    work.observing = true;
    const bool ok = operation(session);
    work.observing = false;
    if (!ok && session.yield_requested()) work.wait_reason = "storage_busy";
    work.complete = ok && (phase == MaintenancePhase::IndexGc ? !index_cleanup_pending() :
        phase == MaintenancePhase::ArtifactGc ? !artifact_cleanup_pending() : true);
    observe_maintenance_progress(work);
    return ok;
}

bool work_once() {
    static const char *reported_index_error = nullptr;
    // Keep a refused admission's FIFO ticket across worker retries.
    static SdStorage::Session session;
    session.set_admission_notify([](void *) { if (wake) xSemaphoreGive(wake); });
    maintenance_io = nullptr;
    finish_maintenance_progress();
    observe_report_jobs(true);
    Job *foreground = nullptr, *days = nullptr, *backfill = nullptr;
    xSemaphoreTake(mutex, portMAX_DELAY);
    for (auto &candidate : jobs)
        if (candidate.state != JobState::Free && reusable(candidate)) reset_job(candidate);
    // Scheduling and queued promotion must agree before a job starts running.
    // Buffer destruction above stays outside this short admission boundary.
    portENTER_CRITICAL(&status_mux);
    for (auto &candidate : jobs) {
        if (candidate.state != JobState::Queued) continue;
        Job *&oldest = candidate.background ? backfill :
            candidate.request.action == Action::Days ? days : foreground;
        if (!oldest || int32_t(candidate.id - oldest->id) < 0) oldest = &candidate;
    }
    // Serve foreground promptly, but reserve a useful bounded backfill turn
    // after a burst. SD ownership and peer turns remain SdStorage's decision.
    const bool backfill_pending = backfill_enabled &&
        (!background_retry || int32_t(millis() - background_retry) >= 0) &&
        (backfill_revision != __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE) || backfill_before ||
         backfill_period < sizeof(PERIODS) / sizeof(PERIODS[0]));
    if (!backfill && !backfill_pending) { backfill_due = false; backfill_work_ms = 0; }
    const bool discover_backfill = !backfill && backfill_pending && (foreground || days) &&
        (backfill_due || foreground_turns >= FOREGROUND_BURST);
    const bool protected_backfill = (foreground || days) && backfill &&
        (backfill_due || foreground_turns >= FOREGROUND_BURST);
    const bool protected_foreground = days && foreground && days_turns >= FOREGROUND_BURST;
    Job *job = discover_backfill ? nullptr : protected_backfill ? backfill : protected_foreground ? foreground :
        days ? days : foreground ? foreground : backfill;
    if (job) {
        job->state = JobState::Running;
        job->protected_turn = protected_backfill || protected_foreground;
        job->turn_budget_ms = protected_backfill ? BACKGROUND_TURN_MS - backfill_work_ms : DAYS_PROTECTED_TURN_MS;
        job->backfill_waiting = (backfill || backfill_pending) && !job->background;
        job->foreground_epoch = __atomic_load_n(&foreground_epoch, __ATOMIC_ACQUIRE);
        job->days_epoch = __atomic_load_n(&days_epoch, __ATOMIC_ACQUIRE);
        job->storage_yielded = false;
    }
    portEXIT_CRITICAL(&status_mux);
    xSemaphoreGive(mutex);
    const char *blocked = !AirSenseState::local_background_allowed() ? "not_idle" :
        !SdStorage::local_access_allowed() ? "usb_storage" : !SdStorage::mounted() ? "no_sd" : nullptr;
    if (blocked) {
        session.end();
        maintenance_wait(blocked);
        observe_report_jobs(true);
        defer_job(job, blocked);
        publish(State::Blocked, blocked);
        return false;
    }
    if (!job && !dirty() && !index_commit_pending && !retired_index.bytes && !index_cleanup_pending() && !artifact_cleanup_pending() && published_index_artifacts == status.artifact_revision &&
        (!backfill_enabled || (!backfill_before && backfill_period >= sizeof(PERIODS) / sizeof(PERIODS[0])) ||
        (background_retry && int32_t(millis() - background_retry) < 0))) { session.end(); publish(State::Ready); return false; }
    session.set_io_observer(observe_report_io, job);
    if (!session.begin()) {
        maintenance_wait("storage_busy");
        observe_report_jobs(true);
        defer_job(job, "storage_busy");
        publish(State::WaitingStorage, "storage_busy");
        return false;
    }
    maintenance_wait("working");
    if (job) {
        if (!job->background && foreground_turns < FOREGROUND_BURST) ++foreground_turns;
        if (!job->background && job->request.action != Action::Days) days_turns = 0;
        else if (job->request.action == Action::Days && days_turns < FOREGROUND_BURST) ++days_turns;
        job->turn_started_ms = millis();
        job->turn_progress_set = false;
        job->progress.attempt();
        job->wait_reason = "working";
        job->indexing = true;
        job->files_revision = SdStorage::files_revision(SdStorage::FileScope::Reports);
        job->recovered_index = false;
    }
    JobTiming timing(job);
    const auto finish_backfill_turn = [&](bool complete = false) {
        if (!protected_backfill) return;
        // SD peers can interrupt even a reserved report turn. Charge its
        // admitted time across retries, including a grant lost to reopen work.
        backfill_work_ms += std::min(BACKGROUND_TURN_MS, uint32_t(millis() - job->turn_started_ms));
        if (complete || backfill_work_ms >= BACKGROUND_TURN_MS) {
            foreground_turns = 0;
            backfill_due = false;
            backfill_work_ms = 0;
        }
    };
    publish(State::Building);
    const uint32_t index_started = timing.stamp();
    bool recovered_index = false;
    if (job && job->request.action == Action::Days && !index_ready) {
        ReportArtifact::Document document;
        Payload saved;
        if (ReportArtifact::document_lookup(session, "/airbridge/report/index", nullptr, document) == ReportArtifact::Lookup::Ready &&
            ReportArtifact::document_read(session, "/airbridge/report/index", document, saved, nullptr) &&
            saved.length()) {
            aircannect::JsonAllocator allocator(false);
            JsonDocument parsed(&allocator);
            if (!deserializeJson(parsed, saved.c_str(), saved.length()) && parsed["source"] == "edf" &&
                parsed["days"].is<JsonArrayConst>() && parsed["periods"].is<JsonArrayConst>() &&
                aircannect::sha256_text_valid(parsed["dataset"].as<const char *>())) {
                parsed["stale"] = true;
                parsed["revision"] = revision() + 1;
                parsed["data_revision"] = __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
                const size_t written = serializeJson(parsed, job->body);
                recovered_index = job->body.good() && written == measureJson(parsed);
            }
            if (!recovered_index) job->body.clear();
        }
    }
    const bool observe_index = !job && (dirty() || maintenance_progress[size_t(MaintenancePhase::Index)].active);
    const bool indexed = recovered_index || (observe_index ?
        maintenance_work(session, MaintenancePhase::Index,
            [](const SdStorage::Session &source) { return refresh_index(source); }) : refresh_index(session, job));
    observe_report_jobs(true);
    if (job) job->indexing = false;
    timing.index_us = timing.stamp() - index_started;
    if (!indexed) {
        const bool preempted = !session.valid() || !AirSenseState::local_background_allowed();
        const bool yielding = session.yield_requested();
        if (preempted || yielding || index_failure == IndexFailure::Preempted)
            maintenance_wait(preempted ? "not_idle" : "storage_busy");
        else if (index_failure == IndexFailure::WaitingCatalog) maintenance_wait("catalog_not_ready");
        session.end();
        timing.finish(preempted || yielding || index_failure == IndexFailure::Preempted);
        finish_backfill_turn();
        if (preempted || yielding || index_failure == IndexFailure::Preempted || index_failure == IndexFailure::WaitingCatalog) {
            defer_job(job, preempted ? "not_idle" : yielding || index_failure == IndexFailure::Preempted ?
                "storage_busy" : "catalog_not_ready");
            publish(preempted ? State::Blocked : State::WaitingStorage,
                    preempted ? "not_idle" : yielding || index_failure == IndexFailure::Preempted ? "storage_busy" : "catalog_not_ready");
        } else {
            Log::logf(CAT_REPORT, reported_index_error == index_error ? LOG_DEBUG : LOG_WARN,
                "EDF index failed: %s entry=%ld\n", index_error,
                index_error_entry == SIZE_MAX ? -1L : long(index_error_entry));
            reported_index_error = index_error;
            if (job) {
                job->build.reset();
                clear_assembly(*job);
                xSemaphoreTake(mutex, portMAX_DELAY);
                job->data_revision = __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
                error_body(*job, job->cancelled ? 410 : 503, job->cancelled ? "cancelled" : "index_unavailable");
                job->completed_ms = millis();
                portENTER_CRITICAL(&status_mux);
                job->state = JobState::Complete;
                portEXIT_CRITICAL(&status_mux);
                log_result(*job);
                timing.log();
                xSemaphoreGive(mutex);
                portENTER_CRITICAL(&status_mux);
                ++status.revision;
                portEXIT_CRITICAL(&status_mux);
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
        if (session.yield_requested()) {
            maintenance_wait("storage_busy");
            session.end();
            publish(State::WaitingStorage, "storage_busy");
            return false;
        }
        if (discover_backfill) {
            // Discover one candidate with the existing owner path, even while
            // one long foreground job remains queued. No extra job slot.
            const bool more = enqueue_backfill(session);
            if (!more) {
                // A matching foreground job already serves this candidate, or
                // both work slots are occupied. Let that work advance first.
                backfill_due = false;
                backfill_work_ms = 0;
                foreground_turns = 0;
            }
            session.end();
            publish(State::Building);
            return true;
        }
        const auto checkpoint_index = [&]() {
            const bool pending = index_commit_pending || retired_index.bytes ||
                published_index_data != __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE) ||
                published_index_artifacts != status.artifact_revision;
            return pending ? maintenance_work(session, MaintenancePhase::Publish, publish_index) : publish_index(session);
        };
        const bool checkpoint = index_commit_pending;
        if (checkpoint && !checkpoint_index())
            Log::logf(CAT_REPORT, LOG_DEBUG, "Report index publication deferred\n");
        const bool more = enqueue_backfill(session);
        // Backfill publishes each day independently. Persist the derived list
        // only when this run finishes or safely pauses, not after every day.
        if (!more && !checkpoint && !checkpoint_index())
            Log::logf(CAT_REPORT, LOG_DEBUG, "Report index publication deferred\n");
        const bool cleaning = !more && !retired_index.bytes &&
            published_index_artifacts == status.artifact_revision && index_cleanup_pending() &&
            maintenance_work(session, MaintenancePhase::IndexGc, cleanup_indexes);
        const bool collecting = !more && !retired_index.bytes && !index_cleanup_pending() &&
            published_index_artifacts == status.artifact_revision && artifact_cleanup_pending() &&
            maintenance_work(session, MaintenancePhase::ArtifactGc, cleanup_artifacts);
        session.end();
        finish_maintenance_progress();
        publish(State::Ready);
        return more || (cleaning && index_cleanup_pending()) || (collecting && artifact_cleanup_pending());
    }
    xSemaphoreTake(mutex, portMAX_DELAY);
    const uint32_t current_data = __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE);
    if (job->data_revision != current_data) {
        clear_assembly(*job);
        if (!recovered_index) job->body.clear();
    }
    job->data_revision = current_data;
    job->code = 0;
    job->error = job->issue = nullptr;
    job->issue_path[0] = 0;
    job->accepted_issue.restore(*job);
    job->recovered_index = recovered_index;
    xSemaphoreGive(mutex);
    bool success = false;
    if (__atomic_load_n(&job->cancelled, __ATOMIC_ACQUIRE)) error_body(*job, 410, "cancelled");
    else if (job->request.revision && job->request.revision != job->data_revision)
        error_body(*job, 409, "stale_revision");
    else {
        bool resumed = true;
        if (job->build) {
            Log::ScopedMicros measured(job->prepare_resume_us,
                job->diagnostics && job->request.action == Action::Prepare);
            uint64_t *phase_time = nullptr;
            if (job->preparation) switch (job->preparation->phase) {
                case PrepareWork::Numeric: phase_time = &job->prepare_numeric_us; break;
                case PrepareWork::Events: phase_time = &job->prepare_events_us; break;
                case PrepareWork::History: phase_time = &job->prepare_history_us; break;
                case PrepareWork::Summary: phase_time = &job->prepare_summary_us; break;
                case PrepareWork::Publish: phase_time = &job->prepare_publish_us; break;
            }
            Log::ScopedMicros resumed_phase(phase_time ? *phase_time : job->prepare_resume_us,
                job->diagnostics && job->request.action == Action::Prepare && phase_time);
            const IndexedSession *entry = nullptr;
            for (size_t i = 0; i < index_count; ++i)
                if (job->build->matches(index[i], job->build->artifacts)) { entry = &index[i]; break; }
            if (!entry) job->build.reset();
            else resumed = build_session(session, *job, *entry, *job->build);
        }
        // Resume before repeating day/publication lookups: small recurring SD
        // grants must advance the source, not be consumed by the same prelude.
        if (!resumed) success = false;
        else if (recovered_index) success = true;
        else if (job->request.action == Action::Reindex) success = reindex(session, *job);
        else if (job->request.action == Action::History) {
            if (!job->history) {
                start_json(job->body, revision() + 1, job->data_revision);
                job->body.print("\"day\":%u", last_day(job->request));
            }
            success = history_json(session, *job, true);
            if (success) job->body.print("}");
        } else if (job->request.action == Action::Prepare) success = prepare_json(session, *job);
        else success = job->request.action == Action::Days ? days_json(session, *job) : summary_json(session, *job);
        if (success) job->code = 200;
    }
    const bool interrupted = !AirSenseState::local_background_allowed() || !session.valid();
    EdfCatalog::Status catalog;
    EdfCatalog::get_status(catalog);
    const bool changed = SdStorage::files_revision(SdStorage::FileScope::Reports) != job->files_revision ||
        job->data_revision != __atomic_load_n(&data_revision, __ATOMIC_ACQUIRE) ||
        (!recovered_index && (catalog.generation != catalog_generation || SdStorage::files_revision(SdStorage::FileScope::Reports) != file_revision));
    const bool yielded = !success && (job->storage_yielded || session.yield_requested());
    if (success && !job->background && job->request.action == Action::Prepare &&
        !interrupted && !changed && !yielded) {
        Log::ScopedMicros measured(job->prepare_publish_us, job->diagnostics);
        // Includes post-therapy preparation before export. Failure to checkpoint
        // must not invalidate an independently committed day publication.
        index_commit_pending = true;
        if (!publish_index(session)) Log::logf(CAT_REPORT, LOG_DEBUG, "Report index publication deferred\n");
    }
    if (job->background && !interrupted && !changed && !yielded) {
        foreground_turns = 0;
        if (job->request.view == View::Period) ++backfill_period;
        else backfill_before = job->request.day;
        if (!success && job->issue && (!strcmp(job->issue, "partial_source") ||
            !strcmp(job->issue, "source_header") || !strcmp(job->issue, "artifact_lookup") ||
            !strcmp(job->issue, "numeric_files_missing"))) {
            char prefix[96]; uint8_t key[32]; ReportArtifact::Document refused;
            if (job->request.view == View::Period) period_prefix(job->request, prefix);
            else publication_prefix(job->request.day, prefix);
            strcat(prefix, ".refused");
            if (publication_key(job->request, key)) ReportArtifact::document_publish(session, prefix, key,
                reinterpret_cast<const uint8_t *>(job->issue), strlen(job->issue), refused);
        } else if (!success && (!job->issue || strcmp(job->issue, "recording_headroom"))) background_retry = millis() + 30000;
    }
    if (job->build) job->build->pause();
    if (interrupted || yielded) maintenance_wait(interrupted ? "not_idle" : "storage_busy");
    observe_report_jobs(true);
    session.end();
    finish_maintenance_progress();
    timing.finish(yielded || interrupted || changed);
    finish_backfill_turn(success);
    xSemaphoreTake(mutex, portMAX_DELAY);
    if (job->cancelled) error_body(*job, 410, "cancelled");
    else if (interrupted || changed || yielded) {
        if (!job->preparation && !job->summary && !job->history) {
            job->body.~Payload();
            new (&job->body) Payload;
        }
        job->code = 0;
        job->wait_reason = interrupted ? "not_idle" : changed ? "data_changed" : "storage_yield";
        portENTER_CRITICAL(&status_mux);
        if (job->consumers) job->background = false;
        job->state = JobState::Queued;
        portEXIT_CRITICAL(&status_mux);
        xSemaphoreGive(mutex);
        publish(interrupted ? State::Blocked : State::Building, interrupted ? "not_idle" : "");
        return yielded;
    } else if (!success && !job->code) error_body(*job, 503, "report_build_failed");
    if (success && !job->cancelled) report_progress(*job, {UINT64_MAX, 0}, false);
    job->build.reset();
    clear_assembly(*job);
    job->completed_ms = millis();
    job->cache_generation = __atomic_load_n(&cache_generation, __ATOMIC_ACQUIRE);
    job->artifact_revision = status.artifact_revision;
    portENTER_CRITICAL(&status_mux);
    job->state = JobState::Complete;
    portEXIT_CRITICAL(&status_mux);
    log_result(*job);
    timing.log();
    xSemaphoreGive(mutex);
    portENTER_CRITICAL(&status_mux);
    ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    publish(State::Ready);
    return true;
}

void worker(void *) {
    while (true) {
        if (work_once()) continue;
        Status publication;
        get_status(publication);
        bool pending = false;
        xSemaphoreTake(mutex, portMAX_DELAY);
        for (const auto &job : jobs) pending |= job.state == JobState::Queued;
        xSemaphoreGive(mutex);
        for (const auto &work : maintenance_progress)
            pending |= work.active && !maintenance_suspended(work.phase);
        const bool retry = background_retry || publication.state == State::WaitingStorage ||
            publication.state == State::Blocked || pending;
        const uint32_t quiet = publication.state == State::Ready ? foreground_wait() : 0;
        xSemaphoreTake(wake, retry ? pdMS_TO_TICKS(1000) : quiet ? pdMS_TO_TICKS(quiet) : portMAX_DELAY);
    }
}

void artifact_read_released() {
    foreground_activity();
    __atomic_sub_fetch(&artifact_reads, 1, __ATOMIC_ACQ_REL);
    if (wake) xSemaphoreGive(wake);
}

}  // namespace

void init() {
    if (mutex) return;
    const auto memory = aircannect::Memory::status();
    if (memory.psram_available) {
        auto *storage = static_cast<Job *>(aircannect::Memory::alloc_large(RETAINED_SLOTS * sizeof(Job), false));
        if (storage) {
            for (size_t i = 0; i < RETAINED_SLOTS; ++i) new (&storage[i]) Job;
            jobs.data = storage;
            jobs.count = RETAINED_SLOTS;
        }
    }
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
    aircannect::Memory::set_reclaimer(reclaim_result);
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
    const uint32_t catalog = EdfCatalog::revision(), files = SdStorage::files_revision(SdStorage::FileScope::Reports);
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
    for (auto &job : jobs) if (job.state != JobState::Free && reusable(job)) reset_job(job);
    xSemaphoreGive(mutex);
    // Arm on every contended read, including one after the completion event.
    // Publish only after taking and releasing the result mutex, so an SSE
    // receiver cannot consume its final wake while the same lock is held.
    portENTER_CRITICAL(&status_mux);
    if (jobs.delivery_waiting) {
        jobs.delivery_waiting = false;
        ++status.revision;
    }
    portEXIT_CRITICAL(&status_mux);
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
    const bool locked = mutex && xSemaphoreTake(mutex, 0) == pdTRUE;
    bool full = false;
    if (join_active(request, id, full)) {
        if (locked) xSemaphoreGive(mutex);
        return true;
    }
    if (!locked || full) {
        if (locked) xSemaphoreGive(mutex);
        return reject("busy");
    }
    EdfCatalog::Status catalog;
    EdfCatalog::get_status(catalog);
    const bool current_index = catalog.ready &&
        catalog.generation == publication.catalog_generation &&
        SdStorage::files_revision(SdStorage::FileScope::Reports) == publication.files_revision;
    Job *slot = nullptr, *delivered = nullptr;
    size_t active = 0;
    bool background_active = false, days_active = false;
    for (auto &job : jobs) {
        if (request.action != Action::Reindex && current_index &&
            !job.recovered_index && !job.cancelled && !job.issue && job.state == JobState::Complete && job.code == 200 &&
            job.data_revision == publication.data_revision && job.request == request &&
            (request.action != Action::Days || job.artifact_revision == publication.artifact_revision) &&
            ((request.action != Action::Summary && request.action != Action::History && request.action != Action::Days) ||
             job.cache_generation == __atomic_load_n(&cache_generation, __ATOMIC_ACQUIRE)) &&
            uint32_t(millis() - job.completed_ms) < RETAIN_MS) {
            if (job.consumers == UINT16_MAX) { xSemaphoreGive(mutex); return reject("busy"); }
            ++job.consumers;
            job.shared = true;
            job.completed_ms = millis();
            id = job.id;
            xSemaphoreGive(mutex);
            foreground_activity();
            portENTER_CRITICAL(&status_mux);
            ++status.revision;
            portEXIT_CRITICAL(&status_mux);
            return true;
        }
        if (job.state == JobState::Queued || job.state == JobState::Running) {
            ++active;
            portENTER_CRITICAL(&status_mux);
            background_active |= job.background;
            portEXIT_CRITICAL(&status_mux);
            days_active |= job.request.action == Action::Days;
        }
        if (reusable(job)) slot = &job;
        if (job.state == JobState::Complete && !job.readers && job.data_revision != publication.data_revision)
            slot = &job;
        if (evictable(job) && (!delivered ||
            int32_t(job.completed_ms - delivered->completed_ms) < 0)) delivered = &job;
    }
    // Retention is opportunistic: routine days->summary->series and repeated
    // zooms can reclaim delivered/unpinned slots without a browser delay timer.
    if (!slot) slot = delivered;
    // A backfill and a foreground build must leave room for calendar discovery.
    // Only one extra Days selector is admitted; normal work keeps its bound.
    const bool calendar_slot = request.action == Action::Days && active == SLOT_COUNT &&
        background_active && !days_active;
    if (!slot || (active >= SLOT_COUNT && !calendar_slot)) { xSemaphoreGive(mutex); return reject("busy"); }
    reset_job(*slot);
    slot->request = request;
    slot->id = next_id++;
    if (!next_id) next_id = 1;
    slot->state = JobState::Queued;
    slot->progress.check(millis());
    admit_job(*slot);
    __atomic_add_fetch(&foreground_epoch, 1, __ATOMIC_RELEASE);
    id = slot->id;
    xSemaphoreGive(mutex);
    foreground_activity();
    portENTER_CRITICAL(&status_mux);
    ++status.revision;
    portEXIT_CRITICAL(&status_mux);
    xSemaphoreGive(wake);
    return true;
}

uint32_t prepare_before_export(const char *therapy_day) {
    Request request; request.action = Action::Prepare;
    if (!therapy_day || !catalog_day(therapy_day, request.day)) return 0;
    uint32_t id = 0;
    return submit(request, id) ? id : 0;
}

bool preparation_pending(uint32_t id) {
    if (!id || !mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return false;
    bool pending = false;
    for (const auto &job : jobs) if (job.id == id)
        pending = job.state == JobState::Queued || job.state == JobState::Running;
    xSemaphoreGive(mutex);
    return pending;
}

void cancel(uint32_t id) {
    if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return;
    for (auto &job : jobs) {
        if (job.id != id) continue;
        portENTER_CRITICAL(&status_mux);
        const bool sole = !job.shared && !job.readers;
        if (sole) __atomic_store_n(&job.cancelled, true, __ATOMIC_RELEASE);
        portEXIT_CRITICAL(&status_mux);
        if (!sole) break;
        if (job.state == JobState::Queued || job.state == JobState::Complete) reset_job(job);
        break;
    }
    xSemaphoreGive(mutex);
    if (wake) xSemaphoreGive(wake);
}

StorageBrowser::StartResult open_artifact(const char *day, const char *digest,
    bool document, StorageBrowser::Request &operation, StorageBrowser::Ready ready,
    std::weak_ptr<StorageBrowser::Transfer> &active) {
    uint16_t parsed = 0;
    const bool root = day && !strcmp(day, "index");
    if ((!root && (!day || !catalog_day(day, parsed))) || !digest || strlen(digest) != 64)
        return StorageBrowser::StartResult::BadRequest;
    for (const char *p = digest; *p; ++p)
        if (!(*p >= '0' && *p <= '9') && !(*p >= 'a' && *p <= 'f')) return StorageBrowser::StartResult::BadRequest;
    if (root && !document) return StorageBrowser::StartResult::BadRequest;
    // Admission pins artifact graphs against GC, not unrelated report work.
    foreground_activity();
    __atomic_add_fetch(&artifact_reads, 1, __ATOMIC_ACQ_REL);
    operation.kind = StorageBrowser::Kind::Immutable;
    operation.selection[0] = 0;
    snprintf(operation.path, sizeof(operation.path), "%s/%s%s%.*s%s%s.%s", CACHE_DIR,
        root ? "" : day, root ? "" : "/", document ? 2 : 0, digest,
        document ? "/" : "", digest, document ? "abp" : "abt");
    const auto result = StorageBrowser::start(operation, std::move(ready), active, artifact_read_released);
    if (result != StorageBrowser::StartResult::Started) artifact_read_released();
    return result;
}

int poll(uint32_t id, Result &out) {
    out.reset();
    if (!mutex) return 503;
    if (xSemaphoreTake(mutex, 0) != pdTRUE) {
        bool retained = false;
        portENTER_CRITICAL(&status_mux);
        for (size_t i = 0; i < jobs.count; ++i) retained |= id && jobs.retained_ids[i] == id;
        if (retained) jobs.delivery_waiting = true;
        portEXIT_CRITICAL(&status_mux);
        return retained ? 202 : 410;
    }
    int code = 410;
    for (auto &job : jobs) {
        if (job.id != id || job.state == JobState::Free) continue;
        if (job.state != JobState::Complete) { code = 202; break; }
        Status publication;
        get_status(publication);
        if (job.code == 200 && (!SdStorage::local_access_allowed() || !SdStorage::mounted() || !publication.mounted ||
            job.data_revision != publication.data_revision ||
            SdStorage::files_revision(SdStorage::FileScope::Reports) != job.files_revision)) { code = 409; break; }
        EdfCatalog::Status catalog;
        EdfCatalog::get_status(catalog);
        // A saved index is explicitly stale, but usable before the RAM index
        // exists. Validate its own read generation, not uninitialized metadata.
        if (job.code == 200 && !job.recovered_index && catalog.generation != publication.catalog_generation) { code = 202; break; }
        if (!job.body.length() || job.readers == UINT16_MAX) { code = 503; break; }
        code = job.code;
        ++job.readers;
        if (job.deliveries < job.consumers) ++job.deliveries;
        job.delivered = true;
        out.id_ = job.id;
        out.readers_ = &job.readers;
        out.data_ = job.body.c_str();
        out.length_ = job.body.length();
        break;
    }
    xSemaphoreGive(mutex);
    return code;
}

void Result::reset() {
    if (!id_) return;
    const uint32_t released = std::exchange(id_, 0);
    auto *readers = std::exchange(readers_, nullptr);
    data_ = nullptr;
    length_ = 0;
    readers->fetch_sub(1, std::memory_order_release);
    if (xSemaphoreTake(mutex, 0) == pdTRUE) {
        for (auto &job : jobs) if (job.id == released) {
            if (reusable(job)) reset_job(job);
            break;
        }
        xSemaphoreGive(mutex);
    }
    // Collection may be asleep solely because this response pinned selectors.
    foreground_activity();
    if (wake) xSemaphoreGive(wake);
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
uint32_t prepare_before_export(const char *) { return 0; }
bool preparation_pending(uint32_t) { return false; }
StorageBrowser::StartResult open_artifact(const char *, const char *, bool, StorageBrowser::Request &,
    StorageBrowser::Ready, std::weak_ptr<StorageBrowser::Transfer> &) {
    return StorageBrowser::StartResult::Unavailable;
}
int poll(uint32_t, Result &out) { out.reset(); return 422; }
void Result::reset() { id_ = 0; readers_ = nullptr; data_ = nullptr; length_ = 0; }
}  // namespace EdfReport
#endif
