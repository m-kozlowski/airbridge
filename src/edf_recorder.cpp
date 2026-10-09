#include "edf_recorder.h"
#include "air10_stream.h"
#include "hex_util.h"

#include "board.h"

#if AB_STORAGE_HAS_SDCARD

#include <Arduino.h>
#include <FS.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "air10_edf.h"
#include "air10_stored.h"
#include "air10_str_timeline.h"
#include "air10_clock.h"
#include "edf_pending.h"
#include "crc.h"
#include "debug_log.h"
#include "edf_catalog.h"
#include "export_sync.h"
#include "live_stream.h"
#include "memory_manager.h"
#include "oxi_arbiter.h"
#include "qframe.h"
#include "sd_storage.h"
#include "uart_arbiter.h"
#include "airsense_state.h"

namespace EdfRecorder {
namespace {

constexpr uint16_t RAW_QUEUE_CAPACITY_PSRAM = 128;
constexpr uint16_t RAW_QUEUE_CAPACITY_FALLBACK = 48;
constexpr uint16_t RAW_PAYLOAD_MAX = 64;
constexpr uint8_t STREAM_COUNT = Air10Stream::SCHEMA_COUNT;
constexpr uint16_t RECORDER_STACK = 8192;
constexpr uint16_t POLL_TIMEOUT_MS = 200;
constexpr uint8_t PBT_SAMPLE_COUNT = 4;
constexpr uint8_t BRH_SAMPLE_COUNT = 4;
constexpr uint16_t PBT_SAMPLE_WINDOW_MS = 200;
constexpr uint16_t PLD_SAMPLE_WINDOW_MS = 500;
constexpr uint16_t MKP_WINDOW_MS = 10000;
// Ten seconds at 25 Hz, plus one PLD collection window and boundary samples.
constexpr uint16_t MKP_HISTORY_COUNT = (MKP_WINDOW_MS + 2 * PLD_SAMPLE_WINDOW_MS) / 40 + 2;
constexpr uint16_t RECORDING_STATE_POLL_MS = 1000;
constexpr uint16_t STORED_TIMEOUT_MS = 2000;
constexpr uint8_t STORED_TRANSFER_ATTEMPTS = 3;
constexpr uint16_t STR_GENERATION_POLL_MS = 1000;
constexpr int16_t EDF_MISSING = -1;
constexpr size_t RECOVERY_HEADER_MAX = 8192;
constexpr size_t RECOVERY_BUFFER_SIZE = 4096;
constexpr size_t IDENTIFICATION_BUFFER_SIZE = 1024;

const char *const MONTH_NAMES[] = {
    "JAN", "FEB", "MAR", "APR", "MAY", "JUN",
    "JUL", "AUG", "SEP", "OCT", "NOV", "DEC",
};

struct RawFrame {
    uint32_t captured_ms;
    uint16_t len;
    uint8_t payload[RAW_PAYLOAD_MAX];
};

enum class ControlKind : uint8_t {
    Start,
    Stop,
};

struct ControlEvent {
    ControlKind kind;
    uint32_t captured_ms;
};

using StreamSchema = Air10Stream::Schema;
using DecodedFrame = Air10Stream::Frame;
using Air10Stream::schema_has_field;
using Air10Stream::decoded_value;
using Air10Stream::decoded_value32;

struct TimedFrame {
    uint32_t received_ms;
    DecodedFrame frame;
};

struct PressureSample {
    uint32_t sample_ms;
    int16_t value;
};

struct OutputFile {
    fs::File file;
    const Air10Edf::Schema *schema;
    uint32_t records;
    uint32_t rest_crc;
    bool created;
    bool failed;
    bool open;
};

struct Accumulator {
    OutputFile output;
    int16_t *samples;
    size_t sample_count;
    uint32_t current_record;
    bool initialized;
};

struct WaveClock {
    bool initialized;
    uint8_t last_sequence;
    uint32_t relative_ms;
};

static Status status = {true};
static portMUX_TYPE status_mux = portMUX_INITIALIZER_UNLOCKED;

static QueueHandle_t raw_queue = nullptr;
static QueueHandle_t control_queue = nullptr;
static StaticQueue_t raw_queue_state;
static uint8_t *raw_queue_storage = nullptr;
static uint16_t raw_queue_capacity = 0;
static TaskHandle_t recorder_task_handle = nullptr;
static uart_frame_listener_t frame_listener = -1;
static volatile bool capture_active = false;
static volatile bool therapy_start_pending = false;
static volatile bool therapy_wanted = false;
static uint32_t therapy_on_capture_ms = 0;
static uint32_t recording_therapy_on_ms = 0;
static ControlEvent latest_stop = {};
static uint32_t next_start_ms = 0;
static uint32_t next_recording_state_ms = 0;
static bool storage_ready = false;
static uint32_t next_storage_ms = 0;

static fs::FS *storage = nullptr;
static bool recording_storage_owned = false;
static uint32_t next_pending_ms = 0;
static bool pending_scanned = false;
static bool clock_write_active = false;
static const char *pending_scan_error = nullptr;
static uint32_t synced_device_generation = 0;
static uint32_t synced_str_generation = 0;
static uint16_t synced_saved_day = 0;
static bool have_synced_generation = false;
static bool summary_export_pending = false;
static EdfCatalog::Entry summary_export_entry = {};
static char pending_cursor[16] = {};
static StreamSchema stream_schemas[STREAM_COUNT];
static LiveStream::internal_handle_t stream_leases[STREAM_COUNT];
static constexpr char wave_tag[] = "TCE";

static Accumulator brp;
static Accumulator pld;
static Accumulator sad;
static Air10Edf::SignalSpec pld_signals[Air10Edf::PLD_MAX_SIGNALS];
static Air10Edf::Schema pld_layout;
static OutputFile eve;
static OutputFile csl;
static uint8_t *header_buffer = nullptr;
static size_t header_capacity = 0;

static uint32_t segment_duration_ms = 0;
static Air10Clock::Anchor session_clock;
static Air10Clock::PhaseAnchor session_phase;
static uint16_t session_native_day = 0;
static char recording_id[81] = {};
static char start_date[9] = {};
static char start_time[9] = {};
static char session_directory[40] = {};
static AirSenseState::Identity recording_identity;
static bool identification_verified = false;
static uint32_t identification_device = 0;
static uint32_t identification_crc = 0;

static WaveClock wave_clock = {};
static TimedFrame pbt_samples[PBT_SAMPLE_COUNT];
static uint8_t pbt_sample_count = 0;
static uint8_t pbt_next_sample = 0;
static TimedFrame brh_samples[BRH_SAMPLE_COUNT];
static uint8_t brh_sample_count = 0;
static uint8_t brh_next_sample = 0;
static PressureSample *mkp_history = nullptr;
static uint16_t mkp_count = 0, mkp_next = 0;
static uint32_t last_pld_slot = UINT32_MAX;
static uint32_t last_oxi_slot = UINT32_MAX;

static void status_error(const char *message, const char *file = nullptr) {
    portENTER_CRITICAL(&status_mux);
    status.write_errors++;
    strncpy(status.last_error, message ? message : "error",
            sizeof(status.last_error) - 1);
    status.last_error[sizeof(status.last_error) - 1] = 0;
    portEXIT_CRITICAL(&status_mux);
    Log::logf(CAT_EDF, LOG_ERROR, "%s%s%s\n",
              file ? file : "", file ? ": " : "", message ? message : "error");
}

static bool post_error_active = false;

static void post_error(const char *message) {
    portENTER_CRITICAL(&status_mux);
    bool changed = !post_error_active || strcmp(status.last_error, message ? message : "post-processing error") != 0;
    post_error_active = true;
    status.post_errors++;
    strncpy(status.last_error, message ? message : "post-processing error",
            sizeof(status.last_error) - 1);
    status.last_error[sizeof(status.last_error) - 1] = 0;
    portEXIT_CRITICAL(&status_mux);
    Log::logf(CAT_EDF, changed ? LOG_ERROR : LOG_DEBUG, "%s\n",
              message ? message : "post-processing error");
}

static bool post_processing_cancelled() {
    return !SdStorage::local_access_allowed() || !AirSenseState::device_standby() ||
           __atomic_load_n(&therapy_start_pending, __ATOMIC_ACQUIRE) ||
           __atomic_load_n(&therapy_wanted, __ATOMIC_ACQUIRE);
}

static bool parse_hex_value(const char *text, size_t width, uint32_t &value) {
    return aircannect::parse_hex(text, width, value);
}

static bool parse_decimal_field(const uint8_t *text, size_t width,
                                uint32_t &value) {
    value = 0;
    bool found_digit = false;
    for (size_t i = 0; i < width; i++) {
        if (text[i] == ' ') continue;
        if (text[i] < '0' || text[i] > '9') return false;
        const uint32_t digit = text[i] - '0';
        if (value > (UINT32_MAX - digit) / 10) return false;
        value = value * 10 + digit;
        found_digit = true;
    }
    return found_digit;
}

static bool has_suffix(const char *text, const char *suffix) {
    if (!text || !suffix) return false;
    const size_t text_len = strlen(text);
    const size_t suffix_len = strlen(suffix);
    return text_len >= suffix_len &&
           strcmp(text + text_len - suffix_len, suffix) == 0;
}

static bool command_value(const char *command, char *out, size_t capacity,
                          uint16_t timeout_ms = POLL_TIMEOUT_MS) {
    if (!command || !out || capacity < 2) return false;
    char response[160] = {};
    uint16_t response_len = sizeof(response);
    if (!Arbiter::send_cmd(command, CMD_SRC_INTERNAL, CMD_PRIO_LOW,
                           response, &response_len, timeout_ms)) {
        return false;
    }
    const char *value = qframe_response_value(response);
    if (!value) return false;
    strncpy(out, value, capacity - 1);
    out[capacity - 1] = 0;
    return true;
}

static bool read_stored_value(const char *tag, uint16_t epoch_day,
                              Air10Stored::Value &value,
                              uint8_t attempts = STORED_TRANSFER_ATTEMPTS) {
    for (uint8_t attempt = 0; attempt < attempts; attempt++) {
        if (post_processing_cancelled()) return false;
        auto result = Air10Stored::read(tag, epoch_day, value, STORED_TIMEOUT_MS, true);
        if (result != Air10Stored::ReadResult::Failed) return true;
        if (attempt + 1 < attempts) {
            Log::logf(CAT_EDF, LOG_DEBUG,
                      "STR retry %s day=%04X attempt=%u/%u\n",
                      tag, epoch_day, attempt + 2, attempts);
        }
    }
    return false;
}

static bool read_numeric_variable(const char *name, int16_t &value,
                                  uint32_t slot, uint32_t target_ms,
                                  uint8_t &queries) {
    const int32_t offset = static_cast<int32_t>(millis() - target_ms);
    if (offset >= PLD_SAMPLE_WINDOW_MS) {
        Log::logf(CAT_EDF, LOG_DEBUG,
                  "PLD #%s slot=%lu skipped offset=%ldms\n",
                  name, (unsigned long)slot, (long)offset);
        return false;
    }
    uint32_t parsed = 0;
    Arbiter::VarReadTrace trace;
    queries++;
    const uint16_t remaining_ms = PLD_SAMPLE_WINDOW_MS - offset;
    const auto result = Arbiter::read_var_hex(name, CMD_SRC_INTERNAL, CMD_PRIO_LOW,
                                             parsed, POLL_TIMEOUT_MS, &trace, remaining_ms);
    if (result != Arbiter::VarResult::Ok || parsed > INT16_MAX) {
        // Temporary capture of PLD failures, without logging live UART frames.
        Log::logf(CAT_EDF, LOG_WARN,
                  "PLD #%s result=%s queue=%lums sent=%u wait=%lums t=%lu\n",
                  name, result == Arbiter::VarResult::Ok ? "out_of_range" : trace.outcome,
                  static_cast<unsigned long>(trace.queue_ms), unsigned(trace.sent),
                  static_cast<unsigned long>(trace.wait_ms), millis());
        return false;
    }
    const int32_t sent_offset = static_cast<int32_t>(trace.sent_ms - target_ms);
    const int32_t received_offset = static_cast<int32_t>(trace.received_ms - target_ms);
    if (sent_offset < -int32_t(PLD_SAMPLE_WINDOW_MS) ||
        received_offset > PLD_SAMPLE_WINDOW_MS) {
        Log::logf(CAT_EDF, LOG_DEBUG,
                  "PLD #%s slot=%lu discarded tx_offset=%ldms rx_offset=%ldms\n",
                  name, (unsigned long)slot, (long)sent_offset, (long)received_offset);
        return false;
    }
    value = static_cast<int16_t>(parsed);
    Log::logf(CAT_EDF, LOG_DEBUG,
              "PLD #%s slot=%lu raw=%d tx=%lu rx=%lu\n", name,
              (unsigned long)slot, int(value),
              (unsigned long)trace.sent_ms, (unsigned long)trace.received_ms);
    return true;
}

static bool read_u32_variable(const char *name, uint32_t &value) {
    return Arbiter::read_var_hex(name, CMD_SRC_INTERNAL, CMD_PRIO_LOW,
                                 value, POLL_TIMEOUT_MS) == Arbiter::VarResult::Ok;
}

static StreamSchema *find_schema(const char *tag) {
    for (StreamSchema &schema : stream_schemas) {
        if (schema.field_count && memcmp(schema.tag, tag, 3) == 0)
            return &schema;
    }
    return nullptr;
}

static void epoch_day_to_civil(uint16_t epoch_day, int &year,
                               unsigned &month, unsigned &day) {
    int z = static_cast<int>(epoch_day) + 719468;
    const int era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned day_of_era = static_cast<unsigned>(z - era * 146097);
    const unsigned year_of_era =
        (day_of_era - day_of_era / 1460 + day_of_era / 36524 -
         day_of_era / 146096) / 365;
    year = static_cast<int>(year_of_era) + era * 400;
    const unsigned day_of_year =
        day_of_era - (365 * year_of_era + year_of_era / 4 -
                      year_of_era / 100);
    const unsigned month_prime = (5 * day_of_year + 2) / 153;
    day = day_of_year - (153 * month_prime + 2) / 5 + 1;
    month = month_prime + (month_prime < 10 ? 3 : -9);
    year += month <= 2;
}

static bool publish_single_file(const char *partial_path,
                                const char *final_path,
                                const char *backup_path) {
    if (storage->exists(backup_path) && !storage->remove(backup_path))
        return false;

    const bool had_final = storage->exists(final_path);
    if (had_final && !storage->rename(final_path, backup_path)) return false;
    if (!storage->rename(partial_path, final_path)) {
        if (had_final) (void)storage->rename(backup_path, final_path);
        return false;
    }
    if (had_final && !storage->remove(backup_path)) {
        Log::logf(CAT_EDF, LOG_WARN,
                  "could not remove metadata backup %s\n", backup_path);
    }
    return true;
}

static void pending_path(const char *prefix, char *path, size_t size) {
    snprintf(path, size, "/airbridge/pending/%s.str", prefix);
}

enum class PendingRead { Ready, Invalid, Unavailable };

static PendingRead read_pending(const char *path, EdfPending::Record &record) {
    fs::File file = storage->open(path, FILE_READ);
    if (!file || file.isDirectory()) return PendingRead::Unavailable;
    uint8_t bytes[EdfPending::WIRE_SIZE];
    if (file.size() != sizeof(bytes)) return PendingRead::Invalid;
    if (file.read(bytes, sizeof(bytes)) != sizeof(bytes)) return PendingRead::Unavailable;
    return EdfPending::decode(bytes, sizeof(bytes), record)
        ? PendingRead::Ready : PendingRead::Invalid;
}

static bool write_pending(const EdfPending::Record &record) {
    if ((!storage->exists("/airbridge") && !storage->mkdir("/airbridge")) ||
        (!storage->exists("/airbridge/pending") && !storage->mkdir("/airbridge/pending")))
        return false;
    char path[80], part[88], backup[88];
    pending_path(record.prefix, path, sizeof(path));
    snprintf(part, sizeof(part), "%s.part", path);
    snprintf(backup, sizeof(backup), "%s.bak", path);
    uint8_t bytes[EdfPending::WIRE_SIZE];
    EdfPending::encode(record, bytes);
    storage->remove(part);
    fs::File file = storage->open(part, FILE_WRITE);
    if (!file || !SdStorage::write_exact(file, bytes, sizeof(bytes))) return false;
    file.flush(); file.close();
    return publish_single_file(part, path, backup);
}

static bool save_pending(const AirSenseState::Identity &identity,
                          const char *prefix = nullptr, const char *day = nullptr) {
    EdfPending::Record pending;
    pending.native_day = session_native_day;
    pending.mid = identity.mid; pending.vid = identity.vid;
    memcpy(pending.srn, identity.srn, sizeof(pending.srn));
    memcpy(pending.prefix, prefix ? prefix : status.file_prefix, sizeof(pending.prefix));
    memcpy(pending.day, day ? day : status.therapy_day, sizeof(pending.day));
    if (!write_pending(pending)) {
        post_error("STR pending journal write failed");
        return false;
    }
    portENTER_CRITICAL(&status_mux);
    status.pending_str++;
    portEXIT_CRITICAL(&status_mux);
    return true;
}

static void recover_pending() {
    fs::File dir = storage->open("/airbridge/pending");
    if (!dir || !dir.isDirectory()) return;
    fs::File item;
    while ((item = dir.openNextFile())) {
        String path = item.path();
        item.close();
        if (!path.endsWith(".part") && !path.endsWith(".bak")) continue;
        String target = path.substring(0, path.lastIndexOf('.'));
        EdfPending::Record record;
        if (read_pending(target.c_str(), record) == PendingRead::Ready) storage->remove(path);
        else if (read_pending(path.c_str(), record) == PendingRead::Ready) {
            storage->remove(target);
            if (!storage->rename(path, target)) post_error("STR journal recovery failed");
        } else post_error("invalid STR pending journal");
    }
}

static void recover_identification_files() {
    constexpr const char *TARGET = "/Identification.tgt";
    constexpr const char *CRC = "/Identification.crc";
    constexpr const char *TARGET_PART = "/Identification.tgt.part";
    constexpr const char *CRC_PART = "/Identification.crc.part";
    constexpr const char *TARGET_BAK = "/Identification.tgt.bak";
    constexpr const char *CRC_BAK = "/Identification.crc.bak";

    const bool complete = storage->exists(TARGET) && storage->exists(CRC);
    const bool backup_complete = storage->exists(TARGET_BAK) &&
                                 storage->exists(CRC_BAK);
    if (!complete && backup_complete) {
        if (storage->exists(TARGET)) storage->remove(TARGET);
        if (storage->exists(CRC)) storage->remove(CRC);
        if (!storage->rename(TARGET_BAK, TARGET) ||
            !storage->rename(CRC_BAK, CRC)) {
            status_error("Identification recovery failed");
        }
    } else if (complete) {
        if (storage->exists(TARGET_BAK)) storage->remove(TARGET_BAK);
        if (storage->exists(CRC_BAK)) storage->remove(CRC_BAK);
    } else {
        if (storage->exists(TARGET)) storage->remove(TARGET);
        if (storage->exists(CRC)) storage->remove(CRC);
        if (storage->exists(TARGET_BAK)) storage->remove(TARGET_BAK);
        if (storage->exists(CRC_BAK)) storage->remove(CRC_BAK);
    }
    if (storage->exists(TARGET_PART)) storage->remove(TARGET_PART);
    if (storage->exists(CRC_PART)) storage->remove(CRC_PART);

    // SD lookups may wait for an interrupt; publish only the result under lock.
    const bool identification_ready = storage->exists(TARGET) && storage->exists(CRC);
    portENTER_CRITICAL(&status_mux);
    status.identification_ready = identification_ready;
    portEXIT_CRITICAL(&status_mux);
}

static void recover_str_file() {
    constexpr const char *FINAL = "/STR.edf";
    constexpr const char *PART = "/STR.edf.part";
    constexpr const char *BACKUP = "/STR.edf.bak";

    if (!storage->exists(FINAL) && storage->exists(BACKUP)) {
        if (!storage->rename(BACKUP, FINAL))
            status_error("STR recovery failed");
    } else if (storage->exists(FINAL) && storage->exists(BACKUP)) {
        storage->remove(BACKUP);
    }
    if (storage->exists(PART)) storage->remove(PART);

    uint32_t records = 0;
    fs::File file = storage->open(FINAL, FILE_READ);
    uint8_t fixed[256];
    if (file && file.read(fixed, sizeof(fixed)) == sizeof(fixed)) {
        (void)parse_decimal_field(fixed + 236, 8, records);
    }
    if (file) file.close();
    portENTER_CRITICAL(&status_mux);
    status.str_records = records;
    portEXIT_CRITICAL(&status_mux);
}

static bool publish_identification_pair() {
    constexpr const char *TARGET = "/Identification.tgt";
    constexpr const char *CRC = "/Identification.crc";
    constexpr const char *TARGET_PART = "/Identification.tgt.part";
    constexpr const char *CRC_PART = "/Identification.crc.part";
    constexpr const char *TARGET_BAK = "/Identification.tgt.bak";
    constexpr const char *CRC_BAK = "/Identification.crc.bak";

    const bool had_pair = storage->exists(TARGET) && storage->exists(CRC);
    if (storage->exists(TARGET_BAK)) storage->remove(TARGET_BAK);
    if (storage->exists(CRC_BAK)) storage->remove(CRC_BAK);
    if (had_pair) {
        if (!storage->rename(TARGET, TARGET_BAK)) return false;
        if (!storage->rename(CRC, CRC_BAK)) {
            (void)storage->rename(TARGET_BAK, TARGET);
            return false;
        }
    }

    if (!storage->rename(TARGET_PART, TARGET)) {
        if (had_pair) {
            (void)storage->rename(TARGET_BAK, TARGET);
            (void)storage->rename(CRC_BAK, CRC);
        }
        return false;
    }
    if (!storage->rename(CRC_PART, CRC)) {
        storage->remove(TARGET);
        if (had_pair) {
            (void)storage->rename(TARGET_BAK, TARGET);
            (void)storage->rename(CRC_BAK, CRC);
        }
        return false;
    }

    if (had_pair) {
        storage->remove(TARGET_BAK);
        storage->remove(CRC_BAK);
    }
    return true;
}

static bool collect_identification(const AirSenseState::Identity &identity,
                                   uint8_t *&content, size_t &content_len) {
    static const char *const tags[] = {
        "IMF", "VIR", "RIR", "PVR", "PVD", "CID", "RID", "VID",
        "SRN", "SID", "PNA", "PCD", "PCB", "MID", "FGT", "BID",
    };
    if (!identity.valid || !identity.pna[0]) return false;
    content = static_cast<uint8_t *>(
        aircannect::Memory::alloc_large(IDENTIFICATION_BUFFER_SIZE));
    if (!content) {
        post_error("Identification buffer allocation failed");
        return false;
    }

    content_len = 0;
    bool built = true;
    bool cancelled = false;
    const char *failed_tag = "";
    Arbiter::VarReadTrace trace;
    for (const char *tag : tags) {
        failed_tag = tag;
        trace = {};
        if (post_processing_cancelled()) {
            cancelled = true;
            built = false;
            break;
        }
        char response[128] = {};
        const char *value = nullptr;
        if (!strcmp(tag, "SRN")) value = identity.srn;
        else if (!strcmp(tag, "MID")) value = identity.mid_text;
        else if (!strcmp(tag, "VID")) value = identity.vid_text;
        else if (!strcmp(tag, "PNA")) value = identity.pna;
        else {
            value = response;
            if (Arbiter::read_var(tag, CMD_SRC_INTERNAL, CMD_PRIO_LOW,
                                  response, sizeof(response), 1000, &trace) !=
                Arbiter::VarResult::Ok) {
                built = false;
                break;
            }
        }
        if (strchr(value, '\r') || strchr(value, '\n')) {
            trace.outcome = "invalid_text";
            built = false;
            break;
        }
        const int line_len = snprintf(
            reinterpret_cast<char *>(content + content_len),
            IDENTIFICATION_BUFFER_SIZE - content_len,
            "\r\n#%s %s\n", tag, value);
        if (line_len <= 0 ||
            static_cast<size_t>(line_len) >=
                IDENTIFICATION_BUFFER_SIZE - content_len) {
            trace.outcome = "buffer_full";
            built = false;
            break;
        }
        content_len += static_cast<size_t>(line_len);
    }
    if (!built) {
        if (!cancelled) {
            char error[160];
            snprintf(error, sizeof(error),
                     "Identification #%s result=%s queue=%lums sent=%u wait=%lums",
                     failed_tag, trace.outcome, (unsigned long)trace.queue_ms,
                     unsigned(trace.sent), (unsigned long)trace.wait_ms);
            post_error(error);
        }
        return false;
    }
    return !post_processing_cancelled();
}

static bool write_identification(const uint8_t *content, size_t content_len) {
    const uint32_t crc = crc32_ieee(content, content_len);
    bool unchanged = false;
    fs::File old_crc = storage->open("/Identification.crc", FILE_READ);
    fs::File old_target = storage->open("/Identification.tgt", FILE_READ);
    uint8_t old_crc_value[4];
    if (old_crc && old_target && old_target.size() == content_len &&
        old_crc.read(old_crc_value, sizeof(old_crc_value)) ==
            sizeof(old_crc_value)) {
        unchanged = SdStorage::get_le32(old_crc_value) == crc;
    }
    if (old_crc) old_crc.close();
    if (old_target) old_target.close();
    if (unchanged) {
        portENTER_CRITICAL(&status_mux);
        status.identification_ready = true;
        portEXIT_CRITICAL(&status_mux);
        return true;
    }

    storage->remove("/Identification.tgt.part");
    storage->remove("/Identification.crc.part");
    fs::File target = storage->open("/Identification.tgt.part", FILE_WRITE);
    fs::File crc_file = storage->open("/Identification.crc.part", FILE_WRITE);
    uint8_t crc_bytes[4];
    SdStorage::put_le32(crc_bytes, crc);
    const bool written = target && crc_file &&
                         SdStorage::write_exact(target, content, content_len) &&
                         SdStorage::write_exact(crc_file, crc_bytes, sizeof(crc_bytes));
    if (target) {
        target.flush();
        target.close();
    }
    if (crc_file) {
        crc_file.flush();
        crc_file.close();
    }
    if (!written || !publish_identification_pair()) {
        storage->remove("/Identification.tgt.part");
        storage->remove("/Identification.crc.part");
        post_error("Identification publish failed");
        return false;
    }

    portENTER_CRITICAL(&status_mux);
    status.identification_ready = true;
    portEXIT_CRITICAL(&status_mux);
    return true;
}

static bool ensure_identification(const AirSenseState::Identity &identity) {
    if (!SdStorage::try_acquire()) return false;
    const bool current = identification_verified && identification_device == identity.generation &&
        storage->exists("/Identification.tgt") && storage->exists("/Identification.crc");
    SdStorage::release();
    if (current) return true;

    uint8_t *content = nullptr;
    size_t size = 0;
    bool success = collect_identification(identity, content, size) &&
        !post_processing_cancelled() &&
        identity.generation == AirSenseState::identity_generation();
    if (success && SdStorage::try_acquire()) {
        success = write_identification(content, size);
        SdStorage::release();
        if (success) {
            identification_device = identity.generation;
            identification_crc = crc32_ieee(content, size);
            identification_verified = true;
        }
    } else {
        success = false;
    }
    aircannect::Memory::free(content);
    return success;
}

static bool fetch_str_record(uint8_t *record, size_t capacity) {
    const Air10Edf::Schema &schema = Air10Edf::str_schema();
    const size_t sample_count = Air10Edf::numeric_sample_count(schema);
    int16_t *samples = static_cast<int16_t *>(
        aircannect::Memory::alloc_large(sample_count * sizeof(int16_t)));
    if (!samples) {
        post_error("STR sample allocation failed");
        return false;
    }
    memset(samples, 0xFF, sample_count * sizeof(int16_t));

    Air10Stored::Value therapy_duration = {};
    if (!read_stored_value("THD", session_native_day, therapy_duration) ||
        !therapy_duration.present || therapy_duration.sample_count != 1) {
        aircannect::Memory::free(samples);
        post_error("STR duration unavailable");
        return false;
    }

    size_t offset = 0;
    bool complete = true;
    const char *failed_tag = "";
    const char *failure = "render failed";
    for (uint8_t signal = 0; signal + 1 < schema.signal_count; signal++) {
        if (post_processing_cancelled()) {
            complete = false;
            break;
        }
        const char *tag = Air10Edf::str_signal_tag(signal);
        failed_tag = tag;
        Air10Stored::Value value = {};
        if (strcmp(tag, "THD") == 0) {
            value = therapy_duration;
        } else if (!read_stored_value(tag, session_native_day, value)) {
            failure = "read failed";
            Log::logf(CAT_EDF, LOG_DEBUG,
                      "STR read failed for %s day=%04X\n",
                      tag, session_native_day);
            complete = false;
            break;
        }
        const uint16_t expected = schema.signals[signal].samples_per_record;
        if (offset + expected > sample_count) {
            failure = "schema exceeds record";
            complete = false;
            break;
        }
        if (!value.present) {
            if (strcmp(tag, "LSD") == 0 || strcmp(tag, "THD") == 0) {
                failure = "required value missing";
                Log::logf(CAT_EDF, LOG_DEBUG,
                          "required STR value missing for %s day=%04X\n",
                          tag, session_native_day);
                complete = false;
                break;
            }
            offset += expected;
            continue;
        }
        if (value.sample_count != expected) {
            failure = "unexpected sample count";
            Log::logf(CAT_EDF, LOG_DEBUG,
                      "STR value invalid for %s day=%04X count=%u\n",
                      tag, session_native_day, value.sample_count);
            complete = false;
            break;
        }
        if (strcmp(tag, "LSD") == 0 &&
            static_cast<uint16_t>(value.samples[0]) != session_native_day) {
            failure = "date mismatch";
            Log::logf(CAT_EDF, LOG_DEBUG,
                      "STR date mismatch wanted=%04X got=%04X\n",
                      session_native_day,
                      static_cast<uint16_t>(value.samples[0]));
            complete = false;
            break;
        }
        memcpy(samples + offset, value.samples,
               expected * sizeof(int16_t));
        offset += expected;
    }

    size_t written = 0;
    const bool rendered = complete && offset == sample_count &&
        Air10Edf::render_numeric_record(schema, samples, sample_count,
                                        record, capacity, written) &&
        written == Air10Edf::record_size(schema);
    aircannect::Memory::free(samples);
    if (!rendered && !post_processing_cancelled()) {
        char error[96];
        snprintf(error, sizeof(error), "STR day=%04X tag=%s: %s",
                 session_native_day, failed_tag, failure);
        post_error(error);
    }
    return rendered;
}

static bool render_str_header(uint16_t first_day, uint32_t records,
                              uint8_t *header, size_t capacity,
                              const AirSenseState::Identity &identity) {
    int year = 0;
    unsigned month = 0;
    unsigned day = 0;
    epoch_day_to_civil(first_day, year, month, day);
    if (month < 1 || month > 12) return false;

    char str_recording[81];
    char str_date[9];
    snprintf(str_recording, sizeof(str_recording),
             "Startdate %02u-%s-%04d X X X SRN=%s  MID=%u  VID=%u",
             day, MONTH_NAMES[month - 1], year, identity.srn,
             identity.mid, identity.vid);
    snprintf(str_date, sizeof(str_date), "%02u.%02u.%02u",
             day, month, year % 100);
    Air10Edf::HeaderInfo info = {
        str_recording,
        str_date,
        "12.00.00",
        records,
    };
    size_t written = 0;
    return Air10Edf::render_header(Air10Edf::str_schema(), info,
                                   header, capacity, written) &&
           written == Air10Edf::header_size(Air10Edf::str_schema());
}

static bool valid_str_record(const uint8_t *record, size_t size) {
    return record && size >= 4 &&
           SdStorage::get_le16(record + size - 2) == crc16_ccitt(record, size - 2);
}

static bool valid_str_record(const uint8_t *record, size_t size,
                             uint16_t expected_day) {
    return record && SdStorage::get_le16(record) == expected_day &&
           valid_str_record(record, size);
}

static bool parse_edf_start_day(const uint8_t *text, uint16_t &epoch_day) {
    if (!text || text[2] != '.' || text[5] != '.') return false;
    const uint8_t digit_offsets[] = {0, 1, 3, 4, 6, 7};
    for (uint8_t offset : digit_offsets)
        if (text[offset] < '0' || text[offset] > '9') return false;

    const unsigned day = (text[0] - '0') * 10 + text[1] - '0';
    const unsigned month = (text[3] - '0') * 10 + text[4] - '0';
    const unsigned short_year = (text[6] - '0') * 10 + text[7] - '0';
    const int year = short_year >= 85 ? 1900 + short_year
                                      : 2000 + short_year;
    const int32_t parsed = Air10Clock::civil_epoch_day(year, month, day);
    int check_year = 0;
    unsigned check_month = 0;
    unsigned check_day = 0;
    if (month < 1 || month > 12 || day < 1 || day > 31 || parsed < 0 ||
        parsed >= UINT16_MAX) {
        return false;
    }
    epoch_day_to_civil(parsed, check_year, check_month, check_day);
    if (check_year != year || check_month != month || check_day != day)
        return false;
    epoch_day = static_cast<uint16_t>(parsed);
    return true;
}

static bool parse_therapy_day(const char *text, uint16_t &epoch_day) {
    if (!text || strlen(text) != 8) return false;
    for (size_t i = 0; i < 8; i++)
        if (text[i] < '0' || text[i] > '9') return false;
    const int year = (text[0] - '0') * 1000 + (text[1] - '0') * 100 +
                     (text[2] - '0') * 10 + text[3] - '0';
    const unsigned month = (text[4] - '0') * 10 + text[5] - '0';
    const unsigned day = (text[6] - '0') * 10 + text[7] - '0';
    if (month < 1 || month > 12 || day < 1 || day > 31) return false;
    const int32_t parsed = Air10Clock::civil_epoch_day(year, month, day);
    if (parsed < 0 || parsed > UINT16_MAX) return false;
    epoch_day = static_cast<uint16_t>(parsed);
    return true;
}

static bool str_contains_day(const char *therapy_day) {
    uint16_t wanted_day = 0;
    if (!parse_therapy_day(therapy_day, wanted_day)) return false;

    const Air10Edf::Schema &schema = Air10Edf::str_schema();
    const size_t header_size = Air10Edf::header_size(schema);
    const size_t record_size = Air10Edf::record_size(schema);
    fs::File file = storage->open("/STR.edf", FILE_READ);
    uint8_t fixed[256];
    uint32_t stored_header_size = 0;
    uint32_t records = 0;
    uint32_t signal_count = 0;
    if (!file || file.read(fixed, sizeof(fixed)) != sizeof(fixed) ||
        !parse_decimal_field(fixed + 184, 8, stored_header_size) ||
        !parse_decimal_field(fixed + 236, 8, records) ||
        !parse_decimal_field(fixed + 252, 4, signal_count) || records == 0 ||
        stored_header_size != header_size ||
        signal_count != schema.signal_count ||
        file.size() != header_size + records * record_size) {
        if (file) file.close();
        return false;
    }

    uint8_t *record = static_cast<uint8_t *>(aircannect::Memory::alloc_large(record_size));
    bool present = false;
    if (record && file.seek(header_size) &&
        file.read(record, record_size) == record_size) {
        const uint16_t first_day = SdStorage::get_le16(record);
        const uint32_t offset = wanted_day >= first_day
            ? static_cast<uint32_t>(wanted_day - first_day) : UINT32_MAX;
        if (valid_str_record(record, record_size, first_day) &&
            offset < records &&
            file.seek(header_size + static_cast<size_t>(offset) * record_size) &&
            file.read(record, record_size) == record_size) {
            present = valid_str_record(record, record_size, wanted_day);
        }
    }
    aircannect::Memory::free(record);
    file.close();
    return present;
}

static bool set_str_record_day(uint16_t epoch_day, uint8_t *record,
                               size_t size) {
    if (!record || size < 4 || epoch_day == UINT16_MAX) return false;
    record[0] = static_cast<uint8_t>(epoch_day);
    record[1] = static_cast<uint8_t>(epoch_day >> 8);
    const uint16_t crc = crc16_ccitt(record, size - 2);
    record[size - 2] = static_cast<uint8_t>(crc);
    record[size - 1] = static_cast<uint8_t>(crc >> 8);
    return true;
}

static bool render_empty_str_record(uint16_t epoch_day, uint8_t *record,
                                    size_t size) {
    if (!record || size < 4) return false;
    memset(record, 0xFF, size);
    return set_str_record_day(epoch_day, record, size);
}

static bool update_str_file(const uint8_t *incoming_record,
                            const EdfCatalog::Entry &latest,
                            const AirSenseState::Identity &identity) {
    constexpr const char *FINAL = "/STR.edf";
    constexpr const char *PART = "/STR.edf.part";
    constexpr const char *BACKUP = "/STR.edf.bak";
    const Air10Edf::Schema &schema = Air10Edf::str_schema();
    const size_t header_size = Air10Edf::header_size(schema);
    const size_t record_size = Air10Edf::record_size(schema);
    if (post_processing_cancelled()) return false;
    if (!valid_str_record(incoming_record, record_size,
                          session_native_day)) {
        post_error("incoming STR record invalid");
        return false;
    }
    uint8_t *header = static_cast<uint8_t *>(aircannect::Memory::alloc_large(header_size));
    uint8_t *work = static_cast<uint8_t *>(aircannect::Memory::alloc_large(record_size));
    if (!header || !work) {
        aircannect::Memory::free(header);
        aircannect::Memory::free(work);
        post_error("STR rewrite buffer allocation failed");
        return false;
    }

    fs::File input;
    fs::File output;
    uint32_t existing_records = 0;
    bool valid = true;
    bool cancelled = false;
    bool same_header = false, same_record = false;
    Air10StrTimeline::Scan scan;
    if (storage->exists(FINAL)) {
        input = storage->open(FINAL, FILE_READ);
        uint8_t fixed[256];
        uint32_t stored_header_size = 0;
        uint32_t signal_count = 0;
        uint16_t header_start_day = 0;
        if (!input || input.read(fixed, sizeof(fixed)) != sizeof(fixed) ||
            !parse_decimal_field(fixed + 184, 8, stored_header_size) ||
            !parse_decimal_field(fixed + 236, 8, existing_records) ||
            !parse_decimal_field(fixed + 252, 4, signal_count) ||
            !parse_edf_start_day(fixed + 168, header_start_day) ||
            stored_header_size != header_size ||
            signal_count != schema.signal_count ||
            existing_records > Air10StrTimeline::RECORD_LIMIT ||
            input.size() != header_size + existing_records * record_size ||
            !Air10StrTimeline::begin(header_start_day, existing_records,
                                     scan) ||
            !render_str_header(header_start_day, existing_records, header, header_size, identity)) {
            valid = false;
        }
        if (valid) same_header = memcmp(fixed, header, sizeof(fixed)) == 0;

        size_t compared = 256;
        while (valid && compared < header_size) {
            if (post_processing_cancelled()) {
                valid = false;
                cancelled = true;
                break;
            }
            const size_t chunk = min(record_size, header_size - compared);
            if (input.read(work, chunk) != chunk ||
                memcmp(work, header + compared, chunk) != 0) {
                valid = false;
                break;
            }
            compared += chunk;
        }
        for (uint32_t i = 0; valid && i < existing_records; i++) {
            if (post_processing_cancelled()) {
                valid = false;
                cancelled = true;
                break;
            }
            if (input.read(work, record_size) != record_size) {
                valid = false;
                break;
            }
            if (!valid_str_record(work, record_size) ||
                !Air10StrTimeline::scan_record(scan, i, SdStorage::get_le16(work))) {
                valid = false;
            }
            if (valid && SdStorage::get_le16(work) == session_native_day)
                same_record = memcmp(work, incoming_record, record_size) == 0;
        }
        if (!valid) {
            input.close();
            aircannect::Memory::free(header);
            aircannect::Memory::free(work);
            if (!cancelled) post_error("existing STR validation failed");
            return false;
        }
    } else if (!Air10StrTimeline::begin(session_native_day, 0, scan)) {
        aircannect::Memory::free(header);
        aircannect::Memory::free(work);
        post_error("STR timeline initialization failed");
        return false;
    }

    Air10StrTimeline::Plan plan;
    if (!Air10StrTimeline::make_plan(scan, session_native_day, plan) ||
        !render_str_header(plan.start_day, plan.record_count,
                           header, header_size, identity)) {
        if (input) input.close();
        aircannect::Memory::free(header);
        aircannect::Memory::free(work);
        post_error("STR timeline range invalid");
        return false;
    }

    if (same_header && same_record && scan.continuous &&
        plan.start_day == scan.header_start_day && plan.record_count == existing_records) {
        input.close();
        aircannect::Memory::free(header);
        aircannect::Memory::free(work);
        return true;
    }

    const size_t timeline_size =
        static_cast<size_t>(plan.record_count) * record_size;
    uint8_t *timeline = static_cast<uint8_t *>(aircannect::Memory::alloc_large(timeline_size));
    uint8_t *present = static_cast<uint8_t *>(
        aircannect::Memory::alloc_large(plan.record_count));
    if (!timeline || !present || post_processing_cancelled()) {
        if (input) input.close();
        aircannect::Memory::free(timeline);
        aircannect::Memory::free(present);
        aircannect::Memory::free(header);
        aircannect::Memory::free(work);
        if (!post_processing_cancelled())
            post_error("STR timeline allocation failed");
        return false;
    }
    memset(present, 0, plan.record_count);
    Air10StrTimeline::Buffer timeline_buffer = {
        timeline, present, plan.record_count,
    };
    Air10StrTimeline::BuildStats build_stats;

    if (input && !input.seek(header_size)) valid = false;
    for (uint32_t i = 0; valid && i < existing_records; i++) {
        if (post_processing_cancelled()) {
            valid = false;
            cancelled = true;
            break;
        }
        uint16_t day = 0;
        if (input.read(work, record_size) != record_size ||
            !valid_str_record(work, record_size) ||
            !Air10StrTimeline::record_day(scan, i, SdStorage::get_le16(work), day) ||
            !set_str_record_day(day, work, record_size) ||
            !Air10StrTimeline::place_record(plan, timeline_buffer, day, work,
                                            record_size, build_stats)) {
            valid = false;
        }
    }
    if (valid &&
        (!Air10StrTimeline::place_record(
             plan, timeline_buffer, session_native_day, incoming_record,
             record_size, build_stats) ||
         !Air10StrTimeline::fill_missing(
             plan, timeline_buffer, record_size, render_empty_str_record,
             build_stats))) {
        valid = false;
    }

    if (input) input.close();
    if (post_processing_cancelled()) {
        aircannect::Memory::free(timeline);
        aircannect::Memory::free(present);
        aircannect::Memory::free(header);
        aircannect::Memory::free(work);
        return false;
    }
    storage->remove(PART);
    output = storage->open(PART, FILE_WRITE);
    if (!valid || !output || !SdStorage::write_exact(output, header, header_size))
        valid = false;

    for (uint32_t i = 0; valid && i < plan.record_count; i++) {
        if (post_processing_cancelled()) {
            valid = false;
            cancelled = true;
            break;
        }
        const uint8_t *record = timeline + static_cast<size_t>(i) * record_size;
        if (!SdStorage::write_exact(output, record, record_size)) valid = false;
    }
    if (output) {
        output.flush();
        output.close();
    }

    if (cancelled) {
        storage->remove(PART);
        aircannect::Memory::free(timeline);
        aircannect::Memory::free(present);
        aircannect::Memory::free(header);
        aircannect::Memory::free(work);
        return false;
    }

    // Preserve the day across a reset between STR publication and catalog commit.
    char pending[80];
    pending_path(latest.file_prefix, pending, sizeof(pending));
    if (valid && !storage->exists(pending))
        valid = save_pending(identity, latest.file_prefix, latest.therapy_day);
    if (!valid || !publish_single_file(PART, FINAL, BACKUP)) {
        storage->remove(PART);
        aircannect::Memory::free(timeline);
        aircannect::Memory::free(present);
        aircannect::Memory::free(header);
        aircannect::Memory::free(work);
        post_error("STR publish failed");
        return false;
    }
    Log::logf(CAT_EDF, LOG_DEBUG,
              "STR timeline %04X-%04X records=%u fillers=%u "
              "replaced=%u discarded=%u\n",
              plan.start_day, plan.end_day, plan.record_count,
              build_stats.filler_records, build_stats.replaced_records,
              build_stats.discarded_records);
    aircannect::Memory::free(timeline);
    aircannect::Memory::free(present);
    aircannect::Memory::free(header);
    aircannect::Memory::free(work);
    portENTER_CRITICAL(&status_mux);
    status.str_records = plan.record_count;
    portEXIT_CRITICAL(&status_mux);
    return true;
}

struct StrReadTrace {
    const char *result = "identity_failed";
    uint32_t generation = 0;
    bool generation_read = false;
};

static bool collect_str_summary(uint8_t *&record, uint32_t generation,
                                StrReadTrace &trace) {
    record = nullptr;
    trace.result = "read_failed";
    Air10Stored::Value date = {};
    if (!read_stored_value("LSD", session_native_day, date)) return false;
    if (date.present) {
        const size_t record_size =
            Air10Edf::record_size(Air10Edf::str_schema());
        record = static_cast<uint8_t *>(aircannect::Memory::alloc_large(record_size));
        if (!record) {
            trace.result = "no_memory";
            post_error("STR record allocation failed");
            return false;
        }
        if (!fetch_str_record(record, record_size)) return false;
    }
    trace.generation_read = read_u32_variable("ZEN", trace.generation);
    if (!trace.generation_read) trace.result = "ZEN_read_failed";
    else if (trace.generation != generation) trace.result = "ZEN_changed";
    else if (post_processing_cancelled()) trace.result = "interrupted";
    else {
        trace.result = date.present ? "ready" : "absent";
        return true;
    }
    return false;
}

static bool session_files_complete_at(const char *directory,
                                      const char *file_prefix) {
    static const char *const suffixes[] = {
        "BRP", "PLD", "SAD", "EVE", "CSL",
    };
    char path[128];
    for (const char *suffix : suffixes) {
        snprintf(path, sizeof(path), "%s/%s_%s.edf",
                 directory, file_prefix, suffix);
        if (!storage->exists(path)) return false;
        snprintf(path, sizeof(path), "%s/%s_%s.crc",
                 directory, file_prefix, suffix);
        if (!storage->exists(path)) return false;
    }
    return true;
}

static bool session_files_complete() {
    return session_files_complete_at(session_directory, status.file_prefix);
}

static bool commit_session_catalog(bool identification_ready,
                                   bool str_ready,
                                   EdfCatalog::Entry &entry) {
    if (!session_files_complete()) {
        post_error("session file set is incomplete");
        return false;
    }
    entry = {};
    strncpy(entry.therapy_day, status.therapy_day,
            sizeof(entry.therapy_day) - 1);
    strncpy(entry.file_prefix, status.file_prefix,
            sizeof(entry.file_prefix) - 1);
    entry.flags = EdfCatalog::ENTRY_LIVE_COMPLETE;
    if (identification_ready)
        entry.flags |= EdfCatalog::ENTRY_IDENTIFICATION_READY;
    if (str_ready)
        entry.flags |= EdfCatalog::ENTRY_STR_READY;
    entry.finalized_epoch = static_cast<uint32_t>(time(nullptr));
    if (!EdfCatalog::commit(entry)) {
        post_error("session catalog commit failed");
        return false;
    }
    return true;
}

static bool recover_partial_output(const char *partial_path) {
    if (!partial_path || !has_suffix(partial_path, ".edf.part")) return false;

    char final_path[128];
    char crc_path[128];
    char crc_partial_path[128];
    char edf_recovery_path[128];
    char crc_recovery_path[128];
    const size_t partial_len = strlen(partial_path);
    if (partial_len >= sizeof(final_path) || partial_len <= 5) return false;
    memcpy(final_path, partial_path, partial_len - 5);
    final_path[partial_len - 5] = 0;
    const size_t final_len = strlen(final_path);
    if (final_len < 4 || !has_suffix(final_path, ".edf")) return false;
    memcpy(crc_path, final_path, final_len + 1);
    memcpy(crc_path + final_len - 3, "crc", 4);
    if (snprintf(crc_partial_path, sizeof(crc_partial_path),
                 "%s.part", crc_path) >=
            static_cast<int>(sizeof(crc_partial_path)) ||
        snprintf(edf_recovery_path, sizeof(edf_recovery_path),
                 "%s.recover", final_path) >=
            static_cast<int>(sizeof(edf_recovery_path)) ||
        snprintf(crc_recovery_path, sizeof(crc_recovery_path),
                 "%s.recover", crc_path) >=
            static_cast<int>(sizeof(crc_recovery_path))) {
        return false;
    }
    if (storage->exists(final_path)) {
        Log::logf(CAT_EDF, LOG_WARN,
                  "keeping partial with final-file collision: %s\n",
                  partial_path);
        return false;
    }
    // A sidecar without its EDF is an interrupted final rename from this
    // recorder. Recovery recalculates it from the complete records.
    if (storage->exists(crc_path)) storage->remove(crc_path);
    if (storage->exists(crc_partial_path)) storage->remove(crc_partial_path);
    if (storage->exists(edf_recovery_path))
        storage->remove(edf_recovery_path);
    if (storage->exists(crc_recovery_path))
        storage->remove(crc_recovery_path);

    fs::File input = storage->open(partial_path, FILE_READ);
    fs::File output;
    fs::File crc_output;
    uint8_t *header = nullptr;
    uint8_t *buffer = nullptr;
    uint32_t records = 0;
    bool recovered = false;

    do {
        uint8_t fixed_header[256];
        if (!input || input.size() < sizeof(fixed_header) ||
            input.read(fixed_header, sizeof(fixed_header)) !=
                sizeof(fixed_header)) {
            break;
        }

        uint32_t header_size = 0;
        uint32_t signal_count = 0;
        if (!parse_decimal_field(fixed_header + 184, 8, header_size) ||
            !parse_decimal_field(fixed_header + 252, 4, signal_count) ||
            signal_count == 0 || header_size > RECOVERY_HEADER_MAX ||
            header_size != 256 + signal_count * 256 ||
            input.size() < header_size) {
            break;
        }

        header = static_cast<uint8_t *>(aircannect::Memory::alloc_large(header_size));
        buffer = static_cast<uint8_t *>(aircannect::Memory::alloc_large(RECOVERY_BUFFER_SIZE));
        if (!header || !buffer) break;
        memcpy(header, fixed_header, sizeof(fixed_header));
        if (input.read(header + sizeof(fixed_header),
                       header_size - sizeof(fixed_header)) !=
            header_size - sizeof(fixed_header)) {
            break;
        }

        const size_t samples_offset = 256 + signal_count * 216;
        uint32_t samples_per_record = 0;
        for (uint32_t i = 0; i < signal_count; i++) {
            uint32_t samples = 0;
            if (!parse_decimal_field(header + samples_offset + i * 8,
                                     8, samples) ||
                samples_per_record > UINT32_MAX - samples) {
                samples_per_record = 0;
                break;
            }
            samples_per_record += samples;
        }
        const uint32_t record_size = samples_per_record * 2;
        if (samples_per_record == 0 || record_size / 2 != samples_per_record ||
            record_size > 65536)
            break;

        const size_t data_bytes = input.size() - header_size;
        records = data_bytes / record_size;
        const size_t complete_bytes = static_cast<size_t>(records) * record_size;
        if (!Air10Edf::update_record_count(header, header_size, records))
            break;

        output = storage->open(edf_recovery_path, FILE_WRITE);
        if (!output || !SdStorage::write_exact(output, header, header_size)) break;
        uint32_t rest_crc = crc32_ieee_update(
            crc32_ieee_initial(), header + 256, header_size - 256);
        size_t remaining = complete_bytes;
        while (remaining) {
            const size_t chunk = remaining < RECOVERY_BUFFER_SIZE
                                     ? remaining : RECOVERY_BUFFER_SIZE;
            if (input.read(buffer, chunk) != chunk ||
                !SdStorage::write_exact(output, buffer, chunk)) {
                remaining = SIZE_MAX;
                break;
            }
            rest_crc = crc32_ieee_update(rest_crc, buffer, chunk);
            remaining -= chunk;
        }
        if (remaining != 0) break;
        output.flush();
        output.close();

        uint8_t sidecar[8];
        SdStorage::put_le32(sidecar, crc32_ieee(header, 256));
        SdStorage::put_le32(sidecar + 4, crc32_ieee_finish(rest_crc));
        crc_output = storage->open(crc_recovery_path, FILE_WRITE);
        if (!crc_output || !SdStorage::write_exact(crc_output, sidecar, sizeof(sidecar)))
            break;
        crc_output.flush();
        crc_output.close();

        if (!storage->rename(crc_recovery_path, crc_path)) break;
        if (!storage->rename(edf_recovery_path, final_path)) {
            storage->remove(crc_path);
            break;
        }
        if (!storage->remove(partial_path)) {
            Log::logf(CAT_EDF, LOG_WARN,
                      "recovered but could not remove %s\n", partial_path);
        }
        recovered = true;
    } while (false);

    if (input) input.close();
    if (output) output.close();
    if (crc_output) crc_output.close();
    aircannect::Memory::free(header);
    aircannect::Memory::free(buffer);
    if (!recovered) {
        storage->remove(edf_recovery_path);
        storage->remove(crc_recovery_path);
        status_error("partial EDF recovery failed");
        return false;
    }

    portENTER_CRITICAL(&status_mux);
    portEXIT_CRITICAL(&status_mux);
    Log::logf(CAT_EDF, LOG_INFO,
              "recovered %s (%u records)\n", final_path, records);
    return true;
}

static void recover_partial_outputs() {
    if (!storage->exists("/DATALOG")) return;
    fs::File root = storage->open("/DATALOG", FILE_READ);
    if (!root || !root.isDirectory()) return;

    fs::File day;
    while ((day = root.openNextFile(FILE_READ))) {
        if (!day.isDirectory()) {
            day.close();
            continue;
        }
        char partial_paths[8][128] = {};
        uint8_t partial_count = 0;
        fs::File entry;
        while ((entry = day.openNextFile(FILE_READ))) {
            if (!entry.isDirectory() && partial_count < 8 &&
                has_suffix(entry.path(), ".edf.part")) {
                strncpy(partial_paths[partial_count], entry.path(),
                        sizeof(partial_paths[partial_count]) - 1);
                partial_count++;
            }
            entry.close();
        }
        day.close();
        for (uint8_t i = 0; i < partial_count; i++)
            (void)recover_partial_output(partial_paths[i]);
    }
    root.close();
    SdStorage::refresh_usage();
}

static bool parse_brp_file_prefix(const char *path, char *prefix,
                                  size_t prefix_size) {
    if (!path || !prefix || prefix_size < 16) return false;
    const char *name = strrchr(path, '/');
    name = name ? name + 1 : path;
    if (strlen(name) != 23 || strcmp(name + 15, "_BRP.edf") != 0 ||
        name[8] != '_') {
        return false;
    }
    for (size_t i = 0; i < 15; i++) {
        if (i == 8) continue;
        if (name[i] < '0' || name[i] > '9') return false;
    }
    memcpy(prefix, name, 15);
    prefix[15] = 0;
    return true;
}

static uint32_t count_catalog_candidates() {
    fs::File root = storage->open("/DATALOG", FILE_READ);
    if (!root || !root.isDirectory()) return 0;
    uint32_t count = 0;
    fs::File day;
    while ((day = root.openNextFile(FILE_READ))) {
        if (!day.isDirectory()) {
            day.close();
            continue;
        }
        const char *day_name = strrchr(day.path(), '/');
        day_name = day_name ? day_name + 1 : day.path();
        uint16_t ignored_day = 0;
        if (!parse_therapy_day(day_name, ignored_day)) {
            day.close();
            continue;
        }
        fs::File file;
        while ((file = day.openNextFile(FILE_READ))) {
            char prefix[16];
            if (!file.isDirectory() &&
                parse_brp_file_prefix(file.path(), prefix, sizeof(prefix)) &&
                count < UINT32_MAX) {
                count++;
            }
            file.close();
        }
        day.close();
    }
    root.close();
    return count;
}

static bool prefix_in_snapshot(const char *prefixes, uint32_t count,
                               const char *prefix) {
    if (!prefixes || !prefix) return false;
    for (uint32_t i = 0; i < count; i++) {
        if (strcmp(prefixes + static_cast<size_t>(i) * 16, prefix) == 0)
            return true;
    }
    return false;
}

static bool reconcile_catalog() {
    if (!storage->exists("/DATALOG")) return true;
    EdfCatalog::Status catalog;
    EdfCatalog::get_status(catalog);
    const uint32_t candidate_count = count_catalog_candidates();
    const size_t prefix_capacity =
        static_cast<size_t>(catalog.entries) + candidate_count;
    char *known_prefixes = nullptr;
    uint32_t known_count = 0;
    if (prefix_capacity <= SIZE_MAX / 16) {
        const size_t bytes = prefix_capacity * 16;
        if (bytes) {
            known_prefixes = static_cast<char *>(aircannect::Memory::alloc_large(
                bytes, bytes <= 16 * 1024));
        }
        if (catalog.entries == 0) {
            known_count = 0;
        } else if (!known_prefixes ||
                   !EdfCatalog::snapshot_prefixes(
                       known_prefixes, bytes, known_count)) {
            aircannect::Memory::free(known_prefixes);
            known_prefixes = nullptr;
            known_count = 0;
        }
    }

    fs::File root = storage->open("/DATALOG", FILE_READ);
    if (!root || !root.isDirectory()) {
        aircannect::Memory::free(known_prefixes);
        return false;
    }

    fs::File day;
    while ((day = root.openNextFile(FILE_READ))) {
        if (!day.isDirectory()) {
            day.close();
            continue;
        }
        char directory[40];
        strncpy(directory, day.path(), sizeof(directory) - 1);
        directory[sizeof(directory) - 1] = 0;
        const char *day_name = strrchr(directory, '/');
        day_name = day_name ? day_name + 1 : directory;
        uint16_t ignored_day = 0;
        if (!parse_therapy_day(day_name, ignored_day)) {
            day.close();
            continue;
        }

        fs::File file;
        while ((file = day.openNextFile(FILE_READ))) {
            char prefix[16];
            const bool candidate = !file.isDirectory() &&
                parse_brp_file_prefix(file.path(), prefix, sizeof(prefix));
            const uint32_t finalized_epoch = candidate
                ? static_cast<uint32_t>(file.getLastWrite()) : 0;
            file.close();
            if (!candidate ||
                !session_files_complete_at(directory, prefix)) {
                continue;
            }

            EdfCatalog::Entry entry;
            const bool known = known_prefixes
                ? prefix_in_snapshot(known_prefixes, known_count, prefix)
                : EdfCatalog::find(prefix, entry);
            if (known) continue;
            entry = {};
            strncpy(entry.therapy_day, day_name,
                    sizeof(entry.therapy_day) - 1);
            strncpy(entry.file_prefix, prefix,
                    sizeof(entry.file_prefix) - 1);
            entry.flags = EdfCatalog::ENTRY_LIVE_COMPLETE;
            if (status.identification_ready)
                entry.flags |= EdfCatalog::ENTRY_IDENTIFICATION_READY;
            char pending[80];
            pending_path(prefix, pending, sizeof(pending));
            if (!storage->exists(pending) && str_contains_day(day_name))
                entry.flags |= EdfCatalog::ENTRY_STR_READY;
            entry.finalized_epoch = finalized_epoch
                ? finalized_epoch : static_cast<uint32_t>(time(nullptr));
            if (!EdfCatalog::commit(entry)) {
                status_error("catalog reconciliation failed");
                day.close();
                root.close();
                aircannect::Memory::free(known_prefixes);
                return false;
            }
            if (known_prefixes && known_count < prefix_capacity) {
                memcpy(known_prefixes + static_cast<size_t>(known_count) * 16,
                       entry.file_prefix, 16);
                known_count++;
            }
            Log::logf(CAT_EDF, LOG_INFO,
                      "catalog recovered %s/%s\n",
                      entry.therapy_day, entry.file_prefix);
        }
        day.close();
    }
    root.close();
    aircannect::Memory::free(known_prefixes);
    return true;
}

static bool render_output_header(OutputFile &output, uint32_t records,
                                 size_t &written) {
    Air10Edf::HeaderInfo info = {
        recording_id,
        start_date,
        start_time,
        records,
    };
    return Air10Edf::render_header(*output.schema, info, header_buffer,
                                   header_capacity, written);
}

static void output_path(const OutputFile &output, const char *extension,
                         char (&path)[128]) {
    snprintf(path, sizeof(path), "%s/%s_%s.%s", session_directory,
             status.file_prefix, output.schema->suffix, extension);
}

static bool open_output(OutputFile &output, const Air10Edf::Schema &schema) {
    output = {};
    output.schema = &schema;

    char path[128];
    for (const char *extension : {"edf.part", "edf", "crc.part", "crc"}) {
        output_path(output, extension, path);
        if (storage->exists(path)) return false;
    }
    output_path(output, "edf.part", path);
    output.file = storage->open(path, FILE_WRITE);
    if (!output.file) return false;
    output.created = true;

    size_t header_len = 0;
    if (!render_output_header(output, 0, header_len) ||
        !SdStorage::write_exact(output.file, header_buffer, header_len)) {
        output.file.close();
        return false;
    }
    output.rest_crc = crc32_ieee_update(
        crc32_ieee_initial(), header_buffer + 256, header_len - 256);
    output.open = true;
    return true;
}

static bool append_output_record(OutputFile &output,
                                 const uint8_t *data, size_t len) {
    if (output.failed || !output.open) return false;
    if (!SdStorage::write_exact(output.file, data, len)) {
        output.failed = true;
        output.file.close();
        output.open = false;
        status_error("SD record write failed", output.schema->suffix);
        return false;
    }
    output.rest_crc = crc32_ieee_update(output.rest_crc, data, len);
    output.records++;
    output.file.flush();
    return true;
}

static bool finalize_output(OutputFile &output) {
    if (output.failed || !output.open) return false;
    size_t header_len = 0;
    if (!render_output_header(output, output.records, header_len) ||
        !output.file.seek(0) ||
        !SdStorage::write_exact(output.file, header_buffer, header_len)) {
        status_error("EDF header finalization failed", output.schema->suffix);
        output.file.close();
        output.open = false;
        return false;
    }
    output.file.flush();
    output.file.close();
    output.open = false;

    uint8_t sidecar[8];
    SdStorage::put_le32(sidecar, crc32_ieee(header_buffer, 256));
    SdStorage::put_le32(sidecar + 4, crc32_ieee_finish(output.rest_crc));
    char partial_path[128], final_path[128];
    output_path(output, "crc.part", partial_path);
    output_path(output, "crc", final_path);
    fs::File crc_file = storage->open(partial_path, FILE_WRITE);
    if (!crc_file || !SdStorage::write_exact(crc_file, sidecar, sizeof(sidecar))) {
        if (crc_file) crc_file.close();
        status_error("CRC sidecar write failed", output.schema->suffix);
        return false;
    }
    crc_file.flush();
    crc_file.close();
    if (!storage->rename(partial_path, final_path)) {
        status_error("CRC sidecar rename failed", output.schema->suffix);
        return false;
    }
    output_path(output, "edf.part", partial_path);
    output_path(output, "edf", final_path);
    if (!storage->rename(partial_path, final_path)) {
        output_path(output, "crc", final_path);
        storage->remove(final_path);
        status_error("EDF rename failed", output.schema->suffix);
        return false;
    }
    return true;
}

static void close_output(OutputFile &output, bool remove_partial) {
    if (output.file) output.file.close();
    if (remove_partial && output.created && storage) {
        char path[128];
        output_path(output, "edf.part", path);
        storage->remove(path);
    }
    output = {};
}

static bool initialize_accumulator(Accumulator &accumulator,
                                   const Air10Edf::Schema &schema) {
    accumulator = {};
    accumulator.sample_count = Air10Edf::numeric_sample_count(schema);
    accumulator.samples = static_cast<int16_t *>(
        aircannect::Memory::alloc_large((accumulator.sample_count + 1) * sizeof(int16_t)));
    if (!accumulator.samples) return false;
    memset(accumulator.samples, 0xFF,
           accumulator.sample_count * sizeof(int16_t));
    accumulator.initialized = true;
    return open_output(accumulator.output, schema);
}

static void release_accumulator(Accumulator &accumulator, bool remove_partial) {
    close_output(accumulator.output, remove_partial);
    aircannect::Memory::free(accumulator.samples);
    accumulator = {};
}

static bool write_current_record(Accumulator &accumulator) {
    static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
                  "EDF accumulator writes require little-endian samples");
    if (!accumulator.initialized) return false;
    const size_t bytes = accumulator.sample_count * sizeof(int16_t);
    uint8_t *record = reinterpret_cast<uint8_t *>(accumulator.samples);
    const uint16_t crc = crc16_ccitt(record, bytes);
    memcpy(record + bytes, &crc, sizeof(crc));
    if (!append_output_record(accumulator.output, record, bytes + sizeof(crc)))
        return false;
    memset(accumulator.samples, 0xFF,
           accumulator.sample_count * sizeof(int16_t));
    return true;
}

static bool advance_record(Accumulator &accumulator, uint32_t target) {
    if (!accumulator.initialized || target < accumulator.current_record)
        return false;
    if (target - accumulator.current_record > 1440) {
        status_error("record timeline jump too large");
        return false;
    }
    while (accumulator.current_record < target) {
        if (!write_current_record(accumulator)) return false;
        accumulator.current_record++;
    }
    return true;
}

static size_t signal_sample_offset(const Air10Edf::Schema &schema,
                                   uint8_t signal) {
    size_t offset = 0;
    for (uint8_t i = 0; i < signal; i++)
        offset += schema.signals[i].samples_per_record;
    return offset;
}

static int signal_index(const Air10Edf::Schema &schema, const char *label) {
    for (uint8_t i = 0; i + 1 < schema.signal_count; i++)
        if (strcmp(schema.signals[i].label, label) == 0) return i;
    return -1;
}

static void store_sample(Accumulator &accumulator, const char *label,
                         uint32_t record, uint16_t slot, int16_t value) {
    if (!advance_record(accumulator, record)) return;
    const Air10Edf::Schema &schema = *accumulator.output.schema;
    const int signal = signal_index(schema, label);
    if (signal < 0 || slot >= schema.signals[signal].samples_per_record) return;
    accumulator.samples[signal_sample_offset(schema, signal) + slot] = value;
}

static uint32_t relative_ms(uint32_t captured_ms) {
    return static_cast<uint32_t>(captured_ms - session_clock.captured_ms);
}

static bool advance_segment(uint32_t captured_ms);
static void drain_raw_frames(TickType_t wait);
static void sample_oximetry();

static bool mean_mkp(uint32_t target_ms, int16_t &value) {
    if (!mkp_history) return false;
    uint32_t end = target_ms;
    int64_t weighted = 0;
    for (uint16_t i = 0; i < mkp_count; i++) {
        const auto &sample = mkp_history[(mkp_next + MKP_HISTORY_COUNT - 1 - i) %
                                          MKP_HISTORY_COUNT];
        const int32_t age = int32_t(target_ms - sample.sample_ms);
        if (age < 0) continue;
        // Bound sample-and-hold to two nominal periods; never bridge a stall.
        if (end - sample.sample_ms > 80) return false;
        const uint32_t begin = age >= MKP_WINDOW_MS
            ? target_ms - MKP_WINDOW_MS : sample.sample_ms;
        weighted += int64_t(sample.value) * (end - begin);
        if (age >= MKP_WINDOW_MS) {
            value = weighted / MKP_WINDOW_MS;
            return true;
        }
        end = begin;
    }
    return false;
}

static void process_wave(const RawFrame &raw, const StreamSchema &schema,
                         const DecodedFrame &decoded) {
    uint32_t sample_ms = relative_ms(raw.captured_ms);
    if (!wave_clock.initialized) {
        Log::logf(CAT_EDF, LOG_DEBUG,
                  "BRP first seq=%u rx=%lu relative=%lu payload=%.*s\n",
                  unsigned(decoded.sequence), (unsigned long)raw.captured_ms,
                  (unsigned long)sample_ms, int(raw.len), raw.payload);
        wave_clock.initialized = true;
        wave_clock.relative_ms = sample_ms;
    } else {
        const uint8_t delta = static_cast<uint8_t>(
            decoded.sequence - wave_clock.last_sequence);
        const uint32_t predicted = wave_clock.relative_ms +
                                   static_cast<uint32_t>(delta) * 40;
        const int32_t drift = static_cast<int32_t>(sample_ms - predicted);
        if (delta == 0) return;
        if (delta != 1) {
            portENTER_CRITICAL(&status_mux);
            const bool first = status.sequence_gaps++ == 0;
            portEXIT_CRITICAL(&status_mux);
            if (first) Log::logf(CAT_EDF, LOG_WARN,
                "BRP sequence discontinuity previous=%u current=%u delta=%u\n",
                unsigned(wave_clock.last_sequence), unsigned(decoded.sequence), unsigned(delta));
        }
        if (delta > 32 || drift > 400 || drift < -400) {
            Log::logf(CAT_EDF, LOG_DEBUG,
                      "BRP resync seq=%u delta=%u rx=%lu drift=%ld\n",
                      unsigned(decoded.sequence), unsigned(delta),
                      (unsigned long)raw.captured_ms, (long)drift);
            wave_clock.relative_ms = sample_ms;
            mkp_count = mkp_next = 0;
        } else
            wave_clock.relative_ms = predicted;
    }
    wave_clock.last_sequence = decoded.sequence;
    if (wave_clock.relative_ms >= segment_duration_ms)
        wave_clock.relative_ms = sample_ms;

    const uint32_t record = wave_clock.relative_ms / 60000;
    const uint16_t slot = (wave_clock.relative_ms % 60000) / 40;
    int16_t value;
    if (decoded_value(schema, decoded, "RFL", value))
        store_sample(brp, "Flow.40ms", record, slot, value);
    if (decoded_value(schema, decoded, "MKP", value)) {
        store_sample(brp, "Press.40ms", record, slot, value);
        if (mkp_history && value >= 0) {
            const uint32_t position = session_clock.captured_ms + record * 60000 + slot * 40;
            bool replaced = false;
            if (mkp_count) {
                auto &last = mkp_history[(mkp_next + MKP_HISTORY_COUNT - 1) % MKP_HISTORY_COUNT];
                if (position == last.sample_ms) {
                    last.value = value;
                    replaced = true;
                } else if (int32_t(position - last.sample_ms) < 0) {
                    mkp_count = mkp_next = 0;
                }
            }
            if (!replaced) {
                mkp_history[mkp_next] = {position, value};
                mkp_next = (mkp_next + 1) % MKP_HISTORY_COUNT;
                if (mkp_count < MKP_HISTORY_COUNT) mkp_count++;
            }
        }
    }
    if (decoded_value(schema, decoded, "TCV", value))
        store_sample(brp, "TrigCycEvt.40ms", record, slot, value);
}

static const TimedFrame *nearest_pbt(uint32_t target_ms) {
    const TimedFrame *nearest = nullptr;
    uint32_t distance = PBT_SAMPLE_WINDOW_MS + 1;
    for (uint8_t i = 0; i < pbt_sample_count; i++) {
        const int32_t offset = static_cast<int32_t>(pbt_samples[i].received_ms - target_ms);
        if (offset < -int32_t(PBT_SAMPLE_WINDOW_MS) ||
            offset > int32_t(PBT_SAMPLE_WINDOW_MS)) continue;
        const uint32_t candidate = offset < 0 ? -offset : offset;
        if (candidate < distance) {
            nearest = &pbt_samples[i];
            distance = candidate;
        }
    }
    return nearest;
}

static bool brh_inspiration(uint32_t target_ms, int16_t &value) {
    const StreamSchema *schema = stream_leases[4] >= 0 ? find_schema("BRH") : nullptr;
    if (!schema) return false;
    const TimedFrame *latest = nullptr;
    for (uint8_t i = 0; i < brh_sample_count; i++) {
        const auto &sample = brh_samples[i];
        if (int32_t(target_ms - sample.received_ms) < 0) continue;
        if (!latest || int32_t(sample.received_ms - latest->received_ms) > 0)
            latest = &sample;
    }
    if (!latest) return false;
    int16_t inspiration;
    if (!decoded_value(*schema, latest->frame, "INT", inspiration) || inspiration < 0)
        return false;
    // BRH is event-driven: a longer next breath does not invalidate the last INT.
    value = inspiration;
    Log::logf(CAT_EDF, LOG_DEBUG, "PLD #INT target=%lu raw=%d brh_rx=%lu age=%lu\n",
              (unsigned long)target_ms, int(value), (unsigned long)latest->received_ms,
              (unsigned long)(target_ms - latest->received_ms));
    return true;
}

static void sample_pld() {
    if (!status.active) return;
    static const struct {
        const char *label;
        const char *tag;
        bool pbt = false;
    } sources[] = {
        {"Press.2s", "MKI"},
        {"EprPress.2s", "MKE"},  {"Leak.2s", "LKF", true},
        {"RespRate.2s", "RRR", true}, {"TidVol.2s", "TDD"},
        {"IERatio.2s", "IER"},  {"B5ITime.2s", "IN5"},
        {"B5ETime.2s", "EX5"},  {"MinVent.2s", "MV5", true},
        {"TgtVent.2s", "TGT", true},
        {"AlvMinVent.2s", "AAV", true},
        {"Snore.2s", "SNI"},    {"FlowLim.2s", "FFL"},
    };

    const uint32_t started_ms = millis();
    const uint32_t elapsed = relative_ms(started_ms);
    // A short previous burst does not predict the next 40/80 ms response mix.
    // Use the full acquisition window without moving the EDF target.
    const uint32_t lead_ms = PLD_SAMPLE_WINDOW_MS;
    if (elapsed + lead_ms < 2000) return;
    const uint32_t absolute_slot = (elapsed + lead_ms) / 2000;
    if (last_pld_slot != UINT32_MAX && absolute_slot <= last_pld_slot) return;
    const uint32_t record = absolute_slot / 30;
    if (absolute_slot * 2000 >= segment_duration_ms) return;
    last_pld_slot = absolute_slot;
    const uint16_t slot = absolute_slot % 30;
    const uint32_t segment_start = session_clock.captured_ms;
    const uint32_t target_ms = segment_start + absolute_slot * 2000;
    const StreamSchema *pbt_schema = stream_leases[3] >= 0 ? find_schema("PBT") : nullptr;
    TimedFrame selected_pbt = {};
    const TimedFrame *pbt = nullptr;
    bool pbt_after_target = false;
    auto collect_frames = [&](TickType_t wait = 0) {
        drain_raw_frames(wait);
        if (!status.active || segment_start != session_clock.captured_ms ||
            !__atomic_load_n(&therapy_wanted, __ATOMIC_ACQUIRE) ||
            int32_t(millis() - target_ms) >= 2000) {
            Log::logf(CAT_EDF, LOG_DEBUG, "PLD series slot=%lu interrupted\n",
                      (unsigned long)absolute_slot);
            return false;
        }
        sample_oximetry();
        for (uint8_t i = 0; pbt_schema && i < pbt_sample_count; i++) {
            const int32_t offset = int32_t(pbt_samples[i].received_ms - target_ms);
            if (offset >= 0 && offset <= PBT_SAMPLE_WINDOW_MS)
                pbt_after_target = true;
        }
        const TimedFrame *candidate = pbt_schema ? nearest_pbt(target_ms) : nullptr;
        if (candidate && (!pbt || abs(int32_t(candidate->received_ms - target_ms)) <
                                 abs(int32_t(pbt->received_ms - target_ms)))) {
            selected_pbt = *candidate;
            pbt = &selected_pbt;
        }
        return true;
    };

    constexpr size_t count = sizeof(sources) / sizeof(sources[0]);
    int16_t values[count];
    uint32_t deferred = 0, polled = 0;
    uint8_t poll_count = 0;
    uint8_t queries = 0, streamed = 0;
    if (!collect_frames()) return;
    for (size_t i = 0; i < count; i++) {
        values[i] = EDF_MISSING;
        const auto &source = sources[i];
        if (signal_index(*pld.output.schema, source.label) < 0) continue;
        if (source.pbt && pbt_schema && schema_has_field(*pbt_schema, source.tag)) {
            deferred |= uint32_t(1) << i;
            continue;
        }
        polled |= uint32_t(1) << i;
        poll_count++;
    }

    // One nominal UART turn per planned read; slower replies consume the slack.
    // Missing stream fields share the same fixed window; late reads never extend it.
    const uint32_t spacing_ms = poll_count
        ? min(uint32_t(40), uint32_t(2 * PLD_SAMPLE_WINDOW_MS) / poll_count) : 0;
    uint32_t not_before = target_ms - poll_count * spacing_ms / 2;
    if (poll_count) {
        const uint32_t defer_at = not_before - Arbiter::AcquisitionWindow::lead_ms();
        Arbiter::AcquisitionWindow acquisition(target_ms + PLD_SAMPLE_WINDOW_MS, defer_at);
        for (size_t i = 0; i < count; i++) {
            if (!(polled & (uint32_t(1) << i))) continue;
            while (int32_t(millis() - not_before) < 0) {
                if (!collect_frames(pdMS_TO_TICKS(10))) return;
            }
            not_before += spacing_ms;
            const auto &source = sources[i];
            (void)read_numeric_variable(source.tag, values[i], absolute_slot, target_ms, queries);
            if (!collect_frames()) return;
        }
    }

    // Bracket the PBT target before choosing nearest, even after a fast burst.
    // A stalled stream cannot hold the worker beyond its freshness window.
    while (int32_t(millis() - target_ms) < 0 ||
           (deferred && !pbt_after_target &&
            int32_t(millis() - target_ms) < PBT_SAMPLE_WINDOW_MS)) {
        if (!collect_frames(pdMS_TO_TICKS(20))) return;
    }

    // Freeze one frame for all covered fields; only failed fields need late G S.
    for (size_t i = 0; i < count; i++) {
        if (!(deferred & (uint32_t(1) << i))) continue;
        const auto &source = sources[i];
        if (pbt && decoded_value(*pbt_schema, pbt->frame, source.tag, values[i]) &&
            values[i] >= 0) {
            streamed++;
        } else {
            values[i] = EDF_MISSING;
            Arbiter::AcquisitionWindow acquisition(target_ms + PLD_SAMPLE_WINDOW_MS);
            (void)read_numeric_variable(source.tag, values[i], absolute_slot, target_ms, queries);
            drain_raw_frames(0);
            if (!status.active || segment_start != session_clock.captured_ms ||
                !__atomic_load_n(&therapy_wanted, __ATOMIC_ACQUIRE)) return;
        }
    }
    const uint32_t finished_ms = millis();
    Log::logf(CAT_EDF, LOG_DEBUG,
              "PLD series slot=%lu target=%lu start=%lu end=%lu queries=%u pbt=%u pbt_rx=%lu\n",
              (unsigned long)absolute_slot, (unsigned long)target_ms,
              (unsigned long)started_ms, (unsigned long)finished_ms,
              unsigned(queries), unsigned(streamed),
              (unsigned long)(streamed ? pbt->received_ms : 0));
    // Do not publish an obsolete burst into a previous slot after a long stall.
    if (int32_t(finished_ms - target_ms) >= 2000) return;
    for (size_t i = 0; i < count; i++)
        store_sample(pld, sources[i].label, record, slot, values[i]);
    int16_t mask_pressure = EDF_MISSING;
    (void)mean_mkp(target_ms, mask_pressure);
    store_sample(pld, "MaskPress.2s", record, slot, mask_pressure);
    int16_t inspiration = EDF_MISSING;
    (void)brh_inspiration(target_ms, inspiration);
    store_sample(pld, "Ti.2s", record, slot, inspiration);
}

static bool append_annotation(OutputFile &output, uint32_t onset,
                              uint32_t duration, const char *label) {
    uint8_t record[40];
    size_t written = 0;
    if (!Air10Edf::render_annotation_record(
            *output.schema, onset, duration, label,
            record, sizeof(record), written)) {
        status_error("annotation encoding failed");
        return false;
    }
    return append_output_record(output, record, written);
}

static void process_apnea(const RawFrame &raw, const StreamSchema &schema,
                          const DecodedFrame &decoded) {
    int16_t event_type = 0;
    int16_t duration = 0;
    if (!decoded_value(schema, decoded, "AET", event_type) || event_type == 0)
        return;
    (void)decoded_value(schema, decoded, "DUR", duration);
    const char *label = nullptr;
    switch (event_type) {
        case 1: label = "Hypopnea"; break;
        case 2: label = "Central Apnea"; break;
        case 3: label = "Obstructive Apnea"; break;
        case 4: label = "Apnea"; break;
        case 5: label = "Arousal"; break;
        default: return;
    }
    const uint32_t onset = relative_ms(raw.captured_ms) / 1000;
    const uint32_t safe_duration = duration > 0 ? duration : 0;
    Log::logf(CAT_EDF, LOG_DEBUG, "APN rx=%lu type=%d duration=%lu onset=%lu\n",
              (unsigned long)raw.captured_ms, int(event_type),
              (unsigned long)safe_duration, (unsigned long)onset);
    // Native EVE timestamps the reported event, without subtracting DUR.
    append_annotation(eve, onset, safe_duration, label);
}

static uint32_t csr_onset(const RawFrame &raw, int32_t event_time) {
    const uint32_t arrival = relative_ms(raw.captured_ms) / 1000;
    if (!session_clock.native_valid || event_time < 0 || event_time >= 86400)
        return arrival;
    // CET counts seconds from noon; the calendar anchor starts at midnight.
    int32_t candidate = static_cast<int32_t>(event_time) -
                        static_cast<int32_t>((session_clock.native_start + 43200) % 86400);
    while (candidate < 0) candidate += 86400;
    while (candidate + 43200 < static_cast<int32_t>(arrival)) candidate += 86400;
    while (candidate > static_cast<int32_t>(arrival) + 43200) candidate -= 86400;
    return candidate >= 0 ? static_cast<uint32_t>(candidate) : arrival;
}

static void process_csr(const RawFrame &raw, const StreamSchema &schema,
                        const DecodedFrame &decoded) {
    int16_t event_type = 0;
    int32_t event_time = -1;
    if (!decoded_value(schema, decoded, "CSR", event_type)) return;
    (void)decoded_value32(schema, decoded, "CET", event_time);
    const char *label = event_type == 1 ? "CSR Start" :
                        event_type == 2 ? "CSR End" : nullptr;
    if (label) append_annotation(csl, csr_onset(raw, event_time), 0, label);
}

static void process_raw_frame(const RawFrame &raw) {
    if (raw.len < 3 || !status.active || !advance_segment(raw.captured_ms)) return;
    if (int32_t(raw.captured_ms - session_clock.captured_ms) < 0) {
        portENTER_CRITICAL(&status_mux);
        status.raw_dropped++;
        portEXIT_CRITICAL(&status_mux);
        return;
    }
    char tag[4] = {
        static_cast<char>(raw.payload[0]),
        static_cast<char>(raw.payload[1]),
        static_cast<char>(raw.payload[2]),
        0,
    };
    StreamSchema *schema = find_schema(tag);
    DecodedFrame decoded = {};
    if (!schema || !Air10Stream::decode_frame(raw.payload, raw.len, *schema, decoded)) {
        portENTER_CRITICAL(&status_mux);
        const bool first = status.decode_errors++ == 0;
        portEXIT_CRITICAL(&status_mux);
        if (first) Log::logf(CAT_EDF, LOG_WARN,
            "Stream rejected tag=%s bytes=%u reason=%s\n",
            tag, unsigned(raw.len), schema ? "decode_failed" : "schema_missing");
        return;
    }

    if (strcmp(tag, wave_tag) == 0) process_wave(raw, *schema, decoded);
    else if (strcmp(tag, "APN") == 0) process_apnea(raw, *schema, decoded);
    else if (strcmp(tag, "CSN") == 0) process_csr(raw, *schema, decoded);
    else if (strcmp(tag, "PBT") == 0) {
        pbt_samples[pbt_next_sample] = {raw.captured_ms, decoded};
        pbt_next_sample = (pbt_next_sample + 1) % PBT_SAMPLE_COUNT;
        if (pbt_sample_count < PBT_SAMPLE_COUNT) pbt_sample_count++;
    } else if (strcmp(tag, "BRH") == 0) {
        brh_samples[brh_next_sample] = {raw.captured_ms, decoded};
        brh_next_sample = (brh_next_sample + 1) % BRH_SAMPLE_COUNT;
        if (brh_sample_count < BRH_SAMPLE_COUNT) brh_sample_count++;
    }
}

static void drain_raw_frames(TickType_t wait) {
    RawFrame raw;
    if (xQueueReceive(raw_queue, &raw, wait) != pdTRUE) return;
    uint16_t processed = 0;
    do {
        if (status.active) process_raw_frame(raw);
    } while (++processed < raw_queue_capacity &&
             __atomic_load_n(&therapy_wanted, __ATOMIC_ACQUIRE) &&
             xQueueReceive(raw_queue, &raw, 0) == pdTRUE);
}

static void sample_oximetry() {
    if (!status.active) return;
    const uint32_t now = millis();
    const uint32_t elapsed = min(relative_ms(now), segment_duration_ms);
    const uint32_t last_slot = min(elapsed / 1000, (segment_duration_ms - 1) / 1000);
    uint32_t first_slot = last_oxi_slot == UINT32_MAX ? 0 : last_oxi_slot + 1;
    // Bounded catch-up after worker stalls; never rewrite a closed minute.
    first_slot = max(first_slot, max(sad.current_record * 60,
                                     last_slot > 2 ? last_slot - 2 : 0));
    for (uint32_t absolute_slot = first_slot; absolute_slot <= last_slot; absolute_slot++) {
        last_oxi_slot = absolute_slot;
        const uint32_t target_ms = session_clock.captured_ms + absolute_slot * 1000;
        oxi_reading_t reading;
        if (!OxiArbiter::snapshot_at(target_ms, reading) || !reading.valid ||
            int32_t(reading.timestamp_ms - session_clock.captured_ms) < 0 ||
            target_ms - reading.timestamp_ms > 2000) continue;
        // State at the boundary, not a future reading rounded down into this slot.
        const uint32_t record = absolute_slot / 60;
        const uint16_t slot = absolute_slot % 60;
        store_sample(sad, "Pulse.1s", record, slot, reading.pulse_bpm);
        store_sample(sad, "SpO2.1s", record, slot, reading.spo2);
    }
    (void)advance_record(sad, elapsed / 60000);
}

static bool frame_sink(const qframe_t *frame, void *) {
    if (!capture_active || !frame || frame->type != QFRAME_TYPE_L ||
        frame->payload_len < 3 || frame->payload_len > RAW_PAYLOAD_MAX) {
        return true;
    }

    const bool wanted = memcmp(wave_tag, frame->payload, 3) == 0 ||
                        (stream_leases[1] >= 0 && memcmp("APN", frame->payload, 3) == 0) ||
                        (stream_leases[2] >= 0 && memcmp("CSN", frame->payload, 3) == 0) ||
                        (stream_leases[3] >= 0 && memcmp("PBT", frame->payload, 3) == 0) ||
                        (stream_leases[4] >= 0 && memcmp("BRH", frame->payload, 3) == 0);
    if (!wanted) return true;

    RawFrame raw = {};
    raw.captured_ms = millis();
    raw.len = frame->payload_len;
    memcpy(raw.payload, frame->payload, frame->payload_len);
    if (xQueueSend(raw_queue, &raw, 0) != pdTRUE) {
        portENTER_CRITICAL(&status_mux);
        status.raw_dropped++;
        portEXIT_CRITICAL(&status_mux);
    }
    return true;
}

static void release_streams() {
    for (LiveStream::internal_handle_t &lease : stream_leases) {
        if (lease >= 0) LiveStream::release_internal(lease);
        lease = -1;
    }
}

static void acquire_stream(uint8_t slot, const char *tag) {
    StreamSchema *schema = find_schema(tag);
    if (!schema || !schema->field_count) return;
    stream_leases[slot] = LiveStream::acquire_internal(tag);
    if (stream_leases[slot] < 0)
        Log::logf(CAT_EDF, LOG_WARN, "%s subscribe failed\n", tag);
}

static bool acquire_streams() {
    for (LiveStream::internal_handle_t &lease : stream_leases) lease = -1;
    acquire_stream(0, "TCE");
    if (stream_leases[0] < 0) return false;
    acquire_stream(1, "APN");
    acquire_stream(2, "CSN");
    acquire_stream(3, "PBT");
    acquire_stream(4, "BRH");
    return true;
}

static void clear_session_memory(bool remove_partial) {
    release_accumulator(brp, remove_partial);
    release_accumulator(pld, remove_partial);
    release_accumulator(sad, remove_partial);
    close_output(eve, remove_partial);
    close_output(csl, remove_partial);
    aircannect::Memory::free(header_buffer);
    header_buffer = nullptr;
    header_capacity = 0;
    aircannect::Memory::free(mkp_history);
    mkp_history = nullptr;
    mkp_count = mkp_next = 0;
}

static bool anchor_session_clock(const ControlEvent &event) {
    session_clock = {};
    session_phase = {};
    session_clock.captured_ms = event.captured_ms;
    Air10Clock::Calendar native_now;
    uint32_t clock_ms = 0;
    session_clock.native_valid = Air10Clock::read(native_now, POLL_TIMEOUT_MS, &clock_ms);
    if (!session_clock.native_valid) {
        status_error("native clock unavailable");
        return false;
    }
    session_clock.native_start = Air10Clock::civil_seconds(native_now) -
        static_cast<uint32_t>(clock_ms - event.captured_ms) / 1000;
    if (Air10Clock::phase_anchor(session_phase)) {
        const int64_t start_ms = session_phase.native_at_ms(event.captured_ms);
        session_clock.native_start = start_ms / 1000;
        // Map the full-second header to the same monotonic origin for every file.
        const uint32_t fraction_ms = static_cast<uint32_t>(start_ms % 1000);
        session_clock.captured_ms = event.captured_ms - fraction_ms;
        Log::logf(CAT_EDF, LOG_DEBUG,
                  "clock phase offset=%lums uncertainty=%ums age=%lums\n",
                  (unsigned long)fraction_ms, unsigned(session_phase.uncertainty_ms),
                  (unsigned long)(clock_ms - session_phase.captured_ms));
    } else {
        Log::logf(CAT_EDF, LOG_DEBUG, "clock phase unavailable; using whole-second read\n");
    }
    Log::logf(CAT_EDF, LOG_DEBUG,
              "anchor start_ms=%lu clock_ms=%lu tic=%02d:%02d:%02d native=%lld\n",
              (unsigned long)event.captured_ms, (unsigned long)clock_ms,
              native_now.hour, native_now.minute, native_now.second,
              (long long)session_clock.native_start);
    const int64_t day = (session_clock.native_start - 43200) / 86400;
    if (day < 0x1000 || day >= 0xffff) {
        status_error("therapy day is outside Air10 range");
        return false;
    }
    session_native_day = static_cast<uint16_t>(day);
    return true;
}

static bool make_paths_and_metadata(const ControlEvent &event,
                                    uint16_t &mid, uint16_t &vid,
                                    bool rollover = false) {
    if (!rollover && !AirSenseState::identity(recording_identity)) {
        AirSenseState::request_refresh();
        status_error("device identity not ready");
        return false;
    }
    if (!rollover && !Air10Stream::snapshot(recording_identity.generation, stream_schemas)) {
        status_error("stream schemas not ready");
        return false;
    }
    if (!rollover && !anchor_session_clock(event)) return false;
    const time_t civil = static_cast<time_t>(session_clock.native_start);
    struct tm start_tm;
    gmtime_r(&civil, &start_tm);
    snprintf(start_date, sizeof(start_date), "%02d.%02d.%02d",
             start_tm.tm_mday, start_tm.tm_mon + 1,
             (start_tm.tm_year + 1900) % 100);
    snprintf(start_time, sizeof(start_time), "%02d.%02d.%02d",
             start_tm.tm_hour, start_tm.tm_min, start_tm.tm_sec);

    const time_t day_civil = int64_t(Air10Clock::therapy_day(civil)) * 86400;
    struct tm therapy_tm;
    gmtime_r(&day_civil, &therapy_tm);
    char therapy_day[9];
    snprintf(therapy_day, sizeof(therapy_day), "%04d%02d%02d",
             therapy_tm.tm_year + 1900, therapy_tm.tm_mon + 1,
             therapy_tm.tm_mday);
    char prefix[16];
    snprintf(prefix, sizeof(prefix), "%04d%02d%02d_%02d%02d%02d",
             start_tm.tm_year + 1900, start_tm.tm_mon + 1, start_tm.tm_mday,
             start_tm.tm_hour, start_tm.tm_min, start_tm.tm_sec);

    if (!storage->exists("/DATALOG") && !storage->mkdir("/DATALOG")) {
        status_error("cannot create DATALOG");
        return false;
    }
    snprintf(session_directory, sizeof(session_directory),
             "/DATALOG/%s", therapy_day);
    if (!storage->exists(session_directory) &&
        !storage->mkdir(session_directory)) {
        status_error("cannot create therapy-day directory");
        return false;
    }

    portENTER_CRITICAL(&status_mux);
    strncpy(status.therapy_day, therapy_day, sizeof(status.therapy_day));
    strncpy(status.file_prefix, prefix, sizeof(status.file_prefix));
    portEXIT_CRITICAL(&status_mux);

    mid = recording_identity.mid;
    vid = recording_identity.vid;
    snprintf(recording_id, sizeof(recording_id),
             "Startdate %02d-%s-%04d X X X SRN=%s  MID=%u  VID=%u",
             start_tm.tm_mday, MONTH_NAMES[start_tm.tm_mon],
             start_tm.tm_year + 1900, recording_identity.srn, mid, vid);
    return true;
}

static bool allocate_session_buffers(const Air10Edf::Schema &brp_schema,
                                     const Air10Edf::Schema &pld_schema) {
    header_capacity = Air10Edf::header_size(pld_schema);
    const size_t brp_header = Air10Edf::header_size(brp_schema);
    if (brp_header > header_capacity) header_capacity = brp_header;
    header_buffer = static_cast<uint8_t *>(aircannect::Memory::alloc_large(header_capacity));

    if (!header_buffer) return false;

    mkp_history = static_cast<PressureSample *>(aircannect::Memory::alloc_large(
        MKP_HISTORY_COUNT * sizeof(PressureSample)));
    if (!mkp_history) return false;

    if (!initialize_accumulator(brp, brp_schema) ||
        !initialize_accumulator(pld, pld_schema) ||
        !initialize_accumulator(sad, Air10Edf::sad_schema()) ||
        !open_output(eve, Air10Edf::eve_schema()) ||
        !open_output(csl, Air10Edf::csl_schema())) {
        return false;
    }
    return append_annotation(eve, 0, 0, "Recording starts") &&
           append_annotation(csl, 0, 0, "Recording starts");
}

static void discard_pending() {
    char path[80], part[88], backup[88];
    pending_path(status.file_prefix, path, sizeof(path));
    snprintf(part, sizeof(part), "%s.part", path);
    snprintf(backup, sizeof(backup), "%s.bak", path);
    if (storage->remove(path)) {
        portENTER_CRITICAL(&status_mux);
        if (status.pending_str) status.pending_str--;
        portEXIT_CRITICAL(&status_mux);
    }
    storage->remove(part);
    storage->remove(backup);
}

static bool begin_segment_files(const Air10Edf::Schema &brp_schema,
                                 const Air10Edf::Schema &pld_schema) {
    char path[80], part[88], backup[88];
    pending_path(status.file_prefix, path, sizeof(path));
    snprintf(part, sizeof(part), "%s.part", path);
    snprintf(backup, sizeof(backup), "%s.bak", path);
    if (storage->exists(path) || storage->exists(part) || storage->exists(backup))
        return false;
    // Persist intent before any EDF can be recovered and catalogued after reset.
    if (!save_pending(recording_identity)) {
        discard_pending();
        return false;
    }
    if (allocate_session_buffers(brp_schema, pld_schema)) return true;
    clear_session_memory(true);
    discard_pending();
    return false;
}

static void reset_session_state(bool rollover = false) {
    segment_duration_ms = Air10Clock::milliseconds_to_noon(session_clock.native_start);
    wave_clock = {};
    pbt_sample_count = 0;
    pbt_next_sample = 0;
    brh_sample_count = 0;
    brh_next_sample = 0;
    mkp_count = mkp_next = 0;
    last_pld_slot = UINT32_MAX;
    last_oxi_slot = UINT32_MAX;
    if (!rollover) xQueueReset(raw_queue);
}

static void start_session(const ControlEvent &event) {
    __atomic_store_n(&therapy_start_pending, false, __ATOMIC_RELEASE);
    next_start_ms = millis() + 5000;
    if (!SdStorage::local_access_allowed() || status.active || !storage_ready || !__atomic_load_n(&therapy_wanted, __ATOMIC_ACQUIRE) ||
        Arbiter::get_state() != SYS_THERAPY || AirSenseState::rop() != 1) return;
    portENTER_CRITICAL(&status_mux);
    const uint32_t mask_on_ms = therapy_on_capture_ms;
    portEXIT_CRITICAL(&status_mux);
    if (int32_t(event.captured_ms - mask_on_ms) < 0) return;
    if (!SdStorage::acquire()) {
        if (SdStorage::local_access_allowed()) status_error("storage busy at recording start");
        return;
    }
    recording_storage_owned = true;
    uint16_t mid = 0;
    uint16_t vid = 0;
    portENTER_CRITICAL(&status_mux);
    status.last_error[0] = 0;
    portEXIT_CRITICAL(&status_mux);
    if (!make_paths_and_metadata(event, mid, vid)) {
        recording_storage_owned = false;
        SdStorage::release();
        return;
    }
    const StreamSchema *tce_schema = find_schema("TCE");
    const bool tcv = tce_schema && schema_has_field(*tce_schema, "TCV");
    const Air10Edf::Schema &brp_layout = Air10Edf::brp_schema(tcv);
    const StreamSchema *brh_schema = find_schema("BRH");
    pld_layout = Air10Edf::pld_schema(
        brh_schema && schema_has_field(*brh_schema, "INT"),
        brh_schema && schema_has_field(*brh_schema, "EXT"), pld_signals);

    reset_session_state();
    if (!acquire_streams()) {
        release_streams();
        status_error("TCE subscription failed");
        recording_storage_owned = false;
        SdStorage::release();
        return;
    }
    if (!begin_segment_files(brp_layout, pld_layout)) {
        status_error("session buffer or file initialization failed");
        release_streams();
        clear_session_memory(true);
        recording_storage_owned = false;
        SdStorage::release();
        return;
    }

    portENTER_CRITICAL(&status_mux);
    const bool same_therapy = mask_on_ms == therapy_on_capture_ms;
    portEXIT_CRITICAL(&status_mux);
    if (!same_therapy || !__atomic_load_n(&therapy_wanted, __ATOMIC_ACQUIRE) ||
        Arbiter::get_state() != SYS_THERAPY || AirSenseState::rop() != 1) {
        release_streams();
        clear_session_memory(true);
        discard_pending();
        recording_storage_owned = false;
        SdStorage::release();
        return;
    }

    portENTER_CRITICAL(&status_mux);
    recording_therapy_on_ms = mask_on_ms;
    status.active = true;
    status.post_processing = false;
    status.raw_dropped = 0;
    status.decode_errors = 0;
    status.sequence_gaps = 0;
    status.write_errors = 0;
    status.post_errors = 0;
    status.brp_records = 0;
    status.pld_records = 0;
    status.sad_records = 0;
    status.eve_records = 1;
    status.csl_records = 1;
    portEXIT_CRITICAL(&status_mux);

    capture_active = true;
    Log::logf(CAT_EDF, LOG_INFO,
              "recording %s/%s MID=%u VID=%u\n",
              status.therapy_day, status.file_prefix, mid, vid);
}

static void update_record_status() {
    portENTER_CRITICAL(&status_mux);
    status.brp_records = brp.output.records;
    status.pld_records = pld.output.records;
    status.sad_records = sad.output.records;
    status.eve_records = eve.records;
    status.csl_records = csl.records;
    portEXIT_CRITICAL(&status_mux);
}

static void close_segment(uint32_t captured_ms) {
    const uint32_t records = min(relative_ms(captured_ms), segment_duration_ms) / 60000;
    // Flush elapsed full minutes only; the unfinished tail stays in RAM.
    bool complete = advance_record(brp, records);
    complete = advance_record(pld, records) && complete;
    complete = advance_record(sad, records) && complete;
    update_record_status();

    // Attempt every finalization even after an earlier output failed.
    for (OutputFile *output : {&brp.output, &pld.output, &sad.output, &eve, &csl})
        complete = finalize_output(*output) && complete;
    update_record_status();
    Status finished;
    portENTER_CRITICAL(&status_mux);
    finished = status;
    portEXIT_CRITICAL(&status_mux);
    complete = complete && !finished.write_errors;
    clear_session_memory(false);

    EdfCatalog::Entry catalog_entry;
    complete = commit_session_catalog(false, false, catalog_entry) && complete;
    const bool gaps = finished.raw_dropped || finished.decode_errors || finished.sequence_gaps;
    Log::logf(CAT_EDF, !complete ? LOG_ERROR : gaps ? LOG_WARN : LOG_INFO,
              "%s %s drops=%u decode=%u seq_gaps=%u\n",
              !complete ? "incomplete" : gaps ? "complete with gaps" : "complete",
              finished.file_prefix, finished.raw_dropped, finished.decode_errors,
              finished.sequence_gaps);
    Log::logf(CAT_EDF, LOG_DEBUG, "records BRP=%u PLD=%u SAD=%u EVE=%u CSL=%u\n",
              finished.brp_records, finished.pld_records,
              finished.sad_records, finished.eve_records, finished.csl_records);
}

static bool advance_segment(uint32_t captured_ms) {
    if (int32_t(captured_ms - session_clock.captured_ms) < 0) return true;
    if (relative_ms(captured_ms) < segment_duration_ms) return true;

    const uint32_t boundary = session_clock.captured_ms + segment_duration_ms;
    const uint32_t seconds = segment_duration_ms / 1000;
    close_segment(boundary);
    session_clock.native_start += seconds;
    session_clock.captured_ms = boundary;
    session_native_day = Air10Clock::therapy_day(session_clock.native_start);
    ControlEvent event = {ControlKind::Start, boundary};
    uint16_t mid = recording_identity.mid, vid = recording_identity.vid;
    const StreamSchema *schema = find_schema("TCE");
    const bool tcv = schema && schema_has_field(*schema, "TCV");
    const bool opened = make_paths_and_metadata(event, mid, vid, true) &&
        begin_segment_files(Air10Edf::brp_schema(tcv), pld_layout);
    if (!opened) {
        capture_active = false;
        release_streams();
        clear_session_memory(true);
        portENTER_CRITICAL(&status_mux);
        status.active = false;
        portEXIT_CRITICAL(&status_mux);
        if (recording_storage_owned) SdStorage::release();
        recording_storage_owned = false;
        status_error("noon segment initialization failed");
        return false;
    }
    reset_session_state(true);
    Log::logf(CAT_EDF, LOG_INFO, "noon rollover %s/%s\n",
              status.therapy_day, status.file_prefix);
    return true;
}

static void stop_session(const ControlEvent &event) {
    if (!status.active) return;
    capture_active = false;
    release_streams();
    RawFrame raw;
    while (xQueueReceive(raw_queue, &raw, 0) == pdTRUE) {
        // Frames arriving after Stop must not extend or roll over the segment.
        if (int32_t(raw.captured_ms - event.captured_ms) < 0)
            process_raw_frame(raw);
    }
    if (status.active) {
        // Do not create an empty next-day segment for a stop exactly at noon.
        if (relative_ms(event.captured_ms) > segment_duration_ms)
            (void)advance_segment(event.captured_ms - 1);
        if (status.active) close_segment(event.captured_ms);
    }

    portENTER_CRITICAL(&status_mux);
    status.active = false;
    // Keep clock synchronization out of the gap before the first STR check.
    status.post_processing = true;
    portEXIT_CRITICAL(&status_mux);
    SdStorage::refresh_usage();
    if (recording_storage_owned) SdStorage::release();
    recording_storage_owned = false;
    next_pending_ms = 0;
    next_start_ms = 0;
}

static bool refresh_str_day(uint16_t day, uint32_t generation,
                             const AirSenseState::Identity &identity,
                             bool required, const char *reason) {
    const time_t civil = int64_t(day) * 86400;
    struct tm date;
    gmtime_r(&civil, &date);
    char day_text[9];
    snprintf(day_text, sizeof(day_text), "%04d%02d%02d",
             date.tm_year + 1900, date.tm_mon + 1, date.tm_mday);

    const uint32_t snapshot_started = millis();
    if (!SdStorage::try_acquire()) return false;
    EdfCatalog::Entry *entries = nullptr;
    uint32_t count = 0;
    const bool valid = EdfCatalog::snapshot_day(day_text, entries, count);
    SdStorage::release();
    const uint32_t snapshot_ms = millis() - snapshot_started;
    if (!valid || !count) {
        aircannect::Memory::free(entries);
        return valid && !required;
    }
    uint32_t latest_index = 0;
    for (uint32_t i = 1; i < count; i++)
        if (strcmp(entries[i].file_prefix, entries[latest_index].file_prefix) > 0)
            latest_index = i;
    EdfCatalog::Entry &latest = entries[latest_index];

    session_native_day = day;
    uint8_t *record = nullptr;
    StrReadTrace trace;
    const uint32_t read_started = millis();
    bool success = ensure_identification(identity);
    const bool read_attempted = success;
    bool summary_ready = success && collect_str_summary(record, generation, trace);
    const uint32_t read_ms = millis() - read_started;
    const uint32_t publish_started = millis();
    bool identification_changed = false, ready_changed = false, content_changed = false;
    if (identity.generation != AirSenseState::identity_generation())
        trace.result = "device_changed";
    if (post_processing_cancelled()) trace.result = "interrupted";
    success = success && !post_processing_cancelled() &&
        identity.generation == AirSenseState::identity_generation();
    if (success && SdStorage::try_acquire()) {
        uint32_t revision = 0;
        bool catalog_changed = false;
        if (summary_ready && record) {
            summary_ready = update_str_file(record, latest, identity);
            if (!summary_ready) trace.result = "publish_failed";
            // A content token survives retries after STR was published but the
            // catalog was not. Consumers compare revisions, never order them.
            uint8_t identification[4];
            SdStorage::put_le32(identification, identification_crc);
            uint32_t crc = crc32_ieee_update(crc32_ieee_initial(), record,
                Air10Edf::record_size(Air10Edf::str_schema()));
            revision = crc32_ieee_finish(crc32_ieee_update(crc, identification, sizeof(identification)));
        }
        for (uint32_t i = 0; success && i < count; i++) {
            success = !post_processing_cancelled();
            if (!success) {
                trace.result = "interrupted";
                break;
            }
            EdfCatalog::Entry &entry = entries[i];
            const uint8_t old_flags = entry.flags;
            const uint32_t old_revision = entry.str_revision;
            entry.flags |= EdfCatalog::ENTRY_IDENTIFICATION_READY;
            if (summary_ready && record) {
                entry.flags |= EdfCatalog::ENTRY_STR_READY;
                if (i == latest_index) entry.str_revision = revision;
            }
            if (old_flags != entry.flags || old_revision != entry.str_revision) {
                success = EdfCatalog::commit(entry);
                if (success) {
                    catalog_changed = true;
                    identification_changed |= !(old_flags & EdfCatalog::ENTRY_IDENTIFICATION_READY);
                    ready_changed |= !(old_flags & EdfCatalog::ENTRY_STR_READY) &&
                        (entry.flags & EdfCatalog::ENTRY_STR_READY);
                    content_changed |= old_revision != entry.str_revision;
                } else {
                    trace.result = "catalog_failed";
                }
            }
        }
        for (uint32_t i = 0; success && summary_ready && i < count; i++) {
            char path[80];
            pending_path(entries[i].file_prefix, path, sizeof(path));
            if (storage->exists(path)) {
                success = storage->remove(path);
                if (success) {
                    portENTER_CRITICAL(&status_mux);
                    if (status.pending_str) status.pending_str--;
                    portEXIT_CRITICAL(&status_mux);
                } else {
                    trace.result = "journal_failed";
                }
            }
        }
        SdStorage::release();
        if (catalog_changed && summary_ready && record) {
            summary_export_entry = latest;
            summary_export_pending = true;
        }
    } else {
        if (success) trace.result = "storage_busy";
        success = false;
    }
    char after[12] = "-";
    if (trace.generation_read)
        snprintf(after, sizeof(after), "%lu", (unsigned long)trace.generation);
    const bool interrupted = post_processing_cancelled();
    if (interrupted) trace.result = "interrupted";
    const bool unchanged = success && summary_ready &&
        !identification_changed && !ready_changed && !content_changed;
    Log::logf(CAT_EDF, read_attempted && !interrupted && !unchanged ? LOG_INFO : LOG_DEBUG,
              "STR %s %s ZEN=%lu>%s %s "
              "ident+=%u ready+=%u token=%u read=%lu sd=%lums\n",
              day_text, reason, (unsigned long)generation, after, trace.result,
              unsigned(identification_changed), unsigned(ready_changed), unsigned(content_changed),
              (unsigned long)read_ms,
              (unsigned long)(snapshot_ms + millis() - publish_started));
    aircannect::Memory::free(entries);
    aircannect::Memory::free(record);
    if (success && summary_ready && post_error_active) {
        post_error_active = false;
        Log::logf(CAT_EDF, LOG_INFO, "STR processing recovered day=%04X\n", day);
    }
    return success && summary_ready;
}

// Caller holds storage access. This inventory does not depend on the catalog or UART.
static bool scan_pending(EdfPending::Record *selected = nullptr) {
    if (selected) *selected = {};
    uint32_t count = 0;
    const char *error = nullptr;
    errno = 0;
    fs::File dir = storage->open("/airbridge/pending");
    bool complete = dir ? dir.isDirectory() : errno == ENOENT;
    if (!complete) error = "STR pending directory unavailable";
    if (dir && dir.isDirectory()) {
        while (true) {
            errno = 0;
            fs::File item = dir.openNextFile();
            if (!item) {
                if (errno) {
                    complete = false;
                    error = "STR pending directory read failed";
                }
                break;
            }
            String path = item.path();
            item.close();
            if (!path.endsWith(".str")) continue;
            EdfPending::Record candidate;
            const PendingRead result = read_pending(path.c_str(), candidate);
            if (result == PendingRead::Invalid) {
                String rejected = path;
                rejected += ".rejected";
                if (!storage->exists(rejected) && storage->rename(path, rejected)) {
                    Log::logf(CAT_EDF, LOG_WARN, "STR journal rejected: %s -> %s\n",
                              path.c_str(), rejected.c_str());
                    continue;
                }
                count++;
                error = "STR journal quarantine failed";
                continue;
            }
            count++;
            if (result == PendingRead::Unavailable) {
                error = "STR pending journal read failed";
                continue;
            }
            if (!selected) continue;
            if (strcmp(candidate.prefix, pending_cursor) <= 0) continue;
            if (!selected->prefix[0] || strcmp(candidate.prefix, selected->prefix) < 0)
                *selected = candidate;
        }
    }
    dir.close();
    portENTER_CRITICAL(&status_mux);
    if (complete) status.pending_str = count;
    pending_scanned = complete;
    const bool new_error = error && error != pending_scan_error;
    pending_scan_error = error;
    portEXIT_CRITICAL(&status_mux);
    if (new_error) post_error(error);
    return complete;
}

static void request_summary_export() {
    if (!status.pending_str && summary_export_pending &&
        ExportSync::request_post_therapy(summary_export_entry))
        summary_export_pending = false;
}

static void sync_pending() {
    EdfPending::Record selected;
    if (!pending_scanned || status.pending_str) {
        if (!SdStorage::try_acquire()) return;
        const bool scanned = scan_pending(&selected);
        SdStorage::release();
        if (!scanned) return;
    }
    AirSenseState::Identity identity;
    if (!AirSenseState::identity(identity)) return;
    const uint32_t device = identity.generation;
    uint32_t generation = 0, saved_day = 0, after = 0;
    if (!read_u32_variable("ZEN", generation)) return;
    const bool changed = !have_synced_generation || device != synced_device_generation ||
                         generation != synced_str_generation;
    if (!changed && !status.pending_str) {
        request_summary_export();
        return;
    }
    if (!read_u32_variable("SSD", saved_day) ||
        !read_u32_variable("ZEN", after) || after != generation) return;

    bool success = true;
    if (selected.prefix[0]) {
        memcpy(pending_cursor, selected.prefix, sizeof(pending_cursor));
        if (strcmp(identity.srn, selected.srn)) {
            char error[128];
            snprintf(error, sizeof(error), "STR pending %s SRN mismatch: recorded=%s current=%s",
                     selected.prefix, selected.srn, identity.srn);
            post_error(error);
            success = false;
        } else {
            success = refresh_str_day(selected.native_day, generation, identity, true, "pending");
        }
    } else if (status.pending_str) {
        pending_cursor[0] = 0;
    }
    if (success && changed && saved_day >= 0x1000 && saved_day < 0xffff) {
        // SSD names the completed write. Revisit the preceding day only after
        // startup/restart, a day change or saves missed while UART was busy.
        const bool catch_up = !have_synced_generation || device != synced_device_generation ||
            saved_day != synced_saved_day || uint32_t(generation - synced_str_generation) != 1;
        const uint16_t days[] = {uint16_t(saved_day), uint16_t(saved_day - 1)};
        for (unsigned i = 0; i < (catch_up ? 2u : 1u); i++) {
            const uint16_t day = days[i];
            if (selected.prefix[0] && day == selected.native_day) continue;
            if (!refresh_str_day(day, generation, identity, false,
                                 i ? "previous_day" : "saved_day")) {
                success = false;
                break;
            }
        }
    }
    if (success) {
        const bool generation_read = read_u32_variable("ZEN", after);
        if (generation_read && after != generation)
            Log::logf(CAT_EDF, LOG_INFO, "STR pass superseded ZEN=%lu->%lu; retry\n",
                      (unsigned long)generation, (unsigned long)after);
        success = generation_read && after == generation &&
                  device == AirSenseState::identity_generation() &&
                  !post_processing_cancelled();
    }
    if (success) {
        synced_str_generation = generation;
        synced_device_generation = device;
        synced_saved_day = saved_day;
        have_synced_generation = true;
    }
    if (success) request_summary_export();
}

static void process_pending() {
    if (status.active || post_processing_cancelled() ||
        (next_pending_ms && int32_t(millis() - next_pending_ms) < 0)) return;
    portENTER_CRITICAL(&status_mux);
    if (clock_write_active) {
        portEXIT_CRITICAL(&status_mux);
        return;
    }
    status.post_processing = true;
    portEXIT_CRITICAL(&status_mux);
    next_pending_ms = millis() + STR_GENERATION_POLL_MS;
    sync_pending();
    portENTER_CRITICAL(&status_mux);
    status.post_processing = false;
    portEXIT_CRITICAL(&status_mux);
}

static bool prepare_storage() {
    if (!SdStorage::local_access_allowed() || (storage_ready && !SdStorage::mounted())) return false;
    if (storage_ready) return true;
    if (next_storage_ms && int32_t(millis() - next_storage_ms) < 0) return false;
    const system_state_t state = Arbiter::get_state();
    if (state != SYS_IDLE && state != SYS_THERAPY) return false;
    next_storage_ms = millis() + 5000;
    if (!SdStorage::mounted()) SdStorage::init();
    if (!SdStorage::try_acquire()) return false;
    storage = SdStorage::filesystem();
    if (storage) {
        recover_pending();
        (void)scan_pending();
        recover_partial_outputs();
        recover_identification_files();
        recover_str_file();
        EdfCatalog::init();
        EdfCatalog::Status catalog;
        EdfCatalog::get_status(catalog);
        if (catalog.ready) {
            storage_ready = reconcile_catalog();
        }
    }
    SdStorage::release();
    if (storage_ready) ExportSync::init();
    return storage_ready;
}

static void retry_recording() {
    if (status.active || !storage_ready ||
        !__atomic_load_n(&therapy_wanted, __ATOMIC_ACQUIRE) ||
        Arbiter::get_state() != SYS_THERAPY || AirSenseState::rop() != 1 ||
        (next_start_ms && int32_t(millis() - next_start_ms) < 0)) return;
    const ControlEvent event = {ControlKind::Start, millis()};
    start_session(event);
}

static void poll_recording_state() {
    const bool wanted = __atomic_load_n(&therapy_wanted, __ATOMIC_ACQUIRE);
    if (Arbiter::get_state() != SYS_THERAPY || AirSenseState::rop() != 1) {
        next_recording_state_ms = 0;
        if (wanted) request_stop();
        return;
    }
    if (next_recording_state_ms && int32_t(millis() - next_recording_state_ms) < 0)
        return;
    next_recording_state_ms = millis() + RECORDING_STATE_POLL_MS;
    if (status.active && session_phase.generation &&
        !Air10Clock::phase_unchanged(session_phase)) {
        session_phase = {};
        // Changing one channel's origin here would disagree with existing headers.
        Log::logf(CAT_EDF, LOG_WARN, "clock phase invalidated; keeping recording timeline\n");
    }
    uint32_t zle = 0;
    if (!read_u32_variable("ZLE", zle) || zle > 1) return;
    // ROP may have changed while the queued read was in flight.
    if (Arbiter::get_state() != SYS_THERAPY || AirSenseState::rop() != 1)
        return;
    if (zle && !wanted) therapy_started();
    else if (!zle && wanted) request_stop();
}

static void recorder_task(void *) {
    while (true) {
        if (!SdStorage::local_access_allowed()) {
            request_stop();
            if (status.active) {
                ControlEvent stop;
                portENTER_CRITICAL(&status_mux);
                stop = latest_stop;
                portEXIT_CRITICAL(&status_mux);
                stop_session(stop);
            }
            portENTER_CRITICAL(&status_mux);
            status.post_processing = false;
            portEXIT_CRITICAL(&status_mux);
            // No normal work or automatic remount while the host owns the card.
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        const bool have_storage = prepare_storage();
        if (have_storage) poll_recording_state();
        ControlEvent control;
        while (xQueueReceive(control_queue, &control, 0) == pdTRUE) {
            if (control.kind == ControlKind::Start) {
                if (have_storage) start_session(control);
            }
            else stop_session(control);
        }

        // The latest desired state survives a full control queue.
        if (status.active && !__atomic_load_n(&therapy_wanted, __ATOMIC_ACQUIRE)) {
            portENTER_CRITICAL(&status_mux);
            control = latest_stop;
            portEXIT_CRITICAL(&status_mux);
            stop_session(control);
        }
        if (status.active) {
            portENTER_CRITICAL(&status_mux);
            const bool restarted = recording_therapy_on_ms != therapy_on_capture_ms;
            control = latest_stop;
            portEXIT_CRITICAL(&status_mux);
            if (restarted) stop_session(control);
        }
        if (!SdStorage::local_access_allowed()) continue;
        retry_recording();

        drain_raw_frames(pdMS_TO_TICKS(20));
        if (uxQueueMessagesWaiting(raw_queue) == 0) {
            if (status.active) (void)advance_segment(millis());
            sample_pld();
            sample_oximetry();
        }
        if (status.active) update_record_status();
        else if (have_storage) process_pending();
    }
}

}  // namespace

void init() {
    if (status.ready) return;
    raw_queue_capacity = RAW_QUEUE_CAPACITY_PSRAM;
    raw_queue_storage = static_cast<uint8_t *>(aircannect::Memory::alloc_large(
        sizeof(RawFrame) * raw_queue_capacity, false));
    if (!raw_queue_storage) {
        raw_queue_capacity = RAW_QUEUE_CAPACITY_FALLBACK;
        raw_queue_storage = static_cast<uint8_t *>(aircannect::Memory::alloc_large(
            sizeof(RawFrame) * raw_queue_capacity));
    }
    if (!raw_queue_storage) {
        status_error("raw queue allocation failed");
        return;
    }

    raw_queue = xQueueCreateStatic(raw_queue_capacity, sizeof(RawFrame),
                                   raw_queue_storage, &raw_queue_state);
    control_queue = xQueueCreate(4, sizeof(ControlEvent));
    if (!raw_queue || !control_queue) {
        status_error("queue initialization failed");
        if (raw_queue) vQueueDelete(raw_queue);
        if (control_queue) vQueueDelete(control_queue);
        raw_queue = control_queue = nullptr;
        aircannect::Memory::free(raw_queue_storage);
        raw_queue_storage = nullptr;
        return;
    }
    for (LiveStream::internal_handle_t &lease : stream_leases) lease = -1;
    frame_listener = Arbiter::add_frame_listener(QFRAME_MASK_L,
                                                  frame_sink, nullptr);
    if (frame_listener < 0) {
        status_error("UART listener allocation failed");
        vQueueDelete(raw_queue); vQueueDelete(control_queue);
        raw_queue = control_queue = nullptr;
        aircannect::Memory::free(raw_queue_storage);
        raw_queue_storage = nullptr;
        return;
    }
    BaseType_t created = xTaskCreatePinnedToCoreWithCaps(
        recorder_task, "edf_rec", RECORDER_STACK, nullptr, 2,
        &recorder_task_handle, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (created != pdPASS)
        created = xTaskCreatePinnedToCore(recorder_task, "edf_rec", RECORDER_STACK,
                                         nullptr, 2, &recorder_task_handle, 1);
    if (created != pdPASS) {
        Arbiter::remove_frame_listener(frame_listener);
        frame_listener = -1;
        status_error("task creation failed");
        vQueueDelete(raw_queue); vQueueDelete(control_queue);
        raw_queue = control_queue = nullptr;
        aircannect::Memory::free(raw_queue_storage);
        raw_queue_storage = nullptr;
        return;
    }

    portENTER_CRITICAL(&status_mux);
    status.ready = true;
    portEXIT_CRITICAL(&status_mux);
    Log::logf(CAT_EDF, LOG_DEBUG,
              "recorder ready queue=%u (%s)\n",
              raw_queue_capacity,
              raw_queue_capacity == RAW_QUEUE_CAPACITY_PSRAM ? "PSRAM" : "internal");
}

void therapy_started() {
    const uint32_t captured_ms = millis();
    portENTER_CRITICAL(&status_mux);
    therapy_on_capture_ms = captured_ms;
    __atomic_store_n(&therapy_wanted, true, __ATOMIC_RELEASE);
    __atomic_store_n(&therapy_start_pending, true, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&status_mux);
    if (!status.ready || !control_queue) return;
    ControlEvent event = {ControlKind::Start, captured_ms};
    if (xQueueSend(control_queue, &event, 0) != pdTRUE) {
        status_error("control queue full at therapy start");
    }
}

void request_stop() {
    ControlEvent event = {ControlKind::Stop, millis()};
    portENTER_CRITICAL(&status_mux);
    if (!__atomic_load_n(&therapy_wanted, __ATOMIC_ACQUIRE)) {
        portEXIT_CRITICAL(&status_mux);
        return;
    }
    latest_stop = event;
    __atomic_store_n(&therapy_start_pending, false, __ATOMIC_RELEASE);
    __atomic_store_n(&therapy_wanted, false, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&status_mux);
    if (!status.ready || !control_queue) return;
    if (xQueueSend(control_queue, &event, 0) != pdTRUE)
        status_error("control queue full at therapy stop");
}

bool begin_clock_write(const char **reason) {
    const bool mounted = SdStorage::mounted();
    portENTER_CRITICAL(&status_mux);
    const char *blocked = status.active ? "EDF recording active" :
        mounted && pending_scan_error ? pending_scan_error :
        mounted && !pending_scanned ? "STR pending journal not checked" :
        status.pending_str ? "STR pending" :
        status.post_processing ? "STR collection in progress" :
        clock_write_active ? "clock write already active" : nullptr;
    if (!blocked) clock_write_active = true;
    portEXIT_CRITICAL(&status_mux);
    if (reason) *reason = blocked;
    return !blocked;
}

void end_clock_write() {
    portENTER_CRITICAL(&status_mux);
    clock_write_active = false;
    portEXIT_CRITICAL(&status_mux);
}

void get_status(Status &out) {
    portENTER_CRITICAL(&status_mux);
    out = status;
    portEXIT_CRITICAL(&status_mux);
}

}  // namespace EdfRecorder

#else

namespace EdfRecorder {

void init() {}
void therapy_started() {}
void request_stop() {}
bool begin_clock_write(const char **reason) {
    if (reason) *reason = nullptr;
    return true;
}
void end_clock_write() {}

void get_status(Status &out) {
    out = {};
    out.supported = false;
}

}  // namespace EdfRecorder

#endif
