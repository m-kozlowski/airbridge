#include "debug_log.h"
#include "memory_manager.h"
#include "build_info.h"
#include "wifi_setup.h"
#include "usb_storage.h"
#include <esp_system.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <Preferences.h>
#include "nvs_optional.h"
#include <stdarg.h>
#include <sys/time.h>
#include <time.h>
#include <errno.h>

static Preferences log_prefs;

static Print *outputs[LOG_MAX_OUTPUTS] = {};
static int output_count = 0;
// Output lifetime and configuration can wait; producers use only queue_mux.
static SemaphoreHandle_t log_mutex = nullptr;
static portMUX_TYPE queue_mux = portMUX_INITIALIZER_UNLOCKED;
static log_level_t cat_levels[CAT_COUNT];

static constexpr size_t SYSLOG_QUEUE_DEPTH = 8;
static constexpr size_t SYSLOG_SEND_BUDGET = 4;
static constexpr uint32_t SYSLOG_RETRY_MS = 100;

struct SyslogRecord {
    int64_t epoch_ms;
    uint8_t cat;
    uint8_t level;
    bool loss_report;
    char text[128];
    uint32_t queue_id;
};
static uint32_t next_queue_id = 0;

static SyslogRecord *local_queue = nullptr;
static size_t local_head = 0;
static size_t local_count = 0;

static SyslogRecord *syslog_queue = nullptr;
static size_t syslog_head = 0;
static size_t syslog_count = 0;
static sockaddr_in syslog_remote = {};
static char syslog_hostname[64] = {};
static bool boot_pending = false;
static uint32_t local_drops = 0, syslog_drops = 0, serial_drops = 0;
static uint32_t udp_errors = 0, udp_retries = 0;
static struct {
    const char *operation;
    int error;
    int64_t epoch_ms;
    uint32_t at_ms;
    in_addr ip;
    bool sta_connected;
    uint8_t ap_clients;
    uint32_t internal_free, largest;
} udp_failure = {};

const char *Log::reset_reason_name() {
    static const char *const names[] = {
        "UNKNOWN", "POWERON", "EXT", "SW", "PANIC", "INT_WDT", "TASK_WDT",
        "WDT", "DEEPSLEEP", "BROWNOUT", "SDIO", "USB", "JTAG", "EFUSE",
        "PWR_GLITCH", "CPU_LOCKUP"
    };
    unsigned reason = esp_reset_reason();
    return reason < sizeof(names) / sizeof(names[0]) ? names[reason] : "?";
}

static SyslogRecord boot_record() {
    SyslogRecord record = {};
    record.cat = CAT_GENERAL;
    record.level = LOG_INFO;
    snprintf(record.text, sizeof(record.text), "[BOOT] %s build=%s reset=%s(%u)",
             airbridge_version(), airbridge_build_date(), Log::reset_reason_name(),
             (unsigned)esp_reset_reason());
    return record;
}

void Log::init() {
    log_mutex = xSemaphoreCreateMutex();
    local_queue = static_cast<SyslogRecord *>(aircannect::Memory::alloc_large(
        SYSLOG_QUEUE_DEPTH * sizeof(SyslogRecord)));
    bool stored = open_optional_preferences(log_prefs, "log_levels");
    for (int i = 0; i < CAT_COUNT; i++) {
        log_level_t fallback = LOG_INFO;
        if (i == CAT_STORAGE || i == CAT_TIME) fallback = cat_levels[CAT_GENERAL];
        if (i == CAT_TIME && cat_levels[CAT_WIFI] < fallback) fallback = cat_levels[CAT_WIFI];
        if (i == CAT_CONFIG || i == CAT_REPORT) fallback = cat_levels[CAT_WEB];
        cat_levels[i] = stored ? (log_level_t)log_prefs.getUChar(
            Log::cat_name((log_cat_t)i), fallback) : fallback;
        if (cat_levels[i] > LOG_DEBUG) cat_levels[i] = fallback;
    }
    log_prefs.end();
}

// Count both rejected arrivals and lower-priority records evicted for them.
static unsigned enqueue(SyslogRecord *queue, size_t &head, size_t &count,
                    const SyslogRecord &record) {
    if (!queue) return !record.loss_report;
    unsigned dropped = 0;
    if (count == SYSLOG_QUEUE_DEPTH) {
        size_t victim = count;
        uint8_t lowest_priority = record.level;
        for (size_t i = 0; i < count; i++) {
            uint8_t level = queue[(head + i) % SYSLOG_QUEUE_DEPTH].level;
            if (level > lowest_priority) {
                victim = i;
                lowest_priority = level;
            }
        }
        if (victim == count) return !record.loss_report;
        dropped = !queue[(head + victim) % SYSLOG_QUEUE_DEPTH].loss_report;
        for (size_t i = victim; i + 1 < count; i++)
            queue[(head + i) % SYSLOG_QUEUE_DEPTH] =
                queue[(head + i + 1) % SYSLOG_QUEUE_DEPTH];
        count--;
    }
    SyslogRecord &slot = queue[(head + count++) % SYSLOG_QUEUE_DEPTH];
    slot = record;
    slot.queue_id = ++next_queue_id;
    return dropped;
}

void Log::boot() {
    if (!log_mutex || cat_levels[CAT_GENERAL] < LOG_INFO) return;
    SyslogRecord record = boot_record();
    xSemaphoreTake(log_mutex, portMAX_DELAY);
    portENTER_CRITICAL(&queue_mux);
    local_drops += enqueue(local_queue, local_head, local_count, record);
    boot_pending = true;
    portEXIT_CRITICAL(&queue_mux);
    xSemaphoreGive(log_mutex);
}

static size_t format_record(const SyslogRecord &record, char *line, size_t capacity) {
    return snprintf(line, capacity, "[%s][%s]%s%s",
        Log::level_name((log_level_t)record.level),
        Log::cat_name((log_cat_t)record.cat),
        record.text[0] == '[' ? "" : " ", record.text);
}

static void poll_local() {
    static char serial_pending[160];
    static size_t serial_pos = 0, serial_len = 0;
    for (size_t i = 0; i < SYSLOG_SEND_BUDGET; i++) {
        const bool serial_ready = UsbStorage::serial_available() && (bool)Serial;
        // An absent USB receiver is not a stalled sink; discard its pending tail.
        if (!serial_ready) serial_pos = serial_len = 0;
        int room = serial_ready ? Serial.availableForWrite() : 0;
        if (room > 0 && serial_pos < serial_len) {
            size_t remaining = serial_len - serial_pos;
            size_t count = remaining < (size_t)room ? remaining : (size_t)room;
            serial_pos += Serial.write(
                (const uint8_t *)serial_pending + serial_pos, count);
        }
        if (xSemaphoreTake(log_mutex, 0) != pdTRUE) return;
        portENTER_CRITICAL(&queue_mux);
        if (!local_count) {
            portEXIT_CRITICAL(&queue_mux);
            xSemaphoreGive(log_mutex);
            return;
        }
        SyslogRecord record = local_queue[local_head];
        local_head = (local_head + 1) % SYSLOG_QUEUE_DEPTH;
        local_count--;
        portEXIT_CRITICAL(&queue_mux);
        char line[160];
        size_t len = format_record(record, line, sizeof(line));
        line[len++] = '\n';
        // Registered sinks must be nonblocking. Serial has its own partial line.
        for (int j = 0; j < output_count; j++)
            outputs[j]->write((const uint8_t *)line, len);
        xSemaphoreGive(log_mutex);
        if (!serial_ready) continue;
        if (serial_pos == serial_len) {
            memcpy(serial_pending, line, len);
            serial_pos = 0;
            serial_len = len;
        } else if (!record.loss_report) serial_drops++;
    }
}

static int64_t record_time() {
    struct timeval now;
    return WiFiSetup::time_synced() && gettimeofday(&now, nullptr) == 0
        ? (int64_t)now.tv_sec * 1000 + now.tv_usec / 1000 : 0;
}

static void note_udp_failure(const char *operation, int error) {
    udp_errors++;
    udp_failure.operation = operation;
    udp_failure.error = error;
    udp_failure.at_ms = millis();
    udp_failure.epoch_ms = record_time();
    udp_failure.sta_connected = WiFi.isConnected();
    udp_failure.ap_clients = WiFi.softAPgetStationNum();
    udp_failure.ip.s_addr = (uint32_t)WiFi.localIP();
    const auto memory = aircannect::Memory::status();
    udp_failure.internal_free = memory.heap_free;
    udp_failure.largest = memory.heap_max_alloc;
}

static void report_losses() {
    static uint32_t reported_ms = 0;
    const uint32_t now = millis();
    if (now - reported_ms < 5000 || Log::get_cat_level(CAT_GENERAL) < LOG_WARN ||
        xSemaphoreTake(log_mutex, 0) != pdTRUE) return;
    portENTER_CRITICAL(&queue_mux);
    const uint32_t lost_local = local_drops, lost_syslog = syslog_drops;
    portEXIT_CRITICAL(&queue_mux);
    // Retried datagrams and an unread optional USB sink are not delivery failures.
    const log_level_t level = lost_local || lost_syslog || udp_errors
        ? LOG_WARN : LOG_DEBUG;
    // Do not recursively lose diagnostics or displace a producer's record.
    const size_t slots = udp_errors ? 2 : 1;
    if (Log::get_cat_level(CAT_GENERAL) >= level &&
        (lost_local || lost_syslog || serial_drops || udp_errors || udp_retries)) {
        SyslogRecord records[2] = {};
        SyslogRecord &record = records[0];
        record.cat = CAT_GENERAL;
        record.level = level;
        record.loss_report = true;
        record.epoch_ms = record_time();
        if (level == LOG_WARN) {
            snprintf(record.text, sizeof(record.text),
                "Log losses local=%lu syslog=%lu UDP_errors=%lu",
                (unsigned long)lost_local, (unsigned long)lost_syslog,
                (unsigned long)udp_errors);
        } else {
            snprintf(record.text, sizeof(record.text),
                "Log backpressure serial_drops=%lu UDP_retries=%lu",
                (unsigned long)serial_drops, (unsigned long)udp_retries);
        }
        if (udp_errors) {
            SyslogRecord &record = records[1];
            record = records[0];
            char ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &udp_failure.ip, ip, sizeof(ip));
            record.epoch_ms = udp_failure.epoch_ms;
            snprintf(record.text, sizeof(record.text),
                "Syslog last=%s errno=%d sta=%u ap=%u ip=%s internal=%lu largest=%lu ms=%lu",
                udp_failure.operation, udp_failure.error,
                (unsigned)udp_failure.sta_connected, (unsigned)udp_failure.ap_clients, ip,
                (unsigned long)udp_failure.internal_free,
                (unsigned long)udp_failure.largest, (unsigned long)udp_failure.at_ms);
        }
        portENTER_CRITICAL(&queue_mux);
        if (local_count + slots <= SYSLOG_QUEUE_DEPTH &&
            (!syslog_queue || syslog_count + slots <= SYSLOG_QUEUE_DEPTH)) {
            for (size_t i = 0; i < slots; ++i) {
                enqueue(local_queue, local_head, local_count, records[i]);
                if (syslog_queue) enqueue(syslog_queue, syslog_head, syslog_count, records[i]);
            }
            if (level == LOG_WARN) {
                local_drops -= lost_local;
                syslog_drops -= lost_syslog;
                udp_errors = 0;
                udp_failure = {};
            } else {
                serial_drops = udp_retries = 0;
            }
            reported_ms = now;
        }
        portEXIT_CRITICAL(&queue_mux);
    }
    xSemaphoreGive(log_mutex);
}

bool Log::configure_syslog(bool enabled, const char *host, uint16_t port,
                           const char *hostname) {
    if (!log_mutex) return false;

    sockaddr_in remote = {};
    remote.sin_family = AF_INET;
    remote.sin_port = htons(port);
    bool valid = !enabled || !*host ||
                 (port && inet_pton(AF_INET, host, &remote.sin_addr) == 1);
    enabled = enabled && *host && valid;

    char name[sizeof(syslog_hostname)];
    strlcpy(name, *hostname ? hostname : "airbridge", sizeof(name));
    for (char *p = name; *p; p++) {
        if ((uint8_t)*p < 33 || (uint8_t)*p > 126) *p = '_';
    }

    xSemaphoreTake(log_mutex, portMAX_DELAY);
    if (!enabled) {
        portENTER_CRITICAL(&queue_mux);
        SyslogRecord *retired = syslog_queue;
        syslog_queue = nullptr;
        syslog_head = syslog_count = 0;
        portEXIT_CRITICAL(&queue_mux);
        aircannect::Memory::free(retired);
    } else {
        SyslogRecord *queue = syslog_queue;
        if (!queue) {
            queue = static_cast<SyslogRecord *>(
                aircannect::Memory::alloc_large(
                    SYSLOG_QUEUE_DEPTH * sizeof(SyslogRecord)));
        }
        const bool changed = syslog_remote.sin_addr.s_addr != remote.sin_addr.s_addr ||
            syslog_remote.sin_port != remote.sin_port ||
            strcmp(syslog_hostname, name) != 0;
        portENTER_CRITICAL(&queue_mux);
        syslog_queue = queue;
        if (changed) syslog_head = syslog_count = 0;
        portEXIT_CRITICAL(&queue_mux);
        syslog_remote = remote;
        strlcpy(syslog_hostname, name, sizeof(syslog_hostname));
        valid = syslog_queue != nullptr;
    }
    xSemaphoreGive(log_mutex);
    return valid;
}

void Log::poll() {
    // Only the loop task owns this socket; producers never touch the network.
    static int fd = -1;
    static bool retry_pending = false;
    static uint32_t retry_started_ms = 0;
    if (!log_mutex) return;
    poll_local();
    report_losses();
    if (!log_mutex || xSemaphoreTake(log_mutex, 0) != pdTRUE) return;
    bool enabled = syslog_queue != nullptr;
    xSemaphoreGive(log_mutex);
    bool network_ready = enabled &&
                         (WiFi.isConnected() || WiFi.softAPgetStationNum() > 0);
    for (size_t i = 0; i < SYSLOG_SEND_BUDGET; i++) {
        if (xSemaphoreTake(log_mutex, 0) != pdTRUE) return;
        if (!syslog_queue || !network_ready) {
            xSemaphoreGive(log_mutex);
            if (fd >= 0) close(fd);
            fd = -1;
            retry_pending = false;
            return;
        }
        portENTER_CRITICAL(&queue_mux);
        if (!syslog_count && !boot_pending) {
            portEXIT_CRITICAL(&queue_mux);
            xSemaphoreGive(log_mutex);
            return;
        }
        if (retry_pending && millis() - retry_started_ms < SYSLOG_RETRY_MS) {
            portEXIT_CRITICAL(&queue_mux);
            xSemaphoreGive(log_mutex);
            return;
        }
        retry_pending = false;

        bool sending_boot = boot_pending;
        SyslogRecord record;
        if (!sending_boot) record = syslog_queue[syslog_head];
        portEXIT_CRITICAL(&queue_mux);
        sockaddr_in remote = syslog_remote;
        char hostname[sizeof(syslog_hostname)];
        memcpy(hostname, syslog_hostname, sizeof(hostname));
        xSemaphoreGive(log_mutex);
        if (sending_boot) record = boot_record();

        static const uint8_t severity[] = {3, 4, 6, 7};
        unsigned pri = 16 * 8 + (record.level <= LOG_DEBUG
                                 ? severity[record.level] : 6);
        char timestamp[25] = "-";
        if (record.epoch_ms) {
            time_t seconds = record.epoch_ms / 1000;
            struct tm utc;
            gmtime_r(&seconds, &utc);
            strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &utc);
            snprintf(timestamp + 19, sizeof(timestamp) - 19, ".%03uZ",
                     (unsigned)(record.epoch_ms % 1000));
        }
        char line[160];
        format_record(record, line, sizeof(line));
        char payload[320];
        int len = snprintf(payload, sizeof(payload),
                           "<%u>1 %s %s airbridge - %s - %s", pri, timestamp, hostname,
                           cat_name((log_cat_t)record.cat), line);

        int error = 0;
        const char *operation = "socket";
        if (fd < 0) fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (fd < 0) {
            error = errno;
        } else if (sendto(fd, payload, len, MSG_DONTWAIT,
                          (sockaddr *)&remote, sizeof(remote)) < 0) {
            error = errno;
            operation = "sendto";
        }
        if (error) {
            if (error == ENOMEM || error == ENOBUFS || error == EAGAIN) {
                // Local backpressure: retain the record and socket, not a busy loop.
                if (!record.loss_report) udp_retries++;
                retry_started_ms = millis();
                retry_pending = true;
                return;
            }
            if (!record.loss_report) note_udp_failure(operation, error);
            if (fd >= 0) close(fd);
            fd = -1;
        }

        xSemaphoreTake(log_mutex, portMAX_DELAY);
        portENTER_CRITICAL(&queue_mux);
        if (sending_boot) {
            if (!error) boot_pending = false;
        } else if (syslog_queue && syslog_count &&
                   syslog_queue[syslog_head].queue_id == record.queue_id) {
            // Enqueue eviction or reconfiguration may have replaced the head.
            syslog_head = (syslog_head + 1) % SYSLOG_QUEUE_DEPTH;
            syslog_count--;
        }
        portEXIT_CRITICAL(&queue_mux);
        xSemaphoreGive(log_mutex);
        if (error) return;
    }
}

static void save_levels() {
    log_prefs.begin("log_levels", false);
    for (int i = 0; i < CAT_COUNT; i++)
        log_prefs.putUChar(Log::cat_name((log_cat_t)i), (uint8_t)cat_levels[i]);
    log_prefs.end();
}

void Log::set_level(log_level_t lvl) {
    for (int i = 0; i < CAT_COUNT; i++)
        cat_levels[i] = lvl;
    save_levels();
}

log_level_t Log::get_level() {
    return cat_levels[CAT_GENERAL];
}

void Log::set_cat_level(log_cat_t cat, log_level_t lvl) {
    if (cat < CAT_COUNT) {
        cat_levels[cat] = lvl;
        log_prefs.begin("log_levels", false);
        log_prefs.putUChar(Log::cat_name(cat), (uint8_t)lvl);
        log_prefs.end();
    }
}

log_level_t Log::get_cat_level(log_cat_t cat) {
    return (cat < CAT_COUNT) ? cat_levels[cat] : LOG_INFO;
}

const char *Log::level_name(log_level_t lvl) {
    switch (lvl) {
        case LOG_ERROR: return "ERROR";
        case LOG_WARN:  return "WARN";
        case LOG_INFO:  return "INFO";
        case LOG_DEBUG: return "DEBUG";
        default:        return "?";
    }
}

const char *Log::cat_name(log_cat_t cat) {
    switch (cat) {
        case CAT_GENERAL: return "GENERAL";
        case CAT_OXI:     return "OXI";
        case CAT_TCP:     return "TCP";
        case CAT_WIFI:    return "WIFI";
        case CAT_OTA:     return "OTA";
        case CAT_WEB:     return "WEB";
        case CAT_ARB:     return "ARB";
        case CAT_HEALTH:  return "HEALTH";
        case CAT_EXPORT:  return "EXPORT";
        case CAT_EDF:     return "EDF";
        case CAT_STREAM:  return "STREAM";
        case CAT_STORAGE: return "STORAGE";
        case CAT_TIME:    return "TIME";
        case CAT_CONFIG:  return "CONFIG";
        case CAT_REPORT:  return "REPORT";
        default:          return "?";
    }
}

void Log::add_output(Print *out) {
    if (!log_mutex) return;
    xSemaphoreTake(log_mutex, portMAX_DELAY);
    if (output_count < LOG_MAX_OUTPUTS) {
        outputs[output_count++] = out;
    }
    xSemaphoreGive(log_mutex);
}

void Log::remove_output(Print *out) {
    if (!log_mutex) return;
    xSemaphoreTake(log_mutex, portMAX_DELAY);
    for (int i = 0; i < output_count; i++) {
        if (outputs[i] == out) {
            for (int j = i; j < output_count - 1; j++)
                outputs[j] = outputs[j + 1];
            output_count--;
            break;
        }
    }
    xSemaphoreGive(log_mutex);
}

static void log_dispatch(log_cat_t cat, log_level_t lvl,
                         const char *fmt, va_list args) {
    SyslogRecord record = {};
    record.cat = cat;
    record.level = lvl;
    record.epoch_ms = record_time();
    int len = vsnprintf(record.text, sizeof(record.text), fmt, args);
    if (len <= 0) return;
    if (len >= (int)sizeof(record.text)) {
        len = sizeof(record.text) - 1;
        memcpy(record.text + len - 3, "...", 3);
    }
    while (len > 0 && (record.text[len - 1] == '\r' || record.text[len - 1] == '\n'))
        record.text[--len] = 0;
    if (!len) return;

    // Only bounded queue copies here, never formatting, allocation or sink I/O.
    portENTER_CRITICAL(&queue_mux);
    local_drops += enqueue(local_queue, local_head, local_count, record);
    if (syslog_queue)
        syslog_drops += enqueue(syslog_queue, syslog_head, syslog_count, record);
    portEXIT_CRITICAL(&queue_mux);
}

void Log::logf(log_cat_t cat, log_level_t lvl, const char *fmt, ...) {
    if (cat < CAT_COUNT && lvl > cat_levels[cat]) return;
    va_list args;
    va_start(args, fmt);
    log_dispatch(cat, lvl, fmt, args);
    va_end(args);
}
