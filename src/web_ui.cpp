#include "web_ui.h"
#include "device_status.h"
#include "web_ui_generated.h"
#include "uart_arbiter.h"
#include "airsense_state.h"
#include "oxi_ble.h"
#include "oxi_arbiter.h"
#include "resmed_ota.h"
#include "debug_log.h"
#include "crash_diagnostics.h"
#include "app_config.h"
#include "build_info.h"
#include "wifi_setup.h"
#include "network_hints.h"
#include "live_web_consumer.h"
#include "crc.h"
#include "sd_storage.h"
#include "storage_browser.h"
#include "edf_recorder.h"
#include "edf_catalog.h"
#include "export_sync.h"
#include "airbridge_ota.h"
#include "custom_settings.h"
#include "clinical_jobs.h"
#include "memory_manager.h"
#include "json_util.h"
#include "string_util.h"
#include "hex_util.h"
#include "air10_clock.h"
#include "board.h"

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <esp_partition.h>
#include <esp_heap_caps.h>
#include <stdarg.h>
#include <time.h>
#include <new>
#include <errno.h>
#include <utility>
#include <mutex>

static AsyncWebServer *http = nullptr;
static AsyncEventSource *events = nullptr;
static AsyncEventSource *live_events = nullptr;

enum class EventTopic : uint8_t { Status, Ota, Wifi, Exports, Count };
struct EventClient {
    AsyncEventSourceClient *client = nullptr;
    uint32_t revision[static_cast<size_t>(EventTopic::Count)] = {};
};
// Four browser sessions; queues remain bounded by SSE_MAX_QUEUED_MESSAGES.
static EventClient event_clients[4];
static std::recursive_mutex event_clients_mutex;

static void registerEventClient(AsyncEventSourceClient *client) {
    std::lock_guard<std::recursive_mutex> lock(event_clients_mutex);
    for (auto &entry : event_clients) {
        if (entry.client) continue;
        entry = {};
        entry.client = client;
        return;
    }
    client->close();
}

static void unregisterEventClient(AsyncEventSourceClient *client) {
    // The library calls this before destroying its client. Never dereference
    // the pointer here; the lock protects main-loop sends from destruction.
    std::lock_guard<std::recursive_mutex> lock(event_clients_mutex);
    for (auto &entry : event_clients)
        if (entry.client == client) entry = {};
}

template <typename Build>
static void publishState(EventTopic topic, uint32_t revision,
                         const char *name, Build build) {
    const size_t index = static_cast<size_t>(topic);
    AsyncEvent_SharedData_t payload[2];
    for (auto &entry : event_clients) {
        std::lock_guard<std::recursive_mutex> lock(event_clients_mutex);
        if (!entry.client || entry.revision[index] == revision ||
            entry.client->packetsWaiting() >= SSE_MAX_QUEUED_MESSAGES) continue;
        const bool full = !entry.revision[index] || entry.revision[index] + 1 != revision;
        auto &message = payload[full];
        if (!message) {
            String json = build(full);
            if (json.isEmpty()) continue;
            char header[64];
            snprintf(header, sizeof(header), "event: %s\nid: %lu\ndata: ",
                     name, static_cast<unsigned long>(millis()));
            // Builders emit single-line JSON. Share the encoded frame across
            // client queues rather than copying it for each recipient.
            message = std::make_shared<String>();
            if (!message->reserve(strlen(header) + json.length() + 2)) {
                message.reset();
                continue;
            }
            *message += header;
            *message += json;
            *message += "\n\n";
        }
        if (entry.client->write(message))
            entry.revision[index] = revision;
    }
}

static bool checkAuth(AsyncWebServerRequest *request) {
    if (request->getResponse()) return false;
    auto &cfg = Config::get();
    if (cfg.http_user.isEmpty() && cfg.http_pass.isEmpty()) return true;
    if (!request->authenticate(cfg.http_user.c_str(), cfg.http_pass.c_str())) {
        request->requestAuthentication();
        return false;
    }
    return true;
}

// iterate json string key-value pairs
// calls fn(key, val) for each pair. Only handles string values.
typedef void (*json_kv_fn)(const char *key, const char *val, void *ctx);
static bool parseJsonObject(const String &body, JsonDocument &doc) {
    return !deserializeJson(doc, body) && doc.is<JsonObject>();
}

static bool json_foreach_kv(const String &body, json_kv_fn fn, void *ctx) {
    aircannect::JsonAllocator allocator;
    JsonDocument doc(&allocator);
    if (!parseJsonObject(body, doc)) return false;
    // Validate the whole object before a callback can mutate configuration.
    for (JsonPairConst pair : doc.as<JsonObjectConst>())
        if (!pair.value().is<const char *>()) return false;
    for (JsonPairConst pair : doc.as<JsonObjectConst>())
        fn(pair.key().c_str(), pair.value().as<const char *>(), ctx);
    return true;
}


static bool writeSetting(const char *cmd, int value) {
    uint16_t timeout = ClinicalJobs::timeout_ms();
    if (!timeout) return false;
    char req[32];
    snprintf(req, sizeof(req), "P S #%s %04X", cmd, (uint16_t)value);
    char resp[64] = {};
    uint16_t resp_len = sizeof(resp);
    return Arbiter::send_cmd(req, CMD_SRC_TCP, CMD_PRIO_NORMAL,
                              resp, &resp_len, timeout);
}

static void jsonQuote(String &out, const char *val) {
    out += '"';
    aircannect::json_escape_append(out, val, val ? strlen(val) : 0, true);
    out += '"';
}

static void jsonAddString(String &out, const char *key, const char *val, bool comma = true) {
    if (comma) out += ',';
    jsonQuote(out, key);
    out += ':';
    jsonQuote(out, val);
}

static void jsonAddInt(String &out, const char *key, int val, bool comma = true) {
    if (comma) out += ',';
    out += '"';
    out += key;
    out += "\":";
    char buf[12];
    snprintf(buf, sizeof(buf), "%d", val);
    out += buf;
}

static void jsonAddUInt32(String &out, const char *key, uint32_t val,
                          bool comma = true) {
    if (comma) out += ',';
    out += '"';
    out += key;
    out += "\":";
    char buf[12];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)val);
    out += buf;
}

static void jsonAddBool(String &out, const char *key, bool val,
                        bool comma = true) {
    if (comma) out += ',';
    out += '"';
    out += key;
    out += "\":";
    out += val ? "true" : "false";
}


static constexpr size_t JSON_BODY_MAX = 4096;
struct JsonBody {
    size_t total;
    size_t received;
    // Bytes follow this POD header; the request destructor frees _tempObject.
};

static void handleJsonBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
    if (request->getResponse()) return;
    if (!checkAuth(request)) return;
    size_t limit = request->url() == "/api/settings"
        ? ClinicalJobs::MAX_BODY_SIZE : JSON_BODY_MAX;
    if (total > limit) {
        request->send(413, "application/json", "{\"error\":\"body_too_large\"}");
        return;
    }
    if (!total || index > total || len > total - index || !len || !data) {
        request->send(400, "application/json", "{\"error\":\"invalid_body_chunk\"}");
        return;
    }

    auto *body = static_cast<JsonBody *>(request->_tempObject);
    if (!body && index == 0) {
        size_t bytes = sizeof(JsonBody) + total + 1;
        body = static_cast<JsonBody *>(aircannect::Memory::alloc_large(bytes));
        if (!body) {
            request->send(503, "application/json", "{\"error\":\"body_allocation_failed\"}");
            return;
        }
        *body = {total, 0};
        request->_tempObject = body;
    }
    if (!body || body->total != total || body->received != index ||
        memchr(data, '\0', len)) {
        request->send(400, "application/json", "{\"error\":\"invalid_body_chunk\"}");
        return;
    }

    char *bytes = reinterpret_cast<char *>(body + 1);
    memcpy(bytes + index, data, len);
    body->received += len;
    bytes[body->received] = '\0';
}

static bool getBody(AsyncWebServerRequest *request, String &out) {
    if (request->getResponse()) return false;
    auto *body = static_cast<JsonBody *>(request->_tempObject);
    if (!body || body->received != body->total) {
        request->send(400, "application/json", "{\"error\":\"incomplete_body\"}");
        return false;
    }

    bool ok = out.concat(reinterpret_cast<const char *>(body + 1), body->total);
    free(body);
    request->_tempObject = nullptr;
    if (!ok)
        request->send(503, "application/json", "{\"error\":\"body_allocation_failed\"}");
    return ok;
}

static void handleRoot(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    AsyncWebServerResponse *response = request->beginResponse(200, "text/html", HTML_PAGE_GZ, HTML_PAGE_GZ_SIZE);
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
}


struct FixedJson {
    char *buf;
    size_t cap;
    size_t len;
    bool overflow;
};

static void fixedJsonPut(FixedJson &json, char c) {
    if (json.len + 1 >= json.cap) {
        json.overflow = true;
        return;
    }
    json.buf[json.len++] = c;
    json.buf[json.len] = '\0';
}

static void fixedJsonAppend(FixedJson &json, const char *s) {
    while (*s) fixedJsonPut(json, *s++);
}

static void fixedJsonPrintf(FixedJson &json, const char *fmt, ...) {
    if (json.len >= json.cap) {
        json.overflow = true;
        return;
    }

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(json.buf + json.len, json.cap - json.len, fmt, ap);
    va_end(ap);

    if (n < 0 || (size_t)n >= json.cap - json.len) {
        json.len = json.cap ? json.cap - 1 : 0;
        if (json.cap) json.buf[json.len] = '\0';
        json.overflow = true;
        return;
    }
    json.len += (size_t)n;
}

static void fixedJsonAddString(FixedJson &json, const char *key, const char *val, bool comma = true) {
    if (comma) fixedJsonPut(json, ',');
    fixedJsonPut(json, '"');
    fixedJsonAppend(json, key);
    fixedJsonAppend(json, "\":\"");

    if (!val) val = "";
    while (*val) {
        char encoded[6];
        size_t count = aircannect::json_escape_char(*val++, encoded, true);
        for (size_t i = 0; i < count; i++) fixedJsonPut(json, encoded[i]);
    }

    fixedJsonPut(json, '"');
}

static void fixedJsonAddInt(FixedJson &json, const char *key, long val, bool comma = true) {
    if (comma) fixedJsonPut(json, ',');
    fixedJsonPut(json, '"');
    fixedJsonAppend(json, key);
    fixedJsonAppend(json, "\":");
    fixedJsonPrintf(json, "%ld", val);
}

static void jsonAddString(FixedJson &json, const char *key, const char *val,
                           bool comma = true) {
    fixedJsonAddString(json, key, val, comma);
}

static void jsonAddInt(FixedJson &json, const char *key, int val, bool comma = true) {
    fixedJsonAddInt(json, key, val, comma);
}

static void jsonAddUInt32(FixedJson &json, const char *key, uint32_t val,
                          bool comma = true) {
    fixedJsonPrintf(json, "%s\"%s\":%lu", comma ? "," : "", key, (unsigned long)val);
}

static void jsonAddBool(FixedJson &json, const char *key, bool val,
                        bool comma = true) {
    fixedJsonPrintf(json, "%s\"%s\":%s", comma ? "," : "", key, val ? "true" : "false");
}

static void statusClocks(char (&esp_time)[20], char (&resmed_time)[20]) {
    strcpy(esp_time, "--");
    time_t now = time(nullptr);
    if (now > 1700000000) {
        struct tm t;
        localtime_r(&now, &t);
        snprintf(esp_time, sizeof(esp_time), "%04d-%02d-%02d %02d:%02d",
                 t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                 t.tm_hour, t.tm_min);
    }
    Air10Clock::status_time(resmed_time);
}

enum StatusFields : uint8_t {
    STATUS_THERAPY = 1, STATUS_OXI = 2, STATUS_HEALTH = 4,
    STATUS_CONFIG = 8, STATUS_IDENTITY = 16, STATUS_REPORT = 32,
    STATUS_STORAGE = 64, STATUS_ALL = 127,
};

template<class Output>
static void appendStatusIdentity(Output &json, bool comma = true) {
    AirSenseState::Identity identity;
    AirSenseState::identity(identity);
    jsonAddString(json, "version", airbridge_version(), comma);
    jsonAddString(json, "built", airbridge_build_date());
    jsonAddString(json, "pna", identity.pna);
    jsonAddString(json, "srn", identity.srn);
}

template<class Output>
static void appendStatusFields(Output &json, const DeviceStatus::Snapshot &status,
                                uint8_t fields) {
    if (fields & STATUS_THERAPY) {
        jsonAddString(json, "system", system_state_name(status.sys));
        jsonAddInt(json, "rop", status.rop);
        jsonAddInt(json, "mhr", status.mhr);
        char mode[24];
        ClinicalSettings::mode_label(status.mop, mode, sizeof(mode));
        jsonAddString(json, "mode", mode);
    }
    if (fields & STATUS_HEALTH) {
        char esp_time[20], resmed_time[20];
        statusClocks(esp_time, resmed_time);
        jsonAddString(json, "esp_time", esp_time);
        jsonAddString(json, "resmed_time", resmed_time);
        jsonAddInt(json, "heap", ESP.getFreeHeap());
        jsonAddInt(json, "psram_free", aircannect::Memory::psram_available() ?
                   static_cast<int>(ESP.getFreePsram()) : -1);
        jsonAddString(json, "ssid", WiFiSetup::connected_ssid());
        jsonAddInt(json, "rssi", WiFiSetup::current_rssi());
        jsonAddInt(json, "uptime", millis() / 1000);
    }
    if (fields & STATUS_STORAGE) {
        const auto &sd = status.storage;
        jsonAddString(json, "sd", SdStorage::state_name(sd));
#if AB_STORAGE_HAS_SDCARD
        jsonAddInt(json, "sd_total_mb", sd.card_bytes / (1024 * 1024));
        jsonAddInt(json, "sd_used_mb", sd.used_bytes / (1024 * 1024));
        jsonAddBool(json, "sd_usb_supported", sd.usb_supported);
        jsonAddString(json, "sd_error", sd.error);
#endif
    }
    if (fields & STATUS_OXI) {
        const auto &r = status.reading;
        jsonAddString(json, "oxi", oxi_state_name(status.oxi));
        jsonAddUInt32(json, "ble_revision", OxiBle::revision());
        jsonAddString(json, "oxi_addr", status.oxi_source);
        jsonAddString(json, "oxi_name", status.oxi_name);
        jsonAddString(json, "feeding", status.feeding ? "yes" : "no");
        jsonAddInt(json, "spo2", r.valid ? r.spo2 : -1);
        jsonAddInt(json, "pulse", r.valid ? r.pulse_bpm : -1);
    }
    if (fields & STATUS_CONFIG) {
        jsonAddUInt32(json, "config_revision", Config::revision());
        jsonAddInt(json, "onboarding_complete", Config::onboarding_complete());
    }
    if (fields & STATUS_REPORT) {
        jsonAddString(json, "report_source", SleepReport::source_name(status.report_source));
        jsonAddUInt32(json, "report_revision", status.report_revision);
        jsonAddUInt32(json, "report_data_revision", status.report_data_revision);
        jsonAddString(json, "report_state", EdfReport::state_name(status.report_state));
        jsonAddString(json, "report_error", status.report_error);
        jsonAddBool(json, "report_available", status.report_available);
    }
    if (fields & STATUS_IDENTITY) appendStatusIdentity(json);
}

static size_t buildStatusJson(char *out, size_t cap) {
    if (!out || cap == 0) return 0;
    out[0] = '\0';

    const auto status = DeviceStatus::snapshot();
#if AB_STORAGE_HAS_SDCARD
    EdfRecorder::Status edf;
    EdfRecorder::get_status(edf);
    EdfCatalog::Status catalog;
    EdfCatalog::get_status(catalog);
    ExportSync::Status export_status;
    ExportSync::get_status(export_status);
    ExportSync::SleepHqStatus sleephq_status;
    ExportSync::get_sleephq_status(sleephq_status);
#endif

    FixedJson json = {out, cap, 0, false};
    fixedJsonPut(json, '{');
    appendStatusIdentity(json, false);
    appendStatusFields(json, status, STATUS_ALL & ~STATUS_IDENTITY);
#if AB_STORAGE_HAS_SDCARD
    fixedJsonAddString(json, "edf", !edf.supported ? "unsupported" :
                       edf.active ? "recording" :
                       edf.post_processing ? "post-processing" :
                       edf.ready ? "ready" : "unavailable");
    fixedJsonAddString(json, "edf_prefix", edf.file_prefix);
    fixedJsonAddInt(json, "edf_dropped", edf.raw_dropped);
    fixedJsonAddInt(json, "edf_errors", edf.write_errors);
    fixedJsonAddInt(json, "edf_post_errors", edf.post_errors);
    fixedJsonAddInt(json, "edf_str_records", edf.str_records);
    fixedJsonAddInt(json, "edf_pending_str", edf.pending_str);
    fixedJsonAddString(json, "edf_identification",
                       edf.identification_ready ? "ready" : "missing");
    fixedJsonAddInt(json, "edf_catalog_entries", catalog.entries);
    fixedJsonAddInt(json, "edf_catalog_generation", catalog.generation);
    fixedJsonAddString(json, "smb_sync",
                       ExportSync::state_name(export_status.state));
    fixedJsonAddInt(json, "smb_files_uploaded",
                    export_status.files_uploaded);
    fixedJsonAddInt(json, "smb_files_skipped",
                    export_status.files_skipped);
    fixedJsonAddString(json, "smb_error", export_status.last_error);
    fixedJsonAddString(json, "sleephq_sync",
                       ExportSync::state_name(sleephq_status.state));
    fixedJsonAddInt(json, "sleephq_files_uploaded",
                    sleephq_status.files_uploaded);
    fixedJsonAddInt(json, "sleephq_files_skipped",
                    sleephq_status.files_skipped);
    fixedJsonAddInt(json, "sleephq_import_id",
                    sleephq_status.import_id);
    fixedJsonAddString(json, "sleephq_import_status",
                       sleephq_status.import_status);
    fixedJsonAddString(json, "sleephq_error",
                       sleephq_status.last_error);
#else
    fixedJsonAddString(json, "edf", "unsupported");
    fixedJsonAddString(json, "smb_sync", "unsupported");
    fixedJsonAddString(json, "sleephq_sync", "unsupported");
#endif
    fixedJsonPut(json, '}');

    return json.overflow ? 0 : json.len;
}

static const uint32_t STATUS_CACHE_TTL_MS = 500;
// Includes bounded owner strings at the WebUI escape worst case (two bytes)
// and both uint32 report revisions.
static const size_t STATUS_JSON_MAX = AB_STORAGE_HAS_SDCARD ? 2304 : 1344;
static String status_cache;
static uint32_t status_cache_built_at = 0;

static bool refreshStatusCacheIfNeeded() {
    uint32_t now = millis();
    if (status_cache.length() > 0 && now - status_cache_built_at < STATUS_CACHE_TTL_MS)
        return true;

    char body[STATUS_JSON_MAX];
    size_t len = buildStatusJson(body, sizeof(body));
    if (len == 0) return false;

    status_cache = body;
    status_cache_built_at = millis();
    return true;
}

static void handleStatus(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    if (!refreshStatusCacheIfNeeded()) {
        request->send(503, "application/json", "{\"ok\":false,\"error\":\"status_unavailable\"}");
        return;
    }
    request->send(200, "application/json", status_cache);
}

static void handleCrashStatus(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    CrashDiagnostics::Snapshot crash;
    if (!CrashDiagnostics::snapshot(crash)) {
        request->send(503, "application/json", "{\"error\":\"busy\"}");
        return;
    }

    char body[2048] = {};
    FixedJson json{body, sizeof(body), 0, false};
    fixedJsonPut(json, '{');
    jsonAddString(json, "state", CrashDiagnostics::state_name(crash.state), false);
    jsonAddUInt32(json, "size", crash.size);
    jsonAddUInt32(json, "stored_size", crash.stored_size);
    jsonAddString(json, "error", crash.error == ESP_OK ? "" : esp_err_to_name(crash.error));
    fixedJsonPrintf(json, ",\"summary_available\":%s",
                    crash.summary_available ? "true" : "false");
    if (crash.summary_available) {
        jsonAddString(json, "task", crash.task);
        jsonAddString(json, "reason", crash.reason);
        jsonAddString(json, "elf_sha", crash.elf_sha);
        fixedJsonPrintf(json, ",\"pc\":\"0x%08lx\"", (unsigned long)crash.pc);
        jsonAddUInt32(json, "cause", crash.cause);
        fixedJsonPrintf(json, ",\"exception_address\":\"0x%08lx\"",
                        (unsigned long)crash.exception_address);
        fixedJsonPrintf(json, ",\"backtrace_corrupt\":%s,\"backtrace\":[",
                        crash.backtrace_corrupt ? "true" : "false");
        for (size_t i = 0; i < crash.backtrace_depth; ++i)
            fixedJsonPrintf(json, "%s\"0x%08lx\"", i ? "," : "",
                            (unsigned long)crash.backtrace[i]);
        fixedJsonPut(json, ']');
    }
    fixedJsonPut(json, '}');
    if (json.overflow) {
        request->send(503, "application/json", "{\"error\":\"response_overflow\"}");
        return;
    }

    auto *response = request->beginResponse(200, "application/json", body);
    if (!response) {
        request->send(503, "application/json", "{\"error\":\"allocation_failed\"}");
        return;
    }
    response->addHeader("Cache-Control", "no-store");
    request->send(response);
}




static int saveSettings(const String &body, String &json) {
    if (!CustomSettings::ensure_loaded()) {
        json = "{\"error\":\"settings_metadata_unavailable\"}";
        return 503;
    }

    struct { int count; String errors; bool lan_changed; } ctx = {0, "", false};
    bool parsed = json_foreach_kv(body, [](const char *key, const char *val, void *p) {
        auto *c = (decltype(ctx)*)p;
        char *end = nullptr;
        errno = 0;
        int64_t raw = strtoll(val, &end, 10);
        if (end == val || *end != '\0' || errno == ERANGE ||
            raw < INT32_MIN || raw > UINT32_MAX) {
            c->errors += key;
            c->errors += ":invalid_number,";
            return;
        }
        if (CustomSettings::contains(key)) {
            if (CustomSettings::write_raw(key, (uint32_t)raw)) {
                c->count++;
            } else {
                c->errors += key;
                c->errors += ":fail,";
                Log::logf(CAT_CONFIG, LOG_DEBUG, "AirSense: Custom write #%.3s failed\n", key);
            }
            return;
        }
        if (!ClinicalSettings::known_stock(key)) {
            c->errors += key;
            c->errors += ":unknown,";
            return;
        }
        if (raw < INT16_MIN || raw > UINT16_MAX) {
            c->errors += key;
            c->errors += ":invalid_number,";
            return;
        }
        if (writeSetting(key, (int)raw)) {
            c->count++;
            if (strcmp(key, "LAN") == 0) c->lan_changed = true;
        } else {
            c->errors += key;
            c->errors += ":fail,";
            Log::logf(CAT_CONFIG, LOG_DEBUG, "AirSense: Stock write #%.3s failed\n", key);
        }
    }, &ctx);
    if (!parsed) {
        json = "{\"error\":\"bad_json\"}";
        return 400;
    }
    int count = ctx.count;
    String errors = ctx.errors;
    if (ctx.lan_changed) CustomSettings::invalidate("LAN write");
    Log::logf(CAT_CONFIG, errors.length() ? LOG_WARN : LOG_INFO,
              "AirSense: Save applied=%d%s%.64s\n", count,
              errors.length() ? " failed=" : "", errors.c_str());

    json = "{";
    jsonAddInt(json, "saved", count, false);
    if (errors.length() > 0) {
        errors.remove(errors.length() - 1);  // trailing comma
        json += ",\"errors\":[";
        jsonQuote(json, errors.c_str());
        json += ']';
    }
    json += '}';
    return 200;
}


class BufferedResponse : public AsyncWebServerResponse {
public:
    void begin(int code, size_t length, const char *type = "application/json") {
        _code = code;
        _contentType = type;
        _contentLength = length;
    }

    void _respond(AsyncWebServerRequest *request) override {
        addHeader("Connection", "close", false);
        _assembleHead(headers_, request->version());
        _state = RESPONSE_HEADERS;
        _ack(request, 0, 0);
    }

    size_t _ack(AsyncWebServerRequest *request, size_t len, uint32_t) override {
        _ackedLength += len;
        size_t written = 0;
        if (_state == RESPONSE_HEADERS) {
            size_t count = request->client()->add(headers_.c_str() + header_offset_,
                                                   headers_.length() - header_offset_);
            header_offset_ += count;
            written += count;
            if (header_offset_ == headers_.length()) _state = RESPONSE_CONTENT;
        }
        if (_state == RESPONSE_CONTENT) {
            while (_sentLength < _contentLength && request->client()->space()) {
                if (buffer_offset_ == buffer_length_) {
                    buffer_length_ = readBody(_sentLength, buffer_, sizeof(buffer_));
                    buffer_offset_ = 0;
                    if (!buffer_length_) {
                        _state = RESPONSE_FAILED;
                        request->client()->close();
                        return written;
                    }
                }
                // Keep generated bytes until TCP has accepted them, even after a zero add.
                size_t count = request->client()->add(buffer_ + buffer_offset_,
                                                       buffer_length_ - buffer_offset_);
                buffer_offset_ += count;
                _sentLength += count;
                written += count;
                if (buffer_offset_ != buffer_length_) break;
            }
            if (_sentLength == _contentLength) _state = RESPONSE_WAIT_ACK;
        }
        _writtenLength += written;
        if (_writtenLength > _ackedLength) request->client()->send();
        if (_state == RESPONSE_WAIT_ACK && _ackedLength >= _writtenLength)
            _state = RESPONSE_END;
        return written;
    }

protected:
    virtual size_t readBody(size_t offset, char *out, size_t capacity) = 0;

private:
    String headers_;
    size_t header_offset_ = 0;
    char buffer_[256];
    size_t buffer_offset_ = 0, buffer_length_ = 0;
};

class CrashResponse : public BufferedResponse {
public:
    explicit CrashResponse(std::unique_ptr<CrashDiagnostics::Dump> dump)
        : dump_(std::move(dump)) {
        begin(200, dump_->size(), "application/octet-stream");
    }

    bool _sourceValid() const override { return bool(dump_); }

protected:
    size_t readBody(size_t offset, char *out, size_t capacity) override {
        return dump_->read(offset, out, capacity);
    }

private:
    std::unique_ptr<CrashDiagnostics::Dump> dump_;
};

static void handleCrashDump(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    const char *error = nullptr;
    auto dump = CrashDiagnostics::open_dump(error);
    if (!dump) {
        int code = 503;
        if (!strcmp(error, "empty") || !strcmp(error, "unsupported")) code = 404;
        if (!strcmp(error, "invalid") || !strcmp(error, "download_active")) code = 409;
        String body = "{";
        jsonAddString(body, "error", error, false);
        body += '}';
        request->send(code, "application/json", body);
        return;
    }

    auto *response = new (std::nothrow) CrashResponse(std::move(dump));
    if (!response) {
        request->send(503, "application/json", "{\"error\":\"allocation_failed\"}");
        return;
    }
    response->addHeader("Content-Disposition", "attachment; filename=\"airbridge-coredump.bin\"");
    response->addHeader("Cache-Control", "no-store");
    response->addHeader("Accept-Ranges", "none");
    request->send(response);
}

class ClinicalResponse : public BufferedResponse {
public:
    explicit ClinicalResponse(ClinicalJobs::Result &&ready) : result(std::move(ready)) {}
    ClinicalJobs::Result result;

    void begin(int code) { BufferedResponse::begin(code, result.length()); }
    bool _sourceValid() const override { return result.available(); }

protected:
    size_t readBody(size_t offset, char *out, size_t capacity) override {
        return result.read(cursor_, offset, out, capacity);
    }

private:
    ClinicalJobs::Cursor cursor_;
};

static void sendReportJob(AsyncWebServerRequest *request, SleepReport::Source source,
                           uint32_t id, bool pending = false) {
    String body = "{";
    const String token = String(SleepReport::source_name(source)) + ':' + String(id);
    jsonAddString(body, "job", token.c_str(), false);
    if (pending) jsonAddBool(body, "pending", true);
    body += '}';
    request->send(202, "application/json", body);
}

static void handleClinicalJob(AsyncWebServerRequest *request, ClinicalJobs::Kind kind,
                              SleepReport::Request report = {}, uint32_t job = 0) {
    if (!checkAuth(request)) return;
    bool write = kind == ClinicalJobs::Kind::Write;
    if (!write && (job || request->hasArg("job"))) {
        ClinicalJobs::Result result;
        int code = ClinicalJobs::poll(job ? job :
            strtoul(request->arg("job").c_str(), nullptr, 10), result);
        if (result.available()) {
            auto *response = new (std::nothrow) ClinicalResponse(std::move(result));
            if (!response) {
                request->send(503, "application/json", "{\"error\":\"settings_allocation_failed\"}");
                return;
            }
            response->begin(code);
            request->send(response);
        } else if (kind == ClinicalJobs::Kind::Report && code == 202) {
            sendReportJob(request, SleepReport::Source::Device, job, true);
        } else {
            request->send(code, "application/json", code == 202 ? "{\"pending\":true}"
                : "{\"error\":\"settings_job_unavailable\"}");
        }
        return;
    }
    uint32_t id = 0;
    String body;
    if (write && !getBody(request, body)) return;
    if (!ClinicalJobs::submit(kind, std::move(body), id, report)) {
        request->send(503, "application/json", "{\"error\":\"settings_busy\"}");
        return;
    }
    if (kind == ClinicalJobs::Kind::Report)
        sendReportJob(request, SleepReport::Source::Device, id);
    else request->send(202, "application/json", "{\"job\":" + String(id) + "}");
}

static void handleGetSettings(AsyncWebServerRequest *request) { handleClinicalJob(request, ClinicalJobs::Kind::Read); }
static void handlePostSettings(AsyncWebServerRequest *request) { handleClinicalJob(request, ClinicalJobs::Kind::Write); }

class ReportResponse : public BufferedResponse {
public:
    ReportResponse(int code, SleepReport::Result &&ready) : result_(std::move(ready)) {
        begin(code, result_.length());
    }

    bool _sourceValid() const override { return result_.available(); }

protected:
    size_t readBody(size_t offset, char *out, size_t capacity) override {
        return result_.read(offset, out, capacity);
    }

private:
    SleepReport::Result result_;
};

static bool reportQueryKeys(AsyncWebServerRequest *request) {
    const char *keys[] = {"source", "action", "view", "day", "period", "job",
                          "from_ms", "to_ms", "px", "revision", "exclude"};
    const size_t count = request->params();
    if (count > sizeof(keys) / sizeof(keys[0])) return false;
    for (size_t i = 0; i < count; i++) {
        const auto *param = request->getParam(i);
        if (!param || param->isPost() || param->isFile()) return false;
        bool known = false;
        for (const char *key : keys) known |= param->name() == key;
        if (!known) return false;
        for (size_t j = 0; j < i; j++)
            if (request->getParam(j)->name() == param->name()) return false;
    }
    return true;
}

static bool reportNumber(AsyncWebServerRequest *request, const char *key,
                          uint64_t maximum, uint64_t &value) {
    const auto *param = request->getParam(key);
    if (!param) return true;
    uint64_t parsed;
    if (param->value().length() > 20 ||
        !aircannect::parse_uint64_decimal(param->value().c_str(), parsed) ||
        parsed > maximum) return false;
    value = parsed;
    return true;
}

static bool reportExclusions(AsyncWebServerRequest *request, SleepReport::LocalRequest &local) {
    const auto *param = request->getParam("exclude");
    if (!param) return true;
#if AB_STORAGE_HAS_SDCARD
    const auto &text = param->value();
    size_t decoded = 0;
    return aircannect::hex_decode(text.c_str(), text.length(), local.excluded,
                                  sizeof(local.excluded), decoded);
#else
    (void)local;
    return false;
#endif
}

static bool reportJob(AsyncWebServerRequest *request, SleepReport::Source &source,
                       uint32_t &id, bool &qualified) {
    const auto *param = request->getParam("job");
    if (!param || param->value().length() > 17) return false;
    const char *text = param->value().c_str();
    auto tagged = source;
    qualified = true;
    if (!strncmp(text, "device:", 7)) {
        tagged = SleepReport::Source::Device;
        text += 7;
    } else if (!strncmp(text, "edf:", 4)) {
        tagged = SleepReport::Source::Edf;
        text += 4;
    } else qualified = false;
    if ((qualified && request->hasParam("source") && tagged != source) ||
        !aircannect::parse_uint32_decimal(text, id) || !id) return false;
    source = tagged;
    return true;
}

static void handleReport(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    auto invalid = [&]() {
        request->send(400, "application/json", "{\"error\":\"report_invalid_selection\"}");
    };
    if (!reportQueryKeys(request)) { invalid(); return; }

    SleepReport::Source source = SleepReport::default_source();
    if (const auto *param = request->getParam("source")) {
        if (param->value() == "edf") source = SleepReport::Source::Edf;
        else if (param->value() == "device") source = SleepReport::Source::Device;
        else { invalid(); return; }
    }

    if (request->hasParam("job")) {
        uint32_t id = 0;
        bool qualified = false;
        if (!reportJob(request, source, id, qualified) ||
            request->params() != (request->hasParam("source") ? 2u : 1u)) {
            invalid(); return;
        }
        if (request->method() == HTTP_DELETE) {
            if (source == SleepReport::Source::Device) {
                request->send(405, "application/json", "{\"error\":\"report_device_cancel_unsupported\"}");
            } else if (!qualified) invalid();
            else {
                SleepReport::cancel(id);
                request->send(204);
            }
            return;
        }
        if (request->method() != HTTP_GET) {
            request->send(405, "application/json", "{\"error\":\"report_job_requires_GET\"}");
            return;
        }
        if (source == SleepReport::Source::Device) {
            handleClinicalJob(request, ClinicalJobs::Kind::Report, {}, id);
            return;
        }
        SleepReport::Result result;
        int code = SleepReport::poll(id, result);
        if (result.available()) {
            auto *response = new (std::nothrow) ReportResponse(code, std::move(result));
            if (!response) {
                request->send(503, "application/json", "{\"error\":\"report_allocation_failed\"}");
                return;
            }
            request->send(response);
        } else if (code == 202) {
            sendReportJob(request, source, id, true);
        } else {
            request->send(code, "application/json", "{\"error\":\"report_job_unavailable\"}");
        }
        return;
    }

    if (request->method() == HTTP_DELETE) { invalid(); return; }

    SleepReport::LocalRequest local;
    if (const auto *param = request->getParam("action")) {
        if (param->value() == "days") local.action = EdfReport::Action::Days;
        else if (param->value() == "series") local.action = EdfReport::Action::Series;
        else if (param->value() != "summary") { invalid(); return; }
    }
    if (const auto *param = request->getParam("view")) {
        if (param->value() == "period") local.view = EdfReport::View::Period;
        else if (param->value() != "day") { invalid(); return; }
    }
    const bool period_view = local.view == EdfReport::View::Period;
    const bool series = local.action == EdfReport::Action::Series;
    uint64_t day = 0, period = 0, from = 0, to = 0, px = local.px, revision = 0;
    if (!reportNumber(request, "day", UINT16_MAX, day) ||
        !reportNumber(request, "period", 5, period) ||
        !reportNumber(request, "from_ms", INT64_MAX, from) ||
        !reportNumber(request, "to_ms", INT64_MAX, to) ||
        !reportNumber(request, "px", 1600, px) ||
        !reportNumber(request, "revision", UINT32_MAX, revision) ||
        (!period_view && request->hasParam("period")) ||
        (!series && (request->hasParam("from_ms") || request->hasParam("to_ms") ||
                     request->hasParam("px") || request->hasParam("exclude")))) {
        invalid(); return;
    }
    local.day = static_cast<uint16_t>(day);
    local.period = SleepReport::period_days(static_cast<uint16_t>(period));
    local.from_ms = static_cast<int64_t>(from);
    local.to_ms = static_cast<int64_t>(to);
    local.px = static_cast<uint16_t>(px);
    local.revision = static_cast<uint32_t>(revision);
    if (!reportExclusions(request, local)) { invalid(); return; }

    if (source == SleepReport::Source::Device) {
        if (local.action != EdfReport::Action::Summary || request->hasParam("revision") ||
            (period_view && request->hasParam("day"))) { invalid(); return; }
        SleepReport::Request device;
        device.view = period_view ? SleepReport::View::Period : SleepReport::View::Day;
        device.selection = static_cast<uint16_t>(period_view ? period : day);
        if (!SleepReport::valid(device)) { invalid(); return; }
        if (request->method() != (period_view ? HTTP_POST : HTTP_GET)) {
            request->send(405, "application/json", period_view
                ? "{\"error\":\"report_period_requires_POST\"}"
                : "{\"error\":\"report_day_requires_GET\"}");
            return;
        }
        handleClinicalJob(request, ClinicalJobs::Kind::Report, device);
        return;
    }

    if (request->method() != HTTP_GET) {
        request->send(405, "application/json", "{\"error\":\"report_edf_requires_GET\"}");
        return;
    }
    if (!EdfReport::valid(local)) { invalid(); return; }
    uint32_t id = 0;
    const char *error = nullptr;
    if (!SleepReport::submit(local, id, &error)) {
        String body = "{";
        jsonAddString(body, "error", error ? error : "report_busy", false);
        body += '}';
        const int code = error && (!strcmp(error, "stale_revision") || !strcmp(error, "not_idle"))
            ? 409 : 503;
        request->send(code, "application/json", body);
        return;
    }
    sendReportJob(request, source, id);
}


static void handleGetConfig(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    Config::Section section = Config::Section::All;
    if (request->hasParam("section") &&
        !Config::parse_section(request->getParam("section")->value().c_str(), section)) {
        request->send(400, "application/json", "{\"error\":\"unknown_section\"}");
        return;
    }

    String json = "{";
    struct { String *json; bool first; } ctx = {&json, true};
    Config::foreach_kv([](const char *key, const String &val, bool sensitive, void *p) {
        auto *c = (decltype(ctx)*)p;
        if (!c->first) *c->json += ',';
        c->first = false;
        jsonAddString(*c->json, key, sensitive ? "" : val.c_str(), false);
    }, &ctx, section);
    json += '}';
    request->send(200, "application/json", json);
}


static void handlePostConfig(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String body;
    if (!getBody(request, body)) return;
    String previous_update_url = Config::get().update_url;
    const auto &config = Config::get();
    const String hostname = config.hostname, country = config.wifi_country;
    const String timezone = config.tz, ntp = config.ntp_server;
    const uint8_t mode = config.wifi_mode;
    struct Changes { int count = 0; } changes;
    if (!json_foreach_kv(body, [](const char *key, const char *val, void *p) {
        auto &changes = *static_cast<Changes *>(p);
        if (!Config::set_value(key, val)) return;
        changes.count++;
    }, &changes)) {
        request->send(400, "application/json", "{\"error\":\"bad_json\"}");
        return;
    }

    const bool saved = !changes.count || Config::save();
    WiFiSetup::request_reconfigure(hostname != config.hostname || country != config.wifi_country ||
        mode != config.wifi_mode, timezone != config.tz || ntp != config.ntp_server);
    if (Config::get().update_url != previous_update_url)
        OtaManager::config_changed();
    if (!saved) {
        request->send(500, "application/json", "{\"ok\":false,\"error\":\"nvs_save_failed\"}");
        return;
    }

    String json = "{\"ok\":true,\"saved\":";
    json += String(changes.count);
    json += '}';
    request->send(200, "application/json", json);
}


enum ExportFields : uint8_t {
    EXPORT_RUN = 1, EXPORT_CHECK = 2, EXPORT_BACKLOG = 4,
    EXPORT_META = 8, EXPORT_ACTIONS = 16, EXPORT_ALL = 31,
};

#if AB_STORAGE_HAS_SDCARD
template <typename T>
static void appendExportStatus(String &json, const T &status, bool enabled,
                               bool configured, bool automatic, const char *endpoint,
                               bool smb, bool full, uint8_t fields = EXPORT_ALL) {
    json += '{';
    jsonAddString(json, "state", ExportSync::state_name(status.state), false);
    if (full) {
        jsonAddBool(json, "configured", configured);
        jsonAddBool(json, "enabled", enabled);
        jsonAddBool(json, "automatic", automatic);
        jsonAddUInt32(json, "files_seen", status.files_seen);
        jsonAddString(json, "current_day", status.current_day);
        jsonAddBool(json, "backlog_known", status.backlog.known);
        jsonAddUInt32(json, "backlog_sessions", status.backlog.sessions);
        char bytes[48];
        snprintf(bytes, sizeof(bytes), ",\"bytes_uploaded\":%llu",
                 static_cast<unsigned long long>(status.bytes_uploaded));
        json += bytes;
    }
    if ((fields & EXPORT_META) && (smb || full)) jsonAddString(json, "endpoint", endpoint);
    if (fields & EXPORT_ACTIONS) {
        jsonAddString(json, "sync_blocked", ExportSync::action_blocked(smb, false));
        jsonAddString(json, "check_blocked", ExportSync::action_blocked(smb, true));
    }
    if (fields & EXPORT_RUN) {
        jsonAddBool(json, "has_files", status.has_files);
        jsonAddUInt32(json, "files_uploaded", status.files_uploaded);
        jsonAddUInt32(json, "files_skipped", status.files_skipped);
        jsonAddUInt32(json, "last_sync_epoch", status.last_sync_epoch);
        jsonAddString(json, "error", status.last_error);
    }
    if (fields & EXPORT_CHECK) {
        jsonAddString(json, "check_state", ExportSync::check_state_name(status.check.state));
        jsonAddString(json, "check_error", status.check.error);
    }
    if (fields & EXPORT_BACKLOG) {
        jsonAddString(json, "backlog_state", ExportSync::backlog_state(status.backlog, smb));
        jsonAddUInt32(json, "backlog_files", status.backlog.files);
        jsonAddUInt32(json, "backlog_waiting", status.backlog.waiting_sessions);
        jsonAddString(json, "backlog_error", status.backlog.error);
    }
}

struct ExportPublication {
    ExportSync::PublicationStamp stamp = {};
    bool observed = false;
    ExportSync::Status smb = {};
    ExportSync::SleepHqStatus shq = {};
    String endpoint, team, device;
    const char *blocked[4] = {};
    const char *backlog[2] = {};
    uint32_t revision = 1;
    uint8_t fields[2] = {EXPORT_ALL, EXPORT_ALL};
};

template <typename T>
static uint8_t exportChanges(const T &a, const T &b) {
    uint8_t fields = 0;
    if (a.state != b.state || a.has_files != b.has_files ||
        a.files_uploaded != b.files_uploaded || a.files_skipped != b.files_skipped ||
        a.last_sync_epoch != b.last_sync_epoch ||
        strcmp(a.last_error, b.last_error)) fields |= EXPORT_RUN;
    if (a.check.state != b.check.state || strcmp(a.check.error, b.check.error))
        fields |= EXPORT_CHECK;
    if (a.backlog.files != b.backlog.files ||
        a.backlog.waiting_sessions != b.backlog.waiting_sessions ||
        strcmp(a.backlog.error, b.backlog.error)) fields |= EXPORT_BACKLOG;
    return fields;
}
#endif

static String buildExportsJson(bool full = true, uint8_t smb_fields = EXPORT_ALL,
                               uint8_t shq_fields = EXPORT_ALL) {
#if AB_STORAGE_HAS_SDCARD
    ExportSync::Status smb;
    ExportSync::SleepHqStatus sleephq;
    ExportSync::get_status(smb);
    ExportSync::get_sleephq_status(sleephq);
    const auto &cfg = Config::get();
    String json = full || (smb_fields == EXPORT_ALL && shq_fields == EXPORT_ALL)
        ? "{\"supported\":true" : "{";
    if (full) {
        jsonAddBool(json, "sd_mounted", SdStorage::mounted());
        jsonAddBool(json, "idle", AirSenseState::device_standby());
        jsonAddBool(json, "online", WiFi.status() == WL_CONNECTED);
        jsonAddUInt32(json, "config_revision", Config::revision());
    }
    if (smb_fields) {
        if (json.length() > 1) json += ',';
        json += "\"smb\":";
        appendExportStatus(json, smb, cfg.smb_enabled, !cfg.smb_endpoint.isEmpty(),
                           cfg.smb_auto_after_therapy, cfg.smb_endpoint.c_str(), true, full, smb_fields);
        json += '}';
    }
    if (shq_fields) {
        if (json.length() > 1) json += ',';
        json += "\"sleephq\":";
        appendExportStatus(json, sleephq, cfg.sleephq_enabled,
                           !cfg.sleephq_client_id.isEmpty() && !cfg.sleephq_client_secret.isEmpty(),
                           cfg.sleephq_auto_after_therapy, "SleepHQ", false, full, shq_fields);
        if (shq_fields & EXPORT_META) {
            jsonAddString(json, "team_id", cfg.sleephq_team_id.c_str());
            jsonAddString(json, "device_id", cfg.sleephq_device_id.c_str());
        }
        if (shq_fields & EXPORT_RUN) {
            jsonAddUInt32(json, "import_id", sleephq.import_id);
            jsonAddString(json, "import_status", sleephq.import_status);
        }
        json += '}';
    }
    json += '}';
    return json;
#else
    return "{\"supported\":false}";
#endif
}

static void handleExportStatus(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    auto *response = request->beginResponse(200, "application/json", buildExportsJson());
    response->addHeader("Cache-Control", "no-store");
    request->send(response);
}

#if AB_STORAGE_HAS_SDCARD
static void publishExports() {
    static ExportPublication *last = nullptr;
    if (!last) {
        void *memory = aircannect::Memory::alloc_large(sizeof(ExportPublication));
        if (!memory) return;
        last = new (memory) ExportPublication;
    }
    const auto stamp = ExportSync::publication_stamp();
    if (!last->observed || !(stamp == last->stamp)) {
        ExportSync::Status smb;
        ExportSync::SleepHqStatus shq;
        ExportSync::get_status(smb);
        ExportSync::get_sleephq_status(shq);
        uint8_t fields[] = {exportChanges(smb, last->smb), exportChanges(shq, last->shq)};
        if (shq.import_id != last->shq.import_id || strcmp(shq.import_status, last->shq.import_status))
            fields[1] |= EXPORT_RUN;
        const auto &cfg = Config::get();
        if (last->endpoint != cfg.smb_endpoint) {
            last->endpoint = cfg.smb_endpoint;
            fields[0] |= EXPORT_META;
        }
        if (last->team != cfg.sleephq_team_id || last->device != cfg.sleephq_device_id) {
            last->team = cfg.sleephq_team_id;
            last->device = cfg.sleephq_device_id;
            fields[1] |= EXPORT_META;
        }
        for (uint8_t i = 0; i < 4; ++i) {
            const char *reason = ExportSync::action_blocked(i < 2, i % 2);
            if (reason == last->blocked[i]) continue;
            last->blocked[i] = reason;
            fields[i / 2] |= EXPORT_ACTIONS;
        }
        const char *backlog[] = {ExportSync::backlog_state(smb.backlog, true),
                                 ExportSync::backlog_state(shq.backlog, false)};
        for (uint8_t i = 0; i < 2; ++i) {
            if (backlog[i] == last->backlog[i]) continue;
            last->backlog[i] = backlog[i];
            fields[i] |= EXPORT_BACKLOG;
        }
        if (fields[0] || fields[1]) {
            last->smb = smb;
            last->shq = shq;
            memcpy(last->fields, fields, sizeof(fields));
            if (!++last->revision) ++last->revision;
        }
        last->stamp = stamp;
        last->observed = true;
    }
    publishState(EventTopic::Exports, last->revision, "exports", [&](bool full) {
        return buildExportsJson(false, full ? EXPORT_ALL : last->fields[0],
                                full ? EXPORT_ALL : last->fields[1]);
    });
}
#endif

static void handleExportRequest(AsyncWebServerRequest *request, bool smb) {
    if (!checkAuth(request)) return;
    const String action = request->hasParam("action") ? request->getParam("action")->value() : "sync";
    if (action != "sync" && action != "check") {
        request->send(400, "application/json", "{\"ok\":false,\"error\":\"invalid_action\"}");
        return;
    }
    const char *error = nullptr;
    const bool queued = smb ? ExportSync::request_manual_smb(action == "check", &error)
                            : ExportSync::request_manual_sleephq(action == "check", &error);
    if (queued) {
        request->send(202, "application/json", "{\"ok\":true,\"state\":\"pending\"}");
    } else {
        String response = "{\"ok\":false";
        jsonAddString(response, "error", error);
        response += '}';
        request->send(409, "application/json", response);
    }
}


static const esp_partition_t *resmed_part = nullptr;
size_t uploadSize = 0;
static bool uploadOk = false;
static uint16_t uploadCrc = 0xFFFF;

typedef enum {
    UPLOAD_NONE,
    UPLOAD_RESMED,
    UPLOAD_ESP,
} upload_kind_t;

static upload_kind_t uploadKind = UPLOAD_NONE;
static AsyncWebServerRequest *uploadOwner = nullptr;
static size_t uploadNextIndex = 0;
static bool uploadComplete = false;
static bool uploadOwnsUart = false;
static constexpr size_t UPLOAD_ERASE_BLOCK = 64 * 1024;
static size_t uploadErased = 0;

static void finishUpload(AsyncWebServerRequest *request, bool success,
                         const char *error) {
    if (uploadOwner != request) return;
    if (uploadKind == UPLOAD_ESP) OtaManager::abort_image();
    if (!success) {
        uploadOk = false;
        uploadComplete = false;
        uploadKind = UPLOAD_NONE;
        resmed_part = nullptr;
    }
    if (uploadOwnsUart && Arbiter::get_state() == SYS_OTA_ESP)
        Arbiter::set_state(SYS_IDLE);
    uploadOwnsUart = false;
    uploadOwner = nullptr;
    OtaManager::end_manual_upload(success, error);
}

static bool claimUpload(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return false;
    if (uploadOwner) {
        // More than one file in the owning request is not a firmware image.
        if (uploadOwner == request) uploadOk = false;
        return false;
    }
    if (!OtaManager::begin_manual_upload()) return false;
    uploadOwner = request;
    uploadNextIndex = 0;
    uploadErased = 0;
    uploadComplete = false;
    uploadOwnsUart = false;
    request->onDisconnect([request]() {
        finishUpload(request, false, "upload_disconnected");
    });
    return true;
}

static bool acceptUploadChunk(AsyncWebServerRequest *request, size_t index,
                               size_t len, bool final) {
    if (uploadOwner != request) return false;
    if (uploadComplete || index != uploadNextIndex || len > SIZE_MAX - index) {
        uploadOk = false;
        return false;
    }
    uploadNextIndex += len;
    uploadComplete = final;
    return true;
}

static bool ownsUpload(AsyncWebServerRequest *request) {
    if (uploadOwner == request) return true;
    request->send(409, "application/json",
                  "{\"ok\":false,\"error\":\"upload_not_owned\"}");
    return false;
}

static bool hasValidResmedUpload() {
    return !uploadOwner && uploadComplete && uploadKind == UPLOAD_RESMED &&
           uploadOk && uploadSize > 0 && resmed_part;
}

static bool eraseUploadThrough(const esp_partition_t *part, size_t end) {
    // ResMed staging checks capacity before erasing ahead of its next write.
    while (uploadErased < end) {
        const size_t size = min(UPLOAD_ERASE_BLOCK, size_t(part->size - uploadErased));
        esp_err_t err = esp_partition_erase_range(part, uploadErased, size);
        if (err != ESP_OK) {
            Log::logf(CAT_OTA, LOG_ERROR, "Upload erase failed in '%s' at %u: %s\n",
                      part->label, uploadErased, esp_err_to_name(err));
            return false;
        }
        uploadErased += size;
    }
    return true;
}

static void handleUploadChunk(AsyncWebServerRequest *request, const String& filename,
                               size_t index, uint8_t *data, size_t len, bool final) {
    if (index == 0) {
        if (!claimUpload(request)) return;
        Log::logf(CAT_OTA, LOG_INFO, "ResMed upload started: %s\n", filename.c_str());
        uploadKind = UPLOAD_RESMED;
        uploadSize = 0;
        uploadOk = false;
        uploadCrc = 0xFFFF;

        resmed_part = ResmedOta::get_staging_partition();
        if (!resmed_part) {
            Log::logf(CAT_OTA, LOG_ERROR, "No staging partition found\n");
            uploadKind = UPLOAD_NONE;
            finishUpload(request, false, "staging_partition_missing");
            return;
        }
        Log::logf(CAT_OTA, LOG_DEBUG, "Staging to '%s' (0x%X, %u bytes)\n",
                     resmed_part->label, resmed_part->address, resmed_part->size);

        uploadOk = true;
    }

    if (!acceptUploadChunk(request, index, len, final)) return;
    if (uploadKind == UPLOAD_RESMED && resmed_part && uploadOk && len > 0) {
        // reject ESP32 binaries uploaded to resmed slot
        if (uploadSize == 0 && len > 0 && data[0] == 0xE9) {
            Log::logf(CAT_OTA, LOG_WARN, "Rejected: ESP32 binary uploaded to ResMed slot\n");
            uploadOk = false;
            return;
        }
        if (len > resmed_part->size - uploadSize) {
            Log::logf(CAT_OTA, LOG_WARN, "File too large for partition!\n");
            uploadOk = false;
            return;
        }
        if (!eraseUploadThrough(resmed_part, uploadSize + len)) {
            uploadOk = false;
            return;
        }
        esp_err_t err = esp_partition_write(resmed_part, uploadSize, data, len);
        if (err != ESP_OK) {
            Log::logf(CAT_OTA, LOG_ERROR, "Write failed at offset %u: %s\n",
                         uploadSize, esp_err_to_name(err));
            uploadOk = false;
            return;
        }
        uploadCrc = crc16_ccitt(data, len, uploadCrc);
        uploadSize += len;
    }

    if (final) {
        if (uploadOk) {
            Log::logf(CAT_OTA, LOG_INFO, "ResMed upload complete: %u bytes\n", uploadSize);
        }
    }
}

static void handleUploadDone(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) {
        finishUpload(request, false, "upload_unauthorized");
        return;
    }
    if (!ownsUpload(request)) return;

    String json = "{";
    bool validUpload = uploadComplete && uploadKind == UPLOAD_RESMED &&
                       uploadOk && uploadSize > 0 && resmed_part;
    jsonAddString(json, "ok", validUpload ? "true" : "false", false);
    jsonAddInt(json, "size", uploadSize);

    if (validUpload) {
        char hexcrc[8];
        snprintf(hexcrc, sizeof(hexcrc), "%04X", uploadCrc);
        jsonAddString(json, "crc", hexcrc);

        // Firmware verification
        fw_verify_result_t v = ResmedOta::verify_image(resmed_part, uploadSize);
        if (v.has_blx) {
            jsonAddString(json, "bid", v.bid);
            jsonAddString(json, "bid_ok", v.bid_ok ? "true" : "false");
            jsonAddString(json, "blx_crc", v.blx_crc_ok ? "ok" : "fail");
            const char *patch = "none";
            if (v.blx_patch == BLX_PATCH_A_DANGEROUS) patch = "method_a";
            else if (v.blx_patch == BLX_PATCH_B_SAFE) patch = "method_b";
            jsonAddString(json, "blx_patch", patch);
        }
        if (v.has_ccx) jsonAddString(json, "ccx_crc", v.ccx_crc_ok ? "ok" : "fail");
        if (v.has_cdx) jsonAddString(json, "cdx_crc", v.cdx_crc_ok ? "ok" : "fail");
    }

    finishUpload(request, validUpload,
                  validUpload ? nullptr : "resmed_upload_failed");
    json += '}';
    request->send(200, "application/json", json);
}


class LiveResponse : public BufferedResponse {
public:
    ~LiveResponse() override { aircannect::Memory::free(samples_); }

    bool prepare(uint16_t since) {
        samples_ = static_cast<LiveWebConsumer::Sample *>(aircannect::Memory::alloc_large(
            LiveWebConsumer::HISTORY_CAPACITY * sizeof(*samples_)));
        if (!samples_) return false;

        count_ = LiveWebConsumer::get_samples(samples_, LiveWebConsumer::HISTORY_CAPACITY,
                                               since, &sequence_);
        oxi_reading_t reading;
        OxiArbiter::snapshot(reading);
        spo2_ = reading.valid ? reading.spo2 : -1;
        pulse_ = reading.valid ? reading.pulse_bpm : -1;
        active_ = LiveWebConsumer::is_active();

        char token[128];
        size_t length = 0;
        for (uint16_t part = 0; part < count_ + 2; part++)
            length += formatPart(part, token, sizeof(token));
        BufferedResponse::begin(200, length);
        return true;
    }

    bool _sourceValid() const override { return samples_ != nullptr; }

protected:
    size_t readBody(size_t, char *out, size_t capacity) override {
        size_t written = 0;
        while (written < capacity && part_ < count_ + 2) {
            char token[128];
            size_t length = formatPart(part_, token, sizeof(token));
            size_t count = min(capacity - written, length - part_offset_);
            memcpy(out + written, token + part_offset_, count);
            written += count;
            part_offset_ += count;
            if (part_offset_ == length) { part_++; part_offset_ = 0; }
        }
        return written;
    }

private:
    size_t formatPart(uint16_t part, char *out, size_t capacity) const {
        if (part == 0) {
            return snprintf(out, capacity,
                "{\"seq\":%u,\"rate\":25,\"active\":\"%s\",\"spo2\":%d,\"pulse\":%d,\"samples\":[",
                sequence_, active_ ? "yes" : "no", spo2_, pulse_);
        }
        if (part <= count_) {
            const auto &sample = samples_[part - 1];
            return snprintf(out, capacity, "%s[%d,%d,%d]", part == 1 ? "" : ",",
                            sample.mkp, sample.rfl, sample.lyk);
        }
        memcpy(out, "]}", 2);
        return 2;
    }

    LiveWebConsumer::Sample *samples_ = nullptr;
    uint16_t count_ = 0, sequence_ = 0, part_ = 0;
    size_t part_offset_ = 0;
    int16_t spo2_ = -1, pulse_ = -1;
    bool active_ = false;
};

static void handleLive(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    uint16_t since = 0;
    if (request->hasArg("since"))
        since = (uint16_t)request->arg("since").toInt();

    auto *response = new (std::nothrow) LiveResponse;
    if (!response || !response->prepare(since)) {
        delete response;
        request->send(503, "application/json", "{\"error\":\"live_allocation_failed\"}");
        return;
    }
    request->send(response);
}


static void handleBleStatus(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    oxi_state_t st = OxiBle::get_state();
    oxi_reading_t r;
    OxiArbiter::snapshot(r);
    auto &cfg = Config::get();

    String json = "{";
    jsonAddString(json, "state", oxi_state_name(st), false);
    jsonAddString(json, "feeding", OxiArbiter::is_feeding() ? "yes" : "no");
    jsonAddInt(json, "spo2", r.valid ? r.spo2 : -1);
    jsonAddInt(json, "pulse", r.valid ? r.pulse_bpm : -1);
    jsonAddString(json, "auto_start", cfg.oxi_auto_start ? "yes" : "no");


    oxi_scan_result_t devs[MAX_SCAN_RESULTS];
    int scan_count = OxiBle::get_scan_results(devs, MAX_SCAN_RESULTS);
    json += ",\"devices\":[";
    for (int i = 0; i < scan_count; i++) {
        if (i > 0) json += ',';
        json += "{";
        jsonAddString(json, "addr", devs[i].addr, false);
        jsonAddString(json, "name", devs[i].name.c_str());
        jsonAddInt(json, "rssi", devs[i].rssi);
        json += "}";
    }
    json += "],\"known\":[";
    oxi_known_device_t known[MAX_KNOWN_DEVICES];
    int nk = OxiBle::get_known_devices(known, MAX_KNOWN_DEVICES);
    for (int i = 0; i < nk; i++) {
        if (i > 0) json += ',';
        json += '{';
        jsonAddString(json, "addr", known[i].addr, false);
        jsonAddString(json, "name", known[i].name);
        jsonAddBool(json, "autoconnect", known[i].autoconnect);
        json += '}';
    }
    json += "]}";
    request->send(200, "application/json", json);
}


static void handleBleAction(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String body;
    if (!getBody(request, body)) return;
    String action, addr, enabled;
    struct { String *action; String *addr; String *enabled; } ctx = {&action, &addr, &enabled};
    if (!json_foreach_kv(body, [](const char *key, const char *val, void *p) {
        auto *c = (decltype(ctx)*)p;
        if (strcmp(key, "action") == 0) *c->action = val;
        else if (strcmp(key, "addr") == 0) *c->addr = val;
        else if (strcmp(key, "enabled") == 0) *c->enabled = val;
    }, &ctx)) {
        request->send(400, "application/json", "{\"error\":\"bad_json\"}");
        return;
    }

    String result = "unknown action";
    bool ok = false;

    if (action == "scan") {
        OxiBle::start_scan();
        result = "scan started";
        ok = true;
    } else if (action == "stop_scan") {
        OxiBle::stop_scan();
        result = "scan stopped";
        ok = true;
    } else if (action == "connect") {
        if (addr.length() > 0) {
            OxiBle::connect(addr.c_str());
        } else {
            OxiBle::connect(nullptr);
        }
        result = "connecting";
        ok = true;
    } else if (action == "autoconnect") {
        if (enabled != "true" && enabled != "false") {
            request->send(400, "application/json", "{\"error\":\"enabled must be true or false\"}");
            return;
        }
        ok = OxiBle::set_autoconnect(addr.c_str(), enabled == "true");
        result = ok ? "autoconnect saved" : "unknown sensor or settings write failed";
    } else if (action == "disconnect") {
        OxiBle::disconnect();
        result = "disconnected";
        ok = true;
    } else if (action == "enable") {
        OxiBle::enable();
        result = "oximetry enabled";
        ok = true;
    } else if (action == "disable") {
        OxiBle::disable();
        result = "oximetry disabled";
        ok = true;
    } else if (action == "start_feed") {
        OxiArbiter::start_feed();
        result = "feeding started";
        ok = true;
    } else if (action == "stop_feed") {
        OxiArbiter::stop_feed();
        result = "feeding stopped";
        ok = true;
    } else if (action == "delete_bond") {
        if (addr.length() > 0) {
            OxiBle::request_remove_known(addr.c_str());
            result = "queued";
            ok = true;
        } else {
            result = "no address specified";
            ok = true;
        }
    } else if (action == "delete_all_bonds") {
        OxiBle::request_clear_all_known();
        result = "queued";
        ok = true;
    }

    String json = "{";
    jsonAddBool(json, "ok", ok, false);
    jsonAddString(json, "result", result.c_str());
    json += '}';
    request->send(200, "application/json", json);
}


extern void dispatch_command(const char *line, String &response);

struct WebCommand {
    AsyncWebServerRequestPtr request;
    uart_transaction_t *transaction = nullptr;
    bool refresh_therapy = false;
};
static WebCommand web_commands[4];
static SemaphoreHandle_t command_mutex = nullptr;

static void queueWebCommand(AsyncWebServerRequest *request, const String &cmd) {
    if (!command_mutex || xSemaphoreTake(command_mutex, 0) != pdTRUE) {
        request->send(503, "application/json", "{\"ok\":false,\"error\":\"command_busy\"}");
        return;
    }
    WebCommand *slot = nullptr;
    for (auto &pending : web_commands) if (!pending.transaction) { slot = &pending; break; }
    auto *transaction = slot ? Arbiter::begin_cmd(cmd.c_str(), CMD_SRC_TCP,
                                                  CMD_PRIO_NORMAL, uint16_t(128)) : nullptr;
    if (transaction) {
        slot->request = request->pause();
        slot->transaction = transaction;
        slot->refresh_therapy = cmd.startsWith("P S #ROP ");
    }
    xSemaphoreGive(command_mutex);
    if (!transaction)
        request->send(503, "application/json", "{\"ok\":false,\"error\":\"command_unavailable\"}");
}

static void serviceWebCommands() {
    if (!command_mutex) return;
    for (auto &slot : web_commands) {
        if (xSemaphoreTake(command_mutex, 0) != pdTRUE) return;
        if (!slot.transaction || (!slot.request.expired() &&
            !Arbiter::transaction_done(slot.transaction) &&
            !Arbiter::transaction_expired(slot.transaction))) {
            xSemaphoreGive(command_mutex);
            continue;
        }
        WebCommand pending = std::move(slot);
        slot.transaction = nullptr;
        xSemaphoreGive(command_mutex);

        auto request = pending.request.lock();
        char response[128] = {};
        uint16_t length = sizeof(response);
        bool ok = false;
        Arbiter::VarReadTrace trace;
        if (request && Arbiter::transaction_done(pending.transaction)) {
            ok = Arbiter::finish_cmd(pending.transaction, response, &length, nullptr, &trace);
        } else {
            Arbiter::cancel_transaction(pending.transaction);
            length = 0;
            trace.outcome = "deadline";
        }
        if (!request) continue;

        if (ok && pending.refresh_therapy) AirSenseState::request_refresh();
        String json = "{";
        jsonAddBool(json, "ok", ok, false);
        if (!ok)
            jsonAddString(json, "error", strcmp(trace.outcome, "ok") ? trace.outcome : "command_failed");
        if (length) jsonAddString(json, "response", response);
        json += '}';
        request->send(200, "application/json", json);
    }
}

static void handleCmd(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String body;
    if (!getBody(request, body)) return;
    aircannect::JsonAllocator allocator;
    JsonDocument doc(&allocator);
    if (!parseJsonObject(body, doc) || !doc["cmd"].is<const char *>()) {
        request->send(400, "application/json", "{\"ok\":false,\"error\":\"missing cmd\"}");
        return;
    }
    String cmd = doc["cmd"].as<const char *>();
    if (!cmd.startsWith("$")) {
        queueWebCommand(request, cmd);
        return;
    }

    String json = "{";
    String response;
    dispatch_command(cmd.c_str() + 1, response);
    response.trim();
    jsonAddBool(json, "ok", true, false);
    jsonAddString(json, "response", response.c_str());
    json += '}';
    request->send(200, "application/json", json);
}


static void handleFlashStatus(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String json = "{";
    jsonAddString(json, "active", ResmedOta::is_active() ? "true" : "false", false);
    jsonAddString(json, "phase", ResmedOta::get_phase());
    jsonAddInt(json, "sent", ResmedOta::get_sent());
    jsonAddInt(json, "total", ResmedOta::get_total());
    const char *err = ResmedOta::last_error();
    if (err && err[0]) {
        jsonAddString(json, "error", err);
    }

    if (hasValidResmedUpload()) {
        const char *detected = ResmedOta::detect_block(uploadSize);
        jsonAddString(json, "detected_block", detected ? detected : "unknown");
        jsonAddInt(json, "fw_size", uploadSize);
    }
    json += '}';
    request->send(200, "application/json", json);
}


static void handleFlashStart(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    String body;
    if (!getBody(request, body)) return;

    aircannect::JsonAllocator allocator;
    JsonDocument doc(&allocator);
    if (!parseJsonObject(body, doc)) {
        request->send(400, "application/json", "{\"error\":\"bad_json\"}");
        return;
    }

    if (ResmedOta::is_active()) {
        request->send(409, "application/json", "{\"ok\":false,\"error\":\"flash already active\"}");
        return;
    }

    if (!OtaManager::begin_resmed_flash()) {
        request->send(409, "application/json",
                      "{\"ok\":false,\"error\":\"another OTA is active\"}");
        return;
    }

    if (!hasValidResmedUpload()) {
        OtaManager::cancel_resmed_flash_claim();
        request->send(400, "application/json", "{\"ok\":false,\"error\":\"no valid ResMed firmware uploaded\"}");
        return;
    }

    String block = doc["block"] | "";
    bool flash_blx = doc["flash_blx"] | false;
    bool force_blx = doc["force_blx"] | false;

    ResmedOta::start_flash(
        block.length() > 0 ? block.c_str() : nullptr,
        uploadSize,
        flash_blx,
        force_blx
    );

    String json = "{";
    jsonAddString(json, "ok", "true", false);
    jsonAddString(json, "block", block.length() > 0 ? block.c_str() :
                  (ResmedOta::detect_block(uploadSize) ? ResmedOta::detect_block(uploadSize) : "auto"));
    json += '}';
    request->send(200, "application/json", json);
}


static void handleFlashCancel(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    ResmedOta::cancel();
    request->send(200, "application/json", "{\"ok\":true}");
}



// ESP32 OTA
static void abortEspOtaUpload(AsyncWebServerRequest *request) {
    if (uploadOwner != request) return;
    OtaManager::abort_image();
    uploadOk = false;
    // Keep the request reservation until completion or disconnect.
}

static void handleEspOtaChunk(AsyncWebServerRequest *request, const String& filename,
                               size_t index, uint8_t *data, size_t len, bool final) {
    if (index == 0) {
        if (!claimUpload(request)) {
            Log::logf(CAT_OTA, LOG_WARN, "ESP OTA rejected: OTA busy\n");
            return;
        }
        Log::logf(CAT_OTA, LOG_DEBUG, "ESP OTA start: %s\n", filename.c_str());
        uploadKind = UPLOAD_ESP;
        resmed_part = nullptr;
        uploadSize = 0;
        uploadOk = false;

        uploadOwnsUart = true;
        Arbiter::set_state(SYS_OTA_ESP);
        uploadOk = OtaManager::begin_image();
    }

    if (!acceptUploadChunk(request, index, len, final)) return;
    if (uploadKind == UPLOAD_ESP && uploadOk && len > 0) {
        if (!OtaManager::write_image(index, data, len)) {
            abortEspOtaUpload(request);
            return;
        }
        uploadSize = OtaManager::image_status().bytes;
    }

    if (final && uploadOk) {
        Log::logf(CAT_OTA, LOG_DEBUG, "ESP OTA upload complete: %u bytes\n", uploadSize);
    }
}

static void handleEspOtaDone(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) {
        finishUpload(request, false, "upload_unauthorized");
        return;
    }
    if (!ownsUpload(request)) return;

    bool ok = false;
    if (uploadComplete && uploadKind == UPLOAD_ESP && uploadOk)
        ok = OtaManager::finish_image();
    const auto &image = OtaManager::image_status();
    const char *error = image.error ? image.error : "invalid firmware image";
    if (ok)
        Log::logf(CAT_OTA, LOG_INFO, "ESP OTA OK, boot set to '%s'\n", image.partition);

    String json = "{";
    jsonAddString(json, "ok", ok ? "true" : "false", false);
    if (!ok) jsonAddString(json, "error", error);
    jsonAddInt(json, "size", image.bytes);
    jsonAddInt(json, "wire_size", image.wire_bytes);
    jsonAddString(json, "encoding", OtaImage::encoding_name(image.encoding));
    if (image.partition[0]) jsonAddString(json, "partition", image.partition);

    finishUpload(request, ok, ok ? nullptr : error);
    uploadKind = UPLOAD_NONE;

    json += '}';
    request->send(200, "application/json", json);
}

static String buildOtaStatus(const OtaManager::Status &status, bool full) {
    String json = "{";
    json.reserve(full ? 512 : 96);
    jsonAddString(json, "state", OtaManager::state_name(status.state), false);
    jsonAddUInt32(json, "revision", status.revision);
    if (status.state == OtaManager::State::Installing ||
        status.state == OtaManager::State::Rebooting)
        jsonAddInt(json, "progress", status.progress);
    else if (status.blocked && status.state != OtaManager::State::Busy &&
             status.state != OtaManager::State::Checking &&
             status.state != OtaManager::State::Disabled)
        jsonAddString(json, "blocked", status.blocked);
    if (full) {
        jsonAddString(json, "version", airbridge_version());
        jsonAddUInt32(json, "uptime", millis() / 1000);
        jsonAddString(json, "release_target", AB_OTA_RELEASE_TARGET);
        json += ",\"upload_encodings\":[\"auto\",\"plain\",\"zlib\"]";
        jsonAddInt(json, "bytes", status.bytes);
        jsonAddInt(json, "total_size", status.total_size);
        jsonAddInt(json, "last_check_age_ms", status.last_check_age_ms);
    }
    if (status.update_version[0]) jsonAddString(json, "update_version", status.update_version);
    if (status.state == OtaManager::State::Error) jsonAddString(json, "error", status.error);
    json += '}';
    return json;
}

static void sendOtaStatus(AsyncWebServerRequest *request, int status_code) {
    OtaManager::Status status;
    if (!OtaManager::get_status(status)) {
        request->send(503, "application/json", "{\"error\":\"ota_status_busy\"}");
        return;
    }
    request->send(status_code, "application/json", buildOtaStatus(status, true));
}

static void publishOta() {
    publishState(EventTopic::Ota, OtaManager::revision() + 1, "ota", [](bool) {
        OtaManager::Status status;
        return OtaManager::get_status(status) ? buildOtaStatus(status, false) : String();
    });
}

static void publishWifi() {
    const uint32_t revision = WiFiSetup::revision();
    publishState(EventTopic::Wifi, revision + 1, "wifi", [revision](bool) {
        String json = "{";
        jsonAddUInt32(json, "revision", revision, false);
        json += '}';
        return json;
    });
}

static void handleOtaStatus(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    sendOtaStatus(request, 200);
}

static void handleOtaRequest(AsyncWebServerRequest *request, bool install) {
    if (!checkAuth(request)) return;
    const char *error = nullptr;
    bool accepted = install ? OtaManager::request_install(&error) : OtaManager::request_check(&error);
    if (accepted) sendOtaStatus(request, 202);
    else {
        String json = "{";
        jsonAddString(json, "error", error, false);
        json += '}';
        request->send(409, "application/json", json);
    }
}

static void handleOtaCheck(AsyncWebServerRequest *request) {
    handleOtaRequest(request, false);
}

static void handleOtaInstall(AsyncWebServerRequest *request) {
    handleOtaRequest(request, true);
}

// WiFi management

static void handleWifiGet(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    auto &cfg = Config::get();

    String json = "{";
    jsonAddString(json, "state", WiFiSetup::state_name(), false);
    jsonAddString(json, "ssid", WiFiSetup::connected_ssid());
    jsonAddInt(json, "rssi", WiFiSetup::current_rssi());
    jsonAddInt(json, "net_idx", WiFiSetup::connected_net_idx());
    jsonAddString(json, "ip", WiFiSetup::is_connected() ? WiFi.localIP().toString().c_str() : "");
    jsonAddString(json, "roam", cfg.wifi_roam ? "1" : "0");

    json += ",\"networks\":[";
    for (int i = 0; i < cfg.wifi_net_count; i++) {
        if (i > 0) json += ',';
        json += "{";
        jsonAddString(json, "ssid", cfg.wifi_nets[i].ssid.c_str(), false);
        jsonAddString(json, "enabled", cfg.wifi_nets[i].enabled ? "1" : "0");
        // Hint info comes from NetworkHints now (one slot may have multiple
        // BSSID hints with multi-AP roaming; pick the most recent here).
        const NetworkHint *h = NetworkHints::find_best(cfg.wifi_nets[i].ssid.c_str());
        jsonAddString(json, "hint", h ? "1" : "0");
        jsonAddInt(json, "channel", h ? h->channel : 0);
        jsonAddInt(json, "rssi", WiFiSetup::net_rssi(i));
        json += "}";
    }
    json += "]}";
    request->send(200, "application/json", json);
}

static void handleWifiPost(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    String body;
    if (!getBody(request, body)) return;
    String action, ssid, pass;
    int idx = -1;

    struct wifi_kv_ctx { String *action; String *ssid; String *pass; int *idx; bool pass_set; };
    wifi_kv_ctx wctx = {&action, &ssid, &pass, &idx, false};
    if (!json_foreach_kv(body, [](const char *key, const char *val, void *p) {
        wifi_kv_ctx *c = (wifi_kv_ctx *)p;
        if (strcmp(key, "action") == 0) *c->action = val;
        else if (strcmp(key, "ssid") == 0) *c->ssid = val;
        else if (strcmp(key, "pass") == 0) { *c->pass = val; c->pass_set = true; }
        else if (strcmp(key, "idx") == 0) *c->idx = atol(val);
    }, &wctx)) {
        request->send(400, "application/json", "{\"error\":\"bad_json\"}");
        return;
    }

    String result = "unknown action";
    bool ok = false;

    if (action == "add") {
        if (ssid.length() > 0) {
            ok = Config::add_network(ssid.c_str(), pass.c_str());
            result = ok ? "network added" : "list full or NVS save failed";
        } else {
            result = "ssid required";
        }
    } else if (action == "remove") {
        if (idx >= 0) {
            ok = Config::remove_network((uint8_t)idx);
            result = ok ? "network removed" : "invalid index or NVS save failed";
        } else {
            result = "idx required";
        }
    } else if (action == "update") {
        auto &cfg = Config::get();
        if (idx >= 0 && idx < cfg.wifi_net_count) {
            if (ssid.length() > 0) cfg.wifi_nets[idx].ssid = ssid;
            if (wctx.pass_set) cfg.wifi_nets[idx].pass = pass;
            ok = Config::save_wifi_nets();
            result = ok ? "network updated" : "NVS save failed";
        } else {
            result = "invalid index";
        }
    }

    if (ok) WiFiSetup::profiles_changed();

    String json = "{";
    jsonAddString(json, "ok", ok ? "true" : "false", false);
    jsonAddString(json, "result", result.c_str());
    json += '}';
    request->send(200, "application/json", json);
}

static void handleOnboardingComplete(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    String body;
    if (!getBody(request, body)) return;
    struct Access {
        String user, password;
        bool has_user = false, has_password = false, invalid = false;
    } access;
    if (!json_foreach_kv(body, [](const char *key, const char *value, void *ctx) {
        auto &a = *static_cast<Access *>(ctx);
        if (!strcmp(key, "http_user")) { a.user = value; a.has_user = true; }
        else if (!strcmp(key, "http_pass")) { a.password = value; a.has_password = true; }
        else a.invalid = true;
    }, &access) || access.invalid) {
        request->send(400, "application/json", "{\"ok\":false,\"error\":\"bad_json\"}");
        return;
    }
    const bool ok = Config::complete_onboarding(
        access.has_user ? access.user.c_str() : nullptr,
        access.has_password ? access.password.c_str() : nullptr);
    request->send(ok ? 200 : 500, "application/json",
                  ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"nvs_save_failed\"}");
}

static void handleReboot(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    // send() stages the response; transmission starts after this handler returns.
    request->onDisconnect([]() { OtaManager::request_reboot(); });
    request->send(200, "application/json", "{\"ok\":true}");
}


void WebUI::push_event(const char *event, const char *json) {
    AsyncEventSource *target = strcmp(event, "live") == 0 ? live_events : events;
    if (target && target->count()) target->send(json, event, millis());
}

void WebUI::push_event(const char *event, const String &json) {
    push_event(event, json.c_str());
}


static void handleTimeAction(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    String body;
    if (!getBody(request, body)) return;
    String action;
    if (!json_foreach_kv(body, [](const char *key, const char *val, void *p) {
        if (strcmp(key, "action") == 0) *(String*)p = val;
    }, &action)) {
        request->send(400, "application/json", "{\"error\":\"bad_json\"}");
        return;
    }

    String result = "unknown action";
    bool ok = false;

    if (action == "ntp_sync") {
        WiFiSetup::force_ntp_sync();
        result = "NTP resync triggered";
        ok = true;
    } else if (action == "sync_to_resmed") {
        Air10Clock::request_sync(true);
        ok = true;
        result = "ResMed clock sync requested";
    } else if (action == "sync_from_resmed") {
        ok = Air10Clock::pull_time(true);
        result = ok ? "ESP32 clock set from ResMed" : "Failed to read ResMed clock";
    }

    String json = "{";
    jsonAddString(json, "ok", ok ? "true" : "false", false);
    jsonAddString(json, "result", result.c_str());
    json += '}';
    request->send(200, "application/json", json);
}

#if AB_STORAGE_HAS_SDCARD
class StorageResponse : public AsyncAbstractResponse {
public:
    StorageResponse(std::shared_ptr<StorageBrowser::Transfer> transfer,
                    StorageBrowser::Kind kind, uint64_t size) : transfer_(std::move(transfer)) {
        _code = 200;
        _contentType = kind == StorageBrowser::Kind::List ? "application/json" :
            kind == StorageBrowser::Kind::Archive ? "application/zip" : "application/octet-stream";
        _contentLength = size;
        _chunked = kind == StorageBrowser::Kind::Archive;
        _sendContentLength = !_chunked;
    }
    ~StorageResponse() override { transfer_->cancel(); }
    bool _sourceValid() const override { return !transfer_->failed(); }
    size_t _fillBuffer(uint8_t *out, size_t capacity) override {
        const size_t count = transfer_->read(out, capacity);
        return count || transfer_->finished() ? count : RESPONSE_TRY_AGAIN;
    }
private:
    std::shared_ptr<StorageBrowser::Transfer> transfer_;
};

static void handleStorageUsb(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    const String enabled = request->hasParam("enabled", true)
        ? request->getParam("enabled", true)->value() : "";
    if (enabled != "0" && enabled != "1") {
        request->send(400, "application/json", "{\"error\":\"enabled must be 0 or 1\"}");
        return;
    }
    const char *error = nullptr;
    if (!SdStorage::request_usb(enabled == "1", &error)) {
        String json = "{";
        jsonAddString(json, "error", error, false);
        json += '}';
        request->send(409, "application/json", json);
        return;
    }
    request->send(202, "application/json", "{\"ok\":true}");
}

static void handleStorage(AsyncWebServerRequest *request, StorageBrowser::Kind kind) {
    if (!checkAuth(request)) return;
    StorageBrowser::Request operation = {};
    operation.kind = kind;
    const bool mutation = kind == StorageBrowser::Kind::Rename || kind == StorageBrowser::Kind::Delete;
    const String path = request->hasParam("path", mutation)
        ? request->getParam("path", mutation)->value() : "/";
    if (path.isEmpty() || path[0] != '/' || path.length() >= sizeof(operation.path) ||
        path.length() != strlen(path.c_str())) {
        request->send(400, "application/json", "{\"error\":\"invalid_path\"}");
        return;
    }
    strcpy(operation.path, path.c_str());
    if (request->hasParam("offset")) {
        const String value = request->getParam("offset")->value();
        char *end = nullptr;
        errno = 0;
        const unsigned long offset = strtoul(value.c_str(), &end, 10);
        if (!value.length() || value[0] == '-' || errno || *end || offset > UINT32_MAX) {
            request->send(400, "application/json", "{\"error\":\"invalid_offset\"}");
            return;
        }
        operation.offset = offset;
    }
    size_t length = 0;
    for (size_t i = 0; i < request->params(); i++) {
        const auto *param = request->getParam(i);
        if (param->name() != "item" || param->isPost() != mutation) continue;
        const String &name = param->value();
        if (name.isEmpty() || name.indexOf('\n') >= 0 || name.length() != strlen(name.c_str()) ||
            length + name.length() + 2 > sizeof(operation.selection)) {
            request->send(400, "application/json", "{\"error\":\"invalid_selection\"}");
            return;
        }
        if (length) operation.selection[length++] = '\n';
        memcpy(operation.selection + length, name.c_str(), name.length() + 1);
        length += name.length();
    }
    if (kind == StorageBrowser::Kind::Rename) {
        const String name = request->hasParam("name", true) ? request->getParam("name", true)->value() : "";
        if (length || name.isEmpty() || name.length() >= 256 || name.length() != strlen(name.c_str())) {
            request->send(400, "application/json", "{\"error\":\"invalid_name\"}");
            return;
        }
        strcpy(operation.selection, name.c_str());
    }
    String disposition;
    if (kind == StorageBrowser::Kind::File || kind == StorageBrowser::Kind::Archive) {
        disposition = "attachment; filename*=UTF-8''";
        const char *name = kind == StorageBrowser::Kind::Archive ? "airbridge.zip" : strrchr(path.c_str(), '/') + 1;
        for (; *name; name++) {
            char encoded[4];
            snprintf(encoded, sizeof(encoded), "%%%02X", static_cast<unsigned char>(*name));
            disposition += encoded;
        }
    }
    auto paused = request->pause();
    std::weak_ptr<StorageBrowser::Transfer> active;
    const auto result = StorageBrowser::start(operation,
        [paused, kind, disposition, mutation](int code, const char *error,
                                  std::shared_ptr<StorageBrowser::Transfer> transfer, uint64_t size) {
            auto request = paused.lock();
            if (!request) { if (transfer) transfer->cancel(); return; }
            if (mutation) {
                String body = "{\"ok\":";
                body += error ? "false" : "true";
                jsonAddUInt32(body, "changed", size);
                if (error) jsonAddString(body, "error", error);
                body += '}';
                request->send(code, "application/json", body);
                return;
            }
            if (error) {
                String body = "{\"error\":";
                jsonQuote(body, error);
                body += '}';
                request->send(code, "application/json", body);
                return;
            }
            auto *response = new(std::nothrow) StorageResponse(transfer, kind, size);
            if (!response) {
                Log::logf(CAT_WEB, LOG_WARN, "Storage response allocation failed\n");
                transfer->cancel();
                request->send(503);
                return;
            }
            if (disposition.length()) response->addHeader("Content-Disposition", disposition);
            request->send(response);
        }, active);
    if (result == StorageBrowser::StartResult::Started) request->onDisconnect([active]() {
        if (auto transfer = active.lock()) transfer->cancel();
    });
    else if (result == StorageBrowser::StartResult::BadRequest)
        request->send(400, "application/json", "{\"error\":\"invalid_path\"}");
    else if (result == StorageBrowser::StartResult::Busy)
        request->send(409, "application/json", "{\"error\":\"storage_busy\"}");
    else request->send(503, "application/json", "{\"error\":\"storage_unavailable\"}");
}
#endif

void WebUI::init(uint16_t port) {
    if (port == 0) return;
    ClinicalJobs::init(saveSettings);
    command_mutex = xSemaphoreCreateMutex();

    http = new AsyncWebServer(port);
    events = new AsyncEventSource("/events");
    live_events = new AsyncEventSource("/events/live");
    auto authenticate_events = [](AsyncWebServerRequest *request, ArMiddlewareNext next) {
        if (checkAuth(request)) next();
    };
    events->addMiddleware(authenticate_events);
    events->onConnect(registerEventClient);
    events->onDisconnect(unregisterEventClient);
    live_events->addMiddleware(authenticate_events);
    live_events->onConnect([](AsyncEventSourceClient *) {
        LiveWebConsumer::acquire();
    });
    live_events->onDisconnect([](AsyncEventSourceClient *) {
        LiveWebConsumer::release();
    });
    http->addHandler(events);
    http->addHandler(live_events);

    http->on("/", HTTP_GET, handleRoot);
    http->on("/wizard", HTTP_GET, handleRoot);
    http->on("/api/onboarding", HTTP_POST, handleOnboardingComplete, NULL, handleJsonBody);
    http->on("/api/status", HTTP_GET, handleStatus);
    http->on(AsyncURIMatcher::exact("/api/crash"), HTTP_GET, handleCrashStatus);
    http->on(AsyncURIMatcher::exact("/api/crash/dump"), HTTP_GET, handleCrashDump);
    http->on("/api/settings", HTTP_GET, handleGetSettings);
    http->on("/api/settings", HTTP_POST, handlePostSettings, NULL, handleJsonBody);
    http->on("/api/config", HTTP_GET, handleGetConfig);
    http->on("/api/config", HTTP_POST, handlePostConfig, NULL, handleJsonBody);
    http->on("/api/export", HTTP_GET, handleExportStatus);
    http->on("/api/export/smb", HTTP_POST, [](AsyncWebServerRequest *r) { handleExportRequest(r, true); });
    http->on("/api/export/sleephq", HTTP_POST, [](AsyncWebServerRequest *r) { handleExportRequest(r, false); });
#if AB_STORAGE_HAS_SDCARD
    http->on("/api/storage/usb", HTTP_POST, handleStorageUsb);
    http->on("/api/storage/list", HTTP_GET, [](AsyncWebServerRequest *r) {
        handleStorage(r, StorageBrowser::Kind::List);
    });
    http->on("/api/storage/download", HTTP_GET, [](AsyncWebServerRequest *r) {
        handleStorage(r, StorageBrowser::Kind::File);
    });
    http->on("/api/storage/archive", HTTP_GET, [](AsyncWebServerRequest *r) {
        handleStorage(r, StorageBrowser::Kind::Archive);
    });
    http->on("/api/storage/rename", HTTP_POST, [](AsyncWebServerRequest *r) {
        handleStorage(r, StorageBrowser::Kind::Rename);
    });
    http->on("/api/storage/delete", HTTP_POST, [](AsyncWebServerRequest *r) {
        handleStorage(r, StorageBrowser::Kind::Delete);
    });
#endif
    http->on("/api/live", HTTP_GET, handleLive);
    http->on("/api/upload", HTTP_POST, handleUploadDone, handleUploadChunk);
    http->on("/api/ble", HTTP_GET, handleBleStatus);
    http->on("/api/ble", HTTP_POST, handleBleAction, NULL, handleJsonBody);
    http->on("/api/cmd", HTTP_POST, handleCmd, NULL, handleJsonBody);
    http->on("/api/flash", HTTP_GET, handleFlashStatus);
    http->on("/api/flash", HTTP_POST, handleFlashStart, NULL, handleJsonBody);
    http->on("/api/flash/cancel", HTTP_POST, handleFlashCancel);
    http->on("/api/time", HTTP_POST, handleTimeAction, NULL, handleJsonBody);
    http->on("/api/report", HTTP_GET, handleReport);
    http->on("/api/report", HTTP_POST, handleReport);
    http->on("/api/report", HTTP_DELETE, handleReport);
    http->on("/api/wifi", HTTP_GET, handleWifiGet);
    http->on("/api/wifi", HTTP_POST, handleWifiPost, NULL, handleJsonBody);
    http->on("/api/esp32/upload", HTTP_POST, handleEspOtaDone, handleEspOtaChunk);
    http->on("/api/ota", HTTP_GET, handleOtaStatus);
    http->on("/api/ota/check", HTTP_POST, handleOtaCheck);
    http->on("/api/ota/install", HTTP_POST, handleOtaInstall);
    http->on("/api/reboot", HTTP_POST, handleReboot);

    DefaultHeaders::Instance().addHeader("Cache-Control", "no-store");

    http->begin();
    Log::logf(CAT_WEB, LOG_INFO, "HTTP server on port %d\n", port);
}

static uint8_t statusChanges(const DeviceStatus::Snapshot &a,
                             const DeviceStatus::Snapshot &b) {
    uint8_t fields = 0;
    if (a.rop != b.rop || a.sys != b.sys || a.mhr != b.mhr || a.mop != b.mop)
        fields |= STATUS_THERAPY;
    if (a.oxi != b.oxi || a.feeding != b.feeding ||
        strcmp(a.oxi_source, b.oxi_source) || strcmp(a.oxi_name, b.oxi_name) ||
        a.reading.valid != b.reading.valid ||
        (a.reading.valid && (a.reading.spo2 != b.reading.spo2 ||
                            a.reading.pulse_bpm != b.reading.pulse_bpm)))
        fields |= STATUS_OXI;
    if (a.report_source != b.report_source || a.report_revision != b.report_revision ||
        a.report_data_revision != b.report_data_revision ||
        a.report_state != b.report_state || strcmp(a.report_error, b.report_error) ||
        a.report_available != b.report_available)
        fields |= STATUS_REPORT;
    if (a.storage.mode != b.storage.mode || a.storage.mounted != b.storage.mounted ||
        a.storage.supported != b.storage.supported ||
        a.storage.card_bytes != b.storage.card_bytes || a.storage.used_bytes != b.storage.used_bytes ||
        a.storage.usb_supported != b.storage.usb_supported ||
        strcmp(a.storage.error, b.storage.error))
        fields |= STATUS_STORAGE;
    return fields;
}

static String build_status_payload(const DeviceStatus::Snapshot &status,
                                   uint8_t fields = STATUS_ALL) {
    String sj = "{";
    jsonAddUInt32(sj, "sample_ms", millis(), false);
    appendStatusFields(sj, status, fields);
    sj += '}';
    return sj;
}

static DeviceStatus::Snapshot last_published = {
    SYS_IDLE, OXI_DISABLED, {}, INT_MIN, INT_MIN, INT_MIN, false
};

static bool status_requested = true;

void WebUI::push_status_event() {
    status_requested = true;
}

static void publishStatus() {
    static uint32_t revision = 1, health_at = 0, oxi_at = 0;
    static uint32_t ble_revision = 0, config_revision = 0, device_revision = 0;
    static uint8_t pending_fields = STATUS_ALL;
    const auto status = DeviceStatus::snapshot();
    const uint32_t now = millis();
    uint8_t fields = statusChanges(status, last_published);
    if (status_requested) fields |= STATUS_THERAPY;
    if ((fields & STATUS_OXI) || OxiBle::revision() != ble_revision ||
        (now - oxi_at >= 2000 && live_events && live_events->count())) {
        fields |= STATUS_OXI;
        oxi_at = now;
    }
    if (now - health_at >= 10000) {
        fields |= STATUS_HEALTH;
        health_at = now;
    }
    if (Config::revision() != config_revision) fields |= STATUS_CONFIG;
    if (AirSenseState::identity_revision() != device_revision) fields |= STATUS_IDENTITY;
    status_requested = false;
    if (fields) {
        last_published = status;
        ble_revision = OxiBle::revision();
        config_revision = Config::revision();
        device_revision = AirSenseState::identity_revision();
        pending_fields = fields;
        if (!++revision) ++revision;
    }
    publishState(EventTopic::Status, revision, "status", [&](bool full) {
        return build_status_payload(status, full ? STATUS_ALL : pending_fields);
    });
}

void WebUI::handle() {
    serviceWebCommands();
    if (!events || events->count() == 0) return;
    static uint32_t last_check = 0;
    uint32_t now = millis();
    if (uint32_t(now - last_check) < 100) return;
    last_check = now;
    publishStatus();
    publishOta();
    publishWifi();
#if AB_STORAGE_HAS_SDCARD
    publishExports();
#endif

}
