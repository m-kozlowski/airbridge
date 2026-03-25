#include "web_ui.h"
#include "web_ui_html.h"
#include "settings_defs.h"
#include "uart_arbiter.h"
#include "ble_oxi.h"
#include "resmed_ota.h"
#include "debug_log.h"
#include "app_config.h"

#include <ESPAsyncWebServer.h>
#include <esp_partition.h>
#include <NimBLEDevice.h>
extern "C" {
#include "nimble/nimble/host/include/host/ble_store.h"
}

extern const char *airbridge_version();
extern const char *airbridge_build_date();

static AsyncWebServer *http = nullptr;
static AsyncEventSource *events = nullptr;

static bool checkAuth(AsyncWebServerRequest *request) {
    auto &cfg = Config::get();
    if (!request->authenticate(cfg.http_user.c_str(), cfg.http_pass.c_str())) {
        request->requestAuthentication();
        return false;
    }
    return true;
}

// Response format: "X S #CMD = XXXX #" or just hex digits after "= "
static int parseResponseValue(const char *resp) {
    const char *eq = strstr(resp, "= ");
    if (!eq) return -1;
    // Skip "= ", read hex
    const char *val = eq + 2;
    // Strip trailing " #" if present
    return (int)strtol(val, nullptr, 16);
}

static bool readSetting(const char *cmd, int &value) {
    char req[16];
    snprintf(req, sizeof(req), "G S #%s", cmd);
    char resp[64] = {};
    uint16_t resp_len = sizeof(resp);
    bool ok = Arbiter::send_cmd(req, CMD_SRC_TCP, CMD_PRIO_NORMAL,
                                 resp, &resp_len, 2000);
    if (!ok) return false;
    value = parseResponseValue(resp);
    return value >= 0;
}

static bool writeSetting(const char *cmd, int value) {
    char req[32];
    snprintf(req, sizeof(req), "P S #%s %04X", cmd, (uint16_t)value);
    char resp[64] = {};
    uint16_t resp_len = sizeof(resp);
    return Arbiter::send_cmd(req, CMD_SRC_TCP, CMD_PRIO_NORMAL,
                              resp, &resp_len, 2000);
}

static void jsonAddString(String &out, const char *key, const char *val, bool comma = true) {
    if (comma) out += ',';
    out += '"';
    out += key;
    out += "\":\"";
    while (*val) {
        if (*val == '"') out += "\\\"";
        else if (*val == '\\') out += "\\\\";
        else if (*val == '\n') out += "\\n";
        else if (*val == '\r') out += "\\r";
        else if (*val == '\t') out += "\\t";
        else if ((uint8_t)*val < 0x20) {} // skip other control chars
        else out += *val;
        val++;
    }
    out += '"';
}

static void jsonAddInt(String &out, const char *key, int val, bool comma = true) {
    if (comma) out += ',';
    out += '"';
    out += key;
    out += "\":";
    out += String(val);
}


static String pending_body;
static bool pending_body_ready = false;

static void handleJsonBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
    if (index == 0) {
        pending_body = "";
        pending_body.reserve(total);
        pending_body_ready = false;
    }
    pending_body.concat((char *)data, len);
    if (index + len == total) {
        pending_body_ready = true;
    }
}

static String getBody(AsyncWebServerRequest *request) {
    if (pending_body_ready) {
        pending_body_ready = false;
        String result = pending_body;
        pending_body = "";
        return result;
    }
    return "";
}

static void handleRoot(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    AsyncWebServerResponse *response = request->beginResponse(200, "text/html", (const uint8_t*)HTML_PAGE, strlen_P(HTML_PAGE));
    request->send(response);
}


static void handleStatus(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    static const char *state_names[] = {
        "IDLE", "THERAPY", "BOOTLOADER", "OTA_ESP",
        "OTA_AIRSENSE", "TRANSPARENT", "ERROR"
    };
    static const char *oxi_names[] = {
        "DISABLED", "SCANNING", "CONNECTING", "BONDING",
        "STREAMING", "DISCONNECTED"
    };

    system_state_t sys = Arbiter::get_state();
    oxi_state_t oxi = BleOxi::get_state();
    const oxi_reading_t &r = BleOxi::get_reading();

    int rop = -1;
    readSetting("ROP", rop);

    Config::refresh_device_info();
    auto &cfg = Config::get();

    String json = "{";
    jsonAddString(json, "version", airbridge_version(), false);
    jsonAddString(json, "built", airbridge_build_date());
    jsonAddString(json, "system", state_names[sys]);
    jsonAddInt(json, "rop", rop);
    jsonAddString(json, "pna", cfg.device_pna.c_str());
    jsonAddString(json, "srn", cfg.device_srn.c_str());
    jsonAddString(json, "oxi", oxi_names[oxi]);
    jsonAddString(json, "feeding", BleOxi::is_feeding() ? "yes" : "no");
    jsonAddInt(json, "spo2", r.valid ? r.spo2 : -1);
    jsonAddInt(json, "pulse", r.valid ? r.pulse_bpm : -1);
    jsonAddInt(json, "heap", ESP.getFreeHeap());
    json += '}';

    request->send(200, "application/json", json);
}


static void emitVar(String &json, const var_def_t *v, const char *group, int raw, bool ok, bool &first) {
    if (!first) json += ',';
    first = false;

    json += '{';
    jsonAddString(json, "cmd", v->cmd, false);
    jsonAddString(json, "label", v->label);
    jsonAddString(json, "group", group);
    jsonAddInt(json, "value", ok ? raw : -1);

    if (v->type == SET_ENUM) {
        jsonAddString(json, "type", "enum");
        if (v->enum_options) {
            json += ",\"options\":[";
            String opts = v->enum_options;
            int start = 0;
            bool firstOpt = true;
            for (int j = 0; j <= (int)opts.length(); j++) {
                if (j == (int)opts.length() || opts[j] == ',') {
                    if (!firstOpt) json += ',';
                    json += '"';
                    json += opts.substring(start, j);
                    json += '"';
                    firstOpt = false;
                    start = j + 1;
                }
            }
            json += ']';
        }
    } else if (v->type == SET_SCALED && v->scale_div > 1) {
        jsonAddString(json, "type", "scaled");
        jsonAddInt(json, "scale_div", v->scale_div);
        jsonAddInt(json, "decimals", v->decimals);
        if (ok) {
            float disp = (float)raw / (float)v->scale_div;
            char buf[16];
            snprintf(buf, sizeof(buf), "%.*f", v->decimals, disp);
            jsonAddString(json, "display", buf);
            float step = 1.0f / (float)v->scale_div;
            snprintf(buf, sizeof(buf), "%g", step);
            jsonAddString(json, "step", buf);
        }
    } else {
        jsonAddString(json, "type", "int");
    }
    json += '}';
}


static void emitVarList(String &json, const char * const *cmds, const char *group, bool &first) {
    for (int i = 0; cmds[i]; i++) {
        const var_def_t *v = var_lookup(cmds[i]);
        if (!v) continue;
        int raw = 0;
        bool ok = readSetting(v->cmd, raw);
        emitVar(json, v, group, raw, ok, first);
    }
}


static void handleGetSettings(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    int mop = 0;
    readSetting("MOP", mop);
    if (mop < 0 || mop >= MODE_COUNT) mop = 0;

    String json = "[";
    bool first = true;

    emitVarList(json, PATIENT_VARS, "patient", first);

    // Mode selector
    {
        const var_def_t *v = var_lookup("MOP");
        if (v) {
            int raw = mop;
            emitVar(json, v, "clinical", raw, true, first);
        }
    }

    // Mode settings
    emitVarList(json, MODE_LAYOUT[mop], "mode", first);
    {
        static const char * const mask_var[] = {"MSK", NULL};
        emitVarList(json, mask_var, "mode", first);
    }

    emitVarList(json, COMFORT_LAYOUT[mop], "comfort", first);
    emitVarList(json, EPR_VARS, "comfort", first);

    emitVarList(json, CLIMATE_VARS, "climate", first);

    emitVarList(json, SYSTEM_VARS, "system", first);

    emitVarList(json, ALARM_VARS, "alarm", first);

    json += ']';
    request->send(200, "application/json", json);
}


static void handlePostSettings(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String body = getBody(request);
    String errors = "";
    int count = 0;

    int pos = 0;
    while (pos < (int)body.length()) {
        int keyStart = body.indexOf('"', pos);
        if (keyStart < 0) break;
        int keyEnd = body.indexOf('"', keyStart + 1);
        if (keyEnd < 0) break;
        String key = body.substring(keyStart + 1, keyEnd);

        int valStart = body.indexOf('"', keyEnd + 1);
        if (valStart < 0) break;
        int valEnd = body.indexOf('"', valStart + 1);
        if (valEnd < 0) break;
        String val = body.substring(valStart + 1, valEnd);
        pos = valEnd + 1;

        const var_def_t *def = var_lookup(key.c_str());

        if (!def) {
            errors += key + ":unknown,";
            continue;
        }

        int raw = atoi(val.c_str());

        if (writeSetting(key.c_str(), raw)) {
            count++;
        } else {
            errors += key + ":fail,";
        }
    }

    String json = "{";
    jsonAddInt(json, "saved", count, false);
    if (errors.length() > 0) {
        errors.remove(errors.length() - 1);  // trailing comma
        json += ",\"errors\":[\"" + errors + "\"]";
    }
    json += '}';
    request->send(200, "application/json", json);
}


static void handleGetConfig(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String json = "{";
    bool first = true;

    auto &cfg = Config::get();
    String dump = Config::dump();
    int start = 0;
    for (int i = 0; i <= (int)dump.length(); i++) {
        if (i == (int)dump.length() || dump[i] == '\n') {
            String line = dump.substring(start, i);
            int eq = line.indexOf('=');
            if (eq > 0) {
                String key = line.substring(0, eq);
                String val = line.substring(eq + 1);
                if (!first) json += ',';
                first = false;
                json += '"';
                json += key;
                json += "\":\"";
                json += val;
                json += '"';
            }
            start = i + 1;
        }
    }
    json += '}';
    request->send(200, "application/json", json);
}


static void handlePostConfig(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String body = getBody(request);
    int count = 0;

    int pos = 0;
    while (pos < (int)body.length()) {
        int keyStart = body.indexOf('"', pos);
        if (keyStart < 0) break;
        int keyEnd = body.indexOf('"', keyStart + 1);
        if (keyEnd < 0) break;
        String key = body.substring(keyStart + 1, keyEnd);

        int valStart = body.indexOf('"', keyEnd + 1);
        if (valStart < 0) break;
        int valEnd = body.indexOf('"', valStart + 1);
        if (valEnd < 0) break;
        String val = body.substring(valStart + 1, valEnd);
        pos = valEnd + 1;

        if (Config::set_value(key.c_str(), val.c_str())) count++;
    }

    if (count > 0) Config::save();

    String json = "{\"ok\":true,\"saved\":";
    json += String(count);
    json += '}';
    request->send(200, "application/json", json);
}


static const esp_partition_t *resmed_part = nullptr;
size_t uploadSize = 0;
static bool uploadOk = false;

static void handleUploadChunk(AsyncWebServerRequest *request, const String& filename,
                               size_t index, uint8_t *data, size_t len, bool final) {
    if (index == 0) {
        Log::logf(CAT_WEB, LOG_INFO, "[WEB] Upload start: %s\n", filename.c_str());
        uploadSize = 0;
        uploadOk = false;

        resmed_part = esp_partition_find_first(
            ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "resmed");
        if (!resmed_part) {
            Log::logf(CAT_WEB, LOG_ERROR, "[WEB] 'resmed' partition not found\n");
            return;
        }
        Log::logf(CAT_WEB, LOG_DEBUG, "[WEB] Partition: offset=0x%X size=%u\n",
                     resmed_part->address, resmed_part->size);

        esp_err_t err = esp_partition_erase_range(resmed_part, 0, resmed_part->size);
        if (err != ESP_OK) {
            Log::logf(CAT_WEB, LOG_ERROR, "[WEB] Erase failed: %s\n", esp_err_to_name(err));
            resmed_part = nullptr;
            return;
        }
        uploadOk = true;
    }

    if (resmed_part && uploadOk && len > 0) {
        if (uploadSize + len > resmed_part->size) {
            Log::logf(CAT_WEB, LOG_ERROR, "[WEB] File too large for partition!\n");
            uploadOk = false;
            return;
        }
        esp_err_t err = esp_partition_write(resmed_part, uploadSize, data, len);
        if (err != ESP_OK) {
            Log::logf(CAT_WEB, LOG_ERROR, "[WEB] Write failed at offset %u: %s\n",
                         uploadSize, esp_err_to_name(err));
            uploadOk = false;
            return;
        }
        uploadSize += len;
    }

    if (final) {
        if (uploadOk) {
            Log::logf(CAT_WEB, LOG_INFO, "[WEB] Upload complete: %u bytes\n", uploadSize);
        }
    }
}

static void handleUploadDone(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String json = "{";
    jsonAddString(json, "ok", uploadOk ? "true" : "false", false);
    jsonAddInt(json, "size", uploadSize);

    if (uploadOk && resmed_part && uploadSize > 0) {
        uint32_t crc = 0;
        uint8_t buf[256];
        size_t offset = 0;
        while (offset < uploadSize) {
            size_t chunk = (uploadSize - offset > sizeof(buf)) ? sizeof(buf) : uploadSize - offset;
            if (esp_partition_read(resmed_part, offset, buf, chunk) == ESP_OK) {
                for (size_t i = 0; i < chunk; i++) crc = (crc << 1) ^ buf[i];
            }
            offset += chunk;
        }
        char hexcrc[12];
        snprintf(hexcrc, sizeof(hexcrc), "%08X", crc);
        jsonAddString(json, "crc", hexcrc);
    }

    json += '}';
    request->send(200, "application/json", json);
}


#define LIVE_BUF_SIZE   64

struct live_sample_t {
    int16_t press;
    int16_t flow;
    int16_t leak;
};

static live_sample_t live_buf[LIVE_BUF_SIZE];
static volatile uint16_t live_head = 0;
static volatile uint16_t live_seq = 0;
static volatile bool live_running = false;
static TaskHandle_t live_task_handle = nullptr;


static int16_t parse_signed_hex(const char *resp, int bits = 16) {
    const char *eq = strstr(resp, "= ");
    if (!eq) return -32768;
    long val = strtol(eq + 2, nullptr, 16);
    long half = 1L << (bits - 1);
    if (val >= half) val -= (half << 1);
    return (int16_t)val;
}

static void live_sampler_task(void *param) {
    TickType_t last_wake = xTaskGetTickCount();
    while (live_running) {
        char resp[64] = {};
        uint16_t resp_len;
        int16_t press = -32768, flow = -32768;

        resp_len = sizeof(resp);
        if (Arbiter::send_cmd("G S #MKP", CMD_SRC_INTERNAL, CMD_PRIO_HIGH,
                               resp, &resp_len, 200)) {
            press = parse_signed_hex(resp);
        }

        resp_len = sizeof(resp);
        memset(resp, 0, sizeof(resp));
        if (Arbiter::send_cmd("G S #RFL", CMD_SRC_INTERNAL, CMD_PRIO_HIGH,
                               resp, &resp_len, 200)) {
            flow = parse_signed_hex(resp, 12);
        }

        uint16_t idx = live_head % LIVE_BUF_SIZE;
        live_buf[idx] = {press, flow, 0};
        live_head++;
        live_seq++;

        // Build JSON and push via SSE
        {
            const oxi_reading_t &r = BleOxi::get_reading();
            String json = "{";
            jsonAddInt(json, "seq", live_seq, false);
            jsonAddInt(json, "press", press);
            jsonAddInt(json, "flow", flow);
            jsonAddInt(json, "spo2", r.valid ? r.spo2 : -1);
            jsonAddInt(json, "pulse", r.valid ? r.pulse_bpm : -1);
            json += '}';
            WebUI::push_event("live", json);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(500));
    }
    live_task_handle = nullptr;
    vTaskDelete(nullptr);
}


static void handleLive(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    if (!live_running && !live_task_handle) {
        live_running = true;
        live_head = 0;
        live_seq = 0;
        xTaskCreatePinnedToCore(live_sampler_task, "live_sample", 4096,
                                nullptr, 2, &live_task_handle, 1);
    }

    uint16_t since = 0;
    if (request->hasArg("since")) {
        since = (uint16_t)request->arg("since").toInt();
    }

    uint16_t cur_seq = live_seq;
    uint16_t available = cur_seq - since;
    if (available > LIVE_BUF_SIZE) available = LIVE_BUF_SIZE;

    const oxi_reading_t &r = BleOxi::get_reading();

    String json = "{";
    jsonAddInt(json, "seq", cur_seq, false);
    jsonAddInt(json, "rate", 2);  // Hz
    jsonAddInt(json, "spo2", r.valid ? r.spo2 : -1);
    jsonAddInt(json, "pulse", r.valid ? r.pulse_bpm : -1);
    json += ",\"samples\":[";

    for (uint16_t i = 0; i < available; i++) {
        uint16_t idx = (cur_seq - available + i) % LIVE_BUF_SIZE;
        if (i > 0) json += ',';
        json += '[';
        json += String(live_buf[idx].press);
        json += ',';
        json += String(live_buf[idx].flow);
        json += ',';
        json += String(live_buf[idx].leak);
        json += ']';
    }

    json += "]}";
    request->send(200, "application/json", json);
}


static void handleLiveControl(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String body = getBody(request);
    if (body.indexOf("stop") >= 0) {
        live_running = false;
        request->send(200, "application/json", "{\"ok\":true,\"running\":false}");
    } else {
        if (!live_running && !live_task_handle) {
            live_running = true;
            live_head = 0;
            live_seq = 0;
            xTaskCreatePinnedToCore(live_sampler_task, "live_sample", 4096,
                                    nullptr, 2, &live_task_handle, 1);
        }
        request->send(200, "application/json", "{\"ok\":true,\"running\":true}");
    }
}


static void handleBleStatus(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    static const char *oxi_names[] = {
        "DISABLED", "SCANNING", "CONNECTING", "BONDING",
        "STREAMING", "DISCONNECTED"
    };

    oxi_state_t st = BleOxi::get_state();
    const oxi_reading_t &r = BleOxi::get_reading();
    auto &cfg = Config::get();

    String json = "{";
    jsonAddString(json, "state", oxi_names[st], false);
    jsonAddString(json, "feeding", BleOxi::is_feeding() ? "yes" : "no");
    jsonAddInt(json, "spo2", r.valid ? r.spo2 : -1);
    jsonAddInt(json, "pulse", r.valid ? r.pulse_bpm : -1);
    jsonAddString(json, "configured_addr", cfg.oxi_device_addr.c_str());
    jsonAddString(json, "auto_start", cfg.oxi_auto_start ? "yes" : "no");


    String results = BleOxi::get_scan_results();
    json += ",\"devices\":[";
    if (results.indexOf("(no oximeters") < 0) {
        bool first = true;
        int start = 0;
        for (int i = 0; i <= (int)results.length(); i++) {
            if (i == (int)results.length() || results[i] == '\n') {
                String line = results.substring(start, i);
                line.trim();
                if (line.length() > 0) {
                    // Format: "AA:BB:CC:DD:EE:FF DeviceName RSSI=-XX"
                    int sp1 = line.indexOf(' ');
                    if (sp1 > 0) {
                        String addr = line.substring(0, sp1);
                        String rest = line.substring(sp1 + 1);
                        int rssiIdx = rest.indexOf("RSSI=");
                        String name = (rssiIdx > 0) ? rest.substring(0, rssiIdx) : rest;
                        String rssi = (rssiIdx >= 0) ? rest.substring(rssiIdx + 5) : "?";
                        name.trim();
                        rssi.trim();

                        if (!first) json += ',';
                        first = false;
                        json += "{";
                        jsonAddString(json, "addr", addr.c_str(), false);
                        jsonAddString(json, "name", name.c_str());
                        jsonAddString(json, "rssi", rssi.c_str());
                        json += "}";
                    }
                }
                start = i + 1;
            }
        }
    }
    json += "],\"bonds\":[";
    int numBonds = NimBLEDevice::getNumBonds();
    for (int i = 0; i < numBonds; i++) {
        if (i > 0) json += ',';
        NimBLEAddress ba = NimBLEDevice::getBondedAddress(i);
        json += '"';
        json += ba.toString().c_str();
        json += '"';
    }
    json += "]}";
    request->send(200, "application/json", json);
}


static void handleBleAction(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String body = getBody(request);
    String action = "";
    String addr = "";


    int pos = 0;
    while (pos < (int)body.length()) {
        int keyStart = body.indexOf('"', pos);
        if (keyStart < 0) break;
        int keyEnd = body.indexOf('"', keyStart + 1);
        if (keyEnd < 0) break;
        String key = body.substring(keyStart + 1, keyEnd);
        int valStart = body.indexOf('"', keyEnd + 1);
        if (valStart < 0) break;
        int valEnd = body.indexOf('"', valStart + 1);
        if (valEnd < 0) break;
        String val = body.substring(valStart + 1, valEnd);
        pos = valEnd + 1;
        if (key == "action") action = val;
        else if (key == "addr") addr = val;
    }

    String result = "unknown action";
    bool ok = false;

    if (action == "scan") {
        BleOxi::start_scan();
        result = "scan started";
        ok = true;
    } else if (action == "stop_scan") {
        BleOxi::stop_scan();
        result = "scan stopped";
        ok = true;
    } else if (action == "connect") {
        if (addr.length() > 0) {
            BleOxi::connect(addr.c_str());
        } else {
            BleOxi::connect(nullptr);
        }
        result = "connecting";
        ok = true;
    } else if (action == "disconnect") {
        BleOxi::disconnect();
        result = "disconnected";
        ok = true;
    } else if (action == "start_feed") {
        BleOxi::start_feed();
        result = "feeding started";
        ok = true;
    } else if (action == "stop_feed") {
        BleOxi::stop_feed();
        result = "feeding stopped";
        ok = true;
    } else if (action == "delete_bond") {
        if (addr.length() > 0) {
            int nb = NimBLEDevice::getNumBonds();
            Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Deleting bond for %s (%d bonds stored)\n", addr.c_str(), nb);
            for (int i = 0; i < nb; i++) {
                NimBLEAddress ba = NimBLEDevice::getBondedAddress(i);
                Log::logf(CAT_BLE, LOG_DEBUG, "[BLE]   bond[%d]: %s type=%d\n", i, ba.toString().c_str(), ba.getType());
            }
            BleOxi::disconnect();
            vTaskDelay(pdMS_TO_TICKS(500));
            bool deleted = false;
            for (int i = 0; i < nb && !deleted; i++) {
                NimBLEAddress ba = NimBLEDevice::getBondedAddress(i);
                if (strcasecmp(ba.toString().c_str(), addr.c_str()) == 0) {
                    int rc = ble_gap_unpair(ba.getBase());
                    deleted = (rc == 0);
                    Log::logf(CAT_BLE, LOG_DEBUG, "[BLE]   match at [%d], unpair rc=%d, type=%d\n", i, rc, ba.getType());
                }
            }
            /*
            if (!deleted) {
                // Fall back to clearing all bonds
                Log::logf(CAT_BLE, LOG_DEBUG, "[BLE]   high-level delete failed, trying ble_store_clear\n");
                ble_store_clear();
                deleted = (NimBLEDevice::getNumBonds() == 0);
            }
            */
            if (deleted && strcasecmp(addr.c_str(), Config::get().oxi_device_addr.c_str()) == 0) {
                Config::set_value("oxi_device_addr", "");
                Config::save();
                Log::logf(CAT_BLE, LOG_INFO, "[BLE] Cleared oxi_device_addr (matched deleted bond)\n");
            }
            result = deleted ? "bond deleted" : "bond not found";
        } else {
            result = "no address specified";
        }
        ok = true;
    } else if (action == "delete_all_bonds") {
        int nb = NimBLEDevice::getNumBonds();
        Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Deleting all %d bonds\n", nb);
        BleOxi::disconnect();
        vTaskDelay(pdMS_TO_TICKS(500));
        NimBLEDevice::deleteAllBonds();
        int remaining = NimBLEDevice::getNumBonds();
        if (remaining > 0) {
            // Fall back to low-level store clear
            Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] High-level delete left %d, trying ble_store_clear\n", remaining);
            ble_store_clear();
            remaining = NimBLEDevice::getNumBonds();
        }
        Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] After delete: %d bonds remain\n", remaining);
        if (remaining == 0 && Config::get().oxi_device_addr.length() > 0) {
            Config::set_value("oxi_device_addr", "");
            Config::save();
            Log::logf(CAT_BLE, LOG_INFO, "[BLE] Cleared oxi_device_addr\n");
        }
        result = remaining == 0 ? "all bonds deleted" : "delete failed";
        ok = true;
    }

    String json = "{";
    jsonAddString(json, "ok", ok ? "true" : "false", false);
    jsonAddString(json, "result", result.c_str());
    json += '}';
    request->send(200, "application/json", json);
}


extern void dispatch_command(const char *line, String &response);

static void handleCmd(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    String body = getBody(request);
    int valStart = body.indexOf("\"cmd\"");
    if (valStart < 0) {
        request->send(400, "application/json", "{\"ok\":false,\"error\":\"missing cmd\"}");
        return;
    }
    int qs = body.indexOf('"', valStart + 5);
    int qe = body.indexOf('"', qs + 1);
    if (qs < 0 || qe < 0) {
        request->send(400, "application/json", "{\"ok\":false,\"error\":\"bad json\"}");
        return;
    }
    String cmd = body.substring(qs + 1, qe);

    String json = "{";

    if (cmd.startsWith("$")) {
        // Internal command
        String response;
        dispatch_command(cmd.c_str() + 1, response);
        response.trim();
        jsonAddString(json, "ok", "true", false);
        jsonAddString(json, "response", response.c_str());
    } else {
        // Q-frame
        char resp[128] = {};
        uint16_t resp_len = sizeof(resp);
        bool ok = Arbiter::send_cmd(cmd.c_str(), CMD_SRC_TCP, CMD_PRIO_NORMAL,
                                     resp, &resp_len, 3000);
        jsonAddString(json, "ok", ok ? "true" : "false", false);
        if (ok && resp_len > 0) {
            jsonAddString(json, "response", resp);
        }
    }

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

    if (uploadSize > 0) {
        const char *detected = ResmedOta::detect_block(uploadSize);
        jsonAddString(json, "detected_block", detected ? detected : "unknown");
        jsonAddInt(json, "fw_size", uploadSize);
    }
    json += '}';
    request->send(200, "application/json", json);
}


static void handleFlashStart(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    if (ResmedOta::is_active()) {
        request->send(409, "application/json", "{\"ok\":false,\"error\":\"flash already active\"}");
        return;
    }

    if (uploadSize == 0) {
        request->send(400, "application/json", "{\"ok\":false,\"error\":\"no firmware uploaded\"}");
        return;
    }

    String body = getBody(request);
    String block = "";
    bool flash_blx = false;
    bool force_blx = false;

    int pos = 0;
    while (pos < (int)body.length()) {
        int keyStart = body.indexOf('"', pos);
        if (keyStart < 0) break;
        int keyEnd = body.indexOf('"', keyStart + 1);
        if (keyEnd < 0) break;
        String key = body.substring(keyStart + 1, keyEnd);

        int valStart = body.indexOf(':', keyEnd);
        if (valStart < 0) break;
        valStart++;
        while (valStart < (int)body.length() && body[valStart] == ' ') valStart++;

        if (key == "force_blx") {
            force_blx = (body.substring(valStart, valStart + 4) == "true");
            pos = valStart + 5;
        } else if (key == "flash_blx") {
            flash_blx = (body.substring(valStart, valStart + 4) == "true");
            pos = valStart + 5;
        } else {
            int qs = body.indexOf('"', valStart);
            int qe = body.indexOf('"', qs + 1);
            if (qs < 0 || qe < 0) break;
            if (key == "block") block = body.substring(qs + 1, qe);
            pos = qe + 1;
        }
    }

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


struct report_var_t {
    const char *cmd;
    const char *label;
    int scale_div;      // 0 = raw int, -60 = raw minutes -> hours
    int decimals;
    const char *unit;
    const char *section;  // "session" or "summary"
};

#define RPT_SESSION "session"
#define RPT_SUMMARY "summary"

static const report_var_t REPORT_VARS[] = {
    // LAST SESSION
    {"UQD", "Usage", -61, 0, "", RPT_SESSION},              {"OND", "Mask On Duration", -61, 0, "", RPT_SESSION},
    {"AQD", "Events/hr", 10, 1, "/hr", RPT_SESSION},        {"MSP", "Median Pressure", 50, 1, "cmH2O", RPT_SESSION},
    {"AIS", "AI (All)", 10, 1, "/hr", RPT_SESSION},          {"PM9", "Pressure P95", 50, 1, "cmH2O", RPT_SESSION},
    {"OPI", "Obstructive AI", 10, 1, "/hr", RPT_SESSION},    {"PMA", "Max Pressure", 50, 1, "cmH2O", RPT_SESSION},
    {"CLI", "Central AI", 10, 1, "/hr", RPT_SESSION},        {"AEP", "Avg EPR Pressure", 50, 1, "cmH2O", RPT_SESSION},
    {"HIS", "Hypopnea Index", 10, 1, "/hr", RPT_SESSION},    {"LKM", "Leak Median", 50, 2, "L/s", RPT_SESSION},
    {"UAI", "Unknown AI", 10, 1, "/hr", RPT_SESSION},        {"LK9", "Leak P95", 50, 2, "L/s", RPT_SESSION},
    {"RIN", "RERA Index", 10, 1, "/hr", RPT_SESSION},

    // SUMMARY
    {"PHM", "Total Used Hrs", 0, 0, "hrs", RPT_SUMMARY},    {"LRD", "Leak", 10, 0, "L/min", RPT_SUMMARY},
    {"DRD", "Days Used", -3, 0, "", RPT_SUMMARY},           {"ZAV", "Vt", 0, 0, "ml", RPT_SUMMARY},
    {"VRD", "Days 4hrs+", -3, 0, "", RPT_SUMMARY},          {"ZAR", "RR", 5, 0, "bpm", RPT_SUMMARY},
    {"WRD", "Avg. Usage", -60, 1, "hrs", RPT_SUMMARY},      {"ZAM", "MV", 8, 1, "L/min", RPT_SUMMARY},
    {"XRD", "Used Hrs", -60, 1, "hrs", RPT_SUMMARY},        {"ZA2", "TgMV", 8, 1, "L/min", RPT_SUMMARY},
    {"ZAI", "Pressure", 50, 1, "cmH2O", RPT_SUMMARY},      {"ZA3", "Va", 8, 1, "L/min", RPT_SUMMARY},
    {"ZAE", "Exp. Pressure", 50, 1, "cmH2O", RPT_SUMMARY}, {"ZAZ", "Ti", 50, 2, "s", RPT_SUMMARY},
    {"ARD", "AHI", 10, 1, "/hr", RPT_SUMMARY},              {"ZA1", "I:E", -1, 0, "", RPT_SUMMARY},
    {"TRD", "Total AI", 10, 1, "/hr", RPT_SUMMARY},         {"ZAS", "Spont Trig", 2, 1, "%", RPT_SUMMARY},
    {"CRD", "Central AI", 10, 1, "/hr", RPT_SUMMARY},       {"ZAY", "Spont Cyc", 2, 1, "%", RPT_SUMMARY},

    {NULL, NULL, 0, 0, NULL, NULL}
};

static String period_label(int raw) {
    switch (raw) {
        case 1:   return "1 Day";
        case 7:   return "1 Week";
        case 30:  return "1 Month";
        case 90:  return "3 Months";
        case 180: return "6 Months";
        case 365: return "1 Year";
        default:  return String(raw) + " days";
    }
}

static void handleReport(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;

    int period = 0;
    readSetting("URD", period);

    String json = "[";
    bool first = true;

    for (const report_var_t *v = REPORT_VARS; v->cmd; v++) {
        int raw = 0;
        bool ok = readSetting(v->cmd, raw);

        if (!first) json += ',';
        first = false;

        json += '{';
        jsonAddString(json, "cmd", v->cmd, false);
        jsonAddString(json, "label", v->label);
        jsonAddInt(json, "raw", ok ? raw : -1);

        char buf[32];
        if (ok && v->scale_div == -61) {
            // Raw minutes -> H:MM
            snprintf(buf, sizeof(buf), "%d:%02d", raw / 60, raw % 60);
            jsonAddString(json, "value", buf);
        } else if (ok && v->scale_div == -60) {
            // Raw minutes -> decimal hours (round properly)
            double hrs = (double)raw / 60.0;
            snprintf(buf, sizeof(buf), "%.*f", v->decimals, hrs);
            jsonAddString(json, "value", buf);
        } else if (ok && v->scale_div == -3) {
            // N/period format (e.g. "6/7")
            snprintf(buf, sizeof(buf), "%d/%d", raw, period);
            jsonAddString(json, "value", buf);
        } else if (ok && v->scale_div == -2) {
            // Period enum
            jsonAddString(json, "value", period_label(raw).c_str());
        } else if (ok && v->scale_div == -1) {
            // I:E ratio: raw/100
            if (raw > 0) {
                float ratio = (float)raw / 100.0f;
                if (ratio >= 1.0f) {
                    snprintf(buf, sizeof(buf), "%.1f:1", ratio);
                } else {
                    snprintf(buf, sizeof(buf), "1:%.1f", 1.0f / ratio);
                }
            } else {
                snprintf(buf, sizeof(buf), "--");
            }
            jsonAddString(json, "value", buf);
        } else if (ok && v->scale_div > 0) {
            double disp = (double)raw / (double)v->scale_div;
            snprintf(buf, sizeof(buf), "%.*f", v->decimals, disp);
            jsonAddString(json, "value", buf);
        } else {
            jsonAddString(json, "value", ok ? String(raw).c_str() : "--");
        }
        jsonAddString(json, "unit", v->unit);
        jsonAddString(json, "section", v->section);
        json += '}';
    }

    json += ']';
    request->send(200, "application/json", json);
}

static void handleReboot(AsyncWebServerRequest *request) {
    if (!checkAuth(request)) return;
    request->send(200, "application/json", "{\"ok\":true}");
    delay(100);
    ESP.restart();
}


void WebUI::push_event(const char *event, const char *json) {
    if (events) events->send(json, event, millis());
}

void WebUI::push_event(const char *event, const String &json) {
    if (events) events->send(json.c_str(), event, millis());
}

void WebUI::init(uint16_t port) {
    if (port == 0) return;

    http = new AsyncWebServer(port);
    events = new AsyncEventSource("/events");
    http->addHandler(events);

    http->on("/", HTTP_GET, handleRoot);
    http->on("/api/status", HTTP_GET, handleStatus);
    http->on("/api/settings", HTTP_GET, handleGetSettings);
    http->on("/api/settings", HTTP_POST, handlePostSettings, NULL, handleJsonBody);
    http->on("/api/config", HTTP_GET, handleGetConfig);
    http->on("/api/config", HTTP_POST, handlePostConfig, NULL, handleJsonBody);
    http->on("/api/live", HTTP_GET, handleLive);
    http->on("/api/live", HTTP_POST, handleLiveControl, NULL, handleJsonBody);
    http->on("/api/upload", HTTP_POST, handleUploadDone, handleUploadChunk);
    http->on("/api/ble", HTTP_GET, handleBleStatus);
    http->on("/api/ble", HTTP_POST, handleBleAction, NULL, handleJsonBody);
    http->on("/api/cmd", HTTP_POST, handleCmd, NULL, handleJsonBody);
    http->on("/api/flash", HTTP_GET, handleFlashStatus);
    http->on("/api/flash", HTTP_POST, handleFlashStart, NULL, handleJsonBody);
    http->on("/api/flash/cancel", HTTP_POST, handleFlashCancel);
    http->on("/api/report", HTTP_GET, handleReport);
    http->on("/api/reboot", HTTP_POST, handleReboot);

    DefaultHeaders::Instance().addHeader("Cache-Control", "no-store");

    http->begin();
    Log::logf(CAT_WEB, LOG_INFO, "[WEB] HTTP server on port %d\n", port);
}

static uint32_t last_status_push = 0;

void WebUI::handle() {
    // Push immediately on BLE state change, or every 3s
    bool ble_changed = BleOxi::state_changed();
    if (events && events->count() > 0 && (ble_changed || millis() - last_status_push >= 3000)) {
        last_status_push = millis();

        static const char *sys_names[] = {"IDLE","THERAPY","BOOTLOADER","OTA_ESP","OTA_AIRSENSE","TRANSPARENT","ERROR"};
        static const char *oxi_names[] = {"DISABLED","SCANNING","CONNECTING","BONDING","STREAMING","DISCONNECTED"};

        system_state_t sys = Arbiter::get_state();
        oxi_state_t oxi = BleOxi::get_state();
        const oxi_reading_t &r = BleOxi::get_reading();

        String sj = "{";
        jsonAddString(sj, "system", sys_names[sys], false);
        jsonAddString(sj, "oxi", oxi_names[oxi]);
        jsonAddString(sj, "feeding", BleOxi::is_feeding() ? "yes" : "no");
        jsonAddInt(sj, "spo2", r.valid ? r.spo2 : -1);
        jsonAddInt(sj, "pulse", r.valid ? r.pulse_bpm : -1);
        jsonAddInt(sj, "heap", ESP.getFreeHeap());
        sj += '}';
        events->send(sj.c_str(), "status", millis());
    }
}
