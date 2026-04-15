#include <Arduino.h>
#include "app_config.h"
#include "debug_log.h"
#include "uart_arbiter.h"
#include "tcp_bridge.h"
#include "wifi.h"
#include "oxi_ble.h"
#include "oxi_udp.h"
#include "oxi_arbiter.h"
#include "airbridge_ota.h"
#include "web_ui.h"
#include "qframe.h"

const char *airbridge_version() { return AIRBRIDGE_VERSION; }
const char *airbridge_build_date() { return AIRBRIDGE_BUILD_DATE; }

extern void dispatch_command(const char *line, String &response);

#define SERIAL_LINE_MAX 256
static char serial_line[SERIAL_LINE_MAX];
static int serial_pos = 0;

static void serial_poll() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (serial_pos > 0) {
                serial_line[serial_pos] = '\0';

                if (serial_line[0] == '$') {
                    // Internal command
                    String response;
                    dispatch_command(serial_line + 1, response);
                    if (response.length() > 0) Serial.print(response);
                } else {
                    // Q-frame
                    char resp_buf[512] = {};
                    uint16_t resp_len = sizeof(resp_buf);
                    bool ok = Arbiter::send_cmd(serial_line, CMD_SRC_INTERNAL,
                                                CMD_PRIO_NORMAL, resp_buf,
                                                &resp_len, 2000);
                    if (ok) {
                        Serial.println(resp_buf);
                    } else {
                        Serial.println("ERR:TIMEOUT");
                    }
                }
                serial_pos = 0;
            }
        } else if (serial_pos < SERIAL_LINE_MAX - 1) {
            serial_line[serial_pos++] = c;
        }
    }
}

// Health monitoring: poll ROP every 10s to detect therapy state
#define HEALTH_POLL_INTERVAL_MS     10000
#define HEALTH_TIMEOUT_MS           500

static uint32_t last_health_poll = 0;
static uint32_t consecutive_timeouts = 0;

static void poll_therapy_state() {
    char resp[64] = {};
    uint16_t resp_len = sizeof(resp);

    uint32_t t0 = millis();
    Log::logf(CAT_HEALTH, LOG_DEBUG, "[HEALTH] ROP poll start t=%lu\n", t0);
    bool ok = Arbiter::send_cmd("G S #ROP", CMD_SRC_INTERNAL, CMD_PRIO_HIGH,
                                resp, &resp_len, HEALTH_TIMEOUT_MS);
    if (ok) {
        consecutive_timeouts = 0;

        const char *rv = qframe_response_value(resp);
        if (rv) {
            uint32_t val = strtoul(rv, nullptr, 16);
            system_state_t current = Arbiter::get_state();

            if (val == 1 && current == SYS_IDLE) {
                Arbiter::set_state(SYS_THERAPY);
                Log::logf(CAT_HEALTH, LOG_INFO, "[HEALTH] Therapy started\n");
            } else if (val == 0 && current == SYS_THERAPY) {
                Arbiter::set_state(SYS_IDLE);
                Log::logf(CAT_HEALTH, LOG_INFO, "[HEALTH] Therapy ended\n");
            }

            if (Arbiter::get_state() != current) {
                int mhr = -1;
                char mhr_resp[32] = {};
                uint16_t mhr_len = sizeof(mhr_resp);
                if (Arbiter::send_cmd("G S #MHR", CMD_SRC_INTERNAL, CMD_PRIO_NORMAL,
                                      mhr_resp, &mhr_len)) {
                    const char *mv = qframe_response_value(mhr_resp);
                    if (mv) mhr = (int)strtol(mv, nullptr, 16);
                }
                char buf[64];
                snprintf(buf, sizeof(buf), "{\"system\":\"%s\",\"mhr\":%d}",
                         system_state_name(Arbiter::get_state()), mhr);
                WebUI::push_event("status", buf);
            }
        }
    } else {
        consecutive_timeouts++;
        Log::logf(CAT_HEALTH, consecutive_timeouts >= 2 ? LOG_WARN : LOG_DEBUG,
                  "[HEALTH] ROP poll timeout (%d consecutive) t=%lu dt=%lu\n",
                  consecutive_timeouts, millis(), millis() - t0);

        if (consecutive_timeouts >= 3) {
            system_state_t current = Arbiter::get_state();
            if (current != SYS_ERROR && current != SYS_TRANSPARENT &&
                current != SYS_OTA_AIRSENSE && current != SYS_OTA_ESP) {
                Arbiter::set_state(SYS_ERROR);
                Log::logf(CAT_HEALTH, LOG_ERROR, "[HEALTH] UART unresponsive, entering ERROR state\n");
            }
        }
    }
}

static void attempt_recovery() {
    if (Arbiter::get_state() != SYS_ERROR) return;

    char resp[32] = {};
    uint16_t resp_len = sizeof(resp);

    bool ok = Arbiter::send_cmd("G S #BLS", CMD_SRC_INTERNAL, CMD_PRIO_HIGH,
                                resp, &resp_len);
    if (ok) {
        Log::logf(CAT_HEALTH, LOG_INFO, "[HEALTH] Device responded, clearing error\n");
        consecutive_timeouts = 0;
        Arbiter::set_state(SYS_IDLE);
        Config::invalidate_device_info();
    }
}

bool pull_time_from_resmed();

void setup() {
    Serial.begin(115200);
    delay(500);
    while (Serial.available()) Serial.read();  // flush boot garbage
    Log::init();

    Log::printf("\n=== AirBridge " AIRBRIDGE_VERSION " ===\n");
    Log::printf("Chip: %s, Heap: %d bytes\n", ESP.getChipModel(), ESP.getFreeHeap());

    Config::init();
    Config::load();
    Log::logf(CAT_INIT, LOG_INFO, "[INIT] Config loaded\n");

    Arbiter::init(Serial1, PIN_AS10_RX, PIN_AS10_TX, Config::get().uart_baud);
    Log::logf(CAT_INIT, LOG_INFO, "[INIT] UART arbiter started\n");

    bool wifi_ok = WiFiSetup::init();

    TcpBridge::init();
    Log::logf(CAT_INIT, LOG_INFO, "[INIT] TCP bridge started\n");

    if (wifi_ok) OtaManager::init();

    // If NTP didn't sync, fall back to resmed device clock
    if (!WiFiSetup::time_synced()) pull_time_from_resmed();

    OxiArbiter::init();
    OxiBle::init();
    OxiUdp::init();
    Log::logf(CAT_INIT, LOG_INFO, "[INIT] BLE oximetry started\n");

    Log::logf(CAT_INIT, LOG_INFO, "[INIT] All systems go\n");
}

static bool resmed_time_set = false;

void reset_resmed_time_sync() { resmed_time_set = false; }

bool push_time_to_resmed() {
    if (!WiFiSetup::time_synced()) return false;

    struct tm t;
    time_t now = time(nullptr);
    localtime_r(&now, &t);

    char dac_cmd[32], tic_cmd[32];
    snprintf(dac_cmd, sizeof(dac_cmd), "P S #DAC %02d%02d%04d",
             t.tm_mday, t.tm_mon + 1, t.tm_year + 1900);
    snprintf(tic_cmd, sizeof(tic_cmd), "P S #TIC %02d%02d%02d",
             t.tm_hour, t.tm_min, t.tm_sec);

    char resp[16] = {};
    uint16_t resp_len = sizeof(resp);
    bool ok_dac = Arbiter::send_cmd(dac_cmd, CMD_SRC_INTERNAL, CMD_PRIO_NORMAL, resp, &resp_len);
    resp_len = sizeof(resp);
    bool ok_tic = Arbiter::send_cmd(tic_cmd, CMD_SRC_INTERNAL, CMD_PRIO_NORMAL, resp, &resp_len);

    if (ok_dac && ok_tic) {
        Log::logf(CAT_INIT, LOG_INFO, "[INIT] ResMed clock set: %02d%02d%04d %02d%02d%02d\n",
                  t.tm_mday, t.tm_mon + 1, t.tm_year + 1900,
                  t.tm_hour, t.tm_min, t.tm_sec);
    } else {
        Log::logf(CAT_INIT, LOG_WARN, "[INIT] ResMed clock set failed (dac=%d tic=%d)\n",
                  ok_dac, ok_tic);
    }
    return ok_dac && ok_tic;
}

bool pull_time_from_resmed() {
    char dac_resp[32] = {}, tic_resp[32] = {};
    uint16_t dac_len = sizeof(dac_resp), tic_len = sizeof(tic_resp);
    Arbiter::send_cmd("G S #DAC", CMD_SRC_INTERNAL, CMD_PRIO_NORMAL, dac_resp, &dac_len);
    Arbiter::send_cmd("G S #TIC", CMD_SRC_INTERNAL, CMD_PRIO_NORMAL, tic_resp, &tic_len);
    const char *dv = qframe_response_value(dac_resp);
    const char *tv = qframe_response_value(tic_resp);
    if (dv && tv && strlen(dv) >= 8 && strlen(tv) >= 6) {
        int dd, mm, yyyy, hh, mn, ss;
        if (sscanf(dv, "%2d%2d%4d", &dd, &mm, &yyyy) == 3 &&
            sscanf(tv, "%2d%2d%2d", &hh, &mn, &ss) == 3) {
            WiFiSetup::set_fallback_time(yyyy, mm, dd, hh, mn, ss, true);
            Log::logf(CAT_INIT, LOG_INFO, "[INIT] Time from ResMed: %04d-%02d-%02d %02d:%02d:%02d\n",
                      yyyy, mm, dd, hh, mn, ss);
            return true;
        }
    }
    return false;
}

static void sync_resmed_clock() {
    if (resmed_time_set) return;
    system_state_t st = Arbiter::get_state();
    if (st != SYS_IDLE && st != SYS_THERAPY) return;
    if (push_time_to_resmed()) resmed_time_set = true;
}

void loop() {
    serial_poll();

    OtaManager::handle();
    WiFiSetup::check();

    // suspend WiFi scanning during therapy/streaming/oximetry
    system_state_t wifi_st = Arbiter::get_state();
    bool oxi_active = OxiArbiter::is_feeding();
    if (wifi_st == SYS_THERAPY || wifi_st == SYS_TRANSPARENT ||
        wifi_st == SYS_OTA_AIRSENSE || wifi_st == SYS_OTA_ESP || oxi_active) {
        WiFiSetup::suspend_roaming();
    } else {
        WiFiSetup::resume_roaming();
    }

    sync_resmed_clock();

    // health monitoring
    if (millis() - last_health_poll >= HEALTH_POLL_INTERVAL_MS) {
        last_health_poll = millis();

        system_state_t st = Arbiter::get_state();
        if (st == SYS_IDLE || st == SYS_THERAPY) {
            poll_therapy_state();
        } else if (st == SYS_ERROR) {
            attempt_recovery();
        }
    }

    delay(10);
}
