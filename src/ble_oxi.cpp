#include "ble_oxi.h"
#include "wifi.h"
#include "uart_arbiter.h"
#include "debug_log.h"
#include "app_config.h"
#include "qframe.h"

#include <NimBLEDevice.h>

#define OXI_TASK_STACK      4096
#define OXI_TASK_PRIO       4
#define SCAN_DURATION_MS    10000
#define RECONNECT_DELAY_MS  3000
#define MAX_SCAN_RESULTS    8

static NimBLEUUID PLX_SERVICE_UUID((uint16_t)0x1822);
static NimBLEUUID PLX_CONTINUOUS_UUID((uint16_t)0x2A5F);
static NimBLEUUID PLX_SPOT_UUID((uint16_t)0x2A5E);
static NimBLEUUID HR_SERVICE_UUID((uint16_t)0x180D);
static NimBLEUUID HR_MEASUREMENT_UUID((uint16_t)0x2A37);

// Nonin proprietary
static NimBLEUUID NONIN_OXI_SERVICE_UUID("46A970E0-0D5F-11E2-8B5E-0002A5D5C51B");
static NimBLEUUID NONIN_CONTINUOUS_UUID("0AAD7EA0-0D60-11E2-8E3C-0002A5D5C51B");
static NimBLEUUID NONIN_CONTROL_POINT_UUID("1447AF80-0D60-11E2-88B6-0002A5D5C51B");

static TaskHandle_t oxi_task_handle = nullptr;
static volatile oxi_state_t state = OXI_DISABLED;
static volatile bool state_dirty = false;
static inline void set_state(oxi_state_t s) { state = s; state_dirty = true; }
static oxi_reading_t reading = { -1, -1, false, 0 };
static volatile bool feeding = false;
static volatile bool scan_requested = false;
static volatile bool connect_requested = false;
static volatile bool disconnect_requested = false;
static volatile bool scan_complete = false;
static String target_addr = "";

static NimBLEClient *pClient = nullptr;

struct ScanResult {
    String addr;
    String name;
    int rssi;
    uint8_t addr_type;
};
static ScanResult scan_results[MAX_SCAN_RESULTS];
static int scan_result_count = 0;


static float parse_sfloat(uint16_t raw) {
    int16_t mantissa = raw & 0x0FFF;
    if (mantissa & 0x0800) mantissa |= 0xF000;
    int8_t exponent = (int8_t)((raw >> 12) & 0x0F);
    if (exponent & 0x08) exponent |= 0xF0;
    return (float)mantissa * powf(10.0f, (float)exponent);
}

static void plx_notify_cb(NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len, bool isNotify) {
    if (len < 5) return;
    uint16_t spo2_raw = data[1] | (data[2] << 8);
    uint16_t pr_raw = data[3] | (data[4] << 8);
    float spo2 = parse_sfloat(spo2_raw);
    float pr = parse_sfloat(pr_raw);
    if (spo2 > 0 && spo2 <= 100 && pr > 0 && pr < 500) {
        reading.spo2 = (int8_t)roundf(spo2);
        reading.pulse_bpm = (int16_t)roundf(pr);
        reading.valid = true;
        reading.timestamp_ms = millis();
    } else {
        reading.spo2 = -1;
        reading.pulse_bpm = -1;
        reading.valid = false;
    }
}

static void hr_notify_cb(NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len, bool isNotify) {
    if (len < 2) return;
    uint8_t flags = data[0];
    uint16_t hr;
    if (flags & 0x01) {
        if (len < 3) return;
        hr = data[1] | (data[2] << 8);
    } else {
        hr = data[1];
    }
    if (hr > 0 && hr < 500) {
        reading.pulse_bpm = (int16_t)hr;
        reading.timestamp_ms = millis();
    } else {
        reading.pulse_bpm = -1;
    }
}

static void nonin_notify_cb(NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len, bool isNotify) {
    if (len >= 5) {
        uint8_t spo2 = data[2];
        uint16_t pr = data[3] | (data[4] << 8);
        if (spo2 > 0 && spo2 <= 100 && pr > 0 && pr < 500) {
            reading.spo2 = (int8_t)spo2;
            reading.pulse_bpm = (int16_t)pr;
            reading.valid = true;
            reading.timestamp_ms = millis();
        } else {
            reading.spo2 = -1;
            reading.pulse_bpm = -1;
            reading.valid = false;
        }
    }
}


class OxiScanCB : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *dev) override {
        String name = dev->getName().c_str();
        bool is_oxi = dev->isAdvertisingService(PLX_SERVICE_UUID) ||
                       dev->isAdvertisingService(NONIN_OXI_SERVICE_UUID) ||
                       dev->isAdvertisingService(HR_SERVICE_UUID) ||
                       name.startsWith("Nonin") ||
                       name.startsWith("O2Ring") ||
                       name.startsWith("CheckMe");

        if (is_oxi && scan_result_count < MAX_SCAN_RESULTS) {
            scan_results[scan_result_count].addr = dev->getAddress().toString().c_str();
            scan_results[scan_result_count].name = name;
            scan_results[scan_result_count].rssi = dev->getRSSI();
            scan_results[scan_result_count].addr_type = dev->getAddress().getType();
            scan_result_count++;
            Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Found: %s (%s) RSSI=%d\n",
                          name.c_str(), dev->getAddress().toString().c_str(), dev->getRSSI());
        }
    }

    void onScanEnd(const NimBLEScanResults &results, int reason) override {
        Log::logf(CAT_BLE, LOG_INFO, "[BLE] Scan complete, %d oximeters found (reason=%d)\n", scan_result_count, reason);
        scan_complete = true;
    }
};

static OxiScanCB scanCB;

class OxiClientCB : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient *client) override {
        Log::logf(CAT_BLE, LOG_INFO, "[BLE] Connected\n");
    }

    void onDisconnect(NimBLEClient *client, int reason) override {
        Log::logf(CAT_BLE, LOG_INFO, "[BLE] Disconnected (reason=0x%X)\n", reason);
        reading.spo2 = -1;
        reading.pulse_bpm = -1;
        reading.valid = false;
        feeding = false;
        if (state == OXI_STREAMING || state == OXI_BONDING) {
            set_state(OXI_DISCONNECTED);
        }
    }

    bool onConnParamsUpdateRequest(NimBLEClient *client, const ble_gap_upd_params *params) override {
        return true;
    }

    void onPassKeyEntry(NimBLEConnInfo &connInfo) override {
        // "Just Works"
        NimBLEDevice::injectPassKey(connInfo, 0);
    }

    void onConfirmPasskey(NimBLEConnInfo &connInfo, uint32_t pin) override {
        NimBLEDevice::injectConfirmPasskey(connInfo, true);
    }

    void onAuthenticationComplete(NimBLEConnInfo &connInfo) override {
        if (connInfo.isEncrypted()) {
            Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Encrypted + bonded\n");
        } else {
            Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Auth complete (no encryption)\n");
        }
    }
};

static OxiClientCB clientCB;


static bool subscribe_services(NimBLEClient *cl) {
    bool got_spo2 = false;
    bool got_hr = false;

    NimBLERemoteService *plxSvc = cl->getService(PLX_SERVICE_UUID);
    if (plxSvc) {
        NimBLERemoteCharacteristic *plxCont = plxSvc->getCharacteristic(PLX_CONTINUOUS_UUID);
        if (plxCont && plxCont->canNotify()) {
            plxCont->subscribe(true, plx_notify_cb);
            Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Subscribed PLX Continuous\n");
            got_spo2 = got_hr = true;
        }
        if (!got_spo2) {
            NimBLERemoteCharacteristic *plxSpot = plxSvc->getCharacteristic(PLX_SPOT_UUID);
            if (plxSpot && plxSpot->canIndicate()) {
                plxSpot->subscribe(false, plx_notify_cb);
                Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Subscribed PLX Spot\n");
                got_spo2 = got_hr = true;
            }
        }
    }

    if (!got_spo2) {
        NimBLERemoteService *noninSvc = cl->getService(NONIN_OXI_SERVICE_UUID);
        if (noninSvc) {
            NimBLERemoteCharacteristic *noninCont = noninSvc->getCharacteristic(NONIN_CONTINUOUS_UUID);
            if (noninCont && noninCont->canNotify()) {
                noninCont->subscribe(true, nonin_notify_cb);
                Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Subscribed Nonin Continuous\n");
                got_spo2 = got_hr = true;
            }
        }
    }

    if (!got_hr) {
        NimBLERemoteService *hrSvc = cl->getService(HR_SERVICE_UUID);
        if (hrSvc) {
            NimBLERemoteCharacteristic *hrMeas = hrSvc->getCharacteristic(HR_MEASUREMENT_UUID);
            if (hrMeas && hrMeas->canNotify()) {
                hrMeas->subscribe(true, hr_notify_cb);
                Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Subscribed Heart Rate\n");
                got_hr = true;
            }
        }
    }

    return got_spo2 || got_hr;
}

// Set date/time on Nonin devices so stored records have correct timestamps.
static void set_nonin_datetime(NimBLEClient *cl) {
    if (!WiFiSetup::time_synced()) {
        Log::logf(CAT_BLE, LOG_WARN, "[BLE] Skipping Nonin datetime — NTP not synced\n");
        return;
    }

    NimBLERemoteService *svc = cl->getService(NONIN_OXI_SERVICE_UUID);
    if (!svc) return;

    NimBLERemoteCharacteristic *cp = svc->getCharacteristic(NONIN_CONTROL_POINT_UUID);
    if (!cp || !cp->canWrite()) {
        Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Nonin control point not writable\n");
        return;
    }

    struct tm timeinfo;
    time_t now = time(nullptr);
    localtime_r(&now, &timeinfo);

    char ts[13];
    strftime(ts, sizeof(ts), "%y%m%d%H%M%S", &timeinfo);

    uint8_t cmd[] = {0x60, 0x4E, 0x4D, 0x49, 0x12, 0x44, 0x54, 0x4D, 0x3D,
                     0,0,0,0,0,0,0,0,0,0,0,0, 0x0D, 0x0A};
    memcpy(cmd + 9, ts, 12);

    if (cp->writeValue(cmd, sizeof(cmd), true)) {
        Log::logf(CAT_BLE, LOG_INFO, "[BLE] Nonin datetime set: %s\n", ts);
    } else {
        Log::logf(CAT_BLE, LOG_WARN, "[BLE] Nonin datetime write failed\n");
    }
}


// Persistent L-frame state: sequence counter and alive toggle bit
static uint8_t oxh_seq    = 0;
static uint8_t oxh_toggle = 0;
static bool    last_valid  = false;

static void inject_reading() {
    system_state_t st = Arbiter::get_state();
    if (st == SYS_TRANSPARENT || st == SYS_OTA_AIRSENSE || st == SYS_OTA_ESP) return;

    auto &cfg = Config::get();
    if (cfg.oxi_feed_therapy_only && st != SYS_THERAPY) return;
    if (!cfg.oxi_lframe_continuous && !reading.valid) return;

    // Log finger placed / lost transitions
    if (reading.valid != last_valid) {
        if (reading.valid)
            Log::logf(CAT_BLE, LOG_INFO, "[BLE] Finger detected: SpO2=%d%% HR=%d bpm\n",
                      reading.spo2, reading.pulse_bpm);
        else
            Log::logf(CAT_BLE, LOG_INFO, "[BLE] Finger lost\n");
        last_valid = reading.valid;
    }

    // Alive toggle: bit 1 (0x02), alternates every frame, synchronized between OXS and SAS
    uint8_t toggle = oxh_toggle ? 0x02 : 0x00;
    oxh_toggle ^= 1;

    uint8_t  oxs, sas;
    uint16_t hrr;
    uint8_t  sar;

    if (reading.valid) {
        // Finger present: OXS bit7=1, bit0=1; SAS bit7=1
        oxs = 0x81 | toggle;   // 0x81 / 0x83
        sas = 0x80 | toggle;   // 0x80 / 0x82
        hrr = (uint16_t)reading.pulse_bpm;
        sar = (uint8_t)reading.spo2;
    } else {
        // No finger / no signal: OXS bit7=1, bit4=1, bit3=1, bit0=1; SAS bit7=1, bit4=1, bit3=1
        oxs = 0x99 | toggle;   // 0x99 / 0x9B
        sas = 0x98 | toggle;   // 0x98 / 0x9A
        hrr = 0x1FF;
        sar = 0x7F;
    }

    // Payload: "OXH" + seq(2) + oxs(2) + hrr(3) + sas(2) + sar(2) + "10" = 16 chars
    char payload[17];
    snprintf(payload, sizeof(payload), "OXH%02X%02X%03X%02X%02X10",
             oxh_seq, oxs, hrr, sas, sar);
    oxh_seq = (oxh_seq + 1) & 0xFF;

    Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] L-frame seq=%02X %s SpO2=%d HR=%d t=%lu\n",
              (oxh_seq - 1) & 0xFF, reading.valid ? "valid" : "no-finger",
              reading.spo2, reading.pulse_bpm, millis());

    uint8_t frame_buf[32];
    int frame_len = qframe_build('L', (const uint8_t *)payload, 16,
                                 frame_buf, sizeof(frame_buf));
    if (frame_len < 0) return;

    Arbiter::send_frame(frame_buf, (uint16_t)frame_len, CMD_SRC_OXI, CMD_PRIO_LOW);
}


void BleOxi::task(void *param) {
    NimBLEDevice::init(Config::get().hostname.c_str());
    NimBLEDevice::setSecurityAuth(true, true, true);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(&clientCB);
    pClient->setConnectionParams(12, 12, 0, 400);

    auto &cfg = Config::get();
    if (cfg.oxi_enabled) {
        set_state(OXI_DISCONNECTED);
        scan_requested = true;
    }

    uint32_t last_inject = 0;
    uint32_t last_reconnect = 0;

    while (true) {
        if (disconnect_requested) {
            disconnect_requested = false;
            Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Disconnect requested\n");
            if (pClient->isConnected()) pClient->disconnect();
            feeding = false;
            set_state(OXI_DISABLED);
        }

        if (scan_complete) {
            scan_complete = false;
            if (state == OXI_SCANNING) {
                Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Scan done, %d results\n", scan_result_count);
                set_state(OXI_DISCONNECTED);
                if (scan_result_count > 0) {
                    String target = cfg.oxi_device_addr;
                    bool found = false;
                    if (target.length() > 0) {
                        for (int i = 0; i < scan_result_count; i++) {
                            if (scan_results[i].addr.equalsIgnoreCase(target)) {
                                found = true;
                                Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Target %s found in scan\n", target.c_str());
                                break;
                            }
                        }
                        if (!found) Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Target %s not in scan results\n", target.c_str());
                    } else {
                        for (int i = 0; i < scan_result_count; i++) {
                            if (scan_results[i].name.startsWith("Nonin")) {
                                NimBLEAddress ba(std::string(scan_results[i].addr.c_str()), scan_results[i].addr_type);
                                bool bonded = NimBLEDevice::isBonded(ba);
                                Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] %s %s bonded=%d\n",
                                          scan_results[i].name.c_str(), scan_results[i].addr.c_str(), bonded);
                                if (bonded) { found = true; break; }
                            } else {
                                found = true;
                                break;
                            }
                        }
                    }
                    if (found) {
                        connect_requested = true;
                        Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Auto-connect triggered\n");
                    }
                }
            }
        }

        if (scan_requested) {
            scan_requested = false;
            scan_result_count = 0;
            scan_complete = false;
            set_state(OXI_SCANNING);
            Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Starting scan (%dms)\n", SCAN_DURATION_MS);
            NimBLEScan *pScan = NimBLEDevice::getScan();
            pScan->setScanCallbacks(&scanCB);
            pScan->setActiveScan(true);
            pScan->setInterval(100);
            pScan->setWindow(99);
            pScan->start(SCAN_DURATION_MS);
        }

        if (connect_requested) {
            connect_requested = false;
            NimBLEDevice::getScan()->stop();

            String addr = target_addr;
            if (addr.length() == 0 && cfg.oxi_device_addr.length() > 0)
                addr = cfg.oxi_device_addr;
            if (addr.length() == 0 && scan_result_count > 0)
                addr = scan_results[0].addr;

            if (addr.length() > 0) {
                set_state(OXI_CONNECTING);

                uint8_t atype = 1;
                for (int i = 0; i < scan_result_count; i++) {
                    if (scan_results[i].addr.equalsIgnoreCase(addr)) {
                        atype = scan_results[i].addr_type;
                        break;
                    }
                }

                // cancel any pending connection and clean up stale state
                if (pClient->isConnected()) {
                    Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Disconnecting stale connection\n");
                    pClient->disconnect();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                } else {
                    // cancel pending connect that hasn't completed yet (EALREADY)
                    pClient->cancelConnect();
                    vTaskDelay(pdMS_TO_TICKS(500));
                }

                Log::logf(CAT_BLE, LOG_INFO, "[BLE] Connecting to %s (type=%d)...\n", addr.c_str(), atype);

                NimBLEAddress bleAddr(std::string(addr.c_str()), atype);
                bool ok = pClient->connect(bleAddr);

                if (!ok) {
                    int err = pClient->getLastError();
                    Log::logf(CAT_BLE, LOG_WARN, "[BLE] connect() returned false (err=%d)\n", err);

                    // EALREADY: previous connect still in flight.
                    // Wait and check if it actually connected.
                    if (err == 2) {
                        Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Waiting for pending connect...\n");
                        for (int i = 0; i < 50 && !pClient->isConnected(); i++)
                            vTaskDelay(pdMS_TO_TICKS(200));
                        ok = pClient->isConnected();
                        Log::logf(CAT_BLE, LOG_INFO, "[BLE] Pending connect %s\n", ok ? "succeeded" : "failed");
                    }

                    // EDONE: stale bond. Delete and retry.
                    if (!ok && err == 13) {
                        Log::logf(CAT_BLE, LOG_INFO, "[BLE] Removing bond and retrying\n");
                        NimBLEDevice::deleteBond(bleAddr);
                        vTaskDelay(pdMS_TO_TICKS(500));
                        ok = pClient->connect(bleAddr);
                        if (!ok) {
                            Log::logf(CAT_BLE, LOG_WARN, "[BLE] Retry failed (err=%d)\n",
                                      pClient->getLastError());
                        }
                    }
                }

                if (ok) {
                    Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Connected, waiting for encryption\n");
                    set_state(OXI_BONDING);

                    for (int i = 0; i < 30 && pClient->isConnected(); i++) {
                        if (pClient->secureConnection()) break;
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                    Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Encryption: secure=%d connected=%d\n",
                              pClient->secureConnection(), pClient->isConnected());

                    if (subscribe_services(pClient)) {
                        set_nonin_datetime(pClient);
                        set_state(OXI_STREAMING);
                        Log::logf(CAT_BLE, LOG_INFO, "[BLE] Streaming started\n");
                        Arbiter::lcd_message("Oximeter Connected", 15000);
                        if (cfg.oxi_auto_start) {
                            feeding = true;
                            Log::logf(CAT_BLE, LOG_INFO, "[BLE] Feeding started (auto)\n");
                        }
                    } else {
                        Log::logf(CAT_BLE, LOG_WARN, "[BLE] No suitable services, disconnecting\n");
                        pClient->disconnect();
                        set_state(OXI_DISCONNECTED);
                    }
                } else {
                    Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Connect failed, backing off %dms\n", RECONNECT_DELAY_MS);
                    set_state(OXI_DISCONNECTED);
                    last_reconnect = millis();
                }
            }
        }

        // Auto-reconnect: scan periodically when disconnected
        if (state == OXI_DISCONNECTED && cfg.oxi_enabled &&
            millis() - last_reconnect > RECONNECT_DELAY_MS) {
            last_reconnect = millis();
            Log::logf(CAT_BLE, LOG_DEBUG, "[BLE] Auto-reconnect: starting scan\n");
            scan_requested = true;
        }

        // Periodic injection
        if (feeding && state == OXI_STREAMING &&
            millis() - last_inject >= cfg.oxi_interval_ms) {
            last_inject = millis();
            inject_reading();
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void BleOxi::init() {
    xTaskCreatePinnedToCore(BleOxi::task, "ble_oxi", OXI_TASK_STACK,
                            nullptr, OXI_TASK_PRIO, &oxi_task_handle, 0);
}

void BleOxi::start_scan()  { scan_requested = true; }
void BleOxi::stop_scan()   { NimBLEDevice::getScan()->stop(); }

void BleOxi::connect(const char *addr) {
    target_addr = addr ? addr : "";
    connect_requested = true;
}

void BleOxi::disconnect()  { disconnect_requested = true; }
void BleOxi::start_feed()  { feeding = true;  Log::logf(CAT_BLE, LOG_INFO, "[BLE] Feeding started\n"); }
void BleOxi::stop_feed()   { feeding = false; Log::logf(CAT_BLE, LOG_INFO, "[BLE] Feeding stopped\n"); }
oxi_state_t BleOxi::get_state()            { return state; }
const oxi_reading_t& BleOxi::get_reading() { return reading; }
bool BleOxi::is_feeding()                  { return feeding; }
bool BleOxi::state_changed()               { bool d = state_dirty; state_dirty = false; return d; }

String BleOxi::get_scan_results() {
    String out;
    for (int i = 0; i < scan_result_count; i++) {
        out += scan_results[i].addr + " " + scan_results[i].name +
               " RSSI=" + String(scan_results[i].rssi) + "\n";
    }
    if (scan_result_count == 0) out = "(no oximeters found)\n";
    return out;
}
