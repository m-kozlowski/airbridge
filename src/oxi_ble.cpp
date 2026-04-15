#include "oxi_ble.h"
#include "oxi_arbiter.h"
#include "wifi.h"
#include "uart_arbiter.h"
#include "debug_log.h"
#include "app_config.h"
#include "crc.h"

#include <NimBLEDevice.h>
#include <Preferences.h>

#define OXI_TASK_STACK      4096
#define OXI_TASK_PRIO       4
#define SCAN_DURATION_MS    10000
#define RECONNECT_DELAY_MS  3000

static const NimBLEUUID PLX_SERVICE_UUID((uint16_t)0x1822);
static const NimBLEUUID PLX_CONTINUOUS_UUID((uint16_t)0x2A5F);
static const NimBLEUUID PLX_SPOT_UUID((uint16_t)0x2A5E);
static const NimBLEUUID HR_SERVICE_UUID((uint16_t)0x180D);
static const NimBLEUUID HR_MEASUREMENT_UUID((uint16_t)0x2A37);

// Nonin proprietary
static const NimBLEUUID NONIN_OXI_SERVICE_UUID("46A970E0-0D5F-11E2-8B5E-0002A5D5C51B");
static const NimBLEUUID NONIN_CONTINUOUS_UUID("0AAD7EA0-0D60-11E2-8E3C-0002A5D5C51B");
static const NimBLEUUID NONIN_CONTROL_POINT_UUID("1447AF80-0D60-11E2-88B6-0002A5D5C51B");

// Wellue/Viatom proprietary (O2Ring, CheckmeO2, SleepU, O2M)
static const NimBLEUUID VIATOM_SERVICE_UUID("14839AC4-7D7E-415C-9A42-167340CF2339");
static const NimBLEUUID VIATOM_READ_UUID("0734594A-A8E7-4B1A-A6B1-CD5243059A57");
static const NimBLEUUID VIATOM_WRITE_UUID("8B00ACE7-EB0B-49B0-BBE9-9AEE0A26E1A3");

static TaskHandle_t oxi_task_handle = nullptr;
static volatile oxi_state_t state = OXI_DISABLED;
static volatile bool state_dirty = false;
static inline void set_state(oxi_state_t s) { state = s; state_dirty = true; }
static oxi_reading_t reading = { -1, -1, false, 0 };  // local copy for callbacks
static volatile bool scan_requested = false;
typedef enum { CONN_NONE, CONN_AUTO, CONN_USER } connect_mode_t;
static volatile connect_mode_t connect_mode = CONN_NONE;
static volatile bool disconnect_requested = false;  // drop connection, stay enabled
static volatile bool disable_requested = false;     // drop connection, disable scanning

#define USER_CONNECT_RETRIES  3
#define USER_RETRY_DELAY_MS   2000
static volatile bool scan_complete = false;
static char target_addr[18] = "";

static NimBLEClient *pClient = nullptr;

static SemaphoreHandle_t scan_mutex = nullptr;
static oxi_scan_result_t scan_results[MAX_SCAN_RESULTS];
static int scan_result_count = 0;
static bool device_needs_encryption = false;  // Nonin needs it, Viatom/O2Ring don't

// Known devices list
// For devices that don't use BLE bonding (O2Ring, CheckMe, etc.)
#define KNOWN_MAX CONFIG_BT_NIMBLE_MAX_BONDS
static char known_addrs[KNOWN_MAX][18] = {};
static int known_count = 0;

static void known_load() {
    Preferences p;
    p.begin("oxi_known", true);
    known_count = p.getUChar("count", 0);
    if (known_count > KNOWN_MAX) known_count = KNOWN_MAX;
    for (int i = 0; i < known_count; i++) {
        char key[4];
        snprintf(key, sizeof(key), "a%d", i);
        String a = p.getString(key, "");
        strncpy(known_addrs[i], a.c_str(), 17);
        known_addrs[i][17] = '\0';
    }
    p.end();
    Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Loaded %d known devices\n", known_count);
}

static void known_save() {
    Preferences p;
    p.begin("oxi_known", false);
    p.putUChar("count", known_count);
    for (int i = 0; i < KNOWN_MAX; i++) {
        char key[4];
        snprintf(key, sizeof(key), "a%d", i);
        if (i < known_count) p.putString(key, known_addrs[i]);
        else p.remove(key);
    }
    p.end();
}

static bool known_contains(const char *addr) {
    for (int i = 0; i < known_count; i++) {
        if (strcasecmp(known_addrs[i], addr) == 0) return true;
    }
    return false;
}

static bool known_add(const char *addr) {
    if (known_contains(addr)) return true;
    if (known_count >= KNOWN_MAX) return false;
    strncpy(known_addrs[known_count], addr, 17);
    known_addrs[known_count][17] = '\0';
    known_count++;
    known_save();
    Log::logf(CAT_OXI, LOG_INFO, "[OXI] Added known device: %s\n", addr);
    return true;
}

static bool known_remove(const char *addr) {
    for (int i = 0; i < known_count; i++) {
        if (strcasecmp(known_addrs[i], addr) == 0) {
            // Shift remaining entries
            for (int j = i; j < known_count - 1; j++)
                memcpy(known_addrs[j], known_addrs[j+1], 18);
            known_count--;
            known_save();
            Log::logf(CAT_OXI, LOG_INFO, "[OXI] Removed known device: %s\n", addr);
            return true;
        }
    }
    return false;
}

static void known_clear() {
    known_count = 0;
    known_save();
    Log::logf(CAT_OXI, LOG_INFO, "[OXI] Cleared all known devices\n");
}

// Check if a device is "known" (either NimBLE-bonded or in our known list)
static bool is_device_known(const char *addr, uint8_t addr_type) {
    NimBLEAddress ba(std::string(addr), addr_type);
    if (NimBLEDevice::isBonded(ba)) return true;
    return known_contains(addr);
}


static void plx_notify_cb(NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len, bool isNotify) {
    if (len < 5) return;
    uint16_t spo2_raw = data[1] | (data[2] << 8);
    uint16_t pr_raw = data[3] | (data[4] << 8);
    int16_t spo2 = parse_sfloat(spo2_raw);
    int16_t pr = parse_sfloat(pr_raw);
    if (spo2 > 0 && spo2 <= 100 && pr > 0 && pr < 500) {
        OxiArbiter::feed(OXI_SRC_BLE, spo2, pr, true);
    } else {
        OxiArbiter::feed(OXI_SRC_BLE, -1, -1, false);
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
    // HR-only service - feed with current SpO2 from arbiter reading
    const oxi_reading_t &r = OxiArbiter::get_reading();
    if (hr > 0 && hr < 500) {
        OxiArbiter::feed(OXI_SRC_BLE, r.spo2, (int16_t)hr, r.valid);
    }
}

static void nonin_notify_cb(NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len, bool isNotify) {
    if (len >= 5) {
        uint8_t spo2 = data[2];
        uint16_t pr = data[3] | (data[4] << 8);
        if (spo2 > 0 && spo2 <= 100 && pr > 0 && pr < 500) {
            OxiArbiter::feed(OXI_SRC_BLE, (int8_t)spo2, (int16_t)pr, true);
        } else {
            OxiArbiter::feed(OXI_SRC_BLE, -1, -1, false);
        }
    }
}


// Viatom/Wellue: response packet header is 7 bytes (0x55, cmd, ~cmd, blk_lo, blk_hi, len_lo, len_hi)
// CMD_READ_SENSORS response: payload byte 0 = SpO2, byte 1 = HR
static NimBLERemoteCharacteristic *viatom_write_chr = nullptr;
static uint8_t viatom_invalid_count = 0;
#define VIATOM_MAX_INVALID  15  // disconnect after 15 invalid readings (~30s)

static void viatom_notify_cb(NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len, bool isNotify) {
    // debug
    if (len > 0) {
        char hex[64] = {};
        int n = len > 20 ? 20 : len;
        for (int i = 0; i < n; i++) snprintf(hex + i*3, 4, "%02X ", data[i]);
        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Viatom RX len=%d: %s\n", len, hex);
    }

    // Response: 55 CMD ~CMD BLK_LO BLK_HI LEN_LO LEN_HI [payload...]
    // Byte 7 = SpO2, Byte 8 = HR, 0xFF = no finger, 0x00 = no reading
    if (len >= 9 && data[0] == 0x55) {
        uint8_t spo2 = data[7];
        uint8_t hr = data[8];
        bool no_finger = (spo2 == 0 || spo2 == 0xFF || hr == 0 || hr == 0xFF);
        if (!no_finger && spo2 <= 100 && hr < 250) {
            OxiArbiter::feed(OXI_SRC_BLE, (int8_t)spo2, (int16_t)hr, true);
            viatom_invalid_count = 0;
        } else {
            OxiArbiter::feed(OXI_SRC_BLE, -1, -1, false);
            if (++viatom_invalid_count >= VIATOM_MAX_INVALID) {
                Log::logf(CAT_OXI, LOG_INFO, "[OXI] Viatom: no valid data for %d readings, disconnecting\n",
                          VIATOM_MAX_INVALID);
                disconnect_requested = true;
            }
        }
    }
}

class OxiScanCB : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *dev) override {
        String name = dev->getName().c_str();
        bool is_oxi = dev->isAdvertisingService(PLX_SERVICE_UUID) ||
                       dev->isAdvertisingService(NONIN_OXI_SERVICE_UUID) ||
                       dev->isAdvertisingService(HR_SERVICE_UUID) ||
                       dev->isAdvertisingService(VIATOM_SERVICE_UUID) ||
                       name.startsWith("Nonin") ||
                       name.startsWith("O2Ring") ||
                       name.startsWith("O2M") ||
                       name.startsWith("CheckMe") ||
                       name.startsWith("Checkme") ||
                       name.startsWith("CheckO2") ||
                       name.startsWith("SleepU");

        if (is_oxi && scan_mutex && xSemaphoreTake(scan_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (scan_result_count < MAX_SCAN_RESULTS) {
                scan_results[scan_result_count].addr = dev->getAddress().toString().c_str();
                scan_results[scan_result_count].name = name;
                scan_results[scan_result_count].rssi = dev->getRSSI();
                scan_results[scan_result_count].addr_type = dev->getAddress().getType();
                scan_result_count++;
            }
            xSemaphoreGive(scan_mutex);
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Found: %s (%s) RSSI=%d\n",
                          name.c_str(), dev->getAddress().toString().c_str(), dev->getRSSI());
        }
    }

    void onScanEnd(const NimBLEScanResults &results, int reason) override {
        Log::logf(CAT_OXI, LOG_INFO, "[OXI] Scan complete, %d oximeters found (reason=%d)\n", scan_result_count, reason);
        scan_complete = true;
    }
};

static OxiScanCB scanCB;

class OxiClientCB : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient *client) override {
        Log::logf(CAT_OXI, LOG_INFO, "[OXI] Connected\n");
    }

    void onDisconnect(NimBLEClient *client, int reason) override {
        Log::logf(CAT_OXI, LOG_INFO, "[OXI] Disconnected (reason=0x%X)\n", reason);
        viatom_write_chr = nullptr;
        OxiArbiter::stop_feed();
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
            Log::logf(CAT_OXI, LOG_INFO, "[OXI] Encrypted + bonded\n");
        } else {
            Log::logf(CAT_OXI, LOG_WARN, "[OXI] Auth complete (no encryption)\n");
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
        if (plxCont && plxCont->canNotify() && plxCont->subscribe(true, plx_notify_cb)) {
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Subscribed PLX Continuous\n");
            got_spo2 = got_hr = true;
        }
        if (!got_spo2) {
            NimBLERemoteCharacteristic *plxSpot = plxSvc->getCharacteristic(PLX_SPOT_UUID);
            if (plxSpot && plxSpot->canIndicate() && plxSpot->subscribe(false, plx_notify_cb)) {
                Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Subscribed PLX Spot\n");
                got_spo2 = got_hr = true;
            }
        }
    }

    if (!got_spo2) {
        NimBLERemoteService *noninSvc = cl->getService(NONIN_OXI_SERVICE_UUID);
        if (noninSvc) {
            NimBLERemoteCharacteristic *noninCont = noninSvc->getCharacteristic(NONIN_CONTINUOUS_UUID);
            if (noninCont && noninCont->canNotify() && noninCont->subscribe(true, nonin_notify_cb)) {
                Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Subscribed Nonin Continuous\n");
                got_spo2 = got_hr = true;
            }
        }
    }

    if (!got_spo2) {
        NimBLERemoteService *viatomSvc = cl->getService(VIATOM_SERVICE_UUID);
        if (viatomSvc) {
            NimBLERemoteCharacteristic *viatomRead = viatomSvc->getCharacteristic(VIATOM_READ_UUID);
            if (viatomRead && viatomRead->canNotify() && viatomRead->subscribe(true, viatom_notify_cb)) {
                Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Subscribed Viatom read\n");
                viatom_invalid_count = 0;
                viatom_write_chr = viatomSvc->getCharacteristic(VIATOM_WRITE_UUID);
                if (viatom_write_chr) Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Viatom write chr found\n");
                got_spo2 = got_hr = true;
            }
        }
    }

    if (!got_hr) {
        NimBLERemoteService *hrSvc = cl->getService(HR_SERVICE_UUID);
        if (hrSvc) {
            NimBLERemoteCharacteristic *hrMeas = hrSvc->getCharacteristic(HR_MEASUREMENT_UUID);
            if (hrMeas && hrMeas->canNotify() && hrMeas->subscribe(true, hr_notify_cb)) {
                Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Subscribed Heart Rate\n");
                got_hr = true;
            }
        }
    }

    return got_spo2 || got_hr;
}

// Set date/time on Nonin devices so stored records have correct timestamps.
static void set_nonin_datetime(NimBLEClient *cl) {
    if (!WiFiSetup::time_synced()) {
        Log::logf(CAT_OXI, LOG_WARN, "[OXI] Skipping Nonin datetime — NTP not synced\n");
        return;
    }

    NimBLERemoteService *svc = cl->getService(NONIN_OXI_SERVICE_UUID);
    if (!svc) return;

    NimBLERemoteCharacteristic *cp = svc->getCharacteristic(NONIN_CONTROL_POINT_UUID);
    if (!cp || !cp->canWrite()) {
        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Nonin control point not writable\n");
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
        Log::logf(CAT_OXI, LOG_INFO, "[OXI] Nonin datetime set: %s\n", ts);
    } else {
        Log::logf(CAT_OXI, LOG_WARN, "[OXI] Nonin datetime write failed\n");
    }
}

static void set_viatom_datetime() {
    if (!WiFiSetup::time_synced() || !viatom_write_chr) return;

    struct tm t;
    time_t now = time(nullptr);
    localtime_r(&now, &t);

    // json {"SetTIME":"YYYY-MM-DD,HH:MM:SS"}
    char json[48];
    strftime(json, sizeof(json), "{\"SetTIME\":\"%Y-%m-%d,%H:%M:%S\"}", &t);
    int json_len = strlen(json);

    // AA CMD ~CMD BLK_LO BLK_HI LEN_LO LEN_HI [json] CRC8
    int pkt_len = 7 + json_len + 1;
    uint8_t pkt[64];
    pkt[0] = 0xAA;
    pkt[1] = 0x16;  // CMD_CONFIG
    pkt[2] = 0x16 ^ 0xFF;
    pkt[3] = 0x00; pkt[4] = 0x00;  // block
    pkt[5] = json_len & 0xFF;
    pkt[6] = (json_len >> 8) & 0xFF;
    memcpy(pkt + 7, json, json_len);

    uint8_t crc = crc8_ccitt(pkt, 7 + json_len);
    pkt[7 + json_len] = crc;

    if (viatom_write_chr->writeValue(pkt, pkt_len, false)) {
        Log::logf(CAT_OXI, LOG_INFO, "[OXI] Viatom datetime set: %s\n", json);
    } else {
        Log::logf(CAT_OXI, LOG_WARN, "[OXI] Viatom datetime write failed\n");
    }
}


void OxiBle::task(void *param) {
    scan_mutex = xSemaphoreCreateMutex();
    known_load();
    NimBLEDevice::init(Config::get().hostname.c_str());
    NimBLEDevice::setSecurityAuth(true, false, false);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(&clientCB);
    pClient->setConnectionParams(12, 12, 0, 400);

    auto &cfg = Config::get();
    if (cfg.oxi_enabled) {
        set_state(OXI_DISCONNECTED);
        scan_requested = true;
    }

    uint32_t last_reconnect = 0;

    while (true) {
        if (disable_requested) {
            disable_requested = false;
            disconnect_requested = false;
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Disable requested\n");
            if (pClient->isConnected()) pClient->disconnect();
            OxiArbiter::stop_feed();
            set_state(OXI_DISABLED);
        }

        if (disconnect_requested) {
            disconnect_requested = false;
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Disconnect requested\n");
            if (pClient->isConnected()) pClient->disconnect();
            OxiArbiter::stop_feed();
            set_state(OXI_DISCONNECTED);
        }

        if (scan_complete) {
            scan_complete = false;
            if (state == OXI_SCANNING) {
                Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Scan done, %d results\n", scan_result_count);
                set_state(OXI_DISCONNECTED);
                if (scan_result_count > 0) {
                    String target = cfg.oxi_device_addr;
                    bool found = false;
                    if (target.length() > 0) {
                        for (int i = 0; i < scan_result_count; i++) {
                            if (scan_results[i].addr.equalsIgnoreCase(target)) {
                                found = true;
                                Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Target %s found in scan\n", target.c_str());
                                break;
                            }
                        }
                        if (!found) Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Target %s not in scan results\n", target.c_str());
                    } else {
                        for (int i = 0; i < scan_result_count; i++) {
                            bool known = is_device_known(scan_results[i].addr.c_str(),
                                                         scan_results[i].addr_type);
                            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] %s %s known=%d\n",
                                      scan_results[i].name.c_str(), scan_results[i].addr.c_str(), known);
                            if (cfg.oxi_require_known) {
                                if (known) { found = true; break; }
                            } else {
                                if (scan_results[i].name.startsWith("Nonin")) {
                                    if (known) { found = true; break; }
                                } else {
                                    found = true;
                                    break;
                                }
                            }
                        }
                    }
                    if (found) {
                        connect_mode = CONN_AUTO;
                        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Auto-connect triggered\n");
                    }
                }
            }
        }

        if (scan_requested) {
            scan_requested = false;
            if (scan_mutex) xSemaphoreTake(scan_mutex, portMAX_DELAY);
            scan_result_count = 0;
            if (scan_mutex) xSemaphoreGive(scan_mutex);
            scan_complete = false;
            set_state(OXI_SCANNING);
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Starting scan (%dms)\n", SCAN_DURATION_MS);
            NimBLEScan *pScan = NimBLEDevice::getScan();
            pScan->setScanCallbacks(&scanCB);
            pScan->setActiveScan(true);
            pScan->setInterval(100);
            pScan->setWindow(99);
            pScan->start(SCAN_DURATION_MS);
        }

        if (connect_mode != CONN_NONE) {
            connect_mode_t mode = connect_mode;
            connect_mode = CONN_NONE;
            NimBLEDevice::getScan()->stop();
            // Wait for scan to actually stop before connecting
            for (int i = 0; i < 20 && NimBLEDevice::getScan()->isScanning(); i++)
                vTaskDelay(pdMS_TO_TICKS(50));
            scan_complete = false;  // discard any pending scan-complete trigger

            String addr = target_addr;
            if (addr.length() == 0 && cfg.oxi_device_addr.length() > 0)
                addr = cfg.oxi_device_addr;
            if (addr.length() == 0 && scan_result_count > 0)
                addr = scan_results[0].addr;

            int max_attempts = (mode == CONN_USER) ? USER_CONNECT_RETRIES : 1;

            if (addr.length() > 0) {
                set_state(OXI_CONNECTING);

                uint8_t atype = 1;
                String dev_name = "";
                for (int i = 0; i < scan_result_count; i++) {
                    if (scan_results[i].addr.equalsIgnoreCase(addr)) {
                        atype = scan_results[i].addr_type;
                        dev_name = scan_results[i].name;
                        break;
                    }
                }

                // Nonin requires Just Works bonding; Viatom/O2Ring/generic PLX do not
                device_needs_encryption = dev_name.startsWith("Nonin");

                NimBLEAddress bleAddr(std::string(addr.c_str()), atype);
                bool connected = false;

                for (int attempt = 1; attempt <= max_attempts; attempt++) {
                    if (disconnect_requested || disable_requested) break;

                    if (attempt > 1) {
                        Log::logf(CAT_OXI, LOG_INFO, "[OXI] Retry %d/%d after %dms\n",
                                  attempt, max_attempts, USER_RETRY_DELAY_MS);
                        vTaskDelay(pdMS_TO_TICKS(USER_RETRY_DELAY_MS));
                    }

                    // cancel any pending connection and clean up stale state
                    if (pClient->isConnected()) {
                        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Disconnecting stale connection\n");
                        pClient->disconnect();
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    }
                    pClient->cancelConnect();
                    vTaskDelay(pdMS_TO_TICKS(500));

                    // connect
                    set_state(OXI_CONNECTING);
                    Log::logf(CAT_OXI, LOG_INFO, "[OXI] Connecting to %s (type=%d, %s, attempt %d/%d)...\n",
                              addr.c_str(), atype, mode == CONN_USER ? "user" : "auto",
                              attempt, max_attempts);

                    bool ok = pClient->connect(bleAddr);

                    if (!ok) {
                        int err = pClient->getLastError();
                        Log::logf(CAT_OXI, LOG_WARN, "[OXI] connect() failed (err=%d)\n", err);

                        // EALREADY: previous connect still in flight
                        if (err == 2) {
                            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Waiting for pending connect...\n");
                            for (int i = 0; i < 50 && !pClient->isConnected(); i++)
                                vTaskDelay(pdMS_TO_TICKS(200));
                            ok = pClient->isConnected();
                            Log::logf(CAT_OXI, LOG_INFO, "[OXI] Pending connect %s\n", ok ? "succeeded" : "failed");
                            if (ok) vTaskDelay(pdMS_TO_TICKS(500));
                        }

                        // EDONE: stale bond - delete and retry within this attempt
                        if (!ok && err == 13) {
                            Log::logf(CAT_OXI, LOG_INFO, "[OXI] Removing stale bond and retrying\n");
                            NimBLEDevice::deleteBond(bleAddr);
                            vTaskDelay(pdMS_TO_TICKS(500));
                            ok = pClient->connect(bleAddr);
                            if (!ok) {
                                Log::logf(CAT_OXI, LOG_WARN, "[OXI] Post-bond-delete retry failed (err=%d)\n",
                                          pClient->getLastError());
                            }
                        }
                    }

                    if (!ok) continue;

                    // encrypt for devices that require it 
                    if (device_needs_encryption) {
                        set_state(OXI_BONDING);
                        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Initiating encryption\n");

                        bool secured = pClient->secureConnection(false);
                        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Encryption: secure=%d connected=%d err=%d\n",
                                  secured, pClient->isConnected(), pClient->getLastError());

                        if (!pClient->isConnected()) {
                            Log::logf(CAT_OXI, LOG_WARN, "[OXI] Lost connection during encryption\n");
                            continue;
                        }

                        if (!secured) {
                            Log::logf(CAT_OXI, LOG_WARN, "[OXI] Encryption failed, disconnecting to retry\n");
                            pClient->disconnect();
                            vTaskDelay(pdMS_TO_TICKS(500));
                            continue;
                        }
                    } else {
                        Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Skipping encryption (not required)\n");
                    }

                    // subscribe
                    if (!subscribe_services(pClient)) {
                        Log::logf(CAT_OXI, LOG_WARN, "[OXI] No suitable services, disconnecting\n");
                        pClient->disconnect();
                        continue;
                    }

                    // Final check - onDisconnect may have fired during subscribe
                    if (!pClient->isConnected()) {
                        Log::logf(CAT_OXI, LOG_WARN, "[OXI] Connection lost after subscribe\n");
                        continue;
                    }

                    set_nonin_datetime(pClient);
                    set_viatom_datetime();
                    OxiArbiter::set_source_id(pClient->getPeerAddress().toString().c_str());
                    set_state(OXI_STREAMING);
                    Log::logf(CAT_OXI, LOG_INFO, "[OXI] Streaming started\n");

                    if (mode == CONN_USER && !device_needs_encryption) {
                        known_add(addr.c_str());
                    }
                    connected = true;
                    break;
                }

                if (!connected) {
                    Log::logf(CAT_OXI, LOG_WARN, "[OXI] Connect sequence failed after %d attempt(s)\n",
                              max_attempts);
                    if (pClient->isConnected()) pClient->disconnect();
                    set_state(OXI_DISCONNECTED);
                    last_reconnect = millis();
                }
            }
        }

        // Auto-reconnect: scan periodically when disconnected and no other source active
        if (state == OXI_DISCONNECTED && cfg.oxi_enabled &&
            OxiArbiter::active_source() == OXI_SRC_NONE &&
            millis() - last_reconnect > RECONNECT_DELAY_MS) {
            last_reconnect = millis();
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Auto-reconnect: starting scan\n");
            scan_requested = true;
        }

        // Viatom: poll sensor readings every 2s while streaming
        if (state == OXI_STREAMING && viatom_write_chr && pClient->isConnected()) {
            static uint32_t last_viatom_poll = 0;
            if (millis() - last_viatom_poll >= 2000) {
                last_viatom_poll = millis();
                // CMD_READ_SENSORS packet: AA 17 E8 00 00 00 00 CRC
                uint8_t cmd[] = {0xAA, 0x17, 0xE8, 0x00, 0x00, 0x00, 0x00, 0x00};
                cmd[7] = crc8_ccitt(cmd, 7);
                viatom_write_chr->writeValue(cmd, sizeof(cmd), false);
            }
        }

        // Arbiter handles injection timing and source gating
        OxiArbiter::poll();

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void OxiBle::init() {
    xTaskCreatePinnedToCore(OxiBle::task, "ble_oxi", OXI_TASK_STACK,
                            nullptr, OXI_TASK_PRIO, &oxi_task_handle, 0);
}

void OxiBle::start_scan()  { scan_requested = true; }
void OxiBle::stop_scan()   { NimBLEDevice::getScan()->stop(); }

void OxiBle::connect(const char *addr) {
    strncpy(target_addr, addr ? addr : "", sizeof(target_addr) - 1);
    target_addr[sizeof(target_addr) - 1] = '\0';
    connect_mode = CONN_USER;
}

void OxiBle::disconnect()  { disconnect_requested = true; }
void OxiBle::disable()     { disable_requested = true; }
void OxiBle::enable() {
    if (state == OXI_DISABLED) {
        set_state(OXI_DISCONNECTED);
        scan_requested = true;
    }
}
oxi_state_t OxiBle::get_state()            { return state; }
bool OxiBle::state_changed()               { bool d = state_dirty; state_dirty = false; return d; }

const oxi_scan_result_t *OxiBle::get_scan_results(int &count) {
    if (scan_mutex) xSemaphoreTake(scan_mutex, portMAX_DELAY);
    count = scan_result_count;
    if (scan_mutex) xSemaphoreGive(scan_mutex);
    return scan_results;
}

int OxiBle::get_all_known(char addrs[][18], int max) {
    int n = 0;
    // NimBLE bonds
    int nb = NimBLEDevice::getNumBonds();
    for (int i = 0; i < nb && n < max; i++) {
        NimBLEAddress ba = NimBLEDevice::getBondedAddress(i);
        strncpy(addrs[n], ba.toString().c_str(), 17);
        addrs[n][17] = '\0';
        n++;
    }
    // Known list (skip duplicates with bonds)
    for (int i = 0; i < known_count && n < max; i++) {
        bool dup = false;
        for (int j = 0; j < n; j++) {
            if (strcasecmp(addrs[j], known_addrs[i]) == 0) { dup = true; break; }
        }
        if (!dup) {
            strncpy(addrs[n], known_addrs[i], 17);
            addrs[n][17] = '\0';
            n++;
        }
    }
    return n;
}

bool OxiBle::remove_known(const char *addr) {
    bool removed = false;
    // Try NimBLE bond first
    NimBLEDevice::getScan()->stop();
    int nb = NimBLEDevice::getNumBonds();
    for (int i = 0; i < nb; i++) {
        NimBLEAddress ba = NimBLEDevice::getBondedAddress(i);
        if (strcasecmp(ba.toString().c_str(), addr) == 0) {
            int rc = ble_gap_unpair(ba.getBase());
            if (rc == 0) removed = true;
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Unpair %s rc=%d\n", addr, rc);
            break;
        }
    }
    // Also remove from known list
    if (known_remove(addr)) removed = true;
    return removed;
}

void OxiBle::clear_all_known() {
    NimBLEDevice::getScan()->stop();
    NimBLEDevice::deleteAllBonds();
    known_clear();
}
