#include "oxi_ble.h"
#include "oxi_arbiter.h"
#include "wifi.h"
#include "uart_arbiter.h"
#include "debug_log.h"
#include "app_config.h"
#include "crc.h"

#include <NimBLEDevice.h>

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

static TaskHandle_t oxi_task_handle = nullptr;
static volatile oxi_state_t state = OXI_DISABLED;
static volatile bool state_dirty = false;
static inline void set_state(oxi_state_t s) { state = s; state_dirty = true; }
static oxi_reading_t reading = { -1, -1, false, 0 };  // local copy for callbacks
static volatile bool scan_requested = false;
typedef enum { CONN_NONE, CONN_AUTO, CONN_USER } connect_mode_t;
static volatile connect_mode_t connect_mode = CONN_NONE;
static volatile bool disconnect_requested = false;

#define USER_CONNECT_RETRIES  3
#define USER_RETRY_DELAY_MS   2000
static volatile bool scan_complete = false;
static char target_addr[18] = "";

static NimBLEClient *pClient = nullptr;

static SemaphoreHandle_t scan_mutex = nullptr;
static oxi_scan_result_t scan_results[MAX_SCAN_RESULTS];
static volatile int scan_result_count = 0;


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


class OxiScanCB : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *dev) override {
        String name = dev->getName().c_str();
        bool is_oxi = dev->isAdvertisingService(PLX_SERVICE_UUID) ||
                       dev->isAdvertisingService(NONIN_OXI_SERVICE_UUID) ||
                       dev->isAdvertisingService(HR_SERVICE_UUID) ||
                       name.startsWith("Nonin") ||
                       name.startsWith("O2Ring") ||
                       name.startsWith("CheckMe");

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


void OxiBle::task(void *param) {
    scan_mutex = xSemaphoreCreateMutex();
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
        if (disconnect_requested) {
            disconnect_requested = false;
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Disconnect requested\n");
            if (pClient->isConnected()) pClient->disconnect();
            OxiArbiter::stop_feed();
            set_state(OXI_DISABLED);
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
                            if (scan_results[i].name.startsWith("Nonin")) {
                                NimBLEAddress ba(std::string(scan_results[i].addr.c_str()), scan_results[i].addr_type);
                                bool bonded = NimBLEDevice::isBonded(ba);
                                Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] %s %s bonded=%d\n",
                                          scan_results[i].name.c_str(), scan_results[i].addr.c_str(), bonded);
                                if (bonded) { found = true; break; }
                            } else {
                                found = true;
                                break;
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
                for (int i = 0; i < scan_result_count; i++) {
                    if (scan_results[i].addr.equalsIgnoreCase(addr)) {
                        atype = scan_results[i].addr_type;
                        break;
                    }
                }

                NimBLEAddress bleAddr(std::string(addr.c_str()), atype);
                bool connected = false;

                for (int attempt = 1; attempt <= max_attempts; attempt++) {
                    if (disconnect_requested) break;

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

                    // encrypt
                    set_state(OXI_BONDING);
                    Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Connected, initiating encryption\n");

                    bool secured = pClient->secureConnection(false);
                    Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] Encryption: secure=%d connected=%d err=%d\n",
                              secured, pClient->isConnected(), pClient->getLastError());

                    if (!secured && pClient->isConnected()) {
                        Log::logf(CAT_OXI, LOG_WARN, "[OXI] Encryption failed, disconnecting to retry\n");
                        pClient->disconnect();
                        vTaskDelay(pdMS_TO_TICKS(500));
                        continue;
                    }

                    if (!pClient->isConnected()) {
                        Log::logf(CAT_OXI, LOG_WARN, "[OXI] Lost connection during encryption\n");
                        continue;
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
                    OxiArbiter::set_source_id(pClient->getPeerAddress().toString().c_str());
                    set_state(OXI_STREAMING);
                    Log::logf(CAT_OXI, LOG_INFO, "[OXI] Streaming started\n");
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
oxi_state_t OxiBle::get_state()            { return state; }
bool OxiBle::state_changed()               { bool d = state_dirty; state_dirty = false; return d; }

const oxi_scan_result_t *OxiBle::get_scan_results(int &count) {
    if (scan_mutex) xSemaphoreTake(scan_mutex, portMAX_DELAY);
    count = scan_result_count;
    if (scan_mutex) xSemaphoreGive(scan_mutex);
    return scan_results;
}
