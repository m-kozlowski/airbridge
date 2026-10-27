#include "wifi_setup.h"
#include "app_config.h"
#include "debug_log.h"
#include "network_hints.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include <esp_wifi.h>
#include <esp_sntp.h>
#include <esp_netif.h>
#include <time.h>
#include <atomic>

typedef enum {
    WF_OFF,
    WF_HINT_TRY,        // fast reconnect via BSSID+channel hint
    WF_SCANNING,
    WF_CONNECTING,      // WiFi.begin() called, waiting for IP
    WF_PMF_RETRY,       // re-tried CONNECTING with pmf_cfg disabled
    WF_CONNECTED,
    WF_ROAM_SCAN,       // scanning for a better AP
    WF_AP_FALLBACK,     // AP+STA mode, periodically retrying STA
} wifi_state_t;

static wifi_state_t wf_state = WF_OFF;
static std::atomic<uint32_t> status_revision{0};
static std::atomic<uint32_t> sta_address{0}, ap_address{0};
static std::atomic<bool> ap_address_pending{false};
static String active_ap_ssid;
static uint32_t state_entered_ms = 0;
static bool ntp_done = false;

// Scan candidates track each visible (ssid, bssid) tuple separately so that
// same-SSID multi-AP setups can roam by BSSID, not just by SSID slot.
#define SCAN_CANDIDATES_MAX 16
struct ScanCandidate {
    uint8_t net_idx;        // index into wifi_nets
    uint8_t bssid[6];
    uint8_t channel;
    int8_t  rssi;
};
static ScanCandidate scan_candidates[SCAN_CANDIDATES_MAX];
static uint8_t scan_candidate_count = 0;
static std::atomic<bool> scan_pending{false};
static std::atomic<bool> scan_done{false};
static std::atomic<uint32_t> scan_status{1};
static uint32_t scan_started_ms = 0;
#define SCAN_TIMEOUT_MS          15000
#define SCAN_ACTIVE_MAX_MS      120
#define SCAN_HOME_DWELL_MS       30

static uint8_t connect_idx = 0xFF;
static uint8_t try_pos = 0;
static uint8_t connect_retries = 0;

#define ROAM_CHECK_INTERVAL_MS  60000
#define ROAM_RSSI_THRESHOLD     (-73)
#define ROAM_CONSECUTIVE_LOW    3
#define ROAM_HYSTERESIS_DB      8
static uint32_t last_roam_check = 0;
static uint8_t low_rssi_count = 0;
static bool roaming_suspended = false;
static bool roam_connecting = false;
static uint8_t roam_target[6] = {};
static std::atomic<uint8_t> reconfigure_pending{0};

#define AP_RETRY_INTERVAL_MS    30000
static uint32_t last_ap_retry = 0;

#define HINT_TIMEOUT_MS         5000
#define CONNECT_TIMEOUT_MS      15000
#define CONNECT_RETRIES         2
#define STA_RESTART_SETTLE_MS   100

static volatile bool ntp_synced = false;
static bool got_ip = false;
static bool sta_disconnected = false;
static volatile bool hint_refresh_pending = false;
static volatile uint8_t last_disconnect_reason = 0;
static bool pending_pmf_disable = false;
static volatile uint8_t ap_client_count = 0;
static uint32_t ap_quiet_since_ms = 0;
static bool pending_ap_teardown = false;
#define AP_TEARDOWN_QUIET_MS    120000

static String mdns_hostname;
static bool mdns_started = false;
static bool mdns_attempted = false;
static uint32_t mdns_last_attempt_ms = 0;
#define MDNS_RETRY_INTERVAL_MS 30000

static void stop_mdns() {
    if (mdns_started) MDNS.end();
    mdns_started = false;
    mdns_attempted = false;
}

static void check_mdns() {
    const wifi_mode_t mode = WiFi.getMode();
    if (mode == WIFI_OFF) {
        stop_mdns();
        return;
    }
    // The responder follows STA and AP events without being recreated.
    if (mdns_started) return;
    const bool available = WiFi.status() == WL_CONNECTED ||
        ((mode & WIFI_AP) && WiFi.softAPIP() != IPAddress());
    if (!available) return;

    const uint32_t now = millis();
    if (mdns_attempted && now - mdns_last_attempt_ms < MDNS_RETRY_INTERVAL_MS) return;
    mdns_attempted = true;
    mdns_last_attempt_ms = now;
    if (mdns_hostname.isEmpty() || !MDNS.begin(mdns_hostname)) {
        // begin() can allocate the responder before failing to set its name.
        MDNS.end();
        Log::logf(CAT_WIFI, LOG_WARN, "mDNS start failed for '%s'; retry in %d s\n",
                  mdns_hostname.c_str(), MDNS_RETRY_INTERVAL_MS / 1000);
        return;
    }
    mdns_started = true;
    Log::logf(CAT_WIFI, LOG_INFO, "mDNS: %s.local\n", mdns_hostname.c_str());
}


static void ntp_sync_cb(struct timeval *tv) {
    const bool first_sync = !ntp_synced;
    ntp_synced = true;
    struct tm t;
    time_t now = time(nullptr);
    localtime_r(&now, &t);
    Log::logf(CAT_TIME, first_sync ? LOG_INFO : LOG_DEBUG, "NTP synced: %04d-%02d-%02d %02d:%02d:%02d\n",
              t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
              t.tm_hour, t.tm_min, t.tm_sec);
}

static void sync_ntp() {
    auto &cfg = Config::get();
    if (cfg.tz.length() > 0) {
        setenv("TZ", cfg.tz.c_str(), 1);
        tzset();
    }
    sntp_set_time_sync_notification_cb(ntp_sync_cb);
    if (esp_sntp_enabled()) esp_sntp_stop();
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    if (cfg.ntp_server.length() > 0) {
#if LWIP_DHCP_GET_NTP_SRV
        esp_sntp_servermode_dhcp(false);
#endif
        esp_sntp_setservername(0, cfg.ntp_server.c_str());
        Log::logf(CAT_TIME, LOG_DEBUG, "NTP: configured server %s\n", cfg.ntp_server.c_str());
    } else {
#if LWIP_DHCP_GET_NTP_SRV
        esp_sntp_servermode_dhcp(true);
#endif
        esp_sntp_setservername(0, "pool.ntp.org");
        Log::logf(CAT_TIME, LOG_DEBUG, "NTP: DHCP + pool.ntp.org fallback\n");
    }
    esp_sntp_init();
}


static void wifi_event_cb(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
    case ARDUINO_EVENT_WIFI_SCAN_DONE:
        if (scan_pending.load(std::memory_order_acquire)) {
            scan_status.store(info.wifi_scan_done.status, std::memory_order_relaxed);
            scan_done.store(true, std::memory_order_release);
        }
        break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        sta_address.store(info.got_ip.ip_info.ip.addr);
        got_ip = true;
        hint_refresh_pending = true;
        status_revision.fetch_add(1);
        break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        sta_address.store(0);
        sta_disconnected = true;
        last_disconnect_reason = info.wifi_sta_disconnected.reason;
        status_revision.fetch_add(1);
        break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
    case ARDUINO_EVENT_WIFI_STA_STOP:
        sta_address.store(0);
        status_revision.fetch_add(1);
        break;
    case ARDUINO_EVENT_WIFI_AP_STOP:
        ap_address.store(0);
        [[fallthrough]];
    case ARDUINO_EVENT_WIFI_AP_START:
        ap_address_pending.store(true);
        status_revision.fetch_add(1);
        break;
    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
        ap_client_count = ap_client_count + 1;
        break;
    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
        if (ap_client_count > 0) ap_client_count = ap_client_count - 1;
        break;
    default:
        break;
    }
}

static void refresh_ap_address() {
    if (!ap_address_pending.exchange(false)) return;
    const uint32_t address = (WiFi.getMode() & WIFI_AP) ?
        static_cast<uint32_t>(WiFi.softAPIP()) : 0;
    active_ap_ssid = address ? WiFi.softAPSSID() : String();
    if (ap_address.exchange(address) != address) status_revision.fetch_add(1);
}

const char *WiFiSetup::access_point_ssid() {
    return ap_address.load() ? active_ap_ssid.c_str() : "";
}


static void set_state(wifi_state_t s) {
    if (wf_state != s) status_revision.fetch_add(1);
    wf_state = s;
    state_entered_ms = millis();
}

static String ap_ssid_str() {
    auto &cfg = Config::get();
    return cfg.hostname + "_" + String((uint32_t)ESP.getEfuseMac(), HEX);
}

static uint8_t find_net_by_ssid(const char *ssid) {
    auto &cfg = Config::get();
    for (uint8_t i = 0; i < cfg.wifi_net_count; i++) {
        if (cfg.wifi_nets[i].enabled && cfg.wifi_nets[i].ssid.equalsIgnoreCase(ssid))
            return i;
    }
    return 0xFF;
}

static void stop_sta_attempt() {
    esp_err_t err = esp_wifi_disconnect();
    if (err != ESP_OK) {
        Log::logf(CAT_WIFI, LOG_DEBUG,
                  "STA disconnect returned err=%d\n", err);
    }
    delay(STA_RESTART_SETTLE_MS);
}

static bool apply_pmf_override(bool disable) {
    wifi_config_t wcfg = {};
    if (esp_wifi_get_config(WIFI_IF_STA, &wcfg) != ESP_OK) return false;
    wcfg.sta.pmf_cfg.capable = !disable;
    wcfg.sta.pmf_cfg.required = false;
    return esp_wifi_set_config(WIFI_IF_STA, &wcfg) == ESP_OK;
}

static void switch_to_pmf_disabled() {
    stop_sta_attempt();
    pending_pmf_disable = apply_pmf_override(true);
    if (!pending_pmf_disable) {
        Log::logf(CAT_WIFI, LOG_WARN,
                  "Failed to disable pmf_cfg\n");
    }
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        Log::logf(CAT_WIFI, LOG_WARN,
                  "PMF reconnect failed (err=%d)\n", err);
    }
}

static void retry_current_connect() {
    stop_sta_attempt();
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        Log::logf(CAT_WIFI, LOG_WARN,
                  "STA reconnect failed (err=%d)\n", err);
    }
    set_state(WF_CONNECTING);
}

// Some routers (e.g. older OpenWrt builds, certain ISP-provisioned units)
// advertise PMF capability but reject the handshake with reason 208
// (WIFI_REASON_ASSOC_COMEBACK_TIME_TOO_LONG). Retry once with pmf_cfg cleared.
static void enter_pmf_retry() {
    auto &cfg = Config::get();
    if (connect_idx >= cfg.wifi_net_count) return;
    WiFiNetwork &net = cfg.wifi_nets[connect_idx];

    Log::logf(CAT_WIFI, LOG_DEBUG,
              "PMF retry: disabling pmf_cfg and reconnecting to '%s'\n",
              net.ssid.c_str());

    // Reuses the staged credentials (set by the previous WiFi.begin) - no
    // second WiFi.begin, which would reset pmf_cfg.
    switch_to_pmf_disabled();
    set_state(WF_PMF_RETRY);
}

static void begin_connect(uint8_t idx, bool use_hint) {
    auto &cfg = Config::get();
    if (idx >= cfg.wifi_net_count) return;
    WiFiNetwork &net = cfg.wifi_nets[idx];

    connect_idx = idx;
    connect_retries = 0;
    pending_pmf_disable = false;
    set_state(use_hint ? WF_HINT_TRY : WF_CONNECTING);

    NetworkHint *h = use_hint ? NetworkHints::find_best(net.ssid.c_str()) : nullptr;

    if (h) {
        Log::logf(CAT_WIFI, LOG_DEBUG, "Fast connect to '%s' ch=%d\n",
                  net.ssid.c_str(), h->channel);
        WiFi.begin(net.ssid.c_str(), net.pass.c_str(), h->channel, h->bssid);
    } else {
        Log::logf(CAT_WIFI, LOG_DEBUG, "Connecting to '%s'...\n", net.ssid.c_str());
        WiFi.begin(net.ssid.c_str(), net.pass.c_str());
        set_state(WF_CONNECTING);
    }

    // If the cached hint says this BSSID needs PMF off, bounce the connect
    // (WiFi.begin always reset pmf_cfg.capable=true) so the actual
    // association attempt has it off.
    if (h && (h->flags & HINT_FLAG_PMF_DISABLE)) {
        Log::logf(CAT_WIFI, LOG_DEBUG,
                  "PMF pre-disabled for '%s' (cached)\n", net.ssid.c_str());
        switch_to_pmf_disabled();
    }
}

// Connect to a specific BSSID/channel from the scan_candidates list.
// Used after process_scan_results when we want to target a particular AP
// (including same-SSID roam targets).
static void begin_connect_candidate(uint8_t cand_idx) {
    auto &cfg = Config::get();
    if (cand_idx >= scan_candidate_count) return;
    const ScanCandidate &c = scan_candidates[cand_idx];
    if (c.net_idx >= cfg.wifi_net_count) return;
    WiFiNetwork &net = cfg.wifi_nets[c.net_idx];

    connect_idx = c.net_idx;
    connect_retries = 0;
    pending_pmf_disable = false;
    set_state(WF_CONNECTING);

    Log::logf(CAT_WIFI, LOG_DEBUG,
              "Connecting to '%s' bssid=%02X:%02X:%02X:%02X:%02X:%02X ch=%d (%d dBm)\n",
              net.ssid.c_str(),
              c.bssid[0], c.bssid[1], c.bssid[2],
              c.bssid[3], c.bssid[4], c.bssid[5],
              c.channel, c.rssi);
    WiFi.begin(net.ssid.c_str(), net.pass.c_str(), c.channel, c.bssid);

    // Apply cached PMF flag for this specific BSSID, if any.
    NetworkHint *h = NetworkHints::find_exact(net.ssid.c_str(), c.bssid);
    if (h && (h->flags & HINT_FLAG_PMF_DISABLE)) {
        Log::logf(CAT_WIFI, LOG_DEBUG,
                  "PMF pre-disabled for this BSSID (cached)\n");
        switch_to_pmf_disabled();
    }
}

static void clear_scan() {
    scan_pending.store(false, std::memory_order_release);
    scan_done.store(false, std::memory_order_relaxed);
    esp_wifi_clear_ap_list();
}

static void start_scan(bool background = false) {
    if (scan_pending.load(std::memory_order_acquire)) return;
    // Arduino retries the first disconnect even with auto-reconnect disabled.
    // Recovery owns STA; a roaming scan must keep the current association.
    if (!background) stop_sta_attempt();
    scan_candidate_count = 0;
    scan_status.store(1, std::memory_order_relaxed);
    scan_done.store(false, std::memory_order_relaxed);
    scan_started_ms = millis();

    // Return to the home channel between channels, including under BLE
    // coexistence. Keep AP service alive while retrying STA in the background.
    wifi_scan_config_t scan = {};
    scan.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    scan.scan_time.active.max = SCAN_ACTIVE_MAX_MS;
    scan.home_chan_dwell_time = SCAN_HOME_DWELL_MS;
    scan.coex_background_scan = true;
    scan_pending.store(true, std::memory_order_release);

    esp_err_t err = esp_wifi_scan_start(&scan, false);
    if (err != ESP_OK) {
        clear_scan();
        Log::logf(CAT_WIFI, LOG_WARN, "Scan start failed: %s (%d)\n",
                  esp_err_to_name(err), err);
    }
}

static int16_t scan_complete() {
    if (!scan_pending.load(std::memory_order_acquire)) return WIFI_SCAN_FAILED;
    if (scan_done.load(std::memory_order_acquire)) {
        const uint32_t status = scan_status.load(std::memory_order_relaxed);
        if (status == 0) return 0;
        clear_scan();
        Log::logf(CAT_WIFI, LOG_WARN, "Scan failed status=%lu\n",
                  (unsigned long)status);
        return WIFI_SCAN_FAILED;
    }
    if (millis() - scan_started_ms >= SCAN_TIMEOUT_MS) {
        scan_pending.store(false, std::memory_order_release);
        esp_wifi_scan_stop();
        clear_scan();
        Log::logf(CAT_WIFI, LOG_WARN, "Scan timed out\n");
        return WIFI_SCAN_FAILED;
    }
    return WIFI_SCAN_RUNNING;
}

static void process_scan_results() {
    auto &cfg = Config::get();
    uint16_t n = 0;
    esp_err_t err = esp_wifi_scan_get_ap_num(&n);
    if (err != ESP_OK) {
        clear_scan();
        scan_candidate_count = 0;
        Log::logf(CAT_WIFI, LOG_WARN, "Scan results unavailable (err=%d)\n", err);
        return;
    }

    // Match visible APs against configured slots. No SSID dedup: each
    // visible BSSID becomes its own candidate so roaming can target a
    // specific AP under a same-SSID setup.
    scan_candidate_count = 0;
    for (int i = 0; i < n && scan_candidate_count < SCAN_CANDIDATES_MAX; i++) {
        wifi_ap_record_t ap = {};
        if (esp_wifi_scan_get_ap_record(&ap) != ESP_OK) break;
        uint8_t idx = find_net_by_ssid(reinterpret_cast<const char *>(ap.ssid));
        if (idx == 0xFF) continue;
        ScanCandidate &c = scan_candidates[scan_candidate_count++];
        c.net_idx = idx;
        memcpy(c.bssid, ap.bssid, sizeof(c.bssid));
        c.channel = ap.primary;
        c.rssi = ap.rssi;
    }

    // Sort by RSSI descending (selection sort, max 16 entries).
    for (uint8_t i = 0; i < scan_candidate_count; i++) {
        for (uint8_t j = i + 1; j < scan_candidate_count; j++) {
            if (scan_candidates[j].rssi > scan_candidates[i].rssi) {
                ScanCandidate tmp = scan_candidates[i];
                scan_candidates[i] = scan_candidates[j];
                scan_candidates[j] = tmp;
            }
        }
    }

    try_pos = 0;
    clear_scan();

    if (scan_candidate_count > 0) {
        const ScanCandidate &best = scan_candidates[0];
        Log::logf(CAT_WIFI, LOG_DEBUG,
                  "Scan: %d candidates of %d visible, best='%s' (%d dBm)\n",
                  scan_candidate_count, n,
                  cfg.wifi_nets[best.net_idx].ssid.c_str(), best.rssi);
    } else {
        Log::logf(CAT_WIFI, LOG_DEBUG, "Scan: 0 known of %d visible\n", n);
    }

    status_revision.fetch_add(1);
}

static void on_connected() {
    auto &cfg = Config::get();
    uint8_t *bssid = WiFi.BSSID();
    if (roam_connecting && bssid) {
        Log::logf(CAT_WIFI, LOG_INFO,
                  "Roaming complete: '%s' %02X:%02X:%02X:%02X:%02X:%02X, IP=%s\n",
                  WiFi.SSID().c_str(), bssid[0], bssid[1], bssid[2],
                  bssid[3], bssid[4], bssid[5], WiFi.localIP().toString().c_str());
    } else {
        Log::logf(CAT_WIFI, LOG_INFO, "Connected to '%s' (%s)\n",
                  WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    }
    roam_connecting = false;

    connect_idx = find_net_by_ssid(WiFi.SSID().c_str());

    // Persist the BSSID+channel we just connected on so the next reboot can
    // fast-path. If we got here via a PMF retry, mark that flag in the hint
    // so subsequent reconnects skip the doomed first attempt.
    if (bssid) {
        NetworkHints::upsert(WiFi.SSID().c_str(), bssid, WiFi.channel(),
                             pending_pmf_disable);
    }
    pending_pmf_disable = false;

    set_state(WF_CONNECTED);
    if (cfg.wifi_mode == WIFI_MODE_AUTO && (WiFi.getMode() & WIFI_AP)) {
        pending_ap_teardown = true;
        ap_quiet_since_ms = 0;
    }
    low_rssi_count = 0;
    last_roam_check = millis();
    // hint_refresh_pending was set by the same STA_GOT_IP that drove us here;
    // we just upserted, so the WF_CONNECTED-branch drain has nothing left to do.
    hint_refresh_pending = false;

    if (!ntp_done) {
        sync_ntp();
        ntp_done = true;
    }
}

static void enter_ap_fallback() {
    auto &cfg = Config::get();
    if (cfg.wifi_mode == WIFI_MODE_STA_ONLY) {
        // STA-only: no softAP, just wait and retry the scan loop.
        Log::logf(CAT_WIFI, LOG_DEBUG, "STA-only: retry in %d s\n",
                  AP_RETRY_INTERVAL_MS / 1000);
        set_state(WF_AP_FALLBACK);
        last_ap_retry = millis();
        return;
    }
    // AUTO / STA_AP: bring AP up (no-op if already up under STA_AP).
    String ap = ap_ssid_str();
    if (!(WiFi.getMode() & WIFI_AP)) {
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAP(ap.c_str(), "airbridge");
    }
    Log::logf(CAT_WIFI, LOG_INFO, "AP+STA fallback: %s (%s)\n",
              ap.c_str(), WiFi.softAPIP().toString().c_str());
    set_state(WF_AP_FALLBACK);
    last_ap_retry = millis();
}


static void apply_country_code() {
    auto &cfg = Config::get();
    if (cfg.wifi_country.length() < 2) return;
    esp_err_t err = esp_wifi_set_country_code(cfg.wifi_country.c_str(), true);
    if (err == ESP_OK) {
        Log::logf(CAT_WIFI, LOG_DEBUG, "Country code: %s\n",
                  cfg.wifi_country.c_str());
    } else {
        Log::logf(CAT_WIFI, LOG_WARN, "Country code '%s' rejected (err=%d)\n",
                  cfg.wifi_country.c_str(), err);
    }
}

bool WiFiSetup::init() {
    auto &cfg = Config::get();
    mdns_hostname = cfg.hostname;

    // STA_START reapplies Arduino's saved policy after every radio restart.
    if (!WiFi.setSleep(false)) {
        Log::logf(CAT_WIFI, LOG_WARN, "Disabling modem sleep failed\n");
    }

    WiFi.onEvent(wifi_event_cb);
    // This state machine owns recovery; do not race driver auto-association
    // against scans and the explicit BSSID/PMF retry paths.
    WiFi.setAutoReconnect(false);

    if (cfg.wifi_mode == WIFI_MODE_OFF) {
        WiFi.mode(WIFI_OFF);
        set_state(WF_OFF);
        return false;
    }

    WiFi.setHostname(cfg.hostname.c_str());

    if (cfg.wifi_mode == WIFI_MODE_AP_ONLY) {
        WiFi.mode(WIFI_AP);
        apply_country_code();
        String ap = ap_ssid_str();
        WiFi.softAP(ap.c_str(), "airbridge");
        delay(100);
        Log::logf(CAT_WIFI, LOG_INFO, "AP mode: %s %s\n",
                  ap.c_str(), WiFi.softAPIP().toString().c_str());
        set_state(WF_OFF);
        return true;
    }

    // AUTO, STA_ONLY, STA_AP all run STA. STA_AP also brings up softAP from boot.
    if (cfg.wifi_mode == WIFI_MODE_STA_AP) {
        WiFi.mode(WIFI_AP_STA);
        String ap = ap_ssid_str();
        WiFi.softAP(ap.c_str(), "airbridge");
        Log::logf(CAT_WIFI, LOG_INFO, "STA+AP mode, AP: %s %s\n",
                  ap.c_str(), WiFi.softAPIP().toString().c_str());
    } else {
        WiFi.mode(WIFI_STA);
    }
    apply_country_code();

    if (cfg.wifi_net_count > 0) {
        begin_connect(0, true);
        return true;
    }
    enter_ap_fallback();
    return true;
}

void WiFiSetup::request_reconfigure(bool network, bool clock) {
    reconfigure_pending.fetch_or((network ? 1 : 0) | (clock ? 2 : 0),
                                 std::memory_order_release);
}

void WiFiSetup::profiles_changed() {
    reconfigure_pending.fetch_or(4, std::memory_order_release);
}

static void apply_network_config() {
    status_revision.fetch_add(1);
    roam_connecting = false;
    const auto &cfg = Config::get();
    if (cfg.wifi_mode == WIFI_MODE_OFF || mdns_hostname != cfg.hostname) {
        stop_mdns();
        mdns_hostname = cfg.hostname;
    }
    if (scan_pending.load(std::memory_order_acquire)) esp_wifi_scan_stop();
    clear_scan();
    got_ip = false;
    sta_disconnected = false;
    hint_refresh_pending = false;
    pending_ap_teardown = false;
    scan_candidate_count = 0;
    connect_idx = 0xFF;

    if (cfg.wifi_mode == WIFI_MODE_OFF) {
        WiFi.mode(WIFI_OFF);
        set_state(WF_OFF);
        return;
    }
    const bool had_ap = WiFi.getMode() & WIFI_AP;
    if (cfg.wifi_mode == WIFI_MODE_AP_ONLY) {
        WiFi.mode(WIFI_AP);
        if (!had_ap) WiFi.softAP(ap_ssid_str().c_str(), "airbridge");
        apply_country_code();
        set_state(WF_OFF);
        return;
    }

    // Keep the AP and its clients while joining STA; normal quiet teardown
    // takes over after a successful AUTO connection.
    if (cfg.wifi_mode == WIFI_MODE_STA_ONLY) {
        WiFi.mode(WIFI_STA);
    } else {
        WiFi.mode(WIFI_AP_STA);
        if (!had_ap) WiFi.softAP(ap_ssid_str().c_str(), "airbridge");
    }
    WiFi.setHostname(cfg.hostname.c_str());
    apply_country_code();
    got_ip = false;
    sta_disconnected = false;
    for (uint8_t i = 0; i < cfg.wifi_net_count; i++) {
        if (!cfg.wifi_nets[i].enabled) continue;
        start_scan();
        set_state(WF_SCANNING);
        return;
    }
    stop_sta_attempt();
    enter_ap_fallback();
}

static void apply_profile_changes() {
    status_revision.fetch_add(1);
    wifi_config_t current = {};
    if (WiFi.status() == WL_CONNECTED &&
        esp_wifi_get_config(WIFI_IF_STA, &current) == ESP_OK) {
        const auto &cfg = Config::get();
        const size_t ssid_len = strnlen(reinterpret_cast<const char *>(current.sta.ssid),
                                        sizeof(current.sta.ssid));
        const size_t pass_len = strnlen(reinterpret_cast<const char *>(current.sta.password),
                                        sizeof(current.sta.password));
        for (uint8_t i = 0; i < cfg.wifi_net_count; i++) {
            const auto &net = cfg.wifi_nets[i];
            if (!net.enabled || net.ssid.length() != ssid_len || net.pass.length() != pass_len ||
                memcmp(net.ssid.c_str(), current.sta.ssid, ssid_len) ||
                memcmp(net.pass.c_str(), current.sta.password, pass_len)) continue;
            connect_idx = i;
            if (scan_pending.load(std::memory_order_acquire)) esp_wifi_scan_stop();
            clear_scan();
            scan_candidate_count = 0;
            roam_connecting = false;
            set_state(WF_CONNECTED);
            return;
        }
    }
    apply_network_config();
}

static void log_roaming_failed() {
    if (!roam_connecting) return;
    Log::logf(CAT_WIFI, LOG_WARN,
              "Roaming failed: %02X:%02X:%02X:%02X:%02X:%02X, connection timeout, last_reason=%u; resuming recovery\n",
              roam_target[0], roam_target[1], roam_target[2],
              roam_target[3], roam_target[4], roam_target[5],
              (unsigned)last_disconnect_reason);
    roam_connecting = false;
}

void WiFiSetup::check() {
    const uint8_t pending = reconfigure_pending.fetch_and(
        roaming_suspended ? uint8_t(~2) : uint8_t(0), std::memory_order_acq_rel);
    if (pending & 2) {
        const auto &cfg = Config::get();
        setenv("TZ", cfg.tz.isEmpty() ? "UTC0" : cfg.tz.c_str(), 1);
        tzset();
        ntp_done = false;
        if (WiFi.status() == WL_CONNECTED) {
            sync_ntp();
            ntp_done = true;
        }
    }
    if (!roaming_suspended) {
        if (pending & 1) apply_network_config();
        else if (pending & 4) apply_profile_changes();
    }

    refresh_ap_address();
    check_mdns();

    auto &cfg = Config::get();
    uint32_t elapsed = millis() - state_entered_ms;

    if (got_ip) {
        got_ip = false;
        sta_disconnected = false;
        if (wf_state == WF_HINT_TRY || wf_state == WF_CONNECTING ||
            wf_state == WF_PMF_RETRY) {
            on_connected();
        } else if (wf_state == WF_AP_FALLBACK) {
            on_connected();
        }
        return;
    }

    if (roaming_suspended) return;

    if (sta_disconnected && (wf_state == WF_CONNECTED || wf_state == WF_ROAM_SCAN)) {
        sta_disconnected = false;
        const auto reason = static_cast<wifi_err_reason_t>(last_disconnect_reason);
        Log::logf(CAT_WIFI, LOG_WARN, "Disconnected reason=%u (%s), scanning...\n",
                  (unsigned)reason, WiFi.STA.disconnectReasonName(reason));
        if (scan_pending.load(std::memory_order_acquire)) esp_wifi_scan_stop();
        clear_scan();
        start_scan();
        set_state(WF_SCANNING);
        return;
    }
    sta_disconnected = false;

    switch (wf_state) {
    case WF_OFF:
        break;

    case WF_HINT_TRY:
        if (elapsed > HINT_TIMEOUT_MS) {
            Log::logf(CAT_WIFI, LOG_DEBUG, "Hint timeout, full scan\n");
            start_scan();
            set_state(WF_SCANNING);
        }
        break;

    case WF_SCANNING: {
        int16_t result = scan_complete();
        if (result >= 0) {
            process_scan_results();
            if (scan_candidate_count > 0) {
                begin_connect_candidate(0);
            } else {
                enter_ap_fallback();
            }
        } else if (result == WIFI_SCAN_FAILED) {
            enter_ap_fallback();
        }
        break;
    }

    case WF_CONNECTING:
        if (elapsed > CONNECT_TIMEOUT_MS) {
            if (last_disconnect_reason == 208) {
                last_disconnect_reason = 0;
                enter_pmf_retry();
                break;
            }
            connect_retries++;
            if (connect_retries < CONNECT_RETRIES) {
                Log::logf(CAT_WIFI, LOG_DEBUG, "Connect timeout, retry=%d last_reason=%u\n",
                          connect_retries, (unsigned)last_disconnect_reason);
                // Keep the staged SSID/BSSID configuration. Calling WiFi.begin()
                // while STA is still connecting makes esp_wifi_set_config fail.
                retry_current_connect();
            } else {
                log_roaming_failed();
                stop_sta_attempt();
                try_pos++;
                if (try_pos < scan_candidate_count) {
                    Log::logf(CAT_WIFI, LOG_DEBUG, "Trying next candidate\n");
                    begin_connect_candidate(try_pos);
                } else {
                    Log::logf(CAT_WIFI, LOG_WARN, "All candidates exhausted, last_reason=%u\n",
                              (unsigned)last_disconnect_reason);
                    enter_ap_fallback();
                }
            }
        }
        break;

    case WF_PMF_RETRY:
        if (elapsed > CONNECT_TIMEOUT_MS) {
            log_roaming_failed();
            Log::logf(CAT_WIFI, LOG_WARN, "PMF retry timed out, last_reason=%u\n",
                      (unsigned)last_disconnect_reason);
            stop_sta_attempt();
            try_pos++;
            if (try_pos < scan_candidate_count) {
                begin_connect_candidate(try_pos);
            } else {
                enter_ap_fallback();
            }
        }
        break;

    case WF_CONNECTED:
        // periodic RSSI check for roaming
        if (cfg.wifi_roam && !roaming_suspended &&
            millis() - last_roam_check >= ROAM_CHECK_INTERVAL_MS) {
            last_roam_check = millis();
            int8_t rssi = WiFi.RSSI();
            if (rssi < ROAM_RSSI_THRESHOLD) {
                low_rssi_count++;
                Log::logf(CAT_WIFI, LOG_DEBUG, "Low RSSI %d dBm (%d/%d)\n",
                          rssi, low_rssi_count, ROAM_CONSECUTIVE_LOW);
                if (low_rssi_count >= ROAM_CONSECUTIVE_LOW) {
                    Log::logf(CAT_WIFI, LOG_DEBUG, "Roaming: scanning for better AP\n");
                    start_scan(true);
                    set_state(WF_ROAM_SCAN);
                }
            } else {
                low_rssi_count = 0;
            }
        }
        // Refresh the hint on any (re)association we caught via STA_GOT_IP,
        // including supplicant-driven reconnects we didn't initiate.
        if (hint_refresh_pending) {
            hint_refresh_pending = false;
            uint8_t *bssid = WiFi.BSSID();
            if (bssid) {
                NetworkHints::upsert(WiFi.SSID().c_str(), bssid,
                                     WiFi.channel(), false);
            }
        }
        // Pending AP teardown after recovering from WF_AP_FALLBACK. Wait for
        // any AP-side clients to disconnect for AP_TEARDOWN_QUIET_MS before
        // collapsing to STA-only.
        if (pending_ap_teardown) {
            if (ap_client_count == 0) {
                if (ap_quiet_since_ms == 0) ap_quiet_since_ms = millis();
                if (millis() - ap_quiet_since_ms >= AP_TEARDOWN_QUIET_MS) {
                    Log::logf(CAT_WIFI, LOG_INFO,
                              "AP teardown after %d s quiet\n",
                              AP_TEARDOWN_QUIET_MS / 1000);
                    WiFi.softAPdisconnect(true);
                    WiFi.mode(WIFI_STA);
                    status_revision.fetch_add(1);
                    pending_ap_teardown = false;
                    ap_quiet_since_ms = 0;
                }
            } else {
                ap_quiet_since_ms = 0;
            }
        }
        break;

    case WF_ROAM_SCAN: {
        int16_t result = scan_complete();
        if (result >= 0) {
            process_scan_results();
            // Compare best candidate to the current connection by BSSID,
            // not by SSID slot, so same-SSID multi-AP setups can roam.
            bool should_switch = false;
            uint8_t *cur_bssid = WiFi.BSSID();
            if (scan_candidate_count > 0 && cur_bssid &&
                memcmp(scan_candidates[0].bssid, cur_bssid, 6) != 0) {
                int8_t current_rssi = WiFi.RSSI();
                int8_t candidate_rssi = scan_candidates[0].rssi;
                if (candidate_rssi > current_rssi + ROAM_HYSTERESIS_DB) {
                    should_switch = true;
                    memcpy(roam_target, scan_candidates[0].bssid, sizeof(roam_target));
                    roam_connecting = true;
                    Log::logf(CAT_WIFI, LOG_INFO,
                              "Roaming '%s': %02X:%02X:%02X:%02X:%02X:%02X (%d dBm) -> "
                              "%02X:%02X:%02X:%02X:%02X:%02X (%d dBm)\n",
                              cfg.wifi_nets[scan_candidates[0].net_idx].ssid.c_str(),
                              cur_bssid[0], cur_bssid[1], cur_bssid[2],
                              cur_bssid[3], cur_bssid[4], cur_bssid[5], current_rssi,
                              roam_target[0], roam_target[1], roam_target[2],
                              roam_target[3], roam_target[4], roam_target[5], candidate_rssi);
                } else {
                    Log::logf(CAT_WIFI, LOG_DEBUG,
                              "Candidate %d dBm vs current %d dBm (<%d hysteresis), staying\n",
                              candidate_rssi, current_rssi, ROAM_HYSTERESIS_DB);
                }
            }
            if (should_switch) {
                WiFi.disconnect();
                delay(100);
                low_rssi_count = 0;
                begin_connect_candidate(0);
            } else {
                set_state(WF_CONNECTED);
                low_rssi_count = 0;
            }
        } else if (result == WIFI_SCAN_FAILED) {
            set_state(WF_CONNECTED);
            low_rssi_count = 0;
        }
        break;
    }

    case WF_AP_FALLBACK:
        if (!cfg.wifi_net_count) break;
        if (millis() - last_ap_retry >= AP_RETRY_INTERVAL_MS) {
            last_ap_retry = millis();
            Log::logf(CAT_WIFI, LOG_DEBUG, "AP fallback: retrying scan\n");
            start_scan();
            set_state(WF_SCANNING);
        }
        break;
    }
}

bool WiFiSetup::is_connected() {
    return wf_state == WF_CONNECTED && WiFi.status() == WL_CONNECTED;
}

bool WiFiSetup::time_synced() {
    return ntp_synced;
}

struct FallbackTime {
    time_t epoch;
    bool force;
    bool applied;
};

static esp_err_t apply_fallback_time(void *ctx) {
    auto *fallback = static_cast<FallbackTime *>(ctx);
    // Serialize with SNTP so a completed sync cannot be overwritten by fallback.
    if (ntp_synced && !fallback->force) return ESP_OK;
    struct timeval tv = { .tv_sec = fallback->epoch, .tv_usec = 0 };
    fallback->applied = settimeofday(&tv, nullptr) == 0;
    if (fallback->applied) ntp_synced = false;
    return ESP_OK;
}

bool WiFiSetup::set_fallback_time(int year, int month, int day, int hour, int min, int sec, bool force) {
    if (ntp_synced && !force) return false;

    auto &cfg = Config::get();
    if (cfg.tz.length() > 0) {
        setenv("TZ", cfg.tz.c_str(), 1);
        tzset();
    }

    struct tm t = {};
    t.tm_year = year - 1900;
    t.tm_mon = month - 1;
    t.tm_mday = day;
    t.tm_hour = hour;
    t.tm_min = min;
    t.tm_sec = sec;
    t.tm_isdst = -1;

    time_t epoch = mktime(&t);
    if (epoch < 0) return false;

    FallbackTime fallback = {epoch, force, false};
    // With WiFi disabled at boot there is no TCPIP task or SNTP client.
    if (WiFi.getMode() == WIFI_OFF) {
        apply_fallback_time(&fallback);
    } else if (esp_netif_tcpip_exec(apply_fallback_time, &fallback) != ESP_OK) {
        return false;
    }
    if (!fallback.applied) return false;

    Log::logf(CAT_TIME, LOG_INFO, "Fallback time from ResMed: %04d-%02d-%02d %02d:%02d\n",
              year, month, day, hour, min);
    return true;
}

void WiFiSetup::force_ntp_sync() {
    Log::logf(CAT_TIME, LOG_DEBUG, "Forcing NTP resync\n");
    sync_ntp();
}

void WiFiSetup::suspend_roaming() { roaming_suspended = true; }
void WiFiSetup::resume_roaming()  { roaming_suspended = false; }

uint32_t WiFiSetup::revision() { return status_revision.load(); }

void WiFiSetup::addresses(uint32_t &sta, uint32_t &ap) {
    sta = sta_address.load();
    ap = ap_address.load();
}

const char *WiFiSetup::state_name() {
    switch (wf_state) {
        case WF_OFF:          return "off";
        case WF_HINT_TRY:     return "hint";
        case WF_SCANNING:     return "scanning";
        case WF_CONNECTING:   return "connecting";
        case WF_PMF_RETRY:    return "pmf_retry";
        case WF_CONNECTED:    return "connected";
        case WF_ROAM_SCAN:    return "roaming";
        case WF_AP_FALLBACK:  return "ap_fallback";
        default:              return "?";
    }
}

int8_t WiFiSetup::current_rssi() {
    if (wf_state == WF_CONNECTED || wf_state == WF_ROAM_SCAN)
        return WiFi.RSSI();
    return 0;
}

const char *WiFiSetup::connected_ssid() {
    static char buf[33] = {};
    if (wf_state == WF_CONNECTED || wf_state == WF_ROAM_SCAN) {
        strncpy(buf, WiFi.SSID().c_str(), 32);
        return buf;
    }
    return "";
}

uint8_t WiFiSetup::connected_net_idx() {
    return (wf_state == WF_CONNECTED || wf_state == WF_ROAM_SCAN) ? connect_idx : 0xFF;
}

int8_t WiFiSetup::net_rssi(uint8_t idx) {
    if (idx >= WIFI_MAX_NETWORKS) return 0;
    // For the connected network, return live RSSI
    if (idx == connect_idx && (wf_state == WF_CONNECTED || wf_state == WF_ROAM_SCAN))
        return WiFi.RSSI();
    // Strongest RSSI seen in the last scan for any BSSID under this slot.
    int8_t best = 0;
    bool any = false;
    for (uint8_t i = 0; i < scan_candidate_count; i++) {
        if (scan_candidates[i].net_idx != idx) continue;
        if (!any || scan_candidates[i].rssi > best) {
            best = scan_candidates[i].rssi;
            any = true;
        }
    }
    return any ? best : 0;
}
