#include "wifi.h"
#include "app_config.h"
#include "debug_log.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_smartconfig.h>
#include <esp_sntp.h>
#include <time.h>

static bool sta_connected = false;
static volatile bool ntp_synced = false;
static bool ap_fallback = false;

static void ntp_sync_cb(struct timeval *tv) {
    ntp_synced = true;
    struct tm t;
    time_t now = time(nullptr);
    localtime_r(&now, &t);
    Log::logf(CAT_TCP, LOG_INFO, "[WIFI] NTP synced: %04d-%02d-%02d %02d:%02d:%02d\n",
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

    // NTP server priority:
    // 1. User-configured, ignore DHCP
    // 2. DHCP-provided
    // 3. pool.ntp.org

    if (esp_sntp_enabled()) esp_sntp_stop();

    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);

    if (cfg.ntp_server.length() > 0) {
        // User configured - explicit server, no DHCP
#if LWIP_DHCP_GET_NTP_SRV
        esp_sntp_servermode_dhcp(false);
#endif
        esp_sntp_setservername(0, cfg.ntp_server.c_str());
        Log::logf(CAT_TCP, LOG_INFO, "[WIFI] NTP: configured server %s\n",
                  cfg.ntp_server.c_str());
    } else {
        // Prefer DHCP, pool.ntp.org as fallback
#if LWIP_DHCP_GET_NTP_SRV
        esp_sntp_servermode_dhcp(true);
#endif
        esp_sntp_setservername(0, "pool.ntp.org");
        Log::logf(CAT_TCP, LOG_INFO, "[WIFI] NTP: DHCP + pool.ntp.org fallback\n");
    }

    esp_sntp_init();
}

// SmartConfig provisioning. Returns true if credentials received and connected.
static bool try_smartconfig(uint32_t timeout_ms = 60000) {
    Log::logf(CAT_TCP, LOG_INFO, "[WIFI] SmartConfig waiting (%ds)...\n", timeout_ms / 1000);

    WiFi.mode(WIFI_STA);
    WiFi.beginSmartConfig();

    uint32_t start = millis();
    while (!WiFi.smartConfigDone() && millis() - start < timeout_ms) {
        delay(500);
    }

    if (!WiFi.smartConfigDone()) {
        WiFi.stopSmartConfig();
        Log::logf(CAT_TCP, LOG_WARN, "[WIFI] SmartConfig timeout\n");
        return false;
    }

    Log::logf(CAT_TCP, LOG_INFO, "[WIFI] SmartConfig received credentials\n");

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
        delay(500);
    }

    if (WiFi.status() != WL_CONNECTED) {
        Log::logf(CAT_TCP, LOG_WARN, "[WIFI] SmartConfig connect failed\n");
        return false;
    }

    // save received credentials
    auto &cfg = Config::get();
    cfg.wifi_ssid = WiFi.SSID();
    cfg.wifi_pass = WiFi.psk();
    cfg.wifi_mode = 0;
    Config::save();
    Log::logf(CAT_TCP, LOG_INFO, "[WIFI] SmartConfig saved SSID='%s'\n", cfg.wifi_ssid.c_str());
    return true;
}

bool WiFiSetup::init() {
    auto &cfg = Config::get();

    if (cfg.wifi_mode == 2) {
        WiFi.mode(WIFI_OFF);
        return false;
    }

    WiFi.setHostname(cfg.hostname.c_str());

    if (cfg.wifi_mode == 0 && cfg.wifi_ssid.length() > 0) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(cfg.wifi_ssid.c_str(), cfg.wifi_pass.c_str());
        Log::logf(CAT_TCP, LOG_INFO, "[WIFI] Connecting to '%s'...\n", cfg.wifi_ssid.c_str());

        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 30) {
            delay(500);
            attempts++;
        }
        if (WiFi.status() == WL_CONNECTED) {
            Log::logf(CAT_TCP, LOG_INFO, "\n[WIFI] Connected: %s\n",
                      WiFi.localIP().toString().c_str());
            sta_connected = true;
            sync_ntp();
            return true;
        }

        // STA failed, try SmartConfig
        Log::logf(CAT_TCP, LOG_WARN, "\n[WIFI] STA connect failed\n");
        if (try_smartconfig()) {
            Log::logf(CAT_TCP, LOG_INFO, "[WIFI] Connected via SmartConfig: %s\n",
                      WiFi.localIP().toString().c_str());
            sta_connected = true;
            sync_ntp();
            return true;
        }

        // AP+STA fallback
        ap_fallback = true;
        WiFi.mode(WIFI_AP_STA);
        String ap_ssid = cfg.hostname + "_" +
                         String((uint32_t)ESP.getEfuseMac(), HEX);
        WiFi.softAP(ap_ssid.c_str(), "airbridge");
        WiFi.begin(cfg.wifi_ssid.c_str(), cfg.wifi_pass.c_str());
        delay(100);
        Log::logf(CAT_TCP, LOG_INFO, "[WIFI] AP+STA fallback: %s (%s)\n",
                  ap_ssid.c_str(), WiFi.softAPIP().toString().c_str());
        return true;
    }

    if (cfg.wifi_mode == 0 && cfg.wifi_ssid.length() == 0) {
        // No SSID configured. Use SmartConfig for initial provisioning
        if (try_smartconfig()) {
            Log::logf(CAT_TCP, LOG_INFO, "[WIFI] Provisioned via SmartConfig: %s\n",
                      WiFi.localIP().toString().c_str());
            sta_connected = true;
            sync_ntp();
            return true;
        }
    }

    // AP mode (explicit wifi_mode=1, or SmartConfig/STA failed with no SSID)
    WiFi.mode(WIFI_AP);
    String ap_ssid = cfg.hostname + "_" +
                     String((uint32_t)ESP.getEfuseMac(), HEX);
    WiFi.softAP(ap_ssid.c_str(), "airbridge");
    delay(100);
    Log::logf(CAT_TCP, LOG_INFO, "[WIFI] AP mode: %s %s\n",
              ap_ssid.c_str(), WiFi.softAPIP().toString().c_str());
    return true;
}

void WiFiSetup::check() {
    if (!ap_fallback || sta_connected) return;

    if (WiFi.status() == WL_CONNECTED) {
        sta_connected = true;
        ap_fallback = false;
        Log::logf(CAT_TCP, LOG_INFO, "[WIFI] STA connected in background: %s\n",
                  WiFi.localIP().toString().c_str());
        sync_ntp();
    }
}

bool WiFiSetup::is_connected() {
    return sta_connected && WiFi.status() == WL_CONNECTED;
}

bool WiFiSetup::time_synced() {
    return ntp_synced;
}


void WiFiSetup::set_fallback_time(int year, int month, int day, int hour, int min, int sec) {
    if (ntp_synced) return;

    struct tm t = {};
    t.tm_year = year - 1900;
    t.tm_mon = month - 1;
    t.tm_mday = day;
    t.tm_hour = hour;
    t.tm_min = min;
    t.tm_sec = sec;
    t.tm_isdst = -1;

    time_t epoch = mktime(&t);
    if (epoch < 0) return;

    struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
    settimeofday(&tv, nullptr);

    auto &cfg = Config::get();
    if (cfg.tz.length() > 0) {
        setenv("TZ", cfg.tz.c_str(), 1);
        tzset();
    }

    Log::logf(CAT_TCP, LOG_INFO, "[WIFI] Fallback time from ResMed: %04d-%02d-%02d %02d:%02d\n",
              year, month, day, hour, min);
}
