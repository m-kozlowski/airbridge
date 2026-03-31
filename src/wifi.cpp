#include "wifi.h"
#include "app_config.h"
#include "debug_log.h"
#include <WiFi.h>
#include <esp_wifi.h>
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

        // STA failed - start AP+STA so device is reachable while retrying
        Log::logf(CAT_TCP, LOG_WARN, "\n[WIFI] STA connect failed, AP+STA fallback\n");
        ap_fallback = true;
        WiFi.mode(WIFI_AP_STA);
        String ap_ssid = cfg.hostname + "_" +
                         String((uint32_t)ESP.getEfuseMac(), HEX);
        WiFi.softAP(ap_ssid.c_str(), "airbridge");
        WiFi.begin(cfg.wifi_ssid.c_str(), cfg.wifi_pass.c_str());
        delay(100);
        Log::logf(CAT_TCP, LOG_INFO, "[WIFI] AP: %s (%s), STA retrying in background\n",
                  ap_ssid.c_str(), WiFi.softAPIP().toString().c_str());
        return true;
    }

    // Pure AP mode (explicit, or no SSID configured)
    WiFi.mode(WIFI_AP);
    String ap_ssid = cfg.hostname + "_" +
                     String((uint32_t)ESP.getEfuseMac(), HEX);
    WiFi.softAP(ap_ssid.c_str(), "airbridge");
    delay(100);

    if (cfg.wifi_ssid.length() == 0) {
        Log::logf(CAT_TCP, LOG_WARN, "[WIFI] No SSID configured, AP only: %s %s\n",
                  ap_ssid.c_str(), WiFi.softAPIP().toString().c_str());
    } else {
        Log::logf(CAT_TCP, LOG_INFO, "[WIFI] AP mode: %s %s\n",
                  ap_ssid.c_str(), WiFi.softAPIP().toString().c_str());
    }
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
