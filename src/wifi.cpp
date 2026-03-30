#include "wifi.h"
#include "app_config.h"
#include "debug_log.h"
#include <WiFi.h>
#include <esp_sntp.h>
#include <time.h>

static bool sta_connected = false;
static volatile bool ntp_synced = false;

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
    // 1. User-configured
    // 2. DHCP-provided
    // 3. pool.ntp.org

    if (cfg.ntp_server.length() > 0) {
        configTzTime(cfg.tz.c_str(), cfg.ntp_server.c_str());
        Log::logf(CAT_TCP, LOG_INFO, "[WIFI] NTP: configured server %s\n",
                  cfg.ntp_server.c_str());
    } else {
        // Check if DHCP provided a server
        const char *dhcp_srv = esp_sntp_getservername(1);
        if (dhcp_srv && dhcp_srv[0]) {
            configTzTime(cfg.tz.c_str(), dhcp_srv);
            Log::logf(CAT_TCP, LOG_INFO, "[WIFI] NTP: DHCP server %s\n", dhcp_srv);
        } else {
            configTzTime(cfg.tz.c_str(), "pool.ntp.org");
            Log::logf(CAT_TCP, LOG_INFO, "[WIFI] NTP: pool.ntp.org\n");
        }
    }
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
        } else {
            Log::logf(CAT_TCP, LOG_WARN, "\n[WIFI] STA connect failed, falling back to AP\n");
            cfg.wifi_mode = 1;
        }
    }

    // AP mode (explicit or fallback)
    WiFi.mode(WIFI_AP);
    String ap_ssid = cfg.hostname + "_" +
                     String((uint32_t)ESP.getEfuseMac(), HEX);
    WiFi.softAP(ap_ssid.c_str(), "airbridge");
    delay(100);

    if (cfg.wifi_mode == 1) {
        Log::logf(CAT_TCP, LOG_INFO, "[WIFI] AP mode: SSID=%s IP=%s\n",
                  ap_ssid.c_str(), WiFi.softAPIP().toString().c_str());
    } else {
        Log::logf(CAT_TCP, LOG_WARN, "[WIFI] No SSID configured, AP fallback: %s IP=%s\n",
                  ap_ssid.c_str(), WiFi.softAPIP().toString().c_str());
    }
    return true;
}

bool WiFiSetup::is_connected() {
    return sta_connected && WiFi.status() == WL_CONNECTED;
}

bool WiFiSetup::time_synced() {
    return ntp_synced;
}
