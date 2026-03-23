#include "airbridge_ota.h"
#include "uart_arbiter.h"
#include "debug_log.h"
#include "app_config.h"
#include <ArduinoOTA.h>


void OtaManager::init() {
    auto &cfg = Config::get();

    if (cfg.wifi_mode == 2) return;     // no WiFi, no OTA

    ArduinoOTA.setHostname(cfg.hostname.c_str());
    ArduinoOTA.setPort(DEFAULT_OTA_PORT);

    if (cfg.ota_password.length() > 0) {
        ArduinoOTA.setPassword(cfg.ota_password.c_str());
    }

    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "firmware" : "filesystem";
        Log::logf(CAT_OTA, LOG_INFO, "[OTA] Start updating %s\n", type.c_str());
        Arbiter::set_state(SYS_OTA_ESP);
    });

    ArduinoOTA.onEnd([]() {
        Log::logf(CAT_OTA, LOG_INFO, "\n[OTA] Complete, rebooting...\n");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static int last_pct = -1;
        int pct = progress / (total / 100);
        if (pct != last_pct) {
            Log::logf(CAT_OTA, LOG_DEBUG, "[OTA] %d%%\r", pct);
            last_pct = pct;
        }
    });

    ArduinoOTA.onError([](ota_error_t error) {
        const char *err_str = "Unknown";
        switch (error) {
            case OTA_AUTH_ERROR:    err_str = "Auth Failed"; break;
            case OTA_BEGIN_ERROR:   err_str = "Begin Failed"; break;
            case OTA_CONNECT_ERROR: err_str = "Connect Failed"; break;
            case OTA_RECEIVE_ERROR: err_str = "Receive Failed"; break;
            case OTA_END_ERROR:     err_str = "End Failed"; break;
        }
        Log::logf(CAT_OTA, LOG_ERROR, "[OTA] Error[%u]: %s\n", error, err_str);
        Arbiter::set_state(SYS_IDLE);
    });

    ArduinoOTA.begin();
    Log::logf(CAT_OTA, LOG_INFO, "[OTA] ArduinoOTA ready on port %d\n", DEFAULT_OTA_PORT);
}

void OtaManager::handle() {
    ArduinoOTA.handle();
}
