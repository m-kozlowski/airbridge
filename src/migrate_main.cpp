#ifdef FIRMWARE_MIGRATE
// Transitional firmware: runs migration, then waits for final OTA upload.

#include <Arduino.h>
#include "app_config.h"
#include "debug_log.h"
#include "uart_arbiter.h"
#include "wifi.h"
#include "airbridge_ota.h"
#include "migrate.h"

const char *airbridge_version()    { return "MIGRATE-" AIRBRIDGE_VERSION; }
const char *airbridge_build_date() { return AIRBRIDGE_BUILD_DATE; }

void dispatch_command(const char *, String &) {}

static void lct_display(const char *msg) {
    Arbiter::lcd_message(msg);
    Log::logf(CAT_OTA, LOG_INFO, "[MIG] LCD: %s\n", msg);
}

static bool ota_enabled = false;

void setup() {
    Serial.begin(115200);
    delay(500);
    while (Serial.available()) Serial.read();
    Log::init();

    Log::printf("\n=== AirBridge MIGRATE " AIRBRIDGE_VERSION " ===\n");
    Log::printf("Heap: %d bytes\n", ESP.getFreeHeap());

    Config::init();
    Config::load();

    Arbiter::init(Serial1, PIN_AS10_RX, PIN_AS10_TX, Config::get().uart_baud);
    Log::logf(CAT_INIT, LOG_INFO, "[INIT] UART arbiter started\n");

    delay(1000);

    Migrate::run(lct_display);

    Log::logf(CAT_INIT, LOG_INFO, "[MIG] Migration complete. Starting WiFi for OTA...\n");

    bool wifi_ok = WiFiSetup::init();
    if (wifi_ok) {
        OtaManager::init();
        ota_enabled = true;
        Log::logf(CAT_INIT, LOG_INFO, "[MIG] OTA ready. Upload target firmware now.\n");
        lct_display("OTA READY");
    } else {
        Log::logf(CAT_INIT, LOG_ERROR, "[MIG] WiFi failed! Cannot receive OTA.\n");
        lct_display("WIFI FAIL");
    }
}

void loop() {
    if (ota_enabled) {
        OtaManager::handle();
    }

    delay(10);
}

#endif
