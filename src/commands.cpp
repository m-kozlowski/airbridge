#include <Arduino.h>
#include "uart_arbiter.h"
#include "oxi_ble.h"
#include "oxi_arbiter.h"
#include "tcp_bridge.h"
#include "resmed_ota.h"
#include "debug_log.h"
#include "app_config.h"
#include <esp_partition.h>
#include <time.h>
#include "wifi.h"


void dispatch_command(const char *line, String &response) {
    String cmd = String(line);
    cmd.trim();
    String upper = cmd;
    upper.toUpperCase();

    if (upper == "STATUS") {
        system_state_t sys = Arbiter::get_state();
        oxi_state_t oxi = OxiBle::get_state();
        const oxi_reading_t &r = OxiArbiter::get_reading();

        Config::refresh_device_info();
        auto &cfg = Config::get();

        response = "system: " + String(system_state_name(sys)) + "\n";
        if (!cfg.device_pna.isEmpty())
            response += "device: " + cfg.device_pna + " (" + cfg.device_srn + ")\n";
        response += "oxi: " + String(oxi_state_name(oxi)) + "\n";
        if (r.valid) {
            response += "spo2: " + String(r.spo2) + "%\n";
            response += "pulse: " + String(r.pulse_bpm) + " bpm\n";
            response += "age: " + String((millis() - r.timestamp_ms) / 1000) + "s\n";
        }
        response += "feeding: " + String(OxiArbiter::is_feeding() ? "yes" : "no") + "\n";
        response += "log_level: " + String(Log::level_name(Log::get_level())) + "\n";
        response += "uart_baud: " + String(Arbiter::get_baud()) + "\n";
        response += "uart_tx: " + String(Arbiter::get_tx_count()) + "\n";
        response += "uart_rx: " + String(Arbiter::get_rx_count()) + "\n";
        response += "uart_timeout: " + String(Arbiter::get_timeout_count()) + "\n";
        response += "uart_error: " + String(Arbiter::get_error_count()) + "\n";
        response += "heap: " + String(ESP.getFreeHeap()) + "\n";
        return;
    }

    if (upper.startsWith("OXI ")) {
        String sub = upper.substring(4);
        sub.trim();

        if (sub == "START") {
            OxiArbiter::start_feed();
            response = "OK: oximetry feed started\n";
        } else if (sub == "STOP") {
            OxiArbiter::stop_feed();
            response = "OK: oximetry feed stopped\n";
        } else if (sub == "STATUS") {
            oxi_state_t st = OxiBle::get_state();
            const oxi_reading_t &r = OxiArbiter::get_reading();
            response = "state: " + String(oxi_state_name(st)) + "\n";
            response += "feeding: " + String(OxiArbiter::is_feeding() ? "yes" : "no") + "\n";
            if (r.valid) {
                response += "spo2: " + String(r.spo2) + "\n";
                response += "pulse: " + String(r.pulse_bpm) + "\n";
            } else {
                response += "data: no valid reading\n";
            }
        } else if (sub == "SCAN") {
            OxiBle::start_scan();
            response = "OK: BLE scan started\n";
        } else if (sub == "RESULTS") {
            response = OxiBle::get_scan_results();
        } else if (sub.startsWith("CONNECT")) {
            String addr = cmd.substring(12);  // "OXI CONNECT <addr>"
            addr.trim();
            if (addr.length() > 0) {
                OxiBle::connect(addr.c_str());
            } else {
                OxiBle::connect(nullptr);
            }
            response = "OK: connecting...\n";
        } else if (sub == "DISCONNECT") {
            OxiBle::disconnect();
            response = "OK: disconnected\n";
        } else {
            response = "ERR: unknown OXI command: " + sub + "\n";
        }
        return;
    }

    if (upper.startsWith("CONFIG")) {
        String sub = cmd.substring(6);
        sub.trim();
        String subUpper = sub;
        subUpper.toUpperCase();

        if (subUpper == "DUMP" || sub.length() == 0) {
            response = Config::dump();
        } else if (subUpper == "SAVE") {
            Config::save();
            response = "OK: config saved to NVS\n";
        } else if (subUpper == "RESET") {
            Config::reset_defaults();
            response = "OK: config reset to defaults\n";
        } else {
            // "CONFIG key value" or "CONFIG key"
            int space = sub.indexOf(' ');
            if (space > 0) {
                String key = sub.substring(0, space);
                String val = sub.substring(space + 1);
                val.trim();
                if (Config::set_value(key.c_str(), val.c_str())) {
                    response = "OK: " + key + "=" + val + "\n";
                } else {
                    response = "ERR: unknown key '" + key + "'\n";
                }
            } else {
                // Get single key
                String val;
                if (Config::get_value(sub.c_str(), val)) {
                    response = sub + "=" + val + "\n";
                } else {
                    response = "ERR: unknown key '" + sub + "'\n";
                }
            }
        }
        return;
    }

    if (upper == "TIME") {
        time_t now = time(nullptr);
        struct tm utc, local;
        gmtime_r(&now, &utc);
        localtime_r(&now, &local);
        char ubuf[20], lbuf[20];
        strftime(ubuf, sizeof(ubuf), "%Y-%m-%d %H:%M:%S", &utc);
        strftime(lbuf, sizeof(lbuf), "%Y-%m-%d %H:%M:%S", &local);
        response = "utc:   " + String(ubuf) + "\n";
        response += "local: " + String(lbuf) + "\n";
        response += "tz:    " + Config::get().tz + "\n";
        response += "ntp:   " + String(WiFiSetup::time_synced() ? "synced" : "not synced") + "\n";
        response += "epoch: " + String((uint32_t)now) + "\n";
        return;
    }

    if (upper == "TIMESYNC") {
        extern void reset_resmed_time_sync();
        reset_resmed_time_sync();
        response = "OK: resmed clock sync will retry\n";
        return;
    }

    if (upper == "VERSION") {
        response = "AirBridge " + String(AIRBRIDGE_VERSION) + "\n";
        response += "Built: " + String(AIRBRIDGE_BUILD_DATE) + "\n";
        response += "ESP32 SDK: " + String(ESP.getSdkVersion()) + "\n";
        response += "Chip: " + String(ESP.getChipModel()) + " rev" + String(ESP.getChipRevision()) + "\n";
        response += "Flash: " + String(ESP.getFlashChipSize() / 1024) + "KB\n";
        return;
    }

    if (upper == "REBOOT") {
        response = "OK: rebooting...\n";
        delay(100);
        ESP.restart();
        return;
    }

    if (upper == "LOG" || upper.startsWith("LOG ")) {
        String sub = upper.substring(3);
        sub.trim();

        if (sub.length() == 0) {
            for (int i = 0; i < CAT_COUNT; i++) {
                response += String(Log::cat_name((log_cat_t)i)) + "=" +
                           String(Log::level_name(Log::get_cat_level((log_cat_t)i))) + "\n";
            }
            return;
        }

        // Parse: $LOG [category] level  OR  $LOG level (sets all)
        int sp = sub.indexOf(' ');
        String cat_str, lvl_str;
        if (sp > 0) {
            cat_str = sub.substring(0, sp);
            lvl_str = sub.substring(sp + 1);
            lvl_str.trim();
        } else {
            lvl_str = sub;
        }

        log_level_t lvl;
        if (lvl_str == "ERROR")      lvl = LOG_ERROR;
        else if (lvl_str == "WARN")  lvl = LOG_WARN;
        else if (lvl_str == "INFO")  lvl = LOG_INFO;
        else if (lvl_str == "DEBUG") lvl = LOG_DEBUG;
        else {
            response = "ERR: valid levels: ERROR WARN INFO DEBUG\n";
            return;
        }

        if (cat_str.length() == 0 || cat_str == "ALL") {
            Log::set_level(lvl);
            response = "OK: all categories set to " + String(Log::level_name(lvl)) + "\n";
        } else {
            bool found = false;
            for (int i = 0; i < CAT_COUNT; i++) {
                if (cat_str.equalsIgnoreCase(Log::cat_name((log_cat_t)i))) {
                    Log::set_cat_level((log_cat_t)i, lvl);
                    response = "OK: " + String(Log::cat_name((log_cat_t)i)) +
                              " set to " + String(Log::level_name(lvl)) + "\n";
                    found = true;
                    break;
                }
            }
            if (!found) {
                response = "ERR: unknown category '" + cat_str + "'. Valid: ";
                for (int i = 0; i < CAT_COUNT; i++) {
                    if (i > 0) response += " ";
                    response += Log::cat_name((log_cat_t)i);
                }
                response += "\n";
            }
        }
        return;
    }

    if (upper == "FLASH" || upper.startsWith("FLASH ")) {
        String sub = upper.substring(5);
        sub.trim();

        if (sub == "STATUS") {
            response = "active: " + String(ResmedOta::is_active() ? "yes" : "no") + "\n";
            response += "phase: " + String(ResmedOta::get_phase()) + "\n";
            response += "sent: " + String(ResmedOta::get_sent()) + "\n";
            response += "total: " + String(ResmedOta::get_total()) + "\n";
            const char *err = ResmedOta::last_error();
            if (err && err[0]) response += "error: " + String(err) + "\n";
        } else if (sub == "CANCEL") {
            ResmedOta::cancel();
            response = "OK: flash cancelled\n";
        } else if (sub.length() == 0) {
            response = "Usage: FLASH [FULL|CMX|CDX|CCX|BLX] [BLX] [FORCE]\n"
                       "       FLASH STATUS | CANCEL\n"
                       "Upload firmware first via HTTP, then flash.\n";
        } else {
            if (ResmedOta::is_active()) {
                response = "ERR: flash already in progress\n";
                return;
            }

            // Parse: FLASH [block] [BLX] [FORCE]
            String block = "";
            bool flash_blx = false;
            bool force_blx = false;

            int pos = 0;
            while (pos < (int)sub.length()) {
                int sp = sub.indexOf(' ', pos);
                if (sp < 0) sp = sub.length();
                String tok = sub.substring(pos, sp);
                tok.trim();

                if (tok == "BLX" && block.length() > 0) {
                    flash_blx = true;
                } else if (tok == "FORCE") {
                    force_blx = true;
                } else if (block.length() == 0) {
                    block = tok;
                }
                pos = sp + 1;
            }

            extern size_t uploadSize;
            size_t fw_size = uploadSize;
            if (fw_size == 0) {
                const esp_partition_t *part = ResmedOta::get_staging_partition();
                if (!part) {
                    response = "ERR: no staging partition found\n";
                    return;
                }
                fw_size = part->size;
            }

            if (block == "BLX") {
                flash_blx = false;  // not relevant, standalone BLX
            }

            // safety checks
            bool has_warnings = false;
            const esp_partition_t *staging = ResmedOta::get_staging_partition();
            if (staging) {
                fw_verify_result_t v = ResmedOta::verify_image(staging, fw_size);
                if (v.has_blx && !v.bid_ok) {
                    response += "WARN: Unknown BID: " + String(v.bid) + " (expected SX577-0200)\n";
                    has_warnings = true;
                }
                if (v.has_blx && !v.blx_crc_ok) { response += "WARN: BLX CRC mismatch\n"; has_warnings = true; }
                if (v.has_ccx && !v.ccx_crc_ok) { response += "WARN: CCX CRC mismatch\n"; has_warnings = true; }
                if (v.has_cdx && !v.cdx_crc_ok) { response += "WARN: CDX CRC mismatch\n"; has_warnings = true; }
                if (v.blx_patch == BLX_PATCH_A_DANGEROUS) {
                    response += "!!! DANGER: Bootloader disables serial flash — needs SWD to recover !!!\n";
                    has_warnings = true;
                } else if (v.blx_patch == BLX_PATCH_B_SAFE) {
                    response += "INFO: Bootloader integrity check disabled (safe method)\n";
                }
            }

            if (has_warnings && !force_blx) {
                response += "ERR: safety checks failed. Add FORCE to override.\n";
                return;
            }

            ResmedOta::start_flash(
                block.length() > 0 ? block.c_str() : nullptr,
                fw_size,
                flash_blx,
                force_blx
            );

            const char *detected = block.length() > 0 ? block.c_str()
                                    : ResmedOta::detect_block(fw_size);
            response += "OK: flashing " + String(detected ? detected : "auto") +
                       " (" + String(fw_size) + " bytes)";
            if (flash_blx) response += " +BLX";
            if (force_blx) response += " FORCE";
            response += "\nUse $FLASH STATUS to monitor progress.\n";
        }
        return;
    }

    if (upper == "HELP" || upper == "?") {
        response = "Commands (prefix with $):\n"
                   "  STATUS              System + oximetry status\n"
                   "  OXI START|STOP      Start/stop oximetry injection\n"
                   "  OXI STATUS          Oximeter connection info\n"
                   "  OXI SCAN            Scan for BLE oximeters\n"
                   "  OXI RESULTS         Show scan results\n"
                   "  OXI CONNECT [addr]  Connect to oximeter\n"
                   "  OXI DISCONNECT      Disconnect oximeter\n"
                   "  CONFIG [key [val]]  Get/set config\n"
                   "  CONFIG SAVE|RESET   Save/reset config\n"
                   "  CONFIG DUMP         Show all config\n"
                   "  FLASH [block] [BLX] [FORCE]  Flash uploaded firmware\n"
                   "  FLASH STATUS|CANCEL Monitor/cancel flash\n"
                   "  LOG                 Show all category log levels\n"
                   "  LOG [cat] level     Set log level (cats: OXI TCP OTA WEB ARB HEALTH ALL)\n"
                   "  TRANSPARENT         Enter raw UART mode\n"
                   "  VERSION             Firmware version info\n"
                   "  REBOOT              Restart ESP32\n"
                   "  HELP                This help\n"
                   "Anything without $ prefix is sent to AirSense.\n";
        return;
    }

    response = "ERR: unknown command '" + String(line) + "' (try $HELP)\n";
}
