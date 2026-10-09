#include <Arduino.h>
#include "uart_arbiter.h"
#include "oxi_ble.h"
#include "oxi_arbiter.h"
#include "tcp_bridge.h"
#include "resmed_ota.h"
#include "debug_log.h"
#include "app_config.h"
#include "device_status.h"
#include "build_info.h"
#include "airbridge_ota.h"
#include "factory_reset.h"
#include <esp_partition.h>
#include <time.h>
#include "wifi_setup.h"
#include "air10_clock.h"
#include "network_hints.h"
#include "live_stream.h"
#include "live_tce.h"
#include "export_sync.h"
#include "board.h"
#include "storage_browser.h"
#include "sd_storage.h"
#include "memory_manager.h"
#include "crash_diagnostics.h"


static String parse_quoted_token(const String &s, int *pos) {
    int i = *pos;
    while (i < (int)s.length() && isspace((unsigned char)s[i])) i++;
    if (i >= (int)s.length()) { *pos = i; return ""; }

    String out;
    if (s[i] == '"') {
        i++;
        int start = i;
        while (i < (int)s.length() && s[i] != '"') i++;
        out = s.substring(start, i);
        if (i < (int)s.length()) i++;  // consume closing quote
    } else {
        int start = i;
        while (i < (int)s.length() && !isspace((unsigned char)s[i])) i++;
        out = s.substring(start, i);
    }
    *pos = i;
    return out;
}


void dispatch_command(const char *line, String &response) {
    String cmd = String(line);
    cmd.trim();
    String upper = cmd;
    upper.toUpperCase();

    if (upper == "STATUS") {
        const auto status = DeviceStatus::snapshot();
        const auto &r = status.reading;
        AirSenseState::Identity identity;
        AirSenseState::identity(identity);

        response.reserve(512);
        response = "system: " + String(system_state_name(status.sys)) + "\n";
        if (identity.valid)
            response += "device: " + String(identity.pna) + " (" + identity.srn + ")\n";
        response += "oxi: " + String(oxi_state_name(status.oxi)) + "\n";
        if (r.valid) {
            response += "spo2: " + String(r.spo2) + "%\n";
            response += "pulse: " + String(r.pulse_bpm) + " bpm\n";
            response += "age: " + String((millis() - r.timestamp_ms) / 1000) + "s\n";
        }
        response += "feeding: " + String(status.feeding ? "yes" : "no") + "\n";
        response += "log_level: " + String(Log::level_name(Log::get_level())) + "\n";
        response += "uart_baud: " + String(Arbiter::get_baud()) + "\n";
        response += "uart_tx: " + String(Arbiter::get_tx_count()) + "\n";
        response += "uart_rx: " + String(Arbiter::get_rx_count()) + "\n";
        response += "uart_l_rx: " + String(Arbiter::get_l_rx_count()) + "\n";
        response += "uart_timeout: " + String(Arbiter::get_timeout_count()) + "\n";
        response += "uart_error: " + String(Arbiter::get_error_count()) + "\n";
        response += "live_tce: " +
                    String(LiveStream::is_stream_active(LiveTce::TAG) ? "subscribed" : "idle") +
                    "\n";

        const auto memory = aircannect::Memory::status();
        response += "heap: " + String(memory.heap_free) + "\n";
        response += "heap_largest: " + String(memory.heap_max_alloc) + "\n";
        if (memory.psram_available) {
            response += "psram: " + String(memory.psram_free) + " bytes free / " +
                        String(memory.psram_total) + " bytes total\n";
            response += "psram_largest: " + String(memory.psram_max_alloc) + " bytes\n";
        } else {
            response += "psram: unavailable\n";
        }
        return;
    }

    if (upper == "STORAGE" || upper.startsWith("STORAGE ")) {
        bool quoted = false;
        for (unsigned i = 0; i < cmd.length(); i++) if (cmd[i] == '"') quoted = !quoted;
        if (quoted) { response = "ERR: unmatched quote\n"; return; }
        int position = 7;
        String action = parse_quoted_token(cmd, &position);
        action.toUpperCase();
        const String path = parse_quoted_token(cmd, &position);
        if ((action.isEmpty() || action == "STATUS") && path.isEmpty()) {
            SdStorage::Status sd;
            SdStorage::get_status(sd);
            response = "state: ";
            response += SdStorage::state_name(sd);
            response += '\n';
            if (sd.mounted) {
                if (sd.total_bytes && sd.used_bytes <= sd.total_bytes) {
                    response += "capacity: " +
                                String(static_cast<double>(sd.total_bytes) / 1048576, 1) + " MiB\n";
                    response += "used: " +
                                String(static_cast<double>(sd.used_bytes) / 1048576, 1) + " MiB\n";
                    response += "free: " +
                                String(static_cast<double>(sd.total_bytes - sd.used_bytes) / 1048576, 1) + " MiB\n";
                } else {
                    response += "space: unavailable\n";
                }
            }
            if (sd.error[0]) response += "error: " + String(sd.error) + "\n";
#if AB_STORAGE_HAS_SDCARD
            StorageBrowser::MutationStatus status;
            StorageBrowser::mutation_status(status);
            if (status.active || status.succeeded || status.error[0]) {
                response += "operation: ";
                response += status.active ? "working" : status.succeeded ? "done" : "failed";
                response += "\nchanged: " + String(status.changed) + " entries\n";
                if (status.error[0]) response += "operation_error: " + String(status.error) + "\n";
            }
#endif
            return;
        }
        if (action == "USB") {
            String value = path;
            value.toUpperCase();
            if ((value != "ON" && value != "OFF") || !parse_quoted_token(cmd, &position).isEmpty()) {
                response = "ERR: usage: $STORAGE USB ON|OFF\n";
                return;
            }
            const char *error = nullptr;
            response = SdStorage::request_usb(value == "ON", &error)
                ? "OK: USB handoff requested; $STORAGE STATUS for result\n"
                : "ERR: " + String(error) + "\n";
            return;
        }
#if AB_STORAGE_HAS_SDCARD
        const String name = parse_quoted_token(cmd, &position);
        const String extra = parse_quoted_token(cmd, &position);
        StorageBrowser::Request request = {};
        const bool rename = action == "RENAME";
        if ((!rename && action != "RM") || path.isEmpty() ||
            path.length() >= sizeof(request.path) ||
            (rename ? name.isEmpty() || name.length() >= 256 : !name.isEmpty()) ||
            !extra.isEmpty()) {
            response = "ERR: usage: $STORAGE RENAME /path new_name | RM /path | STATUS\n";
            return;
        }
        request.kind = rename ? StorageBrowser::Kind::Rename : StorageBrowser::Kind::Delete;
        strcpy(request.path, path.c_str());
        strcpy(request.selection, name.c_str());
        std::weak_ptr<StorageBrowser::Transfer> active;
        const auto result = StorageBrowser::start(request,
            [](int, const char *, std::shared_ptr<StorageBrowser::Transfer>, uint64_t) {}, active);
        switch (result) {
        case StorageBrowser::StartResult::Started:
            response = "OK: storage operation started; $STORAGE STATUS for result\n"; break;
        case StorageBrowser::StartResult::BadRequest:
            response = "ERR: invalid path or name\n"; break;
        case StorageBrowser::StartResult::Busy:
            response = "ERR: storage busy\n"; break;
        default:
            response = "ERR: storage unavailable\n"; break;
        }
#else
        response = "ERR: storage unsupported\n";
#endif
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
            oxi_reading_t r;
            OxiArbiter::snapshot(r);
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
            oxi_scan_result_t devs[MAX_SCAN_RESULTS];
            int count = OxiBle::get_scan_results(devs, MAX_SCAN_RESULTS);
            if (count == 0) {
                response = "(no oximeters found)\n";
            } else {
                for (int i = 0; i < count; i++) {
                    response += devs[i].addr;
                    response += ' ';
                    response += devs[i].name;
                    response += " RSSI=";
                    response += devs[i].rssi;
                    response += '\n';
                }
            }
        } else if (sub == "KNOWN") {
            oxi_known_device_t devices[MAX_KNOWN_DEVICES];
            int count = OxiBle::get_known_devices(devices, MAX_KNOWN_DEVICES);
            if (!count) response = "(no known sensors)\n";
            for (int i = 0; i < count; i++) {
                response += String(devices[i].addr) + " autoconnect=" +
                            (devices[i].autoconnect ? "on" : "off");
                if (devices[i].name[0]) response += " name=" + String(devices[i].name);
                response += '\n';
            }
        } else if (sub.startsWith("AUTOCONNECT ")) {
            String args = cmd.substring(16);
            args.trim();
            int space = args.indexOf(' ');
            String addr = space > 0 ? args.substring(0, space) : "";
            String value = space > 0 ? args.substring(space + 1) : "";
            value.trim();
            value.toUpperCase();
            if (value != "ON" && value != "OFF") {
                response = "ERR: OXI AUTOCONNECT <addr> ON|OFF\n";
            } else {
                response = OxiBle::set_autoconnect(addr.c_str(), value == "ON")
                    ? "OK: autoconnect saved\n" : "ERR: unknown sensor or settings write failed\n";
            }
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
        } else if (sub == "ENABLE") {
            OxiBle::enable();
            response = "OK: oximetry enabled\n";
        } else if (sub == "DISABLE") {
            OxiBle::disable();
            response = "OK: oximetry disabled\n";
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
            response = Config::save() ? "OK: config saved to NVS\n" :
                                        "ERROR: NVS save failed\n";
        } else if (subUpper == "RESET") {
            Config::reset_defaults();
            OtaManager::config_changed();
            response = "OK: config reset to defaults\n";
        } else {
            // "CONFIG key value" or "CONFIG key"
            int space = sub.indexOf(' ');
            if (space > 0) {
                String key = sub.substring(0, space);
                String val = sub.substring(space + 1);
                val.trim();
                if (Config::set_value(key.c_str(), val.c_str())) {
                    if (key.equalsIgnoreCase("update_url"))
                        OtaManager::config_changed();
                    response = "OK: " + key + "=" +
                               (Config::is_sensitive(key.c_str()) && val.length()
                                    ? "****" : val) + "\n";
                } else {
                    response = "ERR: unknown key '" + key + "'\n";
                }
            } else {
                // Get single key
                String val;
                if (Config::get_value(sub.c_str(), val)) {
                    response = sub + "=" +
                               (Config::is_sensitive(sub.c_str()) && val.length()
                                    ? "****" : val) + "\n";
                } else {
                    response = "ERR: unknown key '" + sub + "'\n";
                }
            }
        }
        return;
    }

    if (upper.startsWith("EXPORT")) {
        String sub = cmd.substring(6);
        sub.trim();
        sub.toUpperCase();
        if (sub.length() == 0 || sub == "STATUS") {
            ExportSync::Status export_status;
            ExportSync::get_status(export_status);
            response = "smb state=" + String(ExportSync::state_name(
                export_status.state));
            response += " files=" + String(export_status.files_uploaded);
            response += "/" + String(export_status.files_seen);
            response += " skipped=" + String(export_status.files_skipped);
            response += " bytes=" + String(
                static_cast<unsigned long long>(export_status.bytes_uploaded));
            if (export_status.current_day[0])
                response += " day=" + String(export_status.current_day);
            if (export_status.last_error[0])
                response += " error=" + String(export_status.last_error);
            response += "\n";
            ExportSync::SleepHqStatus sleephq_status;
            ExportSync::get_sleephq_status(sleephq_status);
            response += "sleephq state=" + String(ExportSync::state_name(
                sleephq_status.state));
            response += " files=" + String(sleephq_status.files_uploaded);
            response += "/" + String(sleephq_status.files_seen);
            response += " skipped=" + String(sleephq_status.files_skipped);
            response += " bytes=" + String(static_cast<unsigned long long>(
                sleephq_status.bytes_uploaded));
            if (sleephq_status.import_id)
                response += " import=" + String(sleephq_status.import_id);
            if (sleephq_status.import_status[0])
                response += " import_status=" +
                            String(sleephq_status.import_status);
            if (sleephq_status.current_day[0])
                response += " day=" + String(sleephq_status.current_day);
            if (sleephq_status.last_error[0])
                response += " error=" + String(sleephq_status.last_error);
            response += "\n";
        } else if (sub == "SMB" || sub == "SLEEPHQ") {
            const char *error = nullptr;
            const bool smb = sub == "SMB";
            const bool queued = smb ? ExportSync::request_manual_smb(false, &error)
                                    : ExportSync::request_manual_sleephq(false, &error);
            response = queued ? (smb ? "OK: SMB sync queued\n" : "OK: SleepHQ sync queued\n")
                              : String("ERR: ") + error + "\n";
        } else {
            response = "ERR: use EXPORT STATUS, EXPORT SMB, or "
                       "EXPORT SLEEPHQ\n";
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
        Air10Clock::request_sync(true);
        response = "OK: resmed clock sync will retry\n";
        return;
    }

    if (upper == "VERSION") {
        response = "AirBridge " + String(airbridge_version()) + "\n";
        response += "Built: " + String(airbridge_build_date()) + "\n";
        response += "ESP32 SDK: " + String(ESP.getSdkVersion()) + "\n";
        response += "Chip: " + String(ESP.getChipModel()) + " rev" + String(ESP.getChipRevision()) + "\n";
        response += "Flash: " + String(ESP.getFlashChipSize() / 1024) + "KB\n";
        return;
    }

    if (upper == "REBOOT") {
        response = OtaManager::request_reboot() ? "OK: reboot queued\n"
                                              : "ERR: OTA/reboot unavailable or busy\n";
        return;
    }

    if (upper == "FACTORYRESET" || upper.startsWith("FACTORYRESET ")) {
        String confirmation = upper.substring(12);
        confirmation.trim();
        if (confirmation != "CONFIRM") {
            response = "ERR: usage: $FACTORYRESET CONFIRM\n"
                       "Erases all NVS settings, WiFi profiles and BLE bonds/known sensors; ";
#if AB_STORAGE_HAS_SDCARD
            response += "replaces the SD partition table and formats FAT32, erasing ALL SD data. ";
#else
            response += "this build resets NVS only (no SD support). ";
#endif
            response += "Restarts; firmware, ESP partition table and crash dump remain unchanged.\n";
            return;
        }
        const char *error = nullptr;
        response = FactoryReset::request(&error)
            ? "OK: factory reset queued; restarting\n"
            : String("ERR: ") + error + "\n";
        return;
    }

    if (upper == "CRASH" || upper.startsWith("CRASH ")) {
        String action = upper.substring(5);
        action.trim();
        if (action == "CLEAR") {
            const char *error = CrashDiagnostics::clear();
            response = error ? "ERR: " + String(error) + "\n"
                             : "OK: crash dump cleared\n";
            return;
        }
        if (!action.isEmpty() && action != "STATUS" && action != "SUMMARY") {
            response = "ERR: CRASH [STATUS|SUMMARY|CLEAR]\n";
            return;
        }
        CrashDiagnostics::Snapshot crash;
        if (!CrashDiagnostics::snapshot(crash)) {
            response = "ERR: crash diagnostics busy\n";
            return;
        }
        response = "state: " + String(CrashDiagnostics::state_name(crash.state)) + "\n";
        response += "size: " + String(crash.size) + " bytes\n";
        if (crash.stored_size && !crash.size)
            response += "stored_size: " + String(crash.stored_size) + " bytes\n";
        if (crash.error != ESP_OK)
            response += "error: " + String(esp_err_to_name(crash.error)) + "\n";
        if (action != "SUMMARY") return;
        if (!crash.summary_available) {
            response += "summary: unavailable\n";
            return;
        }
        response += "task: " + String(crash.task) + "\n";
        response += "reason: " + String(crash.reason) + "\n";
        response += "pc: 0x" + String(crash.pc, HEX) + "\n";
        response += "cause: " + String(crash.cause) + "\n";
        response += "exception_address: 0x" + String(crash.exception_address, HEX) + "\n";
        response += "elf_sha: " + String(crash.elf_sha) + "\n";
        response += "backtrace_corrupt: " + String(crash.backtrace_corrupt ? "yes" : "no") + "\n";
        response += "backtrace:";
        for (size_t i = 0; i < crash.backtrace_depth; ++i)
            response += " 0x" + String(crash.backtrace[i], HEX);
        response += '\n';
        return;
    }

    if (upper == "RESETREASON") {
        esp_reset_reason_t reason = esp_reset_reason();
        const char *name = Log::reset_reason_name();
        response = "reset reason: " + String(name) + " (" + String(reason) + ")\n";
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

    if (upper == "WIFI" || upper.startsWith("WIFI ")) {
        String sub = upper.substring(4);
        sub.trim();
        auto &wfg = Config::get();
        if (sub == "" || sub == "STATUS") {
            response = "state: " + String(WiFiSetup::state_name()) + "\n";
            response += "ssid: " + String(WiFiSetup::connected_ssid()) + "\n";
            response += "rssi: " + String(WiFiSetup::current_rssi()) + " dBm\n";
            response += "roaming: " + String(wfg.wifi_roam ? "enabled" : "disabled") + "\n";
        } else if (sub == "LIST") {
            if (wfg.wifi_net_count == 0) {
                response = "(no networks configured)\n";
            } else {
                for (int i = 0; i < wfg.wifi_net_count; i++) {
                    response += String(i) + ": " + wfg.wifi_nets[i].ssid;
                    if (i == WiFiSetup::connected_net_idx()) response += " [connected]";
                    if (!wfg.wifi_nets[i].enabled) response += " [disabled]";
                    response += "\n";
                }
            }
        } else if (sub == "HINTS") {
            int n = NetworkHints::count();
            if (n == 0) {
                response = "(no cached hints)\n";
            } else {
                for (int i = 0; i < n; i++) {
                    const NetworkHint *h = NetworkHints::at(i);
                    if (!h) continue;
                    char line[96];
                    snprintf(line, sizeof(line),
                             "%d: %s ch=%d bssid=%02X:%02X:%02X:%02X:%02X:%02X flags=0x%02X age=%lus\n",
                             i, h->ssid, h->channel,
                             h->bssid[0], h->bssid[1], h->bssid[2],
                             h->bssid[3], h->bssid[4], h->bssid[5],
                             h->flags,
                             (unsigned long)((millis() - h->last_used_ms) / 1000UL));
                    response += line;
                }
            }
        } else if (sub == "HINTS CLEAR") {
            NetworkHints::clear_all();
            response = "OK: hints cleared\n";
        } else if (sub.startsWith("ADD ")) {
            // WIFI ADD ssid password
            String args = cmd.substring(upper.indexOf("ADD ") + 4);
            int pos = 0;
            String ssid = parse_quoted_token(args, &pos);
            String pass = parse_quoted_token(args, &pos);
            if (ssid.length() == 0)
                response = "ERR: empty SSID\n";
            else if (Config::add_network(ssid.c_str(), pass.c_str()))
                response = "OK: added '" + ssid + "'\n";
            else
                response = "ERR: list full or NVS save failed\n";
        } else if (sub.startsWith("REMOVE ")) {
            int idx = sub.substring(7).toInt();
            if (Config::remove_network((uint8_t)idx))
                response = "OK: removed slot " + String(idx) + "\n";
            else
                response = "ERR: invalid index or NVS save failed\n";
        } else {
            response = "ERR: WIFI [STATUS|LIST|ADD ssid pass|REMOVE N|HINTS|HINTS CLEAR]\n"
                       "     ADD: use \"quotes\" if SSID or password contains spaces\n";
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
                   "  OXI KNOWN           List known sensors and autoconnect\n"
                   "  OXI AUTOCONNECT <addr> ON|OFF\n"
                   "  OXI CONNECT [addr]  Connect to oximeter\n"
                   "  OXI DISCONNECT      Disconnect oximeter\n"
                   "  OXI ENABLE|DISABLE  Enable/disable oximetry\n"
                   "  CONFIG [key [val]]  Get/set config\n"
                   "  CONFIG SAVE|RESET   Save/reset config\n"
                   "  CONFIG DUMP         Show all config\n"
                   "  EXPORT STATUS|SMB|SLEEPHQ  Export status/manual sync\n"
#if AB_STORAGE_HAS_SDCARD
                   "  STORAGE RENAME path name  Rename a file or folder\n"
                   "  STORAGE RM path     Delete a file or folder recursively\n"
#endif
                   "  STORAGE STATUS      SD state, space and last file operation\n"
                   "  STORAGE USB ON|OFF  Share SD over USB / return after disconnect\n"
                   "  FLASH [block] [BLX] [FORCE]  Flash uploaded firmware\n"
                   "  FLASH STATUS|CANCEL Monitor/cancel flash\n"
                   "  LOG                 Show all category log levels\n"
                   "  LOG [cat] level     Set log level (use LOG to list categories, or ALL)\n"
                   "  WIFI                WiFi status/management\n"
                   "  WIFI LIST           List configured networks\n"
                   "  WIFI ADD ssid pass  Add network (use \"quotes\" if either has spaces)\n"
                   "  WIFI REMOVE N       Remove network at index\n"
                   "  WIFI HINTS          Show cached BSSID/channel hints\n"
                   "  WIFI HINTS CLEAR    Drop all cached hints\n"
                   "  TRANSPARENT         Enter raw UART mode\n"
                   "  VERSION             Firmware version info\n"
                   "  RESETREASON         Last reset reason\n"
                   "  CRASH STATUS|SUMMARY|CLEAR  Retained crash dump\n"
                   "  REBOOT              Restart ESP32\n"
                   "  FACTORYRESET CONFIRM Erase all NVS and SD data, restart\n"
                   "  HELP                This help\n"
                   "Anything without $ prefix is sent to AirSense.\n";
        return;
    }

    response = "ERR: unknown command '" + String(line) + "' (try $HELP)\n";
}
