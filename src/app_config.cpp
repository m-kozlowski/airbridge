#include "app_config.h"
#include "uart_arbiter.h"
#include <Preferences.h>

#define CFG_DEFAULT_TCP_PORT    23
#define CFG_DEFAULT_BAUD        57600

static Preferences prefs;
static AirBridgeConfig cfg;

static void apply_defaults() {
    cfg.hostname = DEFAULT_HOSTNAME;
    cfg.wifi_ssid = "";
    cfg.wifi_pass = "";
    cfg.wifi_mode = 1;                  // AP mode by default
    cfg.tcp_port = CFG_DEFAULT_TCP_PORT;

    cfg.oxi_enabled = true;
    cfg.oxi_auto_start = true;
    cfg.oxi_feed_therapy_only = false;
    cfg.oxi_device_type = 0;
    cfg.oxi_device_addr = "";
    cfg.oxi_interval_ms = 1000;

    cfg.uart_baud = CFG_DEFAULT_BAUD;
    cfg.uart_cmd_timeout_ms = 2000;
    cfg.uart_max_retries = 3;

    cfg.allow_transparent_during_therapy = false;

    cfg.debug_port = 8023;

    cfg.http_port = 80;
    cfg.http_user = "admin";
    cfg.http_pass = "airbridge";

    cfg.ota_password = "airbridge";

    cfg.mitm_mode = 0;
}

void Config::init() {
    apply_defaults();
    prefs.begin("airbridge", false);
}

void Config::load() {
    cfg.hostname        = prefs.getString("hostname", cfg.hostname);
    cfg.wifi_ssid       = prefs.getString("wifi_ssid", cfg.wifi_ssid);
    cfg.wifi_pass       = prefs.getString("wifi_pass", cfg.wifi_pass);
    cfg.wifi_mode       = prefs.getUChar("wifi_mode", cfg.wifi_mode);
    cfg.tcp_port        = prefs.getUShort("tcp_port", cfg.tcp_port);

    cfg.oxi_enabled     = prefs.getBool("oxi_enabled", cfg.oxi_enabled);
    cfg.oxi_auto_start  = prefs.getBool("oxi_autostart", cfg.oxi_auto_start);
    cfg.oxi_feed_therapy_only = prefs.getBool("oxi_thronly", cfg.oxi_feed_therapy_only);
    cfg.oxi_device_type = prefs.getUChar("oxi_devtype", cfg.oxi_device_type);
    cfg.oxi_device_addr = prefs.getString("oxi_devaddr", cfg.oxi_device_addr);
    cfg.oxi_interval_ms = prefs.getUShort("oxi_interval", cfg.oxi_interval_ms);

    cfg.uart_baud       = prefs.getULong("uart_baud", cfg.uart_baud);
    cfg.uart_cmd_timeout_ms = prefs.getUShort("uart_timeout", cfg.uart_cmd_timeout_ms);
    cfg.uart_max_retries = prefs.getUChar("uart_retries", cfg.uart_max_retries);

    cfg.allow_transparent_during_therapy = prefs.getBool("allow_transp", cfg.allow_transparent_during_therapy);

    cfg.debug_port      = prefs.getUShort("debug_port", cfg.debug_port);

    cfg.http_port       = prefs.getUShort("http_port", cfg.http_port);
    cfg.http_user       = prefs.getString("http_user", cfg.http_user);
    cfg.http_pass       = prefs.getString("http_pass", cfg.http_pass);

    cfg.ota_password    = prefs.getString("ota_pass", cfg.ota_password);
    cfg.mitm_mode       = prefs.getUChar("mitm_mode", cfg.mitm_mode);
}

void Config::save() {
    prefs.putString("hostname", cfg.hostname);
    prefs.putString("wifi_ssid", cfg.wifi_ssid);
    prefs.putString("wifi_pass", cfg.wifi_pass);
    prefs.putUChar("wifi_mode", cfg.wifi_mode);
    prefs.putUShort("tcp_port", cfg.tcp_port);

    prefs.putBool("oxi_enabled", cfg.oxi_enabled);
    prefs.putBool("oxi_autostart", cfg.oxi_auto_start);
    prefs.putBool("oxi_thronly", cfg.oxi_feed_therapy_only);
    prefs.putUChar("oxi_devtype", cfg.oxi_device_type);
    prefs.putString("oxi_devaddr", cfg.oxi_device_addr);
    prefs.putUShort("oxi_interval", cfg.oxi_interval_ms);

    prefs.putULong("uart_baud", cfg.uart_baud);
    prefs.putUShort("uart_timeout", cfg.uart_cmd_timeout_ms);
    prefs.putUChar("uart_retries", cfg.uart_max_retries);

    prefs.putBool("allow_transp", cfg.allow_transparent_during_therapy);

    prefs.putUShort("debug_port", cfg.debug_port);

    prefs.putUShort("http_port", cfg.http_port);
    prefs.putString("http_user", cfg.http_user);
    prefs.putString("http_pass", cfg.http_pass);

    prefs.putString("ota_pass", cfg.ota_password);
    prefs.putUChar("mitm_mode", cfg.mitm_mode);
}

void Config::reset_defaults() {
    prefs.clear();
    apply_defaults();
    save();
}

AirBridgeConfig& Config::get() {
    return cfg;
}

static String parse_response_str(const char *resp) {
    const char *eq = strstr(resp, "= ");
    if (!eq) return "";
    String val = eq + 2;
    int end = val.indexOf(" #");
    if (end > 0) val = val.substring(0, end);
    val.trim();
    return val;
}

void Config::refresh_device_info() {
    char resp[64] = {};
    uint16_t len;

    if (cfg.device_pna.isEmpty()) {
        len = sizeof(resp);
        memset(resp, 0, sizeof(resp));
        if (Arbiter::send_cmd("G S #PNA", CMD_SRC_INTERNAL, CMD_PRIO_NORMAL,
                               resp, &len, 2000)) {
            cfg.device_pna = parse_response_str(resp);
        }
    }

    if (cfg.device_srn.isEmpty()) {
        len = sizeof(resp);
        memset(resp, 0, sizeof(resp));
        if (Arbiter::send_cmd("G S #SRN", CMD_SRC_INTERNAL, CMD_PRIO_NORMAL,
                               resp, &len, 2000)) {
            cfg.device_srn = parse_response_str(resp);
        }
    }
}

void Config::invalidate_device_info() {
    cfg.device_pna = "";
    cfg.device_srn = "";
}

struct KVEntry {
    const char *key;
    enum { STR, U8, U16, U32, BOOL } type;
    void *ptr;
};

#define KV_STR(k, f)  { k, KVEntry::STR,  &cfg.f }
#define KV_U8(k, f)   { k, KVEntry::U8,   &cfg.f }
#define KV_U16(k, f)  { k, KVEntry::U16,  &cfg.f }
#define KV_U32(k, f)  { k, KVEntry::U32,  &cfg.f }
#define KV_BOOL(k, f) { k, KVEntry::BOOL, &cfg.f }

static const KVEntry kv_table[] = {
    KV_STR("hostname", hostname),
    KV_STR("wifi_ssid", wifi_ssid),
    KV_STR("wifi_pass", wifi_pass),
    KV_U8("wifi_mode", wifi_mode),
    KV_U16("tcp_port", tcp_port),
    KV_BOOL("oxi_enabled", oxi_enabled),
    KV_BOOL("oxi_auto_start", oxi_auto_start),
    KV_BOOL("oxi_feed_therapy_only", oxi_feed_therapy_only),
    KV_U8("oxi_device_type", oxi_device_type),
    KV_STR("oxi_device_addr", oxi_device_addr),
    KV_U16("oxi_interval_ms", oxi_interval_ms),
    KV_U32("uart_baud", uart_baud),
    KV_U16("uart_cmd_timeout_ms", uart_cmd_timeout_ms),
    KV_U8("uart_max_retries", uart_max_retries),
    KV_BOOL("allow_transparent_during_therapy", allow_transparent_during_therapy),
    KV_U16("debug_port", debug_port),
    KV_U16("http_port", http_port),
    KV_STR("http_user", http_user),
    KV_STR("http_pass", http_pass),
    KV_STR("ota_password", ota_password),
    KV_U8("mitm_mode", mitm_mode),
    { nullptr, KVEntry::U8, nullptr }
};

bool Config::get_value(const char *key, String &out) {
    for (const KVEntry *e = kv_table; e->key; e++) {
        if (strcasecmp(key, e->key) == 0) {
            switch (e->type) {
            case KVEntry::STR:  out = *(String*)e->ptr; break;
            case KVEntry::U8:   out = String(*(uint8_t*)e->ptr); break;
            case KVEntry::U16:  out = String(*(uint16_t*)e->ptr); break;
            case KVEntry::U32:  out = String(*(uint32_t*)e->ptr); break;
            case KVEntry::BOOL: out = (*(bool*)e->ptr) ? "1" : "0"; break;
            }
            return true;
        }
    }
    return false;
}

bool Config::set_value(const char *key, const char *value) {
    for (const KVEntry *e = kv_table; e->key; e++) {
        if (strcasecmp(key, e->key) == 0) {
            switch (e->type) {
            case KVEntry::STR:  *(String*)e->ptr = value; break;
            case KVEntry::U8:   *(uint8_t*)e->ptr = atoi(value); break;
            case KVEntry::U16:  *(uint16_t*)e->ptr = atoi(value); break;
            case KVEntry::U32:  *(uint32_t*)e->ptr = atol(value); break;
            case KVEntry::BOOL: *(bool*)e->ptr = (atoi(value) != 0); break;
            }
            return true;
        }
    }
    return false;
}

String Config::dump() {
    String out;
    for (const KVEntry *e = kv_table; e->key; e++) {
        String val;
        get_value(e->key, val);
        if ((strcmp(e->key, "wifi_pass") == 0 || strcmp(e->key, "http_pass") == 0) && val.length() > 0) {
            val = "****";
        }
        out += String(e->key) + "=" + val + "\n";
    }
    return out;
}
