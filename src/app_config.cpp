#include "app_config.h"
#include "uart_arbiter.h"
#include "qframe.h"
#include <Preferences.h>

#define CFG_DEFAULT_TCP_PORT    23
#define CFG_DEFAULT_BAUD        57600

static Preferences prefs;
static AirBridgeConfig cfg;

static void apply_defaults() {
    cfg.hostname = DEFAULT_HOSTNAME;
    cfg.wifi_net_count = 0;
    for (int i = 0; i < WIFI_MAX_NETWORKS; i++) {
        cfg.wifi_nets[i].ssid = "";
        cfg.wifi_nets[i].pass = "";
        memset(cfg.wifi_nets[i].bssid, 0, 6);
        cfg.wifi_nets[i].channel = 0;
        cfg.wifi_nets[i].enabled = false;
    }
    cfg.wifi_mode = 1;                  // AP mode by default
    cfg.wifi_roam = true;
    cfg.tcp_port = CFG_DEFAULT_TCP_PORT;

    cfg.oxi_enabled = true;
    cfg.oxi_auto_start = true;
    cfg.oxi_feed_therapy_only = false;
    cfg.oxi_device_type = 0;
    cfg.oxi_device_addr = "";
    cfg.oxi_interval_ms = 500;
    cfg.oxi_lframe_continuous = true;
    cfg.oxi_require_known = false;

    cfg.uart_baud = CFG_DEFAULT_BAUD;
    cfg.uart_cmd_timeout_ms = 500;
    cfg.uart_max_retries = 3;

    cfg.allow_transparent_during_therapy = false;

    cfg.debug_port = 8023;

    cfg.http_port = 80;
    cfg.http_user = "admin";
    cfg.http_pass = "airbridge";

    cfg.ota_password = "airbridge";

    cfg.ntp_server = "";
    cfg.tz = "UTC0";
    cfg.udp_oxi_port = 8025;

    cfg.mitm_mode = 0;
}

void Config::init() {
    apply_defaults();
    prefs.begin("airbridge", false);
}

static void load_wifi_nets() {
    Preferences wp;
    wp.begin("wnet", true);
    cfg.wifi_net_count = wp.getUChar("count", 0);
    if (cfg.wifi_net_count > WIFI_MAX_NETWORKS) cfg.wifi_net_count = WIFI_MAX_NETWORKS;
    for (int i = 0; i < cfg.wifi_net_count; i++) {
        char key[14];
        snprintf(key, sizeof(key), "ssid_%d", i);
        cfg.wifi_nets[i].ssid = wp.getString(key, "");
        snprintf(key, sizeof(key), "pass_%d", i);
        cfg.wifi_nets[i].pass = wp.getString(key, "");
        snprintf(key, sizeof(key), "bssid_%d", i);
        wp.getBytes(key, cfg.wifi_nets[i].bssid, 6);
        snprintf(key, sizeof(key), "chan_%d", i);
        cfg.wifi_nets[i].channel = wp.getUChar(key, 0);
        snprintf(key, sizeof(key), "ena_%d", i);
        cfg.wifi_nets[i].enabled = wp.getBool(key, true);
    }
    wp.end();

    // migrate from old single wifi_ssid/wifi_pass
    if (cfg.wifi_net_count == 0) {
        String old_ssid = prefs.getString("wifi_ssid", "");
        if (old_ssid.length() > 0) {
            cfg.wifi_nets[0].ssid = old_ssid;
            cfg.wifi_nets[0].pass = prefs.getString("wifi_pass", "");
            memset(cfg.wifi_nets[0].bssid, 0, 6);
            cfg.wifi_nets[0].channel = 0;
            cfg.wifi_nets[0].enabled = true;
            cfg.wifi_net_count = 1;
            Config::save_wifi_nets();
            prefs.remove("wifi_ssid");
            prefs.remove("wifi_pass");
        }
    }
}

void Config::load() {
    cfg.hostname        = prefs.getString("hostname", cfg.hostname);
    cfg.wifi_mode       = prefs.getUChar("wifi_mode", cfg.wifi_mode);
    cfg.wifi_roam       = prefs.getBool("wifi_roam", cfg.wifi_roam);
    cfg.tcp_port        = prefs.getUShort("tcp_port", cfg.tcp_port);
    load_wifi_nets();

    cfg.oxi_enabled     = prefs.getBool("oxi_enabled", cfg.oxi_enabled);
    cfg.oxi_auto_start  = prefs.getBool("oxi_autostart", cfg.oxi_auto_start);
    cfg.oxi_feed_therapy_only = prefs.getBool("oxi_thronly", cfg.oxi_feed_therapy_only);
    cfg.oxi_device_type = prefs.getUChar("oxi_devtype", cfg.oxi_device_type);
    cfg.oxi_device_addr = prefs.getString("oxi_devaddr", cfg.oxi_device_addr);
    cfg.oxi_interval_ms = prefs.getUShort("oxi_interval", cfg.oxi_interval_ms);
    cfg.oxi_lframe_continuous = prefs.getBool("oxi_lframe_cont", cfg.oxi_lframe_continuous);
    cfg.oxi_require_known = prefs.getBool("oxi_req_known", cfg.oxi_require_known);

    cfg.uart_baud       = prefs.getULong("uart_baud", cfg.uart_baud);
    cfg.uart_cmd_timeout_ms = prefs.getUShort("uart_timeout", cfg.uart_cmd_timeout_ms);
    cfg.uart_max_retries = prefs.getUChar("uart_retries", cfg.uart_max_retries);

    cfg.allow_transparent_during_therapy = prefs.getBool("allow_transp", cfg.allow_transparent_during_therapy);

    cfg.debug_port      = prefs.getUShort("debug_port", cfg.debug_port);

    cfg.http_port       = prefs.getUShort("http_port", cfg.http_port);
    cfg.http_user       = prefs.getString("http_user", cfg.http_user);
    cfg.http_pass       = prefs.getString("http_pass", cfg.http_pass);

    cfg.ota_password    = prefs.getString("ota_pass", cfg.ota_password);
    cfg.ntp_server      = prefs.getString("ntp_server", cfg.ntp_server);
    cfg.tz              = prefs.getString("tz", cfg.tz);
    cfg.udp_oxi_port    = prefs.getUShort("udp_oxi_port", cfg.udp_oxi_port);
    cfg.mitm_mode       = prefs.getUChar("mitm_mode", cfg.mitm_mode);
}

void Config::save() {
    prefs.putString("hostname", cfg.hostname);
    prefs.putUChar("wifi_mode", cfg.wifi_mode);
    prefs.putBool("wifi_roam", cfg.wifi_roam);
    prefs.putUShort("tcp_port", cfg.tcp_port);
    // wifi_nets saved separately via save_wifi_nets()

    prefs.putBool("oxi_enabled", cfg.oxi_enabled);
    prefs.putBool("oxi_autostart", cfg.oxi_auto_start);
    prefs.putBool("oxi_thronly", cfg.oxi_feed_therapy_only);
    prefs.putUChar("oxi_devtype", cfg.oxi_device_type);
    prefs.putString("oxi_devaddr", cfg.oxi_device_addr);
    prefs.putUShort("oxi_interval", cfg.oxi_interval_ms);
    prefs.putBool("oxi_lframe_cont", cfg.oxi_lframe_continuous);
    prefs.putBool("oxi_req_known", cfg.oxi_require_known);

    prefs.putULong("uart_baud", cfg.uart_baud);
    prefs.putUShort("uart_timeout", cfg.uart_cmd_timeout_ms);
    prefs.putUChar("uart_retries", cfg.uart_max_retries);

    prefs.putBool("allow_transp", cfg.allow_transparent_during_therapy);

    prefs.putUShort("debug_port", cfg.debug_port);

    prefs.putUShort("http_port", cfg.http_port);
    prefs.putString("http_user", cfg.http_user);
    prefs.putString("http_pass", cfg.http_pass);

    prefs.putString("ota_pass", cfg.ota_password);
    prefs.putString("ntp_server", cfg.ntp_server);
    prefs.putString("tz", cfg.tz);
    prefs.putUShort("udp_oxi_port", cfg.udp_oxi_port);
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


void Config::refresh_device_info() {
    char resp[64] = {};
    uint16_t len;

    if (cfg.device_pna.isEmpty()) {
        len = sizeof(resp);
        memset(resp, 0, sizeof(resp));
        if (Arbiter::send_cmd("G S #PNA", CMD_SRC_INTERNAL, CMD_PRIO_NORMAL,
                               resp, &len)) {
            const char *v = qframe_response_value(resp);
            if (v) cfg.device_pna = v;
        }
    }

    if (cfg.device_srn.isEmpty()) {
        len = sizeof(resp);
        memset(resp, 0, sizeof(resp));
        if (Arbiter::send_cmd("G S #SRN", CMD_SRC_INTERNAL, CMD_PRIO_NORMAL,
                               resp, &len)) {
            const char *v = qframe_response_value(resp);
            if (v) cfg.device_srn = v;
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
    KV_U8("wifi_mode", wifi_mode),
    KV_BOOL("wifi_roam", wifi_roam),
    KV_U16("tcp_port", tcp_port),
    KV_BOOL("oxi_enabled", oxi_enabled),
    KV_BOOL("oxi_auto_start", oxi_auto_start),
    KV_BOOL("oxi_feed_therapy_only", oxi_feed_therapy_only),
    KV_U8("oxi_device_type", oxi_device_type),
    KV_STR("oxi_device_addr", oxi_device_addr),
    KV_U16("oxi_interval_ms", oxi_interval_ms),
    KV_BOOL("oxi_lframe_continuous", oxi_lframe_continuous),
    KV_BOOL("oxi_require_known", oxi_require_known),
    KV_U32("uart_baud", uart_baud),
    KV_U16("uart_cmd_timeout_ms", uart_cmd_timeout_ms),
    KV_U8("uart_max_retries", uart_max_retries),
    KV_BOOL("allow_transparent_during_therapy", allow_transparent_during_therapy),
    KV_U16("debug_port", debug_port),
    KV_U16("http_port", http_port),
    KV_STR("http_user", http_user),
    KV_STR("http_pass", http_pass),
    KV_STR("ota_password", ota_password),
    KV_STR("ntp_server", ntp_server),
    KV_STR("tz", tz),
    KV_U16("udp_oxi_port", udp_oxi_port),
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

void Config::foreach_kv(kv_visitor_fn fn, void *ctx) {
    for (const KVEntry *e = kv_table; e->key; e++) {
        String val;
        get_value(e->key, val);
        fn(e->key, val, ctx);
    }
}

String Config::dump() {
    String out;
    // wifi networks
    for (int i = 0; i < cfg.wifi_net_count; i++) {
        out += "wifi_net_" + String(i) + "=" + cfg.wifi_nets[i].ssid;
        if (cfg.wifi_nets[i].channel > 0)
            out += " (ch" + String(cfg.wifi_nets[i].channel) + ")";
        if (!cfg.wifi_nets[i].enabled) out += " [disabled]";
        out += "\n";
    }
    // KV table entries
    foreach_kv([](const char *key, const String &val, void *p) {
        String v = val;
        if (strstr(key, "pass") && v.length() > 0) v = "****";
        *(String*)p += String(key) + "=" + v + "\n";
    }, &out);
    return out;
}


void Config::save_wifi_nets() {
    Preferences wp;
    wp.begin("wnet", false);
    wp.putUChar("count", cfg.wifi_net_count);
    for (int i = 0; i < WIFI_MAX_NETWORKS; i++) {
        char key[14];
        if (i < cfg.wifi_net_count) {
            snprintf(key, sizeof(key), "ssid_%d", i);
            wp.putString(key, cfg.wifi_nets[i].ssid);
            snprintf(key, sizeof(key), "pass_%d", i);
            wp.putString(key, cfg.wifi_nets[i].pass);
            snprintf(key, sizeof(key), "bssid_%d", i);
            wp.putBytes(key, cfg.wifi_nets[i].bssid, 6);
            snprintf(key, sizeof(key), "chan_%d", i);
            wp.putUChar(key, cfg.wifi_nets[i].channel);
            snprintf(key, sizeof(key), "ena_%d", i);
            wp.putBool(key, cfg.wifi_nets[i].enabled);
        } else {
            snprintf(key, sizeof(key), "ssid_%d", i); wp.remove(key);
            snprintf(key, sizeof(key), "pass_%d", i); wp.remove(key);
            snprintf(key, sizeof(key), "bssid_%d", i); wp.remove(key);
            snprintf(key, sizeof(key), "chan_%d", i); wp.remove(key);
            snprintf(key, sizeof(key), "ena_%d", i); wp.remove(key);
        }
    }
    wp.end();
}

bool Config::add_network(const char *ssid, const char *pass) {
    if (cfg.wifi_net_count >= WIFI_MAX_NETWORKS) return false;
    int idx = cfg.wifi_net_count;
    cfg.wifi_nets[idx].ssid = ssid;
    cfg.wifi_nets[idx].pass = pass ? pass : "";
    memset(cfg.wifi_nets[idx].bssid, 0, 6);
    cfg.wifi_nets[idx].channel = 0;
    cfg.wifi_nets[idx].enabled = true;
    cfg.wifi_net_count++;
    save_wifi_nets();
    return true;
}

bool Config::remove_network(uint8_t idx) {
    if (idx >= cfg.wifi_net_count) return false;
    // shift remaining entries down
    for (int i = idx; i < cfg.wifi_net_count - 1; i++)
        cfg.wifi_nets[i] = cfg.wifi_nets[i + 1];
    cfg.wifi_net_count--;
    // clear the vacated slot
    cfg.wifi_nets[cfg.wifi_net_count].ssid = "";
    cfg.wifi_nets[cfg.wifi_net_count].pass = "";
    memset(cfg.wifi_nets[cfg.wifi_net_count].bssid, 0, 6);
    cfg.wifi_nets[cfg.wifi_net_count].channel = 0;
    cfg.wifi_nets[cfg.wifi_net_count].enabled = false;
    save_wifi_nets();
    return true;
}

void Config::update_network_hint(uint8_t idx, const uint8_t *bssid, uint8_t channel) {
    if (idx >= cfg.wifi_net_count) return;
    memcpy(cfg.wifi_nets[idx].bssid, bssid, 6);
    cfg.wifi_nets[idx].channel = channel;
    save_wifi_nets();
}
