#pragma once
#include <Arduino.h>
#include "board.h"

#define WIFI_MAX_NETWORKS 4

struct WiFiNetwork {
    String      ssid;
    String      pass;
    uint8_t     bssid[6];
    uint8_t     channel;
    bool        enabled;
};

struct AirBridgeConfig {
    String      hostname;
    WiFiNetwork wifi_nets[WIFI_MAX_NETWORKS];
    uint8_t     wifi_net_count;     // populated slots
    uint8_t     wifi_mode;          // 0=STA, 1=AP, 2=off
    bool        wifi_roam;          // hysteresis-based roaming
    uint16_t    tcp_port;

    bool        oxi_enabled;
    bool        oxi_auto_start;
    bool        oxi_feed_therapy_only;
    uint8_t     oxi_device_type;    // 0=auto, 1=Nonin, 2=O2Ring, 3=PLX generic
    String      oxi_device_addr;
    uint16_t    oxi_interval_ms;
    bool        oxi_lframe_continuous; // send L-frames even when no valid reading
    bool        oxi_require_known;     // only auto-connect to bonded/known devices

    uint32_t    uart_baud;
    uint16_t    uart_cmd_timeout_ms;
    uint8_t     uart_max_retries;

    bool        allow_transparent_during_therapy;

    uint16_t    debug_port;

    uint16_t    http_port;
    String      http_user;
    String      http_pass;

    String      ota_password;

    String      ntp_server;         // empty = DHCP or pool.ntp.org
    String      tz;                 // POSIX TZ string, e.g. CET-1CEST,M3.5.0,M10.5.0/3

    uint16_t    udp_oxi_port;       // UDP oximetry port, 0=disabled

    uint8_t     mitm_mode;          // 0=off, 1=forward, 2=log, 3=filter

    // Runtime cache
    String      device_pna;         // #PNA
    String      device_srn;         // #SRN
};

namespace Config {
    void init();
    void load();
    void save();
    void reset_defaults();

    AirBridgeConfig& get();

    bool get_value(const char *key, String &out);
    bool set_value(const char *key, const char *value);

    void refresh_device_info();
    void invalidate_device_info();

    String dump();

    typedef void (*kv_visitor_fn)(const char *key, const String &val, void *ctx);
    void foreach_kv(kv_visitor_fn fn, void *ctx);

    bool add_network(const char *ssid, const char *pass);
    bool remove_network(uint8_t idx);
    void update_network_hint(uint8_t idx, const uint8_t *bssid, uint8_t channel);
    void save_wifi_nets();
}
