#pragma once
#include <Arduino.h>
#include "board.h"

struct AirBridgeConfig {
    String      hostname;
    String      wifi_ssid;
    String      wifi_pass;
    uint8_t     wifi_mode;          // 0=STA, 1=AP, 2=off
    uint16_t    tcp_port;

    bool        oxi_enabled;
    bool        oxi_auto_start;
    bool        oxi_feed_therapy_only;
    uint8_t     oxi_device_type;    // 0=auto, 1=Nonin, 2=O2Ring, 3=PLX generic
    String      oxi_device_addr;
    uint16_t    oxi_interval_ms;
    bool        oxi_lframe_continuous; // send L-frames even when no valid reading

    uint32_t    uart_baud;
    uint16_t    uart_cmd_timeout_ms;
    uint8_t     uart_max_retries;

    bool        allow_transparent_during_therapy;

    uint16_t    debug_port;

    uint16_t    http_port;
    String      http_user;
    String      http_pass;

    String      ota_password;

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
}
