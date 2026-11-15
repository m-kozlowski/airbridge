#pragma once
#include <Arduino.h>
#include "board.h"
#include "wireguard_config.h"

#define WIFI_MAX_NETWORKS 4

// wifi_mode values. The numeric IDs are pinned (0/1/2 match the legacy
// 3-state field) so existing devices with saved configs keep working.
#define WIFI_MODE_AUTO        0  // STA with auto AP+STA fallback (and quiet teardown)
#define WIFI_MODE_AP_ONLY     1  // softAP only, no STA
#define WIFI_MODE_OFF         2  // WiFi off
#define WIFI_MODE_STA_ONLY    3  // STA only, no AP fallback
#define WIFI_MODE_STA_AP      4  // STA + softAP always, no teardown

struct WiFiNetwork {
    String      ssid;
    String      pass;
    bool        enabled;
};

struct AirBridgeConfig {
#if AB_WIREGUARD_ENABLED
    aircannect::WireguardConfig wireguard;
#endif
    String      hostname;
    WiFiNetwork wifi_nets[WIFI_MAX_NETWORKS];
    uint8_t     wifi_net_count;     // populated slots
    uint8_t     wifi_mode;          // see WIFI_MODE_* in app_config.h
    bool        wifi_roam;          // hysteresis-based roaming
    String      wifi_country;       // ISO 3166 country code, "01" = worldwide
    uint16_t    tcp_port;

    bool        oxi_enabled;
    bool        oxi_auto_start;
    bool        oxi_feed_therapy_only;
    uint16_t    oxi_interval_ms;
    bool        oxi_lframe_continuous; // send L-frames even when no valid reading

    uint16_t    uart_cmd_timeout_ms;

    bool        allow_transparent_during_therapy;

    uint16_t    debug_port;
    bool        syslog_enabled;
    String      syslog_host;
    uint16_t    syslog_port;

    uint16_t    http_port;
    String      http_user;
    String      http_pass;
    String      auth_whitelist;
    String      ui_language;
    String      ui_theme;

    String      update_url;          // release manifest, empty disables checks

    String      ntp_server;         // empty = DHCP or pool.ntp.org
    String      tz;                 // POSIX TZ string, e.g. CET-1CEST,M3.5.0,M10.5.0/3
    bool        resmed_time;        // automatically push NTP time to ResMed

    uint16_t    udp_oxi_port;       // UDP oximetry port, 0=disabled

    bool        smb_enabled;
    bool        smb_auto_after_therapy;
    String      smb_endpoint;       // //host/share/optional/path
    String      smb_user;
    String      smb_password;

    bool        sleephq_enabled;
    bool        sleephq_auto_after_therapy;
    String      sleephq_client_id;
    String      sleephq_client_secret;
    String      sleephq_team_id;
    String      sleephq_device_id;
};

namespace Config {
    enum class Section : uint8_t {
        All, Network, Access, Time, Oximetry, Uart, Smb, SleepHq, Updates, Logging, Vpn
    };

    void init();
    void load();
    bool save(const char **error = nullptr);
    uint32_t revision(Section section = Section::All);
    void reset_defaults();

    bool onboarding_complete();
    bool complete_onboarding(const char *user, const char *password);

    AirBridgeConfig& get();
    const char *ui_language();
    const char *ui_theme();

    bool is_sensitive(const char *key);
    bool get_value(const char *key, String &out);
    bool set_value(const char *key, const char *value);

    class Guard {
    public:
#if AB_WIREGUARD_ENABLED
        Guard();
        ~Guard();
#else
        Guard() = default;
#endif
        Guard(const Guard &) = delete;
        Guard &operator=(const Guard &) = delete;
    };

    // Keep a multi-field VPN edit private until the complete profile is saved.
    class Update {
    public:
        Update();
        ~Update();
        bool set_value(const char *key, const char *value);
        bool save(const char **error = nullptr);
    private:
        Guard guard_;
        const char *error_ = nullptr;
        bool saved_ = false;
        bool has_values_ = false;
#if AB_WIREGUARD_ENABLED
        aircannect::WireguardConfig before_;
#endif
    };

#if AB_WIREGUARD_ENABLED
    // Nonblocking main-loop copy; only saved/loaded complete profiles are published.
    bool copy_wireguard(aircannect::WireguardConfig &out, uint32_t &revision);
#endif

    String dump();

    typedef void (*kv_visitor_fn)(const char *key, const String &val,
                                  bool sensitive, void *ctx);
    bool parse_section(const char *name, Section &out);
    void foreach_kv(kv_visitor_fn fn, void *ctx, Section section = Section::All);

    bool add_network(const char *ssid, const char *pass);
    bool remove_network(uint8_t idx);
    bool save_wifi_nets();
}
