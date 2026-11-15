#pragma once

#include "board.h"

#if AB_WIREGUARD_ENABLED

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <lwip/dns.h>
#include <lwip/ip_addr.h>
#include <stdint.h>

#include "wireguard_config.h"
#include "wireguard_routes.h"

namespace aircannect {

enum class WireguardState : uint8_t {
    Disabled,
    Home,
    WaitingNetwork,
    WaitingClock,
    Connecting,
    Up,
    Backoff,
};

struct WireguardStatus {
    WireguardState state = WireguardState::Disabled;
    WireguardMode mode = WireguardMode::Off;
    bool default_route = false; // Active AllowedIPs includes /0.
    // Seconds since the last completed handshake, or -1 when none.
    int32_t handshake_age_s = -1;
    uint32_t connects = 0;
    char error[48] = {};
};

const char *wireguard_state_name(WireguardState state);

// Owns the single esp_wireguard tunnel. Lifecycle runs from the main loop;
// the lwIP hook reads published routes. status() may be read from any task.
class WireguardService {
public:
    void configure(const WireguardConfig &config);
    // clock_valid must come from NTP: a fallback clock that runs behind a
    // previous handshake makes the peer drop every new initiation.
    void poll(bool sta_online,
              const String &ssid,
              bool clock_valid,
              uint32_t now_ms);
    WireguardStatus status() const;

private:
    bool wanted(bool sta_online, const String &ssid) const;
    bool start(uint32_t now_ms);
    bool continue_connect(uint32_t now_ms);
    void stop(const char *reason);
    void poll_up(uint32_t now_ms);
    void schedule_retry(uint32_t now_ms, const char *error);
    void set_state(WireguardState state);
    void set_error(const char *error);
    // Callers hold the lwIP core lock.
    void apply_tunnel_dns();
    void restore_wifi_dns();

    WireguardConfig config_;
    bool restart_pending_ = false;

    // Backing storage for the library config, which keeps raw pointers.
    String address_;
    String endpoint_host_;
    bool tunnel_open_ = false;
    bool peer_connected_ = false;

    // Prepared once per connection; immutable while published to lwIP.
    WireguardRoutes routes_;

    // Resolvers the Wi-Fi network handed out, saved while the tunnel's
    // resolver is installed so they can be put back when it goes down.
    bool has_tunnel_dns_ = false;
    ip_addr_t tunnel_dns_ = {};
    bool tunnel_dns_active_ = false;
    ip_addr_t wifi_dns_[DNS_MAX_SERVERS] = {};

    uint32_t connect_started_ms_ = 0;
    uint32_t retry_at_ms_ = 0;
    uint32_t retry_delay_ms_ = 0;
    uint32_t last_check_ms_ = 0;

    mutable portMUX_TYPE status_lock_ = portMUX_INITIALIZER_UNLOCKED;
    WireguardStatus status_;
};

WireguardService &wireguard_service();

}  // namespace aircannect

#endif  // AB_WIREGUARD_ENABLED
