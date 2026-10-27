#pragma once
#include <stdint.h>

namespace WiFiSetup {
    bool init();
    void check();
    void request_reconfigure(bool network, bool clock);
    void profiles_changed();
    bool is_connected();
    bool time_synced();
    bool set_fallback_time(int year, int month, int day, int hour, int min, int sec, bool force = false);
    void force_ntp_sync();

    void suspend_roaming();
    void resume_roaming();

    const char *state_name();
    uint32_t revision();
    // Published IPv4 addresses in network byte order; zero means unavailable.
    void addresses(uint32_t &sta, uint32_t &ap);
    int8_t current_rssi();
    const char *connected_ssid();
    // Main-loop snapshot of the running AP; empty while it is unavailable.
    const char *access_point_ssid();
    uint8_t connected_net_idx();
    int8_t net_rssi(uint8_t idx);
}
