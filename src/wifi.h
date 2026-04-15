#pragma once
#include <stdint.h>

namespace WiFiSetup {
    bool init();
    void check();
    bool is_connected();
    bool time_synced();
    void set_fallback_time(int year, int month, int day, int hour, int min, int sec, bool force = false);
    void force_ntp_sync();

    void suspend_roaming();
    void resume_roaming();

    const char *state_name();
    int8_t current_rssi();
    const char *connected_ssid();
    uint8_t connected_net_idx();
    int8_t net_rssi(uint8_t idx);
}
