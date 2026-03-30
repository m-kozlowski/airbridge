#pragma once

namespace WiFiSetup {
    bool init();
    void check();
    bool is_connected();
    bool time_synced();
    void set_fallback_time(int year, int month, int day, int hour, int min, int sec);
}
