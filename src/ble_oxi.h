#pragma once
#include <Arduino.h>

typedef enum {
    OXI_DISABLED,
    OXI_SCANNING,
    OXI_CONNECTING,
    OXI_BONDING,
    OXI_STREAMING,
    OXI_DISCONNECTED,
} oxi_state_t;

typedef struct {
    int8_t      spo2;
    int16_t     pulse_bpm;
    bool        valid;
    uint32_t    timestamp_ms;
} oxi_reading_t;

namespace BleOxi {
    void init();

    void start_scan();
    void stop_scan();
    void connect(const char *addr = nullptr);
    void disconnect();
    void start_feed();
    void stop_feed();

    oxi_state_t get_state();
    const oxi_reading_t& get_reading();
    bool is_feeding();

    void task(void *param);

    String get_scan_results();
}
