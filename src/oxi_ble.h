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

namespace OxiBle {
    void init();

    void start_scan();
    void stop_scan();
    void connect(const char *addr = nullptr);
    void disconnect();

    oxi_state_t get_state();
    bool state_changed();

    void task(void *param);

    String get_scan_results();
}
