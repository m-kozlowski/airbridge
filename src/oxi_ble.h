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

inline const char *oxi_state_name(oxi_state_t s) {
    static const char *names[] = {
        "DISABLED","SCANNING","CONNECTING","BONDING","STREAMING","DISCONNECTED"
    };
    return (s < sizeof(names)/sizeof(names[0])) ? names[s] : "?";
}

typedef struct {
    int8_t      spo2;
    int16_t     pulse_bpm;
    bool        valid;
    uint32_t    timestamp_ms;
} oxi_reading_t;

#define MAX_SCAN_RESULTS 8

struct oxi_scan_result_t {
    String addr;
    String name;
    int rssi;
    uint8_t addr_type;
};

namespace OxiBle {
    void init();

    void start_scan();
    void stop_scan();
    void connect(const char *addr = nullptr);
    void disconnect();
    void enable();
    void disable();

    oxi_state_t get_state();
    bool state_changed();

    void task(void *param);

    const oxi_scan_result_t *get_scan_results(int &count);
}
