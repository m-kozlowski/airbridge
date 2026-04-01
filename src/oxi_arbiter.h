#pragma once
#include <Arduino.h>
#include "oxi_ble.h"  // oxi_state_t, oxi_reading_t

typedef enum {
    OXI_SRC_NONE,
    OXI_SRC_BLE,
    OXI_SRC_UDP,
} oxi_source_t;

namespace OxiArbiter {
    void init();

    void feed(oxi_source_t src, int8_t spo2, int16_t pulse_bpm, bool valid);

    void start_feed();
    void stop_feed();
    bool is_feeding();

    const oxi_reading_t& get_reading();
    oxi_source_t active_source();

    void poll();
}
