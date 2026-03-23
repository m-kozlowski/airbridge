#pragma once
#include <Arduino.h>

namespace WebUI {
    void init(uint16_t port = 80);

    void handle();

    // event: event type name (e.g., "status", "ble", "flash", "live")
    void push_event(const char *event, const char *json);
    void push_event(const char *event, const String &json);
}
