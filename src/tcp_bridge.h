#pragma once
#include <Arduino.h>

namespace TcpBridge {
    void init();
    void task(void *param);

    bool has_client();
    void send_to_client(const char *msg);
    void send_to_client(const uint8_t *data, size_t len);

    void respond(const String &msg);

    void init_debug_server(uint16_t port);
    void poll_debug_clients();
}
