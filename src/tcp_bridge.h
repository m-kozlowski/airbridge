#pragma once
#include <Arduino.h>

namespace TcpBridge {
    void init();
    void task(void *param);

    void init_debug_server(uint16_t port);
    void poll_debug_clients();
}
