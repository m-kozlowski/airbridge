#include "oxi_udp.h"
#include "oxi_arbiter.h"
#include "app_config.h"
#include "debug_log.h"
#include <WiFiUdp.h>

#define UDP_TASK_STACK  2048
#define UDP_TASK_PRIO   3

// Protocol: [0x55] [0xAB] [flags] [spo2_lo] [spo2_hi] [hr_lo] [hr_hi]
#define UDP_MAGIC_0     0x55
#define UDP_MAGIC_1     0xAB
#define UDP_PACKET_SIZE 7


static void udp_task(void *param) {
    auto &cfg = Config::get();
    uint16_t port = cfg.udp_oxi_port;
    if (port == 0) {
        Log::logf(CAT_OXI, LOG_INFO, "[OXI] UDP disabled (port=0)\n");
        vTaskDelete(NULL);
        return;
    }

    WiFiUDP udp;
    if (!udp.begin(port)) {
        Log::logf(CAT_OXI, LOG_ERROR, "[OXI] UDP bind failed on port %u\n", port);
        vTaskDelete(NULL);
        return;
    }
    Log::logf(CAT_OXI, LOG_INFO, "[OXI] UDP listening on port %u\n", port);

    while (true) {
        int len = udp.parsePacket();
        if (len == UDP_PACKET_SIZE) {
            uint8_t buf[UDP_PACKET_SIZE];
            udp.read(buf, UDP_PACKET_SIZE);

            // validate magic
            if (buf[0] != UDP_MAGIC_0 || buf[1] != UDP_MAGIC_1) goto next;

            // validate flags
            if (buf[2] & 0xE0) goto next;

            {
                uint16_t spo2_raw = buf[3] | (buf[4] << 8);
                uint16_t hr_raw = buf[5] | (buf[6] << 8);
                float spo2 = parse_sfloat(spo2_raw);
                float hr = parse_sfloat(hr_raw);

                if (spo2 >= 0 && spo2 <= 100 && hr >= 0 && hr <= 500) {
                    OxiArbiter::feed(OXI_SRC_UDP, (int8_t)spo2, (int16_t)hr, true);
                } else if (spo2 < 0 || hr < 0) {
                    OxiArbiter::feed(OXI_SRC_UDP, -1, -1, false);
                }
            }
        }
next:
        if (len <= 0) vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void OxiUdp::init() {
    xTaskCreatePinnedToCore(udp_task, "oxi_udp", UDP_TASK_STACK,
                            nullptr, UDP_TASK_PRIO, nullptr, 0);
}
