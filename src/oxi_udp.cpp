#include "oxi_udp.h"
#include "oxi_arbiter.h"
#include "app_config.h"
#include "debug_log.h"
#include <WiFiUdp.h>

#define UDP_TASK_STACK  3072
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

    uint32_t pkt_count = 0;
    while (true) {
        int len = udp.parsePacket();
        if (len <= 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        vTaskDelay(1);  // yield after each packet

        if (len != UDP_PACKET_SIZE) {
            // drain the packet
            uint8_t drain[32];
            while (udp.available()) udp.read(drain, sizeof(drain));
            Log::logf(CAT_OXI, LOG_WARN, "[OXI] UDP bad length %d (expected %d)\n",
                      len, UDP_PACKET_SIZE);
            continue;
        }

        uint8_t buf[UDP_PACKET_SIZE];
        udp.read(buf, UDP_PACKET_SIZE);

        if (buf[0] != UDP_MAGIC_0 || buf[1] != UDP_MAGIC_1) {
            Log::logf(CAT_OXI, LOG_WARN, "[OXI] UDP bad magic %02X %02X\n",
                      buf[0], buf[1]);
            continue;
        }

        if (buf[2] & 0xE0) {
            Log::logf(CAT_OXI, LOG_WARN, "[OXI] UDP bad flags %02X\n", buf[2]);
            continue;
        }

        uint16_t spo2_raw = buf[3] | (buf[4] << 8);
        uint16_t hr_raw = buf[5] | (buf[6] << 8);
        int16_t spo2 = parse_sfloat(spo2_raw);
        int16_t hr = parse_sfloat(hr_raw);

        if (spo2 >= 0 && spo2 <= 100 && hr >= 0 && hr <= 500) {
            pkt_count++;
            if (OxiArbiter::active_source() == OXI_SRC_NONE)
                OxiArbiter::set_source_id(udp.remoteIP().toString().c_str());
            Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] UDP SpO2=%d HR=%d\n", spo2, hr);
            OxiArbiter::feed(OXI_SRC_UDP, spo2, hr, true);
        } else if (spo2 < 0 || hr < 0) {
            Log::logf(CAT_OXI, LOG_WARN, "[OXI] UDP invalid (raw=%04X,%04X)\n", spo2_raw, hr_raw);
            OxiArbiter::feed(OXI_SRC_UDP, -1, -1, false);
        } else {
            Log::logf(CAT_OXI, LOG_WARN, "[OXI] UDP out of range (spo2=%d hr=%d)\n", spo2, hr);
        }
    }
}

void OxiUdp::init() {
    xTaskCreatePinnedToCore(udp_task, "oxi_udp", UDP_TASK_STACK,
                            nullptr, UDP_TASK_PRIO, nullptr, 0);
}
