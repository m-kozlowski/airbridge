#include "oxi_arbiter.h"
#include "uart_arbiter.h"
#include "qframe.h"
#include "app_config.h"
#include "debug_log.h"
#include <math.h>

float parse_sfloat(uint16_t raw) {
    if (raw == 0x07FF || raw == 0x0800 || raw == 0x07FE) return -1;
    int16_t mantissa = raw & 0x0FFF;
    if (mantissa & 0x0800) mantissa |= 0xF000;
    int8_t exponent = (int8_t)((raw >> 12) & 0x0F);
    if (exponent & 0x08) exponent |= 0xF0;
    return (float)mantissa * powf(10.0f, (float)exponent);
}

static oxi_reading_t reading = { -1, -1, false, 0 };
static volatile bool feeding = false;
static oxi_source_t src_active = OXI_SRC_NONE;
static uint32_t src_last_time = 0;

#define SOURCE_TIMEOUT_MS 10000

static uint8_t oxh_seq = 0;
static uint8_t oxh_toggle = 0;
static bool last_valid = false;
static uint32_t last_inject = 0;

static const char *src_name(oxi_source_t s) {
    switch (s) {
        case OXI_SRC_BLE: return "BLE";
        case OXI_SRC_UDP: return "UDP";
        default: return "NONE";
    }
}

static void inject_lframe() {
    system_state_t st = Arbiter::get_state();
    if (st == SYS_TRANSPARENT || st == SYS_OTA_AIRSENSE || st == SYS_OTA_ESP) return;

    auto &cfg = Config::get();
    if (cfg.oxi_feed_therapy_only && st != SYS_THERAPY) return;
    if (!cfg.oxi_lframe_continuous && !reading.valid) return;

    if (reading.valid != last_valid) {
        if (reading.valid)
            Log::logf(CAT_OXI, LOG_INFO, "[OXI] Finger detected: SpO2=%d%% HR=%d bpm (%s)\n",
                      reading.spo2, reading.pulse_bpm, src_name(src_active));
        else
            Log::logf(CAT_OXI, LOG_INFO, "[OXI] Finger lost\n");
        last_valid = reading.valid;
    }

    uint8_t toggle = oxh_toggle ? 0x02 : 0x00;
    oxh_toggle ^= 1;

    uint8_t oxs, sas;
    uint16_t hrr;
    uint8_t sar;

    if (reading.valid) {
        oxs = 0x81 | toggle;
        sas = 0x80 | toggle;
        hrr = (uint16_t)reading.pulse_bpm;
        sar = (uint8_t)reading.spo2;
    } else {
        oxs = 0x99 | toggle;
        sas = 0x98 | toggle;
        hrr = 0x1FF;
        sar = 0x7F;
    }

    char payload[17];
    snprintf(payload, sizeof(payload), "OXH%02X%02X%03X%02X%02X10",
             oxh_seq, oxs, hrr, sas, sar);
    oxh_seq = (oxh_seq + 1) & 0xFF;

    Log::logf(CAT_OXI, LOG_DEBUG, "[OXI] L-frame seq=%02X %s SpO2=%d HR=%d t=%lu\n",
              (oxh_seq - 1) & 0xFF, reading.valid ? "valid" : "no-finger",
              reading.spo2, reading.pulse_bpm, millis());

    uint8_t frame_buf[32];
    int frame_len = qframe_build('L', (const uint8_t *)payload, 16,
                                 frame_buf, sizeof(frame_buf));
    if (frame_len < 0) return;

    Arbiter::send_frame(frame_buf, (uint16_t)frame_len, CMD_SRC_OXI, CMD_PRIO_LOW);
}

void OxiArbiter::init() {
    reading = { -1, -1, false, 0 };
    feeding = false;
    src_active = OXI_SRC_NONE;
    oxh_seq = 0;
    oxh_toggle = 0;
    last_valid = false;
    last_inject = 0;
}

void OxiArbiter::feed(oxi_source_t src, int8_t spo2, int16_t pulse_bpm, bool valid) {
    if (src_active != OXI_SRC_NONE && src_active != src) return;

    if (src_active == OXI_SRC_NONE && src != OXI_SRC_NONE) {
        Log::logf(CAT_OXI, LOG_INFO, "[OXI] Source active: %s\n", src_name(src));
        Arbiter::lcd_message("Oximeter Connected", 15000);
    }

    src_active = src;
    src_last_time = millis();
    reading.spo2 = spo2;
    reading.pulse_bpm = pulse_bpm;
    reading.valid = valid;
    reading.timestamp_ms = millis();
}

void OxiArbiter::start_feed() {
    feeding = true;
    Log::logf(CAT_OXI, LOG_INFO, "[OXI] Feeding started\n");
}

void OxiArbiter::stop_feed() {
    feeding = false;
    Log::logf(CAT_OXI, LOG_INFO, "[OXI] Feeding stopped\n");
}

bool OxiArbiter::is_feeding() { return feeding; }

const oxi_reading_t& OxiArbiter::get_reading() { return reading; }

oxi_source_t OxiArbiter::active_source() { return src_active; }

void OxiArbiter::poll() {
    // release source after timeout
    if (src_active != OXI_SRC_NONE && millis() - src_last_time > SOURCE_TIMEOUT_MS) {
        Log::logf(CAT_OXI, LOG_INFO, "[OXI] Source %s timed out\n", src_name(src_active));
        src_active = OXI_SRC_NONE;
        reading.valid = false;
    }

    // inject at configured interval
    auto &cfg = Config::get();
    if (feeding && millis() - last_inject >= cfg.oxi_interval_ms) {
        last_inject = millis();
        inject_lframe();
    }
}
