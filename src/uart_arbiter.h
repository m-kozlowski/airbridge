#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "qframe.h"

typedef enum {
    CMD_SRC_OXI,
    CMD_SRC_TCP,
    CMD_SRC_INTERNAL,
    CMD_SRC_OTA,
} cmd_source_t;

typedef enum {
    CMD_PRIO_LOW      = 0,   // oximetry feed, status polls
    CMD_PRIO_NORMAL   = 1,   // TCP client commands
    CMD_PRIO_HIGH     = 2,   // safety queries, session detect
    CMD_PRIO_CRITICAL = 3,   // OTA (exclusive)
} cmd_priority_t;

typedef enum {
    SYS_IDLE,
    SYS_THERAPY,
    SYS_BOOTLOADER,
    SYS_OTA_ESP,
    SYS_OTA_AIRSENSE,
    SYS_TRANSPARENT,
    SYS_ERROR,
} system_state_t;

inline const char *system_state_name(system_state_t s) {
    static const char *names[] = {
        "IDLE","THERAPY","BOOTLOADER","OTA_ESP","OTA_AIRSENSE","TRANSPARENT","ERROR"
    };
    return (s < sizeof(names)/sizeof(names[0])) ? names[s] : "?";
}

typedef struct {
    cmd_source_t    source;
    cmd_priority_t  priority;
    uint8_t         frame[QFRAME_MAX_RAW];
    uint16_t        frame_len;
    uint32_t        ticket_id;
    uint16_t        timeout_ms;
    bool            no_ack;

    uint8_t         resp_payload[QFRAME_MAX_PAYLOAD];
    uint16_t        resp_len;
    uint8_t         resp_type;
    bool            success;
    bool            timed_out;

    volatile bool   cancelled;
    SemaphoreHandle_t done;
} uart_ticket_t;

namespace Arbiter {
    void init(HardwareSerial &serial, int rx_pin, int tx_pin, uint32_t baud);

    bool send_cmd(const char *cmd, cmd_source_t src, cmd_priority_t prio,
                  char *resp_buf, uint16_t *resp_len,
                  uint16_t timeout_ms = 0);  // 0 = use cfg.uart_cmd_timeout_ms

    bool send_frame(const uint8_t *frame, uint16_t frame_len,
                    cmd_source_t src, cmd_priority_t prio);

    bool submit(uart_ticket_t *ticket);

    system_state_t get_state();
    void set_state(system_state_t state);

    void enter_transparent(Stream *bridge);
    void exit_transparent();

    void write_raw(const uint8_t *data, size_t len);

    bool wait_frame(qframe_t *out, uint16_t timeout_ms);

    void set_baud(uint32_t baud);
    uint32_t get_baud();

    // BDD key (0=57600, 1=115200, 2=460800)
    uint32_t bdd_key_to_baud(uint16_t key);

    uint32_t get_tx_count();
    uint32_t get_rx_count();
    uint32_t get_timeout_count();
    uint32_t get_error_count();

    void lcd_message(const char *msg, uint32_t timeout_ms = 0);  // 0 = persistent
    void lcd_clear();

    uint32_t transparent_activity();
}
