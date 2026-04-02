#pragma once
#include <stdint.h>
#include <stddef.h>

// ResMed UART frame format:
// [0x55] [Type] [Len:3hex] [Payload with 0x55 escaping] [CRC16:4hex]

#define QFRAME_SYNC         0x55
#define QFRAME_MAX_PAYLOAD  502
#define QFRAME_MAX_RAW      600  // worst case: all 0x55 payload + overhead

#define QFRAME_TYPE_Q       'Q'  // command (host -> device)
#define QFRAME_TYPE_R       'R'  // response (device -> host)
#define QFRAME_TYPE_E       'E'  // error response
#define QFRAME_TYPE_F       'F'  // flash data
#define QFRAME_TYPE_P       'P'  // flash progress


typedef enum {
    QFP_IDLE,
    QFP_TYPE,
    QFP_LEN0,
    QFP_LEN1,
    QFP_LEN2,
    QFP_PAYLOAD,
    QFP_PAYLOAD_ESC,
    QFP_CRC0,
    QFP_CRC1,
    QFP_CRC2,
    QFP_CRC3,
    QFP_COMPLETE,
    QFP_ERROR,
} qframe_parse_state_t;

typedef struct {
    uint8_t     type;
    uint8_t     payload[QFRAME_MAX_PAYLOAD];
    uint16_t    payload_len;
    uint16_t    declared_len;
    uint16_t    crc_received;
    uint16_t    crc_computed;
    bool        crc_valid;
} qframe_t;

typedef struct {
    qframe_parse_state_t state;
    qframe_t    frame;
    uint16_t    raw_count;
    uint8_t     raw_buf[QFRAME_MAX_RAW];
    uint8_t     len_chars[3];
    uint8_t     crc_chars[4];
} qframe_parser_t;

void        qframe_parser_init(qframe_parser_t *p);
void        qframe_parser_reset(qframe_parser_t *p);

bool        qframe_parser_feed(qframe_parser_t *p, uint8_t byte);

bool        qframe_parser_complete(const qframe_parser_t *p);
bool        qframe_parser_error(const qframe_parser_t *p);
const qframe_t* qframe_parser_frame(const qframe_parser_t *p);

int         qframe_build(uint8_t type, const uint8_t *payload, uint16_t payload_len,
                         uint8_t *out_buf, uint16_t out_buf_size);

int         qframe_build_cmd(const char *cmd, uint8_t *out_buf, uint16_t out_buf_size);

int         hex_nibble(uint8_t c);
uint8_t     nibble_hex(uint8_t n);

const char *qframe_response_value(const char *resp);
