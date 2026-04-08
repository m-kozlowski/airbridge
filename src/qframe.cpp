#include "qframe.h"
#include "crc.h"
#include <string.h>

int hex_nibble(uint8_t c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

uint8_t nibble_hex(uint8_t n) {
    return (n < 10) ? ('0' + n) : ('A' + n - 10);
}


void qframe_parser_init(qframe_parser_t *p) {
    qframe_parser_reset(p);
}

void qframe_parser_reset(qframe_parser_t *p) {
    memset(p, 0, sizeof(*p));
    p->state = QFP_IDLE;
}

bool qframe_parser_complete(const qframe_parser_t *p) {
    return p->state == QFP_COMPLETE;
}

bool qframe_parser_error(const qframe_parser_t *p) {
    return p->state == QFP_ERROR;
}

const qframe_t* qframe_parser_frame(const qframe_parser_t *p) {
    return (p->state == QFP_COMPLETE) ? &p->frame : NULL;
}

bool qframe_parser_feed(qframe_parser_t *p, uint8_t byte) {
    if ((p->state == QFP_COMPLETE || p->state == QFP_ERROR) && byte == QFRAME_SYNC) {
        qframe_parser_reset(p);
    }

    switch (p->state) {
    case QFP_IDLE:
        if (byte == QFRAME_SYNC) {
            qframe_parser_reset(p);
            p->raw_buf[p->raw_count++] = byte;
            p->state = QFP_TYPE;
        }
        break;

    case QFP_TYPE:
        p->frame.type = byte;
        p->raw_buf[p->raw_count++] = byte;
        p->state = QFP_LEN0;
        break;

    case QFP_LEN0:
        p->len_chars[0] = byte;
        p->raw_buf[p->raw_count++] = byte;
        p->state = QFP_LEN1;
        break;

    case QFP_LEN1:
        p->len_chars[1] = byte;
        p->raw_buf[p->raw_count++] = byte;
        p->state = QFP_LEN2;
        break;

    case QFP_LEN2: {
        p->len_chars[2] = byte;
        p->raw_buf[p->raw_count++] = byte;

        int n0 = hex_nibble(p->len_chars[0]);
        int n1 = hex_nibble(p->len_chars[1]);
        int n2 = hex_nibble(p->len_chars[2]);
        if (n0 < 0 || n1 < 0 || n2 < 0) {
            p->state = QFP_ERROR;
            break;
        }
        p->frame.declared_len = (n0 << 8) | (n1 << 4) | n2;

        if (p->frame.declared_len < 9 || p->frame.declared_len > QFRAME_MAX_RAW) {
            p->state = QFP_ERROR;
            break;
        }
        p->frame.payload_len = 0;
        p->state = QFP_PAYLOAD;
        break;
    }

    case QFP_PAYLOAD:
        if (p->raw_count >= (p->frame.declared_len - 4)) {
            p->crc_chars[0] = byte;
            p->state = QFP_CRC1;
        } else if (byte == QFRAME_SYNC) {
            p->raw_buf[p->raw_count++] = byte;
            p->state = QFP_PAYLOAD_ESC;
        } else {
            p->raw_buf[p->raw_count++] = byte;
            if (p->frame.payload_len < QFRAME_MAX_PAYLOAD) {
                p->frame.payload[p->frame.payload_len++] = byte;
            }
        }
        break;

    case QFP_PAYLOAD_ESC:
        if (byte == QFRAME_SYNC) {
            p->raw_buf[p->raw_count++] = byte;
            if (p->frame.payload_len < QFRAME_MAX_PAYLOAD) {
                p->frame.payload[p->frame.payload_len++] = QFRAME_SYNC;
            }
            p->state = QFP_PAYLOAD;
        } else {
            // Restart with the first 0x55 as new sync
            qframe_parser_reset(p);
            p->raw_buf[p->raw_count++] = QFRAME_SYNC;
            p->state = QFP_TYPE;
            return qframe_parser_feed(p, byte);
        }
        break;

    case QFP_CRC1:
        p->crc_chars[1] = byte;
        p->state = QFP_CRC2;
        break;

    case QFP_CRC2:
        p->crc_chars[2] = byte;
        p->state = QFP_CRC3;
        break;

    case QFP_CRC3: {
        p->crc_chars[3] = byte;

        int c0 = hex_nibble(p->crc_chars[0]);
        int c1 = hex_nibble(p->crc_chars[1]);
        int c2 = hex_nibble(p->crc_chars[2]);
        int c3 = hex_nibble(p->crc_chars[3]);
        if (c0 < 0 || c1 < 0 || c2 < 0 || c3 < 0) {
            p->state = QFP_ERROR;
            break;
        }
        p->frame.crc_received = (c0 << 12) | (c1 << 8) | (c2 << 4) | c3;
        p->frame.crc_computed = crc16_ccitt(p->raw_buf, p->raw_count);
        p->frame.crc_valid = (p->frame.crc_received == p->frame.crc_computed);
        p->state = QFP_COMPLETE;
        return true;
    }

    case QFP_COMPLETE:
    case QFP_ERROR:
        break;
    }

    return false;
}


int qframe_build(uint8_t type, const uint8_t *payload, uint16_t payload_len,
                 uint8_t *out_buf, uint16_t out_buf_size)
{
    if (payload_len > QFRAME_MAX_PAYLOAD) return -1;

    uint16_t esc_len = 0;
    for (uint16_t i = 0; i < payload_len; i++) {
        esc_len += (payload[i] == QFRAME_SYNC) ? 2 : 1;
    }

    uint16_t total = 1 + 1 + 3 + esc_len + 4;
    if (total > out_buf_size || total > 0xFFF) return -1;

    uint16_t pos = 0;

    out_buf[pos++] = QFRAME_SYNC;

    out_buf[pos++] = type;

    out_buf[pos++] = nibble_hex((total >> 8) & 0xF);
    out_buf[pos++] = nibble_hex((total >> 4) & 0xF);
    out_buf[pos++] = nibble_hex(total & 0xF);

    for (uint16_t i = 0; i < payload_len; i++) {
        out_buf[pos++] = payload[i];
        if (payload[i] == QFRAME_SYNC) {
            out_buf[pos++] = QFRAME_SYNC;
        }
    }

    uint16_t crc = crc16_ccitt(out_buf, pos);
    out_buf[pos++] = nibble_hex((crc >> 12) & 0xF);
    out_buf[pos++] = nibble_hex((crc >> 8) & 0xF);
    out_buf[pos++] = nibble_hex((crc >> 4) & 0xF);
    out_buf[pos++] = nibble_hex(crc & 0xF);

    return (int)pos;
}

int qframe_build_cmd(const char *cmd, uint8_t *out_buf, uint16_t out_buf_size) {
    return qframe_build(QFRAME_TYPE_Q, (const uint8_t *)cmd, strlen(cmd),
                        out_buf, out_buf_size);
}

const char *qframe_response_value(const char *resp) {
    if (!resp) return NULL;
    const char *eq = strstr(resp, "= ");
    return eq ? eq + 2 : NULL;
}
