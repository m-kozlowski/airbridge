#include "uart_arbiter.h"
#include "app_config.h"
#include "debug_log.h"
#include <freertos/queue.h>

#define ARBITER_QUEUE_DEPTH     8
#define ARBITER_TASK_STACK      4096
#define ARBITER_TASK_PRIO       5
#define RX_TASK_STACK           3072
#define RX_TASK_PRIO            6
#define RX_BUF_SIZE             1024

static HardwareSerial *uart = nullptr;
static QueueHandle_t cmd_queue = nullptr;
static TaskHandle_t arbiter_task_handle = nullptr;
static TaskHandle_t rx_task_handle = nullptr;

static volatile system_state_t sys_state = SYS_IDLE;
static volatile uart_ticket_t *current_ticket = nullptr;
static SemaphoreHandle_t rx_ready = nullptr;
static qframe_parser_t rx_parser;
static qframe_t last_rx_frame;
static volatile bool rx_frame_available = false;

static volatile bool transparent_active = false;
static Stream *transparent_bridge = nullptr;
static qframe_parser_t transparent_parser;      // shadow parser for BDD sniffing

static uint32_t current_baud = 0;

static uint32_t stat_tx = 0;
static uint32_t stat_rx = 0;
static uint32_t stat_timeout = 0;
static uint32_t stat_error = 0;

static uint32_t next_ticket_id = 1;


typedef struct {
    uart_ticket_t *tickets[ARBITER_QUEUE_DEPTH];
    int count;
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t available;
} prio_queue_t;

static prio_queue_t pq;

static void pq_init() {
    pq.count = 0;
    pq.mutex = xSemaphoreCreateMutex();
    pq.available = xSemaphoreCreateCounting(ARBITER_QUEUE_DEPTH, 0);
}

static bool pq_push(uart_ticket_t *t) {
    xSemaphoreTake(pq.mutex, portMAX_DELAY);
    if (pq.count >= ARBITER_QUEUE_DEPTH) {
        xSemaphoreGive(pq.mutex);
        return false;
    }
    pq.tickets[pq.count++] = t;
    xSemaphoreGive(pq.mutex);
    xSemaphoreGive(pq.available);
    return true;
}

static uart_ticket_t* pq_pop(TickType_t wait) {
    if (xSemaphoreTake(pq.available, wait) != pdTRUE) {
        return nullptr;
    }
    xSemaphoreTake(pq.mutex, portMAX_DELAY);

    // highest prio value wins, then FIFO by ticket_id
    int best = 0;
    for (int i = 1; i < pq.count; i++) {
        if (pq.tickets[i]->priority > pq.tickets[best]->priority ||
            (pq.tickets[i]->priority == pq.tickets[best]->priority &&
             pq.tickets[i]->ticket_id < pq.tickets[best]->ticket_id)) {
            best = i;
        }
    }
    uart_ticket_t *t = pq.tickets[best];
    // Remove by shifting
    for (int i = best; i < pq.count - 1; i++) {
        pq.tickets[i] = pq.tickets[i + 1];
    }
    pq.count--;
    xSemaphoreGive(pq.mutex);
    return t;
}


static void rx_task(void *param) {
    uint8_t buf[64];
    qframe_parser_init(&rx_parser);

    while (true) {
        if (transparent_active) {
            // In transparent mode, forward raw bytes to bridge
            // Also feed a shadow parser to detect BDD R-frame responses
            int avail = uart->available();
            if (avail > 0) {
                int n = uart->readBytes(buf, min(avail, (int)sizeof(buf)));
                if (transparent_bridge && n > 0) {
                    transparent_bridge->write(buf, n);
                }
                // Sniff for BDD R-frame
                for (int i = 0; i < n; i++) {
                    if (qframe_parser_feed(&transparent_parser, buf[i])) {
                        const qframe_t *f = qframe_parser_frame(&transparent_parser);
                        if (f && f->crc_valid && f->type == QFRAME_TYPE_R &&
                            f->payload_len >= 12 &&
                            memcmp(f->payload, "P S #BDD = ", 11) == 0) {
                            // Parse the BDD key from "P S #BDD = XXXX"
                            char key_str[5] = {0};
                            memcpy(key_str, f->payload + 11, min((int)(f->payload_len - 11), 4));
                            uint16_t key = (uint16_t)strtoul(key_str, nullptr, 16);
                            uint32_t new_baud = Arbiter::bdd_key_to_baud(key);
                            if (new_baud && new_baud != current_baud) {
                                // Flush bridge output first (R-frame already forwarded)
                                if (transparent_bridge) transparent_bridge->flush();
                                vTaskDelay(pdMS_TO_TICKS(10));
                                uart->updateBaudRate(new_baud);
                                Log::logf(CAT_ARB, LOG_INFO, "[ARB] BDD transparent: baud %u -> %u\n",
                                            current_baud, new_baud);
                                current_baud = new_baud;
                            }
                        }
                        qframe_parser_reset(&transparent_parser);
                    }
                }
            } else {
                vTaskDelay(1);
            }
            continue;
        }

        int avail = uart->available();
        if (avail <= 0) {
            vTaskDelay(1);
            continue;
        }

        int n = uart->readBytes(buf, min(avail, (int)sizeof(buf)));
        for (int i = 0; i < n; i++) {
            if (qframe_parser_feed(&rx_parser, buf[i])) {
                // Complete frame
                const qframe_t *f = qframe_parser_frame(&rx_parser);
                if (f && f->crc_valid) {
                    memcpy((void*)&last_rx_frame, f, sizeof(qframe_t));
                    rx_frame_available = true;
                    stat_rx++;
                    Log::logf(CAT_ARB, LOG_DEBUG, "[ARB] RX frame type=%c len=%u t=%lu\n",
                              f->type, f->payload_len, millis());
                    if (rx_ready) {
                        xSemaphoreGive(rx_ready);
                    }
                } else {
                    stat_error++;
                    Log::logf(CAT_ARB, LOG_DEBUG, "[ARB] RX frame CRC error t=%lu\n", millis());
                }
                qframe_parser_reset(&rx_parser);
            }
        }
    }
}


static void arbiter_task(void *param) {
    while (true) {
        uart_ticket_t *t = pq_pop(pdMS_TO_TICKS(100));
        if (!t) continue;

        // Send frame
        current_ticket = t;
        rx_frame_available = false;
        xSemaphoreTake(rx_ready, 0);

        uart->write(t->frame, t->frame_len);
        uart->flush();
        stat_tx++;
        Log::logf(CAT_ARB, LOG_DEBUG, "[ARB] TX type=%c len=%u src=%d prio=%d t=%lu\n",
                  (char)t->frame[1], t->frame_len, t->source, t->priority, millis());

        if (t->no_ack) {
            t->success = true;
            t->timed_out = false;
            t->resp_len = 0;
        } else if (xSemaphoreTake(rx_ready, pdMS_TO_TICKS(t->timeout_ms)) == pdTRUE
            && rx_frame_available) {
            // Got response
            t->resp_type = last_rx_frame.type;
            t->resp_len = last_rx_frame.payload_len;
            if (t->resp_len > 0) {
                memcpy(t->resp_payload, last_rx_frame.payload,
                       min((int)t->resp_len, (int)sizeof(t->resp_payload)));
            }
            t->success = (last_rx_frame.type == QFRAME_TYPE_R);
            t->timed_out = false;
            if (last_rx_frame.type == QFRAME_TYPE_E) {
                stat_error++;
            }
        } else {
            t->success = false;
            t->timed_out = true;
            t->resp_len = 0;
            stat_timeout++;
        }

        current_ticket = nullptr;
        rx_frame_available = false;

        if (t->done) {
            xSemaphoreGive(t->done);
        } else {
            free(t);  // no_ack heap ticket, arbiter is sole owner
        }
    }
}


void Arbiter::init(HardwareSerial &serial, int rx_pin, int tx_pin, uint32_t baud) {
    uart = &serial;
    uart->setRxBufferSize(RX_BUF_SIZE);
    uart->begin(baud, SERIAL_8N1, rx_pin, tx_pin);
    current_baud = baud;

    rx_ready = xSemaphoreCreateBinary();
    qframe_parser_init(&transparent_parser);
    pq_init();

    xTaskCreatePinnedToCore(rx_task, "uart_rx", RX_TASK_STACK, nullptr,
                            RX_TASK_PRIO, &rx_task_handle, 1);
    xTaskCreatePinnedToCore(arbiter_task, "arbiter", ARBITER_TASK_STACK, nullptr,
                            ARBITER_TASK_PRIO, &arbiter_task_handle, 1);
}

bool Arbiter::submit(uart_ticket_t *ticket) {
    if (!ticket->done) {
        ticket->done = xSemaphoreCreateBinary();
    }
    ticket->ticket_id = next_ticket_id++;
    return pq_push(ticket);
}

bool Arbiter::send_cmd(const char *cmd, cmd_source_t src, cmd_priority_t prio,
                       char *resp_buf, uint16_t *resp_len, uint16_t timeout_ms)
{
    if (timeout_ms == 0) timeout_ms = Config::get().uart_cmd_timeout_ms;

    // Reject all commands while in transparent mode
    if (sys_state == SYS_TRANSPARENT) {
        if (resp_len) *resp_len = 0;
        return false;
    }

    uart_ticket_t ticket = {};
    ticket.source = src;
    ticket.priority = prio;
    ticket.timeout_ms = timeout_ms;
    ticket.done = xSemaphoreCreateBinary();

    int len = qframe_build_cmd(cmd, ticket.frame, sizeof(ticket.frame));
    if (len < 0) {
        vSemaphoreDelete(ticket.done);
        return false;
    }
    ticket.frame_len = len;

    if (!pq_push(&ticket)) {
        vSemaphoreDelete(ticket.done);
        return false;
    }

    xSemaphoreTake(ticket.done, pdMS_TO_TICKS(timeout_ms + 100));
    vSemaphoreDelete(ticket.done);

    // BDD baud switching (arbiter mode)
    if (ticket.success && strncmp(cmd, "P S #BDD ", 9) == 0 &&
        ticket.resp_len >= 12 &&
        memcmp(ticket.resp_payload, "P S #BDD = ", 11) == 0) {
        char key_str[5] = {0};
        memcpy(key_str, ticket.resp_payload + 11,
               min((int)(ticket.resp_len - 11), 4));
        uint16_t key = (uint16_t)strtoul(key_str, nullptr, 16);
        uint32_t new_baud = bdd_key_to_baud(key);
        if (new_baud && new_baud != current_baud) {
            uart->updateBaudRate(new_baud);
            Log::logf(CAT_ARB, LOG_INFO, "[ARB] BDD arbiter: baud %u -> %u\n",
                        current_baud, new_baud);
            current_baud = new_baud;
        }
    }

    if (resp_buf && ticket.resp_len > 0) {
        uint16_t copy_len = ticket.resp_len;
        if (resp_len && *resp_len > 0) {
            copy_len = min(copy_len, (uint16_t)(*resp_len - 1));
        }
        memcpy(resp_buf, ticket.resp_payload, copy_len);
        resp_buf[copy_len] = '\0';
    }
    if (resp_len) {
        *resp_len = ticket.resp_len;
    }

    return ticket.success;
}

bool Arbiter::send_frame(const uint8_t *frame, uint16_t frame_len,
                         cmd_source_t src, cmd_priority_t prio)
{
    if (sys_state == SYS_TRANSPARENT) return false;
    if (frame_len > QFRAME_MAX_RAW) return false;

    // Heap-allocated: arbiter owns the ticket after push and frees it after send.
    // Caller returns immediately — drop the frame if queue is full.
    uart_ticket_t *ticket = (uart_ticket_t*)malloc(sizeof(uart_ticket_t));
    if (!ticket) return false;

    memset(ticket, 0, sizeof(*ticket));
    ticket->source = src;
    ticket->priority = prio;
    ticket->no_ack = true;
    ticket->done = nullptr;  // no semaphore — arbiter frees ticket
    memcpy(ticket->frame, frame, frame_len);
    ticket->frame_len = frame_len;

    if (!pq_push(ticket)) {
        free(ticket);
        return false;
    }
    return true;
}

system_state_t Arbiter::get_state()         { return sys_state; }
void Arbiter::set_state(system_state_t s)   { sys_state = s; }

void Arbiter::enter_transparent(Stream *bridge) {
    transparent_bridge = bridge;
    qframe_parser_reset(&transparent_parser);
    transparent_active = true;
    sys_state = SYS_TRANSPARENT;
}

void Arbiter::exit_transparent() {
    transparent_active = false;
    transparent_bridge = nullptr;
    qframe_parser_reset(&rx_parser);
    qframe_parser_reset(&transparent_parser);
    sys_state = SYS_IDLE;
}

void Arbiter::write_raw(const uint8_t *data, size_t len) {
    if (uart && (transparent_active || sys_state == SYS_OTA_AIRSENSE)) {
        uart->write(data, len);
        uart->flush();
    }
}

bool Arbiter::wait_frame(qframe_t *out, uint16_t timeout_ms) {
    rx_frame_available = false;
    xSemaphoreTake(rx_ready, 0);  // clear stale
    if (xSemaphoreTake(rx_ready, pdMS_TO_TICKS(timeout_ms)) == pdTRUE
        && rx_frame_available) {
        if (out) memcpy(out, (void*)&last_rx_frame, sizeof(qframe_t));
        rx_frame_available = false;
        return true;
    }
    return false;
}

void Arbiter::set_baud(uint32_t baud) {
    if (uart && baud != current_baud) {
        uart->flush();
        uart->updateBaudRate(baud);
        // Flush RX hardware buffer (contains garbage from old baud)
        while (uart->available()) uart->read();
        qframe_parser_reset(&rx_parser);
        rx_frame_available = false;
        Log::logf(CAT_ARB, LOG_INFO, "[ARB] set_baud: %u -> %u\n", current_baud, baud);
        current_baud = baud;
    }
}

uint32_t Arbiter::get_baud() {
    return current_baud;
}

uint32_t Arbiter::bdd_key_to_baud(uint16_t key) {
    switch (key) {
        case 0: return 57600;
        case 1: return 115200;
        case 2: return 460800;
        default: return 0;
    }
}

uint32_t Arbiter::get_tx_count()       { return stat_tx; }
uint32_t Arbiter::get_rx_count()       { return stat_rx; }
uint32_t Arbiter::get_timeout_count()  { return stat_timeout; }
uint32_t Arbiter::get_error_count()    { return stat_error; }

void Arbiter::lcd_message(const char *msg) {
    char cmd[32], resp[8];
    uint16_t rlen;
    rlen = sizeof(resp);
    send_cmd("P S #LCA 0000", CMD_SRC_INTERNAL, CMD_PRIO_NORMAL, resp, &rlen);
    snprintf(cmd, sizeof(cmd), "P S #LCT %s", msg);
    rlen = sizeof(resp);
    send_cmd(cmd, CMD_SRC_INTERNAL, CMD_PRIO_NORMAL, resp, &rlen);
    rlen = sizeof(resp);
    send_cmd("P S #LCA 0001", CMD_SRC_INTERNAL, CMD_PRIO_NORMAL, resp, &rlen);
}

void Arbiter::lcd_clear() {
    char resp[8];
    uint16_t rlen = sizeof(resp);
    send_cmd("P S #LCA 0000", CMD_SRC_INTERNAL, CMD_PRIO_NORMAL, resp, &rlen);
}
