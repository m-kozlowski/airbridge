#include "uart_arbiter.h"
#include "app_config.h"
#include "debug_log.h"
#include "live_stream.h"
#include "custom_settings.h"
#include "hex_util.h"
#include <driver/gpio.h>
#include <freertos/queue.h>
#include <esp_timer.h>
#include <atomic>
#include <cstddef>

#define ARBITER_QUEUE_DEPTH     8
#define ARBITER_TASK_STACK      4096
#define ARBITER_TASK_PRIO       5
#define RX_TASK_STACK           3072
#define RX_TASK_PRIO            6
#define RX_BUF_SIZE             1024
#define RX_FRAME_QUEUE_DEPTH    4
#define FRAME_LISTENER_MAX      4

static HardwareSerial *uart = nullptr;
static TaskHandle_t arbiter_task_handle = nullptr;
static TaskHandle_t rx_task_handle = nullptr;

static SemaphoreHandle_t rx_ready = nullptr;

static volatile system_state_t sys_state = SYS_IDLE;
static qframe_parser_t rx_parser;
static std::atomic<bool> rx_reset_requested{false};

static portMUX_TYPE rx_frame_mux = portMUX_INITIALIZER_UNLOCKED;
static qframe_t rx_frame_storage[RX_FRAME_QUEUE_DEPTH];
static uint8_t rx_frame_queue[RX_FRAME_QUEUE_DEPTH];
static uint8_t rx_frame_slot_state[RX_FRAME_QUEUE_DEPTH];
static uint8_t rx_frame_head = 0;
static uint8_t rx_frame_tail = 0;
static uint8_t rx_frame_count = 0;
static uint32_t rx_frame_epoch = 0;

typedef struct {
    bool active;
    uint16_t inflight;
    qframe_type_mask_t types;
    uart_frame_sink_t sink;
    void *context;
} frame_listener_slot_t;

static portMUX_TYPE frame_listener_mux = portMUX_INITIALIZER_UNLOCKED;
static frame_listener_slot_t frame_listeners[FRAME_LISTENER_MAX];

static volatile bool transparent_active = false;
static Stream *transparent_bridge = nullptr;
static portMUX_TYPE transparent_mux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t transparent_inflight = 0;
static volatile uint32_t transparent_last_activity = 0;
static uint32_t transparent_pending_baud = 0;
static uint32_t transparent_pending_baud_at = 0;
static uint32_t transparent_reboot_baud_at = 0;

static const uint32_t TRANSPARENT_REBOOT_BAUD_DELAY_MS = 25;
static const uint32_t RESMED_DEFAULT_BAUD = 57600;

static qframe_parse_state_t transparent_tx_state = QFP_IDLE;
static uint8_t transparent_tx_type = 0;
static uint16_t transparent_tx_declared_len = 0;
static uint16_t transparent_tx_raw_count = 0;
static uint8_t transparent_tx_payload_len = 0;
static uint8_t transparent_tx_len_chars[3];
static uint8_t transparent_tx_payload[17];

static uint32_t current_baud = 0;

static uint32_t stat_tx = 0;
static uint32_t stat_rx = 0;
static uint32_t stat_l_rx = 0;
static uint32_t stat_timeout = 0;
static uint32_t stat_error = 0;

static uint32_t next_ticket_id = 1;

static constexpr uint32_t SILENT_FRAME_GUARD_MS = 80;
static std::atomic<uint32_t> acquisition_until{0};
static std::atomic<uint32_t> acquisition_oxh_from{0};

static uint32_t acquisition_remaining_ms() {
    uint32_t until = acquisition_until.load(std::memory_order_acquire);
    if (!until) return 0;
    const int32_t remaining = int32_t(until - millis());
    if (remaining > 0) return uint32_t(remaining);
    acquisition_until.compare_exchange_strong(until, 0, std::memory_order_acq_rel);
    return 0;
}

static constexpr size_t LCD_COMMAND_SIZE = sizeof("P S #LCT ") + Arbiter::LCD_TEXT_MAX;
static constexpr size_t LCD_FRAME_SIZE = 9 + 2 * (LCD_COMMAND_SIZE - 1);

enum class LcdStep : uint8_t { None, Hide, Text, Show, Clear, Expire };

struct lcd_request_t {
    char text[LCD_COMMAND_SIZE - sizeof("P S #LCT ") + 1];
    uint32_t timeout_ms;
    uint32_t expires_at;
    bool idle_only;
};

static uint32_t lcd_clear_at = 0;
static bool lcd_clear_pending = false;

struct alignas(std::max_align_t) uart_transaction_t {
    cmd_source_t source;
    cmd_priority_t priority;
    uint8_t *frame;
    uint16_t frame_len;
    uint32_t ticket_id;
    uart_response_policy_t policy;
    uart_frame_sink_t sink;
    void *sink_context;
    size_t context_size;
    uint32_t queued_ms;
    uint32_t sent_after_ms;
    uart_send_window_t send_window;
    unsigned references;

    uart_transaction_result_t result;

    bool completed;
    bool cancelled;
    LcdStep lcd_step;
    SemaphoreHandle_t done;
};

static std::atomic<uart_transaction_t *> current_ticket{nullptr};


typedef struct {
    uart_transaction_t *tickets[ARBITER_QUEUE_DEPTH];
    int count;
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t available;
} prio_queue_t;

static prio_queue_t pq;

static uint32_t transaction_remaining_ms(const uart_transaction_t *t) {
    const uint32_t elapsed = millis() - t->queued_ms;
    // TX can start between these reads; a newer TX timestamp has zero age.
    const uint32_t sent_after = __atomic_load_n(&t->sent_after_ms, __ATOMIC_ACQUIRE);
    const bool sent = sent_after != UINT32_MAX;
    const uint32_t phase_elapsed = sent
        ? (elapsed > sent_after ? elapsed - sent_after : 0) : elapsed;
    const uint32_t phase_limit = sent ? t->policy.response_timeout_ms
                                    : t->policy.queue_timeout_ms;
    uint32_t remaining = phase_elapsed < phase_limit ? phase_limit - phase_elapsed : 0;
    if (t->policy.overall_timeout_ms) {
        const uint32_t hard_left = elapsed < t->policy.overall_timeout_ms
            ? t->policy.overall_timeout_ms - elapsed : 0;
        remaining = min(remaining, hard_left);
    }
    return remaining;
}

static void release_ticket(uart_transaction_t *t) {
    if (__atomic_sub_fetch(&t->references, 1, __ATOMIC_ACQ_REL) == 0) {
        if (t->done) vSemaphoreDelete(t->done);
        free(t);
    }
}

static bool pq_remove(uart_transaction_t *t) {
    bool removed = false;
    xSemaphoreTake(pq.mutex, portMAX_DELAY);
    for (int i = 0; i < pq.count; i++) {
        if (pq.tickets[i] != t) continue;
        for (int j = i; j + 1 < pq.count; j++) pq.tickets[j] = pq.tickets[j + 1];
        pq.count--;
        removed = true;
        break;
    }
    xSemaphoreGive(pq.mutex);
    return removed;
}

static void pq_init() {
    pq.count = 0;
    pq.mutex = xSemaphoreCreateMutex();
    pq.available = xSemaphoreCreateBinary();
}

static bool pq_push(uart_transaction_t *t, TickType_t wait = portMAX_DELAY) {
    if (xSemaphoreTake(pq.mutex, wait) != pdTRUE) return false;
    if (pq.count >= ARBITER_QUEUE_DEPTH) {
        xSemaphoreGive(pq.mutex);
        return false;
    }
    pq.tickets[pq.count++] = t;
    xSemaphoreGive(pq.mutex);
    xSemaphoreGive(pq.available);
    return true;
}

static uart_transaction_t* pq_pop(TickType_t wait) {
    const TickType_t started = xTaskGetTickCount();
    for (;;) {
        const TickType_t elapsed = xTaskGetTickCount() - started;
        TickType_t remaining = elapsed < wait ? wait - elapsed : 0;
        xSemaphoreTake(pq.mutex, portMAX_DELAY);
        const int64_t now_us = esp_timer_get_time();
        int best = -1;
        for (int i = 0; i < pq.count; i++) {
            const auto *candidate = pq.tickets[i];
            int64_t delay_us = candidate->send_window.not_before_us - now_us;
            if (candidate->send_window.outside_acquisition)
                delay_us = max(delay_us, int64_t(acquisition_remaining_ms()) * 1000);
            const uint32_t queue_left = transaction_remaining_ms(candidate);
            if (queue_left && !__atomic_load_n(&candidate->cancelled, __ATOMIC_ACQUIRE)) {
                // Do not start non-preemptible work across an admitted TX window.
                // Higher-priority safety work may still cause a window to miss.
                const int64_t wire_us = current_baud
                    ? (int64_t(candidate->frame_len) * 10000000 + current_baud - 1) / current_baud : 0;
                const int64_t busy_us = wire_us +
                    int64_t(candidate->policy.response_timeout_ms + portTICK_PERIOD_MS) * 1000;
                for (int j = 0; j < pq.count; ++j) {
                    const auto *reserved = pq.tickets[j];
                    if (reserved == candidate || !reserved->send_window.before_us ||
                        (candidate->send_window.before_us &&
                         candidate->send_window.before_us <= reserved->send_window.before_us) ||
                        reserved->send_window.before_us <= now_us ||
                        reserved->priority < candidate->priority ||
                        (reserved->send_window.idle_only && sys_state != SYS_IDLE) ||
                        __atomic_load_n(&reserved->cancelled, __ATOMIC_ACQUIRE) ||
                        !transaction_remaining_ms(reserved)) continue;
                    const int64_t edge = reserved->send_window.not_before_us;
                    if (edge > now_us && now_us + busy_us >= edge)
                        delay_us = max(delay_us, edge - now_us);
                }
            }
            if (delay_us > 0 && !__atomic_load_n(&candidate->cancelled, __ATOMIC_ACQUIRE) &&
                queue_left) {
                const int64_t tick_us = int64_t(portTICK_PERIOD_MS) * 1000;
                delay_us = min(delay_us, int64_t(queue_left) * 1000);
                remaining = min(remaining, TickType_t((delay_us + tick_us - 1) / tick_us));
                continue;
            }
            // At equal priority, an open TX window precedes ordinary FIFO work.
            const bool timed = candidate->send_window.before_us != 0;
            const bool best_timed = best >= 0 && pq.tickets[best]->send_window.before_us != 0;
            if (best < 0 || candidate->priority > pq.tickets[best]->priority ||
                (candidate->priority == pq.tickets[best]->priority &&
                 ((timed && !best_timed) || (timed == best_timed &&
                  (timed ? candidate->send_window.before_us < pq.tickets[best]->send_window.before_us
                         : candidate->ticket_id < pq.tickets[best]->ticket_id))))) best = i;
        }
        if (best >= 0) {
            auto *t = pq.tickets[best];
            for (int i = best; i + 1 < pq.count; i++) pq.tickets[i] = pq.tickets[i + 1];
            pq.count--;
            xSemaphoreGive(pq.mutex);
            return t;
        }
        xSemaphoreGive(pq.mutex);
        if (!remaining) return nullptr;
        xSemaphoreTake(pq.available, remaining);
    }
}

static uint32_t parse_bdd_baud(const uint8_t *payload, uint16_t len);
static void transparent_apply_baud(uint32_t new_baud, const char *reason);

static bool uart_source_allowed(cmd_source_t src) {
    system_state_t st = sys_state;
    if (st == SYS_TRANSPARENT) return false;
    if (st == SYS_OTA_AIRSENSE && src != CMD_SRC_OTA) return false;
    return true;
}

typedef enum {
    RX_SLOT_FREE,
    RX_SLOT_RESERVED,
    RX_SLOT_QUEUED,
} rx_slot_state_t;

typedef enum {
    RX_PUSH_STORED,
    RX_PUSH_STORED_DROPPED_OLD,
    RX_PUSH_DROPPED_FULL,
    RX_PUSH_DROPPED_STALE,
} rx_push_result_t;

static int rx_frame_priority(uint8_t type) {
    switch (type) {
        case QFRAME_TYPE_E: return 4;
        case QFRAME_TYPE_R: return 3;
        case QFRAME_TYPE_K:
        case QFRAME_TYPE_P:
        case QFRAME_TYPE_O: return 2;
        default: return 1;
    }
}

static uint8_t rx_queue_slot(uint8_t logical_index) {
    return (rx_frame_tail + logical_index) % RX_FRAME_QUEUE_DEPTH;
}

static int rx_find_free_slot() {
    for (uint8_t i = 0; i < RX_FRAME_QUEUE_DEPTH; i++) {
        if (rx_frame_slot_state[i] == RX_SLOT_FREE) {
            return i;
        }
    }
    return -1;
}

static uint8_t rx_queue_clear_locked() {
    uint8_t cleared = rx_frame_count;
    for (uint8_t i = 0; i < rx_frame_count; i++) {
        uint8_t slot = rx_frame_queue[rx_queue_slot(i)];
        rx_frame_slot_state[slot] = RX_SLOT_FREE;
    }
    rx_frame_head = 0;
    rx_frame_tail = 0;
    rx_frame_count = 0;
    rx_frame_epoch++;
    return cleared;
}

static uint8_t rx_queue_drop_at(uint8_t logical_index) {
    uint8_t dropped_slot = rx_frame_queue[rx_queue_slot(logical_index)];

    for (uint8_t i = logical_index; i + 1 < rx_frame_count; i++) {
        rx_frame_queue[rx_queue_slot(i)] = rx_frame_queue[rx_queue_slot(i + 1)];
    }
    rx_frame_head = (rx_frame_head + RX_FRAME_QUEUE_DEPTH - 1) % RX_FRAME_QUEUE_DEPTH;
    rx_frame_count--;

    return dropped_slot;
}

static rx_push_result_t rx_queue_push(const qframe_t *frame) {
    bool overflow = false;
    uint8_t slot = 0;
    uint32_t epoch = 0;

    portENTER_CRITICAL(&rx_frame_mux);
    if (rx_frame_count >= RX_FRAME_QUEUE_DEPTH) {
        int new_prio = rx_frame_priority(frame->type);
        uint8_t drop_index = 0;
        bool found_lower_priority = false;

        for (uint8_t i = 0; i < rx_frame_count; i++) {
            uint8_t queued_slot = rx_frame_queue[rx_queue_slot(i)];
            if (rx_frame_priority(rx_frame_storage[queued_slot].type) < new_prio) {
                drop_index = i;
                found_lower_priority = true;
                break;
            }
        }

        if (!found_lower_priority) {
            portEXIT_CRITICAL(&rx_frame_mux);
            return RX_PUSH_DROPPED_FULL;
        }

        slot = rx_queue_drop_at(drop_index);
        overflow = true;
    } else {
        int free_slot = rx_find_free_slot();
        if (free_slot < 0) {
            portEXIT_CRITICAL(&rx_frame_mux);
            return RX_PUSH_DROPPED_FULL;
        }
        slot = (uint8_t)free_slot;
    }

    rx_frame_slot_state[slot] = RX_SLOT_RESERVED;
    epoch = rx_frame_epoch;
    portEXIT_CRITICAL(&rx_frame_mux);

    qframe_copy(&rx_frame_storage[slot], frame);

    portENTER_CRITICAL(&rx_frame_mux);
    if (epoch != rx_frame_epoch) {
        rx_frame_slot_state[slot] = RX_SLOT_FREE;
        portEXIT_CRITICAL(&rx_frame_mux);
        return RX_PUSH_DROPPED_STALE;
    }
    rx_frame_queue[rx_frame_head] = slot;
    rx_frame_head = (rx_frame_head + 1) % RX_FRAME_QUEUE_DEPTH;
    rx_frame_count++;
    rx_frame_slot_state[slot] = RX_SLOT_QUEUED;
    portEXIT_CRITICAL(&rx_frame_mux);

    return overflow ? RX_PUSH_STORED_DROPPED_OLD : RX_PUSH_STORED;
}

static bool rx_queue_pop(qframe_t *out) {
    bool ok = false;

    portENTER_CRITICAL(&rx_frame_mux);
    if (rx_frame_count > 0) {
        uint8_t slot = rx_frame_queue[rx_frame_tail];
        rx_frame_tail = (rx_frame_tail + 1) % RX_FRAME_QUEUE_DEPTH;
        rx_frame_count--;
        if (out) qframe_copy(out, &rx_frame_storage[slot]);
        rx_frame_slot_state[slot] = RX_SLOT_FREE;
        ok = true;
    }
    portEXIT_CRITICAL(&rx_frame_mux);

    return ok;
}

static void dispatch_frame_listeners(const qframe_t *frame) {
    uart_frame_sink_t sinks[FRAME_LISTENER_MAX] = {};
    void *contexts[FRAME_LISTENER_MAX] = {};
    uint8_t slots[FRAME_LISTENER_MAX] = {};
    uint8_t count = 0;
    qframe_type_mask_t type_mask = qframe_type_mask(frame->type);

    if (!type_mask) return;

    portENTER_CRITICAL(&frame_listener_mux);
    for (uint8_t i = 0; i < FRAME_LISTENER_MAX; i++) {
        if (!frame_listeners[i].active ||
            !(frame_listeners[i].types & type_mask)) {
            continue;
        }
        sinks[count] = frame_listeners[i].sink;
        contexts[count] = frame_listeners[i].context;
        slots[count] = i;
        frame_listeners[i].inflight++;
        count++;
    }
    portEXIT_CRITICAL(&frame_listener_mux);

    for (uint8_t i = 0; i < count; i++) {
        if (sinks[i]) (void)sinks[i](frame, contexts[i]);
        portENTER_CRITICAL(&frame_listener_mux);
        if (frame_listeners[slots[i]].inflight > 0) {
            frame_listeners[slots[i]].inflight--;
        }
        portEXIT_CRITICAL(&frame_listener_mux);
    }
}

static void transparent_apply_baud(uint32_t new_baud, const char *reason) {
    if (new_baud && new_baud != current_baud) {
        vTaskDelay(pdMS_TO_TICKS(10));
        uart->updateBaudRate(new_baud);
        Log::logf(CAT_ARB, LOG_INFO,
                  "transparent baud (%s): %u -> %u\n",
                  reason ? reason : "unknown", current_baud, new_baud);
        current_baud = new_baud;
    }
}

static const char *transparent_reboot_command(const uint8_t *payload,
                                              uint16_t len) {
    if (len < 13 || payload[8] != ' ') return nullptr;

    const char *name = nullptr;
    if (memcmp(payload, "P S #RES", 8) == 0) {
        name = "RES";
    } else if (memcmp(payload, "P S #BLL", 8) == 0) {
        name = "BLL";
    } else {
        return nullptr;
    }

    const char *value = (const char *)payload + 9;
    char *end = nullptr;
    unsigned long raw = strtoul(value, &end, 16);
    if (end == value || raw == 0) return nullptr;

    return name;
}

static void transparent_schedule_reboot_baud(const char *cmd) {
    transparent_reboot_baud_at = millis() + TRANSPARENT_REBOOT_BAUD_DELAY_MS;
    Log::logf(CAT_ARB, LOG_DEBUG,
              "transparent %s pending default baud in %ums\n",
              cmd ? cmd : "reset", TRANSPARENT_REBOOT_BAUD_DELAY_MS);
}

static void transparent_check_reboot_baud(bool force) {
    if (!transparent_reboot_baud_at) return;
    if (!force && (int32_t)(millis() - transparent_reboot_baud_at) < 0) return;

    transparent_reboot_baud_at = 0;
    transparent_pending_baud = 0;
    transparent_pending_baud_at = 0;
    transparent_apply_baud(RESMED_DEFAULT_BAUD, "reset");
}

static void transparent_tx_reset() {
    transparent_tx_state = QFP_IDLE;
    transparent_tx_type = 0;
    transparent_tx_declared_len = 0;
    transparent_tx_raw_count = 0;
    transparent_tx_payload_len = 0;
}

static void transparent_tx_handle_frame() {
    if (transparent_tx_type != QFRAME_TYPE_Q) return;

    transparent_tx_payload[min((size_t)transparent_tx_payload_len,
                               sizeof(transparent_tx_payload) - 1)] = '\0';
    if (transparent_tx_payload_len >= 4 &&
        memcmp(transparent_tx_payload, "P F ", 4) == 0) {
        CustomSettings::invalidate("external flash");
    }

    uint32_t new_baud = parse_bdd_baud(transparent_tx_payload,
                                       transparent_tx_payload_len);
    if (new_baud) {
        transparent_pending_baud = new_baud;
        transparent_pending_baud_at = millis();
        Log::logf(CAT_ARB, LOG_DEBUG,
                  "BDD transparent pending baud %u\n", new_baud);
    }

    const char *reboot_cmd = transparent_reboot_command(transparent_tx_payload,
                                                        transparent_tx_payload_len);
    if (reboot_cmd) {
        CustomSettings::invalidate(reboot_cmd);
        transparent_schedule_reboot_baud(reboot_cmd);
    }
}

static void transparent_tx_feed(uint8_t byte) {
    switch (transparent_tx_state) {
    case QFP_IDLE:
        if (byte == QFRAME_SYNC) {
            transparent_tx_reset();
            transparent_tx_raw_count = 1;
            transparent_tx_state = QFP_TYPE;
        }
        break;

    case QFP_TYPE:
        transparent_tx_type = byte;
        transparent_tx_raw_count++;
        transparent_tx_state = QFP_LEN0;
        break;

    case QFP_LEN0:
        transparent_tx_len_chars[0] = byte;
        transparent_tx_raw_count++;
        transparent_tx_state = QFP_LEN1;
        break;

    case QFP_LEN1:
        transparent_tx_len_chars[1] = byte;
        transparent_tx_raw_count++;
        transparent_tx_state = QFP_LEN2;
        break;

    case QFP_LEN2: {
        transparent_tx_len_chars[2] = byte;
        transparent_tx_raw_count++;
        int n0 = hex_nibble(transparent_tx_len_chars[0]);
        int n1 = hex_nibble(transparent_tx_len_chars[1]);
        int n2 = hex_nibble(transparent_tx_len_chars[2]);
        if (n0 < 0 || n1 < 0 || n2 < 0) {
            transparent_tx_reset();
            break;
        }
        transparent_tx_declared_len = (n0 << 8) | (n1 << 4) | n2;
        if (transparent_tx_declared_len < 9 || transparent_tx_declared_len > QFRAME_MAX_RAW) {
            transparent_tx_reset();
            break;
        }
        transparent_tx_state = QFP_PAYLOAD;
        break;
    }

    case QFP_PAYLOAD:
        if (transparent_tx_raw_count >= (transparent_tx_declared_len - 4)) {
            transparent_tx_state = QFP_CRC1;
        } else if (byte == QFRAME_SYNC) {
            transparent_tx_raw_count++;
            transparent_tx_state = QFP_PAYLOAD_ESC;
        } else {
            transparent_tx_raw_count++;
            if (transparent_tx_payload_len < sizeof(transparent_tx_payload)) {
                transparent_tx_payload[transparent_tx_payload_len++] = byte;
            }
        }
        break;

    case QFP_PAYLOAD_ESC:
        transparent_tx_raw_count++;
        if (byte == QFRAME_SYNC) {
            if (transparent_tx_payload_len < sizeof(transparent_tx_payload)) {
                transparent_tx_payload[transparent_tx_payload_len++] = QFRAME_SYNC;
            }
            transparent_tx_state = QFP_PAYLOAD;
        } else {
            transparent_tx_reset();
            transparent_tx_feed(byte);
        }
        break;

    case QFP_CRC1:
        transparent_tx_state = QFP_CRC2;
        break;

    case QFP_CRC2:
        transparent_tx_state = QFP_CRC3;
        break;

    case QFP_CRC3:
        transparent_tx_handle_frame();
        transparent_tx_reset();
        break;

    case QFP_COMPLETE:
    case QFP_ERROR:
        transparent_tx_reset();
        break;
    }
}

static void transparent_sniff_tx(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        transparent_tx_feed(data[i]);
    }
}


static void rx_task(void *param) {
    uint8_t buf[64];
    qframe_parser_init(&rx_parser);
    bool crc_failed = false;
    bool queue_failed = false;

    while (true) {
        portENTER_CRITICAL(&transparent_mux);
        bool transparent = transparent_active;
        Stream *bridge = transparent_bridge;
        if (transparent && bridge) transparent_inflight++;
        bool reset_parser = rx_reset_requested.exchange(false);
        portEXIT_CRITICAL(&transparent_mux);
        if (reset_parser) qframe_parser_reset(&rx_parser);

        if (transparent) {
            if (!bridge) {
                vTaskDelay(1);
                continue;
            }
            transparent_check_reboot_baud(false);
            if (transparent_pending_baud &&
                (uint32_t)(millis() - transparent_pending_baud_at) > 1000) {
                Log::logf(CAT_ARB, LOG_WARN,
                          "BDD transparent pending baud %u expired\n",
                          transparent_pending_baud);
                transparent_pending_baud = 0;
                transparent_pending_baud_at = 0;
            }

            // In transparent mode, forward raw bytes to bridge
            // Also feed a shadow parser to detect BDD R-frame responses
            int avail = uart->available();
            if (avail > 0) {
                int n = uart->readBytes(buf, min(avail, (int)sizeof(buf)));
                transparent_last_activity = millis();
                Log::logf(CAT_ARB, LOG_DEBUG, "TRANSP RX %d bytes t=%lu\n", n, millis());
                if (n > 0) {
                    bridge->write(buf, n);
                }
                // Sniff for BDD R-frame
                for (int i = 0; i < n; i++) {
                    if (qframe_parser_feed(&rx_parser, buf[i])) {
                        const qframe_t *f = qframe_parser_frame(&rx_parser);
                        if (f && f->crc_valid) {
                            char pl[48] = {};
                            int plen = f->payload_len < sizeof(pl)-1 ? f->payload_len : sizeof(pl)-1;
                            memcpy(pl, f->payload, plen);
                            Log::logf(CAT_ARB, LOG_DEBUG, "Shadow type=%c len=%u: %s\n",
                                      (char)f->type, f->payload_len, pl);
                        } else {
                            Log::logf(CAT_ARB, LOG_DEBUG, "Shadow type=%c len=%u crc=BAD\n",
                                      f ? (char)f->type : '?', f ? f->payload_len : 0);
                        }
                        if (f && f->crc_valid && f->type == QFRAME_TYPE_R) {
                            uint32_t new_baud = parse_bdd_baud(f->payload, f->payload_len);
                            if (!new_baud && transparent_pending_baud) {
                                new_baud = transparent_pending_baud;
                            }
                            transparent_pending_baud = 0;
                            transparent_pending_baud_at = 0;
                            transparent_apply_baud(new_baud, "BDD");
                        } else if (f && f->crc_valid && f->type == QFRAME_TYPE_E) {
                            transparent_pending_baud = 0;
                            transparent_pending_baud_at = 0;
                            transparent_reboot_baud_at = 0;
                        }
                        qframe_parser_reset(&rx_parser);
                    }
                }
            } else {
                vTaskDelay(1);
            }
            portENTER_CRITICAL(&transparent_mux);
            transparent_inflight--;
            portEXIT_CRITICAL(&transparent_mux);
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
                    if (crc_failed) Log::logf(CAT_ARB, LOG_INFO, "RX CRC recovered\n");
                    crc_failed = false;
                    dispatch_frame_listeners(f);
                    if (f->type == QFRAME_TYPE_L) {
                        // Unsolicited live stream sample. Route to LiveStream
                        stat_rx++;
                        stat_l_rx++;
                        LiveStream::on_l_frame(f->payload, f->payload_len);
                    } else {
                        stat_rx++;
                        Log::logf(CAT_ARB, LOG_DEBUG, "RX-%c arrived %.32s t=%lu\n",
                                  char(f->type), f->payload, millis());
                        rx_push_result_t push_result = rx_queue_push(f);
                        if (push_result == RX_PUSH_STORED ||
                            push_result == RX_PUSH_STORED_DROPPED_OLD) {
                            if (rx_ready) {
                                xSemaphoreGive(rx_ready);
                            }
                        }
                        if (push_result == RX_PUSH_STORED_DROPPED_OLD) {
                            stat_error++;
                            Log::logf(CAT_ARB, queue_failed ? LOG_DEBUG : LOG_WARN,
                                      "RX queue full, dropped queued frame t=%lu\n",
                                      millis());
                        } else if (push_result == RX_PUSH_DROPPED_FULL) {
                            stat_error++;
                            Log::logf(CAT_ARB, queue_failed ? LOG_DEBUG : LOG_WARN,
                                      "RX queue full, dropped incoming frame t=%lu\n",
                                      millis());
                        }
                        bool dropped = push_result != RX_PUSH_STORED;
                        if (queue_failed && !dropped)
                            Log::logf(CAT_ARB, LOG_INFO, "RX queue recovered\n");
                        queue_failed = dropped;
                    }
                } else {
                    stat_error++;
                    Log::logf(CAT_ARB, crc_failed ? LOG_DEBUG : LOG_WARN,
                              "RX frame CRC error t=%lu\n", millis());
                    crc_failed = true;
                }
                qframe_parser_reset(&rx_parser);
            }
        }
    }
}


static void lcd_check();
static bool lcd_advance(uart_transaction_t *t);

static bool transaction_source_allowed(const uart_transaction_t *t) {
    if (t->send_window.idle_only && sys_state != SYS_IDLE) return false;
    if (t->lcd_step != LcdStep::None) {
        const auto *request = static_cast<const lcd_request_t *>(t->sink_context);
        if (sys_state != SYS_IDLE &&
            (request->idle_only || sys_state != SYS_THERAPY)) return false;
    }
    return uart_source_allowed(t->source);
}

static void finish_ticket(uart_transaction_t *t) {
    if (t->lcd_step == LcdStep::Expire) lcd_clear_pending = false;
    __atomic_store_n(&t->completed, true, __ATOMIC_RELEASE);
    if (t->done) xSemaphoreGive(t->done);
    release_ticket(t);
}

static uint32_t transaction_wait_ms(const uart_transaction_t *t,
                                    uint32_t started_ms,
                                    bool accepted_any,
                                    uint32_t last_accepted_ms,
                                    bool *idle_wait) {
    uint32_t now = millis();
    uint32_t elapsed = (uint32_t)(now - started_ms);
    uint32_t overall_left = transaction_remaining_ms(t);
    *idle_wait = false;
    if (!overall_left) return 0;

    if (!accepted_any) {
        uint32_t first_left = elapsed < t->policy.first_timeout_ms
                            ? t->policy.first_timeout_ms - elapsed : 0;
        return min(overall_left, first_left);
    }
    if (t->policy.complete_on_idle) {
        uint32_t idle_elapsed = (uint32_t)(now - last_accepted_ms);
        uint32_t idle_left = idle_elapsed < t->policy.interframe_timeout_ms
                           ? t->policy.interframe_timeout_ms - idle_elapsed : 0;
        *idle_wait = idle_left <= overall_left;
        return min(overall_left, idle_left);
    }
    return overall_left;
}

static void arbiter_task(void *param) {
    uart_transaction_t *t = nullptr;
    while (true) {
        if (!t) {
            lcd_check();
            t = pq_pop(pdMS_TO_TICKS(100));
            if (!t) continue;

            if (t->lcd_step == LcdStep::Expire) {
                auto *request = static_cast<lcd_request_t *>(t->sink_context);
                if (request->expires_at != lcd_clear_at) {
                    finish_ticket(t);
                    t = nullptr;
                    continue;
                }
            }
        }

        if (__atomic_load_n(&t->cancelled, __ATOMIC_ACQUIRE)) {
            finish_ticket(t);
            t = nullptr;
            continue;
        }
        if (!transaction_remaining_ms(t)) {
            t->result.timed_out = true;
            stat_timeout++;
            finish_ticket(t);
            t = nullptr;
            continue;
        }

        if (!transaction_source_allowed(t)) {
            t->result.success = false;
            stat_error++;
            Log::logf(CAT_ARB, LOG_DEBUG,
                      "TX blocked by state=%s src=%d t=%lu\n",
                      system_state_name(sys_state), t->source, millis());
            finish_ticket(t);
            t = nullptr;
            continue;
        }

        // Send frame
        current_ticket = t;
        if (!transaction_source_allowed(t)) {
            current_ticket = nullptr;
            t->result.success = false;
            stat_error++;
            Log::logf(CAT_ARB, LOG_DEBUG,
                      "TX blocked before write by state=%s src=%d t=%lu\n",
                      system_state_name(sys_state), t->source, millis());
            finish_ticket(t);
            t = nullptr;
            continue;
        }
        if (__atomic_load_n(&t->cancelled, __ATOMIC_ACQUIRE)) {
            current_ticket = nullptr;
            finish_ticket(t);
            t = nullptr;
            continue;
        }
        if (t->source == CMD_SRC_OXI && Arbiter::oximetry_deferred()) {
            // Do not replay an old reading after the acquisition window.
            // OxiArbiter will submit the current reading when its feed is due.
            current_ticket = nullptr;
            Log::logf(CAT_ARB, LOG_DEBUG, "Queued OXH discarded during acquisition t=%lu\n",
                      millis());
            finish_ticket(t);
            t = nullptr;
            continue;
        }
        if (t->lcd_step == LcdStep::Hide || t->lcd_step == LcdStep::Clear ||
            t->lcd_step == LcdStep::Expire) lcd_clear_at = 0;
        if (t->policy.accepted_types) Arbiter::clear_rx_frames();

        const int64_t sent_us = esp_timer_get_time();
        if (t->send_window.before_us && sent_us >= t->send_window.before_us) {
            t->result.send_window_missed = true;
            current_ticket = nullptr;
            finish_ticket(t);
            t = nullptr;
            continue;
        }
        if (t->send_window.before_us)
            t->result.send_lateness_us = int32_t(sent_us - t->send_window.not_before_us);
        // LCD keeps one response budget across its three dependent commands.
        const uint32_t started_ms = millis();
        if (__atomic_load_n(&t->sent_after_ms, __ATOMIC_ACQUIRE) == UINT32_MAX)
            __atomic_store_n(&t->sent_after_ms, started_ms - t->queued_ms, __ATOMIC_RELEASE);
        uart->write(t->frame, t->frame_len);
        uart->flush();
        stat_tx++;
        if (Log::get_cat_level(CAT_ARB) >= LOG_DEBUG) {
            // payload starts at offset=5
            char snip[33] = {};
            int plen = t->frame_len > 5 ? t->frame_len - 9 : 0;  // minus header(5)+crc(4)
            if (plen > 0) memcpy(snip, t->frame + 5, min(plen, 32));
            Log::logf(CAT_ARB, LOG_DEBUG,
                      "TX %s src=%d prio=%d t=%lu ticket=%lu queue=%lums\n",
                      snip, t->source, t->priority, millis(),
                      (unsigned long)t->ticket_id, (unsigned long)t->sent_after_ms);
        }

        if (!t->policy.accepted_types) {
            // One 40 ms cycle still loses following Q frames on hardware.
            // Allow two; the extra tick prevents rounding the wait below 80 ms.
            vTaskDelay(pdMS_TO_TICKS(SILENT_FRAME_GUARD_MS) + 1);
            t->result.success = true;
        } else {
            bool accepted_any = false;
            bool saw_success = false;
            uint32_t last_accepted_ms = started_ms;

            while (true) {
                bool idle_wait = false;
                uint32_t wait_ms = transaction_wait_ms(t, started_ms,
                                                       accepted_any,
                                                       last_accepted_ms,
                                                       &idle_wait);
                if (!wait_ms) {
                    if (idle_wait && accepted_any) {
                        t->result.success = saw_success;
                    } else {
                        t->result.timed_out = true;
                        stat_timeout++;
                    }
                    break;
                }

                qframe_t rx;
                uint32_t wait_slice_ms = min(wait_ms, (uint32_t)50);
                if (!Arbiter::wait_frame(&rx, wait_slice_ms)) {
                    // Keep draining a sent command after caller cancellation.
                    // Its late response must not complete the next ticket.
                    if (wait_slice_ms < wait_ms) continue;
                    if (idle_wait && accepted_any) {
                        t->result.success = saw_success;
                    } else {
                        t->result.timed_out = true;
                        stat_timeout++;
                    }
                    break;
                }

                qframe_type_mask_t mask = qframe_type_mask(rx.type);
                qframe_type_mask_t accepted = t->policy.accepted_types |
                                               QFRAME_MASK_E;
                if (!mask || !(accepted & mask) ||
                    !qframe_response_matches(t->frame, t->frame_len, rx)) {
                    Log::logf(CAT_ARB, LOG_DEBUG,
                              "Ignored unexpected RX type=%c (%s) ticket=%lu\n",
                              (char)rx.type, qframe_type_name(rx.type),
                              (unsigned long)t->ticket_id);
                    continue;
                }

                accepted_any = true;
                last_accepted_ms = millis();
                t->result.frame_count++;

                if (t->sink && !__atomic_load_n(&t->cancelled, __ATOMIC_ACQUIRE) &&
                    !t->sink(&rx, t->sink_context)) {
                    t->result.sink_failed = true;
                    stat_error++;
                    break;
                }

                if (mask & t->policy.success_types) saw_success = true;
                if (rx.type == QFRAME_TYPE_E) {
                    t->result.protocol_error = true;
                    stat_error++;
                }

                qframe_type_mask_t terminal = t->policy.terminal_types |
                                              QFRAME_MASK_E;
                if (terminal & mask) {
                    t->result.terminal_type = rx.type;
                    t->result.success = !t->result.protocol_error &&
                                        (t->policy.success_types & mask);
                    break;
                }
            }
        }

        if (t->result.timed_out || t->result.sink_failed) {
            // Drain trailing frames without holding up the caller. Live frames
            // bypass this queue and cannot keep recovery waiting indefinitely.
            uint32_t quiet = millis();
            uint32_t started = quiet;
            qframe_t discarded;
            __atomic_store_n(&t->completed, true, __ATOMIC_RELEASE);
            if (t->done) xSemaphoreGive(t->done);
            while ((uint32_t)(millis() - quiet) < 200 &&
                   (uint32_t)(millis() - started) < 1000) {
                if (Arbiter::wait_frame(&discarded, 20)) quiet = millis();
            }
        }
        current_ticket = nullptr;
        if (lcd_advance(t)) continue;
        finish_ticket(t);
        t = nullptr;
    }
}


void Arbiter::prepare_tx(int tx_pin) {
    // Preload idle before enabling output; digitalWrite requires pinMode first.
    gpio_set_level(static_cast<gpio_num_t>(tx_pin), 1);
    pinMode(tx_pin, OUTPUT);
}

void Arbiter::init(HardwareSerial &serial, int rx_pin, int tx_pin) {
    uart = &serial;
    uart->setRxBufferSize(RX_BUF_SIZE);
    uart->begin(RESMED_DEFAULT_BAUD, SERIAL_8N1, rx_pin, tx_pin);
    current_baud = RESMED_DEFAULT_BAUD;

    rx_ready = xSemaphoreCreateBinary();
    pq_init();

    xTaskCreatePinnedToCore(rx_task, "uart_rx", RX_TASK_STACK, nullptr,
                            RX_TASK_PRIO, &rx_task_handle, 1);
    xTaskCreatePinnedToCore(arbiter_task, "arbiter", ARBITER_TASK_STACK, nullptr,
                            ARBITER_TASK_PRIO, &arbiter_task_handle, 1);
}

static uart_response_policy_t normalize_policy(uart_response_policy_t policy) {
    if (!policy.queue_timeout_ms)
        policy.queue_timeout_ms = policy.first_timeout_ms ? policy.first_timeout_ms
            : Config::get().uart_cmd_timeout_ms;
    if (!policy.accepted_types) {
        if (!policy.response_timeout_ms)
            policy.response_timeout_ms = SILENT_FRAME_GUARD_MS + portTICK_PERIOD_MS;
        return policy;
    }

    if (!policy.first_timeout_ms) {
        policy.first_timeout_ms = Config::get().uart_cmd_timeout_ms;
    }
    if (policy.complete_on_idle && !policy.interframe_timeout_ms) {
        policy.interframe_timeout_ms = 350;
    }
    if (!policy.response_timeout_ms) {
        policy.response_timeout_ms = policy.first_timeout_ms;
        if (policy.complete_on_idle) {
            policy.response_timeout_ms += max((uint32_t)1000,
                                             (uint32_t)policy.interframe_timeout_ms * 4);
        }
    }
    return policy;
}

static uart_transaction_t *allocate_transaction(uint16_t frame_len,
                                             cmd_source_t src,
                                             cmd_priority_t prio,
                                             const uart_response_policy_t &policy,
                                             uart_frame_sink_t sink,
                                             bool auto_release,
                                             size_t context_size = 0,
                                             LcdStep lcd_step = LcdStep::None) {
    if (!frame_len || frame_len > QFRAME_MAX_RAW ||
        !uart_source_allowed(src) || context_size > 4096) {
        return nullptr;
    }

    size_t frame_capacity = lcd_step == LcdStep::None ? frame_len : LCD_FRAME_SIZE;
    uart_transaction_t *t =
        (uart_transaction_t *)calloc(1, sizeof(uart_transaction_t) + context_size + frame_capacity);
    if (!t) return nullptr;

    t->source = src;
    t->priority = prio;
    t->policy = normalize_policy(policy);
    t->sink = sink;
    t->references = auto_release ? 1 : 2;
    t->queued_ms = millis();
    t->sent_after_ms = UINT32_MAX;
    t->context_size = context_size;
    t->lcd_step = lcd_step;
    if (context_size) {
        t->sink_context = t + 1;
    }
    t->ticket_id = __atomic_fetch_add(&next_ticket_id, 1, __ATOMIC_RELAXED);
    t->frame = reinterpret_cast<uint8_t *>(t + 1) + context_size;
    t->frame_len = frame_len;

    if (!auto_release) {
        t->done = xSemaphoreCreateBinary();
        if (!t->done) {
            free(t);
            return nullptr;
        }
    }

    return t;
}

static uart_transaction_t *publish_transaction(uart_transaction_t *t) {
    if (!t) return nullptr;
    if (!pq_push(t, t->lcd_step == LcdStep::None ? portMAX_DELAY : 0)) {
        if (t->done) vSemaphoreDelete(t->done);
        free(t);
        return nullptr;
    }
    return t;
}

static uart_transaction_t *queue_transaction(const uint8_t *frame,
                                             uint16_t frame_len,
                                             cmd_source_t src,
                                             cmd_priority_t prio,
                                             const uart_response_policy_t &policy,
                                             uart_frame_sink_t sink,
                                             const void *sink_context,
                                             bool auto_release,
                                             size_t context_size = 0,
                                             LcdStep lcd_step = LcdStep::None) {
    if (!frame || (context_size && !sink_context) || (sink_context && !context_size))
        return nullptr;
    auto *t = allocate_transaction(frame_len, src, prio, policy, sink,
                                    auto_release, context_size, lcd_step);
    if (!t) return nullptr;
    memcpy(t->frame, frame, frame_len);
    if (context_size) memcpy(t->sink_context, sink_context, context_size);
    return publish_transaction(t);
}

uart_transaction_t *Arbiter::begin_frame(
        const uint8_t *frame, uint16_t frame_len,
        cmd_source_t src, cmd_priority_t prio,
        const uart_response_policy_t &policy,
        uart_frame_sink_t sink, const void *sink_context, size_t context_size) {
    return queue_transaction(frame, frame_len, src, prio, policy,
                             sink, sink_context, false, context_size);
}

static uart_transaction_t *create_command(
        const char *cmd, cmd_source_t src, cmd_priority_t prio,
        const uart_response_policy_t &policy,
        uart_frame_sink_t sink, const void *sink_context, size_t context_size) {
    if (!cmd || (sink_context && !context_size)) return nullptr;
    size_t payload_len = strnlen(cmd, QFRAME_MAX_PAYLOAD + 1);
    if (payload_len > QFRAME_MAX_PAYLOAD) return nullptr;
    int frame_len = qframe_encoded_size(reinterpret_cast<const uint8_t *>(cmd), payload_len);
    if (frame_len < 0) return nullptr;

    const char *reboot_cmd = transparent_reboot_command(
        reinterpret_cast<const uint8_t *>(cmd), strlen(cmd));
    if (reboot_cmd) CustomSettings::invalidate(reboot_cmd);

    auto *t = allocate_transaction(frame_len, src, prio, policy, sink, false, context_size);
    if (!t) return nullptr;
    qframe_build(QFRAME_TYPE_Q, reinterpret_cast<const uint8_t *>(cmd), payload_len,
                 t->frame, frame_len);
    if (sink_context) memcpy(t->sink_context, sink_context, context_size);
    return t;
}

uart_transaction_t *Arbiter::begin_cmd(
        const char *cmd, cmd_source_t src, cmd_priority_t prio,
        const uart_response_policy_t &policy,
        uart_frame_sink_t sink, const void *sink_context, size_t context_size) {
    if (context_size && !sink_context) return nullptr;
    return publish_transaction(create_command(cmd, src, prio, policy,
                                               sink, sink_context, context_size));
}

bool Arbiter::transaction_done(const uart_transaction_t *transaction) {
    return transaction &&
           __atomic_load_n(&transaction->completed, __ATOMIC_ACQUIRE);
}

bool Arbiter::transaction_expired(const uart_transaction_t *transaction) {
    return transaction && !transaction_remaining_ms(transaction);
}

static bool await_transaction(uart_transaction_t *transaction, uint32_t wait_ms) {
    if (!transaction) return false;

    const uint32_t started = millis();
    while (!__atomic_load_n(&transaction->completed, __ATOMIC_ACQUIRE)) {
        const uint32_t elapsed = millis() - started;
        const uint32_t caller_left = elapsed < wait_ms ? wait_ms - elapsed : 0;
        const uint32_t remaining = min(caller_left, transaction_remaining_ms(transaction));
        if (!remaining) return false;
        // A queue-phase wait may expire after TX began; recheck the new phase.
        const TickType_t ticks = pdMS_TO_TICKS(remaining) +
            (remaining % portTICK_PERIOD_MS != 0);
        xSemaphoreTake(transaction->done, ticks);
    }
    return true;
}

bool Arbiter::finish_transaction(uart_transaction_t *transaction,
                                 uart_transaction_result_t *result,
                                 uint32_t wait_ms, void *context_out,
                                 size_t context_size) {
    if (!await_transaction(transaction, wait_ms)) return false;
    if (result) *result = transaction->result;
    if (context_out && context_size == transaction->context_size)
        memcpy(context_out, transaction->sink_context, context_size);
    release_ticket(transaction);
    return true;
}

void Arbiter::cancel_transaction(uart_transaction_t *transaction) {
    if (!transaction) return;

    __atomic_store_n(&transaction->cancelled, true, __ATOMIC_RELEASE);
    if (pq_remove(transaction)) release_ticket(transaction);
    release_ticket(transaction);
}

bool Arbiter::transact_cmd(const char *cmd,
                           cmd_source_t src,
                           cmd_priority_t prio,
                           const uart_response_policy_t &policy,
                           uart_frame_sink_t sink,
                           void *sink_context,
                           uart_transaction_result_t *result, size_t context_size) {
    uart_response_policy_t normalized = normalize_policy(policy);
    uart_transaction_t *transaction = begin_cmd(cmd, src, prio, normalized,
                                                sink, sink_context, context_size);
    if (!transaction) return false;

    if (!finish_transaction(transaction, result, UINT32_MAX, sink_context, context_size)) {
        cancel_transaction(transaction);
        if (result) { *result = {}; result->timed_out = true; }
        return false;
    }
    return true;
}

typedef struct {
    uint32_t received_ms;
    uint16_t capacity;
    uint16_t length;
    bool received;
    bool bdd;
    bool missing;
    uint8_t clock_probe;
    // The retained payload and its terminator follow this header.
} single_response_capture_t;

static bool capture_response(const qframe_t *frame, void *context, uint16_t offset) {
    single_response_capture_t *capture =
        static_cast<single_response_capture_t *>(context);
    capture->received_ms = millis();
    uint16_t length = frame->payload_len - offset;
    uint16_t len = min(length, capture->capacity);
    auto *payload = reinterpret_cast<uint8_t *>(capture + 1);
    memcpy(payload, frame->payload + offset, len);
    payload[len] = '\0';
    capture->length = length;
    capture->received = true;
    return true;
}

static bool capture_single_response(const qframe_t *frame, void *context) {
    return capture_response(frame, context, 0);
}

static bool capture_variable(const qframe_t *frame, void *context) {
    // A rejected value is not a sink failure: the matched R/E is terminal.
    if (frame->type == QFRAME_TYPE_E) {
        auto *capture = static_cast<single_response_capture_t *>(context);
        capture->missing =
            (frame->payload_len == 4 && memcmp(frame->payload, "6009", 4) == 0) ||
            (frame->payload_len == 15 && memcmp(frame->payload + 8, " = 6009", 7) == 0);
        return true;
    }
    if (frame->type != QFRAME_TYPE_R || frame->payload_len <= 11 ||
        memcmp(frame->payload + 8, " = ", 3) != 0) return true;
    return capture_response(frame, context, 11);
}

static uart_transaction_t *begin_read_command(const char *cmd, cmd_source_t src,
                         cmd_priority_t prio, uint16_t output_capacity,
                         uint16_t timeout_ms, uart_frame_sink_t sink,
                         const uart_send_window_t &window = {},
                         uint32_t overall_timeout_ms = 0)
{
    if (timeout_ms == 0) timeout_ms = Config::get().uart_cmd_timeout_ms;
    const int64_t delay_us = max(int64_t(0), window.not_before_us - esp_timer_get_time());
    // Scheduled commands are bounded to the next second, not long-lived jobs.
    if ((window.not_before_us || window.before_us) &&
        (window.not_before_us <= 0 || window.before_us <= window.not_before_us ||
         delay_us > 1000000))
        return nullptr;
    uart_response_policy_t policy = {};
    policy.accepted_types = QFRAME_MASK_R | QFRAME_MASK_E;
    policy.terminal_types = QFRAME_MASK_R | QFRAME_MASK_E;
    policy.success_types = QFRAME_MASK_R;
    policy.first_timeout_ms = timeout_ms;
    policy.queue_timeout_ms = timeout_ms + uint32_t((delay_us + 999) / 1000);
    policy.overall_timeout_ms = overall_timeout_ms;

    bool bdd = cmd && strncmp(cmd, "P S #BDD ", 9) == 0;
    uint16_t capacity = bdd ? max(output_capacity, (uint16_t)32) : output_capacity;
    auto *t = create_command(cmd, src, prio, policy, sink,
                              nullptr, sizeof(single_response_capture_t) + capacity + 1);
    if (!t) return nullptr;
    auto *capture = static_cast<single_response_capture_t *>(t->sink_context);
    capture->capacity = capacity;
    capture->bdd = bdd;
    capture->clock_probe = strcmp(cmd, "G S #TIC") == 0 ? 1 :
        strcmp(cmd, "G S #STK") == 0 ? 2 : 0;
    t->send_window = window;
    return publish_transaction(t);
}

static void read_trace(const uart_transaction_t *t, bool completed,
                       Arbiter::VarReadTrace &trace) {
    trace = {};
    const uint32_t sent_after = __atomic_load_n(&t->sent_after_ms, __ATOMIC_ACQUIRE);
    const uint32_t elapsed = millis() - t->queued_ms;
    trace.sent = sent_after != UINT32_MAX;
    trace.queue_ms = trace.sent ? sent_after : elapsed;
    trace.wait_ms = trace.sent ? elapsed - sent_after : 0;
    trace.sent_ms = trace.sent ? t->queued_ms + sent_after : 0;
    if (completed)
        trace.received_ms = static_cast<const single_response_capture_t *>(
            t->sink_context)->received_ms;
    // Worker-owned fields may only be read after completion.
    trace.outcome = !completed ? "deadline" :
        t->result.timed_out ? "timeout" :
        t->result.protocol_error ? "device_error" :
        t->result.sink_failed ? "sink_failed" :
        t->result.success ? "ok" : "blocked";
}

bool Arbiter::finish_cmd(uart_transaction_t *t, char *resp_buf, uint16_t *resp_len,
                         uart_transaction_result_t *result, VarReadTrace *trace) {
    if (!transaction_done(t)) return false;
    if (result) *result = t->result;
    if (trace) read_trace(t, true, *trace);
    auto *capture = static_cast<single_response_capture_t *>(t->sink_context);
    if (capture->clock_probe && Log::get_cat_level(CAT_TIME) >= LOG_DEBUG) {
        const auto *payload = reinterpret_cast<const char *>(capture + 1);
        const uint32_t sent_after = __atomic_load_n(&t->sent_after_ms, __ATOMIC_ACQUIRE);
        Log::logf(CAT_TIME, LOG_DEBUG,
                  "probe %s tx=%lu rx=%lu value=%.*s\n",
                  capture->clock_probe == 1 ? "TIC" : "STK",
                  sent_after == UINT32_MAX ? 0UL : (unsigned long)(t->queued_ms + sent_after),
                  (unsigned long)capture->received_ms,
                  int(min(capture->length, capture->capacity)), payload);
    }
    uint16_t output_capacity = resp_buf ? capture->capacity : 0;
    if (resp_buf && resp_len)
        output_capacity = *resp_len ? min(output_capacity, uint16_t(*resp_len - 1)) : 0;
    bool ok = t->result.success;
    if (!capture->received) {
        if (resp_len) *resp_len = 0;
        release_ticket(t);
        return false;
    }
    auto *payload = reinterpret_cast<uint8_t *>(capture + 1);

    // BDD baud switching (arbiter mode)
    if (ok && capture->bdd) {
        uint32_t new_baud = parse_bdd_baud(payload, min(capture->length, capture->capacity));
        if (new_baud && new_baud != current_baud) {
            uart->updateBaudRate(new_baud);
            Log::logf(CAT_ARB, LOG_INFO, "BDD arbiter: baud %u -> %u\n",
                      current_baud, new_baud);
            current_baud = new_baud;
        }
    }

    if (resp_buf && (!resp_len || *resp_len)) {
        uint16_t copy_len = min(min(capture->length, capture->capacity), output_capacity);
        memcpy(resp_buf, payload, copy_len);
        resp_buf[copy_len] = '\0';
    }
    if (resp_len) *resp_len = capture->length;
    release_ticket(t);
    return ok;
}

uart_transaction_t *Arbiter::begin_cmd(const char *cmd, cmd_source_t src,
                                     cmd_priority_t prio, uint16_t capacity,
                                     uint16_t timeout_ms, const uart_send_window_t &window,
                                     uint32_t hard_timeout_ms) {
    return begin_read_command(cmd, src, prio,
        capacity ? min(uint16_t(capacity - 1), uint16_t(QFRAME_MAX_PAYLOAD)) : 0,
        timeout_ms, capture_single_response, window, hard_timeout_ms);
}

static bool read_command(const char *cmd, cmd_source_t src, cmd_priority_t prio,
                       char *resp_buf, uint16_t *resp_len, uint16_t timeout_ms,
                         uart_frame_sink_t sink, bool *missing = nullptr,
                         Arbiter::VarReadTrace *trace = nullptr,
                         uint32_t overall_timeout_ms = 0) {
    if (trace) *trace = {};
    if (missing) *missing = false;
    uint16_t capacity = resp_buf ? QFRAME_MAX_PAYLOAD : 0;
    if (resp_buf && resp_len) capacity = *resp_len ? min(capacity, uint16_t(*resp_len - 1)) : 0;
    auto *t = begin_read_command(cmd, src, prio, capacity, timeout_ms, sink,
                                 {}, overall_timeout_ms);
    if (!t) { if (resp_len) *resp_len = 0; return false; }
    const bool completed = await_transaction(t, UINT32_MAX);
    if (trace) read_trace(t, completed, *trace);
    if (!completed) {
        Arbiter::cancel_transaction(t);
        if (resp_len) *resp_len = 0;
        return false;
    }
    if (missing)
        *missing = static_cast<single_response_capture_t *>(t->sink_context)->missing;
    bool ok = Arbiter::finish_cmd(t, resp_buf, resp_len);
    if (trace && !ok && strcmp(trace->outcome, "ok") == 0)
        trace->outcome = "no_value";
    return ok;
}

bool Arbiter::send_cmd(const char *cmd, cmd_source_t src, cmd_priority_t prio,
                       char *resp_buf, uint16_t *resp_len, uint16_t timeout_ms,
                       uint32_t hard_timeout_ms) {
    return read_command(cmd, src, prio, resp_buf, resp_len, timeout_ms,
                         capture_single_response, nullptr, nullptr, hard_timeout_ms);
}

uart_transaction_t *Arbiter::begin_var(const char *name, cmd_source_t src,
                                     cmd_priority_t prio, uint16_t capacity,
                                     uint16_t timeout_ms, const uart_send_window_t &window,
                                     uint16_t hard_timeout_ms) {
    if (!name || strlen(name) != 3 || capacity < 2) return nullptr;
    char command[9];
    snprintf(command, sizeof(command), "G S #%s", name);
    if (!timeout_ms) timeout_ms = Config::get().uart_cmd_timeout_ms;
    timeout_ms = max(timeout_ms, SCALAR_TIMEOUT_MIN_MS);
    return begin_read_command(command, src, prio,
        min(uint16_t(capacity - 1), uint16_t(QFRAME_MAX_PAYLOAD)), timeout_ms,
        capture_variable, window, hard_timeout_ms);
}

Arbiter::VarResult Arbiter::finish_var(uart_transaction_t *t, char *out,
                                      uint16_t capacity, VarReadTrace *trace) {
    if (out && capacity) out[0] = 0;
    if (!transaction_done(t)) return VarResult::Failed;
    const auto *capture = static_cast<single_response_capture_t *>(t->sink_context);
    const bool missing = capture->missing;
    const bool truncated = capture->length > capture->capacity;
    uint16_t length = capacity;
    const bool ok = finish_cmd(t, out, &length, nullptr, trace);
    if (missing) {
        if (trace) trace->outcome = "missing";
        return VarResult::Missing;
    }
    if (!ok || truncated || !out || capacity < 2 || length >= capacity) {
        if (trace && (ok || strcmp(trace->outcome, "ok") == 0))
            trace->outcome = ok ? "truncated" : "no_value";
        if (out && capacity) out[0] = 0;
        return VarResult::Failed;
    }
    return VarResult::Ok;
}

Arbiter::VarResult Arbiter::read_var(const char *name, cmd_source_t src,
                                    cmd_priority_t prio, char *out,
                                    uint16_t capacity, uint16_t timeout_ms,
                                    VarReadTrace *trace, uint16_t hard_timeout_ms) {
    if (trace) { *trace = {}; trace->outcome = "invalid_request"; }
    if (out && capacity) out[0] = 0;
    if (!name || strlen(name) != 3 || !out || capacity < 2) return VarResult::Failed;
    if (trace) *trace = {};
    auto *t = begin_var(name, src, prio, capacity, timeout_ms, {}, hard_timeout_ms);
    if (!t) return VarResult::Failed;
    if (!await_transaction(t, UINT32_MAX)) {
        if (trace) read_trace(t, false, *trace);
        cancel_transaction(t);
        return VarResult::Failed;
    }
    return finish_var(t, out, capacity, trace);
}

Arbiter::VarResult Arbiter::read_var_hex(const char *name, cmd_source_t src,
                                        cmd_priority_t prio, uint32_t &out,
                                        uint16_t timeout_ms, VarReadTrace *trace,
                                        uint16_t hard_timeout_ms) {
    char text[9];
    VarResult result = read_var(name, src, prio, text, sizeof(text), timeout_ms,
                                trace, hard_timeout_ms);
    if (result != VarResult::Ok) return result;
    if (aircannect::parse_hex(text, strlen(text), out)) return VarResult::Ok;
    if (trace) trace->outcome = "invalid_hex";
    return VarResult::Failed;
}

bool Arbiter::get_var(const char *name, cmd_source_t src, cmd_priority_t prio,
                      char *out, uint16_t capacity, uint16_t timeout_ms) {
    return read_var(name, src, prio, out, capacity, timeout_ms) == VarResult::Ok;
}

bool Arbiter::oximetry_deferred() {
    return acquisition_remaining_ms() &&
        int32_t(millis() - acquisition_oxh_from.load(std::memory_order_acquire)) >= 0;
}

uint32_t Arbiter::AcquisitionWindow::lead_ms() {
    const uint32_t baud = current_baud ? current_baud : 57600;
    constexpr uint32_t oxh_max_wire_bytes = 9 + 2 * 16;
    return SILENT_FRAME_GUARD_MS + 2 * portTICK_PERIOD_MS +
        (oxh_max_wire_bytes * 10 * 1000 + baud - 1) / baud;
}

Arbiter::AcquisitionWindow::AcquisitionWindow(uint32_t until_ms, uint32_t oxh_from_ms) {
    const uint32_t now = millis();
    const int32_t remaining = int32_t(until_ms - now);
    if (remaining <= 0) return;
    uint32_t until = now + min(uint32_t(remaining), uint32_t(1000));
    // Zero is the inactive sentinel; shorten a wrapping window by one ms.
    if (!until) until = UINT32_MAX;
    uint32_t previous = acquisition_until.load(std::memory_order_acquire);
    if (previous && int32_t(previous - now) > 0) return;
    // Only the recorder creates windows; publish the OXH start before the gate.
    acquisition_oxh_from.store(oxh_from_ms ? oxh_from_ms : now, std::memory_order_relaxed);
    if (acquisition_until.compare_exchange_strong(previous, until,
                                                   std::memory_order_acq_rel))
        until_ms_ = until;
}

Arbiter::AcquisitionWindow::~AcquisitionWindow() {
    if (until_ms_)
        acquisition_until.compare_exchange_strong(until_ms_, 0,
                                                    std::memory_order_acq_rel);
    if (pq.available) xSemaphoreGive(pq.available);
}

bool Arbiter::send_frame(const uint8_t *frame, uint16_t frame_len,
                         cmd_source_t src, cmd_priority_t prio)
{
    if (!uart_source_allowed(src)) return false;
    if (src == CMD_SRC_OXI && oximetry_deferred()) return false;
    if (frame_len > QFRAME_MAX_RAW) return false;

    uart_response_policy_t no_response = {};
    return queue_transaction(frame, frame_len, src, prio, no_response,
                             nullptr, nullptr, true) != nullptr;
}

uart_frame_listener_t Arbiter::add_frame_listener(qframe_type_mask_t types,
                                                   uart_frame_sink_t sink,
                                                   void *context) {
    if (!types || !sink) return -1;

    uart_frame_listener_t handle = -1;
    portENTER_CRITICAL(&frame_listener_mux);
    for (uint8_t i = 0; i < FRAME_LISTENER_MAX; i++) {
        if (frame_listeners[i].active || frame_listeners[i].inflight) continue;
        frame_listeners[i].types = types;
        frame_listeners[i].sink = sink;
        frame_listeners[i].context = context;
        frame_listeners[i].active = true;
        handle = (uart_frame_listener_t)i;
        break;
    }
    portEXIT_CRITICAL(&frame_listener_mux);
    return handle;
}

void Arbiter::remove_frame_listener(uart_frame_listener_t listener) {
    if (listener < 0 || listener >= FRAME_LISTENER_MAX) return;

    uint8_t slot = (uint8_t)listener;
    portENTER_CRITICAL(&frame_listener_mux);
    frame_listeners[slot].active = false;
    portEXIT_CRITICAL(&frame_listener_mux);

    while (true) {
        portENTER_CRITICAL(&frame_listener_mux);
        bool idle = frame_listeners[slot].inflight == 0;
        if (idle) {
            memset(&frame_listeners[slot], 0,
                   sizeof(frame_listeners[slot]));
        }
        portEXIT_CRITICAL(&frame_listener_mux);
        if (idle) break;
        vTaskDelay(1);
    }
}

system_state_t Arbiter::get_state()         { return sys_state; }
void Arbiter::set_state(system_state_t s)   { sys_state = s; }

bool Arbiter::wait_idle(uint16_t timeout_ms) {
    uint32_t start = millis();
    while (current_ticket != nullptr) {
        if ((uint32_t)(millis() - start) >= timeout_ms) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return true;
}

void Arbiter::enter_transparent(Stream *bridge) {
    transparent_tx_reset();
    transparent_pending_baud = 0;
    transparent_pending_baud_at = 0;
    transparent_reboot_baud_at = 0;
    sys_state = SYS_TRANSPARENT;
    set_baud(RESMED_DEFAULT_BAUD);
    transparent_last_activity = millis();
    portENTER_CRITICAL(&transparent_mux);
    transparent_bridge = bridge;
    transparent_active = true;
    rx_reset_requested.store(true);
    portEXIT_CRITICAL(&transparent_mux);
}

uint32_t Arbiter::transparent_activity() {
    return transparent_last_activity;
}

void Arbiter::exit_transparent() {
    while (true) {
        portENTER_CRITICAL(&transparent_mux);
        transparent_bridge = nullptr;
        bool busy = transparent_inflight != 0;
        portEXIT_CRITICAL(&transparent_mux);
        if (!busy) break;
        vTaskDelay(1);
    }
    transparent_tx_reset();
    transparent_pending_baud = 0;
    transparent_pending_baud_at = 0;
    transparent_check_reboot_baud(true);
    sys_state = SYS_IDLE;
    portENTER_CRITICAL(&transparent_mux);
    transparent_active = false;
    rx_reset_requested.store(true);
    portEXIT_CRITICAL(&transparent_mux);
}

void Arbiter::write_raw(const uint8_t *data, size_t len) {
    if (uart && (transparent_active || sys_state == SYS_OTA_AIRSENSE)) {
        if (transparent_active) transparent_sniff_tx(data, len);
        uart->write(data, len);
        uart->flush();
        if (transparent_active) {
            transparent_last_activity = millis();
            Log::logf(CAT_ARB, LOG_DEBUG, "TRANSP TX %u bytes t=%lu\n",
                      unsigned(len), millis());
        }
    }
}

void Arbiter::clear_rx_frames() {
    uint8_t cleared;
    portENTER_CRITICAL(&rx_frame_mux);
    cleared = rx_queue_clear_locked();
    portEXIT_CRITICAL(&rx_frame_mux);
    if (rx_ready) {
        xSemaphoreTake(rx_ready, 0);
    }
    if (cleared > 0) {
        Log::logf(CAT_ARB, LOG_DEBUG, "RX queue cleared (%u frame%s) t=%lu\n",
                  cleared, cleared == 1 ? "" : "s", millis());
    }
}

bool Arbiter::wait_frame(qframe_t *out, uint16_t timeout_ms) {
    if (rx_queue_pop(out)) {
        return true;
    }
    if (timeout_ms == 0 || !rx_ready) {
        return false;
    }

    uint32_t start = millis();
    while (true) {
        uint32_t elapsed = (uint32_t)(millis() - start);
        if (elapsed >= timeout_ms) {
            break;
        }
        uint32_t remaining = timeout_ms - elapsed;
        if (xSemaphoreTake(rx_ready, pdMS_TO_TICKS(remaining)) != pdTRUE) {
            break;
        }
        if (rx_queue_pop(out)) {
            return true;
        }
        // A stale coalesced wake can remain after immediate queue drains.
        // Consume it and keep waiting for the real timeout window.
    }
    // Covers a frame pushed exactly as the semaphore wait timed out.
    return rx_queue_pop(out);
}

void Arbiter::set_baud(uint32_t baud) {
    if (uart && baud != current_baud) {
        uart->flush();
        uart->updateBaudRate(baud);
        // Flush RX hardware buffer (contains garbage from old baud)
        while (uart->available()) uart->read();
        rx_reset_requested.store(true);
        Arbiter::clear_rx_frames();
        Log::logf(CAT_ARB, LOG_INFO, "set_baud: %u -> %u\n", current_baud, baud);
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

// Parse BDD response payload, return new baud rate or 0 if not BDD.
static uint32_t parse_bdd_baud(const uint8_t *payload, uint16_t len) {
    if (len < 10 || memcmp(payload, "P S #BDD", 8) != 0) return 0;

    const char *text = (const char *)payload;
    const char *val = qframe_response_value(text);
    if (!val) {
        if (len < 13 || payload[8] != ' ') return 0;
        val = text + 9;
    }

    char *end = nullptr;
    unsigned long key = strtoul(val, &end, 16);
    if (end == val) return 0;
    return Arbiter::bdd_key_to_baud((uint16_t)key);
}

uint32_t Arbiter::get_tx_count()       { return stat_tx; }
uint32_t Arbiter::get_rx_count()       { return stat_rx; }
uint32_t Arbiter::get_l_rx_count()     { return stat_l_rx; }
uint32_t Arbiter::get_timeout_count()  { return stat_timeout; }
uint32_t Arbiter::get_error_count()    { return stat_error; }

static bool queue_lcd(const lcd_request_t &request, LcdStep step) {
    if (sys_state != SYS_IDLE && sys_state != SYS_THERAPY) return false;
    if (request.idle_only && sys_state != SYS_IDLE) return false;

    uint8_t frame[LCD_FRAME_SIZE];
    int len = qframe_build_cmd("P S #LCA 0000", frame, sizeof(frame));
    if (len < 0) return false;

    uart_response_policy_t policy = {};
    policy.accepted_types = policy.terminal_types = QFRAME_MASK_R | QFRAME_MASK_E;
    policy.success_types = QFRAME_MASK_R;
    policy.first_timeout_ms = Config::get().uart_cmd_timeout_ms;
    policy.response_timeout_ms = policy.first_timeout_ms * (step == LcdStep::Hide ? 3u : 1u);
    policy.queue_timeout_ms = policy.response_timeout_ms;
    return queue_transaction(frame, len, CMD_SRC_INTERNAL, CMD_PRIO_NORMAL,
                             policy, nullptr, &request, true, sizeof(request), step) != nullptr;
}

bool Arbiter::lcd_message(const char *msg, uint32_t timeout_ms, bool idle_only) {
    if (!msg) return false;
    lcd_request_t request = {};
    snprintf(request.text, sizeof(request.text), "%s", msg);
    request.timeout_ms = timeout_ms;
    request.idle_only = idle_only;
    return queue_lcd(request, LcdStep::Hide);
}

bool Arbiter::lcd_clear() {
    return queue_lcd({}, LcdStep::Clear);
}

static bool lcd_advance(uart_transaction_t *t) {
    if (t->lcd_step == LcdStep::None) return false;
    if (!t->result.success) {
        Log::logf(CAT_ARB, LOG_WARN, "LCD command failed at step %u\n",
                  static_cast<unsigned>(t->lcd_step));
        return false;
    }

    auto *request = static_cast<lcd_request_t *>(t->sink_context);
    char cmd[LCD_COMMAND_SIZE];
    if (t->lcd_step == LcdStep::Hide) {
        snprintf(cmd, sizeof(cmd), "P S #LCT %s", request->text);
        t->lcd_step = LcdStep::Text;
    } else if (t->lcd_step == LcdStep::Text) {
        strcpy(cmd, "P S #LCA 0001");
        t->lcd_step = LcdStep::Show;
    } else {
        if (t->lcd_step == LcdStep::Show && request->timeout_ms) {
            lcd_clear_at = millis() + request->timeout_ms;
            if (!lcd_clear_at) lcd_clear_at = 1;
        }
        return false;
    }

    int len = qframe_build_cmd(cmd, t->frame, LCD_FRAME_SIZE);
    if (len < 0) return false;
    t->frame_len = len;
    t->result = {};
    return true;
}

static void lcd_check() {
    if (!lcd_clear_at || lcd_clear_pending ||
        static_cast<int32_t>(millis() - lcd_clear_at) < 0) return;

    lcd_request_t request = {};
    request.expires_at = lcd_clear_at;
    lcd_clear_pending = queue_lcd(request, LcdStep::Expire);
}
