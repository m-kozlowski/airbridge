#include "tcp_bridge.h"
#include "uart_arbiter.h"
#include "debug_log.h"
#include "app_config.h"
#include "build_info.h"
#include "airbridge_ota.h"
#include "live_stream.h"
#include "memory_manager.h"
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <esp_heap_caps.h>
#include <freertos/queue.h>
#include <freertos/stream_buffer.h>
#include <lwip/sockets.h>
#include <errno.h>

extern void dispatch_command(const char *line, String &response);

static WiFiServer *server = nullptr;
static WiFiClient client;
static TaskHandle_t tcp_task_handle = nullptr;

#define TCP_TASK_STACK  6144
#define TCP_TASK_PRIO   3
#define TCP_LINE_MAX    512
#define FRAMED_TX_DEPTH 8
#define FRAMED_MAX_STREAMS 12

static char line_buf[TCP_LINE_MAX];
static int line_pos = 0;

static StaticQueue_t framed_tx_queue_state;
static QueueHandle_t framed_tx_queue = nullptr;
static uint8_t *framed_tx_storage = nullptr;
static bool framed_accepting = false;
static bool framed_queue_failed = false;
static uint32_t framed_sink_inflight = 0;
static uint32_t framed_generation = 0;
static portMUX_TYPE framed_mux = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    char tag[4];
    LiveStream::external_handle_t handle;
} framed_stream_lease_t;

static framed_stream_lease_t framed_streams[FRAMED_MAX_STREAMS];
static uint8_t framed_stream_count = 0;

static bool framed_queue_init() {
    size_t bytes = FRAMED_TX_DEPTH * sizeof(qframe_t);
    framed_tx_storage = static_cast<uint8_t *>(
        aircannect::Memory::alloc_large(bytes));
    if (!framed_tx_storage) return false;

    framed_tx_queue = xQueueCreateStatic(FRAMED_TX_DEPTH, sizeof(qframe_t),
                                         framed_tx_storage,
                                         &framed_tx_queue_state);
    if (!framed_tx_queue) {
        heap_caps_free(framed_tx_storage);
        framed_tx_storage = nullptr;
        return false;
    }
    portENTER_CRITICAL(&framed_mux);
    framed_generation++;
    framed_accepting = true;
    portEXIT_CRITICAL(&framed_mux);
    __atomic_store_n(&framed_queue_failed, false, __ATOMIC_RELEASE);
    return true;
}

static bool framed_enqueue(const qframe_t *frame, void *context) {
    portENTER_CRITICAL(&framed_mux);
    bool accepting = framed_accepting &&
        (!context || *static_cast<const uint32_t *>(context) == framed_generation);
    QueueHandle_t queue = accepting ? framed_tx_queue : nullptr;
    if (queue) framed_sink_inflight++;
    portEXIT_CRITICAL(&framed_mux);
    if (!queue) return false;

    bool queued = xQueueSend(queue, frame, 0) == pdTRUE;
    if (!queued) {
        __atomic_store_n(&framed_queue_failed, true, __ATOMIC_RELEASE);
    }
    portENTER_CRITICAL(&framed_mux);
    framed_sink_inflight--;
    portEXIT_CRITICAL(&framed_mux);
    return queued;
}

static bool framed_enqueue_text(uint8_t type, const char *text) {
    if (!text) return false;
    qframe_t frame;
    frame.type = type;
    frame.payload_len = min(strlen(text), (size_t)QFRAME_MAX_PAYLOAD);
    frame.declared_len = 0;
    frame.crc_received = frame.crc_computed = 0;
    memcpy(frame.payload, text, frame.payload_len);
    if (frame.payload_len < QFRAME_MAX_PAYLOAD) frame.payload[frame.payload_len] = '\0';
    frame.crc_valid = true;
    return framed_enqueue(&frame, nullptr);
}

static bool framed_flush() {
    qframe_t frame;
    uint8_t raw[QFRAME_MAX_RAW];
    while (framed_tx_queue && xQueueReceive(framed_tx_queue, &frame, 0) == pdTRUE) {
        int raw_len = qframe_build(frame.type, frame.payload, frame.payload_len,
                                   raw, sizeof(raw));
        if (raw_len < 0) return false;

        size_t sent = 0;
        while (sent < (size_t)raw_len && client.connected()) {
            size_t n = client.write(raw + sent, raw_len - sent);
            if (!n) return false;
            sent += n;
        }
        if (sent != (size_t)raw_len) return false;
    }
    return true;
}

static void framed_queue_shutdown() {
    while (true) {
        portENTER_CRITICAL(&framed_mux);
        framed_accepting = false;
        bool busy = framed_sink_inflight != 0;
        portEXIT_CRITICAL(&framed_mux);
        if (!busy) break;
        vTaskDelay(1);
    }
    if (framed_tx_queue) vQueueDelete(framed_tx_queue);
    framed_tx_queue = nullptr;
    if (framed_tx_storage) heap_caps_free(framed_tx_storage);
    framed_tx_storage = nullptr;
}

static bool payload_starts_with(const qframe_t *frame, const char *prefix) {
    size_t prefix_len = strlen(prefix);
    return frame && frame->payload_len >= prefix_len &&
           memcmp(frame->payload, prefix, prefix_len) == 0;
}

static bool framed_requires_transparent(const qframe_t *request) {
    return payload_starts_with(request, "P F ") ||
           payload_starts_with(request, "P S #BDD ") ||
           payload_starts_with(request, "P S #BLL ") ||
           payload_starts_with(request, "P S #RES ") ||
           payload_starts_with(request, "P S #PIP ");
}

static uart_response_policy_t framed_response_policy(const qframe_t *request) {
    uart_response_policy_t policy = {};
    uint16_t normal_timeout = Config::get().uart_cmd_timeout_ms;

    policy.accepted_types = QFRAME_MASK_R | QFRAME_MASK_E;
    policy.terminal_types = QFRAME_MASK_R | QFRAME_MASK_E;
    policy.success_types = QFRAME_MASK_R;
    policy.first_timeout_ms = normal_timeout;

    if (payload_starts_with(request, "G V ")) {
        policy.first_timeout_ms = max(normal_timeout, (uint16_t)1500);
    } else if (payload_starts_with(request, "G F ")) {
        policy.accepted_types = QFRAME_MASK_R | QFRAME_MASK_K | QFRAME_MASK_E;
        policy.terminal_types = QFRAME_MASK_E;
        policy.success_types = QFRAME_MASK_R | QFRAME_MASK_K;
        policy.first_timeout_ms = max(normal_timeout, (uint16_t)1500);
        policy.interframe_timeout_ms = 350;
        policy.response_timeout_ms = 15000;
        policy.complete_on_idle = true;
    }
    return policy;
}

static int framed_find_stream(const char tag[4]) {
    for (uint8_t i = 0; i < framed_stream_count; i++) {
        if (memcmp(framed_streams[i].tag, tag, 3) == 0) return i;
    }
    return -1;
}

static bool framed_parse_stream_control(const qframe_t *frame,
                                        char tag[4], bool *enabled) {
    if (!frame || frame->payload_len != 10 ||
        memcmp(frame->payload, "P S &", 5) != 0 ||
        frame->payload[8] != ' ' ||
        (frame->payload[9] != '0' && frame->payload[9] != '1')) {
        return false;
    }
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t c = frame->payload[5 + i];
        if (!((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9'))) return false;
        tag[i] = (char)c;
    }
    tag[3] = 0;
    *enabled = frame->payload[9] == '1';
    return true;
}

static bool framed_stream_control(const qframe_t *frame) {
    char tag[4];
    bool enabled = false;
    if (!framed_parse_stream_control(frame, tag, &enabled)) return false;

    int existing = framed_find_stream(tag);
    bool ok = true;
    if (enabled && existing < 0) {
        if (framed_stream_count >= FRAMED_MAX_STREAMS) {
            ok = false;
        } else {
            LiveStream::external_handle_t handle =
                LiveStream::acquire_external(tag);
            if (handle < 0) {
                ok = false;
            } else {
                memcpy(framed_streams[framed_stream_count].tag, tag, 4);
                framed_streams[framed_stream_count].handle = handle;
                framed_stream_count++;
            }
        }
    } else if (!enabled && existing >= 0) {
        LiveStream::release_external(framed_streams[existing].handle);
        for (uint8_t i = existing; i + 1 < framed_stream_count; i++) {
            framed_streams[i] = framed_streams[i + 1];
        }
        framed_stream_count--;
    }

    char response[32];
    if (ok) {
        snprintf(response, sizeof(response), "P S &%s %u = %u",
                 tag, enabled ? 1 : 0, enabled ? 1 : 0);
        framed_enqueue_text(QFRAME_TYPE_R, response);
    } else {
        snprintf(response, sizeof(response), "P S &%s %u = 6009",
                 tag, enabled ? 1 : 0);
        framed_enqueue_text(QFRAME_TYPE_E, response);
    }
    return true;
}

static void framed_release_streams() {
    while (framed_stream_count > 0) {
        framed_stream_count--;
        LiveStream::release_external(framed_streams[framed_stream_count].handle);
    }
}



static void handle_line(const char *line) {
    if (line[0] == '\0') return;

    String response;

    if (line[0] == '$') {
        // Internal command
        dispatch_command(line + 1, response);
    } else if (strcmp(line, "$TRANSPARENT") == 0 || strcmp(line, "TRANSPARENT") == 0) {
        // Should not reach here since $ is stripped above, but handle both forms
        response = "ERR: use $TRANSPARENT\n";
    } else {
        // Forward as Q-frame
        char resp_buf[QFRAME_MAX_PAYLOAD + 1] = {};
        uint16_t resp_len = sizeof(resp_buf);

        bool ok = Arbiter::send_cmd(line, CMD_SRC_TCP, CMD_PRIO_NORMAL,
                                    resp_buf, &resp_len);

        if (ok) {
            if (resp_len >= 2 && resp_buf[resp_len-1] == '#' && resp_buf[resp_len-2] == ' ') {
                resp_buf[resp_len-2] = '\0';
                resp_len -= 2;
            }
            response = String(resp_buf) + "\n";
        } else if (resp_len > 0) {
            response = "ERR:" + String(resp_buf) + "\n";
        } else {
            response = "ERR:TIMEOUT\n";
        }
    }

    if (client.connected() && response.length() > 0) {
        client.print(response);
    }
}

class TransparentOutput : public Stream {
public:
    TransparentOutput() {
        // Hold sixteen maximum-size replies even while TCP is stalled.
        storage = static_cast<uint8_t *>(aircannect::Memory::alloc_large(8192 + 1));
        buffer_size = storage ? 8192 : sizeof(fallback_storage) - 1;
        queue = xStreamBufferCreateStatic(buffer_size + 1, 1,
                                         storage ? storage : fallback_storage,
                                         &queue_state);
    }
    ~TransparentOutput() {
        vStreamBufferDelete(queue);
        aircannect::Memory::free(storage);
    }

    size_t reply_buffer() const { return buffer_size; }

    size_t write(uint8_t byte) override { return write(&byte, 1); }
    size_t write(const uint8_t *data, size_t len) override {
        if (__atomic_load_n(&failed, __ATOMIC_ACQUIRE)) return 0;
        size_t n = xStreamBufferSend(queue, data, len, 0);
        if (n != len) __atomic_store_n(&failed, true, __ATOMIC_RELEASE);
        return n;
    }

    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
    void flush() override {}

    bool pump(int fd) {
        if (__atomic_load_n(&failed, __ATOMIC_ACQUIRE)) return false;
        while (true) {
            if (pending_pos == pending_len) {
                pending_pos = 0;
                pending_len = xStreamBufferReceive(queue, pending, sizeof(pending), 0);
            }
            if (!pending_len) return true;

            // NetworkClient::write() can destroy the shared receive buffer on error.
            int n = send(fd, pending + pending_pos, pending_len - pending_pos, MSG_DONTWAIT);
            if (n > 0) {
                pending_pos += n;
            } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
                return true;
            } else {
                __atomic_store_n(&failed, true, __ATOMIC_RELEASE);
                return false;
            }
        }
    }

private:
    StaticStreamBuffer_t queue_state;
    uint8_t *storage;
    size_t buffer_size;
    uint8_t fallback_storage[513];
    StreamBufferHandle_t queue;
    bool failed = false;
    // Coalesce queued UART fragments instead of allocating a TCP segment for
    // each small RX fragment. Retain the unsent tail when the socket is full.
    uint8_t pending[1460];
    size_t pending_pos = 0;
    size_t pending_len = 0;
};

static void handle_transparent(bool hold) {
    auto &cfg = Config::get();
    system_state_t st = Arbiter::get_state();

    if (st == SYS_THERAPY && !cfg.allow_transparent_during_therapy) {
        client.println("ERR: transparent mode blocked during therapy");
        return;
    }

    if (OtaManager::busy()) {
        client.println("ERR: transparent mode blocked during OTA/reboot");
        return;
    }

    // Detect a vanished peer without expiring a connected, paused receiver.
    const int enabled = 1, idle = 30, interval = 10, probes = 3;
    if (hold &&
        (client.setSocketOption(SOL_SOCKET, SO_KEEPALIVE, &enabled, sizeof(enabled)) ||
         client.setSocketOption(IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle)) ||
         client.setSocketOption(IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval)) ||
         client.setSocketOption(IPPROTO_TCP, TCP_KEEPCNT, &probes, sizeof(probes)))) {
        client.println("ERR: unable to enable TCP keepalive");
        return;
    }

    TransparentOutput output;
    Arbiter::enter_transparent(&output);

    // Publish raw mode before rechecking a racing factory reboot reservation.
    if (OtaManager::busy()) {
        Arbiter::exit_transparent();
        client.println("ERR: transparent mode blocked during OTA/reboot");
        return;
    }

    client.printf("OK: entering transparent mode idle_timeout=%u reply_buffer=%u\r\n",
                  hold ? 0u : 5u, unsigned(output.reply_buffer()));

    static const uint32_t TRANSPARENT_IDLE_TIMEOUT = 5000;

    bool output_ok = true;
    while (Arbiter::get_state() == SYS_TRANSPARENT) {
        // Read first: even a reset socket can still have Arduino-buffered bytes.
        uint8_t buf[256];
        int n = client.read(buf, sizeof(buf));
        if (n > 0) {
            Arbiter::write_raw(buf, n);
        }
        output_ok = output.pump(client.fd());
        if (n <= 0 && (!output_ok || !client.connected())) break;

        // HOLD sessions end on TCP disconnect. A flow-controlled UART sender can
        // legitimately be silent while the receiver is blocked.
        // Other sessions time out after 5s without activity in either direction.
        // TCP->UART tracked here, UART->TCP tracked by rx_task via transparent_last_activity
        uint32_t last = Arbiter::transparent_activity();
        if (!hold && millis() - last > TRANSPARENT_IDLE_TIMEOUT) {
            break;
        }

        vTaskDelay(1);
    }

    Arbiter::exit_transparent();
    if (!output_ok) {
        client.stop();
        Log::logf(CAT_TCP, LOG_WARN, "Transparent output failed\n");
    } else if (client.connected()) {
        client.println("OK: transparent mode exited");
    }
}

static void handle_framed() {
    if (!framed_queue_init()) {
        client.println("ERR: unable to allocate framed queue");
        return;
    }

    uart_frame_listener_t listener = Arbiter::add_frame_listener(
        QFRAME_MASK_L, framed_enqueue, nullptr);
    if (listener < 0) {
        framed_queue_shutdown();
        client.println("ERR: no UART frame listener available");
        return;
    }

    framed_stream_count = 0;
    client.println("OK: entering framed arbiter mode");
    Log::logf(CAT_TCP, LOG_INFO, "Framed arbiter mode entered\n");

    qframe_parser_t parser;
    qframe_parser_init(&parser);
    uart_transaction_t *active = nullptr;

    while (client.connected()) {
        if (!framed_flush() ||
            __atomic_load_n(&framed_queue_failed, __ATOMIC_ACQUIRE)) {
            break;
        }

        if (active) {
            if (!Arbiter::transaction_done(active)) {
                if (Arbiter::transaction_expired(active)) {
                    Arbiter::cancel_transaction(active);
                    active = nullptr;
                    framed_enqueue_text(QFRAME_TYPE_E, "AIRBRIDGE TIMEOUT");
                    continue;
                }
                vTaskDelay(1);
                continue;
            }

            uart_transaction_result_t result = {};
            if (!Arbiter::finish_transaction(active, &result)) {
                framed_enqueue_text(QFRAME_TYPE_E,
                                    "AIRBRIDGE TRANSACTION FINALIZE FAILED");
            } else if (result.sink_failed) {
                __atomic_store_n(&framed_queue_failed, true,
                                 __ATOMIC_RELEASE);
            } else if (result.timed_out) {
                framed_enqueue_text(QFRAME_TYPE_E, "AIRBRIDGE TIMEOUT");
            } else if (!result.success && !result.protocol_error) {
                framed_enqueue_text(QFRAME_TYPE_E,
                                    "AIRBRIDGE TRANSACTION FAILED");
            }
            active = nullptr;
            continue;
        }

        while (client.available() && !active &&
               !__atomic_load_n(&framed_queue_failed, __ATOMIC_ACQUIRE)) {
            uint8_t byte = (uint8_t)client.read();
            if (!qframe_parser_feed(&parser, byte)) {
                if (parser.state == QFP_ERROR) {
                    framed_enqueue_text(QFRAME_TYPE_E,
                                        "AIRBRIDGE INVALID FRAME");
                    qframe_parser_reset(&parser);
                }
                continue;
            }

            const qframe_t *request = qframe_parser_frame(&parser);
            if (!request || !request->crc_valid) {
                framed_enqueue_text(QFRAME_TYPE_E, "AIRBRIDGE BAD CRC");
                qframe_parser_reset(&parser);
                continue;
            }

            if (request->type == QFRAME_TYPE_Q &&
                framed_stream_control(request)) {
                qframe_parser_reset(&parser);
                continue;
            }

            if (request->type == QFRAME_TYPE_Q &&
                framed_requires_transparent(request)) {
                framed_enqueue_text(QFRAME_TYPE_E,
                                    "AIRBRIDGE USE TRANSPARENT");
                qframe_parser_reset(&parser);
                continue;
            }

            uint8_t raw[QFRAME_MAX_RAW];
            int raw_len = qframe_build(request->type, request->payload,
                                       request->payload_len,
                                       raw, sizeof(raw));
            if (raw_len < 0) {
                framed_enqueue_text(QFRAME_TYPE_E,
                                    "AIRBRIDGE FRAME TOO LARGE");
                qframe_parser_reset(&parser);
                continue;
            }

            if (request->type == QFRAME_TYPE_Q) {
                uart_response_policy_t policy =
                    framed_response_policy(request);
                active = Arbiter::begin_frame(raw, (uint16_t)raw_len,
                                              CMD_SRC_TCP, CMD_PRIO_NORMAL,
                                              policy, framed_enqueue,
                                              &framed_generation,
                                              sizeof(framed_generation));
                if (!active) {
                    framed_enqueue_text(QFRAME_TYPE_E,
                                        "AIRBRIDGE UART BUSY");
                }
            } else if (request->type == QFRAME_TYPE_L) {
                if (!Arbiter::send_frame(raw, (uint16_t)raw_len,
                                         CMD_SRC_TCP, CMD_PRIO_NORMAL)) {
                    framed_enqueue_text(QFRAME_TYPE_E,
                                        "AIRBRIDGE UART BUSY");
                }
            } else {
                framed_enqueue_text(QFRAME_TYPE_E,
                                    "AIRBRIDGE USE TRANSPARENT");
            }
            qframe_parser_reset(&parser);
        }

        vTaskDelay(1);
    }

    portENTER_CRITICAL(&framed_mux);
    framed_accepting = false;
    portEXIT_CRITICAL(&framed_mux);
    if (active) {
        Arbiter::cancel_transaction(active);
    }
    Arbiter::remove_frame_listener(listener);
    framed_release_streams();
    framed_queue_shutdown();
    Log::logf(CAT_TCP, LOG_INFO, "Framed arbiter mode exited\n");
}


#define DEBUG_MAX_CLIENTS 3

static WiFiServer *debug_server = nullptr;
static WiFiClient debug_clients[DEBUG_MAX_CLIENTS];
static SemaphoreHandle_t debug_mutex = nullptr;

struct DebugPending {
    uint8_t data[160];
    size_t pos = 0;
    size_t len = 0;
};
static DebugPending debug_pending[DEBUG_MAX_CLIENTS];

static void flush_debug_client(int slot) {
    auto &pending = debug_pending[slot];
    if (pending.pos == pending.len) return;
    int n = send(debug_clients[slot].fd(), pending.data + pending.pos,
                 pending.len - pending.pos, MSG_DONTWAIT);
    if (n > 0) pending.pos += n;
    else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        debug_clients[slot].stop();
        pending.pos = pending.len = 0;
    }
}

// Print adapter that writes to all connected debug clients
class DebugPrint : public Print {
public:
    size_t write(uint8_t c) override { return write(&c, 1); }
    size_t write(const uint8_t *buf, size_t len) override {
        if (!debug_mutex || xSemaphoreTake(debug_mutex, 0) != pdTRUE) return 0;
        size_t written = 0;
        for (int i = 0; i < DEBUG_MAX_CLIENTS; i++) {
            if (debug_clients[i] && debug_clients[i].connected()) {
                auto &pending = debug_pending[i];
                // Keep a partial line intact; a slow client loses whole new lines.
                if (pending.pos != pending.len || len > sizeof(pending.data)) continue;
                memcpy(pending.data, buf, len);
                pending.pos = 0;
                pending.len = len;
                flush_debug_client(i);
                written = len;
            }
        }
        xSemaphoreGive(debug_mutex);
        return written;
    }
};

static DebugPrint debug_print;

void TcpBridge::init_debug_server(uint16_t port) {
    if (port == 0) return;
    debug_mutex = xSemaphoreCreateMutex();
    if (!debug_mutex) {
        Log::logf(CAT_TCP, LOG_ERROR, "[DBG] Client mutex allocation failed\n");
        return;
    }
    debug_server = new WiFiServer(port);
    debug_server->begin();
    debug_server->setNoDelay(true);
    Log::add_output(&debug_print);
    Log::logf(CAT_TCP, LOG_INFO, "[DBG] Debug server on port %d\n", port);
}

void TcpBridge::poll_debug_clients() {
    if (!debug_server || xSemaphoreTake(debug_mutex, 0) != pdTRUE) return;

    for (int i = 0; i < DEBUG_MAX_CLIENTS; i++) flush_debug_client(i);

    static uint32_t last_check = 0;
    uint32_t now = millis();
    if (uint32_t(now - last_check) < 100) {
        xSemaphoreGive(debug_mutex);
        return;
    }
    last_check = now;

    for (int i = 0; i < DEBUG_MAX_CLIENTS; i++) {
        if (!debug_clients[i].connected()) {
            debug_clients[i].stop();
            debug_pending[i].pos = debug_pending[i].len = 0;
        }
    }

    int connected_slot = -1;
    IPAddress connected_address;
    WiFiClient nc = debug_server->accept();
    if (nc) {
        int slot = -1;
        for (int i = 0; i < DEBUG_MAX_CLIENTS; i++) {
            if (!debug_clients[i] || !debug_clients[i].connected()) {
                slot = i;
                break;
            }
        }
        if (slot >= 0) {
            debug_clients[slot] = nc;
            debug_clients[slot].setNoDelay(true);
            connected_slot = slot;
            connected_address = nc.remoteIP();
        } else {
            const char message[] = "ERR: max debug clients\n";
            send(nc.fd(), message, sizeof(message) - 1, MSG_DONTWAIT);
            nc.stop();
        }
    }

    // Drain any input from debug clients (read-only port)
    for (int i = 0; i < DEBUG_MAX_CLIENTS; i++) {
        if (debug_clients[i] && debug_clients[i].connected()) {
            uint8_t ignored[64];
            debug_clients[i].read(ignored, sizeof(ignored));
        }
    }
    xSemaphoreGive(debug_mutex);
    if (connected_slot >= 0)
        Log::logf(CAT_TCP, LOG_INFO, "[DBG] Debug client %d connected from %s\n",
                  connected_slot, connected_address.toString().c_str());
}

void TcpBridge::task(void *param) {
    auto &cfg = Config::get();

    if (cfg.wifi_mode == WIFI_MODE_OFF) {
        Log::logf(CAT_TCP, LOG_INFO, "WiFi disabled, TCP bridge not starting\n");
        vTaskDelete(nullptr);
        return;
    }

    server = new WiFiServer(cfg.tcp_port);
    server->begin();
    server->setNoDelay(true);
    Log::logf(CAT_TCP, LOG_INFO, "Listening on port %d\n", cfg.tcp_port);

    while (true) {
        if (!client.connected() && !client.available()) {
            client.stop();
            WiFiClient newClient = server->accept();
            if (newClient) {
                client = newClient;
                client.setNoDelay(true);
                line_pos = 0;
                Log::logf(CAT_TCP, LOG_INFO, "Client connected from %s\n",
                            client.remoteIP().toString().c_str());
                client.printf("AirBridge %s\n", airbridge_version());
            }
        }

        if (!client.connected() && !client.available()) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }


        while (client.available()) {
            char c = client.read();

            if (c == '\n' || c == '\r') {
                if (line_pos > 0) {
                    line_buf[line_pos] = '\0';

                    if (strcasecmp(line_buf, "$TRANSPARENT") == 0 ||
                        strcasecmp(line_buf, "$TRANSPARENT HOLD") == 0) {
                        handle_transparent(strcasecmp(line_buf, "$TRANSPARENT HOLD") == 0);
                    } else if (strcasecmp(line_buf, "$FRAMED") == 0) {
                        handle_framed();
                    } else {
                        handle_line(line_buf);
                    }
                    line_pos = 0;
                }
            } else if (c == '\b' || c == '\x7f') {
                if (line_pos > 0) --line_pos;
            } else if (line_pos < TCP_LINE_MAX - 1) {
                line_buf[line_pos++] = c;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void TcpBridge::init() {
    xTaskCreatePinnedToCore(TcpBridge::task, "tcp_srv", TCP_TASK_STACK,
                            nullptr, TCP_TASK_PRIO, &tcp_task_handle, 0);
}
