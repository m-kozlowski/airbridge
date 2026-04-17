#include "tcp_bridge.h"
#include "uart_arbiter.h"
#include "debug_log.h"
#include "web_ui.h"
#include "app_config.h"
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiClient.h>

extern const char *airbridge_version();

extern void dispatch_command(const char *line, String &response);

static WiFiServer *server = nullptr;
static WiFiClient client;
static TaskHandle_t tcp_task_handle = nullptr;

#define TCP_TASK_STACK  6144
#define TCP_TASK_PRIO   3
#define TCP_LINE_MAX    512

static char line_buf[TCP_LINE_MAX];
static int line_pos = 0;



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
        } else {
            response = "ERR:TIMEOUT\n";
        }
    }

    if (client.connected() && response.length() > 0) {
        client.print(response);
    }
}

static void handle_transparent() {
    auto &cfg = Config::get();
    system_state_t st = Arbiter::get_state();

    if (st == SYS_THERAPY && !cfg.allow_transparent_during_therapy) {
        client.println("ERR: transparent mode blocked during therapy");
        return;
    }

    client.println("OK: entering transparent mode (idle timeout 5s)");
    Arbiter::enter_transparent(&client);

    static const uint32_t TRANSPARENT_IDLE_TIMEOUT = 5000;

    while (client.connected() && Arbiter::get_state() == SYS_TRANSPARENT) {
        // TCP -> UART
        if (client.available()) {
            uint8_t buf[256];
            int n = client.readBytes(buf, min(client.available(), (int)sizeof(buf)));
            if (n > 0) {
                Arbiter::write_raw(buf, n);
            }
        }

        // Idle timeout: 5s since last activity in either direction
        // TCP->UART tracked here, UART->TCP tracked by rx_task via transparent_last_activity
        uint32_t last = Arbiter::transparent_activity();
        if (millis() - last > TRANSPARENT_IDLE_TIMEOUT) {
            break;
        }

        vTaskDelay(1);
    }

    Arbiter::exit_transparent();
    if (client.connected()) {
        client.println("OK: transparent mode exited");
    }
}


#define DEBUG_MAX_CLIENTS 3

static WiFiServer *debug_server = nullptr;
static WiFiClient debug_clients[DEBUG_MAX_CLIENTS];

// Print adapter that writes to all connected debug clients
class DebugPrint : public Print {
public:
    size_t write(uint8_t c) override { return write(&c, 1); }
    size_t write(const uint8_t *buf, size_t len) override {
        size_t written = 0;
        for (int i = 0; i < DEBUG_MAX_CLIENTS; i++) {
            if (debug_clients[i] && debug_clients[i].connected()) {
                debug_clients[i].write(buf, len);
                written = len;
            }
        }
        return written;
    }
};

static DebugPrint debug_print;
static bool debug_registered = false;

void TcpBridge::init_debug_server(uint16_t port) {
    if (port == 0) return;
    debug_server = new WiFiServer(port);
    debug_server->begin();
    debug_server->setNoDelay(true);
    Log::add_output(&debug_print);
    debug_registered = true;
    Log::logf(CAT_TCP, LOG_INFO, "[DBG] Debug server on port %d\n", port);
}

void TcpBridge::poll_debug_clients() {
    if (!debug_server) return;

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
            Log::logf(CAT_TCP, LOG_INFO, "[DBG] Debug client %d connected from %s\n",
                        slot, nc.remoteIP().toString().c_str());
        } else {
            nc.println("ERR: max debug clients");
            nc.stop();
        }
    }

    // Drain any input from debug clients (read-only port)
    for (int i = 0; i < DEBUG_MAX_CLIENTS; i++) {
        if (debug_clients[i] && debug_clients[i].connected()) {
            while (debug_clients[i].available()) debug_clients[i].read();
        }
    }
}

void TcpBridge::task(void *param) {
    auto &cfg = Config::get();

    if (cfg.wifi_mode == 2) {
        Log::logf(CAT_TCP, LOG_INFO, "[TCP] WiFi disabled, TCP bridge not starting\n");
        vTaskDelete(nullptr);
        return;
    }

    server = new WiFiServer(cfg.tcp_port);
    server->begin();
    server->setNoDelay(true);
    Log::logf(CAT_TCP, LOG_INFO, "[TCP] Listening on port %d\n", cfg.tcp_port);

    while (true) {
        if (!client || !client.connected()) {
            WiFiClient newClient = server->accept();
            if (newClient) {
                client = newClient;
                client.setNoDelay(true);
                line_pos = 0;
                Log::logf(CAT_TCP, LOG_INFO, "[TCP] Client connected from %s\n",
                            client.remoteIP().toString().c_str());
                client.printf("AirBridge %s\n", airbridge_version());
            }
        }

        if (!client || !client.connected()) {
            poll_debug_clients();
            WebUI::handle();
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }


        while (client.available()) {
            char c = client.read();

            if (c == '\n' || c == '\r') {
                if (line_pos > 0) {
                    line_buf[line_pos] = '\0';

                    if (strcasecmp(line_buf, "$TRANSPARENT") == 0) {
                        handle_transparent();
                    } else {
                        handle_line(line_buf);
                    }
                    line_pos = 0;
                }
            } else if (line_pos < TCP_LINE_MAX - 1) {
                line_buf[line_pos++] = c;
            }
        }

        poll_debug_clients();
        WebUI::handle();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void TcpBridge::init() {
    xTaskCreatePinnedToCore(TcpBridge::task, "tcp_srv", TCP_TASK_STACK,
                            nullptr, TCP_TASK_PRIO, &tcp_task_handle, 0);
}


