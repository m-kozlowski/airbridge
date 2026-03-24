#include "resmed_ota.h"
#include "uart_arbiter.h"
#include "qframe.h"
#include "debug_log.h"
#include "app_config.h"
#include <esp_partition.h>

#define FLASH_TASK_STACK    8192
#define FLASH_TASK_PRIO     3

#define CHUNK_SIZE          250
#define BID_OFFSET_SX577    0x3F80  // BID string offset within BLX

struct block_info_t {
    const char *name;
    uint32_t base_addr;
    uint32_t max_size;
    uint32_t file_offset;
};

static const block_info_t BLOCKS_SX577[] = {
    {"BLX", 0x08000000, 0x04000,  0x00000},
    {"CCX", 0x08004000, 0x3C000,  0x04000},
    {"CDX", 0x08040000, 0xC0000,  0x40000},
    {"CMX", 0x08004000, 0xFC000,  0x04000},
};
#define BLOCK_COUNT 4

static volatile bool flash_active = false;
static volatile bool flash_cancel = false;
static volatile size_t flash_sent = 0;
static volatile size_t flash_total = 0;
static char flash_phase[32] = "";
static char flash_error[128] = "";
static TaskHandle_t flash_task_handle = nullptr;

struct flash_params_t {
    char block[8];
    size_t fw_size;
    bool flash_blx;
    bool force_blx;
};
static flash_params_t flash_params;

static const block_info_t* find_block(const char *name) {
    for (int i = 0; i < BLOCK_COUNT; i++) {
        if (strcmp(BLOCKS_SX577[i].name, name) == 0) return &BLOCKS_SX577[i];
    }
    return nullptr;
}

// Format: [0x03] [length] [address:4 big-endian] [data...] [0x00]
// length = 4 (addr) + data_len + 1 (trailing zero)
static int build_record_03(uint8_t *out, size_t out_size,
                           uint32_t addr, const uint8_t *data, size_t data_len) {
    size_t rec_len = 2 + 4 + data_len + 1;  // type + len + addr + data + zero
    if (out_size < rec_len) return -1;

    uint8_t payload_len = 4 + data_len + 1;
    out[0] = 0x03;
    out[1] = payload_len;
    out[2] = (addr >> 24) & 0xFF;
    out[3] = (addr >> 16) & 0xFF;
    out[4] = (addr >> 8) & 0xFF;
    out[5] = addr & 0xFF;
    memcpy(out + 6, data, data_len);
    out[6 + data_len] = 0x00;

    return (int)rec_len;
}

// Data frame: block_name + 0x00 + seq + record
// Completion: block_name + 'F' + seq
static int build_f_payload(uint8_t *out, size_t out_size,
                           const char *block_name, uint8_t seq,
                           const uint8_t *record, size_t record_len,
                           bool is_completion) {
    size_t name_len = strlen(block_name);
    size_t total = name_len + 1 + 1 + (is_completion ? 0 : record_len);
    if (out_size < total) return -1;

    memcpy(out, block_name, name_len);
    out[name_len] = is_completion ? 'F' : 0x00;
    out[name_len + 1] = seq;
    if (!is_completion && record && record_len > 0) {
        memcpy(out + name_len + 2, record, record_len);
    }

    return is_completion ? (int)(name_len + 2) : (int)(name_len + 2 + record_len);
}

// Bypasses arbiter queue
static bool send_raw_cmd(const char *cmd, char *resp, uint16_t resp_size,
                         uint16_t timeout_ms = 2000) {
    uint8_t frame[QFRAME_MAX_RAW];
    int frame_len = qframe_build_cmd(cmd, frame, sizeof(frame));
    if (frame_len < 0) return false;

    Arbiter::write_raw(frame, frame_len);

    qframe_t rx;
    if (Arbiter::wait_frame(&rx, timeout_ms)) {
        if (resp && resp_size > 0) {
            uint16_t copy = min((uint16_t)rx.payload_len, (uint16_t)(resp_size - 1));
            memcpy(resp, rx.payload, copy);
            resp[copy] = '\0';
        }
        return (rx.type == QFRAME_TYPE_R);
    }
    return false;
}


static bool send_and_check(const char *cmd, char *resp, uint16_t resp_size,
                           uint16_t timeout_ms = 3000) {
    uint16_t len = resp_size;
    return Arbiter::send_cmd(cmd, CMD_SRC_OTA, CMD_PRIO_CRITICAL,
                              resp, &len, timeout_ms);
}


static bool check_bid(const esp_partition_t *part, size_t blx_partition_offset, bool force) {
    if (force) return true;

    strncpy(flash_phase, "BID check", sizeof(flash_phase));

    char img_bid[64] = {};
    esp_partition_read(part, blx_partition_offset + BID_OFFSET_SX577,
                       img_bid, sizeof(img_bid) - 1);

    char dev_bid[128] = {};
    if (!send_and_check("G S #BID", dev_bid, sizeof(dev_bid))) {
        snprintf(flash_error, sizeof(flash_error), "Failed to read device BID");
        return false;
    }

    char *eq = strstr(dev_bid, "= ");
    const char *dev_bid_str = eq ? eq + 2 : dev_bid;

    if (strncmp(img_bid, dev_bid_str, 20) != 0) {
        snprintf(flash_error, sizeof(flash_error),
                 "BID mismatch: image=%.20s device=%.20s", img_bid, dev_bid_str);
        return false;
    }
    Log::logf(CAT_OTA, LOG_INFO, "[OTA] BID check passed\n");
    return true;
}


static bool enter_bootloader(bool send_bll = true) {
    char resp[128] = {};
    strncpy(flash_phase, "Enter bootloader", sizeof(flash_phase));
    Log::logf(CAT_OTA, LOG_INFO, "[OTA] Entering bootloader (bll=%s)...\n", send_bll ? "yes" : "no");

    if (send_bll) {
        // Check if already in bootloader
        if (send_raw_cmd("G S #BLS", resp, sizeof(resp), 300)) {
            char *eq = strstr(resp, "= ");
            if (eq && strtol(eq + 2, nullptr, 16) >= 1) {
                Log::logf(CAT_OTA, LOG_INFO, "[OTA] Already in bootloader\n");
                return true;
            }
        }
        Log::logf(CAT_OTA, LOG_INFO, "[OTA] Triggering reboot...\n");
        send_raw_cmd("P S #BLL 0001", resp, sizeof(resp), 2000);
    }


    int bls0_count = 0;
    for (int i = 0; i < 100 && !flash_cancel; i++) {
        memset(resp, 0, sizeof(resp));
        bool got = send_raw_cmd("G S #BLS", resp, sizeof(resp), 100);
        if (got) {
            char *eq = strstr(resp, "= ");
            if (eq) {
                int bls = (int)strtol(eq + 2, nullptr, 16);
                if (bls >= 1) {
                    Log::logf(CAT_OTA, LOG_INFO, "[OTA] In bootloader (BLS=%d) after %d polls\n", bls, i);
                    return true;
                }
                // BLS=0: app is running; send BLL to reboot into bootloader
                bls0_count++;
                Log::logf(CAT_OTA, LOG_DEBUG, "[OTA] BLS poll %d: app running (BLS=0)\n", i);
                if (bls0_count >= 3) {
                    Log::logf(CAT_OTA, LOG_INFO, "[OTA] Sending BLL...\n");
                    send_raw_cmd("P S #BLL 0001", resp, sizeof(resp), 2000);
                    bls0_count = 0;
                }
            }
        } else {
            // No response; device resetting? keep polling fast
            bls0_count = 0;
            if (i < 3 || i % 20 == 0) Log::logf(CAT_OTA, LOG_DEBUG, "[OTA] BLS poll %d: no response\n", i);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    snprintf(flash_error, sizeof(flash_error), "Failed to enter bootloader");
    return false;
}

static bool wait_for_erase(uint32_t timeout_ms) {
    uint32_t t0 = millis();
    int p_count = 0;

    while (millis() - t0 < timeout_ms && !flash_cancel) {
        qframe_t rx;
        if (Arbiter::wait_frame(&rx, 500)) {
            if (rx.type == QFRAME_TYPE_P) {
                p_count++;
            } else if (rx.type == QFRAME_TYPE_R) {
                Log::logf(CAT_OTA, LOG_INFO, "[OTA] Erase done (%d ACKs)\n", p_count);
                return true;
            } else if (rx.type == QFRAME_TYPE_E) {
                char err[32] = {};
                memcpy(err, rx.payload, min((int)rx.payload_len, 31));
                snprintf(flash_error, sizeof(flash_error), "Erase error: %s", err);
                return false;
            }
        }
    }
    snprintf(flash_error, sizeof(flash_error), "Erase timeout (%d ACKs received)", p_count);
    return false;
}


static bool flash_one_block(const esp_partition_t *part, size_t part_offset,
                            const block_info_t *block, size_t data_size,
                            bool send_completion = true) {
    // Trim trailing 0xFF
    size_t trimmed_size = data_size;
    {
        uint8_t tail[256];
        while (trimmed_size > 0) {
            size_t check_len = min(trimmed_size, sizeof(tail));
            size_t check_off = trimmed_size - check_len;
            esp_partition_read(part, part_offset + check_off, tail, check_len);
            bool all_ff = true;
            for (int i = check_len - 1; i >= 0; i--) {
                if (tail[i] != 0xFF) {
                    trimmed_size = check_off + i + 1;
                    all_ff = false;
                    break;
                }
            }
            if (!all_ff) break;
            trimmed_size = check_off;
        }
    }
    // Align to 4 bytes
    trimmed_size = (trimmed_size + 3) & ~3;
    if (trimmed_size == 0) {
        Log::logf(CAT_OTA, LOG_INFO, "[OTA] %s data is all 0xFF, skipping\n", block->name);
        return true;
    }

    Log::logf(CAT_OTA, LOG_INFO, "[OTA] %s: %u bytes (trimmed from %u)\n",
                 block->name, trimmed_size, data_size);

    // ERASE
    {
        char erase_cmd[24];
        snprintf(erase_cmd, sizeof(erase_cmd), "P F *%s 0000", block->name);
        snprintf(flash_phase, sizeof(flash_phase), "Erase %s", block->name);
        Log::logf(CAT_OTA, LOG_INFO, "[OTA] Erasing %s...\n", block->name);

        uint8_t frame[64];
        int frame_len = qframe_build_cmd(erase_cmd, frame, sizeof(frame));
        if (frame_len < 0) {
            snprintf(flash_error, sizeof(flash_error), "Erase frame build error");
            return false;
        }
        Arbiter::write_raw(frame, frame_len);

        if (!wait_for_erase(30000)) return false;
    }

    vTaskDelay(pdMS_TO_TICKS(300));

    // WRITE
    snprintf(flash_phase, sizeof(flash_phase), "Flash %s", block->name);
    Log::logf(CAT_OTA, LOG_INFO, "[OTA] Writing %u bytes to %s @ %u baud...\n",
                 trimmed_size, block->name, Arbiter::get_baud());

    uint8_t seq = 0;
    size_t offset = 0;
    int frame_count = 0;

    while (offset < trimmed_size && !flash_cancel) {
        size_t chunk_len = min((size_t)CHUNK_SIZE, trimmed_size - offset);

        uint8_t chunk_buf[CHUNK_SIZE];
        esp_err_t err = esp_partition_read(part, part_offset + offset,
                                            chunk_buf, chunk_len);
        if (err != ESP_OK) {
            snprintf(flash_error, sizeof(flash_error),
                     "Read error at %u: %s", offset, esp_err_to_name(err));
            return false;
        }

        // Build binary record
        uint8_t record[CHUNK_SIZE + 8];
        uint32_t addr = block->base_addr + offset;
        int rec_len = build_record_03(record, sizeof(record), addr, chunk_buf, chunk_len);
        if (rec_len < 0) {
            snprintf(flash_error, sizeof(flash_error), "Record build error at %u", offset);
            return false;
        }

        // Build F-frame payload
        uint8_t f_payload[CHUNK_SIZE + 16];
        int f_len = build_f_payload(f_payload, sizeof(f_payload),
                                     block->name, seq, record, rec_len, false);
        if (f_len < 0) {
            snprintf(flash_error, sizeof(flash_error), "F-payload build error");
            return false;
        }

        // Build and send Q-frame type 'f'
        uint8_t frame[QFRAME_MAX_RAW];
        int frame_len = qframe_build('f', f_payload, f_len, frame, sizeof(frame));
        if (frame_len < 0) {
            snprintf(flash_error, sizeof(flash_error), "F-frame build error");
            return false;
        }

        Arbiter::write_raw(frame, frame_len);
        frame_count++;
        offset += chunk_len;
        flash_sent += chunk_len;
        seq = (seq + 1) & 0xFF;

        // Drain responses
        if (frame_count % 20 == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            qframe_t rx;
            while (Arbiter::wait_frame(&rx, 10)) {
                if (rx.type == QFRAME_TYPE_E) {
                    char err_str[32] = {};
                    memcpy(err_str, rx.payload, min((int)rx.payload_len, 31));
                    snprintf(flash_error, sizeof(flash_error),
                             "Flash error at %s+0x%X: %s", block->name, offset, err_str);
                    return false;
                }
            }
        }
    }

    if (flash_cancel) return false;

    if (send_completion) {
        // Completion frame
        snprintf(flash_phase, sizeof(flash_phase), "Complete %s", block->name);
        Log::logf(CAT_OTA, LOG_INFO, "[OTA] Sending completion frame for %s (%d frames sent)...\n",
                     block->name, frame_count);

        {
            uint8_t f_payload[8];
            int f_len = build_f_payload(f_payload, sizeof(f_payload),
                                         block->name, seq, nullptr, 0, true);
            uint8_t frame[64];
            int frame_len = qframe_build('f', f_payload, f_len, frame, sizeof(frame));
            if (frame_len > 0) {
                Arbiter::write_raw(frame, frame_len);
            }
        }

        // Device resets after completion - restore baud
        Arbiter::set_baud(57600);
    } else {
        Log::logf(CAT_OTA, LOG_INFO, "[OTA] %s data sent (%d frames), skipping completion (chaining)\n",
                     block->name, frame_count);
    }

    Log::logf(CAT_OTA, LOG_INFO, "[OTA] %s flash done\n", block->name);
    return true;
}


static void flash_task(void *param) {
    flash_params_t *p = (flash_params_t*)param;
    char resp[128] = {};
    bool is_full = (strcmp(p->block, "FULL") == 0);

    flash_active = true;
    flash_cancel = false;
    flash_sent = 0;
    flash_error[0] = '\0';

    // Calculate total bytes
    if (is_full) {
        const block_info_t *cmx = find_block("CMX");
        size_t cmx_size = min(p->fw_size - 0x4000, (size_t)cmx->max_size);
        flash_total = cmx_size;
        if (p->flash_blx) {
            flash_total += find_block("BLX")->max_size;
        }
    } else {
        flash_total = p->fw_size;
    }

    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "resmed");
    if (!part) {
        snprintf(flash_error, sizeof(flash_error), "resmed partition not found");
        goto done;
    }

    // BLX safety check
    if ((is_full && p->flash_blx) || strcmp(p->block, "BLX") == 0) {
        if (!check_bid(part, 0, p->force_blx)) goto done;
    }


    Arbiter::set_state(SYS_OTA_AIRSENSE);

    if (!enter_bootloader()) goto cleanup;
    if (flash_cancel) goto cleanup;

    // Negotiate baud
    {
        strncpy(flash_phase, "Baud negotiate", sizeof(flash_phase));
        Log::logf(CAT_OTA, LOG_INFO, "[OTA] Negotiating baud 460800...\n");
        uint8_t bdd_frame[64];
        int bdd_len = qframe_build_cmd("P S #BDD 0002", bdd_frame, sizeof(bdd_frame));
        if (bdd_len > 0) {
            Arbiter::write_raw(bdd_frame, bdd_len);
            vTaskDelay(pdMS_TO_TICKS(100));
            // Consume ACK at old baud
            qframe_t rx;
            Arbiter::wait_frame(&rx, 500);
            // Switch
            Arbiter::set_baud(460800);
            Log::logf(CAT_OTA, LOG_INFO, "[OTA] Baud set to %u\n", Arbiter::get_baud());
        }
    }

    if (is_full) {
        // FULL image: BLX (optional, no completion) -> CMX (with completion)
        if (p->flash_blx) {
            const block_info_t *blx = find_block("BLX");
            // Skip completion frame
            if (!flash_one_block(part, 0, blx, blx->max_size, false)) goto cleanup;

            // Bootloader enters mode 5 (upgrade) after receiving data.
            // Without completion frame, mode 5 times out after ~2s and
            // bootloader reverts to idle at 57600 baud.
            strncpy(flash_phase, "BLX mode timeout", sizeof(flash_phase));
            Log::logf(CAT_OTA, LOG_INFO, "[OTA] Waiting for mode 5 timeout (~2s)...\n");
            vTaskDelay(pdMS_TO_TICKS(2500));
            Arbiter::set_baud(57600);

            // Verify still in bootloader
            char bl_resp[128] = {};
            if (!send_raw_cmd("G S #BLS", bl_resp, sizeof(bl_resp), 500)) {
                snprintf(flash_error, sizeof(flash_error),
                         "Lost bootloader after BLX flash");
                goto cleanup;
            }
            Log::logf(CAT_OTA, LOG_INFO, "[OTA] Still in bootloader after BLX\n");

            // Re-negotiate baud for CMX
            {
                Log::logf(CAT_OTA, LOG_INFO, "[OTA] Re-negotiating baud 460800...\n");
                uint8_t bdd_frame[64];
                int bdd_len = qframe_build_cmd("P S #BDD 0002", bdd_frame, sizeof(bdd_frame));
                if (bdd_len > 0) {
                    Arbiter::write_raw(bdd_frame, bdd_len);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    qframe_t rx;
                    Arbiter::wait_frame(&rx, 500);
                    Arbiter::set_baud(460800);
                    Log::logf(CAT_OTA, LOG_INFO, "[OTA] Baud re-negotiated to %u\n", Arbiter::get_baud());
                }
            }
        }

        const block_info_t *cmx = find_block("CMX");
        size_t cmx_size = min(p->fw_size - 0x4000, (size_t)cmx->max_size);

        if (!flash_one_block(part, 0x4000, cmx, cmx_size, true)) goto cleanup;

    } else {
        const block_info_t *block = find_block(p->block);
        if (!block) {
            snprintf(flash_error, sizeof(flash_error), "Unknown block: %s", p->block);
            goto cleanup;
        }
        if (p->fw_size > block->max_size) {
            snprintf(flash_error, sizeof(flash_error), "Firmware %u > %s max %u",
                     p->fw_size, block->name, block->max_size);
            goto cleanup;
        }
        if (!flash_one_block(part, 0, block, p->fw_size)) goto cleanup;
    }

    // Wait for device to boot new firmware
    {
        strncpy(flash_phase, "Verifying", sizeof(flash_phase));
        Arbiter::set_baud(57600);
        bool app_running = false;
        for (int i = 0; i < 30 && !flash_cancel; i++) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            memset(resp, 0, sizeof(resp));
            if (send_raw_cmd("G S #BLS", resp, sizeof(resp), 2000)) {
                char *eq = strstr(resp, "= ");
                if (eq) {
                    int bls = (int)strtol(eq + 2, nullptr, 16);
                    if (bls == 0) { app_running = true; break; }
                }
            }
        }

        if (app_running) {
            strncpy(flash_phase, "Complete", sizeof(flash_phase));
            Log::logf(CAT_OTA, LOG_INFO, "[OTA] Flash complete, device running\n");
            Config::invalidate_device_info();
        } else if (!flash_cancel) {
            snprintf(flash_error, sizeof(flash_error),
                     "Device did not return to app after flash");
        }
    }

cleanup:
    Arbiter::set_baud(57600);
    Arbiter::set_state(SYS_IDLE);
    if (flash_cancel && flash_error[0] == '\0') {
        strncpy(flash_error, "Cancelled by user", sizeof(flash_error));
        strncpy(flash_phase, "Cancelled", sizeof(flash_phase));
    }

done:
    if (flash_error[0] != '\0') {
        strncpy(flash_phase, "Error", sizeof(flash_phase));
        Log::logf(CAT_OTA, LOG_ERROR, "[OTA] Error: %s\n", flash_error);
    }
    flash_active = false;
    flash_task_handle = nullptr;
    vTaskDelete(nullptr);
}



const char* ResmedOta::detect_block(size_t fw_size) {
    if (fw_size == 0) return nullptr;
    if (fw_size <= 0x4000)   return "BLX";
    if (fw_size <= 0x3C000)  return "CCX";
    if (fw_size <= 0xC0000)  return "CDX";
    if (fw_size <= 0xFC000)  return "CMX";
    if (fw_size <= 0x100000) return "FULL";
    return nullptr;
}

void ResmedOta::start_flash(const char *block, size_t fw_size,
                            bool flash_blx, bool force_blx) {
    if (flash_active) return;

    if (!block || block[0] == '\0') {
        block = detect_block(fw_size);
        if (!block) {
            strncpy(flash_error, "Cannot detect block for this file size", sizeof(flash_error));
            return;
        }
    }

    strncpy(flash_params.block, block, sizeof(flash_params.block) - 1);
    flash_params.block[sizeof(flash_params.block) - 1] = '\0';
    flash_params.fw_size = fw_size;
    flash_params.flash_blx = flash_blx;
    flash_params.force_blx = force_blx;

    xTaskCreatePinnedToCore(flash_task, "resmed_ota", FLASH_TASK_STACK,
                            &flash_params, FLASH_TASK_PRIO, &flash_task_handle, 1);
}

void ResmedOta::cancel()            { flash_cancel = true; }
bool ResmedOta::is_active()         { return flash_active; }
const char* ResmedOta::get_phase()  { return flash_phase; }
size_t ResmedOta::get_sent()        { return flash_sent; }
size_t ResmedOta::get_total()       { return flash_total; }
const char* ResmedOta::last_error() { return flash_error; }
