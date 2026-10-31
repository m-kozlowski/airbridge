#include "resmed_ota.h"

#include "airbridge_ota.h"
#include "airsense_state.h"
#include "custom_settings.h"
#include "debug_log.h"
#include "live_stream.h"
#include "memory_manager.h"
#include "qframe.h"
#include "resmed_firmware_repository.h"
#include "file_upload.h"
#include "sd_storage.h"
#include "uart_arbiter.h"
#include "board.h"

#if AB_STORAGE_HAS_SDCARD
#include <FS.h>
#endif

#include <atomic>
#include <stdarg.h>
#include <esp_ota_ops.h>
#include <freertos/task.h>

namespace ResmedOta {
namespace {

namespace Image = ResmedFirmwareImage;
constexpr size_t BLX_BYTES = 16 * 1024;
constexpr size_t SOURCE_CHUNK = 4096;
constexpr size_t UART_CHUNK = 250;
constexpr size_t INTERNAL_RESERVE = 32 * 1024;
constexpr uint32_t FLASH_TASK_STACK = 8192;

portMUX_TYPE status_mux = portMUX_INITIALIZER_UNLOCKED;
Status published;
std::atomic<bool> active{false};
std::atomic<bool> cancel_requested{false};
bool programming = false;  // Protected by status_mux, like published.

struct Work {
    bool dump = false;
    bool waiting = false;
    bool recovery = false;
    bool bootloader_entered = false;
    bool app_confirmed = false;
    bool quiet_done = false;
    char expected_bid[32] = {};
    char expected_serial[24] = {};
    InstallRequest request;
    Source source = Source::None;
    FileUpload::Source staged;
};
Work work;

void phase(const char *name) {
    portENTER_CRITICAL(&status_mux);
    snprintf(published.phase, sizeof(published.phase), "%s", name);
    ++published.revision;
    portEXIT_CRITICAL(&status_mux);
}

bool fail(const char *format, ...) {
    char message[128];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    portENTER_CRITICAL(&status_mux);
    if (!published.error[0]) {
        memcpy(published.error, message, sizeof(message));
        ++published.revision;
    }
    portEXIT_CRITICAL(&status_mux);
    return false;
}

bool cancelled() {
    portENTER_CRITICAL(&status_mux);
    const bool result = cancel_requested.load() && !programming;
    portEXIT_CRITICAL(&status_mux);
    return result;
}

bool begin_programming() {
    portENTER_CRITICAL(&status_mux);
    const bool allowed = programming || !cancel_requested.load();
    if (allowed && !programming) {
        programming = true;
        published.cancel_allowed = false;
        ++published.revision;
    }
    portEXIT_CRITICAL(&status_mux);
    return allowed;
}

uint8_t *allocate_buffer(size_t size) {
    void *buffer = aircannect::Memory::alloc_large(size, false);
    if (!buffer) {
        // Fixed workload bounds, with headroom for normal UART/BLE operation.
        const auto memory = aircannect::Memory::status();
        if (size <= BLX_BYTES && memory.heap_free >= size + INTERNAL_RESERVE &&
            memory.heap_max_alloc >= size)
            buffer = aircannect::Memory::alloc_large(size, true);
    }
    return static_cast<uint8_t *>(buffer);
}

struct Input {
    SdStorage::Session session;
    SdStorage::Reader file;
    const esp_partition_t *partition = nullptr;
    uint64_t size = 0;
    uint8_t *blx = nullptr;
    uint8_t *scratch = nullptr;

    ~Input() {
        file.close();
        session.end();
        aircannect::Memory::free(blx);
        aircannect::Memory::free(scratch);
    }

    bool read_source(uint64_t offset, uint8_t *out, size_t length) {
        if (offset > size || length > size - offset || length > SOURCE_CHUNK) return false;
        if (partition)
            return esp_partition_read(partition, offset, out, length) == ESP_OK;
        return session.valid() && file.read_at(offset, out, length);
    }

    static bool read(void *context, uint64_t offset, uint8_t *out, size_t length) {
        auto &input = *static_cast<Input *>(context);
        if (offset > input.size || length > input.size - offset) return false;
        if (input.blx && offset < BLX_BYTES) {
            const size_t count = min(length, BLX_BYTES - static_cast<size_t>(offset));
            memcpy(out, input.blx + offset, count);
            offset += count;
            out += count;
            length -= count;
        }
        return !length || input.read_source(offset, out, length);
    }

    Image::Reader reader() { return {this, size, read}; }
};

bool acquire_firmware(Input &input, const char *path) {
    const uint32_t started = millis();
    while (!input.session.begin(SdStorage::Access::Firmware, path)) {
        if (cancelled()) return false;
        if (!SdStorage::local_access_allowed() || !SdStorage::mounted()) return fail("sd_unavailable");
        if (millis() - started >= 5000) return fail("sd_busy");
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return true;
}

bool open_input(Input &input) {
    if (work.source == Source::Staging) {
        input.partition = work.staged.partition;
        input.size = work.staged.size;
        if (!input.partition || !input.size || input.size > input.partition->size)
            return fail("invalid_staged_source");
    } else {
        if (!ResmedFirmwareRepository::valid_path(work.request.path))
            return fail("invalid_repository_path");
        phase("Open SD image");
        if (!acquire_firmware(input, work.request.path)) return false;
        if (!input.file.open(input.session, work.request.path))
            return fail("source_open_failed");
        input.size = input.file.size();
    }
    if (!input.size || input.size > Image::FULL_SIZE) return fail("invalid_image_size");
    input.scratch = allocate_buffer(SOURCE_CHUNK);
    if (!input.scratch) return fail("source_buffer_unavailable");
    if (input.size == Image::FULL_SIZE ||
        Image::parse_block(work.request.block) == Image::Block::BLX ||
        (!work.request.block[0] && input.size == BLX_BYTES)) {
        if (input.size < BLX_BYTES) return fail("incomplete_blx");
        input.blx = allocate_buffer(BLX_BYTES);
        if (!input.blx) return fail("blx_buffer_unavailable");
        // Finish every BLX source read before any device erase, including FULL.
        for (size_t pos = 0; pos < BLX_BYTES; pos += SOURCE_CHUNK) {
            if (cancelled()) return false;
            if (!input.read_source(pos, input.blx + pos, SOURCE_CHUNK))
                return fail("blx_preflight_read_failed");
        }
    }
    return true;
}

bool device_text(const char *tag, char *out, size_t capacity) {
    return Arbiter::read_var(tag, CMD_SRC_OTA, CMD_PRIO_CRITICAL,
                             out, capacity, 1000) == Arbiter::VarResult::Ok;
}

struct Plan {
    const Image::Profile *profile = nullptr;
    Image::Result image;
    Image::Range blocks[2] = {};
    uint32_t offsets[2] = {};
    size_t count = 0;
    char bid[32] = {};
    char serial[24] = {};
};

bool freeze_device(Plan &plan) {
    if (!AirSenseState::system_idle()) return fail("device_not_idle");
    if (!device_text("BID", plan.bid, sizeof(plan.bid))) return fail("device_bid_unavailable");
    plan.profile = Image::profile_for_bid(plan.bid);
    if (!plan.profile) return fail("unsupported_device_bid: %s", plan.bid);
    if (!device_text("SRN", plan.serial, sizeof(plan.serial)) || !plan.serial[0])
        return fail("device_identity_unavailable");
    return true;
}

bool prepare(Input &input, Plan &plan) {
    phase("Prepare image");
    if (!freeze_device(plan)) return false;
    Image::Block selected = Image::parse_block(work.request.block);
    const bool full = input.size == Image::FULL_SIZE;
    Image::Options options;
    options.platform = plan.profile->platform;
    options.target = full ? Image::Block::FULL : selected;
    const auto shape = full ? Image::Block::FULL : selected != Image::Block::Unknown
        ? selected : input.size == BLX_BYTES ? Image::Block::BLX : Image::Block::Unknown;
    if (!Image::inspect(input.reader(), plan.image, options, input.scratch, SOURCE_CHUNK)) {
        portENTER_CRITICAL(&status_mux);
        published.image = plan.image;
        ++published.revision;
        portEXIT_CRITICAL(&status_mux);
        return fail("image_inspection_failed: %u", unsigned(plan.image.status));
    }
    if (selected == Image::Block::Unknown) selected = plan.image.target;
    const bool has_blx = shape == Image::Block::FULL || shape == Image::Block::BLX;
    const bool warnings = !plan.image.crc_ok() ||
        (has_blx && !plan.image.bid_matches) ||
        plan.image.integrity_patch == Image::IntegrityPatch::Unsafe ||
        plan.image.integrity_patch == Image::IntegrityPatch::Partial;

    // Publish factual inspection even when explicit warning confirmation is needed.
    portENTER_CRITICAL(&status_mux);
    published.image = plan.image;
    snprintf(published.device_bid, sizeof(published.device_bid), "%s", plan.bid);
    ++published.revision;
    portEXIT_CRITICAL(&status_mux);
    if (warnings && !work.request.force_blx) return fail("image_warnings_require_force");

    if (selected == Image::Block::FULL) {
        if (work.request.flash_blx) plan.blocks[plan.count++] = *Image::range(*plan.profile, Image::Block::BLX);
        plan.blocks[plan.count++] = *Image::range(*plan.profile, Image::Block::CMX);
    } else {
        const auto *block = Image::range(*plan.profile, selected);
        if (!block || (!full && input.size != block->size)) return fail("invalid_selected_region");
        plan.blocks[plan.count++] = *block;
    }
    size_t total = 0;
    for (size_t i = 0; i < plan.count; ++i) {
        plan.offsets[i] = full ? plan.blocks[i].image_offset : 0;
        if (plan.offsets[i] > input.size || plan.blocks[i].size > input.size - plan.offsets[i])
            return fail("selected_region_out_of_bounds");
        if (plan.blocks[i].block == Image::Block::BLX && !input.blx)
            return fail("blx_not_preloaded");
        total += plan.blocks[i].size;
    }
    if (cancelled()) return false;
    portENTER_CRITICAL(&status_mux);
    published.prepared = true;
    published.total = total;
    snprintf(published.block, sizeof(published.block), "%s", Image::block_name(selected));
    ++published.revision;
    portEXIT_CRITICAL(&status_mux);
    return true;
}

// Existing raw UART framing is used only after exclusive stream suspension.
bool send_raw_cmd(const char *cmd, char *resp, uint16_t capacity, uint16_t timeout_ms = 2000) {
    uint8_t frame[QFRAME_MAX_RAW];
    const int length = qframe_build_cmd(cmd, frame, sizeof(frame));
    if (length <= 0) return false;
    Arbiter::clear_rx_frames();
    Arbiter::write_raw(frame, length);
    if (!timeout_ms) return true;
    qframe_t reply;
    const uint32_t started = millis();
    while (millis() - started < timeout_ms) {
        const uint16_t remaining = timeout_ms - (millis() - started);
        if (!Arbiter::wait_frame(&reply, remaining)) break;
        if ((reply.type != QFRAME_TYPE_R && reply.type != QFRAME_TYPE_E) ||
            !qframe_response_matches(frame, length, reply)) continue;
        if (resp && capacity) {
            const size_t count = min(size_t(reply.payload_len), size_t(capacity - 1));
            memcpy(resp, reply.payload, count);
            resp[count] = 0;
        }
        return reply.type == QFRAME_TYPE_R;
    }
    return false;
}

bool raw_text(const char *command, char *out, size_t capacity, uint16_t timeout = 1000) {
    char response[128] = {};
    if (!send_raw_cmd(command, response, sizeof(response), timeout)) return false;
    const char *value = qframe_response_value(response);
    if (!value || !*value || strlen(value) >= capacity) return false;
    snprintf(out, capacity, "%s", value);
    return true;
}

bool raw_hex(const char *command, uint32_t &out, uint16_t timeout = 1000) {
    char value[16];
    if (!raw_text(command, value, sizeof(value), timeout)) return false;
    char *end = nullptr;
    const unsigned long parsed = strtoul(value, &end, 16);
    if (!end || end == value || *end || parsed > UINT32_MAX) return false;
    out = parsed;
    return true;
}

bool enter_bootloader() {
    phase("Enter bootloader");
    uint32_t state = 0;
    if (cancelled()) return false;
    work.bootloader_entered = true;
    work.app_confirmed = false;
    work.quiet_done = false;
    if (raw_hex("G S #BLS", state, 300) && (state == 1 || state == 2)) return true;
    if (!send_raw_cmd("P S #BLL 0001", nullptr, 0, 0)) return fail("bootloader_entry_failed");
    Arbiter::set_baud(57600);
    for (unsigned attempt = 0; attempt < 100 && !cancelled(); ++attempt) {
        if (raw_hex("G S #BLS", state, 100) && (state == 1 || state == 2)) return true;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return cancelled() ? false : fail("bootloader_entry_timeout");
}

bool negotiate_baud() {
    phase("Baud negotiate");
    if (!send_raw_cmd("P S #BDD 0002", nullptr, 0, 1000)) return fail("baud_negotiate_failed");
    Arbiter::set_baud(460800);
    return true;
}

int build_record_03(uint8_t *out, size_t capacity, uint32_t address,
                    const uint8_t *data, size_t count) {
    if (count > UART_CHUNK || capacity < count + 7) return -1;
    out[0] = 0x03;
    out[1] = count + 5;
    out[2] = address >> 24;
    out[3] = address >> 16;
    out[4] = address >> 8;
    out[5] = address;
    memcpy(out + 6, data, count);
    out[6 + count] = 0;
    return count + 7;
}

int build_f_payload(uint8_t *out, size_t capacity, const char *block,
                    uint8_t sequence, const uint8_t *record, size_t length, bool complete) {
    if (strlen(block) != 3 || capacity < 5 + (complete ? 0 : length)) return -1;
    memcpy(out, block, 3);
    out[3] = complete ? 'F' : 0;
    out[4] = sequence;
    if (!complete && length) memcpy(out + 5, record, length);
    return 5 + (complete ? 0 : length);
}

bool wait_for_erase(const uint8_t *request, size_t length) {
    const uint32_t started = millis();
    qframe_t reply;
    while (millis() - started < 30000) {
        if (!Arbiter::wait_frame(&reply, 500)) continue;
        if (!qframe_response_matches(request, length, reply)) continue;
        if (reply.type == QFRAME_TYPE_R) return true;
        if (reply.type == QFRAME_TYPE_E) return fail("device_erase_error");
    }
    return fail("device_erase_timeout");
}

bool flash_one_block(const Image::Reader &reader, size_t source_offset,
                     const Image::Range &block, uint8_t *scratch, bool complete) {
    const char *name = Image::block_name(block.block);
    char label[32];
    snprintf(label, sizeof(label), "Erase %s", name);
    phase(label);
    if (!begin_programming()) return false;
    char command[24];
    snprintf(command, sizeof(command), "P F *%s 0000", name);
    uint8_t erase[64];
    const int erase_length = qframe_build_cmd(command, erase, sizeof(erase));
    if (erase_length <= 0) return fail("erase_frame_failed");
    Arbiter::clear_rx_frames();
    Arbiter::write_raw(erase, erase_length);
    if (!wait_for_erase(erase, erase_length)) return false;
    vTaskDelay(pdMS_TO_TICKS(300));

    snprintf(label, sizeof(label), "Flash %s", name);
    phase(label);
    uint8_t sequence = 0;
    unsigned frames = 0;
    // A source read is 4 KiB, independent of the UART's 250-byte record limit.
    // Never consult cancellation after erase: especially BLX must finish.
    for (size_t offset = 0; offset < block.size;) {
        const size_t count = min(SOURCE_CHUNK, size_t(block.size - offset));
        if (!reader.read_at(reader.context, source_offset + offset, scratch, count))
            return fail("source_read_failed: %s+0x%X", name, unsigned(offset));
        for (size_t pos = 0; pos < count;) {
            const size_t chunk = min(UART_CHUNK, count - pos);
            uint8_t record[UART_CHUNK + 7], payload[UART_CHUNK + 16], frame[QFRAME_MAX_RAW];
            const int record_length = build_record_03(record, sizeof(record),
                block.flash_start + offset + pos, scratch + pos, chunk);
            if (record_length <= 0) return fail("record_frame_failed");
            const int payload_length = build_f_payload(payload, sizeof(payload), name,
                sequence, record, record_length, false);
            if (payload_length <= 0) return fail("payload_frame_failed");
            const int frame_length = qframe_build('f', payload, payload_length, frame, sizeof(frame));
            if (frame_length <= 0) return fail("data_frame_failed");
            Arbiter::write_raw(frame, frame_length);
            pos += chunk;
            ++sequence;
            portENTER_CRITICAL(&status_mux);
            published.sent += chunk;
            ++published.revision;
            portEXIT_CRITICAL(&status_mux);
            if (++frames % 20 == 0) {
                vTaskDelay(pdMS_TO_TICKS(10));
                qframe_t reply;
                while (Arbiter::wait_frame(&reply, 10)) {
                    if (reply.type == QFRAME_TYPE_E) return fail("device_program_error: %s", name);
                }
            }
        }
        offset += count;
    }
    uint32_t error = 0;
    if (!raw_hex("G S #BLE", error) || (error != 0 && error != 0x700D))
        return fail("device_program_error: %s %04X", name, unsigned(error));
    if (complete) {
        phase("Complete transfer");
        uint8_t payload[8], frame[64];
        const int payload_length = build_f_payload(payload, sizeof(payload), name,
            sequence, nullptr, 0, true);
        const int length = qframe_build('f', payload, payload_length, frame, sizeof(frame));
        if (length <= 0) return fail("completion_frame_failed");
        Arbiter::write_raw(frame, length);
    }
    return true;
}

bool confirm_install(const Plan &plan) {
    phase("Verify device");
    // Completion clears BLE, but does not verify the image. The transfer status
    // was checked before that marker; require fresh app status/identity here.
    uint32_t error = 0;
    Arbiter::set_baud(57600);
    for (unsigned attempt = 0; attempt < 10; ++attempt) {
        // Successful bootloader commands restart its ~2s activity timeout.
        // Leave a real quiet window, including after a clean-status retry.
        vTaskDelay(pdMS_TO_TICKS(3000));
        uint32_t state = UINT32_MAX;
        if (!raw_hex("G S #BLS", state, 1000)) continue;
        if (state != 0) {
            if (raw_hex("G S #BLE", error) && error && error != 0x700D)
                return fail("device_bootloader_error: %04X", unsigned(error));
            continue;
        }
        char bid[32], serial[24], sid[64];
        if (!raw_hex("G S #BLE", error) || error != 0)
            return fail("device_bootloader_error: %04X", unsigned(error));
        if (!raw_text("G S #BID", bid, sizeof(bid)) ||
            !raw_text("G S #SRN", serial, sizeof(serial)) ||
            !raw_text("G S #SID", sid, sizeof(sid))) continue;
        // FORCE permits an explicitly selected different BLX; it does not
        // permit a different physical device to satisfy this operation.
        const char *expected_bid = plan.bid;
        bool wrote_blx = false;
        for (size_t i = 0; i < plan.count; ++i)
            wrote_blx |= plan.blocks[i].block == Image::Block::BLX;
        if (wrote_blx && plan.image.bid_present) expected_bid = plan.image.bid;
        if (strcmp(bid, expected_bid) || strcmp(serial, plan.serial))
            return fail("device_identity_changed");
        work.app_confirmed = true;
        return true;
    }
    return fail("device_did_not_return_to_application");
}

bool claim_uart(Input &input, const Plan &plan) {
    if (cancelled()) return false;
    snprintf(work.expected_bid, sizeof(work.expected_bid), "%s", plan.bid);
    snprintf(work.expected_serial, sizeof(work.expected_serial), "%s", plan.serial);
    if (!AirSenseState::system_idle()) return fail("device_not_idle");
    Arbiter::set_state(SYS_OTA_AIRSENSE);
    // Detect an incompatible/revoked lease before any erase or bootloader entry.
    if (!input.partition && !input.session.valid()) return fail("sd_ota_lease_unavailable");
    phase("Claim UART");
    if (!Arbiter::wait_idle(3000)) return fail("uart_busy");
    phase("Suspend streams");
    if (!LiveStream::suspend_for_ota()) return fail("stream_suspend_failed");
    char current_bid[32], current_serial[24];
    if (!raw_text("G S #BID", current_bid, sizeof(current_bid)) ||
        !raw_text("G S #SRN", current_serial, sizeof(current_serial)) ||
        strcmp(current_bid, plan.bid) || strcmp(current_serial, plan.serial))
        return fail("device_identity_changed");
    if (cancelled()) return false;
    CustomSettings::invalidate("ResMed OTA");
    AirSenseState::invalidate_identity();
    return true;
}

bool run_install() {
    Input input;
    Plan plan;
    if (!open_input(input) || cancelled() || !prepare(input, plan) || !claim_uart(input, plan)) return false;
    bool success = false;
    do {
        if (work.request.preserve_settings) {
            phase("Preserve settings");
            uint32_t rop = 0;
            if (cancelled()) break;
            if (!send_raw_cmd("P S #ROP 0005", nullptr, 0, 1000) ||
                !raw_hex("G S #ROP", rop) || rop != 5) {
                fail("preserve_settings_failed");
                break;
            }
        }
        if (!enter_bootloader() || !negotiate_baud()) break;
        bool programmed = true;
        for (size_t i = 0; i < plan.count; ++i) {
            if (!flash_one_block(input.reader(), plan.offsets[i], plan.blocks[i],
                                  input.scratch, i + 1 == plan.count)) {
                programmed = false; break;
            }
            if (i + 1 < plan.count) {
                phase("BLX mode timeout");
                vTaskDelay(pdMS_TO_TICKS(2500));
                Arbiter::set_baud(57600);
                uint32_t state = 0;
                if (!raw_hex("G S #BLS", state) || (state != 1 && state != 2)) {
                    fail("lost_bootloader_after_blx"); programmed = false; break;
                }
                if (!negotiate_baud()) { programmed = false; break; }
            }
        }
        if (programmed) success = confirm_install(plan);
    } while (false);
    return success;
}

#if AB_STORAGE_HAS_SDCARD
bool dump_chunk(uint32_t offset, uint8_t *out, size_t length) {
    if (!length || length > 240 || offset > Image::FULL_SIZE || length > Image::FULL_SIZE - offset)
        return false;
    uint8_t payload[7] = {'D'};
    SdStorage::put_le32(payload + 1, offset);
    SdStorage::put_le16(payload + 5, length);
    uint8_t frame[64];
    const int count = qframe_build('O', payload, sizeof(payload), frame, sizeof(frame));
    if (count <= 0) return false;
    for (unsigned attempt = 0; attempt < 3 && !cancelled(); ++attempt) {
        Arbiter::clear_rx_frames();
        Arbiter::write_raw(frame, count);
        const uint32_t started = millis();
        qframe_t reply;
        while (millis() - started < 1000) {
            if (!Arbiter::wait_frame(&reply, 1000 - (millis() - started))) break;
            if (reply.type != 'O' || !reply.crc_valid || reply.payload_len < 7) continue;
            if (reply.payload[0] == 'E' && SdStorage::get_le32(reply.payload + 1) == offset) return false;
            if (reply.payload[0] != 'd' || reply.payload_len != 7 + length ||
                SdStorage::get_le32(reply.payload + 1) != offset ||
                SdStorage::get_le16(reply.payload + 5) != length) continue;
            memcpy(out, reply.payload + 7, length);
            return true;
        }
    }
    return false;
}

bool dump_range(uint32_t offset, uint8_t *out, size_t length) {
    while (length) {
        const size_t count = min(size_t(240), length);
        if (!dump_chunk(offset, out, count)) return false;
        offset += count;
        out += count;
        length -= count;
    }
    return true;
}

bool recover_dump(Input &input, Plan &plan) {
    char path[256];
    if (!ResmedFirmwareRepository::bootloader_path(plan.bid, path, sizeof(path)))
        return fail("unsupported_device_bid");
    portENTER_CRITICAL(&status_mux);
    snprintf(published.recovery_path, sizeof(published.recovery_path), "%s", path);
    ++published.revision;
    portEXIT_CRITICAL(&status_mux);
    if (!input.file.open(input.session, path)) return fail("matching_bootloader_missing");
    input.size = input.file.size();
    if (input.size != BLX_BYTES) return fail("invalid_recovery_blx_size");
    for (size_t offset = 0; offset < BLX_BYTES; offset += SOURCE_CHUNK) {
        if (cancelled()) return false;
        if (!input.read_source(offset, input.blx + offset, SOURCE_CHUNK))
            return fail("blx_preflight_read_failed");
    }
    input.file.close();
    Image::Options options;
    options.target = Image::Block::BLX;
    options.platform = plan.profile->platform;
    const bool inspected = Image::inspect(input.reader(), plan.image, options, input.scratch, SOURCE_CHUNK);
    portENTER_CRITICAL(&status_mux);
    published.image = plan.image;
    ++published.revision;
    portEXIT_CRITICAL(&status_mux);
    // Canonical path plus user consent declares the extension. Exact payload
    // recognition is informational; BID, CRC and unsafe integrity patches gate.
    if (!inspected || !plan.image.bid_matches || !plan.image.crc_ok() ||
        plan.image.integrity_patch == Image::IntegrityPatch::Unsafe ||
        plan.image.integrity_patch == Image::IntegrityPatch::Partial)
        return fail("invalid_recovery_blx");

    if (!work.recovery) {
        // Leave no task, RAM buffer or reservation waiting on human input.
        // Consent resumes this ID, reclaims admission and rereads the source.
        snprintf(work.expected_bid, sizeof(work.expected_bid), "%s", plan.bid);
        snprintf(work.expected_serial, sizeof(work.expected_serial), "%s", plan.serial);
        snprintf(work.request.path, sizeof(work.request.path), "%s", path);
        Arbiter::set_baud(57600);
        vTaskDelay(pdMS_TO_TICKS(3000));
        work.quiet_done = true;
        if (cancelled()) return false;
        work.waiting = true;
        portENTER_CRITICAL(&status_mux);
        published.prepared = true;
        published.recovery_available = true;
        snprintf(published.source_path, sizeof(published.source_path), "%s", path);
        ++published.revision;
        portEXIT_CRITICAL(&status_mux);
        return false;
    }
    if (!enter_bootloader() || !negotiate_baud()) return false;
    plan.blocks[0] = *Image::range(*plan.profile, Image::Block::BLX);
    plan.count = 1;
    if (!flash_one_block(input.reader(), 0, plan.blocks[0], input.scratch, true) || !confirm_install(plan))
        return false;
    // Cancellation during BLX programming was deferred until device confirmation.
    // It may safely stop the subsequent read-only dump now.
    portENTER_CRITICAL(&status_mux);
    programming = false;
    published.cancel_allowed = true;
    published.sent = 0;
    ++published.revision;
    portEXIT_CRITICAL(&status_mux);
    if (cancelled()) return false;
    if (!enter_bootloader() || !negotiate_baud()) return false;
    if (!dump_chunk(0, input.blx, 240)) return fail("dump_unavailable_after_recovery");
    return true;
}

struct DumpReader {
    Input &input;
    SdStorage::Writer &output;
    uint32_t written = 0;

    static bool read(void *context, uint64_t offset, uint8_t *out, size_t length) {
        auto &dump = *static_cast<DumpReader *>(context);
        if (cancelled() || length > SOURCE_CHUNK || offset > Image::FULL_SIZE ||
            length > Image::FULL_SIZE - offset) return false;
        if (offset < BLX_BYTES) {
            if (length > BLX_BYTES - offset) return false;
            memcpy(out, dump.input.blx + offset, length);
        } else if (!dump_range(offset, out, length)) return false;
        // The inspector's BID lookup borrows preloaded BLX, not another UART
        // pass or an out-of-order write. Its CRC reads then stream the full file.
        if (offset == dump.written) {
            if (!dump.output.write(out, length)) return fail("dump_sd_write_failed");
            dump.written += length;
            portENTER_CRITICAL(&status_mux);
            published.sent = dump.written;
            ++published.revision;
            portEXIT_CRITICAL(&status_mux);
        } else if (offset >= BLX_BYTES) return false;
        return true;
    }
};

struct DumpOutput {
    Input &input;
    SdStorage::Writer writer;
    char path[192] = {}, temporary[256] = {};
    bool private_created = false;
    bool committed = false;

    ~DumpOutput() {
        writer.close();
        if (private_created && !committed)
            input.session.mutate(temporary, [&](fs::FS &fs) {
                return !fs.exists(temporary) || fs.remove(temporary);
            });
    }

    bool prepare_name() {
        char pna[64] = {}, fgt[32] = {}, sid[64] = {};
        if (!device_text("FGT", fgt, sizeof(fgt)) || !fgt[0])
            return fail("device_fgt_unavailable");
        if (!device_text("SID", sid, sizeof(sid)) || !sid[0])
            return fail("device_sid_unavailable");
        if (!ResmedFirmwareRepository::valid_filename(fgt) ||
            !ResmedFirmwareRepository::valid_filename(sid))
            return fail("invalid_dump_identity");
        if (!device_text("PNA", pna, sizeof(pna))) pna[0] = '\0';
        const char *family = "air10";
        if (!pna[11] || pna[11] == '_' || pna[11] == ' ') {
            if (!strncasecmp(pna, "AirSense_10", 11) || !strncasecmp(pna, "AirSense 10", 11))
                family = "airsense10";
            else if (!strncasecmp(pna, "AirCurve_10", 11) || !strncasecmp(pna, "AirCurve 10", 11))
                family = "aircurve10";
        }
        const int length = snprintf(path, sizeof(path), "%s/dump-%s-%s_%s",
            ResmedFirmwareRepository::ROOT, family, fgt, sid);
        return (length > 0 && static_cast<size_t>(length) < sizeof(path)) ||
            fail("dump_name_too_long");
    }

    bool open() {
        if (!input.session.mutate(ResmedFirmwareRepository::ROOT, [](fs::FS &fs) {
            return (fs.exists("/airbridge") || fs.mkdir("/airbridge")) &&
                (fs.exists(ResmedFirmwareRepository::ROOT) || fs.mkdir(ResmedFirmwareRepository::ROOT));
        })) return fail("repository_unavailable");
        const size_t stem_length = strlen(path);
        for (unsigned copy = 1; copy <= 256; ++copy) {
            const int length = copy == 1 ?
                snprintf(path + stem_length, sizeof(path) - stem_length, ".bin") :
                snprintf(path + stem_length, sizeof(path) - stem_length, "-%u.bin", copy);
            if (length < 0 || static_cast<size_t>(length) >= sizeof(path) - stem_length)
                return fail("dump_name_too_long");
            snprintf(temporary, sizeof(temporary), "%s.part", path);
            bool available = false;
            if (!input.session.run([&](fs::FS &fs) {
                available = !fs.exists(path) && !fs.exists(temporary);
                return true;
            })) return fail("dump_path_unavailable");
            if (!available) continue;
            // Both names were absent under the exclusive firmware lease. Even
            // a failed create may leave our own private file; clean only that.
            private_created = true;
            if (!writer.open(input.session, temporary)) return fail("dump_sd_open_failed");
            return true;
        }
        return fail("dump_name_exhausted");
    }

    bool publish() {
        if (!writer.finish(Image::FULL_SIZE)) return fail("dump_sd_finish_failed");
        if (cancelled()) return false;
        committed = input.session.mutate(temporary, path, [&](fs::FS &fs) {
            return !fs.exists(path) && fs.rename(temporary, path);
        });
        if (!committed) return fail("dump_publish_failed");
        ResmedFirmwareRepository::notify_published(path);
        portENTER_CRITICAL(&status_mux);
        snprintf(published.source_path, sizeof(published.source_path), "%s", path);
        ++published.revision;
        portEXIT_CRITICAL(&status_mux);
        return true;
    }
};

bool run_dump() {
    Input input;
    Plan plan;
    phase("Prepare dump");
    if (!acquire_firmware(input, ResmedFirmwareRepository::ROOT) || !freeze_device(plan)) return false;
    if (work.recovery && (strcmp(plan.bid, work.expected_bid) ||
        strcmp(plan.serial, work.expected_serial))) return fail("device_identity_changed");
    DumpOutput output{input};
    if (!output.prepare_name()) return false;
    input.blx = allocate_buffer(BLX_BYTES);
    input.scratch = allocate_buffer(SOURCE_CHUNK);
    if (!input.blx || !input.scratch) return fail("dump_buffer_unavailable");
    portENTER_CRITICAL(&status_mux);
    published.total = Image::FULL_SIZE;
    snprintf(published.device_bid, sizeof(published.device_bid), "%s", plan.bid);
    snprintf(published.block, sizeof(published.block), "FULL");
    ++published.revision;
    portEXIT_CRITICAL(&status_mux);
    if (!claim_uart(input, plan)) return false;
    if (work.recovery) {
        if (!recover_dump(input, plan)) return false;
    } else {
        if (!enter_bootloader() || !negotiate_baud()) return false;
        phase("Probe dump");
        if (!dump_chunk(0, input.blx, 240) && (cancelled() || !recover_dump(input, plan))) return false;
    }
    if (!dump_range(240, input.blx + 240, BLX_BYTES - 240))
        return cancelled() ? false : fail("dump_blx_read_failed");
    if (!output.open()) return false;
    phase("Dump firmware");
    portENTER_CRITICAL(&status_mux);
    published.prepared = true;
    ++published.revision;
    portEXIT_CRITICAL(&status_mux);
    DumpReader source{input, output.writer};
    Image::Reader reader{&source, Image::FULL_SIZE, DumpReader::read};
    Image::Options options;
    options.target = Image::Block::FULL;
    options.platform = plan.profile->platform;
    Image::Result image;
    const bool inspected = Image::inspect(reader, image, options, input.scratch, SOURCE_CHUNK);
    portENTER_CRITICAL(&status_mux);
    published.image = image;
    ++published.revision;
    portEXIT_CRITICAL(&status_mux);
    if (!inspected || source.written != Image::FULL_SIZE)
        return cancelled() ? false : fail("dump_read_failed");
    // A dump preserves device evidence, including CRC warnings. No re-read or
    // extra full-image BID comparison is needed for this captured byte stream.
    return output.publish();
}
#else
bool run_dump() { return fail("sd_unsupported"); }
#endif

void finish(bool success) {
    portENTER_CRITICAL(&status_mux);
    const bool waiting = work.waiting && !cancel_requested.load() && !published.error[0];
    published.active = false;
    published.cancel_allowed = waiting;
    published.confirmation_required = waiting;
    published.result = waiting ? Result::None : success ? Result::Succeeded :
        (!programming && cancel_requested.load() && !published.error[0])
        ? Result::Cancelled : Result::Failed;
    if (published.result == Result::Failed && !published.error[0])
        snprintf(published.error, sizeof(published.error), "install_failed");
    snprintf(published.phase, sizeof(published.phase), "%s", waiting ? "Confirmation" : success ? "Complete" :
        published.result == Result::Cancelled ? "Cancelled" : "Error");
    ++published.revision;
    active.store(false);
    portEXIT_CRITICAL(&status_mux);
}

bool restore_uart() {
    if (Arbiter::get_state() != SYS_OTA_AIRSENSE) return true;
    Arbiter::set_baud(57600);
    if (work.bootloader_entered && !work.app_confirmed) {
        phase("Restore application");
        for (unsigned attempt = 0; attempt < 3 && !work.app_confirmed; ++attempt) {
            if (!work.quiet_done) vTaskDelay(pdMS_TO_TICKS(3000));
            work.quiet_done = false;
            uint32_t state = UINT32_MAX;
            if (!raw_hex("G S #BLS", state) || state != 0) continue;
            char bid[32], serial[24];
            if (!raw_text("G S #BID", bid, sizeof(bid)) ||
                !raw_text("G S #SRN", serial, sizeof(serial))) continue;
            if (strcmp(bid, work.expected_bid) || strcmp(serial, work.expected_serial)) {
                fail("device_identity_changed");
                break;
            }
            work.app_confirmed = true;
        }
        if (!work.app_confirmed) {
            work.waiting = false;
            fail("device_did_not_return_to_application");
            Arbiter::set_state(SYS_BOOTLOADER);
            return false;
        }
    }
    Arbiter::set_state(SYS_IDLE);
    AirSenseState::request_refresh();
    return true;
}

void flash_task(void *) {
    Status status;
    get_status(status);
    Log::logf(CAT_OTA, LOG_INFO, "ResMed %s started id=%lu source=%s\n",
        work.dump ? "dump" : "install", static_cast<unsigned long>(status.id), status.source_path);
    bool success = work.dump ? run_dump() : run_install();  // Owners unwind here.
    if (!restore_uart()) success = false;
    if (work.staged.token) FileUpload::release_source(work.staged.token);
    finish(success);
    get_status(status);
    Log::logf(CAT_OTA, status.error[0] ? LOG_ERROR : LOG_INFO, "ResMed %s %s id=%lu bytes=%u: %s\n",
        work.dump ? "dump" : "install", status.phase, static_cast<unsigned long>(status.id),
        unsigned(status.sent), status.error);
    // No timer infers our lifetime; this is the only normal reservation release.
    OtaManager::end_resmed_flash();
    vTaskDelete(nullptr);
}

bool launch(const Work &next, const char **error, uint32_t resumed_id = 0) {
    // Reservation, params and visible activity are initialized before scheduling.
    portENTER_CRITICAL(&status_mux);
    if (resumed_id && (published.id != resumed_id || !published.confirmation_required)) {
        portEXIT_CRITICAL(&status_mux);
        OtaManager::end_resmed_flash();
        if (error) *error = "stale_dump_confirmation";
        return false;
    }
    work = next;
    const uint32_t previous_revision = published.revision;
    uint32_t id = resumed_id ? resumed_id : published.id + 1;
    if (!id) ++id;
    published = {};
    published.id = id;
    published.revision = previous_revision + 1;
    published.active = true;
    published.result = Result::Running;
    published.source = next.source;
    published.upload_id = next.request.upload_id;
    snprintf(published.source_path, sizeof(published.source_path), "%s", next.request.path);
    snprintf(published.block, sizeof(published.block), "%s", next.request.block);
    snprintf(published.phase, sizeof(published.phase), "Preparing");
    published.cancel_allowed = true;
    cancel_requested.store(false);
    programming = false;
    active.store(true);
    portEXIT_CRITICAL(&status_mux);
    if (xTaskCreatePinnedToCore(flash_task, "resmed_ota", FLASH_TASK_STACK,
                                nullptr, 3, nullptr, 1) != pdPASS) {
        fail("worker_create_failed");
        if (next.staged.token) FileUpload::release_source(next.staged.token);
        finish(false);
        OtaManager::end_resmed_flash();
        if (error) *error = "worker_create_failed";
        return false;
    }
    return true;
}

}  // namespace

bool start_install(const InstallRequest &request, const char **error) {
    if (error) *error = nullptr;
    const auto reject = [error](const char *reason) {
        if (error) *error = reason;
        return false;
    };
    if (!memchr(request.path, 0, sizeof(request.path)) ||
        !memchr(request.block, 0, sizeof(request.block)) ||
        (bool(request.upload_id) == bool(request.path[0]))) return reject("invalid_install_request");
    if (request.block[0] && Image::parse_block(request.block) == Image::Block::Unknown)
        return reject("unknown_block");
    if (request.path[0] && !ResmedFirmwareRepository::valid_path(request.path))
        return reject("invalid_repository_path");
    Work next;
    next.request = request;
    next.source = Source::Repository;
    bool claimed = false;
    if (request.upload_id) {
        FileUpload::Status upload;
        if (!FileUpload::copy_status(request.upload_id, upload) ||
            upload.state != FileUpload::State::Ready || !upload.source_ready)
            return reject("upload_not_ready");
        if (upload.destination == FileUpload::Destination::Staging) {
            if (!FileUpload::claim_source(request.upload_id, next.staged, error)) return false;
            next.source = Source::Staging;
            claimed = true;
        } else {
            if (!ResmedFirmwareRepository::valid_path(upload.path)) return reject("invalid_repository_path");
            snprintf(next.request.path, sizeof(next.request.path), "%s", upload.path);
        }
    }
    if (!claimed && !OtaManager::begin_resmed_flash(error)) return false;
    return launch(next, error);
}

bool start_dump(const char **error) {
    if (error) *error = nullptr;
#if AB_STORAGE_HAS_SDCARD
    if (!SdStorage::mounted()) {
        if (error) *error = "sd_unavailable";
        return false;
    }
    if (!OtaManager::begin_resmed_flash(error)) return false;
    Work next;
    next.dump = true;
    return launch(next, error);
#else
    if (error) *error = "sd_unsupported";
    return false;
#endif
}

bool confirm_dump(uint32_t id, const char **error) {
    if (error) *error = nullptr;
    if (!OtaManager::begin_resmed_flash(error)) return false;
    portENTER_CRITICAL(&status_mux);
    const bool accepted = id && published.id == id && !published.active &&
        published.confirmation_required && published.recovery_available && work.dump;
    Work next;
    if (accepted) {
        next = work;
        next.waiting = false;
        next.recovery = true;
        next.bootloader_entered = next.app_confirmed = next.quiet_done = false;
    }
    portEXIT_CRITICAL(&status_mux);
    if (!accepted) {
        OtaManager::end_resmed_flash();
        if (error) *error = "stale_dump_confirmation";
        return false;
    }
    return launch(next, error, id);
}

void cancel() {
    portENTER_CRITICAL(&status_mux);
    if (published.active && !cancel_requested.exchange(true)) {
        published.cancellation_requested = true;
        ++published.revision;
    } else if (!published.active && published.confirmation_required) {
        published.confirmation_required = false;
        published.cancel_allowed = false;
        published.result = Result::Cancelled;
        snprintf(published.phase, sizeof(published.phase), "Cancelled");
        ++published.revision;
    }
    portEXIT_CRITICAL(&status_mux);
}

bool is_active() { return active.load(); }

void poll() {
    // Admission facts are cached-owner reads, never UART or filesystem work.
    FileUpload::Status upload;
    FileUpload::get_status(upload);
    const bool staged = upload.state == FileUpload::State::Ready &&
        upload.destination == FileUpload::Destination::Staging && upload.source_ready;
    const char *blocked = OtaManager::resmed_flash_blocked(staged);
    const char *dump_blocked = "sd_unsupported";
#if AB_STORAGE_HAS_SDCARD
    dump_blocked = SdStorage::mounted() ? OtaManager::resmed_flash_blocked() : "sd_unavailable";
#endif
    OtaManager::Status ota;
    const bool can_upload = OtaManager::get_status(ota) && !ota.upload_blocked;
    portENTER_CRITICAL(&status_mux);
    const bool allowed = !published.active && !blocked;
    const bool upload_allowed = !published.active && can_upload;
    const bool dump_allowed = !published.active && !dump_blocked;
    const char *reason = published.active ? "resmed_ota_active" : blocked ? blocked : "";
    const char *dump_reason = published.active ? "resmed_ota_active" : dump_blocked ? dump_blocked : "";
    if (published.install_allowed != allowed || published.upload_allowed != upload_allowed ||
        published.dump_allowed != dump_allowed || strcmp(published.blocked, reason) ||
        strcmp(published.dump_blocked, dump_reason)) {
        published.install_allowed = allowed;
        published.upload_allowed = upload_allowed;
        published.dump_allowed = dump_allowed;
        snprintf(published.blocked, sizeof(published.blocked), "%s", reason);
        snprintf(published.dump_blocked, sizeof(published.dump_blocked), "%s", dump_reason);
        ++published.revision;
    }
    portEXIT_CRITICAL(&status_mux);
}

void get_status(Status &out) {
    portENTER_CRITICAL(&status_mux);
    out = published;
    portEXIT_CRITICAL(&status_mux);
}

uint32_t revision() {
    portENTER_CRITICAL(&status_mux);
    const uint32_t value = published.revision;
    portEXIT_CRITICAL(&status_mux);
    return value;
}

const esp_partition_t *get_staging_partition() {
    const auto *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, "resmed");
    return partition ? partition : esp_ota_get_next_update_partition(esp_ota_get_running_partition());
}

}  // namespace ResmedOta
