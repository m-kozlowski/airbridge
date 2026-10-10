#include "usb_storage.h"
#include "board.h"

#if AB_USB_MSC_ENABLED
#include <Arduino.h>
#include <USB.h>
#include <esp_heap_caps.h>
#include <esp32-hal-tinyusb.h>
#include <device/usbd_pvt.h>
#include <device/dcd.h>
#include "sd_storage.h"

namespace {

constexpr size_t BUFFER_BYTES = CONFIG_TINYUSB_MSC_BUFSIZE;
static_assert(BUFFER_BYTES >= 512 && BUFFER_BYTES % 512 == 0, "MSC buffer must hold whole SD sectors");
uint8_t *dma_buffer = nullptr;
uint32_t sector_count = 0;
bool media_present = false;
bool registered = false;
bool initialized = false;
bool prevented = false;
uint32_t bus_generation = 0;
uint32_t media_generation = 0;
TaskHandle_t inline_completion_task = nullptr;

// One BOT command is in flight. Only the USB task accesses the TinyUSB buffer;
// the SD worker uses a separate DMA buffer, also across reset/disconnect.
struct {
    bool busy = false;
    bool write = false;
    bool success = false;
    uint32_t bus = 0, media = 0, size = 0;
    void *buffer = nullptr;
} transfer;

uint16_t descriptor(uint8_t *out, uint8_t *interface) {
    const uint8_t endpoint = tinyusb_get_free_duplex_endpoint();
    if (!endpoint) return 0;
    const uint8_t name = tinyusb_add_string_descriptor("AirBridge SD");
    const uint8_t bytes[] = {
        TUD_MSC_DESCRIPTOR(*interface, name, endpoint, uint8_t(0x80 | endpoint), CFG_TUD_ENDOINT_SIZE)
    };
    ++*interface;
    memcpy(out, bytes, sizeof(bytes));
    return sizeof(bytes);
}

struct Register {
    Register() {
        registered = tinyusb_enable_interface(USB_INTERFACE_MSC, TUD_MSC_DESC_LEN, descriptor) == ESP_OK;
    }
} registration;

bool ready() {
    static uint32_t seen_media = 0;
    const uint32_t current = __atomic_load_n(&media_generation, __ATOMIC_ACQUIRE);
    if (seen_media != current) {
        seen_media = current;
        prevented = false;
    }
    if (__atomic_load_n(&media_present, __ATOMIC_ACQUIRE)) return true;
    tud_msc_set_sense(0, SCSI_SENSE_NOT_READY, 0x3a, 0);
    return false;
}

void finish_io(void *) {
    if (transfer.bus == __atomic_load_n(&bus_generation, __ATOMIC_ACQUIRE) &&
        transfer.media == __atomic_load_n(&media_generation, __ATOMIC_ACQUIRE)) {
        if (transfer.success && !transfer.write) memcpy(transfer.buffer, dma_buffer, transfer.size);
        if (!transfer.success)
            tud_msc_set_sense(0, SCSI_SENSE_MEDIUM_ERROR, transfer.write ? 0x0c : 0x11, 0);
        // Already in the USB task. Complete before another reset/CBW can run,
        // without blocking this task on a second enqueue to its own full queue.
        __atomic_store_n(&inline_completion_task, xTaskGetCurrentTaskHandle(), __ATOMIC_RELEASE);
        tud_msc_async_io_done(transfer.success ? int32_t(transfer.size) : TUD_MSC_RET_ERROR, false);
        __atomic_store_n(&inline_completion_task, nullptr, __ATOMIC_RELEASE);
    }
    transfer.buffer = nullptr;
    transfer.busy = false;
}

void io_done(bool success) {
    transfer.success = success;
    usbd_defer_func(finish_io, nullptr, false);
}

int32_t submit(bool write, uint32_t sector, uint32_t offset, void *buffer, uint32_t size) {
    if (!ready()) return TUD_MSC_RET_ERROR;
    if (transfer.busy) return TUD_MSC_RET_BUSY;
    if (!dma_buffer || offset % 512 || !size || size > BUFFER_BYTES || size % 512 ||
        uint64_t(sector) + offset / 512 + size / 512 > __atomic_load_n(&sector_count, __ATOMIC_ACQUIRE)) {
        tud_msc_set_sense(0, SCSI_SENSE_ILLEGAL_REQUEST, 0x21, 0);
        return TUD_MSC_RET_ERROR;
    }
    transfer.busy = true;
    transfer.write = write;
    transfer.size = size;
    transfer.buffer = buffer;
    transfer.bus = __atomic_load_n(&bus_generation, __ATOMIC_ACQUIRE);
    transfer.media = __atomic_load_n(&media_generation, __ATOMIC_ACQUIRE);
    if (write) memcpy(dma_buffer, buffer, size);
    if (!SdStorage::usb_transfer(write, sector + offset / 512, dma_buffer, size, io_done)) {
        transfer.buffer = nullptr;
        transfer.busy = false;
        return TUD_MSC_RET_ERROR;
    }
    return TUD_MSC_RET_ASYNC;
}

void usb_event(void *, esp_event_base_t, int32_t event, void *) {
    if (event == ARDUINO_USB_STARTED_EVENT || event == ARDUINO_USB_STOPPED_EVENT)
        SdStorage::usb_host_changed(event == ARDUINO_USB_STARTED_EVENT);
    // Suspend is not eject: a sleeping host still owns its filesystem cache.
}

}  // namespace

extern "C" {

void __real_usbd_defer_func(osal_task_func_t function, void *argument, bool in_isr);

void __wrap_usbd_defer_func(osal_task_func_t function, void *argument, bool in_isr) {
    if (!in_isr && __atomic_load_n(&inline_completion_task, __ATOMIC_ACQUIRE) == xTaskGetCurrentTaskHandle()) {
        __atomic_store_n(&inline_completion_task, nullptr, __ATOMIC_RELEASE);
        function(argument);
        return;
    }
    __real_usbd_defer_func(function, argument, in_isr);
}

void __real_mscd_reset(uint8_t rhport);
bool __real_mscd_control_xfer_cb(uint8_t rhport, uint8_t stage,
                               const tusb_control_request_t *request);

void __wrap_mscd_reset(uint8_t rhport) {
    __atomic_add_fetch(&bus_generation, 1, __ATOMIC_ACQ_REL);
    prevented = false;
    __real_mscd_reset(rhport);
}

bool __wrap_mscd_control_xfer_cb(uint8_t rhport, uint8_t stage,
                               const tusb_control_request_t *request) {
    const bool result = __real_mscd_control_xfer_cb(rhport, stage, request);
    if (result && stage == CONTROL_STAGE_SETUP &&
        request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS &&
        request->bRequest == MSC_REQ_RESET) {
        // BOT reset does not invoke mscd_reset, but also invalidates async I/O.
        __atomic_add_fetch(&bus_generation, 1, __ATOMIC_ACQ_REL);
        prevented = false;
    }
    return result;
}

void IRAM_ATTR tud_event_hook_cb(uint8_t, uint32_t event, bool) {
    if (event == DCD_EVENT_BUS_RESET || event == DCD_EVENT_UNPLUGGED)
        __atomic_add_fetch(&bus_generation, 1, __ATOMIC_ACQ_REL);
}

uint8_t tud_msc_get_maxlun_cb() { return 1; }

void tud_msc_inquiry_cb(uint8_t, uint8_t vendor[8], uint8_t product[16], uint8_t revision[4]) {
    memcpy(vendor, "AirBridg", 8);
    memcpy(product, "SD card         ", 16);
    memcpy(revision, "1.0 ", 4);
}

bool tud_msc_test_unit_ready_cb(uint8_t) { return ready(); }
bool tud_msc_is_writable_cb(uint8_t) { return true; }

void tud_msc_capacity_cb(uint8_t, uint32_t *blocks, uint16_t *size) {
    *blocks = ready() ? __atomic_load_n(&sector_count, __ATOMIC_ACQUIRE) : 0;
    *size = 512;
}

int32_t tud_msc_read10_cb(uint8_t, uint32_t sector, uint32_t offset, void *buffer, uint32_t size) {
    return submit(false, sector, offset, buffer, size);
}

int32_t tud_msc_write10_cb(uint8_t, uint32_t sector, uint32_t offset, uint8_t *buffer, uint32_t size) {
    return submit(true, sector, offset, buffer, size);
}

bool tud_msc_prevent_allow_medium_removal_cb(uint8_t, uint8_t prevent, uint8_t) {
    prevented = prevent != 0;
    return true;
}

bool tud_msc_start_stop_cb(uint8_t, uint8_t, bool start, bool eject) {
    if (!eject || start || !ready()) return true;
    if (prevented || transfer.busy) {
        tud_msc_set_sense(0, SCSI_SENSE_ILLEGAL_REQUEST, 0x53, 2);
        return false;
    }
    UsbStorage::withdraw();
    SdStorage::usb_ejected();
    return true;
}

int32_t tud_msc_scsi_cb(uint8_t, const uint8_t command[16], void *, uint16_t) {
    if (!ready()) return TUD_MSC_RET_ERROR;
    // SYNCHRONIZE CACHE (10): WRITE10 is acknowledged only after SD completion.
    if (command[0] == 0x35 && !transfer.busy) return 0;
    tud_msc_set_sense(0, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0);
    return TUD_MSC_RET_ERROR;
}

}  // extern "C"

namespace UsbStorage {

bool init() {
    if (initialized) return true;
    if (!registered) return false;
    // SDMMC DMA must not borrow TinyUSB's buffer or use external RAM.
    dma_buffer = static_cast<uint8_t *>(heap_caps_malloc(BUFFER_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
    if (!dma_buffer) return false;
    USB.onEvent(usb_event);
    SdStorage::usb_host_changed(tud_mounted());
#if defined(AB_BOARD_WROOM_S3)
    pinMode(14, INPUT);
#endif
    initialized = true;
    return true;
}

void poll() {
#if defined(AB_BOARD_WROOM_S3)
    if (!initialized) return;
    static int previous = -1;
    const int present = digitalRead(14);
    if (present == previous) return;
    previous = present;
    if (present) tud_connect();
    else {
        tud_disconnect();
        __atomic_add_fetch(&bus_generation, 1, __ATOMIC_ACQ_REL);
        SdStorage::usb_host_changed(false);
    }
#endif
}

void expose(uint32_t sectors) {
    USBSerial.enableReboot(false);
    __atomic_store_n(&sector_count, sectors, __ATOMIC_RELEASE);
    __atomic_add_fetch(&media_generation, 1, __ATOMIC_ACQ_REL);
    __atomic_store_n(&media_present, true, __ATOMIC_RELEASE);
}

void withdraw() {
    __atomic_store_n(&media_present, false, __ATOMIC_RELEASE);
    __atomic_add_fetch(&media_generation, 1, __ATOMIC_ACQ_REL);
    USBSerial.enableReboot(true);
}

}  // namespace UsbStorage

#else
namespace UsbStorage {
bool init() { return false; }
void poll() {}
void expose(uint32_t) {}
void withdraw() {}
}  // namespace UsbStorage
#endif
