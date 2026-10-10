#include "usb_storage_device.h"
#include "board.h"

#if AB_USB_MSC_ENABLED
#include <Arduino.h>
#include <freertos/semphr.h>
#include <esp_mac.h>
#include <esp_private/periph_ctrl.h>
#include <esp_private/usb_phy.h>
#include <hal/usb_serial_jtag_ll.h>
#include <soc/periph_defs.h>
#include <tusb.h>
#include <device/usbd_pvt.h>
#include <device/dcd.h>

namespace UsbStorageDevice {
namespace {

bool serial_enabled = true;
StaticSemaphore_t ready_storage, done_storage;
SemaphoreHandle_t ready = nullptr, done = nullptr;
TaskHandle_t task = nullptr;
esp_err_t start_result = ESP_OK, stop_result = ESP_OK;
bool stopping = false;  // USB task only.
char serial_number[13];

#if defined(AB_BOARD_WROOM_S3)
StaticSemaphore_t vbus_mutex_storage;
SemaphoreHandle_t vbus_mutex = nullptr;
int previous_vbus = -1;
#endif

void request_stop(void *) { stopping = true; }

void run(void *) {
    __atomic_store_n(&serial_enabled, false, __ATOMIC_RELEASE);
    usb_serial_jtag_ll_phy_enable_pad(false);
    vTaskDelay(pdMS_TO_TICKS(100));

    periph_module_enable(PERIPH_USB_MODULE);
    periph_module_reset(PERIPH_USB_MODULE);
    usb_phy_handle_t phy = nullptr;
    usb_phy_config_t config = {};
    config.controller = USB_PHY_CTRL_OTG;
    config.target = USB_PHY_TARGET_INT;
    config.otg_mode = USB_OTG_MODE_DEVICE;
    config.otg_speed = USB_PHY_SPEED_FULL;
    start_result = usb_new_phy(&config, &phy);
    const tusb_rhport_init_t device = {TUSB_ROLE_DEVICE, TUSB_SPEED_FULL};
    if (start_result == ESP_OK && !tusb_init(0, &device)) start_result = ESP_FAIL;
#if defined(AB_BOARD_WROOM_S3)
    if (start_result == ESP_OK && !digitalRead(14)) tud_disconnect();
    previous_vbus = -1;
#endif
    xSemaphoreGive(ready);

    while (start_result == ESP_OK && !stopping) tud_task();

    // This task owns interrupt allocation and teardown on the same CPU.
    stop_result = ESP_OK;
    if (tud_inited() && !tud_deinit(0)) stop_result = ESP_FAIL;
    if (phy) {
        const esp_err_t result = usb_del_phy(phy);
        if (result != ESP_OK) stop_result = result;
    }
    periph_module_disable(PERIPH_USB_MODULE);
    vTaskDelay(pdMS_TO_TICKS(100));

    usb_serial_jtag_ll_phy_enable_external(false);
#if defined(AB_BOARD_WROOM_S3)
    usb_serial_jtag_ll_phy_enable_pad(digitalRead(14) == HIGH);
    previous_vbus = -1;
#else
    usb_serial_jtag_ll_phy_enable_pad(true);
#endif
    __atomic_store_n(&serial_enabled, true, __ATOMIC_RELEASE);
    xSemaphoreGive(done);
    vTaskDelete(nullptr);
}

}  // namespace

void begin() {
    usb_serial_jtag_ll_phy_enable_external(false);
#if defined(AB_BOARD_WROOM_S3)
    pinMode(14, INPUT);
    vbus_mutex = xSemaphoreCreateMutexStatic(&vbus_mutex_storage);
#endif
}

void poll() {
#if defined(AB_BOARD_WROOM_S3)
    // Do not touch OTG or its event queue while the worker changes USB owners.
    if (!vbus_mutex || xSemaphoreTake(vbus_mutex, 0) != pdTRUE) return;
    const int present = digitalRead(14);
    if (present != previous_vbus) {
        previous_vbus = present;
        if (serial_available()) usb_serial_jtag_ll_phy_enable_pad(present == HIGH);
        else if (present) tud_connect();
        else {
            // Explicit VBUS loss must invalidate queued MSC I/O just like an IRQ.
            dcd_event_bus_signal(0, DCD_EVENT_UNPLUGGED, false);
            tud_disconnect();
        }
    }
    xSemaphoreGive(vbus_mutex);
#endif
}

esp_err_t start() {
    if (task || stop_result != ESP_OK) return ESP_ERR_INVALID_STATE;
    if (!ready) ready = xSemaphoreCreateBinaryStatic(&ready_storage);
    if (!done) done = xSemaphoreCreateBinaryStatic(&done_storage);

    uint8_t mac[6];
    const esp_err_t error = esp_read_mac(mac, ESP_MAC_WIFI_STA);
    if (error != ESP_OK) return error;
    snprintf(serial_number, sizeof(serial_number), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    stopping = false;
#if defined(AB_BOARD_WROOM_S3)
    xSemaphoreTake(vbus_mutex, portMAX_DELAY);
#endif
    if (xTaskCreatePinnedToCore(run, "ab_usb", 4096, nullptr,
                              configMAX_PRIORITIES - 1, &task,
                              xPortGetCoreID()) != pdPASS) {
        task = nullptr;
        start_result = ESP_ERR_NO_MEM;
    } else {
        xSemaphoreTake(ready, portMAX_DELAY);
        if (start_result != ESP_OK) {
            xSemaphoreTake(done, portMAX_DELAY);
            task = nullptr;
        }
    }
#if defined(AB_BOARD_WROOM_S3)
    xSemaphoreGive(vbus_mutex);
#endif
    return start_result;
}

esp_err_t stop() {
    if (!task) return stop_result;
#if defined(AB_BOARD_WROOM_S3)
    xSemaphoreTake(vbus_mutex, portMAX_DELAY);
#endif
    // FIFO after the last SD completion; never delete a task inside a callback.
    usbd_defer_func(request_stop, nullptr, false);
    xSemaphoreTake(done, portMAX_DELAY);
    task = nullptr;
#if defined(AB_BOARD_WROOM_S3)
    xSemaphoreGive(vbus_mutex);
#endif
    return stop_result;
}

bool serial_available() { return __atomic_load_n(&serial_enabled, __ATOMIC_ACQUIRE); }

}  // namespace UsbStorageDevice

extern "C" {

// The pinned DWC2 port has no deinit hook. tud_deinit already disconnected the
// endpoints and freed the interrupt; reset the controller before freeing queues.
bool dcd_deinit(uint8_t) {
    periph_module_reset(PERIPH_USB_MODULE);
    return true;
}

const uint8_t *tud_descriptor_device_cb() {
    static const tusb_desc_device_t descriptor = {
        sizeof(tusb_desc_device_t), TUSB_DESC_DEVICE, 0x0200,
        0, 0, 0, CFG_TUD_ENDPOINT0_SIZE, 0x303a, 0x1001, 0x0200,
        1, 2, 3, 1
    };
    return reinterpret_cast<const uint8_t *>(&descriptor);
}

const uint8_t *tud_descriptor_configuration_cb(uint8_t) {
    static const uint8_t descriptor[] = {
        TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN,
                              TUSB_DESC_CONFIG_ATT_SELF_POWERED, 100),
        TUD_MSC_DESCRIPTOR(0, 0, 0x01, 0x81, 64)
    };
    return descriptor;
}

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t) {
    static uint16_t descriptor[32];
    if (index == 0) {
        descriptor[0] = (TUSB_DESC_STRING << 8) | 4;
        descriptor[1] = 0x0409;
        return descriptor;
    }
    const char *text = nullptr;
    switch (index) {
    case 1: text = "AirBridge"; break;
    case 2: text = "AirBridge SD"; break;
    case 3: text = UsbStorageDevice::serial_number; break;
    default: return nullptr;
    }
    const size_t length = strlen(text);
    descriptor[0] = (TUSB_DESC_STRING << 8) | (2 * length + 2);
    for (size_t i = 0; i < length; ++i) descriptor[i + 1] = text[i];
    return descriptor;
}

}  // extern "C"
#else
namespace UsbStorageDevice {
void begin() {}
void poll() {}
bool serial_available() { return true; }
esp_err_t start() { return ESP_ERR_NOT_SUPPORTED; }
esp_err_t stop() { return ESP_OK; }
}  // namespace UsbStorageDevice
#endif
