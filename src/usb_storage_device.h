#pragma once

#include <esp_err.h>

namespace UsbStorageDevice {

// Restore Serial/JTAG before Serial.begin(), also after OTA from USB-OTG.
void begin();
void poll();
bool serial_available();

// SD worker only. Stop after sector I/O and its completion have been queued.
esp_err_t start();
esp_err_t stop();

}  // namespace UsbStorageDevice
