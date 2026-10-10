#pragma once

#include <stdint.h>
#include <esp_err.h>

namespace UsbStorage {

// Call before Serial.begin(); local mode uses hardware Serial/JTAG.
void begin();
void poll();
bool serial_available();

// SD worker only. Admission must be closed and sector I/O drained before stop.
esp_err_t expose(uint32_t sectors);
esp_err_t stop();
void withdraw();

}  // namespace UsbStorage
