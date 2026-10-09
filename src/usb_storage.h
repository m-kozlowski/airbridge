#pragma once

#include <stdint.h>

namespace UsbStorage {

// Registers MSC before Arduino starts TinyUSB; init prepares the bounded DMA buffer.
bool init();
void poll();
void expose(uint32_t sectors);
void withdraw();

}  // namespace UsbStorage
