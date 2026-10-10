#pragma once

#include "sdkconfig.h"

#if !CONFIG_IDF_TARGET_ESP32S3 || !AB_USB_MSC_ENABLED || ARDUINO_USB_MODE != 1
#error "The local TinyUSB configuration requires esp32-s3 SD handoff with Serial/JTAG"
#endif

#define CFG_TUSB_MCU OPT_MCU_ESP32S3
#define CFG_TUSB_OS OPT_OS_FREERTOS
#define CFG_TUSB_DEBUG 0
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN TU_ATTR_ALIGNED(4)

#define CFG_TUD_ENABLED 1
#define CFG_TUH_ENABLED 0
#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE
#define CFG_TUD_MAX_SPEED OPT_MODE_FULL_SPEED
#define BOARD_TUD_RHPORT 0
#define BOARD_TUD_MAX_SPEED CFG_TUD_MAX_SPEED

#define CFG_TUD_CDC 0
#define CFG_TUD_MSC 1
#define CFG_TUD_MSC_BUFSIZE CONFIG_TINYUSB_MSC_BUFSIZE

#define CFG_TUD_ENDPOINT0_SIZE 64
// Return to our task after each event, including the shutdown notification.
#define CFG_TUD_TASK_EVENTS_PER_RUN 1

// All other device classes use TinyUSB's disabled defaults.
