#pragma once


// UART1 -> AirSense USART3
#ifndef AB_AS10_TX_GPIO
#define AB_AS10_TX_GPIO 26
#endif

#ifndef AB_AS10_RX_GPIO
#define AB_AS10_RX_GPIO 36      // input-only on M5Stamp Pico
#endif

#ifndef AB_LED_GPIO
#define AB_LED_GPIO 27
#endif

#ifndef AB_STORAGE_SDMMC_ENABLED
#define AB_STORAGE_SDMMC_ENABLED 0
#endif

#ifndef AB_SDMMC_WIDTH
#define AB_SDMMC_WIDTH 4
#endif

#ifndef AB_SDMMC_FREQ_KHZ
#define AB_SDMMC_FREQ_KHZ 20000
#endif

#ifndef AB_SDMMC_CLK_GPIO
#define AB_SDMMC_CLK_GPIO -1
#endif

#ifndef AB_SDMMC_CMD_GPIO
#define AB_SDMMC_CMD_GPIO -1
#endif

#ifndef AB_SDMMC_D0_GPIO
#define AB_SDMMC_D0_GPIO -1
#endif

#ifndef AB_SDMMC_D1_GPIO
#define AB_SDMMC_D1_GPIO -1
#endif

#ifndef AB_SDMMC_D2_GPIO
#define AB_SDMMC_D2_GPIO -1
#endif

#ifndef AB_SDMMC_D3_GPIO
#define AB_SDMMC_D3_GPIO -1
#endif

#ifndef AB_STORAGE_HAS_SDCARD
#define AB_STORAGE_HAS_SDCARD (AB_STORAGE_SDMMC_ENABLED != 0)
#endif

#ifndef AB_USB_MSC_ENABLED
#define AB_USB_MSC_ENABLED 0
#endif

// Backward-compatible aliases used by the current firmware.
#define PIN_AS10_TX     AB_AS10_TX_GPIO
#define PIN_AS10_RX     AB_AS10_RX_GPIO
#define PIN_LED         AB_LED_GPIO

// Phase 2: MITM modem interception
// #define MODEM_UART_NUM  2
// #define PIN_MODEM_TX    18
// #define PIN_MODEM_RX    19


#define DEFAULT_HOSTNAME    "airbridge"

#ifndef AB_OTA_RELEASE_TARGET
#if defined(AB_BOARD_WAVESHARE_ESP32_S3_GEEK)
#define AB_OTA_RELEASE_TARGET "waveshare-esp32-s3-geek"
#elif defined(AB_BOARD_WROOM_S3)
#define AB_OTA_RELEASE_TARGET "wroom-s3-n16r8-sdmmc4"
#elif defined(AB_BOARD_XIAO_ESP32S3_PLUS) && AB_STORAGE_SDMMC_ENABLED
#define AB_OTA_RELEASE_TARGET "xiao-esp32s3-plus-sdmmc4"
#elif defined(AB_BOARD_XIAO_ESP32S3_PLUS)
#define AB_OTA_RELEASE_TARGET "xiao-esp32s3-plus"
#else
#define AB_OTA_RELEASE_TARGET "m5stamp-pico"
#endif
#endif

#ifndef AB_DEFAULT_UPDATE_URL
#define AB_DEFAULT_UPDATE_URL                                                \
    "https://github.com/m-kozlowski/airbridge/releases/latest/download/"   \
    "airbridge-release.json"
#endif
