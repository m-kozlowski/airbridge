#pragma once
#include <Arduino.h>
#include <esp_partition.h>

typedef void (*flash_progress_cb)(size_t sent, size_t total, const char *phase);

namespace ResmedOta {
    const esp_partition_t* get_staging_partition();

    // block: "FULL", "CMX", "CDX", "CCX", "BLX" (nullptr = auto-detect)
    //   FULL = 1MB image containing BLX+CMX. Flashes CMX by default,
    //          BLX only if flash_blx is set (with BID safety check).
    // fw_size: actual firmware size in bytes
    // flash_blx: for FULL images, also flash the BLX region
    // force_blx: skip BID version check for BLX flashing
    void start_flash(const char *block, size_t fw_size,
                     bool flash_blx, bool force_blx);

    void cancel();

    bool is_active();
    const char* get_phase();
    size_t get_sent();
    size_t get_total();
    const char* last_error();

    // Returns "FULL", "CMX", "CDX", "CCX", "BLX", or nullptr
    const char* detect_block(size_t fw_size);
}
