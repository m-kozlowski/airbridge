#pragma once
#include <Arduino.h>
#include <esp_partition.h>

typedef void (*flash_progress_cb)(size_t sent, size_t total, const char *phase);

enum blx_patch_t {
    BLX_PATCH_NONE,         // stock bootloader, integrity checks enabled
    BLX_PATCH_A_DANGEROUS,  // 0xF0 patch, bricks serial flash
    BLX_PATCH_B_SAFE,       // safe method
};

struct fw_verify_result_t {
    bool has_blx;
    bool bid_ok;
    char bid[32];

    bool has_ccx, has_cdx;
    bool blx_crc_ok;
    bool ccx_crc_ok;
    bool cdx_crc_ok;

    blx_patch_t blx_patch;
};

namespace ResmedOta {
    const esp_partition_t* get_staging_partition();

    fw_verify_result_t verify_image(const esp_partition_t *part, size_t fw_size);

    // block: "FULL", "CMX", "CDX", "CCX", "BLX" (nullptr = auto-detect)
    void start_flash(const char *block, size_t fw_size,
                     bool flash_blx, bool force_blx);

    void cancel();

    bool is_active();
    const char* get_phase();
    size_t get_sent();
    size_t get_total();
    const char* last_error();

    const char* detect_block(size_t fw_size);
}
