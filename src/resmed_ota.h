#pragma once
#include <Arduino.h>
#include <esp_partition.h>
#include "resmed_firmware_image.h"


namespace ResmedOta {
    enum class Source : uint8_t { None, Repository, Staging };
    enum class Result : uint8_t { None, Running, Succeeded, Failed, Cancelled };

    struct InstallRequest {
        // Exactly one source. Empty block means inspector auto-detection.
        uint32_t upload_id = 0;
        char path[256] = {};
        char block[8] = {};
        bool flash_blx = false;
        // Explicit manual override of CRC/BID/patch warnings, not I/O/bounds.
        bool force_blx = false;
        bool preserve_settings = true;
    };

    struct Status {
        uint32_t revision = 0;
        uint32_t id = 0;
        bool active = false;
        Result result = Result::None;
        char phase[32] = "Idle";
        size_t sent = 0;
        size_t total = 0;
        char error[128] = {};
        Source source = Source::None;
        uint32_t upload_id = 0;
        char source_path[256] = {};
        char block[8] = {};
        char device_bid[32] = {};
        bool prepared = false;
        bool cancellation_requested = false;
        bool cancel_allowed = false;
        bool upload_allowed = false;
        bool install_allowed = false;
        bool dump_allowed = false;
        char blocked[48] = {};
        char dump_blocked[48] = {};
        bool recovery_available = false;
        char recovery_path[256] = {};
        bool confirmation_required = false;
        ResmedFirmwareImage::Result image;
    };

    // Callback-safe admission/copy only. The owner reserves synchronously,
    // then validates/prepares/programs on its worker. No hardware I/O here.
    bool start_install(const InstallRequest &request, const char **error = nullptr);
    // Dump probes the live extension first. Recovery, when available, waits for
    // explicit consent bound to this operation ID; neither callback does I/O.
    bool start_dump(const char **error = nullptr);
    bool confirm_dump(uint32_t id, const char **error = nullptr);
    // Main-loop publication of cached admission facts. No UART/storage I/O.
    void poll();
    // Copy only; does not refresh admission or join other owners.
    void get_status(Status &out);
    uint32_t revision();

    const esp_partition_t* get_staging_partition();

    void cancel();

    bool is_active();
}
