#pragma once
#include <Arduino.h>
#include "ota_image_writer.h"

namespace OtaManager {
    enum class State : uint8_t {
        Disabled, Idle, Checking, Current, Available, Installing, Rebooting,
        Busy, Error,
    };

    struct Status {
        uint32_t revision;
        State state;
        const char *blocked;
        const char *upload_blocked;
        uint8_t progress;
        size_t bytes;
        size_t total_size;
        uint32_t last_check_age_ms;
        char update_version[48];
        char error[64];
    };

    void init();
    void handle();
    // Normal requests retain reboot/coalescing behavior during OTA. Prepared
    // requests require an idle lease and the running image selected for boot.
    // Preparation runs in handle(); it cannot replace any pending reboot.
    bool request_reboot(bool (*prepare)() = nullptr);

    bool request_check(const char **error = nullptr);
    bool request_install(const char **error = nullptr);
    void config_changed();
    uint32_t revision();
    bool get_status(Status &status);
    const char *state_name(State state);

    bool begin_manual_upload(const char **error = nullptr);
    void end_manual_upload(bool success, const char *error = nullptr);

    // Image I/O requires the existing manual-upload or release-install lease.
    bool begin_image(size_t wire_size = 0, size_t image_size = 0,
                     OtaImage::Encoding encoding = OtaImage::Encoding::Auto);
    bool write_image(size_t index, const uint8_t *data, size_t len);
    bool finish_image();
    void abort_image();
    const OtaImage::Status &image_status();

    bool begin_resmed_flash();
    void cancel_resmed_flash_claim();
    bool busy();
}
