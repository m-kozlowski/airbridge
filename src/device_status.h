#pragma once

#include "uart_arbiter.h"
#include "airsense_state.h"
#include "oxi_arbiter.h"
#include "oxi_ble.h"
#include "sleep_report.h"
#include "clinical_jobs.h"
#include "sd_storage.h"
#include "wifi_setup.h"
#include "app_config.h"
#include "resmed_firmware_repository.h"
#include "file_upload.h"
#include "resmed_ota.h"
#include <string.h>

namespace DeviceStatus {

struct Snapshot {
    system_state_t sys;
    oxi_state_t oxi;
    oxi_reading_t reading;
    int rop;
    int mhr;
    int mop;
    bool feeding;
    char oxi_source[32];
    char oxi_name[32];
    uint32_t ip = 0;
    uint32_t ap_ip = 0;
    char ui_language[16] = "auto";
    char ui_theme[32] = "auto";
    SleepReport::Source report_source = SleepReport::Source::Device;
    uint32_t report_revision = 0;
    uint32_t report_data_revision = 0;
    uint32_t report_artifact_revision = 0;
    EdfReport::State report_state = EdfReport::State::Blocked;
    char report_error[48] = {};
    bool report_available = false;
    uint32_t clinical_revision = 0;
    SdStorage::Status storage = {};
    ResmedFirmwareRepository::Status firmware_repository = {};
    FileUpload::Status file_upload = {};
    ResmedOta::Status resmed = {};
};

// Copy published owner state only; presentation must not query the device.
inline Snapshot snapshot() {
    Snapshot out;
    out.sys = Arbiter::get_state();
    out.oxi = OxiBle::get_state();
    OxiArbiter::snapshot(out.reading);
    out.rop = AirSenseState::rop();
    out.mhr = AirSenseState::mhr();
    out.mop = out.sys == SYS_IDLE || out.sys == SYS_THERAPY
        ? AirSenseState::mop() : -1;
    out.feeding = OxiArbiter::is_feeding();
    OxiArbiter::get_source(out.oxi_source, sizeof(out.oxi_source),
                           out.oxi_name, sizeof(out.oxi_name));
    WiFiSetup::addresses(out.ip, out.ap_ip);
    strncpy(out.ui_language, Config::ui_language(), sizeof(out.ui_language) - 1);
    strncpy(out.ui_theme, Config::ui_theme(), sizeof(out.ui_theme) - 1);
    SleepReport::Publication report;
    SleepReport::get_publication(report);
    out.report_source = report.source;
    out.report_revision = report.local.revision;
    out.report_data_revision = report.local.data_revision;
    out.report_artifact_revision = report.local.artifact_revision;
    out.report_state = report.local.state;
    memcpy(out.report_error, report.local.error, sizeof(out.report_error));
    out.report_available = report.local.available;
    out.clinical_revision = ClinicalJobs::revision();
    SdStorage::get_status(out.storage);
    ResmedFirmwareRepository::get_status(out.firmware_repository);
    FileUpload::get_status(out.file_upload);
    ResmedOta::get_status(out.resmed);
    return out;
}

}  // namespace DeviceStatus
