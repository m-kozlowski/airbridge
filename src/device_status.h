#pragma once

#include "uart_arbiter.h"
#include "airsense_state.h"
#include "oxi_arbiter.h"
#include "oxi_ble.h"
#include "sleep_report.h"
#include "sd_storage.h"
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
    SleepReport::Source report_source = SleepReport::Source::Device;
    uint32_t report_revision = 0;
    uint32_t report_data_revision = 0;
    EdfReport::State report_state = EdfReport::State::Blocked;
    char report_error[48] = {};
    bool report_available = false;
    SdStorage::Status storage = {};
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
    SleepReport::Publication report;
    SleepReport::get_publication(report);
    out.report_source = report.source;
    out.report_revision = report.local.revision;
    out.report_data_revision = report.local.data_revision;
    out.report_state = report.local.state;
    memcpy(out.report_error, report.local.error, sizeof(out.report_error));
    out.report_available = report.local.available;
    SdStorage::get_status(out.storage);
    return out;
}

}  // namespace DeviceStatus
