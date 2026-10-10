#pragma once

#include <stdint.h>

namespace FactoryReset {

enum class Scope : uint8_t { All = 1, StorageOnly = 2 };

// Schedules only; marker persistence runs in OtaManager's main-loop reboot.
// StorageOnly keeps NVS settings and is unavailable on non-SD builds.
bool request(const char **error = nullptr, Scope scope = Scope::All);

// Before Config/network/SD initialization. Consumes the marker before any
// destructive action; any full NVS erase attempt restarts without loading config.
// False means pending intent is unreadable or cannot be consumed: boot must halt.
bool run_pending_on_boot();

}  // namespace FactoryReset
