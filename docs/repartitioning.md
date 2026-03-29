# Repartitioning

AirBridge v1.2.0 uses larger app partitions (1.9MB each, up from 1.4MB) and drops the dedicated ResMed SPIFFS partition.
ResMed firmware staging now uses the inactive OTA slot instead.

If your device is running the old layout, you need to repartition before updating to v1.2.0

## Wireless way: OTA migration firmware stub

If the device is already running AirBridge with WiFi + OTA working:

```bash
# 1. Build and OTA the migration firmware
export AIRBRIDGE_OTA_PASS=airbridge
pio run -e migrate-ota -t upload

# 2. Wait for "OTA READY" on the CPAP LCD (takes a few seconds)

# 3. Flash the real firmware
pio run -e ota -t upload
```

That's it. The migration firmware handles everything automatically.

## Wired way: serial connection

If you have serial access, just flash normally — `pio run -t upload` writes the new partition table directly, no migration needed.

## How it works

The migration runs in two stages:

**Stage 1 (slot sync):** If the migrate firmware landed on app1 (which happens with OTA), it copies itself to app0 and reboots. This is needed because Stage 2 rewrites the partition table, and app0 must contain valid firmware at offset 0x10000 (which is the same in both old and new layouts).

**Stage 2 (repartition):** Running from app0, the firmware writes the new partition table to flash, erases the OTA data (so the bootloader defaults to app0), and reboots.

The target partition table is generated at build time from `partitions.csv`, so if the layout ever changes again, just rebuild the migration firmware.

## Partition layouts

**v1 (old):** `partitions_v1.csv`
```
app0:    0x10000,  1.4MB
app1:    0x180000, 1.4MB
resmed:  0x2F0000, 1MB (SPIFFS)
```

**v2 (current):** `partitions.csv`
```
app0:    0x10000,  1.9MB
app1:    0x200000, 1.9MB
```
