# UDP Oximetry

Feed SpO2 and heart rate from external sources over UDP. This allows integrating oximeters that AirBridge doesn't support natively - write a small script or app that reads your device and sends readings over the network.

## Packet format

7-byte UDP packet:

```
[0x55] [0xAB] [flags] [spo2_lo] [spo2_hi] [hr_lo] [hr_hi]
```

| Byte | Value |
|------|-------|
| 0-1 | Magic: `0x55 0xAB` |
| 2 | Flags: `0x00` |
| 3-4 | SpO2 as 16-bit little-endian (e.g. 95 = `0x5F 0x00`) |
| 5-6 | Heart rate as 16-bit little-endian (e.g. 72 = `0x48 0x00`) |

To signal no finger / invalid reading, send `0xFF 0x07` for both fields.

The encoding is compatible with the Bluetooth PLX standard (SFLOAT), so raw BLE notifications can be forwarded directly without conversion.

Invalid packets (wrong size, bad magic, out-of-range values) are silently dropped.

## Config

`udp_oxi_port` - default 8025, set to 0 to disable.

## Source arbitration

Only one source feeds at a time. The first source to deliver valid data (BLE or UDP) takes ownership. The other is ignored until the active source goes silent for 10 seconds.
