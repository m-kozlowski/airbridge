# Quick Start

## What you need

- M5Stamp Pico (ESP32-PICO-D4)
- MP1584 buck converter (24V to 3.3V)
- AirSense 10 with edge connector access
- USB-to-serial adapter (3.3V) for initial flash
- PlatformIO installed

## Wiring

See [hardware.md](hardware.md) for the pinout, wiring diagram, and power notes.

## Flash firmware

```bash
pio run -t upload
```

## Configure WiFi

Three options, pick whichever suits you:

**SmartConfig (no cable needed):** On first boot the device waits 60 seconds for SmartConfig. Install the [EspTouch](https://github.com/EspressifApp/EsptouchForAndroid/releases) app on your phone. Connect your phone to the target WiFi network, select **EspTouch v1**, enter the WiFi password, and hit confirm. The device picks up the credentials and connects automatically. Credentials are saved.

**Provisioning file:** Create `provision.env` from the example and flash:

```bash
cp provision.env.example provision.env
# Edit provision.env with your WiFi credentials
python provision.py <serial port>
```

Provisioning also runs automatically after every serial flash (`pio run -t upload`).

**Manual via AP:** If SmartConfig times out, the device creates a WiFi access point (`airbridge_XXXXXX`, password `airbridge`). Connect to it, open `http://192.168.4.1/`, go to the **Device** tab, set your WiFi SSID and password, wifi_mode to `0`. Save and reboot.

## Verify it works

After reboot, the device connects to your WiFi. Open `http://airbridge/` in your browser.

If mDNS doesn't resolve, check your router's DHCP leases for the device IP.

Default credentials:
- Username: `admin`
- Password: `airbridge`

The **Status** tab shows the system state. If the AirSense is powered on and connected, you should see `system: IDLE` along with the device name and serial number.

## Ports

| Port | Purpose |
|------|---------|
| 80   | Web UI |
| 23   | TCP command port (telnet) |
| 8023 | Debug log stream (read-only) |
| 3232 | OTA firmware updates |

## OTA updates

After initial setup, you can update firmware over WiFi:

```bash
export AIRBRIDGE_OTA_PASS=airbridge
pio run -e ota -t upload
```

## Oximetry

The device scans for BLE pulse oximeters automatically. Supported devices:
- Nonin 3230 (BLE)
- Wellue O2Ring
- Generic BLE PLX / Heart Rate sensors

Go to the **Bluetooth** tab in the web UI to scan, connect, and manage oximeter devices. When connected, SpO2 and pulse data are injected into the AirSense data stream.

## Command line

Connect via telnet to port 23 for direct control:

```bash
telnet airbridge 23
```

Commands use `$` prefix. Type `$HELP` for the full list. Anything without `$` is sent directly to the AirSense as a UART command.

Debug logs stream on port 8023:

```bash
nc airbridge 8023
```
