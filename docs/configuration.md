# Configuration

Settings are stored in NVS (non-volatile storage) and persist across reboots.

## How to configure

**Web UI:** Device tab -> edit fields -> Save

**CLI:** `$CONFIG key value` then `$CONFIG SAVE`

**Provisioning:** Fill in `provision.env` and flash. Settings are applied automatically after upload.

## Settings reference

### WiFi & Network

| Key | Default | Description |
|-----|---------|-------------|
| `hostname` | airbridge | Device hostname (mDNS + WiFi AP name) |
| `wifi_ssid` | *(empty)* | WiFi network name (station mode) |
| `wifi_pass` | *(empty)* | WiFi password |
| `wifi_mode` | 1 | 0 = station (connect to router), 1 = AP (create hotspot), 2 = off |
| `tcp_port` | 23 | TCP command port |
| `debug_port` | 8023 | Debug log stream port (read-only) |

In AP mode, the device creates a network named `<hostname>_<MAC>` with password "airbridge" and serves the web UI at `192.168.4.1`.

### Web UI

| Key | Default | Description |
|-----|---------|-------------|
| `http_port` | 80 | Web server port |
| `http_user` | admin | Web UI username |
| `http_pass` | airbridge | Web UI password |

### OTA

| Key | Default | Description |
|-----|---------|-------------|
| `ota_password` | airbridge | ArduinoOTA password |

Set the `AIRBRIDGE_OTA_PASS` environment variable to match when uploading:
```bash
export AIRBRIDGE_OTA_PASS=airbridge
pio run -e ota -t upload
```

### Oximetry

| Key | Default | Description |
|-----|---------|-------------|
| `oxi_enabled` | true | Enable BLE oximeter support |
| `oxi_auto_start` | true | Start feeding data automatically on connect |
| `oxi_feed_therapy_only` | false | Only inject readings during active therapy |
| `oxi_device_type` | 0 | 0 = auto-detect, 1 = Nonin, 2 = O2Ring, 3 = PLX |
| `oxi_device_addr` | *(empty)* | Preferred oximeter MAC (AA:BB:CC:DD:EE:FF) |
| `oxi_interval_ms` | 500 | Injection interval in milliseconds |
| `oxi_lframe_continuous` | true | Send L-frames even when no valid reading (keeps link alive) |

You don't usually need to set these manually. Use the **Bluetooth** tab in the web UI to scan and connect.

### UART

| Key | Default | Description |
|-----|---------|-------------|
| `uart_baud` | 57600 | AirSense UART baud rate (don't change unless you know what you're doing) |
| `uart_cmd_timeout_ms` | 500 | Command response timeout |
| `uart_max_retries` | 3 | Retry count for failed commands |

### Advanced

| Key | Default | Description |
|-----|---------|-------------|
| `allow_transparent_during_therapy` | false | Allow raw UART passthrough during therapy |
| `mitm_mode` | 0 | 0 = off, 1 = forward, 2 = log, 3 = filter |

## CLI commands

All commands are prefixed with `$`. Anything without `$` is sent to the AirSense as a Q-frame command.

| Command | Description |
|---------|-------------|
| `$STATUS` | System state, oximetry, UART stats, heap |
| `$OXI SCAN` | Scan for BLE oximeters |
| `$OXI RESULTS` | Show scan results |
| `$OXI CONNECT [addr]` | Connect to oximeter |
| `$OXI DISCONNECT` | Disconnect oximeter |
| `$OXI START` / `STOP` | Start/stop data injection |
| `$OXI STATUS` | Oximeter connection details |
| `$CONFIG` | Dump all config |
| `$CONFIG key` | Get single value |
| `$CONFIG key value` | Set value (not saved until SAVE) |
| `$CONFIG SAVE` | Persist to NVS |
| `$CONFIG RESET` | Reset all to defaults |
| `$FLASH [block] [FORCE]` | Flash uploaded ResMed firmware |
| `$FLASH STATUS` | Flash progress |
| `$FLASH CANCEL` | Abort flash |
| `$LOG` | Show log levels |
| `$LOG category level` | Set log level (ERROR/WARN/INFO/DEBUG) |
| `$TRANSPARENT` | Raw UART passthrough (5s idle timeout) |
| `$VERSION` | Firmware version |
| `$REBOOT` | Restart device |

Log categories: `BLE`, `TCP`, `OTA`, `WEB`, `ARB`, `HEALTH`, `ALL`
