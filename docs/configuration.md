# Configuration

Settings are stored in NVS (non-volatile storage) and persist across reboots.

## How to configure

**Web UI:** Config tab -> choose a category -> edit fields -> Save & Persist

**CLI:** `$CONFIG key value` then `$CONFIG SAVE`

**Provisioning:** Copy `provision.env.example` to `provision.env`, fill in
your values, and flash. `provision.py` runs after the upload completes and
applies each line via the appropriate command:

- Most keys go through `$CONFIG key value`.
- `wifi_net=SSID PASSWORD` is routed through `$WIFI ADD ssid pass`. Use
  shell-style double quotes if either field contains spaces:
  `wifi_net="My Network" "correct horse battery staple"`. Multiple
  `wifi_net=` lines provision multiple networks (up to 4). Provisioning
  appends; existing networks on the device are kept.

## Settings reference

### WiFi & Network

WiFi networks are managed as a list (up to 4 SSIDs) via the web UI's WiFi
Networks card or the `$WIFI ADD ssid pass` / `$WIFI REMOVE N` commands. The
top-level WiFi-related config keys are:

| Key | Default | Description |
|-----|---------|-------------|
| `hostname` | airbridge | DHCP and mDNS hostname, and AP SSID prefix |
| `wifi_mode` | 1 | Operating mode, see table below |
| `wifi_roam` | true | Roam automatically between saved networks |
| `wifi_country` | 01 | ISO 3166 country code; "01" = worldwide |

**`wifi_mode` values:**

| Value | Name | Behavior |
|-------|------|----------|
| 0 | auto | Connect to a saved network; use AP fallback while disconnected. |
| 1 | AP only | Access point only; does not connect to saved networks. |
| 2 | off | WiFi disabled entirely (serial/UART only). |
| 3 | STA only | Connect to saved networks without AP fallback. |
| 4 | STA+AP always | Keep the access point available while connecting to saved networks. |

The softAP (when used) is named `<hostname>_<MAC>` with password `airbridge`,
and serves the web UI at `192.168.4.1`.

The client must support mDNS and the network must allow multicast traffic.
If name resolution is unavailable, use the device IP address instead.
For a non-default `http_port`, append `:<port>` to the URL.

With no saved networks, Auto and STA+AP modes provide an access point for
setup. STA-only does not start an access point.

The `wifi_country` key affects channel allocation and TX power limits per
regulatory domain. Use a 2-letter ISO code (`US`, `DE`, `JP`, `PL`, ...) or
`01` to use the worldwide-safe defaults. Bad values are rejected with a
log warning and the previous setting stays in effect.

### WireGuard

Included in esp32-s3 builds (`AB_WIREGUARD_ENABLED=1`). Tunnel mode defaults
to `off`. VPN settings take effect after saving the complete profile.

| Key | Default | Description |
|-----|---------|-------------|
| `wg_mode` | off | `off`, `away` (except on home WiFi), or `always` |
| `wg_home` | *(empty)* | Comma-separated home SSIDs; matching is case-sensitive |
| `wg_addr` | *(empty)* | Tunnel IPv4 address, optionally followed by `/32` |
| `wg_key` | *(empty)* | Private key, base64-encoded |
| `wg_peer` | *(empty)* | Peer public key, base64-encoded |
| `wg_psk` | *(empty)* | Optional preshared key, base64-encoded |
| `wg_ep` | *(empty)* | Peer hostname or IPv4 address, optionally with `:port`; default port 51820 |
| `wg_ka` | 25 | Keepalive interval in seconds, 0-65535; 0 disables keepalive |
| `wg_allowed` | 0.0.0.0/0 | Up to seven IPv4 addresses or CIDR networks routed through the tunnel |
| `wg_dns` | *(empty)* | IPv4 DNS server while the tunnel is up; empty retains WiFi DNS |

### Web UI

| Key | Default | Description |
|-----|---------|-------------|
| `http_port` | 80 | Web server port |
| `http_user` | admin | Web UI username |
| `http_pass` | airbridge | Web UI password |
| `auth_whitelist` | *(empty)* | Trusted IPv4 addresses that bypass web login. Comma-separated addresses, CIDR networks, start-end ranges or `*`; maximum 160 characters |
| `ui_language` | auto | Web UI language: `auto` follows browser preferences; `en` selects English; `pl` selects Polish |
| `ui_theme` | auto | Web UI theme: `auto` follows browser preferences; an available theme ID overrides them for all browsers. Default themes: `light`, `dark`; also selectable from the header button |

### OTA

| Key | Default | Description |
|-----|---------|-------------|
| `update_url` | GitHub latest-release manifest | Release manifest URL; empty disables update checks |

Update checks are automatic. Installing an available update requires
confirmation on the OTA tab.

### Time & Timezone

| Key | Default | Description |
|-----|---------|-------------|
| `ntp_server` | *(empty)* | NTP server address. Empty = use DHCP-provided server, or pool.ntp.org as fallback |
| `tz` | UTC0 | POSIX timezone string (e.g. `CET-1CEST,M3.5.0,M10.5.0/3`) |
| `resmed_time` | 1 | Automatically synchronize the ResMed clock from NTP while idle. `0` disables automatic synchronization; manual sync remains available |

**Config > Time** includes a helper for detecting your browser's timezone.

If NTP is unavailable, the ResMed clock supplies an approximate time for
AirBridge, independently of `resmed_time`.

### Storage export

SMB and SleepHQ export require SD support and a mounted card. Both are disabled
by default. Automatic export sends pending recordings after startup and therapy,
only while therapy is inactive.

| Key | Default | Description |
|-----|---------|-------------|
| `smb_enabled` | false | Enable SMB export |
| `smb_auto_after_therapy` | true | Automatically sync pending recordings after startup and therapy |
| `smb_endpoint` | *(empty)* | Destination in `//host/share/optional/path` form |
| `smb_user` | *(empty)* | SMB username |
| `smb_password` | *(empty)* | SMB password |
| `sleephq_enabled` | false | Enable SleepHQ export |
| `sleephq_auto_after_therapy` | true | Automatically sync pending recordings after startup and therapy |
| `sleephq_client_id` | *(empty)* | SleepHQ API client ID |
| `sleephq_client_secret` | *(empty)* | SleepHQ API client secret |
| `sleephq_team_id` | *(empty)* | Optional numeric team ID; current team is resolved when empty |
| `sleephq_device_id` | *(empty)* | Optional SleepHQ device ID attached to new imports |

### Oximetry

| Key | Default | Description |
|-----|---------|-------------|
| `oxi_enabled` | true | Enable BLE oximeter support |
| `oxi_auto_start` | true | Start feeding data automatically on connect |
| `oxi_feed_therapy_only` | false | Only inject readings during active therapy |
| `oxi_interval_ms` | 500 | Injection interval in milliseconds |
| `oxi_lframe_continuous` | true | Send L-frames even when no valid reading (keeps link alive) |
| `udp_oxi_port` | 8025 | UDP oximetry listener port, 0 = disabled |

BLE oximeters are managed from the **Oximetry** tab. For UDP oximetry, see [udp_oximetry.md](udp_oximetry.md).

Only one source feeds at a time. First to deliver data wins, 10 seconds of silence releases.

### UART

| Key | Default | Description |
|-----|---------|-------------|
| `tcp_port` | 23 | TCP command port |
| `allow_transparent_during_therapy` | false | Allow raw UART passthrough during therapy |
| `uart_cmd_timeout_ms` | 500 | Command response timeout |

### Logging

| Key | Default | Description |
|-----|---------|-------------|
| `debug_port` | 8023 | Debug log stream port (read-only) |
| `syslog_en` | 0 | Enable UDP syslog forwarding |
| `syslog_host` | *(empty)* | Syslog server IPv4 address; required for forwarding |
| `syslog_port` | 514 | Syslog server UDP port (1-65535) |

## CLI commands

All commands are prefixed with `$`. Anything without `$` is sent to the AirSense as a Q-frame command.

| Command | Description |
|---------|-------------|
| `$STATUS` | System state, oximetry, UART stats, heap |
| `$TIME` | Show UTC, local time, timezone, NTP sync state |
| `$TIMESYNC` | Re-trigger ResMed clock sync from NTP |
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
| `$EXPORT STATUS` | Show SMB and SleepHQ state, progress, and last error |
| `$EXPORT SMB` | Sync pending recordings to SMB |
| `$EXPORT SLEEPHQ` | Sync pending recordings to SleepHQ |
| `$WIFI` / `$WIFI STATUS` | Connection state, RSSI, roaming flag |
| `$WIFI LIST` | List configured networks |
| `$WIFI ADD ssid pass` | Add a network (max 4) |
| `$WIFI REMOVE N` | Remove network at index N |
| `$WIFI HINTS` | Show cached BSSID/channel/PMF hints |
| `$WIFI HINTS CLEAR` | Drop all cached hints |
| `$FLASH [block] [FORCE]` | Flash uploaded ResMed firmware |
| `$FLASH FILE "path" [block] [BLX] [FORCE]` | Install an image from the SD firmware repository |
| `$FLASH LIST` / `$FLASH REFRESH` | List saved images or refresh the repository |
| `$FLASH DUMP` | Save the device firmware to the SD repository |
| `$FLASH CONFIRM id INSTALL_PATCHED_BOOTLOADER` | Confirm the offered bootloader installation, then retry that dump |
| `$FLASH STATUS` | Upload, flash or dump progress and errors |
| `$FLASH CANCEL` | Request cancellation; an in-progress programming sequence must finish |
| `$LOG` | Show log levels |
| `$LOG category level` | Set log level (ERROR/WARN/INFO/DEBUG) |
| `$TRANSPARENT` | Raw UART passthrough (60s idle timeout) |
| `$FRAMED` | Switch to framed UART mode |
| `$VERSION` | Firmware version, build time and target |
| `$REBOOT` | Restart device |

Log categories: `GENERAL`, `OXI`, `TCP`, `WIFI`, `OTA`, `WEB`, `ARB`, `HEALTH`,
`EXPORT`, `EDF`, `STREAM`, `STORAGE`, `TIME`, `CONFIG`, `REPORT`, `ALL`
