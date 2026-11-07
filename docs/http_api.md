# HTTP API

**Contents**

- [Errors](#errors)
- [Dates](#dates)
- [Status](#status)
- [Sessions and EDF files](#sessions-and-edf-files)
- [Configuration and actions](#configuration-and-actions)
- [Export](#export)
- [Clinical settings and reports](#clinical-settings-and-reports)
- [Events and live data](#events-and-live-data)
- [File uploads](#file-uploads)
- [Firmware](#firmware)
- [Crash diagnostics](#crash-diagnostics)

Base URL: `http://<hostname>:<http_port>` (default port: `80`). All routes
use the configured web credentials. The server challenges with HTTP Digest;
`curl --anyauth` handles it. Authentication is bypassed for `auth_whitelist`
clients, or when both credentials are empty. Traffic is unencrypted;
Digest avoids sending the password itself.

Examples use a POSIX shell and the default credentials:

```sh
BASE=http://airbridge
AUTH=admin:airbridge
curl --anyauth -u "$AUTH" "$BASE/api/status"
```

Parameters below are query parameters unless marked **JSON**, **form**
(`application/x-www-form-urlencoded`) or **multipart**. Responses are JSON
unless specified otherwise. `202` means accepted, not completed. Check the
response body as well as the HTTP status; some actions return `200` with
`ok: false`. Additional response fields may be added.

JSON request bodies are limited to 4096 bytes, or 2048 for `/api/settings`;
larger bodies return `413`. File uploads use separate limits below.

## Errors

JSON errors usually contain `error`, optionally with `ok: false`:

```json
{"error":"catalog_busy"}
```

| Status | Meaning |
|---|---|
| `400` | Invalid parameters or body |
| `401` | Authentication required or credentials rejected |
| `409` | Request conflicts with the current device/storage state |
| `413` | Request body too large |
| `503` | Data, storage or resources unavailable; may be temporary |

Endpoint-specific codes are listed below. Check `Content-Type` before
decoding errors: authentication and release-note errors may be plain text
or HTML. A failed write may have applied some changes; read back its state
before retrying.

## Dates

- **Therapy date**: `YYYYMMDD`, naming the local noon-to-noon recording
  day. Morning sessions belong to the preceding date. Used by
  `/api/sessions` and report artifacts.
- **Day number**: the integer day count from `1970-01-01` for a therapy
  date. Used by `/api/report`.

## Status

`GET /api/status` returns the current status object. Common fields:

| Fields | Meaning |
|---|---|
| `version`, `built`, `target` | AirBridge version, build timestamp and hardware build target |
| `pna`, `srn` | AirSense product name and serial number |
| `system` | AirBridge state, e.g. `IDLE`, `THERAPY`, `ERROR` |
| `rop` | Raw AirSense operating mode, e.g. `0` standby, `1` therapy, `5` upgrade; `-1` unknown |
| `mhr` | AirSense blower runtime in seconds; `-1` unknown |
| `mode` | Therapy mode label, e.g. `CPAP` or `AutoSet` |
| `uptime`, `heap`, `psram_free` | Uptime in seconds; free memory in bytes (`psram_free: -1` if absent) |
| `ssid`, `rssi`, `ip`, `ap_ip` | Network connection and addresses |
| `esp_time`, `resmed_time` | Local clock strings; `--` when unknown |
| `oxi`, `oxi_addr`, `oxi_name`, `feeding`, `spo2`, `pulse` | Oximetry; invalid readings are `-1`; `feeding` is `"yes"` or `"no"` |
| `sd`, `sd_error`, `sd_bus`, `sd_bus_clock_khz` | Storage state; all except `sd` require an SD build |
| `edf`, `edf_prefix`, `edf_errors` | Recording state; non-SD builds return only `edf: "unsupported"` |
| `clinical_revision`, `report_revision`, `config_revision` | Change counters; see below |
| `file_upload`, `resmed` | Upload and AirSense firmware operation objects |

Use `/events` for subsequent updates rather than repeatedly fetching status.
Change counters restart after reboot; use them to detect changes within a
connection, and read current state after reconnecting.

## Sessions and EDF files

### GET /api/sessions

Lists sessions and metadata for the shared `STR.edf`. Requires an SD build.

| Parameter | Default | Values |
|---|---|---|
| `since` | All dates | Inclusive therapy date |
| `limit` | `32` | Integer `1`-`64` |
| `order` | `asc` | `asc` or `desc`, by `(day, file_prefix)` |
| `cursor` | First page | Value of `next_cursor` from the preceding response |

```sh
# Latest session, which may still be recording.
curl --anyauth -u "$AUTH" "$BASE/api/sessions?order=desc&limit=1"

# First page from a therapy date onwards.
curl --anyauth -u "$AUTH" "$BASE/api/sessions?since=20261001"

# Continue using the returned next_cursor, with the same since and order.
curl --anyauth -u "$AUTH" -G "$BASE/api/sessions" \
  --data-urlencode since=20261001 \
  --data-urlencode cursor=20261010/20261011_094849
```

Example response with `limit=1`:

```json
{
  "sessions": [{
    "day": "20261010",
    "start": "2026-10-11T09:48:49",
    "end": "2026-10-11T10:08:49",
    "closed": true,
    "files": [
      {"path":"/DATALOG/20261010/20261011_094849_BRP.edf","size":1922048},
      {"path":"/DATALOG/20261010/20261011_094849_PLD.edf","size":510720},
      {"path":"/DATALOG/20261010/20261011_094849_SAD.edf","size":6144},
      {"path":"/DATALOG/20261010/20261011_094849_EVE.edf","size":1024},
      {"path":"/DATALOG/20261010/20261011_094849_CSL.edf","size":1024}
    ]
  }],
  "str": {
    "path": "/STR.edf",
    "size": 36772,
    "modified": 1791709918,
    "etag": "\"32822c9b\""
  },
  "next_cursor": "20261010/20261011_094849"
}
```

- `day` is the therapy date.
- `start` and `end` are native EDF local times, without timezone or DST
  offset. `end` describes recorded coverage; it is `null` for an open
  session or when the remaining files cannot establish it.
- Download recordings only when `closed` is `true`. Closure does not require
  a matching STR record. `files` contains existing EDF files and byte sizes;
  an open recording may contain `.part` paths.
- `next_cursor: null` ends the listing. Otherwise, keep the same `since` and
  `order` and pass the cursor to the next request. It survives restarts;
  rescan relevant dates to see changes to earlier pages.
- `str` is independent of the date filter and is `null` if absent.
  `modified` is Unix seconds or `null`. The quoted ETag is a whole-file
  CRC32; compare it between listings to decide whether to download STR
  again.

Malformed/unknown/duplicate parameters return `400`. Busy responses return
`503 catalog_busy`; unavailable storage/catalog/metadata returns
`503 catalog_unavailable`. This includes USB sharing and metadata rebuilds
after startup, a remount or changes to `/DATALOG` files through Storage API.
Keep the cursor and retry the same GET later. At most two session listings
may be served concurrently.

### Storage

Requires an SD build. Paths are absolute, at most 255 bytes; URL-encode
them. USB sharing and active therapy block directory listings, downloads
and file changes. A closed session may be listed by `/api/sessions` while
its download is blocked by another recording. Files may change or disappear
between listing and download.

| Method and path | Parameters | Success |
|---|---|---|
| `GET /api/storage/list` | `path=/`, `offset=0` | Directory listing, up to 32 entries |
| `GET /api/storage/download` | `path` | Complete file, binary attachment |
| `GET /api/storage/archive` | `path=/`; optional repeated `item` child names | ZIP of selected files, recursively; empty directories are omitted |
| `POST /api/storage/rename` | **form**: `path`, `name` (new absolute path or path relative to the source parent) | `{"ok":true,"changed":1}` |
| `POST /api/storage/delete` | **form**: `path`; optional repeated `item` child names | `{"ok":true,"changed":N}`; directory deletion is recursive |
| `POST /api/storage/usb` | **form**: `enabled=1` or `enabled=0` | `202 {"ok":true}`; observe `sd` in status |

Directory listing:

```json
{"ok":true,"path":"/","offset":0,"entries":[{"name":"STR.edf","size":36772,"modified":1791709918,"directory":false}],"more":false}
```

If `more` is true, increase `offset` by the returned entry count. Entries
can be skipped or repeated if the directory changes between pages.
Mutations can report a nonzero `changed` count even if a later step fails.
Busy/missing-file requests can return `409`; unavailable storage or
resources can return `503`.

```sh
curl --anyauth -u "$AUTH" -G "$BASE/api/storage/download" \
  --data-urlencode path=/STR.edf -o STR.edf
curl --anyauth -u "$AUTH" -G "$BASE/api/storage/download" \
  --data-urlencode path=/DATALOG/20261010/20261011_094849_PLD.edf -o session_PLD.edf
```

General storage downloads do not support byte ranges or conditional GET.

## Configuration and actions

`GET /api/config` returns string-valued settings; sensitive values are empty
strings. Optional `section`: `network`, `access`, `time`, `oximetry`,
`uart`, `smb`, `sleephq`, `updates`, `logging`. Omit it for all settings.

`POST /api/config` accepts a **JSON object with string values** and returns
`{"ok":true,"saved":N}`. Invalid/unknown values may be ignored: inspect
`saved` and read back non-secret settings. Do not write the masked GET
response back wholesale. Keys and allowed values:
[Configuration](configuration.md).

```sh
curl --anyauth -u "$AUTH" -H 'Content-Type: application/json' \
  -d '{"ui_language":"pl"}' "$BASE/api/config"
```

| Method and path | Request | Response |
|---|---|---|
| `GET /api/wifi` | None | `state`, `ssid`, `rssi`, `ip`, `ap_ip`, `net_idx`, `networks[]`; passwords omitted |
| `POST /api/wifi` | **JSON strings**: `action=add` with `ssid`, `pass`; `remove` with `idx`; `update` with `idx` and optional `ssid`, `pass` | `{"ok":"true","result":"..."}`; `ok` is a string |
| `GET /api/ble` | None | Oximetry state/readings, discovered `devices[]`, remembered `known[]` |
| `POST /api/ble` | **JSON strings**: `action`; see below | `{"ok":true,"result":"..."}` |
| `POST /api/time` | **JSON strings**: `action=ntp_sync`, `sync_to_resmed` or `sync_from_resmed` | `{"ok":"true","result":"..."}`; `ok` is a string |
| `POST /api/cmd` | **JSON**: `{"cmd":"$VERSION"}` | `ok`, optional `response`/`error` |
| `POST /api/onboarding` | **JSON strings**: optional `http_user`, `http_pass`; `{}` keeps current credentials | `{"ok":true}`; marks setup complete |
| `POST /api/reboot` | No body | `{"ok":true}`, then reboots AirBridge |

BLE actions: `scan`, `stop_scan`, `connect` (optional `addr`), `disconnect`,
`enable`, `disable`, `start_feed`, `stop_feed`, `delete_bond` (`addr`),
`delete_all_bonds`, `autoconnect` (`addr`, `enabled: "true"` or `"false"`).
Scan completion changes `ble_revision` in status; then fetch `/api/ble`.

For `/api/cmd`, `$` commands target AirBridge; other commands target
AirSense UART. Inspect `response` text for errors even when `ok` is true.

## Export

| Method and path | Parameters | Response |
|---|---|---|
| `GET /api/export` | None | `supported`, `smb`, `sleephq` status objects; non-SD builds return `{"supported":false}` |
| `POST /api/export/smb` | `action=sync` (default) or `action=check` | `202 {"ok":true,"state":"pending"}`, or `409` with `error` |
| `POST /api/export/sleephq` | Same | Same |

Common export fields include `state`, `activity` (`phase`, `wait`, `file`),
`last_run`, `last_sync_epoch`, `check_state`, `check_error`,
`backlog_state`, `backlog_files`, `backlog_sessions`, `sync_blocked` and
`error`. Follow changes through the `exports` SSE event.

```sh
curl --anyauth -u "$AUTH" -X POST "$BASE/api/export/sleephq"
```

## Clinical settings and reports

| Method and path | Request | Response |
|---|---|---|
| `GET /api/settings` | None | `202 {"job":N}`; completed result is an array of setting descriptors |
| `POST /api/settings` | **JSON string values**, e.g. `{"LAN":"0"}` | `202 {"job":N}`; completed result is `{"saved":N}` with optional `errors` |
| `GET /api/settings` | `job=N` | `202 {"pending":true}` or the completed result; `410` if expired/unavailable |
| `GET /api/report` | `source=device&view=day&day=0` | Native AirSense daily report job |
| `POST /api/report` | `source=device&view=period&period=1`; no body | Native 7-day report job |
| `GET /api/report` | `source=edf`, optional `action`, `view`, `day`, `period`, `revision` | EDF result or job |
| `POST /api/report` | `source=edf&action=reindex`; no body | Rebuild job |
| `GET /api/report` | `job=edf:N` or `job=device:N` | Completed result, or `202` with `job` and `pending:true`; `410` if unavailable |
| `DELETE /api/report` | `job=edf:N` | `204`; device jobs cannot be cancelled (`405`) |

Settings descriptors include `cmd`, raw `value`, `editable` and type/range/
scaling metadata. Writes use raw integers encoded as strings. A timed-out
POST may have saved some values; read the settings back before retrying.

Report defaults: `action=summary`, `view=day`; default source is `edf` on SD
builds, otherwise `device`. EDF actions: `days`, `summary`, `history`,
`prepare`, `reindex`. `prepare` returns an artifact manifest. Device reports
support only `summary`. Query `period=0..5` selects 1, 7, 30, 90, 180 or 365
days and requires `view=period`. `day` is a day number; use values returned
by the report API. With `day=0`, EDF selects the latest date with a
completed recording; device reports select the last day the device saved.
Optional EDF `revision` takes the returned `data_revision`.

```sh
curl --anyauth -u "$AUTH" "$BASE/api/settings"
curl --anyauth -u "$AUTH" "$BASE/api/report?source=edf&action=days"
curl --anyauth -u "$AUTH" "$BASE/api/report?source=edf&action=prepare"
# After the notification for {"job":"edf:7"}:
curl --anyauth -u "$AUTH" "$BASE/api/report?job=edf:7"
```

Subscribe to `/events` before submitting: `clinical_revision` changes signal
settings/native-report completion; `report_revision` changes signal EDF job
updates. Fetch the job result after that notification or reconnection.
Reports can return `409` (`stale_revision`, `not_idle`) or `503` (`no_sd`,
busy/unavailable worker).
Preserve `partial`, `events_known` and null measurements in report results.

### Report artifacts

`GET` or `HEAD /api/report/artifact` takes `day` (therapy date), `hash` from
the manifest and `kind=document|tiles`. Index documents use `day=index`.
`document` is JSON; `tiles` is binary. The hash is a 64-character SHA-256
identity.

This route supports `ETag`, `If-None-Match` (`304`), `If-Range`, and
explicit closed byte ranges (`206`), e.g. `Range: bytes=0-255`. Up to eight
sorted, non-overlapping ranges and 256 KiB total are accepted; invalid
ranges return `416`. Full downloads return `200`. Artifacts are immutable
and may be cached.

## Events and live data

Both streams use Server-Sent Events (`text/event-stream`) and the same HTTP
authentication. Events sent while disconnected are lost. The first `status`
event after reconnecting supplies the full event state; merge later updates.

| Stream | Event | Payload |
|---|---|---|
| `/events` | `status` | Initial state, then changed fields; merge into existing state |
| `/events` | `exports` | Initial export state, then patches to `smb`/`sleephq`; SD builds only |
| `/events` | `ota` | Dynamic OTA status |
| `/events` | `wifi` | `{"revision":N}`; fetch `/api/wifi` after a change |
| `/events/live` | `live` | `{"seq":N,"samples":[[mkp,rfl,lyk],...]}` |

Events may omit fields available in the corresponding GET response.

```sh
curl --anyauth -u "$AUTH" -N "$BASE/events"
```

Opening `/events/live` enables live acquisition. `GET /api/live?since=SEQ`
retrieves buffered samples but does not start acquisition by itself:

```json
{"seq":123,"rate":25,"active":"yes","spo2":97,"pulse":65,"samples":[[500,100,0]]}
```

Samples are raw `[mask pressure, respiratory flow, leak]` values. Multiply
pressure by 0.02 (cmH2O), flow by 0.12 (L/min) and leak by 1.2 (L/min).

`seq` wraps at 16 bits; history holds up to 128 samples. Use the last
sequence to backfill a short interruption. Invalid SpO2 or pulse is `-1`;
`active` is a string (`"yes"`/`"no"`).

## File uploads

`POST /api/upload` writes an SD file when `path` is supplied. Without
`path`, it stages an AirSense image for a later `/api/flash` request.

| Format | Content-Type | Success |
|---|---|---|
| Raw chunks | `application/octet-stream` | `202`: `ok`, `upload_id`, `written`, `chunk_bytes`, `complete` |
| One multipart file | `multipart/form-data` | `202`: `ok`, `upload_id` |

All options are query parameters. Multipart text fields are ignored.

- Raw: first request sends `name` (staging) or absolute `path` (SD), total
  `size` in bytes, `offset=0`, and `final=0` or `1`. The body is the first
  chunk, at most 4096 bytes.
- Raw: wait for each response before sending the next chunk. Send `id`
  equal to `upload_id`, `offset` equal to `written`, and `final=1` on the
  last chunk. Follow the returned `chunk_bytes` limit (at most 65536).
- Multipart: send one file field, e.g. `file`, with optional query `path`
  and `size`. Its filename supplies the staging name.
- For SD, `overwrite=1` permits replacement; default is `0`.
- For SD, `bootloader=1` uploads a 16 KiB patched bootloader to
  `/airbridge/resmed-firmware/bootloaders/<BID>/patched.bin`, overriding
  the requested destination. Use `file_upload.path` for the stored path.

Maximum staging size is 1 MiB; SD files are limited to 4294967295 bytes.
Wait for the matching `file_upload.id` and `file_upload.ready` in status
before using the uploaded file. Follow `/events` for status changes.
Failures return `400`, `409` or `503` with an `error`, such as
`invalid_chunk`, `unknown_id`, `busy` or `no_memory`.

```sh
curl --anyauth -u "$AUTH" -F file=@sample.txt \
  "$BASE/api/upload?path=/sample.txt"
```

## Firmware

### AirBridge firmware

| Method and path | Request | Response |
|---|---|---|
| `GET /api/ota` | None | OTA status |
| `POST /api/ota/check` | No body | `202` with OTA status |
| `POST /api/ota/install` | Optional `url` | `202` with OTA status |
| `GET /api/ota/release-notes` | `id` from `release_notes_id` | Markdown text; `404` if unavailable |
| `POST /api/esp32/upload` | **multipart**: one firmware file | `200`; inspect string-valued `ok` |

`/api/ota/install` installs the checked release, or a direct HTTP(S) image
URL supplied in `url`. Check/install requests return `409` when blocked.

OTA status includes `state`, `version`, `update_version` (when known),
`release_target`, `version_comparable`, `release_installable`,
`release_notes_id`, `bytes`, `total_size`, and optional `progress`,
`blocked`, `error`. Follow the `ota` SSE event.

`release_installable` means a checked release image is available for the
build target, including a same-version or older release. `blocked` gives
the current reason installation cannot start, even when that flag is true.
`version_comparable` says whether the local version could be compared with
the release version. Missing, invalid or oversized release images cannot
be installed.

Manual `/api/esp32/upload` accepts `.bin` or zlib-compressed firmware. Its
response contains `ok: "true"`, `size`, `wire_size`, `encoding`,
`partition`, or `ok: "false"` with `error`. After success, call
`POST /api/reboot` to run the uploaded firmware.

```sh
curl --anyauth -u "$AUTH" -X POST "$BASE/api/ota/check"
# NOTES_ID is release_notes_id from OTA status.
curl --anyauth -u "$AUTH" "$BASE/api/ota/release-notes?id=$NOTES_ID"
curl --anyauth -u "$AUTH" -F firmware=@firmware.bin "$BASE/api/esp32/upload"
```

### AirSense firmware

| Method and path | Request | Response |
|---|---|---|
| `GET /api/resmed/repository` | Optional `path`, `offset=0` | Directory listing |
| `POST /api/resmed/repository` | **JSON**: `{"action":"refresh"}` | `202 {"ok":true}` |
| `GET /api/flash` | None | AirSense operation status |
| `POST /api/flash` | **JSON**: install or dump, below | `202 {"ok":true}`; `409` if blocked |
| `POST /api/flash/cancel` | No body | `200 {"ok":true}`; requests cancellation |

The repository defaults to `/airbridge/resmed-firmware`. Its directory
listing adds `root`, `supported`, `available`, `ready`; entries add `path`
and `bootloader`. Use Storage routes to download, rename or delete files.
Non-SD builds return `{"supported":false,"ready":false,"entries":[]}`.

To install, send exactly one of `upload_id` or repository `path`.
`upload_id` must refer to a ready staged image or an SD upload inside the
firmware repository. Optional fields:

| Field | Default | Effect |
|---|---|---|
| `block` | Autodetect | Select `FULL`, `CMX`, `CDX`, `CCX` or `BLX` |
| `flash_blx` | `false` | Include BLX when installing FULL |
| `force_blx` | `false` | Accept image integrity/compatibility warnings |
| `preserve_settings` | `true` | Enter upgrade mode before flashing |

`FULL` writes CMX only unless `flash_blx` is true. Explicit `block: "BLX"`
writes the bootloader regardless of `flash_blx`. `force_blx` accepts CRC
failure, a mismatched bootloader ID or an unsafe/partial integrity patch.
`preserve_settings` requests and confirms `ROP=5` to retain settings;
setting it to false skips this step.

To dump, send `{"action":"dump"}`. When a patched bootloader is needed,
confirmation requires `bootloaders/<BID>/patched.bin` to pass size, BID,
CRC and integrity checks. Upload this file with `bootloader=1`.
A missing file gives `matching_bootloader_missing`; size or validation
failures give `invalid_recovery_blx_size` or `invalid_recovery_blx`.

If status reports `confirmation_required`, send `action: "confirm_dump"`,
the operation `id`, and `confirm: "INSTALL_PATCHED_BOOTLOADER"`. This
permanently installs the patched bootloader before reading the firmware.
The dump includes the new bootloader. Restoring the original requires a
separate original image and an explicit BLX installation.

Operation status includes `id`, `active`, `phase`, `sent`, `total`, `error`,
`prepared`, `cancel_allowed`, `install_allowed`, `dump_allowed`, `blocked`,
`confirmation_required`. It also appears as `resmed` in status events and
`/api/status`.

## Crash diagnostics

| Method and path | Response |
|---|---|
| `GET /api/crash` | Crash status and available summary |
| `GET /api/crash/dump` | Binary `airbridge-coredump.bin` |

Status includes `state`, `size`, `stored_size`, `error`,
`summary_available`; available summaries add `task`, `reason`, `pc`,
`backtrace` and other diagnostics. The dump returns `404` if
empty/unsupported, or `409` if invalid or another download is active.
Byte ranges are unsupported.

```sh
curl --anyauth -u "$AUTH" "$BASE/api/crash/dump" -o airbridge-coredump.bin
```
