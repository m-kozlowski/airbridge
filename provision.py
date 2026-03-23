#!/usr/bin/env python3
"""
AirBridge provisioning script.

Reads key=value pairs from provision.env and sends them to the device
via serial $CONFIG SET commands. Can be run standalone or hooked into
the PlatformIO upload process via extra_scripts.

Usage:
    python provision.py [port]           # explicit serial port
    python provision.py                  # auto-detect from platformio
    pio run -t provision                 # via PlatformIO target
"""

import sys
import os
import time
import serial
import serial.tools.list_ports

try:
    _script_dir = os.path.dirname(os.path.abspath(__file__))
except NameError:
    _script_dir = os.getcwd()
PROVISION_FILE = os.path.join(_script_dir, "provision.env")
EXAMPLE_FILE = PROVISION_FILE + ".example"
BAUD = 115200
TIMEOUT = 2




def parse_provision_file(path):
    entries = []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                continue
            key, _, value = line.partition("=")
            key = key.strip()
            value = value.strip()
            if key and value:
                entries.append((key, value))
    return entries


def send_command(ser, cmd, expect_ok=True):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    ser.flush()
    time.sleep(0.1)

    response = ""
    deadline = time.time() + TIMEOUT
    while time.time() < deadline:
        if ser.in_waiting:
            response += ser.read(ser.in_waiting).decode(errors="ignore")
            if "\n" in response:
                break
        time.sleep(0.05)

    return response.strip()


def provision(port):
    if not os.path.exists(PROVISION_FILE):
        print(f"[*] No {os.path.basename(PROVISION_FILE)} found.")
        if os.path.exists(EXAMPLE_FILE):
            print(f"    Copy {os.path.basename(EXAMPLE_FILE)} to {os.path.basename(PROVISION_FILE)}")
            print(f"    and fill in your values, then run: python {os.path.basename(__file__)}")
        return False

    entries = parse_provision_file(PROVISION_FILE)
    if not entries:
        print("[*] provision.env is empty or has no values set, skipping.")
        return True

    print(f"[*] Provisioning {len(entries)} settings via {port}...")

    try:
        ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
    except serial.SerialException as e:
        print(f"[!] Cannot open {port}: {e}")
        return False

    time.sleep(2)
    ser.reset_input_buffer()

    resp = send_command(ser, "$STATUS")
    if "system" not in resp.lower() and "idle" not in resp.lower():
        time.sleep(1)
        resp = send_command(ser, "$STATUS")
        if "system" not in resp.lower() and "idle" not in resp.lower():
            print(f"[!] Device not responding. Got: {resp[:100]}")
            ser.close()
            return False

    print("[+] Device responding")

    errors = 0
    for key, value in entries:
        resp = send_command(ser, f"$CONFIG SET {key} {value}")
        if "OK" in resp or "ok" in resp or key in resp:
            print(f"    {key} = {value if 'pass' not in key else '****'}")
        else:
            print(f"    {key}: FAILED ({resp})")
            errors += 1

    resp = send_command(ser, "$CONFIG SAVE")
    if "OK" in resp or "ok" in resp:
        print("[+] Config saved to NVS")
    else:
        print(f"[!] Save failed: {resp}")
        errors += 1

    print("[*] Rebooting...")
    send_command(ser, "$REBOOT", expect_ok=False)

    ser.close()

    if errors:
        print(f"[!] Provisioning completed with {errors} errors")
    else:
        print("[+] Provisioning complete")

    return errors == 0


def main():
    if len(sys.argv) < 2:
        print(f"Usage: python {os.path.basename(sys.argv[0])} <serial-port>")
        print(f"  e.g. python {os.path.basename(sys.argv[0])} /dev/ttyUSB0")
        sys.exit(1)

    success = provision(sys.argv[1])
    sys.exit(0 if success else 1)


try:
    Import("env")

    def pio_after_upload(source, target, env):
        protocol = env.GetProjectOption("upload_protocol", "esptool")
        if protocol != "esptool":
            return  # skip for OTA and other non-serial protocols
        upload_port = env.get("UPLOAD_PORT")
        if upload_port:
            provision(upload_port)

    env.AddPostAction("upload", pio_after_upload)
except Exception:
    pass  # Not running under PlatformIO

if __name__ == "__main__":
    main()
