"""Restore serial upload addresses when PlatformIO skips framework setup."""

from pathlib import Path
import runpy


def configure(env):
    framework = Path(env.PioPlatform().get_package_dir("framework-arduinoespressif32"))
    build = Path(env.Dir("$BUILD_DIR").abspath)
    board = env.BoardConfig()
    mcu = board.get("build.mcu")
    boot_offset = {"esp32": "0x1000", "esp32s3": "0x0"}[mcu]

    # Use the table built with this firmware, not a possibly edited CSV.
    parser = runpy.run_path(str(framework / "tools/gen_esp32part.py"))
    table = parser["PartitionTable"].from_binary((build / "partitions.bin").read_bytes())
    app = next(table.find_by_type("app", "ota_0"), None)
    if app is None:
        app = next(table.find_by_type("app", "factory"), None)
    otadata = next(table.find_by_type("data", "ota"), None)
    if app is None or otadata is None:
        raise RuntimeError("Built partition table lacks application or OTA data partition")

    images = [
        (boot_offset, str(build / "bootloader.bin")),
        (board.get("upload.arduino.partitions_bin", "0x8000"), str(build / "partitions.bin")),
        (hex(otadata.offset), str(framework / "tools/partitions/boot_app0.bin")),
    ]
    for _, path in images:
        if not Path(path).is_file():
            raise RuntimeError("Missing upload image: " + path + "; build this environment first")

    env.Replace(ESP32_APP_OFFSET=hex(app.offset), FLASH_EXTRA_IMAGES=images)
    # The platform already assembled UPLOADERFLAGS before running post scripts.
    for offset, path in images:
        env.Append(UPLOADERFLAGS=[offset, '"' + path + '"'])


if "Import" in globals():
    from SCons.Script import COMMAND_LINE_TARGETS

    Import("env")
    if (
        "nobuild" in COMMAND_LINE_TARGETS
        and "upload" in COMMAND_LINE_TARGETS
        and env.get("UPLOAD_PROTOCOL") == "esptool"
    ):
        configure(env)
