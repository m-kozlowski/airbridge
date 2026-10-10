"""Replace only the SDK's TinyUSB archive for USB SD builds."""

from pathlib import Path


SOURCES = (
    "tusb.c",
    "common/tusb_fifo.c",
    "device/usbd.c",
    "device/usbd_control.c",
    "class/msc/msc_device.c",
    "portable/synopsys/dwc2/dcd_dwc2.c",
    "portable/synopsys/dwc2/dwc2_common.c",
)


def select_core_source(env, node):
    definitions = dict(
        item for item in env["CPPDEFINES"] if isinstance(item, (tuple, list))
    )
    if str(definitions.get("AB_USB_MSC_ENABLED")) == "1":
        if Path(node.srcnode().get_abspath()).name in (
            "USB.cpp", "USBCDC.cpp", "USBMSC.cpp", "FirmwareMSC.cpp",
            "esp32-hal-tinyusb.c",
        ):
            return None

    return node


def configure(env):
    definitions = dict(
        item for item in env["CPPDEFINES"] if isinstance(item, (tuple, list))
    )
    if str(definitions.get("AB_USB_MSC_ENABLED")) != "1":
        return

    sdk = Path(env.PioPlatform().get_package_dir("framework-arduinoespressif32-libs"))
    versions = dict(
        line.split(": ", 1) for line in (sdk / "versions.txt").read_text().splitlines()
        if ": " in line
    )
    if versions.get("tinyusb", "").split()[-1:] != ["2883403ed"]:
        raise RuntimeError("Review the local TinyUSB source pin after this SDK change")

    source = Path(env.subst("$PROJECT_LIBDEPS_DIR/$PIOENV/TinyUSB/src"))
    env.Prepend(CPPPATH=[str(source)])
    usb = env.Clone()
    usb.Prepend(CPPPATH=[str(source)])
    archive = usb.BuildLibrary(
        env.subst("$BUILD_DIR/tinyusb-device"), str(source),
        src_filter=" ".join(f"+<{name}>" for name in SOURCES),
    )

    libraries = env["LIBS"]
    if "arduino_tinyusb" not in libraries and "-larduino_tinyusb" not in libraries:
        raise RuntimeError("The SDK TinyUSB archive was not found in the link inputs")

    env.Replace(LIBS=[
        lib for lib in libraries if lib not in ("arduino_tinyusb", "-larduino_tinyusb")
    ])
    env.Prepend(LIBS=archive)
    print("TinyUSB: on-demand MSC only; Serial/JTAG remains the default")


if "Import" in globals():
    Import("env")
    configure(env)
