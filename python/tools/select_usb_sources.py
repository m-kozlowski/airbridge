"""Keep Arduino Serial/JTAG, but omit its always-on TinyUSB owner."""

from build_tinyusb import select_core_source

Import("env")
env.AddBuildMiddleware(select_core_source, "*/cores/esp32/*")
