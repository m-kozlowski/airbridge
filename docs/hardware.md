# Hardware

## AirSense 10 edge connector


| Left | Right |
|------|-------|
| Tx   | nc    |
| Rx   | GND   |
| nc   | nc    |
| nc   | nc    |
| nc   | +24V  |

UART: 3.3V logic, 57600 8N1.

## Power

+24V rail from the edge connector. Use a buck converter (e.g. MP1584) to step down to 3.3V for the ESP32.

Hot-plugging the stepdown module may trip the AirSense overcurrent protection, causing it to shut down. A ~47 Ohm series resistor on the +24V input limits inrush current and prevents this.

## Enclosure

STL files for 3D-printed case in [`docs/stl/`](stl/).
