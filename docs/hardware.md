# Hardware

## AirSense 10 edge connector


| Left | Right |
|------|-------|
| **Tx**   | nc    |
| **Rx**   | **GND**   |
| nc   | nc    |
| nc   | nc    |
| nc   | **+24V**  |

UART: 3.3V logic, 57600 8N1.

## Power

+24V rail from the edge connector. Use a buck converter (e.g. MP1584) to step down to 3.3V for the ESP32.

Hot-plugging the stepdown module may trip the AirSense overcurrent protection, causing it to shut down. A ~47 Ohm series resistor on the +24V input limits inrush current and prevents this.

## Bare minimum hardware diagram

```

    AirSense 10                          M5Stamp Pico
    Edge Connector                       +-----------+
                                         |           |
    Tx  -------------------------------- G36 (RX)    |
    Rx  -------------------------------- G26 (TX)    |
    GND ----------------+--------------- GND         |
                        |                |           |
                        |    +--------> 3V3          |
                        |    |           |           |
                        |    |           +-----------+
                        |    |
    +24V ---[47R]--+    |    |
                   |    |    |
               +---+----+----+---+
               |  IN+  GND  OUT+ |
               |    MP1584EN     |
               |   (3.3V out)    |
               +-----------------+


    Notes:
    - 47 ohm resistor in series with +24V prevents
      overcurrent trip when hot-plugging the step-down module
    - MP1584EN is a cheap ready-made module. Any 24V-to-3.3V
      step-down will work as long as it can supply 500mA+
    - TODO: design proper power supply closer to IEC 60601
      for medical electrical equipment
```

## Enclosure

STL files for 3D-printed case in [`docs/stl/`](stl/).
