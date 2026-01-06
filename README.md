# ESP32 OBD-II Emulator (Enhanced Fork)

This is an improved fork of the original ESP32 OBD-II Emulator, featuring:
- **Modern Build System**: migrated to full CMake support for ESP-IDF.
- **Improved Debugging**: Enhanced logging capabilities and debug modes.
- **VS Code Integration**: Ready-to-use development environment configuration.
- **Performance**: Optimized task stack sizes and memory usage.

Open-source OBD-II emulator based on an ESP32 + CAN transceiver IC, controllable via WiFi through a simple web UI (or via API).

![Screenshot 1](docs/ui.jpg) ![Screenshot 2](docs/rpm.jpg) ![Screenshot 3](docs/throttle.jpg) ![Screenshot 4](docs/info.jpg)

## Supported protocols
- ISO 15765-4 CAN (11 bit, 500 Kbps)

## Supported modes & PIDs
| Mode | PID  | Description                         |
|------|------|-------------------------------------|
| 0x01 | 0x0C | RPM                                 |
| 0x01 | 0x0D | Vehicle speed                       |
| 0x01 | 0x11 | Throttle position                   |
| 0x09 | 0x02 | Vehicle Identification Number (VIN) |

## Usage
1. Connect to the WiFi network `ESP32-OBD2` (with password `88888888`)
2. Navigate to `192.168.4.1`
3. Enjoy :)

## Hardware
- ESP32-S3 (or ESP32-WROOM-32)
- TJA1051 CAN transceiver (5V) with level shifter, or SN65HVD230 (3.3V)
- Bi-directional logic level shifter (3.3V ↔ 5V) - required for TJA1051
- Serial->USB adapter
- Power supply (3.3V for ESP32, 5V for TJA1051)

![ESP32-WROOM-32](docs/esp32-wroom.jpg)
![SN65HVD230](docs/transceiver.jpg)
![Schematic](docs/schematic.jpg)

### Connections

**ESP32-S3 + TJA1051 (5V) with Voltage Divider:**
Since the TJA1051 is a 5V device and the ESP32-S3 is 3.3V, a voltage divider is used on the RX line to protect the board.

| ESP32 Pin | Connection | TJA1051 Pin |
|---|---|---|
| GPIO 15 (TX) | Direct Wire | TXD |
| GPIO 16 (RX) | Voltage Divider | RXD |
| - | 5V Power Supply | VCC |
| GND | Common Ground | GND |
| - | CAN Bus | CANH/CANL |

**Voltage Divider (5V to 3.3V):**
Protect the ESP32 RX pin using two resistors (2kΩ and 3kΩ) as a simple level shifter:

```text
                  2kΩ
TJA1051 RXD o----[RES]----o----o ESP32 GPIO 16
(5V Signal)               |
                          |
                         [RES] 3kΩ
                          |
                         GND
```
(This scales the 5V signal down to ~3V, which is safe for the ESP32).

*Note: The TJA1051 board used in this project already has the 120Ω termination resistor installed. Ensure the other end of the bus is also terminated.*

**Legacy ESP32-WROOM-32 with SN65HVD230 (3.3V):**
- IO 4 → CAN RX
- IO 5 → CAN TX

## Flash / Install (via [esptool](https://github.com/espressif/esptool))
1. Download the latest release binaries
2. Flash (app, bootloader, FAT filesystem): `esptool.py write_flash --flash_mode dio --flash_freq 40m --flash_size detect 0x1000 bootloader.bin 0x10000 obd2-emu.bin 0x8000 partitions.bin 0x110000 fatfs_image.img`

## Build
1. Install the [Espressif IoT Development Framework](https://github.com/espressif/esp-idf)
2. Clone this repo: `git clone ...`
3. (Optional) Configure: `make menuconfig`
4. Build: `make all`
5. Flash: `make flash`
6. Build & flash FAT image: `make flashfatfs`

**Note:** You might want to change some config values, for example: serial flasher, baud rate, pins, etc.

## API

PATCH `/api/vehicle`
- Content-Type: x-www-form-urlencoded
- Data:
  - `name`
    - speed
    - rpm
    - throttle
    - vin
  - `value`
- Example (CURL): `curl -XPATCH -H 'Content-Type: application/x-www-form-urlencoded' -d 'name=speed&value=50' '/api/vehicle'`

## Acknowledgements

- [ESP32-CAN-Driver](https://github.com/ThomasBarth/ESP32-CAN-Driver)
- [ESP32_makefatfs](https://github.com/jkearins/ESP32_mkfatfs)
- [esp32-http-server](https://github.com/igrr/esp32-http-server)
- [Espressif IoT Development Framework](https://github.com/espressif/esp-idf)
- [OBDSim](https://icculus.org/obdgpslogger/obdsim.html)