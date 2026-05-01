# Planetary Joint Controller

Firmware for a planetary gearbox actuator using an ESP32-S3 controller board.

This project reads an AS5048A magnetic absolute encoder over SPI and drives a Trinamic TMC2209 stepper motor driver via UART. The core firmware is built with the Arduino framework and PlatformIO.

## Key Features

- AS5048A 14-bit SPI encoder support
- Raw and continuous angle computation for multi-turn tracking
- SPI parity and AS5048A error flag validation
- TMC2209 UART communication and safe register configuration
- Motor enable/disable and velocity test support
- USB CDC serial console with interactive commands
- Persistent configuration storage in ESP32 NVS
- WS2812 RGB LED diagnostic state machine
- Modular source layout with headers in `include/`

## Hardware Target

- Board: Waveshare ESP32-S3 Zero
- MCU: ESP32-S3
- Framework: Arduino
- Build system: PlatformIO

## Project Layout

- `platformio.ini` — build configuration
- `src/main.cpp` — firmware entry point
- `src/as5048a.cpp`, `include/as5048a.h` — encoder support
- `src/Tmc2209Driver.cpp`, `include/Tmc2209Driver.h` — stepper driver interface
- `src/SerialConsole.cpp`, `include/SerialConsole.h` — USB command console
- `src/led_status.cpp`, `include/led_status.h` — status LED state management
- `src/params.h` — persistent parameter storage

## Pinout

| Function | GPIO |
|---|---:|
| AS5048A SCK | GPIO12 |
| AS5048A MISO | GPIO13 |
| AS5048A MOSI | GPIO11 |
| AS5048A CS | GPIO10 |
| TMC2209 UART TX | GPIO7 |
| TMC2209 UART RX | GPIO8 |
| TMC2209 STEP | GPIO4 |
| TMC2209 DIR | GPIO5 |
| TMC2209 EN | GPIO6 |
| WS2812 DIN | GPIO21 |
| Reserved / unused | GPIO1, GPIO2, GPIO3 |

## Firmware Behavior

- `Serial` is used for the USB CDC console at 115200 baud
- `Serial2` is used for TMC2209 UART communication at 115200 baud
- HSPI is used for AS5048A encoder communication
- Parameters are initialized, loaded from NVS, and printed at boot
- Encoder reads happen periodically and trace output can be enabled
- TMC2209 is configured safely before any motor enable request

## Supported Serial Commands

- `help` — list available commands
- `get <key>` — read a stored parameter
- `set <key> <value>` — write a parameter value
- `load` — load parameters from NVS
- `save` — save parameters to NVS
- `export` — export current parameters over serial
- `import` — import parameters from serial input
- `cancel` — cancel import mode
- `trace` — toggle encoder trace output
- `test <speed>` — run the motor at a target microstep speed for validation
- `reboot` — reboot the board

## Persistent Parameters

The firmware initializes the following parameters by default:

- `kp` — proportional gain
- `ki` — integral gain
- `kd` — derivative gain
- `ustep` — microstep resolution
- `irun` — TMC2209 run current scale
- `ihold` — TMC2209 hold current scale

These values persist across power cycles using ESP32 NVS.

## LED Status

| State | Behavior |
|---|---|
| BOOT | Blue fixed |
| READY | Green fixed |
| ENCODER_OK | Dim green |
| ENCODER_ERROR | Yellow blinking |
| FAULT | Red fast blinking |

## PlatformIO Configuration

Current supported environment:

```ini
[env:esp32-s3-fh4r2]
platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.30-2/platform-espressif32.zip
framework = arduino
board = esp32-s3-fh4r2

lib_deps =
  adafruit/Adafruit NeoPixel@^1.15.1

build_flags =
  -DBOARD_HAS_PSRAM
  -mfix-esp32-psram-cache-issue
  -DARDUINO_USB_CDC_ON_BOOT=1
  -DARDUINO_USB_MODE=1

monitor_port = /dev/ttyACM0
monitor_speed = 115200
monitor_filters =
  default,
  log2file,
  time
```

## Build and Run

1. Open the project in PlatformIO.
2. Build the `esp32-s3-fh4r2` environment.
3. Upload firmware to the ESP32-S3 Zero module.
4. Open the serial monitor at `115200` baud.

## Encoder Output

When `trace` mode is enabled, the console prints periodic encoder values like:

```text
@rdeg:271.384,cdeg:271.384
```

- `rdeg` — absolute encoder angle in degrees (0..360)
- `cdeg` — continuous angle in degrees across revolutions

## Continuous Angle Tracking

The AS5048A reports an absolute position inside one revolution. The firmware computes a continuous angle by:

- tracking the previous raw encoder value
- computing delta on each read
- detecting wrap-around at half revolution
- incrementing or decrementing a turn counter
- outputting an unwrapped angle that can exceed 360° or go below 0°

```text
absolute angle:    350° -> 355° ->   2° ->   8°
continuous angle:  350° -> 355° -> 362° -> 368°
```

### Important Reliability Notes

Continuous angle tracking is reliable only if the output shaft moves less than half a revolution between two valid encoder readings.

For a 14-bit encoder:

```text
counts per revolution = 16384
half revolution       = 8192 counts
```

If the shaft moves more than 180 degrees between two consecutive valid samples, the software cannot reliably determine the direction of the wrap-around.

Recommended precautions:

- sample the encoder significantly faster than the maximum expected output shaft speed
- update the continuous angle only after a valid encoder read
- do not update the continuous angle after parity errors or AS5048A error flags
- keep the continuous counter state inside the encoder object, not in local static variables
- use a wide integer type, such as `int64_t`, for internal continuous count tracking

## Development Roadmap

Planned steps:

- [x] Define pin assignment
- [x] Initialize USB CDC console
- [x] Initialize SPI bus for AS5048A
- [x] Implement AS5048A basic reading
- [x] Add parity and error flag checking
- [x] Add WS2812 diagnostic LED state machine
- [x] Add continuous angle computation for multi-turn tracking
- [x] Add persistent parameter storage and console commands
- [x] Add TMC2209 UART configuration
- [x] Add motor test command
- [ ] Add encoder offset calibration
- [ ] Add angle normalization helpers
- [ ] Add angle unwrapping diagnostics
- [ ] Add STEP/DIR motion generation
- [ ] Add closed-loop position control
- [ ] Add motion profiles
- [ ] Add safety and fault handling

## Build and Upload

Build the firmware:

```bash
pio run
```

Upload to the board:

```bash
pio run --target upload
```

Open the serial monitor:

```bash
pio device monitor
```

Or explicitly:

```bash
pio device monitor --baud 115200 --port /dev/ttyACM0
```

## Status

Work in progress.

The current firmware focuses on:

- hardware resource assignment
- diagnostic LED handling
- reliable AS5048A encoder reading with parity and error checks
- continuous encoder angle tracking over multiple turns
- USB CDC command console with persistent parameter storage
- TMC2209 UART communication, probe, and safe register configuration
- motor test mode for controlled driver enable/disable

Motor position control and motion profiles are still under development.

## License

MIT
