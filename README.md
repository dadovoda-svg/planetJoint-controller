# Planetary Joint Controller

Embedded motor controller and motion control framework for a planetary joint actuator.

The project is based on a stepper motor driving a planetary gearbox.  
The angular position of the gearbox output shaft is measured using an AS5048A 14-bit magnetic absolute encoder.

The target hardware is a Waveshare ESP32-S3 Zero module, developed using the Arduino framework on PlatformIO.

## Features

- AS5048A magnetic encoder support over SPI
- 14-bit absolute angular position reading
- Encoder parity and error flag checking
- WS2812 RGB LED diagnostic state machine
- USB CDC serial console
- UART0 initialized for future use
- UART2 reserved for TMC2209 stepper driver configuration
- Safe motor GPIO initialization
- Modular firmware structure

## Hardware

### Target Board

- Waveshare ESP32-S3 Zero
- ESP32-S3 based module
- Arduino framework
- PlatformIO build system

### Encoder

- AS5048A magnetic absolute encoder
- SPI interface
- 14-bit resolution
- Used to measure the output angle of the planetary gearbox

### Motor Driver

- Trinamic TMC2209 stepper motor driver
- STEP/DIR motion control
- UART configuration interface

The TMC2209 support is planned but not yet implemented.  
At the current stage, the related GPIO and UART resources are initialized but the driver is not configured.

### Diagnostic LED

- WS2812 RGB LED connected to GPIO21
- Controlled using the Adafruit NeoPixel library
- Used to report firmware and encoder status

## Pin Assignment

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

## Serial Interfaces

| Interface | Purpose |
|---|---|
| USB CDC Serial | Debug console |
| UART0 | Reserved for future use |
| UART2 | Reserved for TMC2209 configuration |

## LED States

| State | LED behavior |
|---|---|
| BOOT | Blue fixed |
| READY | Green fixed |
| ENCODER_OK | Dim green |
| ENCODER_ERROR | Yellow blinking |
| FAULT | Red fast blinking |

## PlatformIO Configuration

Example `platformio.ini`:

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

## Current Firmware Structure

Suggested source structure:

```text
src/
├── main.cpp
├── led_status.h
├── led_status.cpp
├── as5048a.h
└── as5048a.cpp
```

## Encoder Output

The firmware periodically reads the AS5048A encoder and prints diagnostic data to the USB serial console.

Example output:

```text
@enc_raw:12345,enc_deg:271.384,enc_ok:1,parity_ok:1,err_flag:0
```

Fields:

| Field | Description |
|---|---|
| `enc_raw` | Raw 14-bit encoder value, from 0 to 16383 |
| `enc_deg` | Converted angle in degrees |
| `enc_ok` | Encoder read status |
| `parity_ok` | SPI response parity check result |
| `err_flag` | AS5048A error flag |

The `@` prefix is intended to make the output easy to filter with serial plotting tools.

## Development Roadmap

Planned steps:

- [x] Define pin assignment
- [x] Initialize USB CDC console
- [x] Initialize SPI bus for AS5048A
- [x] Implement AS5048A basic reading
- [x] Add parity and error flag checking
- [x] Add WS2812 diagnostic LED state machine
- [ ] Add encoder offset calibration
- [ ] Add angle unwrapping / multi-turn tracking
- [ ] Add TMC2209 UART configuration
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
- reliable AS5048A encoder reading
- basic serial diagnostic output

Motor driver configuration and motion control will be added later.

## License

MIT
