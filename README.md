# Planetary Joint Controller

Firmware for a planetary gearbox actuator using an ESP32-S3 controller board.

This project implements a compact closed-loop joint controller based on:

- an AS5048A 14-bit magnetic absolute encoder over SPI
- a Trinamic TMC2209 stepper motor driver configured over UART
- a stepper motor driving a planetary gearbox actuator
- a PID + S-curve position controller running on an ESP32-S3

The firmware is built with the Arduino framework and PlatformIO.

## Key Features

- AS5048A 14-bit SPI encoder support
- Raw encoder angle reading
- Continuous multi-turn angle tracking
- Mechanical angle scaling between encoder rotation and real joint motion
- SPI parity and AS5048A error flag validation
- TMC2209 UART communication and safe register configuration
- STEP/DIR motor velocity control
- Closed-loop joint position control
- PID + S-curve motion controller
- Runtime tunable PID, feed-forward, motion profile and settling parameters
- `stdeg` calibration routine for estimating motor microsteps per real joint degree
- Direct velocity test mode in real joint degrees/second
- Direct STEP/DIR pulse test mode
- USB CDC serial console with interactive commands
- Persistent configuration storage in ESP32 NVS
- Selectable trace modes for tuning, overshoot analysis and diagnostics
- WS2812 RGB LED diagnostic state machine
- Modular source layout

## Hardware Target

- Board: Waveshare ESP32-S3 Zero
- MCU: ESP32-S3
- Framework: Arduino
- Build system: PlatformIO

## Mechanical Model

The encoder does not map 1:1 to real joint movement.

In the current mechanism:

```text
360 encoder degrees = 15.6 real joint degrees
```

Therefore the firmware distinguishes between:

- encoder angle: the raw AS5048A angle modulo 360 degrees
- joint angle: the real scaled joint angle, unwrapped across multiple turns

In the encoder API:

```text
readDegrees()           -> encoder angle modulo 360 degrees
readContinuousDegrees() -> real unrolled joint angle in degrees
```

The closed-loop controller, `pos` command, `zero` command, trace output and `stdeg` calibration all work in **real joint degrees**.

## Project Layout

```text
platformio.ini
src/
├── main.cpp
├── SCurvePosVelController.h
├── as5048a.h
├── as5048a.cpp
├── led_status.h
├── led_status.cpp
├── params.h
├── SerialConsole.h
├── SerialConsole.cpp
├── Tmc2209Driver.h
└── Tmc2209Driver.cpp
```

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
- UART0 is initialized for future use
- Parameters are initialized, loaded from NVS and printed at boot
- TMC2209 is configured safely before motor enable
- Motor GPIO is initialized in a safe disabled state
- Encoder reads update the real unrolled joint angle
- The PID + S-curve controller commands motor velocity in real joint deg/s
- TMC2209 velocity commands are converted using `stdeg`

## Motor Direction

Real hardware tests showed that the motor polarity is inverted.

The current firmware uses:

```cpp
static constexpr float MOTOR_DIRECTION_SIGN = -1.0f;
```

This is important for closed-loop control. A wrong sign would turn negative feedback into positive feedback.

## Supported Serial Commands

| Command | Description |
|---|---|
| `help` | List available commands |
| `get <key>` | Read a stored parameter |
| `set <key> <value>` | Write a parameter value in RAM |
| `load` | Load parameters from NVS |
| `save` | Save parameters to NVS |
| `export` | Export current parameters over serial |
| `import` | Import parameters from serial input |
| `cancel` | Cancel import mode |
| `zero` | Set current real joint position as zero |
| `pos <deg>` | Move to target position relative to zero |
| `stop` | Stop all motion |
| `servo` | Print controller status |
| `trace` | Toggle trace output using the current trace mode |
| `trace on` | Enable trace output |
| `trace off` | Disable trace output |
| `trace <mode>` | Enable trace output and select trace mode |
| `go <deg/s>` | Toggle direct velocity test in real joint deg/s |
| `step <steps>` | Toggle direct STEP/DIR pulse test |
| `calstdeg [deg]` | Estimate `stdeg` by moving the joint and measuring encoder feedback |
| `reboot` | Reboot the board |

## Runtime Parameters

All parameter keys are limited to 6 characters.

### PID and Feed-forward

| Key | Meaning | Default |
|---|---|---:|
| `kp` | Proportional gain | `0.8` |
| `ki` | Integral gain | `0.0` |
| `kd` | Derivative gain on measured velocity | `0.02` |
| `ffv` | Velocity feed-forward gain | `1.0` |
| `ilim` | Absolute integrator limit | `0.0` |

### Motion Profile and Output Limits

| Key | Meaning | Default |
|---|---|---:|
| `vmax` | S-curve reference max velocity, deg/s | `2.0` |
| `amax` | S-curve reference max acceleration, deg/s² | `6.0` |
| `sct` | S-curve acceleration ramp time, seconds | `0.150` |
| `outmax` | Final command velocity clamp, deg/s | `2.5` |

`vmax` limits the internal S-curve reference velocity.

`outmax` limits the final velocity command sent to the motor after PID and feed-forward processing.

Normally `outmax` should be slightly higher than `vmax`, so the controller has a small correction margin without being overly aggressive.

### Settling, Deadband and Velocity Estimate

| Key | Meaning | Default |
|---|---|---:|
| `ptol` | Settled position tolerance, degrees | `0.08` |
| `vtol` | Settled velocity tolerance, deg/s | `0.15` |
| `dbent` | Deadband enter threshold, degrees | `0.05` |
| `dbext` | Deadband exit threshold, degrees | `0.12` |
| `dbvel` | Deadband velocity threshold, deg/s | `0.20` |
| `vtau` | Measured velocity low-pass filter tau, seconds | `0.050` |

### Motor and Driver

| Key | Meaning |
|---|---|
| `stdeg` | Motor microsteps per real joint degree |
| `ustep` | TMC2209 microstep setting |
| `irun` | TMC2209 run current scale |
| `ihold` | TMC2209 hold current scale |

The velocity conversion is:

```text
microsteps_per_second = joint_deg_per_second * stdeg
```

## `stdeg` Calibration

The `calstdeg` command estimates the conversion between real joint degrees and motor microsteps.

```text
calstdeg
calstdeg 30
calstdeg -30
```

The calibration routine:

1. takes exclusive control of the motor
2. stops normal velocity or position control
3. reads the real unrolled joint angle
4. moves the motor using the current `stdeg` value as seed
5. measures the actual real joint movement
6. updates `stdeg` in RAM

Use `save` after a successful calibration to persist the new value.

## Trace Modes

Trace output always uses this plotter-friendly format:

```text
@name:value,name:value,name:value
```

Available modes:

| Command | Fields | Purpose |
|---|---|---|
| `trace 0` | Full diagnostic trace | General debug |
| `trace 1` | `joint_zeroed_deg`, `target_deg` | Overshoot and settling view |
| `trace 2` | `joint_zeroed_deg`, `target_deg`, `cmd_deg_s`, `meas_vel` | Position and velocity tuning |
| `trace 3` | `err_deg`, `cmd_deg_s`, `meas_vel` | PID-focused tuning |
| `trace 4` | `joint_zeroed_deg`, `ref_deg`, `target_deg`, `ref_vel`, `cmd_deg_s` | S-curve/reference tracking |

Recommended overshoot check:

```text
zero
trace 1
pos 1
pos 0
pos -1
pos 0
```

If the position response looks suspicious, switch to:

```text
trace 2
```

or:

```text
trace 3
```

## Encoder Output and Angle Tracking

The AS5048A reports an absolute position inside one encoder revolution. The firmware computes a continuous angle by:

- tracking the previous raw encoder value
- computing delta on each valid read
- detecting wrap-around at half revolution
- incrementing or decrementing a turn counter
- scaling the continuous encoder count to real joint degrees

Example for encoder degrees:

```text
absolute angle:    350° -> 355° ->   2° ->   8°
continuous angle:  350° -> 355° -> 362° -> 368°
```

Then the firmware scales encoder revolutions to real joint movement:

```text
joint_degrees = continuous_encoder_counts * 15.6 / 16384
```

### Important Reliability Notes

Continuous angle tracking is reliable only if the encoder moves less than half a revolution between two valid encoder readings.

For the AS5048A:

```text
counts per encoder revolution = 16384
half encoder revolution       = 8192 counts
```

If the encoder moves more than 180 encoder degrees between two consecutive valid samples, the firmware cannot reliably determine the direction of wrap-around.

Recommended precautions:

- sample the encoder significantly faster than the maximum expected output shaft speed
- update the continuous angle only after a valid encoder read
- do not update the continuous angle after parity errors or AS5048A error flags
- keep the continuous counter state inside the encoder object
- use a wide integer type, such as `int64_t`, for internal continuous count tracking

## LED Status

| State | Behavior |
|---|---|
| BOOT | Blue fixed |
| READY | Green fixed |
| ENCODER_OK | Dim green |
| ENCODER_ERROR | Yellow blinking |
| FAULT | Red fast blinking |

The tested Waveshare ESP32-S3 Zero onboard WS2812 uses RGB channel order in this project build. If colors appear swapped on another board revision, check the NeoPixel order setting.

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

## Recommended Bring-up Sequence

Start with the default conservative parameters.

The real joint was observed to start losing steps above roughly 15 deg/s. The default controller values are intentionally much lower.

Suggested first sequence:

```text
get
zero
trace 1
pos 1
pos 0
pos -1
pos 0
servo
```

Then tune one parameter at a time:

```text
set kp 1.0
set kd 0.03
set vmax 3.0
set amax 8.0
pos 1
```

Only after stable tests:

```text
save
```

## Development Roadmap

Completed:

- [x] Define pin assignment
- [x] Initialize USB CDC console
- [x] Initialize UART0 for future use
- [x] Initialize SPI bus for AS5048A
- [x] Implement AS5048A basic reading
- [x] Add parity and error flag checking
- [x] Add WS2812 diagnostic LED state machine
- [x] Add continuous angle computation for multi-turn tracking
- [x] Add mechanical encoder-to-joint angle scaling
- [x] Add persistent parameter storage and console commands
- [x] Add TMC2209 UART configuration
- [x] Add direct motor velocity test mode
- [x] Add direct STEP/DIR pulse test mode
- [x] Add `stdeg` calibration routine
- [x] Add closed-loop PID + S-curve position control
- [x] Add runtime PID and motion parameter tuning
- [x] Add selectable trace modes

Planned:

- [ ] Add encoder offset calibration workflow
- [ ] Add configurable software position limits
- [ ] Add hard fault latch and reset workflow
- [ ] Add more complete safety handling
- [ ] Add homing/calibration procedures if required by the final mechanics
- [ ] Add consolidated release documentation

## Status

Work in progress.

The current firmware is a functional closed-loop prototype. It supports real joint position control using AS5048A feedback, TMC2209 velocity control, persistent runtime tuning, trace-based analysis and conservative safe defaults for hardware bring-up.

## License

MIT
