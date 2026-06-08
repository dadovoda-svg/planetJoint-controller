# Planetary Joint Controller

Firmware for a planetary gearbox actuator using an ESP32-S3 controller board.

This project implements a compact closed-loop joint controller based on:

- a compile-time selectable magnetic absolute encoder: AS5048A 14-bit SPI or AS5600 12-bit I2C
- a Trinamic TMC2209 stepper motor driver configured over UART
- a stepper motor driving a planetary gearbox actuator
- a PID + S-curve position controller running on an ESP32-S3
- a dedicated motion/planner module separated from the main firmware wiring

The firmware is built with the Arduino framework and PlatformIO.

## Key Features

- compile-time selectable AS5048A 14-bit SPI or AS5600 12-bit I2C encoder support
- Raw encoder angle reading
- Continuous multi-turn angle tracking
- Mechanical angle scaling between encoder rotation and real joint motion
- SPI parity and AS5048A error flag validation
- TMC2209 UART communication and safe register configuration
- STEP/DIR motor velocity control
- Closed-loop joint position control
- PID + S-curve motion controller
- Runtime tunable PID, feed-forward, motion profile, settling, encoder scale, motor hold and joint travel limit parameters
- `stdeg` calibration routine for estimating motor microsteps per real joint degree
- Direct velocity test mode in real joint degrees/second
- Direct STEP/DIR pulse test mode
- Planner-style position commands: `move` and `moveb`
- Smart retargeting for `moveb`: blend only when safe, otherwise safe replan
- USB CDC serial console with interactive commands
- Dedicated serial interface object passed to the planner constructor for future external planner integration
- Persistent configuration storage in ESP32 NVS
- Header-only logger with selectable runtime log level
- Optional motor hold at target using TMC2209 `IHOLD` current
- Configurable joint travel window with target clipping and out-of-range fault latch
- Selectable trace modes for tuning, overshoot analysis and diagnostics
- WS2812 RGB LED diagnostic state machine
- Modular source layout with motion/planner logic separated from `main.cpp`

## Hardware Target

- Board: Waveshare ESP32-S3 Zero
- MCU: ESP32-S3
- Framework: Arduino
- Build system: PlatformIO

## Mechanical Model

The encoder does not map 1:1 to real joint movement.

In the reference mechanism the default scale is:

```text
360 encoder degrees = 15.6 real joint degrees
```

This value is no longer hardcoded as the only usable mechanical ratio. It is now exposed as the persistent runtime parameter:

```text
jrev
```

`jrev` means **real joint degrees per one full encoder revolution**.

Examples:

```text
get jrev
set jrev 15.6
save
```

Changing `jrev` lets the same firmware adapt to different planetary joint mechanical configurations.

Therefore the firmware distinguishes between:

- encoder angle: the raw AS5048A angle modulo 360 degrees
- joint angle: the real scaled joint angle, unwrapped across multiple turns

In the encoder API:

```text
readDegrees()           -> encoder angle modulo 360 degrees
readContinuousDegrees() -> real unrolled joint angle in degrees
```

The closed-loop controller, `pos`, `move`, `moveb`, `zero`, trace output and `stdeg` calibration all work in **real joint degrees**.

## Project Layout

```text
platformio.ini
README.md
README_TUNING.md
README_PLANNER_API_FIX.md
README_BLENDED_MOVE.md
README_MOVEB_RETARGET_FIX.md
README_PLANNER_REFACTOR.md
README_JOINT_LIMITS.md

src/
├── main.cpp
├── JointPlanner.h
├── JointPlanner.cpp
├── SCurvePosVelController.h
├── as5048a.h
├── as5048a.cpp
├── led_status.h
├── led_status.cpp
├── logger.h
├── params.h
├── SerialConsole.h
├── SerialConsole.cpp
├── Tmc2209Driver.h
└── Tmc2209Driver.cpp
```

### Main / Planner Split

`main.cpp` remains responsible for low-level firmware ownership and hardware wiring:

- serial ports
- SPI and encoder instance
- TMC2209 driver instance
- PID/S-curve controller instance
- persistent parameters
- LED state machine
- console setup
- periodic servo loop

The motion/planner-facing API has been moved to:

```text
src/JointPlanner.h
src/JointPlanner.cpp
```

The planner module currently owns the higher-level motion commands:

- `jointMoveTo(...)`
- `jointMoveToBlended(...)`
- `moveJointToDeg(...)`
- `jointStop()`

The current refactor is intentionally conservative: the planner module still binds to the already tested low-level objects through existing firmware symbols instead of taking ownership of all hardware. This keeps the validated hardware behavior stable while making the motion layer easier to extend.

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
| Reserved / planner serial | GPIO1, GPIO2, GPIO3 |

## Firmware Behavior

- `Serial` is used for the USB CDC console at 115200 baud
- `Serial2` is used for TMC2209 UART communication at 115200 baud
- HSPI is used for AS5048A encoder communication
- the additional UART/serial object reserved for future use is now passed to the planner constructor
- parameters are initialized, loaded from NVS and printed at boot
- TMC2209 is configured safely before motor enable
- motor GPIO is initialized in a safe disabled state
- encoder reads update the real unrolled joint angle
- the PID + S-curve controller commands motor velocity in real joint deg/s
- TMC2209 velocity commands are converted using `stdeg`
- the planner `update()` method is called from the main loop
- the serial console prompt prints a newline after the prompt in normal mode to keep trace/log output clean

Planner object construction follows this model:

```cpp
JointPlanner planner(SerialFuture);
```

and initialization follows:

```cpp
planner.begin(UART0_BAUD);
```

The planner serial interface is reserved for future integration with an external planner or upstream motion coordinator.

## Motor Direction

Real hardware tests showed that the motor polarity is inverted.

The current firmware uses:

```cpp
static constexpr float MOTOR_DIRECTION_SIGN = -1.0f;
```

This is important for closed-loop control. A wrong sign would turn negative feedback into positive feedback.

## Logger

The firmware includes a lightweight header-only logger.

Supported log levels:

| Level | Prefix | Meaning |
|---:|---|---|
| `0` | none | logging disabled |
| `1` | `[ERR]` | errors only |
| `2` | `[NFO]` | errors and informational messages |
| `3` | `[DBG]` | errors, informational messages and debug messages |

The active level is controlled by the persistent runtime parameter:

```text
loglvl
```

Examples:

```text
set loglvl 0
set loglvl 2
save
```

Trace lines are separate from logger lines. Trace lines remain plotter-friendly and start with `@`.

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
| `pos <deg>` | Move to target position relative to zero using the default position command |
| `move <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]` | Planner-style move with explicit motion limits |
| `moveb <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]` | Planner-style smart retarget move |
| `stop` | Stop all motion and disable the motor bridge |
| `servo` | Print controller status |
| `trace` | Toggle trace output using the current trace mode |
| `trace on` | Enable trace output |
| `trace off` | Disable trace output |
| `trace <mode>` | Enable trace output and select trace mode |
| `go <deg/s>` | Toggle direct velocity test in real joint deg/s |
| `step <steps>` | Toggle direct STEP/DIR pulse test |
| `calstdeg [deg]` | Estimate `stdeg` by moving the joint and measuring encoder feedback |
| `reboot` | Reboot the board |

## Planner Motion Commands

### `move`

```text
move <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]
```

`move` starts a normal planner-style move toward the target using the supplied motion limits.

Example:

```text
move 20 5 5
move 0 5 5
```

This command is appropriate for explicit point-to-point moves where preserving the previous reference velocity is not required.

### `moveb`

```text
moveb <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]
```

`moveb` is a smart retarget command.

Important: `moveb` does **not** mean "always blend".

It means:

- blend if the new target is compatible with the current motion direction and can be reached smoothly
- use safe replan if the target requires braking, reversal, or a retarget behind the current motion

Typical log examples:

```text
[NFO] MOVEB target=45.000 deg current=-14.496 deg vmax=8.000 deg/s amax=4.000 deg/s2 mode=blend
[NFO] MOVEB target=-20.000 deg current=20.054 deg vmax=8.000 deg/s amax=4.000 deg/s2 mode=safe-replan
```

This behavior was validated on real hardware with repeated retargeting tests, including intentionally aggressive `+45/-45` direction changes.

### Stop Behavior

`stop` stops the motion and freezes the internal reference near the current joint position.

After a stop, a new command such as:

```text
move 0 5 5
```

starts cleanly from the stopped position.

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

### Joint Travel Limits

All joint limit parameters are expressed in zeroed real joint degrees.

| Key | Meaning | Default |
|---|---|---:|
| `jmin` | Minimum allowed target position | `-170.0` |
| `jmax` | Maximum allowed target position | `170.0` |
| `jtol` | Allowed measured overshoot outside `jmin`/`jmax` before fault latch | `1.0` |

The interval `[jmin, jmax]` is a clipping window for commanded targets. If a command asks for a target below `jmin` or above `jmax`, the firmware does not reject the move; it emits an informational warning and clips the target to the nearest limit.

Example:

```text
set jmin -90
set jmax 90
set jtol 1
save

move 120 5 5     # target is clipped to +90 deg
moveb -120 5 5   # target is clipped to -90 deg
```

During motion, a small PID ringing overshoot is allowed. With the default `jtol=1.0`, the measured zeroed joint position may temporarily reach `jmin - 1.0` or `jmax + 1.0`. If the measured position goes beyond that fault window, the firmware latches `MotionMode::FAULT`, commands zero velocity, disables the driver bridge, and blocks further movement commands.

`stop` still disables the motor, but it does not hide a real out-of-range condition. After a limit fault, check the mechanics before continuing.

### Settling, Deadband and Velocity Estimate

| Key | Meaning | Default |
|---|---|---:|
| `ptol` | Settled position tolerance, degrees | `0.08` |
| `vtol` | Settled velocity tolerance, deg/s | `0.15` |
| `dbent` | Deadband enter threshold, degrees | `0.05` |
| `dbext` | Deadband exit threshold, degrees | `0.12` |
| `dbvel` | Deadband velocity threshold, deg/s | `0.20` |
| `vtau` | Measured velocity low-pass filter tau, seconds | `0.050` |

### Motor, Driver and Hold Behavior

| Key | Meaning | Default |
|---|---|---:|
| `stdeg` | Motor microsteps per real joint degree | `100.0` |
| `ustep` | TMC2209 microstep setting | `16` |
| `irun` | TMC2209 run current scale | `10` |
| `ihold` | TMC2209 hold current scale | `4` |
| `mhold` | Motor behavior after a completed position move: `0` disables the bridge, `1` keeps the driver enabled at hold current | `0` |
| `jrev` | Real joint degrees per one complete encoder revolution | `15.6` |

The velocity conversion is:

```text
microsteps_per_second = joint_deg_per_second * stdeg
```

The encoder-to-joint scale conversion is:

```text
joint_degrees = continuous_encoder_revolutions * jrev
```

For the current reference mechanics:

```text
jrev = 15.6
```

If the mechanical ratio changes, update `jrev`, then save it:

```text
set jrev <joint_degrees_per_encoder_revolution>
save
```

`jrev` must be greater than zero. Invalid values are rejected at runtime. When `jrev` is changed from the console, the firmware stops current motion, applies the new scale, and preserves the current zeroed position as much as possible to avoid a sudden coordinate jump. For a mechanically meaningful setup, it is still recommended to run `zero` after changing the mechanical scale.

### Motor Hold at Target

The default behavior remains conservative:

```text
set mhold 0
```

With `mhold = 0`, a position command enables the TMC2209 driver at the start of the move and disables the motor bridge when the target is reached.

For a loaded joint, this may not be acceptable because the axis may not be able to hold its position with the motor disabled. In that case enable hold mode:

```text
set mhold 1
save
```

With `mhold = 1`, when the target is reached the firmware commands zero velocity and leaves the TMC2209 driver enabled. The TMC2209 then drops to the configured hold current through its `IHOLD/TPOWERDOWN` behavior, so the joint remains energized without staying at full run current.

In this mode the motor is intentionally disabled with:

```text
stop
```

Fault conditions still disable the driver for safety.

| Situation | `mhold=0` | `mhold=1` |
|---|---|---|
| Position move starts | Driver enabled | Driver enabled |
| Target reached | Driver disabled | Driver remains enabled and transitions to hold current |
| `stop` command | Driver disabled | Driver disabled |
| Encoder/controller fault | Driver disabled | Driver disabled |

### Logging

| Key | Meaning | Default |
|---|---|---:|
| `loglvl` | Runtime logger level: 0 off, 1 error, 2 info, 3 debug | `2` |

## Encoder Scale Parameter `jrev`

`jrev` configures the mechanical relationship between the AS5048A encoder and the real joint output.

```text
jrev = real joint degrees per one full 360° encoder revolution
```

Default:

```text
jrev = 15.6
```

This parameter replaces the previous hardcoded internal constant used for:

```text
360 encoder degrees = 15.6 real joint degrees
```

Use it when the same firmware is installed on a joint with a different encoder/magnet/gearbox relationship.

Typical setup sequence:

```text
stop
set jrev 15.6
zero
save
```

After changing `jrev`, review `stdeg` as well: `jrev` changes the measured joint angle scale, while `stdeg` converts commanded joint velocity to motor microsteps. They represent two different parts of the mechanism and both must be coherent.

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

Recommended S-curve and retargeting check:

```text
stop
zero
trace 4
moveb 30 8 4
moveb 35 8 4
moveb 40 8 4
moveb 45 8 4
```

Recommended aggressive safe-replan check:

```text
stop
zero
trace 4
moveb 45 8 4
moveb -45 8 4
moveb 45 8 4
moveb -45 8 4
```

If the position response looks suspicious, switch to:

```text
trace 2
```

or:

```text
trace 3
```

## Serial Console Output Notes

The console prompt is printed on its own line in normal command mode. This avoids mixing the prompt with trace or logger lines.

Current behavior:

```cpp
void SerialConsole::printPrompt()
{
  if (_mode == Mode::Import) {
    _serial.print("import> ");
  } else {
    _serial.print(_prompt);
    _serial.println();
  }
}
```

Import mode intentionally keeps the prompt on the same line:

```text
import>
```

This is useful for manual parameter import and keeps the import workflow compact.

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
joint_degrees = continuous_encoder_counts * jrev / 16384
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
platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.38-1/platform-espressif32.zip
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

If the mechanical load requires the joint to remain energized at the target:

```text
set mhold 1
save
```

Use `stop` when you intentionally want to disable the motor.

Recommended planner validation sequence:

```text
stop
zero
trace 4
moveb -45 8 4
moveb 45 8 4
moveb -45 8 4
moveb 45 8 4
stop
move 0 5 5
```

Expected behavior:

- repeated direction changes should use `mode=safe-replan`
- compatible forward retargets may use `mode=blend`
- `ref_deg` should remain continuous
- `joint_zeroed_deg` should track `ref_deg`
- after `stop`, `target_deg` should be frozen near the current reference position
- a subsequent `move 0 5 5` should return cleanly to zero

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
- [x] Add header-only runtime logger with `loglvl`
- [x] Add planner-style `move` command
- [x] Add smart retarget `moveb` command
- [x] Refactor motion/planner logic from `main.cpp` into `JointPlanner.h/.cpp`
- [x] Validate `moveb` blend and safe-replan behavior on real hardware
- [x] Adjust console prompt formatting to avoid dirty trace/log output
- [x] Add configurable motor hold at target with `mhold`
- [x] Replace hardcoded encoder-to-joint mechanical scale with persistent `jrev` parameter
- [x] Add configurable software joint travel limits with target clipping and fault latch

Planned:

- [ ] Extend the planner serial protocol for external planner integration
- [ ] Add encoder offset calibration workflow
- [ ] Add hard fault latch and reset workflow
- [ ] Add more complete safety handling
- [ ] Add homing/calibration procedures if required by the final mechanics
- [ ] Add consolidated release documentation

## Status

Work in progress.

The current firmware is a functional closed-loop prototype. It supports real joint position control using AS5048A feedback, TMC2209 velocity control, persistent runtime tuning, trace-based analysis, conservative safe defaults for hardware bring-up, runtime logging, configurable encoder-to-joint scaling through `jrev`, configurable joint limits through `jmin`/`jmax`/`jtol`, and a separated motion/planner module ready to be extended toward external planner integration.

The planner refactor has been compiled and tested on real hardware. The currently validated behavior includes:

- normal point-to-point move
- smart retarget move
- blend when the new target is compatible with the current motion
- safe replan when the target requires braking or direction reversal
- clean stop and restart from the stopped position

## License

MIT
