# Planetary Joint Controller - AS5048A PID + S-curve Planner Build

This firmware variant integrates the `SCurvePosVelController` with the AS5048A-based Planetary Joint Controller and adds a first reusable planner module.

The firmware is intentionally conservative for real hardware tests. The current validated baseline includes:

- AS5048A absolute encoder feedback
- TMC2209 UART-configured stepper driver
- PID + S-curve position/velocity control
- runtime-tunable parameters stored in NVS
- selectable trace output for tuning
- header-only logger with configurable log level
- `JointPlanner` module split into `JointPlanner.h` / `JointPlanner.cpp`
- planner-style `move` and intelligent retarget command `moveb`

This build has been tested on real hardware with normal moves, blended retargets, forced direction reversals, and `stop` followed by recovery move.

## Safety Defaults

The real joint was observed to lose steps above about 15 deg/s during early tests. This build starts much lower:

- servo max velocity: `2.0 deg/s`
- servo output clamp: `2.5 deg/s`
- acceleration: `6.0 deg/s^2`
- S-curve jerk time: `0.150 s`
- `ki = 0.0` by default

Start with very small moves:

```text
zero
pos 1
pos 0
pos -1
pos 0
```

If the joint moves away from the target instead of toward it, run:

```text
stop
```

Then check the compile-time motor polarity constant:

```cpp
static constexpr float MOTOR_DIRECTION_SIGN = -1.0f;
```

According to the real hardware tests performed on this joint, the correct value is currently:

```cpp
static constexpr float MOTOR_DIRECTION_SIGN = -1.0f;
```

## Important Units

The encoder does not map 1:1 to real joint motion.

```text
360 encoder degrees = 15.6 real joint degrees
```

Therefore:

- `readDegrees()` reports encoder angle modulo 360 degrees.
- `readContinuousDegrees()` reports real unrolled joint angle in degrees.
- Motion commands such as `pos`, `move`, and `moveb` use real joint degrees relative to the current zero.

The conversion from real joint speed to step generation is:

```text
microsteps_per_second = joint_deg_per_second * stdeg
```

## Architecture

The movement-related planner code has been moved out of `main.cpp` and into a dedicated module:

```text
src/JointPlanner.h
src/JointPlanner.cpp
```

The goal of this refactoring is to keep `main.cpp` focused on hardware wiring, initialization, console setup, and the fast control loop, while keeping the planner logic isolated and easier to extend.

At this stage the refactoring is intentionally conservative:

- `main.cpp` still owns the low-level hardware objects.
- `main.cpp` still runs the real-time servo update.
- `JointPlanner` owns high-level move request handling.
- The existing console commands remain compatible.
- The planner receives the previously unused serial port in its constructor.

Example initialization pattern:

```cpp
JointPlanner planner(SerialFuture);

void setup()
{
  // ...
  planner.begin(UART0_BAUD);
}

void loop()
{
  // ...
  planner.update();
}
```

The serial port passed to the planner is reserved for future integration with an external motion planner or host controller.

## Files

```text
src/
├── main.cpp
├── JointPlanner.h
├── JointPlanner.cpp
├── Logger.h
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

## Commands

| Command | Description |
|---|---|
| `zero` | Set current real joint position as zero |
| `pos <deg>` | Move to target position relative to zero using the default position command |
| `move <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]` | Planner-style move with explicit speed, acceleration, and optional S-curve time |
| `moveb <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]` | Intelligent retarget command; blends only when safe, otherwise safe-replans |
| `stop` | Stop all motion and disable motor bridge |
| `servo` | Print controller status |
| `trace` | Toggle periodic trace output |
| `trace <mode>` | Enable trace and select trace mode |
| `go <deg/s>` | Toggle direct velocity test in real joint deg/s |
| `step <steps>` | Toggle direct STEP/DIR pulse test |
| `calstdeg [deg]` | Calibrate `stdeg` using a measured real joint move |
| `get <key>` | Read parameter |
| `set <key> <value>` | Set parameter in RAM |
| `save` | Save parameters to NVS |
| `load` | Load parameters from NVS |

## Parameters

| Parameter | Description |
|---|---|
| `kp` | Position proportional gain |
| `ki` | Integral gain, default 0 |
| `kd` | Derivative on measurement |
| `ffv` | Velocity feed-forward gain |
| `ilim` | Integral limit |
| `vmax` | Default servo/planner max velocity in deg/s |
| `amax` | Default acceleration in deg/s^2 |
| `sct` | Default S-curve jerk time in seconds |
| `outmax` | Servo output clamp in deg/s |
| `ptol` | Position tolerance for settled detection |
| `vtol` | Velocity tolerance for settled detection |
| `dbent` | Deadband enter threshold |
| `dbext` | Deadband exit threshold |
| `dbvel` | Deadband velocity threshold |
| `vtau` | Velocity filter time constant |
| `ustep` | TMC2209 microstep resolution |
| `irun` | TMC2209 run current scale |
| `ihold` | TMC2209 hold current scale |
| `stdeg` | Motor microsteps per real joint degree |
| `loglvl` | Logger verbosity level |

## Logger

The firmware uses a header-only logger with four levels:

| `loglvl` | Meaning |
|---:|---|
| `0` | Logging disabled |
| `1` | Errors only, prefixed with `[ERR]` |
| `2` | Errors and info, prefixed with `[NFO]` |
| `3` | Errors, info, and debug, prefixed with `[DBG]` |

Example:

```text
set loglvl 2
save
```

The default validated runtime logging level is normally `2`, which keeps useful movement status messages without flooding the serial console.

Trace output is separate from logger output. Trace lines are intended for the serial plotter and are emitted with the `@` prefix.

## STDEG Calibration

Command:

```text
calstdeg [deg]
```

Default target is 30 degrees. The routine takes exclusive motor control, moves the joint by the requested calibration angle using the current `stdeg` as seed, measures the real unrolled joint angle, and updates `stdeg` in RAM.

Use:

```text
save
```

to persist the calibrated value.

## Trace Modes

This build adds selectable trace outputs for tuning and overshoot analysis.

```text
trace          toggle trace on/off
trace on       enable trace
trace off      disable trace
trace 0        full diagnostic trace
trace 1        joint_zeroed_deg, target_deg
trace 2        joint_zeroed_deg, target_deg, cmd_deg_s, meas_vel
trace 3        err_deg, cmd_deg_s, meas_vel
trace 4        joint_zeroed_deg, ref_deg, target_deg, ref_vel, cmd_deg_s
```

For planner validation and retarget tests, `trace 4` is the most useful mode:

```text
trace 4
```

It shows:

```text
@joint_zeroed_deg:<...>,ref_deg:<...>,target_deg:<...>,ref_vel:<...>,cmd_deg_s:<...>
```

## Move and MoveB

### `move`

Syntax:

```text
move <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]
```

`move` starts a planner-style point-to-point move toward the requested target. It is the right command when the previous trajectory should not be blended.

Example:

```text
move 20 5 5
move 0 5 5
```

### `moveb`

Syntax:

```text
moveb <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]
```

`moveb` means intelligent retargeting, not unconditional blending.

When a new target is compatible with the current direction of motion, `moveb` keeps the current S-curve reference velocity and creates a smoother continuation.

When the new target is behind the current motion direction, or when reaching it requires a direction reversal, `moveb` automatically switches to safe-replan mode.

This is intentional and was validated on real hardware.

Example log for safe replanning:

```text
[NFO] MOVEB target=-45.000 deg current=22.158 deg vmax=8.000 deg/s amax=4.000 deg/s2 mode=safe-replan
```

Example log for true blending:

```text
[NFO] MOVEB target=45.000 deg current=-14.496 deg vmax=8.000 deg/s amax=4.000 deg/s2 mode=blend
```

In short:

```text
moveb = blend when safe, safe-replan when not safe
```

This behavior avoids aggressive velocity discontinuities during retargets and keeps the motion predictable even during hostile command sequences.

## Suggested Validation Tests

Basic move test:

```text
stop
zero
trace 4
pos 1
pos 0
pos -1
pos 0
```

Forward retarget test:

```text
stop
zero
trace 4
moveb 30 8 4
moveb 35 8 4
moveb 40 8 4
moveb 45 8 4
```

Direction reversal stress test:

```text
stop
zero
trace 4
moveb 45 8 4
moveb -45 8 4
moveb 45 8 4
moveb -45 8 4
```

Stop and recovery test:

```text
stop
move 0 5 5
```

Expected behavior:

- `moveb` reports `mode=blend` only when the retarget is kinematically safe.
- `moveb` reports `mode=safe-replan` for reversals or unsafe retargets.
- `ref_deg` remains continuous.
- `ref_vel` changes sign progressively.
- `stop` freezes the target at the current reference position.
- A subsequent `move 0 5 5` returns cleanly to zero.

## Notes for External Planner Integration

The current `JointPlanner` module is the first step toward integration with an external planner.

The dedicated serial port passed to the planner constructor is reserved for receiving planner commands independently from the interactive USB console.

Current status:

- console commands still call the same movement API;
- external planner serial infrastructure is initialized;
- planner `update()` is called from the main loop;
- command grammar can be extended inside `JointPlanner` without growing `main.cpp`.

Future likely extensions:

- binary or line-based external planner protocol;
- queue of motion segments;
- planner acknowledgements;
- busy/idle status reporting;
- synchronization command equivalent to `M400`;
- multi-axis generalization.
