# JointBus servo-hold status fix

This build fixes the JointBus status semantics when `shold=1`.

## Problem

With servo-hold enabled, the controller intentionally keeps the position control loop armed after the target has been reached, so it can correct small disturbances.

The previous JointBus mapping treated every `MotionMode::POSITION` condition as busy. As a result, after a completed move the compact status could still report:

```text
QSTATUS = BUSY | ENABLED | HOMED
STATUS.state = SETTLING
```

even when position error and measured velocity were already inside the configured settling tolerances.

Example:

```text
position : -0.09 deg
target   :  0.00 deg
velocity :  0.00 deg/s
state    : SETTLING
```

With typical parameters:

```text
ptol  = 0.15 deg
vtol  = 0.20 deg/s
dbent = 0.12 deg
dbext = 0.30 deg
dbvel = 0.25 deg/s
```

this condition should be reported as command completed, not busy.

## Fix

The firmware now distinguishes between:

- the internal position controller still being armed for servo-hold;
- the external JointBus command still being busy.

When `motionMode == MotionMode::POSITION` and `jointCtrl.isSettled()` is true:

```text
STATUS.state = HOLDING
QSTATUS      = DONE | ENABLED | HOMED
```

When `motionMode == MotionMode::POSITION` and `jointCtrl.isSettled()` is false:

```text
STATUS.state = MOVING
QSTATUS      = BUSY | ENABLED | HOMED
```

This keeps active servo-hold behavior unchanged while allowing the external master to correctly detect move completion through `QSTATUS`.

## Notes

No settling/deadband parameters were changed by this fix.

The hardware-specific motor polarity remains:

```cpp
static constexpr float MOTOR_DIRECTION_SIGN = 1.0f;
```
