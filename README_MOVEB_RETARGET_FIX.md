# MOVEB Retarget Fix

This build fixes two issues in the blended move implementation.

## 1. Do not re-arm TMC2209 while already moving

Previous behavior:

```text
moveb 30 3 5
moveb 45 4 4
[ERR] Unable to arm TMC power stage
ERR blended move rejected
```

The second `moveb` was rejected because the firmware tried to call `armPowerStage()` while the TMC2209 was already enabled.

New behavior:

- if the servo is already in `POSITION` mode and the TMC2209 is already enabled, `moveb` only updates the motion target/profile
- it does not re-arm the TMC power stage
- `ensureDriverEnabled()` now returns success immediately when the driver is already enabled

## 2. Safe fallback when the new target is behind the current velocity

`moveb` now checks:

- current S-curve reference position: `jointCtrl.refPos()`
- current S-curve reference velocity: `jointCtrl.refVel()`
- new target position

If the new target is ahead of the current reference velocity, the command performs a real blend:

```text
mode=blend
```

If the new target requires reversal or is ambiguous, the command falls back to a safe replan from the current measured zeroed position:

```text
mode=safe-replan
```

This prevents the controller from carrying an incompatible reference velocity beyond the new target.

## 3. Trace target fix

Trace output now uses:

```cpp
jointCtrl.target()
```

instead of a cached target variable. This keeps `target_deg` aligned with the real controller target.

## Suggested Test

```text
stop
zero
trace 4
moveb 30 3 5
moveb 45 4 4
moveb 0 4 4
moveb 30 4 4
```

Expected:

- second command should not fail with `Unable to arm TMC power stage`
- same-direction retarget should report `mode=blend`
- reversal retarget should report `mode=safe-replan`
- trace `target_deg` should follow the controller target
