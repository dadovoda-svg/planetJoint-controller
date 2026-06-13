# Active Servo Hold (`shold`)

This build adds the persistent parameter:

```text
shold
```

`shold` controls whether the position controller remains active after a position move reaches the target.

```text
shold = 0   default: stop the position controller at target and enter IDLE
shold = 1   keep the position controller active at target
```

## Why it exists

`mhold` and `shold` are different features.

`mhold` only controls the TMC2209 bridge policy after a completed move:

```text
mhold = 0   disable the motor bridge at target
mhold = 1   keep the motor bridge enabled at hold current
```

With `mhold = 1`, the motor is energized, but the firmware does not actively correct a position error after the move has completed.

`shold = 1` keeps the PID/S-curve controller running after the target has been reached. While the joint is still at target, the commanded velocity is zero. If the joint is moved by an external force, the controller commands motion back toward the stored target.

## Recommended use

For a real joint that can be bumped or back-driven, use:

```text
set shold 1
save
```

Keep `kp`, `kd`, `outmax`, `vmax`, and `amax` conservative during the first hardware tests.

## Runtime behavior

When `shold = 0`:

1. The move reaches the target.
2. The controller commands zero velocity.
3. The firmware enters `IDLE`.
4. The motor bridge follows `mhold`.

When `shold = 1`:

1. The move reaches the target.
2. The controller commands zero velocity.
3. The firmware remains in `POSITION` mode.
4. If the measured position moves away from target, the controller corrects it.
5. The `stop` command still disables the motor bridge and exits active hold.

## Status output

The `servo` command reports:

```text
shold=0|1
```

When active hold is enabled and a move settles, the log reports:

```text
[NFO] Move complete zeroed=<value> deg servo-hold active
```
