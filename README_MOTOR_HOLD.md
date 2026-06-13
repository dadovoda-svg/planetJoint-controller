# Motor Hold at Target

This build adds the persistent runtime parameter:

```text
mhold
```

Meaning:

```text
mhold = 0   default: disable the TMC2209 motor bridge when a position move completes
mhold = 1   keep the TMC2209 enabled when a position move completes
```

With `mhold = 1`, the firmware commands zero velocity at the target and leaves the driver enabled. The TMC2209 then uses its configured hold current (`ihold`) through the normal `IHOLD/TPOWERDOWN` behavior, so the joint remains energized without staying at full run current.

Use:

```text
set mhold 1
save
```

To intentionally release the motor in hold mode:

```text
stop
```

Fault conditions still disable the driver for safety.

The `servo` command now prints:

```text
hold=0/1
```

and the numeric TMC status value, so it is easier to verify whether the driver is still enabled after a move.


## Active servo hold (`shold`)

`mhold` only controls whether the TMC2209 bridge remains enabled after a move completes.
It does not keep the position controller active.

The separate parameter `shold` enables active position hold at target:

```text
shold = 0   default: after a completed move, stop the controller and enter IDLE
shold = 1   keep the PID/S-curve controller active after the target is reached
```

With `shold = 1`, the firmware keeps `motionMode = POSITION` after the move settles.
The commanded velocity is zero while the joint remains at target, but if the joint is disturbed externally, the controller resumes correcting toward the stored target.

Use:

```text
set shold 1
save
```

`shold` and `mhold` are intentionally separate:

- `mhold` = passive electrical hold / driver bridge policy.
- `shold` = active servo-position hold.

When `shold = 1`, the driver must remain enabled so the controller can correct position errors.
The `stop` command still stops all motion and disables the motor bridge.
