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
