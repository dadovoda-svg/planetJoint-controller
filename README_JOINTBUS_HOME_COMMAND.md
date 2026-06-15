# JointBus HOME command implementation

This baseline implements the JointBus `HOME` command.

## Behavior

`HOME` moves the already-referenced joint to the logical zero position using
fixed motion limits:

```text
target = 0.0 deg
vmax   = 8.0 deg/s
amax   = 15.0 deg/s^2
```

The command returns an immediate ACK when the motion is accepted. The motion is
asynchronous; completion must be checked through `QSTATUS` or `STATUS`.

## Park interaction

`HOME` does not run `PARK` implicitly.

If the firmware is configured for multi-turn park mode (`pkdir` is `+1` or `-1`)
and the park procedure has not completed yet, the joint is not referenced and
`HOME` is rejected:

```text
HOME -> NACK NOT_HOMED
```

This keeps the startup/reference sequence explicit:

```text
PARK -> wait for DONE -> HOME
```

When `pkdir=0`, absolute-encoder mode is selected and park is not required. In
that case `HOME` can be accepted as soon as the firmware has completed its normal
startup initialization.

## Implementation notes

The hook `jointBusHome()` now calls:

```cpp
jointMoveTo(0.0f, 8.0f, 15.0f);
```

It preserves the usual planner checks: referenced state, fault state, encoder
state, driver state, joint limits and controller safety checks.
