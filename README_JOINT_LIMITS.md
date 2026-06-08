# Joint Travel Limits

This build adds configurable joint travel limits in zeroed real joint degrees.

## Parameters

| Key | Meaning | Default |
|---|---|---:|
| `jmin` | Minimum allowed target position | `-170.0` |
| `jmax` | Maximum allowed target position | `170.0` |
| `jtol` | Allowed measured overshoot outside the limit window before fault | `1.0` |

All values are expressed in real joint degrees relative to the software zero set with `zero`.

## Target clipping

The interval `[jmin, jmax]` is used as a clipping window for requested targets.

A command outside the window is accepted, but the target is clipped to the nearest valid limit and an informational warning is logged.

Example:

```text
set jmin -90
set jmax 90
set jtol 1
save

move 120 5 5
```

The last command is internally executed as a move to `+90 deg`.

The same logic applies to:

```text
pos <deg>
move <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]
moveb <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]
```

## Runtime fault window

The measured joint position is allowed to overshoot the software travel window by `jtol` degrees. This is intended to tolerate small PID ringing near the limits.

With the default values:

```text
jmin = -170
jmax =  170
jtol =    1
```

The measured zeroed joint position may temporarily be in this range:

```text
-171 deg <= measured_position <= +171 deg
```

If the measured joint position goes below `jmin - jtol` or above `jmax + jtol`, the firmware latches `MotionMode::FAULT`, commands zero velocity, disables the TMC2209 bridge and blocks further motion commands.

## Notes

`stop` still explicitly disables the motor. It is not intended to hide a real out-of-range mechanical condition.

After changing `jmin`, `jmax` or `jtol`, use:

```text
save
```

to persist the new limits in NVS.
