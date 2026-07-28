# Planner API / zeroed-coordinate fix

This build fixes the coordinate mismatch introduced while adding planner-style movement APIs.

## Important convention

The position controller now always works in **zeroed real joint degrees**:

```text
joint_position_deg = joint_unrolled_deg - encoder_zero_deg
```

Therefore:

- `pos <deg>` targets a position relative to the current logical zero.
- `move <target_deg> <vmax_deg_s> <amax_deg_s2>` also targets a position relative to the current logical zero.
- `zero` resets the logical zero and resets the PID/quintic controller to `0.0 deg`.
- trace fields `joint_zeroed_deg`, `target_deg`, `ref_deg` are all in the same coordinate system.

## Commands

### Human/default move

Uses runtime parameters `vmax`, `amax`, and `outmax`:

```text
pos 1
pos 0
pos -1
```

### Planner-style move

Uses explicit motion limits for a single command:

```text
move <target_deg> <vmax_deg_s> <amax_deg_s2>
```

Examples:

```text
move 1 2 6
move 0 2 6
move 5 4 10
```

## Safety defaults

Motor polarity is configured through the persistent `mdir` parameter. Its
default is `+1`; only `-1` and `+1` are accepted.

The planner API clamps requested speed to a conservative hard limit:

```cpp
PLANNER_HARD_VMAX_LIMIT_DEG_S = 90.0f
```

This remains below the observed step-loss region around 15 deg/s.
