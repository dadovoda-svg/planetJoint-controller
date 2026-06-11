# Planner API / zeroed-coordinate fix

This build fixes the coordinate mismatch introduced while adding planner-style movement APIs.

## Important convention

The position controller now always works in **zeroed real joint degrees**:

```text
joint_position_deg = joint_unrolled_deg - encoder_zero_deg
```

Therefore:

- `pos <deg>` targets a position relative to the current logical zero.
- `move <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]` also targets a position relative to the current logical zero.
- `zero` resets the logical zero and resets the PID/S-curve controller to `0.0 deg`.
- trace fields `joint_zeroed_deg`, `target_deg`, `ref_deg` are all in the same coordinate system.

## Commands

### Human/default move

Uses runtime parameters `vmax`, `amax`, `sct`, `outmax`:

```text
pos 1
pos 0
pos -1
```

### Planner-style move

Uses explicit motion limits for a single command:

```text
move <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]
```

Examples:

```text
move 1 2 6
move 0 2 6
move 5 4 10 0.150
```

## Safety defaults

`MOTOR_DIRECTION_SIGN` is set to `-1.0f`, as found during real hardware tests.

The planner API clamps requested speed to a conservative hard limit:

```cpp
PLANNER_HARD_VMAX_LIMIT_DEG_S = 90.0f
```

This remains below the observed step-loss region around 15 deg/s.
