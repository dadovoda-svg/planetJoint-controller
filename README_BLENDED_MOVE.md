# Blended Move API

This build adds a second planner-style motion primitive intended for smooth target updates while a joint is already moving.

## Commands

### `move`

```text
move <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]
```

Normal planner-style move. It stops/reinitializes the local S-curve reference from the current measured position before applying the new target.

Use it for isolated movements, tests, and safe restart from a known state.

### `moveb`

```text
moveb <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]
```

Blended planner-style move. If the controller is already in `POSITION` mode, it updates the target and limits without resetting the internal S-curve reference state.

It preserves:

- current reference position
- current reference velocity
- current reference acceleration

This avoids the artificial stop/restart behavior when a new target is issued in the same direction while the joint is moving.

If no position move is active, `moveb` behaves like a normal move start: it initializes the reference from the current zeroed joint position and then applies the target.

## Suggested Test

```text
stop
zero
trace 4
move 10 3 8
moveb 20 3 8
moveb 30 3 8
```

With `trace 4`, watch:

```text
joint_zeroed_deg, ref_deg, target_deg, ref_vel, cmd_deg_s
```

When `moveb` is sent during a move in the same direction, `ref_vel` should not drop to zero unless required by the new target geometry.

## Notes

`moveb` does not mean the joint can never stop. If the new target is behind the current motion direction, or too close to brake within the requested acceleration, the controller may decelerate, stop, or overshoot slightly before converging. The key point is that the S-curve state is no longer deliberately reset by the command itself.
