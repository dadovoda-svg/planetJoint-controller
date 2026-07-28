# Blended Move API

This build adds a second planner-style motion primitive intended for smooth target updates while a joint is already moving.

## Commands

### `move`

```text
move <target_deg> <vmax_deg_s> <amax_deg_s2>
```

Normal planner-style move. It stops/reinitializes the local reference from the
current measured position and builds a rest-to-rest analytic quintic.

Use it for isolated movements, tests, and safe restart from a known state.

### `moveb`

```text
moveb <target_deg> <vmax_deg_s> <amax_deg_s2>
```

Blended planner-style move. If the controller is already in `POSITION` mode,
it tries to construct a new analytic quintic from the current reference state.

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
joint_zeroed_deg, ref_deg, target_deg, ref_vel, ref_acc, cmd_deg_s
```

When `moveb` is sent during a move in the same direction, `ref_vel` should not drop to zero unless required by the new target geometry.

## Notes

`moveb` does not mean "always blend". The candidate polynomial is sampled and
accepted only when it is finite, monotonic and respects the velocity,
acceleration and mechanical-limit envelopes. Reverse, too-close or infeasible
retargets use a safe rest-to-rest replan from the measured position.
