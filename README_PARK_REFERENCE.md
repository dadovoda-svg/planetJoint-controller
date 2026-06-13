# Multi-turn park reference

The park sensor is connected to GPIO3 and is active LOW. The firmware enables
`INPUT_PULLUP` on this pin.

## Parameters

- `pkdir = 0`: absolute-encoder mode. No park operation is required.
- `pkdir = +1`: search the park sensor in the positive joint direction.
- `pkdir = -1`: search the park sensor in the negative joint direction.
- `pkvel`: park velocity in real joint degrees per second.
- `pkenc`: final absolute encoder angle, modulo 360 degrees.
- `pkpos`: known logical joint position at the completed park reference.

The park acceleration is fixed in the source code by
`PARK_ACCEL_DEG_S2`.

The safety timeouts are fixed in the source code:

- `PARK_RELEASE_TIMEOUT_MS`: maximum time allowed to leave an already-active sensor window.
- `PARK_SEARCH_TIMEOUT_MS`: maximum time allowed to find the falling edge.
- `PARK_ALIGN_TIMEOUT_MS`: maximum time allowed to reach the final `pkenc` angle after the falling edge.

## Absolute-encoder mode

When `pkdir` is zero, the encoder is treated as directly absolute for the joint.
At startup the controller target is initialized at the measured logical
position and motion is enabled immediately. The `park` command performs no
motion and prints an informational diagnostic.

## Multi-turn mode

When `pkdir` is +1 or -1, motion remains locked after every startup until the
`park` command completes. The encoder revolution counter is not stored in NVM.

The park sequence is:

1. Read the active-LOW park sensor.
2. If the sensor is already active when `park` starts, move in the opposite
   direction to `pkdir` until the sensor is released. This makes the capture
   deterministic and avoids using an arbitrary point inside the active sensor
   window.
3. Once the sensor is released, or if it was already inactive at the start,
   move only in the fixed `pkdir` direction until a falling edge is detected.
4. After the falling edge, continue in the configured park direction toward the
   exact modulo encoder angle `pkenc`, using controlled acceleration and
   deceleration.
5. Assign the unwrapped joint position corresponding to `zoff + pkpos`.
6. Rebase the controller and target at the referenced position and unlock
   normal motion commands.

If the release, search, or final alignment phase exceeds its timeout, the park
sequence is aborted, the driver is stopped, and the joint remains unreferenced.

The logical zero remains defined by `zoff`. The `zero` command changes `zoff`
without resetting the encoder multi-turn tracking. Use `save` to persist the
updated parameters.
