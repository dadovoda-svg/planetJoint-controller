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
2. If the sensor is inactive, move only in the fixed `pkdir` direction until a
   falling edge is detected.
3. If the sensor is already active when `park` starts, do not continue the
   search motion. Continuing in that direction could damage the mechanism.
   The active sensor window already identifies the correct encoder revolution,
   so alignment to `pkenc` starts immediately.
4. Move in the configured park direction to the exact modulo encoder angle
   `pkenc`, using controlled acceleration and deceleration.
5. Assign the unwrapped joint position corresponding to `zoff + pkpos`.
6. Rebase the controller and target at the referenced position and unlock
   normal motion commands.

The logical zero remains defined by `zoff`. The `zero` command changes `zoff`
without resetting the encoder multi-turn tracking. Use `save` to persist the
updated parameters.
