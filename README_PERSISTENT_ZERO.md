# Persistent logical zero (`zoff`)

The logical joint zero is stored in the runtime parameter `zoff`.

## Behavior

- `zero` reads the current absolute encoder position and copies it to `zoff`.
- `zero` applies the new offset immediately and resets the controller position and target to `0.0 deg`.
- `zero` updates RAM only. Run `save` to persist the new offset in NVS.
- At boot, the saved `zoff` value is loaded. The firmware no longer makes the current startup position the logical zero.
- After the first valid encoder reading, the controller target is initialized to the measured zeroed position. Therefore startup does not request a movement.
- If no saved parameter image exists, `zoff` defaults to `0.0 deg`; this is not treated as a fault.
- `load` now reapplies all loaded parameters, including `zoff`, and rebases the target to the current measured position.

## Typical calibration

```text
zero
save
```

After a reboot, the logical position is calculated as:

```text
joint_position = absolute_encoder_position - zoff
```

## Current limitation

This implementation persists one absolute angular offset. It is intended primarily for joints where one encoder revolution corresponds to one joint revolution. Multi-turn mechanical mappings, such as `360 deg` encoder rotation corresponding to `15.6 deg` joint rotation, require additional turn-state or homing information to recover the same multi-turn position after power loss.
