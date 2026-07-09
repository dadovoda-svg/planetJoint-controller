# JointBus Coordinated Segment Mode

This baseline adds a first coordinated multi-axis movement layer on top of the existing immediate `MOVE` / `MOVEB` commands.

The goal is to let an external arm-level planner prepare one segment on each joint independently, then start all prepared segments with a single broadcast frame.

## Design scope

Initial implementation:

```text
queue capacity: 2 logical levels
  active segment
  one prepared/next segment
```

This is enough for first coordinated blending tests:

1. Prepare segment `N` on all joints, addressed one by one.
2. Start segment `N` with one broadcast frame.
3. While segment `N` is active, prepare segment `N+1` on all joints.
4. Start segment `N+1` with one broadcast frame.

The implementation intentionally avoids deeper queues for now. The protocol exposes queue status fields so a future firmware can increase the internal depth without changing the external model too much.

## Broadcast address

Address `15` is reserved as the JointBus broadcast address:

```text
BROADCAST_ADDRESS = 0x0F
```

Broadcast frames are no-response by design, to avoid RS485 response collisions.

Only these commands are accepted as broadcast frames in this baseline:

```text
START_SEGMENT
ABORT_SEGMENT
EMERGENCY_STOP
```

All other broadcast commands are ignored.

For this reason, normal nodes should use addresses `0..14`. The current six-axis arm layout uses addresses `1..6`, so this is compatible.

## New commands

```text
0x0A PREPARE_MOVEB
0x0B START_SEGMENT
0x0C ABORT_SEGMENT
0x0D QUEUE_STATUS
0x0E EMERGENCY_STOP
0x8D QUEUE_STATUS_RSP
```

Existing commands remain available:

```text
MOVE
MOVEB
HOME
STOP
ZERO
PARK
STATUS
QSTATUS
PING
NOP
```

## PREPARE_MOVEB

Addressed command. It validates and stores one future blended move, but does not start motion.

Payload, 7 bytes:

```text
uint8_t  segment_id       0..254, 255 is reserved
int16_t  target_cdeg      target in centidegrees
uint16_t vmax_cdeg_s      velocity in centidegrees/s
uint16_t amax_cdeg_s2     acceleration in centidegrees/s^2
```

Response:

```text
ACK SEGMENT_PREPARED      segment accepted and stored
ACK CLIPPED_TO_MIN/MAX    target was clipped to local joint limits and stored clipped
NACK QUEUE_FULL           the prepared slot is already occupied
NACK NOT_HOMED            joint is not referenced yet
NACK FAULT_ACTIVE         fault is active
NACK BUSY                 park/calibration/test mode is active
```

The prepared target is clipped locally to `jmin/jmax` exactly like normal `MOVE/MOVEB` commands.

## START_SEGMENT

Payload, 1 byte:

```text
uint8_t segment_id
```

Normal coordinated use is broadcast/no-response:

```text
addr = 15
cmd  = START_SEGMENT
```

Each slave starts its prepared segment only if the prepared segment id matches the received id.

Addressed `START_SEGMENT` is also supported for debugging and returns `ACK` / `NACK`. The arm-level planner should use broadcast start.

Internally, the segment starts through the existing `jointMoveToBlended()` path. This keeps the current per-joint blending behavior.

## ABORT_SEGMENT

Payload, 1 byte:

```text
uint8_t segment_id
```

`segment_id = 255` cancels any prepared segment.

`ABORT_SEGMENT` cancels only the prepared/next segment. It does not stop the currently active motion. Use `STOP` for a recoverable immediate stop, or broadcast `EMERGENCY_STOP` for a latched hard fault stop of all joints.

Normal coordinated use may use broadcast/no-response:

```text
addr = 15
cmd  = ABORT_SEGMENT
```

Addressed use returns `ACK` / `NACK` for debugging.

## QUEUE_STATUS

Addressed command. Payload length is zero.

Response payload, 5 bytes:

```text
uint8_t capacity             currently 2
uint8_t free_prepared_slots  0 or 1
uint8_t active_segment_id    0..254, or 255 = none
uint8_t prepared_segment_id  0..254, or 255 = none
uint8_t flags
```

Queue flags:

```text
bit 0 ACTIVE_VALID
bit 1 PREPARED_VALID
bit 2 ACTIVE_BUSY
```

This is intentionally separate from `QSTATUS`: compact motion polling stays very small, while queue details are available when needed.

## Baud rate selection

The default JointBus baud rate remains:

```text
500000 bps
```

A compile-time switch is available for 921600 bps:

```cpp
#define JOINTBUS_BAUD_921600 1
```

With PlatformIO this can be enabled for a test build using a build flag, for example:

```ini
build_flags =
  -D JOINTBUS_BAUD_921600=1
```

When not defined or set to `0`, the firmware uses 500000 bps.

## Python debug tool

The Python tool under:

```text
examples/rs485_debug_tool/
```

has been updated with:

```text
prepmb <addr> <segment_id> <angle_deg> <vmax_deg_s> <amax_deg_s2>
start <segment_id> [addr]
abortseg <segment_id|all> [addr]
qqueue <addr>
queue <addr>
```

`start <segment_id>` without an address sends broadcast `START_SEGMENT` to address 15 and expects no response.

Example for two joints:

```text
prepmb 1 42 10 8 15
prepmb 2 42 -20 8 15
qqueue 1
qqueue 2
start 42
qstatus 1
qstatus 2
```
