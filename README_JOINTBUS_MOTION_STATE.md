# JointBus analytic motion state

This extension lets the master inspect the polynomial that a joint actually
installed after `START_SEGMENT`. Together with `SEGMENT_TIMING` and timed
start, it implements four related functions:

1. expose analytic reference state, phase, duration, and retarget outcome;
2. base the master motion watchdogs on the longest duration reported by the
   six slaves instead of only on a master-side estimate;
3. negotiate and impose one common duration on all six polynomials;
4. schedule their start after one shared relative delay.

The extension keeps JointBus protocol version 0 and uses previously unused
command values. Master and all six slaves must nevertheless be flashed with
the updated firmware before coordinated motion is used, because the planner
now requires a valid `MOTION_STATE_RSP` during start verification.

## Commands

```text
0x10 MOTION_STATE       addressed request, empty payload
0x90 MOTION_STATE_RSP   response, 16-byte payload
```

`MOTION_STATE` is never broadcast: simultaneous replies would collide on the
half-duplex RS485 bus. Invalid request lengths produce `NACK BadLength`.

## Response payload

All multi-byte integers are little-endian.

| Bytes | Type | Scale | Meaning |
|---:|---|---|---|
| 0 | `uint8_t` | - | Active segment ID, or `0xFF` |
| 1 | `uint8_t` | - | Motion-state flags |
| 2..3 | `int16_t` | 0.01 deg | Analytic reference position |
| 4..5 | `int16_t` | 0.01 deg/s | Analytic reference velocity |
| 6..7 | `int16_t` | 0.1 deg/s² | Analytic reference acceleration |
| 8..11 | `uint32_t` | 1 ms | Elapsed time of the installed polynomial |
| 12..15 | `uint32_t` | 1 ms | Total duration of the installed polynomial |

The acceleration uses deci-degrees rather than centidegrees so a signed
16-bit field covers up to approximately 3276 deg/s². Elapsed time stops at the
duration when the analytic phase finishes. Duration remains available while
the follower PID settles.

## Flags

| Mask | Name | Meaning |
|---:|---|---|
| `0x01` | `MSTATE_ACTIVE_VALID` | Byte 0 contains the active coordinated segment |
| `0x02` | `MSTATE_TRAJECTORY_ACTIVE` | The analytic polynomial is still advancing |
| `0x04` | `MSTATE_BLEND_ACCEPTED` | The segment retained the preceding reference position, velocity, and acceleration |
| `0x08` | `MSTATE_SAFE_REPLAN` | The slave rejected continuous retargeting and installed a rest-to-rest polynomial |
| `0x10` | `MSTATE_SERVO_SETTLING` | The polynomial ended but the follower PID has not completed the position command |

`MSTATE_BLEND_ACCEPTED` and `MSTATE_SAFE_REPLAN` are mutually exclusive for a
started segment. An ordinary non-blended start can have neither flag.

## Slave implementation

The JointBus hook obtains `refPos`, `refVel`, `refAcc`, `trajectoryElapsed`,
and `trajectoryDuration` directly from `SCurvePosVelController`. Despite the
historical class name, that controller now generates a recomputed analytic
quintic polynomial followed by the position PID.

The accepted retarget mode is latched when the prepared segment starts and is
cleared with the coordinated active slot. `MSTATE_SERVO_SETTLING` separates
the interval after the polynomial ends from final physical completion.

## Master start and watchdog sequence

For every coordinated segment the matching master performs:

```text
PREPARE_MOVEB J1..J6
verify QUEUE_STATUS J1..J6
read SEGMENT_TIMING J1..J6
common duration = guarded max(minimum duration J1 .. J6)
START_SEGMENT broadcast with common duration and relative delay
verify QUEUE_STATUS J1..J6
read MOTION_STATE J1..J6
verify installed durations against the common duration
start progress and absolute watchdogs from verified duration
```

The local rest-to-rest calculation remains a preliminary lower bound.
`SEGMENT_TIMING` lets each slave apply the same quintic validator to its live
analytic state without changing the active trajectory. The timed broadcast
then imposes one duration on all six nodes. Every non-zero duration reported
after start must match the common value within 2 ms.

The polynomial duration does not include additional follower-PID settling.
The planner's progress and absolute margins cover this phase, and normal
`STATUS` / `QSTATUS` polling still determines physical completion.
