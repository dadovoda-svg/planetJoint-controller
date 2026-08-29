# Motion Completion Latch

## Purpose

Position command completion and servo-hold correction are different states.
An accepted command has a one-way lifecycle:

```text
inactive -> active -> completed
```

After completion, a physical disturbance may make the controller leave and
re-enter its deadband. That does not reactivate the command.

The host-testable lifecycle logic is in `include/MotionLifecycle.h`.

## State

| Field | Meaning |
|---|---|
| `positionCommandActive` | An accepted position command has not reached its first valid settled state |
| `servoHoldActive` | A completed position is being maintained by servo hold |
| `servoCorrectionArmed` | The completed hold has entered deadband at least once |
| `servoCorrectionActive` | An armed hold is outside deadband and correcting a disturbance |

Only `FirmwareJointPlannerRuntime::beginPositionMotion()` opens a new position
command lifecycle. Completion, stop, fault, park, re-reference, calibration,
and test-mode transitions clear or invalidate it through the centralized
lifecycle state operations.

`SCurvePosVelController::isSettled()` is deliberately unchanged. It is sampled
to detect the first completion, but it remains a dynamic physical condition and
may later alternate during hold corrections.

## State transitions

| Event/state | Command active | Servo hold | JointBus state | BUSY |
|---|---:|---:|---|---:|
| Idle before command | 0 | 0 | READY | 0 |
| New position command accepted | 1 | 0 | MOVING | 1 |
| Initial trajectory and settling | 1 | 0 | MOVING | 1 |
| First completion with `shold=0` | 0 | 0 | READY or HOLDING according to `mhold` | 0 |
| First completion with `shold=1` | 0 | 1 | HOLDING | 0 |
| Hold correction starts | 0 | 1 | HOLDING | 0 |
| Hold correction completes | 0 | 1 | HOLDING | 0 |
| New command during completed hold | 1 | 0 | MOVING | 1 |
| Explicit stop | 0 | 0 | READY or IDLE | 0 |
| Fault | 0 | 0 | FAULT | 0 |
| Park | 0 | 0 | PARKING | 1 |

For Quick Status, a completed hold and every later correction report
`QSTAT_DONE=1` and `QSTAT_BUSY=0`. A fault reports `QSTAT_FAULT=1` and does not
report DONE. Driver-enabled and homed flags remain independent.

The coordinated-segment updater clears the active segment when the latched
command becomes complete. Later servo corrections cannot recreate it or set
`QQUEUE_ACTIVE_BUSY`.

## Planner retargeting

`moveb` can perform a full analytic blend only while
`positionCommandActive=true`. `MotionMode::POSITION` alone is insufficient
because it is intentionally retained during completed servo hold.

When `moveb` arrives during a completed hold, the planner performs a safe
replan:

1. leave the completed hold safely;
2. enable the driver;
3. reset the controller from the measured joint position;
4. generate a new quintic trajectory;
5. open a new command lifecycle.

The result is `SafeReplan`, not `BlendAccepted`. Existing target-ahead checks
and quintic blend mathematics remain unchanged for genuinely active commands.

## Runtime `shold` changes

Disabling `shold` during a completed hold clears correction state, stops
internal motion, applies the existing `mhold` driver policy, and leaves
`MotionMode::POSITION`. Disabling it while a command is still active lets that
command complete normally without hold.

Enabling `shold` never fabricates an active command. It applies to a currently
active command, if any, or to future commands.

## Diagnostics

Each accepted command produces exactly one INFO completion message. Once the
completed hold has reached deadband, every disturbance produces one INFO
correction-start transition and one INFO correction-complete transition.
Optional active-correction DEBUG detail is rate limited to four lines per
second. See `src/README_LOGGER.md` for the log-level matrix.
