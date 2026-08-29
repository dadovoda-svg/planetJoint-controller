# JointBus common-duration timed segments

This extension makes coordinated segments share both one analytic duration and
one delayed start event. It builds on `PREPARE_MOVEB`, `PREPARE_HOLD`,
`MOTION_STATE`, and the existing broadcast `START_SEGMENT`.

## Timing negotiation

The addressed request is:

```text
command  0x11 SEGMENT_TIMING
payload  uint8_t segment_id
```

The response is:

```text
command  0x91 SEGMENT_TIMING_RSP
payload  uint8_t segment_id
         uint32_t minimum_duration_ms, little-endian
```

The slave validates the prepared segment and previews its analytic quintic
without altering the controller state. The reported duration covers the
continuous retarget candidate and the safe rest-to-rest fallback from the
measured position, current reference, and endpoint of any active trajectory.
Including the active endpoint keeps the estimate valid while the reference
advances between timing negotiation and delayed start.

For a prepared HOLD segment the minimum duration is zero. HOLD therefore never
raises the common duration; it inherits the duration selected from the moving
axes and keeps the existing local PID target unchanged.

The master requests timing from J1 through J6, takes the largest response and
also compares it with its preliminary local estimate. It then applies an 8%
guard plus 2 ms because the active references continue advancing while the
six addressed transactions are exchanged.

## Timed START_SEGMENT

The legacy one-byte `START_SEGMENT` payload remains supported for diagnostic
and backward-compatible use. Coordinated G-code motion uses the extended
nine-byte payload:

| Bytes | Type | Meaning |
|---:|---|---|
| 0 | `uint8_t` | Segment ID |
| 1..4 | `uint32_t` | Common polynomial duration in milliseconds |
| 5..8 | `uint32_t` | Start delay in microseconds |

The frame is sent to broadcast address `0x0F` and receives no replies. The
current master uses a 10000 us delay. Each slave timestamps receipt with its
local `micros()`, keeps the preceding motion active during the delay, and
installs the prepared segment when its relative deadline expires.

This relative-delay scheme does not require synchronized clocks. Its residual
start skew consists mainly of UART parser and firmware-loop latency. Absolute
clock synchronization remains a separate future extension.

Allowed delay range:

```text
1000 .. 1000000 us
```

## Slave state and failure behavior

While waiting for the deadline, `QUEUE_STATUS` retains the prepared segment
and sets:

```text
QQUEUE_START_PENDING = 0x08
```

At the deadline, every non-trivial axis must install exactly the common
duration. Continuous blending is attempted first; if it is inadmissible, the
slave installs a rest-to-rest quintic with the same duration and reports
`MSTATE_SAFE_REPLAN`.

A HOLD axis does not install a polynomial. It reports `QQUEUE_ACTIVE_HOLD` and
`MSTATE_HOLD_AXIS`, retains the common duration for coordinated diagnostics,
and remains logically active until that duration has elapsed and any preceding
local trajectory/settling has finished.

A slave never silently lengthens a timed profile. If the common duration is
still infeasible, it stops locally without latching a controller fault, retains
the prepared segment for diagnosis/recovery, and does not expose the expected
active segment. The master recognizes a healthy node that did not start,
stops all six nodes, rebuilds the prepared segment, and retries with a common
duration increased by at least 50%. At most two automatic retries are allowed;
other faults and communication failures remain terminal fail-safe conditions.

After start, the master reads `MOTION_STATE` from all six slaves. Every
non-zero reported duration must match the commanded common duration within
2 ms. The verified duration becomes the watchdog basis.

## Administrative diagnostics

```text
timing <addr> <segment_id>
startt <segment_id> <duration_ms> <delay_us> [addr]
```

Without `addr`, `startt` uses the no-response broadcast form. These commands
are intended for diagnostics; normal G-code planning performs negotiation and
timed start automatically.

## Deployment

The timed planner requires the updated master and all six updated slaves.
Legacy `start` remains available, but mixing old slaves into a timed segment
causes start verification to fail safely.
