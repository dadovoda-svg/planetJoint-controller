# Analytic Quintic S-Curve Migration

## Summary

PlanetJoint now generates position references with a precomputed analytic
quintic polynomial. The PID follower, encoder feedback, measured-velocity
filtering, TMC2209 command path, safety limits, JointBus protocol and hardware
configuration remain unchanged.

The active control path is:

```text
analytic quintic position / velocity / acceleration
                         |
                         v
existing PID and velocity feed-forward follower
                         |
                         v
TMC2209 velocity command
```

## Algorithm

Normalized time is:

```text
u = t / T
```

The trajectory is:

```text
p(u) = c0 + c1*u + c2*u^2 + c3*u^3 + c4*u^4 + c5*u^5
```

Each trajectory uses the boundary conditions:

```text
p(0) = p0       v(0) = v0       a(0) = a0
p(1) = target   v(1) = 0        a(1) = 0
```

Normal moves reset at the measured position and therefore use `v0=0` and
`a0=0`. Their duration is:

```text
Tv = 1.875 * abs(distance) / vmax
Ta = sqrt(5.7735027 * abs(distance) / amax)
T  = max(minimum_duration, Tv, Ta)
```

Forward `moveb` retargets start from the current analytic `refPos`, `refVel`
and `refAcc`. Candidate durations are increased through a bounded search and
each polynomial is sampled for finite values, monotonicity, joint position,
velocity and acceleration envelopes. A rejected, reverse or infeasible blend
uses the planner's safe rest-to-rest replan.

Position, velocity and acceleration are evaluated directly from the
polynomial. They are not separately integrated. The final reference velocity
and acceleration are zero by construction.

## PID and Completion Behavior

The motor command remains:

```text
command = ffv * reference_velocity
        + kp * (reference_position - measured_position)
        + integral_term
        - kd * measured_velocity
```

Measured encoder velocity remains filtered and is used for the derivative
term, `dbvel`, `vtol` and diagnostics. It does not affect trajectory evolution.

Deadband and settled logic cannot terminate an active analytic trajectory.
With `shold=1`, PID correction remains active after profile completion and is
suppressed only when the configured deadband has latched.

## API and Parameter Changes

Removed:

- the iterative acceleration state update;
- the internal configurable jerk limit;
- `setSCurveTime()` and `setJerkMax()`;
- `JointMoveCommand::sCurveTimeS`;
- planner and firmware overloads accepting a profile-ramp time;
- the persistent `sct` parameter;
- the optional console argument formerly accepted after acceleration.

Current console syntax is exactly:

```text
move  <target_deg> <vmax_deg_s> <amax_deg_s2>
moveb <target_deg> <vmax_deg_s> <amax_deg_s2>
```

Old four-value forms are rejected. An old text export must have its
`sct=<value>` line removed before import. No NVS erase is needed: the fixed-size
storage image performs a key-based merge and ignores obsolete stored keys.

The runtime planner interface now uses:

```cpp
configureController(vmax, amax, outmax, clearFault)
bool blendControllerTarget(target)
```

Changing `vmax` or `amax` while a profile is active attempts a continuous
replan. Failure logs an error and stops motion safely.

## Diagnostics

The detailed `servo` output now includes:

```text
refa=<reference acceleration>
profile=<0|1>
time=<elapsed>/<duration>
```

Trace mode 4 includes `ref_acc` in addition to measured position, reference
position, target, reference velocity and command velocity. Full trace also
exports profile state and elapsed/duration fields.

## Files Changed

Core implementation:

- `include/SCurvePosVelController.h`
- `include/JointPlanner.h`
- `include/JointMotionApi.h`
- `src/JointPlanner.cpp`
- `src/SerialConsole.cpp`
- `src/main.cpp`

Tests:

- `test/Arduino.h`
- `test/test_joint_planner.cpp`
- `test/test_quintic_controller.cpp`

Documentation:

- `README.md`
- `README_TUNING.md`
- `README_BLENDED_MOVE.md`
- `README_JOINT_LIMITS.md`
- `README_PLANNER_API_FIX.md`
- `README_PLANNER_REFACTOR.md`
- `README_MOVEB_RETARGET_FIX.md`
- `README_SERVO_HOLD.md`
- `README_MOTOR_HOLD.md`
- this report

## Preserved Target-Revision Values

The following values were recorded before the migration and remain unchanged:

| Setting | Preserved value |
|---|---:|
| Control period | `5000 us` / `200 Hz` |
| Default `vmax` | `2.0 deg/s` |
| Default `amax` | `6.0 deg/s^2` |
| Default `outmax` | `2.5 deg/s` |
| Planner maximum velocity | `240.0 deg/s` |
| Planner maximum acceleration | `650.0 deg/s^2` |
| PID `kp`, `ki`, `kd`, `ffv`, `ilim` | `0.8`, `0.0`, `0.02`, `1.0`, `0.0` |
| Settled `ptol`, `vtol` | `0.08 deg`, `0.15 deg/s` |
| Deadband `dbent`, `dbext`, `dbvel` | `0.05 deg`, `0.12 deg`, `0.20 deg/s` |
| Velocity filter `vtau` | `0.050 s` |
| Joint `jmin`, `jmax`, `jtol` | `-170.0`, `+170.0`, `1.0 deg` |
| Default `jrev` | `15.6 deg/encoder revolution` |
| Default `mdir`, `edir` | `+1`, `+1` |
| JointBus baud | `500000` unless the existing `921600` build option is enabled |
| TMC UART baud | `230400` |

Pin assignments, TMC2209 current and microstep defaults, encoder-selection
macros, park/home settings, JointBus IDs and packet layouts were not changed.
The angular `jmin` and `jmax` NVS parameters remain intact.

## Verification

Commands executed:

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Werror -Itest -Iinclude test/test_quintic_controller.cpp -o /tmp/planetjoint_test_quintic
/tmp/planetjoint_test_quintic

g++ -std=c++17 -Wall -Wextra -Werror -Iinclude src/JointPlanner.cpp test/test_joint_planner.cpp -o /tmp/planetjoint_test_planner
/tmp/planetjoint_test_planner

g++ -std=c++17 -Wall -Wextra -Werror -Itest -Iinclude test/test_parser.cpp -o /tmp/planetjoint_test_parser
/tmp/planetjoint_test_parser

g++ -std=c++17 -Wall -Wextra -Werror -Itest -Iinclude test/test_motion_config.cpp -o /tmp/planetjoint_test_motion_config
/tmp/planetjoint_test_motion_config

g++ -std=c++17 -Wall -Wextra -Werror -Itest -Iinclude src/as5048a.cpp src/as5600.cpp test/test_encoder_direction.cpp -o /tmp/planetjoint_test_encoder_direction
/tmp/planetjoint_test_encoder_direction

/home/davide/.platformio/penv/bin/platformio run -e esp32-s3-as5048a -e esp32-s3-as5600
```

Results:

- analytic controller tests: passed, including 1000 randomized rest profiles
  and 300 randomized forward-retarget attempts;
- planner tests: passed, including controller-rejected blend safe replan;
- JointBus parser test: passed;
- motion configuration test: passed;
- encoder direction test: passed;
- ESP32-S3 AS5048A build: passed;
- ESP32-S3 AS5600 build: passed.

## Design Trade-Offs

- Position, velocity and acceleration are continuous.
- Final velocity and acceleration are zero by construction.
- There is no explicit jerk limit or runtime jerk parameter.
- The polynomial has a finite mathematical derivative of acceleration, but it
  is neither configured nor limited.
- A single rest-to-rest quintic has no constant-velocity plateau.
- Duration respects `vmax` and `amax`; it is not a time-optimal industrial
  seven-segment trajectory.
- A continuous replan cannot instantly reduce boundary velocity or
  acceleration. The validation envelope therefore includes the current
  boundary magnitude.
- Rejected blends use safe replan.
- Hardware performance still depends on PID tuning, encoder noise, load,
  torque margin, output saturation and control-loop timing.

## Required Hardware Validation

Hardware validation has not been performed for this migration.

1. Export the existing parameter set before flashing.
2. Use the target revision's conservative `vmax`, `amax`, and `outmax`.
3. Enable `servo` status and `trace 4`.
4. Run a small positive move and return to zero.
5. Verify `ref_vel` and `ref_acc` start and finish at zero.
6. Verify no terminal jump is visible.
7. Issue a farther same-direction `moveb` and verify reference continuity.
8. Issue reverse and too-close `moveb` commands and verify safe replan.
9. With `shold=1`, verify correction continues until deadband latches.
10. Confirm hard `jmin`/`jmax`/`jtol` behavior is unchanged.
11. Repeat on one complete joint before coordinated multi-joint validation.
