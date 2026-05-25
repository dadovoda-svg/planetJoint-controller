# Planner refactor

This build moves the planner-style movement orchestration out of `main.cpp` and into a dedicated module:

- `src/JointPlanner.h`
- `src/JointPlanner.cpp`

The goal is to keep `main.cpp` focused on board bring-up, hardware wiring, encoder/TMC initialization, the USB console, LEDs and the 200 Hz servo update loop.

## What changed

### New `JointPlanner` module

`JointPlanner` owns the high-level movement entry points:

- `moveTo()`
- `moveToBlended()`
- `stop()`
- planner serial command parsing

The old console-compatible global API is still available as wrappers:

- `moveJointToDeg()`
- `jointMoveTo()`
- `jointMoveToBlended()`
- `jointStop()`

This keeps the existing USB console commands unchanged:

- `pos <deg>`
- `move <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]`
- `moveb <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]`
- `stop`

### Planner serial port

The previously unused `SerialFuture` port is now passed to the planner constructor:

```cpp
JointPlanner planner(SerialFuture);
```

and initialized through:

```cpp
planner.begin(UART0_BAUD);
```

The loop now calls:

```cpp
planner.update();
```

This prepares the firmware for an external planner connected to that serial port.

Supported commands on the planner serial port are intentionally minimal for now:

```text
help
status
stop
move <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]
moveb <target_deg> <vmax_deg_s> <amax_deg_s2> [sct_s]
```

Line endings are tolerant of `\n`, `\r` and `\r\n`.

## Conservative refactoring note

This is deliberately a first-step refactor, not a full architecture rewrite. Hardware ownership still remains in `main.cpp`; the planner module uses a small set of `extern` bindings to the already tested objects and state.

That choice keeps the validated behavior of the logger build intact while giving us a clear place to extend planner integration in the next steps.
