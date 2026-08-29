# JointBus `CLEAR_FAULT`

## Protocol

`CLEAR_FAULT` is an addressed, zero-payload request. It is deliberately not
accepted as a broadcast command.

| Item | Value |
|---|---:|
| Command | `ClearFault` |
| Command ID | `0x12` |
| Request payload | empty |
| Successful ACK | `FaultCleared` (`0x0C`) |
| No active fault ACK | `AlreadyDone` (`0x04`) |

The command never enables the motor. A real recovery first stops internal
motion and disables the TMC2209 bridge, then requires all of the following:

- a fresh, finite encoder reading;
- a configured TMC2209 with a successful UART probe;
- finite current `jmin`, `jmax`, and `jtol` parameter values, loaded from NVM
  or corrected in RAM through the local console;
- `jmax > jmin` and `jtol >= 0`;
- current zeroed position inside the strict `jmin..jmax` range.

If recovery succeeds, the controller is reset at the measured position, its
target is rebased to that position, coordinated segments are cleared, and the
node enters `IDLE`. Referencing state is preserved, so an unreferenced node
still requires park before motion.

Rejected recovery returns a NACK and leaves the fault latched:

| Condition | NACK | Detail |
|---|---|---|
| Invalid `jmin`/`jmax`/`jtol` | `BadPayload` | `PlannerError` |
| Position outside `jmin..jmax` | `RejectedByState` | `PositionLimit` |
| Encoder unavailable | `RejectedByState` | `EncoderError` |
| Driver unavailable | `RejectedByState` | `DriverError` |

Invalid joint-limit configuration is always latched internally as
`SCurvePosVelController::FaultCode::BadLimits`, so `STATUS` consistently
reports `JointFault::PlannerError`.

The equivalent local console command is:

```text
clearfault
```

## Master integration

Add the command and ACK values without changing existing IDs:

```cpp
enum class Command : uint8_t {
    // Existing commands remain unchanged.
    ClearFault = 0x12
};

enum class AckCode : uint8_t {
    // Existing ACK values remain unchanged.
    FaultCleared = 0x0C
};
```

Send an addressed request with `payloadLen=0`. Accept either `FaultCleared` or
`AlreadyDone` as success. On NACK, inspect both the NACK code and detail byte,
then poll `STATUS`; do not retry continuously while the physical rejection
condition remains present.

Example master-side operation:

```cpp
CommandResult clearFault(uint8_t address)
{
    return transact(address, Command::ClearFault, nullptr, 0);
}
```

After `FaultCleared`, poll `STATUS` or `QSTATUS` and require the fault flag to
be clear before accepting a new motion command.
