# JointBus Parser Fix

This baseline fixes a bug in the JointBus frame parser.

## Problem

The previous parser wrote decoded frame fields (`address`, `seq`, `command`, `payloadLen`, payload bytes) into the `Frame& outFrame` object passed to `Parser::push()`.

`Slave::update()` called `Parser::push()` once per received byte using a local `Frame` object. Therefore, when the final CRC byte produced `FrameReady`, the local `Frame` instance no longer contained the fields decoded from the previous bytes.

The observed effect was that a valid frame such as:

```text
A5 40 46 00 7F B5 D1
```

which means:

```text
address = 4
seq     = 70
cmd     = PING
```

was reported by the firmware as:

```text
address = 0
seq     = 0
cmd     = PING
```

## Fix

The parser now owns an internal `Frame _frame` member. Incoming bytes update this internal frame while the parser state machine advances. When CRC and protocol version checks pass, the parser copies `_frame` into `outFrame` immediately before returning `FrameReady`.

This makes the parser robust even when bytes arrive across multiple `Slave::update()` calls.

## Additional baseline update

This baseline also keeps the updated hardware configuration:

```cpp
static constexpr float MOTOR_DIRECTION_SIGN = 1.0f;
```
