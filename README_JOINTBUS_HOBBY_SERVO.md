# GPIO3 hobby servo and JointBus command

GPIO3 has two mutually exclusive roles:

- `pkdir=-1` or `pkdir=1`: active-low park sensor input;
- `pkdir=0`, `servo=0`: unused input;
- `pkdir=0`, `servo=1`: 50 Hz hobby-servo PWM output.

The firmware rejects `servo=1` unless `pkdir=0`. Changing `pkdir` to a
non-zero value immediately detaches PWM, restores `INPUT_PULLUP`, and resets
the runtime `servo` value to `0`.

## NVM parameters

| Key | Default | Meaning |
| --- | ---: | --- |
| `servo` | `0` | Enable GPIO3 hobby-servo output (`0` or `1`) |
| `srvzero` | `1500` | Rest/startup pulse width in microseconds |
| `srvmin` | `1000` | Pulse width generated for normalized position `0` |
| `srvmax` | `2000` | Pulse width generated for normalized position `999` |

Pulse values are raw PWM high times in microseconds. They must be integer,
positive, shorter than the 20 ms PWM period, and satisfy
`srvmin <= srvzero <= srvmax` with `srvmin < srvmax`.

Example console setup:

```text
set pkdir 0
set srvmin 1000
set srvzero 1500
set srvmax 2000
set servo 1
save
srvmove 750 4
```

When PWM is first enabled, the output starts at `srvzero`. A completed command
keeps producing the target pulse until another command arrives or servo mode
is disabled.

## Serial command

```text
srvmove <position_0_999> <speed_1_10>
```

Position is mapped linearly between `srvmin` and `srvmax`. Speed `1` takes
5 seconds for a complete `srvmin` to `srvmax` traversal. Intermediate speeds
linearly reduce that full-range duration; speed `10` applies the new pulse
immediately. Shorter movements take proportionally less time.

## JointBus protocol

`SERVO_MOVE` is addressed command `0x15`. Broadcast use is not accepted.

Request payload, 3 bytes:

| Offset | Type | Meaning |
| ---: | --- | --- |
| 0 | `uint16`, little-endian | normalized position, `0..999` |
| 2 | `uint8` | speed, `1..10` |

Successful requests return the normal two-byte `ACK` payload with
`AckCode::Accepted`. Invalid ranges return `NACK/BadPayload`; a valid command
sent while hobby-servo mode is disabled returns `NACK/RejectedByState`.

## Master-side changes

Add the command to the master's protocol enum without changing protocol
version 0:

```cpp
enum class Command : uint8_t {
  // existing commands...
  ServoMove = 0x15,
};
```

Encode and send an addressed request as follows:

```cpp
bool servoMove(uint8_t address, uint16_t position, uint8_t speed)
{
  if (address == JointBus::BROADCAST_ADDRESS ||
      position > 999 || speed < 1 || speed > 10) {
    return false;
  }

  JointBus::Frame request;
  request.address = address;
  request.type = JointBus::FrameType::Request;
  request.seq = nextSequence();
  request.command = JointBus::Command::ServoMove;
  request.payloadLen = 3;
  JointBus::putU16LE(&request.payload[0], position);
  request.payload[2] = speed;

  return transactAndRequireAck(request, JointBus::AckCode::Accepted);
}
```

The master should validate all ranges before transmission, use the usual
addressed request timeout/retry policy, and treat `RejectedByState` as a node
configuration error rather than retrying indefinitely.

## Electrical note

GPIO3 supplies only the PWM logic signal. Power the servo from a suitable
external supply and connect its ground to the controller ground; do not power
the servo from the GPIO pin.
