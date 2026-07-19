# JointBus motion configuration query

## Purpose

The addressed `MotionConfig` request lets a JointBus master read the four motion-planning parameters of one joint with a single transaction:

- `jmin`: minimum target position, joint degrees
- `jmax`: maximum target position, joint degrees
- `vmax`: configured maximum velocity, joint degrees/second
- `amax`: configured maximum acceleration, joint degrees/second²

The slave returns the current values held in RAM. Therefore unsaved changes made with the local `set` command are visible immediately. The values are quantized to `0.01` in their respective units.

## Protocol definition

The extension is additive and keeps `PROTOCOL_VERSION = 0`.

```cpp
enum class Command : uint8_t {
    // existing commands...
    MotionConfig    = 0x0F,
    // existing responses...
    MotionConfigRsp = 0x8F
};
```

`MotionConfig` must be sent to one joint address. It is not a broadcast command, because multiple slaves replying together would collide on RS-485.

### Request

```text
type        Request
command     0x0F MotionConfig
payloadLen  0
payload     none
```

The complete request frame is the normal seven-byte JointBus frame:

```text
SOF | HEADER | SEQ | 0x00 | 0x0F | CRC_LO | CRC_HI
```

CRC-16/MODBUS is calculated over `HEADER` through `COMMAND`, excluding `SOF` and the CRC bytes.

### Successful response

```text
type        Response
command     0x8F MotionConfigRsp
payloadLen  8
```

Payload layout, little-endian:

| Offset | Type | Value | Scale |
|---:|---|---|---:|
| 0 | `int16_t` | `jmin` | × 0.01 deg |
| 2 | `int16_t` | `jmax` | × 0.01 deg |
| 4 | `uint16_t` | `vmax` | × 0.01 deg/s |
| 6 | `uint16_t` | `amax` | × 0.01 deg/s² |

For example, `jmin=-170.00`, `jmax=170.00`, `vmax=240.00` and `amax=650.00` are encoded as:

```text
98 BD | 68 42 | C0 5D | E8 FD
```

The complete response is 15 bytes including framing and CRC. It uses the same address and sequence number as the request.

If the payload length is not zero, the slave returns `Nack(BadLength)`. If a parameter is invalid or cannot be represented by the wire format, it returns `Nack(InternalError)`.

## Changes required on the master

### 1. Update the shared protocol definitions

The preferred solution is to use the updated `JointBusProtocol.h` on both nodes. If the master maintains its own copy, add the two command values, `MOTION_CONFIG_PAYLOAD_SIZE`, `MotionConfig`, and the payload decoder:

```cpp
static constexpr size_t MOTION_CONFIG_PAYLOAD_SIZE = 8;

struct MotionConfig {
    int16_t jminCdeg;
    int16_t jmaxCdeg;
    uint16_t vmaxCdegS;
    uint16_t amaxCdegS2;
};

bool decodeMotionConfigPayload(const uint8_t* payload,
                               size_t payloadLen,
                               MotionConfig& config)
{
    if (payload == nullptr || payloadLen != MOTION_CONFIG_PAYLOAD_SIZE) {
        return false;
    }
    config.jminCdeg = getI16LE(&payload[0]);
    config.jmaxCdeg = getI16LE(&payload[2]);
    config.vmaxCdegS = getU16LE(&payload[4]);
    config.amaxCdegS2 = getU16LE(&payload[6]);
    return true;
}
```

Do not decode by casting the payload to a packed C++ struct: explicit little-endian reads avoid alignment and host-endianness problems.

### 2. Add a master transaction

Build and send an addressed request using the master's normal sequence allocator:

```cpp
Frame request;
request.address = jointAddress;
request.type = FrameType::Request;
request.seq = nextSequence();
request.command = Command::MotionConfig;
request.payloadLen = 0;

sendFrame(request);
```

Register the pending transaction with these expected response properties:

```text
address     requested joint address
type        Response
sequence    request.seq
command     MotionConfigRsp (0x8F), or Nack (0x81)
payloadLen  8 for MotionConfigRsp
```

Use the same timeout and retry policy already used by `Status`. A retry must normally use a new sequence number unless the existing master transaction layer explicitly supports idempotent retransmission with the same sequence.

### 3. Decode into engineering units

```cpp
struct JointMotionLimits {
    float jminDeg;
    float jmaxDeg;
    float vmaxDegS;
    float amaxDegS2;
};

bool toEngineeringUnits(const Frame& response, JointMotionLimits& out)
{
    if (response.type != FrameType::Response ||
        response.command != Command::MotionConfigRsp) {
        return false;
    }

    MotionConfig wire;
    if (!decodeMotionConfigPayload(response.payload, response.payloadLen, wire)) {
        return false;
    }

    out.jminDeg = static_cast<float>(wire.jminCdeg) * 0.01f;
    out.jmaxDeg = static_cast<float>(wire.jmaxCdeg) * 0.01f;
    out.vmaxDegS = static_cast<float>(wire.vmaxCdegS) * 0.01f;
    out.amaxDegS2 = static_cast<float>(wire.amaxCdegS2) * 0.01f;

    return out.jminDeg < out.jmaxDeg &&
           out.vmaxDegS > 0.0f && out.amaxDegS2 > 0.0f;
}
```

### 4. Use the returned limits

The master should query every joint during discovery or initialization and cache the result by address. Before sending `Move`, `MoveB` or `PrepareMoveB`:

1. reject or clip a target outside `[jminDeg, jmaxDeg]` according to the master's policy;
2. limit the requested velocity to `vmaxDegS`;
3. limit the requested acceleration to `amaxDegS2`;
4. convert the final values to JointBus centi-units;
5. still process slave-side clipping ACKs, because parameters may change after the query.

For coordinated motion, calculate segment timing using the limits returned by every participating node. The usable global constraint is normally the most restrictive value among the joints involved in that segment.

There is no automatic invalidation message when a parameter changes through the local console. Re-query before creating a new motion plan when configuration may have changed, or periodically refresh the cache if live tuning is allowed.

## Debug tool

`examples/rs485_debug_tool/planetjoint_rs485_debug.py` supports the new transaction both on a real RS-485 adapter and in offline simulation:

```text
mconfig <address>
config <address>    # alias
```

The decoded response is printed directly in degrees, degrees/second and degrees/second².

## Slave implementation points

The firmware extension consists of:

- wire types and helpers in `include/JointBusProtocol.h`;
- request dispatch and response serialization in `src/JointBusSlave.cpp`;
- the `motionConfig` slave hook in `include/JointBusSlave.h`;
- parameter acquisition and validation in `src/main.cpp`;
- host coverage in `test/test_motion_config.cpp`.
