# UART / RS-485 Baud Rate Notes

This document records the baud-rate considerations for the PlanetJoint UART/RS-485 bus.

The current design uses ESP32-S3 UART0 connected to an SP3485 half-duplex RS-485 transceiver. Communication is master-driven polling, with short binary frames, CRC-16 checking, addressed slaves, and compact `QSTATUS` polling frames.

## Summary Recommendation

Recommended project defaults:

| Baud rate | Recommendation | Notes |
|---:|---|---|
| 115200 bps | Safe debug speed | Useful for first bring-up and logic-analyzer inspection. |
| 250000 bps | Conservative field speed | Good fallback if wiring or EMC conditions are poor. |
| 500000 bps | Recommended baseline | Good balance between robustness and polling throughput. |
| 1000000 bps | Realistic high-speed option | Suitable for short, well-terminated internal machine wiring after testing. |
| 2000000 bps | Lab/performance test speed | Plausible on a short clean bus, but should not be the first baseline. |
| 5000000 bps | ESP32-class upper UART range | Not recommended as a robust baseline for a multi-drop RS-485 bus. |
| 10000000 bps | SP3485 transceiver capability | Above the practical ESP32 UART baseline and strongly cable/layout dependent. |

Recommended default for the library and examples:

```cpp
static constexpr uint32_t JOINTBUS_BAUD = 500000;
```

## Component-Level Limits

### ESP32-S3 UART

The ESP32-S3 includes three UART controllers: UART0, UART1 and UART2. The ESP-IDF UART documentation describes the UART controllers as independently configurable for baud rate, data bits, stop bits, parity, and related parameters.

For ESP32-family UARTs, Espressif headers commonly define a maximum UART bitrate of 5,000,000 bps (`UART_BITRATE_MAX = 5000000`). This should be treated as a controller/driver capability limit, not as a guarantee that a real multi-drop RS-485 bus will be reliable at that rate.

Practical conclusion:

```text
ESP32-S3 UART0 is not limited to 115200 bps.
500000 bps and 1000000 bps are realistic targets.
5 Mbps should be considered a theoretical/edge operating point, not a project baseline.
```

### SP3485 RS-485 Transceiver

The MaxLinear SP3485 datasheet states that the device is a 3.3 V low-power half-duplex RS-485 / RS-422 transceiver and can meet RS-485 / RS-422 electrical specifications up to 10 Mbps under load.

Practical conclusion:

```text
The SP3485 itself is not the limiting component at 500 kbps or 1 Mbps.
The real limit will be the physical RS-485 bus, wiring, topology, termination, stubs, EMC, and DE turnaround handling.
```

## RS-485 Bus-Level Limits

The limiting factor is not the UART or the SP3485 alone. The practical limit depends on:

- cable length;
- cable type and characteristic impedance;
- 120 ohm termination at the bus ends;
- failsafe biasing;
- number of nodes;
- stub/drop length to each node;
- PCB routing;
- motor/driver EMC noise;
- accuracy of RS-485 driver-enable timing;
- master/slave turnaround timing.

Texas Instruments' RS-485 design material describes the length/data-rate tradeoff and uses jitter as a practical reliability metric. A conservative rule of thumb often used for RS-485 is:

```text
line length [m] * data rate [bps] < 10^7
```

This gives conservative design points such as:

```text
1 Mbps    -> approximately 10 m
500 kbps  -> approximately 20 m
250 kbps  -> approximately 40 m
```

This rule is intentionally conservative. A short, well-routed machine-internal bus can usually do better, but it is a useful sanity check.

MaxLinear's SP3485 material also indicates that RS-485 / RS-422 can support high data rates over shorter cable lengths, with the SP3485 page mentioning 10 Mbps around 15 m / 50 ft under suitable conditions and much lower rates over very long cable runs.

## PlanetJoint Bus Assumptions

Expected conditions for the PlanetJoint internal joint bus:

```text
Topology:       single master, multiple addressed slaves
Nodes:          typically 6 to 8, protocol limit 16
Physical layer: RS-485 half-duplex through SP3485
Controller:     ESP32-S3 UART0
Cable length:   expected short machine-internal wiring, likely below a few meters
Environment:    noisy, due to stepper drivers, motors, switching regulators and cabling
Protocol:       compact binary frames with CRC-16/MODBUS
Access model:   polling only; slaves answer only when addressed
```

Given these assumptions, the recommended baseline is:

```text
500000 bps, 8N1
```

After testing on the real harness with all nodes connected, the project may move to:

```text
1000000 bps, 8N1
```

only if waveform quality, error rate, and turnaround timing are acceptable.

## Throughput Estimate

UART 8N1 transmits 10 line bits per byte: 1 start bit, 8 data bits, 1 stop bit.

Approximate raw throughput:

```text
115200 bps  -> about 11520 byte/s
500000 bps  -> about 50000 byte/s
1000000 bps -> about 100000 byte/s
2000000 bps -> about 200000 byte/s
```

The compact `QSTATUS` transaction is approximately:

```text
QSTATUS request  ~= 7 bytes
QSTATUS response ~= 8 bytes
Total            ~= 15 bytes
```

On the wire:

```text
15 bytes * 10 bits/byte = 150 bits per QSTATUS transaction
```

Theoretical transaction rates:

```text
115200 bps  / 150 ~= 768 QSTATUS transactions/s
500000 bps  / 150 ~= 3333 QSTATUS transactions/s
1000000 bps / 150 ~= 6666 QSTATUS transactions/s
```

For 6 joints, theoretical full polling cycles are approximately:

```text
115200 bps  -> about 128 cycles/s
500000 bps  -> about 555 cycles/s
1000000 bps -> about 1111 cycles/s
```

These numbers are theoretical and exclude:

- RS-485 turnaround time;
- firmware scheduling;
- command processing;
- retries;
- occasional extended `STATUS` frames;
- safety margins.

Even using only 20-30% of the theoretical rate, 500000 bps provides ample margin for fast `QSTATUS` polling of 6 to 8 joints.

## Practical Test Plan

Recommended validation sequence:

1. Start at 115200 bps for first bring-up.
2. Verify DE control with oscilloscope or logic analyzer.
3. Verify that DE remains active until the final stop bit has left the UART.
4. Verify that DE is released quickly after `Serial.flush()` completes.
5. Move to 500000 bps and run long polling tests with all expected nodes connected.
6. Measure frame errors, missed ACKs, retries, and CRC failures.
7. Repeat with motors enabled and moving, because EMC conditions are worse.
8. Try 1000000 bps only after 500000 bps is stable.
9. Treat 2000000 bps as a lab/performance experiment.

Suggested acceptance criteria for the project baseline:

```text
500000 bps must run with all nodes connected,
with motors enabled,
while executing repeated movement and QSTATUS polling,
without CRC errors or missed replies over an extended test run.
```

## Hardware Notes

For the current hardware revision, DE has no external pull-down. This means the firmware mitigation is important but cannot guarantee DE state during ROM boot or reset.

Current mitigation strategy:

- force DE inactive as the first instruction in `setup()`;
- keep DE inactive by default;
- enable DE only inside frame transmission;
- call UART `flush()` before disabling DE;
- master waits after power-up before talking;
- master flushes RX garbage;
- master performs addressed `NOP/SYNC` passes before real commands.

Recommended next hardware revision:

- add physical pull-down on SP3485 `DE`;
- optionally tie `/RE` appropriately if used separately;
- add correct RS-485 line termination;
- add failsafe biasing if not already guaranteed elsewhere;
- keep stubs short;
- use a twisted pair with controlled impedance where possible.

## Final Project Decision

Project baseline:

```text
Default baud rate: 500000 bps, 8N1
Debug fallback:    115200 bps, 8N1
Conservative mode: 250000 bps, 8N1
Performance mode:  1000000 bps, 8N1, after validation
```

Do not use the SP3485 10 Mbps capability or the ESP32 UART 5 Mbps capability as the default design target. Those are component-level capabilities; the robust multi-drop bus speed must be chosen from complete-system tests.

## References

- Espressif ESP-IDF UART documentation, ESP32-S3 UART controllers and UART configuration.
- Espressif ESP32-S3 GPIO documentation, strapping pins and flash/PSRAM pin reservations.
- MaxLinear SP3485 datasheet, 3.3 V half-duplex RS-485 / RS-422 transceiver up to 10 Mbps.
- MaxLinear SP3485 product notes / RS-485 cable length discussion.
- Texas Instruments, The RS-485 Design Guide, length/data-rate tradeoff and jitter considerations.


## ESP32-S3 RS485 direction-control baseline

The current JointBus baseline uses the ESP32-S3 UART RS485 half-duplex mode rather than manually toggling the SP3485 `DE` GPIO. GPIO9 is mapped to UART RTS, normal RTS/CTS flow control is disabled, and `UART_MODE_RS485_HALF_DUPLEX` controls the transmission window. This gives more deterministic turnaround timing at 500 kbit/s and above.

The early `GPIO9 LOW` operation is still retained because UART hardware control only starts after the UART has been initialized.
