# JointBus in this firmware baseline

This firmware baseline integrates only the **JointBus slave** endpoint.

For the firmware-specific integration notes, see:

```text
README_JOINTBUS_SLAVE_INTEGRATION.md
```

For the coordinated segment / broadcast start extension, see:

```text
README_JOINTBUS_COORDINATED_SEGMENTS.md
```

For the emergency stop command, see:

```text
README_JOINTBUS_EMERGENCY_STOP.md
```

For the single-call motion configuration query (`jmin`, `jmax`, `vmax`, `amax`), including master integration instructions, see:

```text
README_JOINTBUS_MOTION_CONFIG.md
```

For analytic trajectory state and the actual-duration watchdog handshake, see:

```text
README_JOINTBUS_MOTION_STATE.md
```

For common-duration negotiation and delayed broadcast start, see:

```text
README_JOINTBUS_TIMED_SEGMENTS.md
```

The coordinated protocol also provides `PREPARE_HOLD` for axes whose target
does not change. HOLD participates in the shared segment timing without
replacing the existing PID target or creating a zero-length polynomial.

For safe recovery from a latched fault, see:

```text
README_JOINTBUS_CLEAR_FAULT.md
```

For the optional GPIO3 hobby-servo output, its NVM parameters, and the
`SERVO_MOVE` master integration, see:

```text
README_JOINTBUS_HOBBY_SERVO.md
```

For RS485 speed considerations, see:

```text
README_UART_RS485_SPEED.md
```

The PC debug tool is included under:

```text
examples/rs485_debug_tool/
```

The master C++ class is intentionally not included in this firmware baseline.
