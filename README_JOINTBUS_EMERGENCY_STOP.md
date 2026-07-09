# JointBus Emergency Stop

This baseline adds a hard emergency-stop command to the JointBus protocol.

## Command

```text
0x0E EMERGENCY_STOP
```

Payload length is zero.

## Addressed use

Addressed emergency stop is useful for debug and single-node tests:

```text
addr = node address
cmd  = EMERGENCY_STOP
len  = 0
```

A valid addressed frame returns:

```text
ACK EMERGENCY_STOPPED
```

## Broadcast use

The normal arm-controller use is broadcast/no-response:

```text
addr = 15
cmd  = EMERGENCY_STOP
len  = 0
```

No slave replies to the broadcast frame. This avoids RS485 collisions and minimizes bus traffic during emergency handling.

## Slave behavior

On a valid `EMERGENCY_STOP`, the slave immediately:

```text
- commands zero motor velocity
- stops internal TMC motion
- disables the TMC power stage, matching the existing hard fault behavior
- aborts park/test/calibration state
- clears the coordinated segment queue
- latches the position controller fault
- enters MotionMode::FAULT
- reports JointBus state FAULT
- reports JointBus fault EMERGENCY_STOP
```

After the latch, motion commands are rejected with `NACK FAULT_ACTIVE` until the node is explicitly recovered, currently by reboot, as with other latched hard faults.

## Status reporting

Extended status reports:

```text
state = FAULT
fault = EMERGENCY_STOP
```

Compact status reports the fault bit. The firmware may also set `WARNING` to indicate a non-normal latched condition.

## Python tool

The debug console supports:

```text
estop             broadcast emergency stop, no response expected
estop all         same as estop
estop <addr>      addressed emergency stop with ACK/NACK
```

Example:

```text
jointbus> estop
TX [ 7]: ...
NO RESPONSE EXPECTED: EMERGENCY_STOP address=15
```

Then poll nodes individually:

```text
jointbus> qstatus 4
jointbus> status 4
```
