# PlanetJoint RS485 Debug Console

`planetjoint_rs485_debug.py` is an interactive Python terminal for testing the PlanetJoint UART/RS485 protocol through a USB-RS485 adapter or in offline simulation mode.

## Requirements

- Python 3.9 or newer
- `pyserial` only for real serial-port operation
- A USB-RS485 adapter with automatic transmit-enable/turnaround

Install the serial dependency:

```bash
python3 -m pip install pyserial
```

Offline mode works even when `pyserial` is not installed.

## Offline start

Start without specifying a port:

```bash
python3 planetjoint_rs485_debug.py
```

or explicitly:

```bash
python3 planetjoint_rs485_debug.py --offline
```

Offline mode simulates nodes `0..7`. It still builds complete binary frames, calculates CRC-16/MODBUS, parses simulated responses and prints human-readable results.

When a specified serial port cannot be opened, the program automatically falls back to offline mode. Use `--no-offline-fallback` to make a port-open failure fatal.

## Real serial-port start

Linux:

```bash
python3 planetjoint_rs485_debug.py --port /dev/ttyUSB0 --baud 500000 --no-offline-fallback
```

Windows:

```powershell
python planetjoint_rs485_debug.py --port COM5 --baud 500000 --no-offline-fallback
```

Options:

```text
--port PORT             Serial device; omit for offline mode
--offline               Force offline simulation
--no-offline-fallback   Exit if the requested port cannot be opened
--baud BAUD             Default: 500000
--timeout MS            Response timeout; default: 20 ms
--no-hex                Disable raw TX/RX hexadecimal dumps
```

## Interactive commands

```text
nop <addr>
ping <addr>
move <addr> <angle_deg> <vmax_deg_s> <amax_deg_s2>
moveb <addr> <angle_deg> <vmax_deg_s> <amax_deg_s2>
prepmb <addr> <segment_id> <angle_deg> <vmax_deg_s> <amax_deg_s2>
preph <addr> <segment_id>
start <segment_id> [addr]
abortseg <segment_id|all> [addr]
qqueue <addr>
queue <addr>
home <addr>
zero <addr>
park <addr>
stop <addr>
estop [addr|all]
clearfault <addr>
reboot <addr> [magic]
status <addr>
qstatus <addr>
mconfig <addr>
config <addr>
sync <first_addr> <last_addr> [passes]
scan [first_addr] [last_addr]
purge
hex on|off
stats
exit
```

`mconfig` (or its `config` alias) sends the addressed `MOTION_CONFIG` request and prints all four planning parameters returned by the node:

```text
jointbus> mconfig 1
addr=1 seq=1 type=RESPONSE cmd=MOTION_CONFIG_RSP
  jmin : -170.00 deg
  jmax : 170.00 deg
  vmax : 2.00 deg/s
  amax : 6.00 deg/s^2
```

The offline simulator uses these same defaults for nodes `0..7`.

`clearfault` sends the addressed `CLEAR_FAULT` request. The real node clears
the latch only after fresh encoder and driver checks, valid persistent joint
limits, and a measured position inside `jmin..jmax`. The motor remains disabled.

## Coordinated segment example

Prepare one segment on two joints and start both with one broadcast frame:

```text
jointbus> prepmb 1 42 10.0 8.0 15.0
jointbus> preph 2 42
jointbus> qqueue 1
jointbus> qqueue 2
jointbus> start 42
jointbus> qstatus 1
jointbus> qstatus 2
```

`start <segment_id>` without an address sends a no-response broadcast frame to address `15`.

For debugging one node, addressed start is also available:

```text
jointbus> start 42 1
```

This addressed form expects an ACK/NACK response.

`abortseg all` sends a no-response broadcast that cancels any prepared segment on all nodes. It does not stop active motion; use `stop <addr>` for a recoverable single-node stop or `estop` for a broadcast latched emergency stop.

## Protocol representation

The terminal accepts human-readable degrees and converts them to the protocol representation:

- target angle: signed `int16`, centidegrees
- maximum velocity: unsigned `uint16`, centidegrees/s
- maximum acceleration: unsigned `uint16`, centidegrees/s²
- motion configuration response: `<hhHH` containing `jmin`, `jmax`, `vmax`, `amax`
- segment id: `uint8`, range `0..254`; `255` is reserved as `none/all`
- all multibyte fields: little-endian
- CRC: CRC-16/MODBUS, transmitted little-endian

The tool validates response address, sequence number, frame type, protocol version, payload length and CRC before presenting a response.

## RS485 note

Most USB-RS485 adapters automatically control their transmitter-enable signal. The Python program therefore does not directly manage a DE GPIO. The adapter must release the bus quickly enough after the last transmitted byte to receive the slave response.

## Emergency stop

```text
estop
estop all
estop <addr>
```

Without an address, `estop` sends broadcast `EMERGENCY_STOP` to address `15` and expects no response. With an address, it sends an addressed command and expects ACK/NACK.

After emergency stop, poll each node with `qstatus` or `status`; the expected extended status is `state=FAULT` and `fault=EMERGENCY_STOP`.
