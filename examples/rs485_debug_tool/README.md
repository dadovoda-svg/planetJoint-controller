# PlanetJoint RS485 Debug Console

`planetjoint_rs485_debug.py` is an interactive Python terminal for testing the PlanetJoint UART/RS485 protocol V0 through a USB-RS485 adapter or in offline simulation mode.

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
python3 planetjoint_rs485_debug.py --port /dev/ttyUSB0 --baud 500000
```

Windows:

```powershell
python planetjoint_rs485_debug.py --port COM5 --baud 500000
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
home <addr>
stop <addr>
reboot <addr> [magic]
status <addr>
qstatus <addr>
sync <first_addr> <last_addr> [passes]
scan [first_addr] [last_addr]
purge
hex on|off
stats
exit
```

Example offline session:

```text
jointbus> scan 0 15
jointbus> move 2 45.0 10.0 30.0
jointbus> status 2
jointbus> qstatus 2
jointbus> stop 2
```

The simulator immediately completes movements and reports the requested target as the current position. Its purpose is protocol and UI testing, not motion-dynamics simulation.

## Protocol representation

The terminal accepts human-readable degrees and converts them to the protocol representation:

- target angle: signed `int16`, centidegrees
- maximum velocity: unsigned `uint16`, centidegrees/s
- maximum acceleration: unsigned `uint16`, centidegrees/s²
- all multibyte fields: little-endian
- CRC: CRC-16/MODBUS, transmitted little-endian

The tool validates response address, sequence number, frame type, protocol version, payload length and CRC before presenting a response.

## RS485 note

Most USB-RS485 adapters automatically control their transmitter-enable signal. The Python program therefore does not directly manage a DE GPIO. The adapter must release the bus quickly enough after the last transmitted byte to receive the slave response.

## ZERO and PARK

```text
zero <address>
park <address>
```

`zero` sets the current joint position as the zero reference. `park` starts the asynchronous movement to the locally configured park position; use `qstatus` or `status` to monitor completion.
