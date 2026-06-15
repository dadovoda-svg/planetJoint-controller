# JointBus slave integration

This firmware baseline integrates the JointBus binary protocol on the joint-controller side only.

The integrated component is the **slave** endpoint used by the higher-level arm controller to command one PlanetJoint node over an RS485 half-duplex bus.

## Scope of this baseline

Included in `src/`:

- `JointBusProtocol.h`
- `JointBusSlave.h`
- `JointBusSlave.cpp`

Not included in the firmware build:

- `JointBusMaster.h`
- `JointBusMaster.cpp`

The master-side protocol implementation is intentionally kept outside this firmware. For PC-based testing, use the Python debug tool under:

```text
examples/rs485_debug_tool/
```

## Physical interface

The binary protocol uses UART0 through an RS485 transceiver.

Current configuration:

```text
UART      : UART0 / Serial0
Baud rate : 500000 bps, 8N1
RS485 DE  : GPIO9, used as UART RTS
Mode      : ESP32-S3 hardware RS485 half-duplex
```

The SP3485 `DE` input is controlled by the ESP32-S3 UART hardware through RTS while `UART_MODE_RS485_HALF_DUPLEX` is active.

The firmware still forces GPIO9 LOW as the first instruction in `setup()` to reduce the boot-time floating-DE window on the current PCB revision, which has no hardware pull-down on DE.

## Addressing

The slave address is stored in the persistent parameter:

```text
addr
```

Valid range:

```text
0..15
```

Default:

```text
0
```

Example from USB console:

```text
set addr 3
save
```

## Commands connected to the real firmware

| Command | Status |
|---|---|
| `NOP` | implemented by protocol layer |
| `PING` | implemented by protocol layer |
| `MOVE` | connected to `jointMoveTo()` |
| `MOVEB` | connected to `jointMoveToBlended()` |
| `STOP` | connected to `jointStop()` |
| `ZERO` | connected to `setZero()` |
| `PARK` | connected to `startPark()` |
| `STATUS` | connected to real joint status |
| `QSTATUS` | connected to compact real joint status |
| `REBOOT` | validates magic, ACKs, then reboots after a short delay |
| `HOME` | connected to a fixed move to logical zero |

## HOME command

`HOME` is implemented as a fixed move to the logical zero position:

```text
target = 0.0 deg
vmax   = 8.0 deg/s
amax   = 15.0 deg/s^2
```

`HOME` is intentionally not an implicit `PARK`. If the firmware is configured for
multi-turn park mode and the park procedure has not completed yet, `HOME` is
rejected with:

```text
HOME -> NACK NOT_HOMED
```

After a successful park, or when `pkdir=0` selects absolute-encoder mode, `HOME`
starts the move to zero and returns an immediate ACK. Completion must be checked
with `QSTATUS` or `STATUS`, exactly like `MOVE` and `PARK`.

## Startup behavior

The firmware performs the following JointBus startup sequence:

```text
1. force GPIO9 LOW immediately at setup() entry
2. initialize USB console and persistent parameters
3. install JointBus slave hooks
4. load the persistent JointBus address from addr
5. initialize UART0 at 500000 bps
6. assign GPIO9 as RTS
7. enable ESP32-S3 UART_MODE_RS485_HALF_DUPLEX
8. flush the JointBus RX parser and UART RX buffer
```

The master should still wait after power-on and run the established `NOP/SYNC` purge sequence before sending real commands.

## Compact status mapping

`QSTATUS` returns one byte:

```text
bit 0 : BUSY
bit 1 : DONE
bit 2 : FAULT
bit 3 : ENABLED
bit 4 : HOMED
bit 5 : WARNING
bit 6 : LIMIT_CLIPPED
bit 7 : reserved
```

`LIMIT_CLIPPED` and `WARNING` are set when the last accepted motion command was clipped to `jmin` or `jmax`.

## Extended status mapping

`STATUS` returns:

```text
int16_t pos_cdeg
int16_t target_cdeg
int16_t vel_cdeg_s
uint8_t state
uint8_t fault
```

All position and velocity values are scaled in centi-degrees.

## PC debug tool

Install dependency:

```bash
python3 -m pip install pyserial
```

Run with a USB-RS485 adapter:

```bash
python3 examples/rs485_debug_tool/planetjoint_rs485_debug.py --port /dev/ttyUSB0 --baud 500000
```

Offline mode:

```bash
python3 examples/rs485_debug_tool/planetjoint_rs485_debug.py --offline
```

Example commands:

```text
sync 0 7 3
scan 0 7
qstatus 3
status 3
move 3 45 10 30
moveb 3 60 12 35
park 3
zero 3
stop 3
```

## Notes

This baseline intentionally does not change the USB console command set. The USB console remains available for debugging, tuning and parameter management.

## Servo-hold status semantics

See `README_JOINTBUS_SHOLD_STATUS_FIX.md`.

When `shold=1`, the position controller may remain armed after the move is complete. JointBus now reports this as `HOLDING`/`DONE`, not `SETTLING`/`BUSY`, as soon as `jointCtrl.isSettled()` is true.
