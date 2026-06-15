#!/usr/bin/env python3
"""PlanetJoint RS485 protocol debug console.

Compatible with the PlanetJoint UART/RS485 protocol V0 baseline.
Requires: pyserial

Examples:
    python3 planetjoint_rs485_debug.py --port /dev/ttyUSB0
    python3 planetjoint_rs485_debug.py --port COM5 --baud 500000

The USB-RS485 adapter is expected to handle driver-enable automatically.
"""

from __future__ import annotations

import argparse
import cmd
import dataclasses
import enum
import struct
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

try:
    import serial
except ImportError:  # Offline mode remains available without pyserial.
    serial = None  # type: ignore[assignment]

SOF = 0xA5
PROTOCOL_VERSION = 0
MAX_PAYLOAD = 16
REBOOT_MAGIC = 0xB007


class FrameType(enum.IntEnum):
    REQUEST = 0
    RESPONSE = 1


class Command(enum.IntEnum):
    NOP = 0x00
    MOVE = 0x01
    MOVEB = 0x02
    HOME = 0x03
    STOP = 0x04
    REBOOT = 0x05
    STATUS = 0x06
    QUICK_STATUS = 0x07
    ZERO = 0x08
    PARK = 0x09
    PING = 0x7F

    ACK = 0x80
    NACK = 0x81
    STATUS_RSP = 0x86
    QUICK_STATUS_RSP = 0x87
    ERROR_RSP = 0xFF


ACK_CODES: Dict[int, str] = {
    0x00: "ACCEPTED",
    0x01: "CLIPPED_TO_MIN",
    0x02: "CLIPPED_TO_MAX",
    0x03: "SAFE_REPLAN",
    0x04: "ALREADY_DONE",
    0x05: "BLEND_ACCEPTED",
    0x06: "NOP_ACCEPTED",
}

NACK_CODES: Dict[int, str] = {
    0x01: "BAD_CRC",
    0x02: "BAD_LENGTH",
    0x03: "BAD_COMMAND",
    0x04: "BAD_PAYLOAD",
    0x05: "BUSY",
    0x06: "FAULT_ACTIVE",
    0x07: "NOT_HOMED",
    0x08: "REJECTED_BY_STATE",
    0x09: "UNSUPPORTED_VERSION",
    0x0A: "INTERNAL_ERROR",
    0x0B: "TIMEOUT",
}

JOINT_STATES: Dict[int, str] = {
    0x00: "INIT",
    0x01: "READY",
    0x02: "MOVING",
    0x03: "SETTLING",
    0x04: "HOLDING",
    0x05: "STOPPED",
    0x06: "DISABLED",
    0x07: "HOMING",
    0x08: "FAULT",
    0x09: "PARKING",
}

JOINT_FAULTS: Dict[int, str] = {
    0x00: "NONE",
    0x01: "POSITION_LIMIT",
    0x02: "ENCODER_ERROR",
    0x03: "DRIVER_ERROR",
    0x04: "PLANNER_ERROR",
    0x06: "BAD_COMMAND",
    0x07: "BAD_PAYLOAD",
    0x09: "NOT_HOMED",
    0x0A: "INTERNAL_ERROR",
}

QUICK_STATUS_FLAGS: Sequence[Tuple[int, str]] = (
    (0x01, "BUSY"),
    (0x02, "DONE"),
    (0x04, "FAULT"),
    (0x08, "ENABLED"),
    (0x10, "HOMED"),
    (0x20, "WARNING"),
    (0x40, "LIMIT_CLIPPED"),
)


@dataclasses.dataclass(frozen=True)
class Frame:
    address: int
    frame_type: FrameType
    version: int
    sequence: int
    command: int
    payload: bytes
    raw: bytes = b""


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def make_header(address: int, frame_type: FrameType, version: int = PROTOCOL_VERSION) -> int:
    if not 0 <= address <= 15:
        raise ValueError("address must be in range 0..15")
    return ((address & 0x0F) << 4) | ((int(frame_type) & 0x03) << 2) | (version & 0x03)


def encode_frame(address: int, sequence: int, command: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload exceeds {MAX_PAYLOAD} bytes")
    body = bytes((
        make_header(address, FrameType.REQUEST),
        sequence & 0xFF,
        len(payload),
        command & 0xFF,
    )) + payload
    return bytes((SOF,)) + body + struct.pack("<H", crc16_modbus(body))


def hex_bytes(data: bytes) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def cdeg_signed(value_deg: float) -> int:
    value = round(value_deg * 100.0)
    if not -32768 <= value <= 32767:
        raise ValueError("angle must be in range -327.68..327.67 degrees")
    return value


def cdeg_unsigned(value: float, unit: str) -> int:
    scaled = round(value * 100.0)
    if not 0 <= scaled <= 65535:
        raise ValueError(f"{unit} must be in range 0..655.35")
    return scaled


class FrameReader:
    """Incremental parser matching the embedded JointBus::Parser."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.bad_crc = 0
        self.bad_length = 0
        self.bad_version = 0

    def clear(self) -> None:
        self.buffer.clear()

    def feed(self, data: bytes) -> List[Frame]:
        self.buffer.extend(data)
        frames: List[Frame] = []

        while True:
            try:
                sof_index = self.buffer.index(SOF)
            except ValueError:
                self.buffer.clear()
                break

            if sof_index:
                del self.buffer[:sof_index]

            if len(self.buffer) < 7:
                break

            header = self.buffer[1]
            payload_len = self.buffer[3]
            if payload_len > MAX_PAYLOAD:
                self.bad_length += 1
                del self.buffer[0]
                continue

            frame_len = 7 + payload_len
            if len(self.buffer) < frame_len:
                break

            raw = bytes(self.buffer[:frame_len])
            del self.buffer[:frame_len]

            body = raw[1:-2]
            received_crc = struct.unpack_from("<H", raw, frame_len - 2)[0]
            if crc16_modbus(body) != received_crc:
                self.bad_crc += 1
                # Search the consumed data for another possible SOF, preserving it.
                nested = raw[1:].find(bytes((SOF,)))
                if nested >= 0:
                    self.buffer[:0] = raw[nested + 1:]
                continue

            version = header & 0x03
            if version != PROTOCOL_VERSION:
                self.bad_version += 1
                continue

            frame_type_value = (header >> 2) & 0x03
            if frame_type_value not in (0, 1):
                continue

            frames.append(Frame(
                address=(header >> 4) & 0x0F,
                frame_type=FrameType(frame_type_value),
                version=version,
                sequence=raw[2],
                command=raw[4],
                payload=raw[5:5 + payload_len],
                raw=raw,
            ))

        return frames


class JointBusClient:
    def __init__(
        self,
        port: Optional[str],
        baud: int,
        timeout_ms: int,
        show_hex: bool = True,
        offline: bool = False,
    ) -> None:
        self.timeout_s = timeout_ms / 1000.0
        self.show_hex = show_hex
        self.sequence = 0
        self.reader = FrameReader()
        self.offline = offline
        self.serial = None
        self.simulated_nodes = set(range(8))
        self.simulated_position_cdeg: Dict[int, int] = {addr: 0 for addr in self.simulated_nodes}
        self.simulated_target_cdeg: Dict[int, int] = {addr: 0 for addr in self.simulated_nodes}
        self.simulated_enabled: Dict[int, bool] = {addr: False for addr in self.simulated_nodes}

        if not self.offline:
            if serial is None:
                raise RuntimeError("pyserial is not installed")
            if not port:
                raise ValueError("a serial port is required unless offline mode is enabled")
            self.serial = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0,
                write_timeout=max(timeout_ms / 1000.0, 0.1),
            )

    def close(self) -> None:
        if self.serial is not None and self.serial.is_open:
            self.serial.close()

    def flush_rx(self) -> int:
        self.reader.clear()
        if self.offline or self.serial is None:
            return 0
        waiting = self.serial.in_waiting
        self.serial.reset_input_buffer()
        return waiting

    def next_sequence(self) -> int:
        self.sequence = (self.sequence + 1) & 0xFF
        return self.sequence

    def _encode_response(self, address: int, sequence: int, command: int, payload: bytes = b"") -> bytes:
        body = bytes((
            make_header(address, FrameType.RESPONSE),
            sequence & 0xFF,
            len(payload),
            command & 0xFF,
        )) + payload
        return bytes((SOF,)) + body + struct.pack("<H", crc16_modbus(body))

    def _simulate_response(self, address: int, sequence: int, command: Command, payload: bytes) -> Optional[Frame]:
        if address not in self.simulated_nodes:
            print(f"TIMEOUT (offline): simulated node {address} is absent")
            return None

        rsp_command = Command.ACK
        rsp_payload = bytes((0x00, 0x00))

        if command == Command.NOP:
            rsp_payload = bytes((0x06, 0x00))
        elif command == Command.MOVE or command == Command.MOVEB:
            if len(payload) != 6:
                rsp_command = Command.NACK
                rsp_payload = bytes((0x04, 0x00))
            else:
                target, _vmax, _amax = struct.unpack("<hHH", payload)
                self.simulated_target_cdeg[address] = target
                self.simulated_position_cdeg[address] = target
                self.simulated_enabled[address] = True
                rsp_payload = bytes((0x05 if command == Command.MOVEB else 0x00, 0x00))
        elif command == Command.HOME:
            self.simulated_target_cdeg[address] = 0
            self.simulated_position_cdeg[address] = 0
            self.simulated_enabled[address] = True
        elif command == Command.ZERO:
            self.simulated_position_cdeg[address] = 0
            self.simulated_target_cdeg[address] = 0
        elif command == Command.PARK:
            self.simulated_target_cdeg[address] = 0
            self.simulated_position_cdeg[address] = 0
            self.simulated_enabled[address] = True
        elif command == Command.STOP:
            self.simulated_enabled[address] = False
        elif command == Command.REBOOT:
            if payload != struct.pack("<H", REBOOT_MAGIC):
                rsp_command = Command.NACK
                rsp_payload = bytes((0x04, 0x00))
            else:
                self.simulated_enabled[address] = False
        elif command == Command.STATUS:
            rsp_command = Command.STATUS_RSP
            state = 0x04 if self.simulated_enabled[address] else 0x06
            rsp_payload = struct.pack(
                "<hhhBB",
                self.simulated_position_cdeg[address],
                self.simulated_target_cdeg[address],
                0,
                state,
                0x00,
            )
        elif command == Command.QUICK_STATUS:
            rsp_command = Command.QUICK_STATUS_RSP
            flags = 0x02 | 0x10
            if self.simulated_enabled[address]:
                flags |= 0x08
            rsp_payload = bytes((flags,))
        elif command == Command.PING:
            rsp_payload = bytes((0x00, 0x00))
        else:
            rsp_command = Command.NACK
            rsp_payload = bytes((0x03, 0x00))

        raw = self._encode_response(address, sequence, rsp_command, rsp_payload)
        frames = self.reader.feed(raw)
        frame = frames[0] if frames else None
        if frame and self.show_hex:
            print(f"RX [{len(frame.raw):2d}]: {hex_bytes(frame.raw)}")
        return frame

    def transact(self, address: int, command: Command, payload: bytes = b"") -> Optional[Frame]:
        sequence = self.next_sequence()
        request = encode_frame(address, sequence, command, payload)

        if self.show_hex:
            print(f"TX [{len(request):2d}]: {hex_bytes(request)}")

        if self.offline:
            return self._simulate_response(address, sequence, command, payload)

        assert self.serial is not None
        self.serial.write(request)
        self.serial.flush()

        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            waiting = self.serial.in_waiting
            if waiting:
                chunk = self.serial.read(waiting)
                for frame in self.reader.feed(chunk):
                    if self.show_hex:
                        print(f"RX [{len(frame.raw):2d}]: {hex_bytes(frame.raw)}")
                    if (
                        frame.frame_type == FrameType.RESPONSE
                        and frame.address == address
                        and frame.sequence == sequence
                    ):
                        return frame
                    print("RX ignored:", describe_frame(frame))
            else:
                time.sleep(0.0005)

        print(f"TIMEOUT: node={address}, seq={sequence}, command={command.name}")
        return None


def name_or_hex(mapping: Dict[int, str], value: int) -> str:
    return mapping.get(value, f"UNKNOWN(0x{value:02X})")


def describe_quick_status(value: int) -> str:
    flags = [name for mask, name in QUICK_STATUS_FLAGS if value & mask]
    reserved = value & 0x80
    if reserved:
        flags.append("RESERVED_BIT7")
    return " | ".join(flags) if flags else "NO_FLAGS"


def describe_frame(frame: Frame) -> str:
    command_name = Command(frame.command).name if frame.command in Command._value2member_map_ else f"0x{frame.command:02X}"
    prefix = (
        f"addr={frame.address} seq={frame.sequence} "
        f"type={frame.frame_type.name} cmd={command_name}"
    )

    if frame.command == Command.ACK:
        if len(frame.payload) != 2:
            return f"{prefix} INVALID_ACK_LENGTH={len(frame.payload)}"
        code, detail = frame.payload
        return f"{prefix} code={name_or_hex(ACK_CODES, code)} detail=0x{detail:02X}"

    if frame.command == Command.NACK:
        if len(frame.payload) != 2:
            return f"{prefix} INVALID_NACK_LENGTH={len(frame.payload)}"
        code, detail = frame.payload
        return f"{prefix} code={name_or_hex(NACK_CODES, code)} detail=0x{detail:02X}"

    if frame.command == Command.STATUS_RSP:
        if len(frame.payload) != 8:
            return f"{prefix} INVALID_STATUS_LENGTH={len(frame.payload)}"
        pos, target, velocity, state, fault = struct.unpack("<hhhBB", frame.payload)
        return (
            f"{prefix}\n"
            f"  position : {pos / 100.0:.2f} deg\n"
            f"  target   : {target / 100.0:.2f} deg\n"
            f"  velocity : {velocity / 100.0:.2f} deg/s\n"
            f"  state    : {name_or_hex(JOINT_STATES, state)}\n"
            f"  fault    : {name_or_hex(JOINT_FAULTS, fault)}"
        )

    if frame.command == Command.QUICK_STATUS_RSP:
        if len(frame.payload) != 1:
            return f"{prefix} INVALID_QUICK_STATUS_LENGTH={len(frame.payload)}"
        value = frame.payload[0]
        return f"{prefix} qstatus=0x{value:02X} [{describe_quick_status(value)}]"

    payload_text = hex_bytes(frame.payload) if frame.payload else "<empty>"
    return f"{prefix} payload={payload_text}"


class JointBusShell(cmd.Cmd):
    intro = "PlanetJoint RS485 debug console. Type 'help' for commands."
    prompt = "jointbus> "

    def __init__(self, client: JointBusClient) -> None:
        super().__init__()
        self.client = client

    @staticmethod
    def _tokens(line: str, minimum: int, usage: str) -> List[str]:
        tokens = line.split()
        if len(tokens) < minimum:
            raise ValueError(f"usage: {usage}")
        return tokens

    def _simple(self, line: str, command: Command) -> None:
        tokens = self._tokens(line, 1, f"{command.name.lower()} <address>")
        address = parse_address(tokens[0])
        frame = self.client.transact(address, command)
        if frame:
            print(describe_frame(frame))

    def do_nop(self, line: str) -> None:
        """nop <address> -- send startup synchronization NOP."""
        self._simple(line, Command.NOP)

    def do_ping(self, line: str) -> None:
        """ping <address> -- probe a node."""
        self._simple(line, Command.PING)

    def do_home(self, line: str) -> None:
        """home <address> -- start homing."""
        self._simple(line, Command.HOME)

    def do_zero(self, line: str) -> None:
        """zero <address> -- set the current joint position as zero."""
        self._simple(line, Command.ZERO)

    def do_park(self, line: str) -> None:
        """park <address> -- move to the locally configured park position."""
        self._simple(line, Command.PARK)

    def do_stop(self, line: str) -> None:
        """stop <address> -- stop immediately and disable the motor."""
        self._simple(line, Command.STOP)

    def do_status(self, line: str) -> None:
        """status <address> -- request extended status."""
        self._simple(line, Command.STATUS)

    def do_qstatus(self, line: str) -> None:
        """qstatus <address> -- request compact status flags."""
        self._simple(line, Command.QUICK_STATUS)

    def _move(self, line: str, command: Command) -> None:
        tokens = self._tokens(line, 4, f"{command.name.lower()} <address> <angle_deg> <vmax_deg_s> <amax_deg_s2>")
        address = parse_address(tokens[0])
        angle = cdeg_signed(float(tokens[1]))
        vmax = cdeg_unsigned(float(tokens[2]), "vmax")
        amax = cdeg_unsigned(float(tokens[3]), "amax")
        payload = struct.pack("<hHH", angle, vmax, amax)
        frame = self.client.transact(address, command, payload)
        if frame:
            print(describe_frame(frame))

    def do_move(self, line: str) -> None:
        """move <address> <angle_deg> <vmax_deg_s> <amax_deg_s2>."""
        self._move(line, Command.MOVE)

    def do_moveb(self, line: str) -> None:
        """moveb <address> <angle_deg> <vmax_deg_s> <amax_deg_s2>."""
        self._move(line, Command.MOVEB)

    def do_reboot(self, line: str) -> None:
        """reboot <address> [magic] -- reboot node; default magic is 0xB007."""
        tokens = self._tokens(line, 1, "reboot <address> [magic]")
        address = parse_address(tokens[0])
        magic = int(tokens[1], 0) if len(tokens) > 1 else REBOOT_MAGIC
        if not 0 <= magic <= 0xFFFF:
            raise ValueError("magic must fit uint16")
        frame = self.client.transact(address, Command.REBOOT, struct.pack("<H", magic))
        if frame:
            print(describe_frame(frame))

    def do_sync(self, line: str) -> None:
        """sync <first_address> <last_address> [passes] -- purge RX and send addressed NOPs."""
        tokens = self._tokens(line, 2, "sync <first_address> <last_address> [passes]")
        first = parse_address(tokens[0])
        last = parse_address(tokens[1])
        passes = int(tokens[2], 0) if len(tokens) > 2 else 3
        if not 1 <= passes <= 255:
            raise ValueError("passes must be in range 1..255")
        if first > last:
            first, last = last, first

        attempted = acknowledged = timed_out = 0
        for pass_no in range(1, passes + 1):
            print(f"Sync pass {pass_no}/{passes}")
            for address in range(first, last + 1):
                self.client.flush_rx()
                attempted += 1
                frame = self.client.transact(address, Command.NOP)
                if frame and frame.command == Command.ACK:
                    acknowledged += 1
                    print(f"  node {address}: {describe_frame(frame)}")
                else:
                    timed_out += 1
                self.client.flush_rx()
        print(f"Sync complete: attempted={attempted}, acknowledged={acknowledged}, failed={timed_out}")

    def do_scan(self, line: str) -> None:
        """scan [first_address] [last_address] -- ping nodes, default 0..15."""
        tokens = line.split()
        first = parse_address(tokens[0]) if len(tokens) >= 1 else 0
        last = parse_address(tokens[1]) if len(tokens) >= 2 else 15
        if first > last:
            first, last = last, first
        found: List[int] = []
        for address in range(first, last + 1):
            self.client.flush_rx()
            frame = self.client.transact(address, Command.PING)
            if frame:
                found.append(address)
                print(f"node {address}: {describe_frame(frame)}")
        print("Found:", ", ".join(map(str, found)) if found else "none")

    def do_purge(self, line: str) -> None:
        """purge -- discard pending received bytes and reset the parser."""
        discarded = self.client.flush_rx()
        print(f"Purged {discarded} byte(s)")

    def do_hex(self, line: str) -> None:
        """hex on|off -- enable or disable raw TX/RX dumps."""
        value = line.strip().lower()
        if value not in ("on", "off"):
            raise ValueError("usage: hex on|off")
        self.client.show_hex = value == "on"
        print(f"Hex dump {'enabled' if self.client.show_hex else 'disabled'}")

    def do_stats(self, line: str) -> None:
        """stats -- show local parser error counters."""
        reader = self.client.reader
        print(
            f"bad_crc={reader.bad_crc} "
            f"bad_length={reader.bad_length} "
            f"bad_version={reader.bad_version}"
        )

    def do_exit(self, line: str) -> bool:
        """exit -- close the serial port and quit."""
        return True

    def do_quit(self, line: str) -> bool:
        """quit -- close the serial port and quit."""
        return True

    def do_EOF(self, line: str) -> bool:
        print()
        return True

    def onecmd(self, line: str):  # type: ignore[override]
        try:
            return super().onecmd(line)
        except Exception as exc:
            serial_exception = getattr(serial, "SerialException", ()) if serial is not None else ()
            if isinstance(exc, (ValueError, RuntimeError)) or (serial_exception and isinstance(exc, serial_exception)):
                print(f"ERROR: {exc}")
                return False
            raise


def parse_address(text: str) -> int:
    value = int(text, 0)
    if not 0 <= value <= 15:
        raise ValueError("address must be in range 0..15")
    return value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PlanetJoint USB-RS485 protocol debug console")
    parser.add_argument("--port", help="serial port, e.g. /dev/ttyUSB0 or COM5; omit for offline mode")
    parser.add_argument("--offline", action="store_true", help="force offline simulator mode")
    parser.add_argument(
        "--no-offline-fallback",
        action="store_true",
        help="exit instead of entering offline mode when the serial port cannot be opened",
    )
    parser.add_argument("--baud", type=int, default=500_000, help="baud rate, default 500000")
    parser.add_argument("--timeout", type=int, default=20, help="response timeout in milliseconds, default 20")
    parser.add_argument("--no-hex", action="store_true", help="do not print raw TX/RX frames")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    offline = args.offline or not args.port

    try:
        client = JointBusClient(
            args.port,
            args.baud,
            args.timeout,
            show_hex=not args.no_hex,
            offline=offline,
        )
    except Exception as exc:
        if args.no_offline_fallback:
            print(f"Unable to open {args.port}: {exc}", file=sys.stderr)
            return 2
        print(f"Unable to open {args.port}: {exc}", file=sys.stderr)
        print("Starting offline simulator instead.")
        client = JointBusClient(None, args.baud, args.timeout, show_hex=not args.no_hex, offline=True)
        offline = True

    if offline:
        print("OFFLINE MODE: no serial port is open; nodes 0..7 are simulated.")
        print("Frames are encoded and decoded exactly as on the real bus.")
    else:
        print(f"Opened {args.port} at {args.baud} bps, 8N1, timeout={args.timeout} ms")
        print("The USB-RS485 adapter must provide automatic TX-enable/turnaround.")
    try:
        JointBusShell(client).cmdloop()
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
