"""ST3215 servo protocol."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

from policy_embodied_runtime.protocol.base import DeviceCommand, DeviceReading, ProtocolFrame

HEADER = b"\xff\xff"
BROADCAST_ID = 0xFE


class St3215Instruction(IntEnum):
    """ST3215 instruction bytes."""

    PING = 0x01
    READ_DATA = 0x02
    WRITE_DATA = 0x03
    REG_WRITE = 0x04
    ACTION = 0x05
    RECOVERY = 0x06
    RESET = 0x0A
    SYNC_READ = 0x82
    SYNC_WRITE = 0x83


class St3215Register(IntEnum):
    """ST3215 register addresses used by the runtime."""

    GOAL_POSITION = 0x2A
    PRESENT_POSITION = 0x38


@dataclass(frozen=True, slots=True)
class St3215Status:
    """ST3215 status packet payload."""

    device_id: int
    error: int
    parameters: bytes


class St3215ProtocolError(ValueError):
    """Invalid ST3215 command or packet."""


@dataclass(frozen=True, slots=True)
class _DecodedPacket:
    device_id: int
    instruction_or_status: int
    parameters: bytes


class St3215Protocol:
    """ST3215 packet codec."""

    @staticmethod
    def ping_command(device_id: int) -> DeviceCommand:
        """Build a PING command."""
        return _instruction_command(device_id, St3215Instruction.PING, b"")

    @staticmethod
    def read_data_command(device_id: int, address: St3215Register, length: int) -> DeviceCommand:
        """Build a READ DATA command."""
        return _instruction_command(device_id, St3215Instruction.READ_DATA, bytes([address, length]))

    @staticmethod
    def write_data_command(device_id: int, address: St3215Register, data: bytes) -> DeviceCommand:
        """Build a WRITE DATA command."""
        return _instruction_command(device_id, St3215Instruction.WRITE_DATA, bytes([address]) + data)

    @staticmethod
    def goal_position_command(
        device_id: int,
        position_units: int,
        time_units: int,
        speed_units: int,
    ) -> DeviceCommand:
        """Build a goal position WRITE DATA command."""
        data = (
            write_u16_le(position_units)
            + write_u16_le(time_units)
            + write_u16_le(speed_units)
        )
        return St3215Protocol.write_data_command(device_id, St3215Register.GOAL_POSITION, data)

    @staticmethod
    def read_present_position_command(device_id: int) -> DeviceCommand:
        """Build a present position READ DATA command."""
        return St3215Protocol.read_data_command(device_id, St3215Register.PRESENT_POSITION, 2)

    @staticmethod
    def parse_status(reading: DeviceReading) -> St3215Status:
        """Parse a decoded device reading as an ST3215 status."""
        if not reading.payload:
            raise St3215ProtocolError("ST3215 status payload is too short")
        return St3215Status(
            device_id=reading.device_id,
            error=reading.payload[0],
            parameters=reading.payload[1:],
        )

    def encode_command(self, command: DeviceCommand) -> ProtocolFrame:
        """Encode a command into an ST3215 packet frame."""
        if not command.payload:
            raise St3215ProtocolError("missing ST3215 instruction byte")
        device_id = _validate_byte("device_id", command.device_id)
        instruction = command.payload[0]
        parameters = command.payload[1:]
        return ProtocolFrame(
            id=command.device_id,
            payload=encode_packet(device_id, instruction, parameters),
        )

    def decode_reading(self, frame: ProtocolFrame) -> DeviceReading:
        """Decode an ST3215 packet frame into a device reading."""
        packet = decode_packet(frame.payload)
        return DeviceReading(
            device_id=packet.device_id,
            payload=bytes([packet.instruction_or_status]) + packet.parameters,
        )


def encode_packet(device_id: int, instruction_or_status: int, parameters: bytes = b"") -> bytes:
    """Encode one ST3215 packet."""
    device_id = _validate_byte("device_id", device_id)
    instruction_or_status = _validate_byte("instruction_or_status", instruction_or_status)
    if len(parameters) > 253:
        raise St3215ProtocolError(f"too many ST3215 parameters: {len(parameters)}")
    length = len(parameters) + 2
    return (
        HEADER
        + bytes([device_id, length, instruction_or_status])
        + parameters
        + bytes([checksum(device_id, length, instruction_or_status, parameters)])
    )


def decode_packet(packet: bytes) -> _DecodedPacket:
    """Decode one complete ST3215 packet."""
    if len(packet) < 6:
        raise St3215ProtocolError(f"ST3215 packet is too short: {len(packet)}")
    if packet[:2] != HEADER:
        raise St3215ProtocolError("invalid ST3215 packet header")
    device_id = packet[2]
    length = packet[3]
    expected_len = length + 4
    if len(packet) != expected_len:
        raise St3215ProtocolError(
            f"invalid ST3215 packet length: expected {expected_len}, got {len(packet)}"
        )
    instruction_or_status = packet[4]
    parameters = packet[5:-1]
    expected = checksum(device_id, length, instruction_or_status, parameters)
    actual = packet[-1]
    if actual != expected:
        raise St3215ProtocolError(
            f"invalid ST3215 checksum: expected 0x{expected:02x}, got 0x{actual:02x}"
        )
    return _DecodedPacket(device_id, instruction_or_status, parameters)


def checksum(device_id: int, length: int, instruction_or_status: int, parameters: bytes) -> int:
    """Compute ST3215 checksum."""
    total = (device_id + length + instruction_or_status + sum(parameters)) & 0xFF
    return (~total) & 0xFF


def read_u16_le(data: bytes) -> int:
    """Read a little-endian uint16."""
    if len(data) < 2:
        raise St3215ProtocolError("expected at least two bytes")
    return int.from_bytes(data[:2], "little")


def write_u16_le(value: int) -> bytes:
    """Write a little-endian uint16."""
    return int(value & 0xFFFF).to_bytes(2, "little")


def _instruction_command(
    device_id: int,
    instruction: St3215Instruction,
    parameters: bytes,
) -> DeviceCommand:
    return DeviceCommand(device_id=device_id, payload=bytes([instruction]) + parameters)


def _validate_byte(name: str, value: int) -> int:
    if not 0 <= value <= 0xFF:
        raise St3215ProtocolError(f"invalid ST3215 {name}: {value}")
    return value
