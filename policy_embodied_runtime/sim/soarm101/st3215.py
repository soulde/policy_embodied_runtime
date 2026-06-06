from __future__ import annotations

from dataclasses import dataclass


HEADER = b"\xff\xff"
BROADCAST_ID = 0xFE

PING = 0x01
READ_DATA = 0x02
WRITE_DATA = 0x03

ADDR_GOAL_POSITION = 0x2A
ADDR_PRESENT_POSITION = 0x38


@dataclass(frozen=True)
class Packet:
    device_id: int
    instruction: int
    parameters: bytes


class PacketError(ValueError):
    pass


def checksum(device_id: int, length: int, instruction_or_error: int, parameters: bytes) -> int:
    total = (device_id + length + instruction_or_error + sum(parameters)) & 0xFF
    return (~total) & 0xFF


def encode_packet(device_id: int, instruction_or_error: int, parameters: bytes = b"") -> bytes:
    length = len(parameters) + 2
    return (
        HEADER
        + bytes([device_id, length, instruction_or_error])
        + parameters
        + bytes([checksum(device_id, length, instruction_or_error, parameters)])
    )


def encode_status(device_id: int, error: int = 0, parameters: bytes = b"") -> bytes:
    return encode_packet(device_id, error, parameters)


def try_decode_packet(buffer: bytearray) -> Packet | None:
    while len(buffer) >= 2 and bytes(buffer[:2]) != HEADER:
        del buffer[0]

    if len(buffer) < 6:
        return None

    length = buffer[3]
    packet_len = length + 4
    if len(buffer) < packet_len:
        return None

    raw = bytes(buffer[:packet_len])
    del buffer[:packet_len]

    device_id = raw[2]
    instruction = raw[4]
    parameters = raw[5:-1]
    expected = checksum(device_id, length, instruction, parameters)
    actual = raw[-1]
    if actual != expected:
        raise PacketError(f"bad checksum: expected 0x{expected:02x}, got 0x{actual:02x}")

    return Packet(device_id=device_id, instruction=instruction, parameters=parameters)


def read_u16_le(data: bytes) -> int:
    if len(data) < 2:
        raise PacketError("expected at least two bytes")
    return int.from_bytes(data[:2], "little")


def write_u16_le(value: int) -> bytes:
    return int(value & 0xFFFF).to_bytes(2, "little")
