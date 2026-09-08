from __future__ import annotations

from dataclasses import dataclass

ENABLE = bytes.fromhex("ff ff ff ff ff ff ff fc")
DISABLE = bytes.fromhex("ff ff ff ff ff ff ff fd")
ZERO_POSITION = bytes.fromhex("ff ff ff ff ff ff ff fe")


@dataclass(frozen=True)
class Limits:
    position: float = 12.5
    velocity: float = 30.0
    torque: float = 10.0


@dataclass(frozen=True)
class MitCommand:
    position: float
    velocity: float
    kp: float
    kd: float
    torque: float


def _decode(code: int, maximum_code: int, minimum: float, maximum: float) -> float:
    return minimum + code / maximum_code * (maximum - minimum)


def _encode(value: float, minimum: float, maximum: float, maximum_code: int) -> int:
    ratio = min(1.0, max(0.0, (value - minimum) / (maximum - minimum)))
    return int(ratio * maximum_code + 0.5)


def decode_mit(payload: bytes, limits: Limits) -> MitCommand:
    if len(payload) != 8:
        raise ValueError("Damiao MIT payload must contain 8 bytes")
    position = payload[0] << 8 | payload[1]
    velocity = payload[2] << 4 | payload[3] >> 4
    kp = (payload[3] & 0x0F) << 8 | payload[4]
    kd = payload[5] << 4 | payload[6] >> 4
    torque = (payload[6] & 0x0F) << 8 | payload[7]
    return MitCommand(
        _decode(position, 0xFFFF, -limits.position, limits.position),
        _decode(velocity, 0xFFF, -limits.velocity, limits.velocity),
        _decode(kp, 0xFFF, 0.0, 500.0),
        _decode(kd, 0xFFF, 0.0, 5.0),
        _decode(torque, 0xFFF, -limits.torque, limits.torque),
    )


def encode_feedback(
    motor_id: int,
    position: float,
    velocity: float,
    torque: float,
    limits: Limits,
    error_code: int = 0,
    mos_temperature_c: int = 25,
    rotor_temperature_c: int = 25,
) -> bytes:
    p = _encode(position, -limits.position, limits.position, 0xFFFF)
    v = _encode(velocity, -limits.velocity, limits.velocity, 0xFFF)
    t = _encode(torque, -limits.torque, limits.torque, 0xFFF)
    return bytes(
        (
            (error_code & 0x0F) << 4 | motor_id & 0x0F,
            p >> 8,
            p & 0xFF,
            v >> 4,
            (v & 0x0F) << 4 | t >> 8,
            t & 0xFF,
            mos_temperature_c & 0xFF,
            rotor_temperature_c & 0xFF,
        )
    )
