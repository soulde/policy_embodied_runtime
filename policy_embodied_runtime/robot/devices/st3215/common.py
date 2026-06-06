"""Shared ST3215 servo helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass

from policy_embodied_runtime.protocol import St3215Status
from policy_embodied_runtime.protocol.st3215 import St3215ProtocolError, read_u16_le
from policy_embodied_runtime.robot.data import RobotData, ServoCommand, ServoFeedbackData


@dataclass(frozen=True, slots=True)
class St3215ServoConfig:
    """ST3215 servo mapping and default motion parameters."""

    servo_id: int
    device_id: int
    feedback_field: str | None = None
    command_field: str | None = None
    max_position_units: int = 4095
    speed_units: int = 0
    time_units: int = 0


class St3215DeviceError(RuntimeError):
    """ST3215 sensor/actuator error."""


def servo_feedback_from_status(
    config: St3215ServoConfig,
    status: St3215Status,
) -> ServoFeedbackData:
    """Convert ST3215 status into robot servo feedback."""
    raw_position = _parse_position(status)
    return ServoFeedbackData(
        servo_id=config.servo_id,
        position_rad=position_units_to_radians(raw_position, config.max_position_units),
        raw_position=raw_position,
        status_error=status.error,
    )


def radians_to_position_units(position_rad: float, max_position_units: int) -> int:
    """Convert radians into ST3215 position units."""
    normalized = position_rad % math.tau / math.tau
    return int(round(normalized * max_position_units))


def position_units_to_radians(position_units: int, max_position_units: int) -> float:
    """Convert ST3215 position units into radians."""
    return position_units / max_position_units * math.tau


def find_servo_command(data: RobotData, servo_id: int) -> ServoCommand | None:
    """Find a servo command by servo id."""
    for command in data.commands.get("servos", []):
        if isinstance(command, ServoCommand) and command.servo_id == servo_id:
            return command
    return None


def _parse_position(status: St3215Status) -> int:
    try:
        return read_u16_le(status.parameters)
    except St3215ProtocolError as exc:
        raise St3215DeviceError(
            f"invalid ST3215 position status from device {status.device_id}: "
            f"{len(status.parameters)} parameter bytes"
        ) from exc
