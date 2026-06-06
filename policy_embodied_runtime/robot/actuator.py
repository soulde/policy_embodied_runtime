"""Robot actuator concepts."""

from __future__ import annotations

from typing import Protocol

from policy_embodied_runtime.robot.data import RobotData


class Actuator(Protocol):
    """Anything that is output from the robot."""

    def name(self) -> str:
        """Return the configured actuator instance name."""

    def write(self, data: RobotData) -> None:
        """Apply current robot commands."""


class RecordingActuator:
    """In-memory actuator useful for tests and examples."""

    def __init__(self, actuator_name: str) -> None:
        self._name = actuator_name
        self.writes: list[dict[str, object]] = []

    def name(self) -> str:
        """Return the actuator name."""
        return self._name

    def write(self, data: RobotData) -> None:
        """Record a command snapshot."""
        self.writes.append(data.commands.as_dict())
