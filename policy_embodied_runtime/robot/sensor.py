"""Robot sensor concepts."""

from __future__ import annotations

from typing import Protocol

from policy_embodied_runtime.robot.data import RobotData


class Sensor(Protocol):
    """Anything that is input to the robot."""

    def name(self) -> str:
        """Return the configured sensor instance name."""

    def read(self, data: RobotData) -> None:
        """Read external state into robot data."""


class StaticSensor:
    """Simple in-memory sensor useful for tests and examples."""

    def __init__(self, sensor_name: str, field: str, value: object) -> None:
        self._name = sensor_name
        self._field = field
        self._value = value

    def name(self) -> str:
        """Return the sensor name."""
        return self._name

    def read(self, data: RobotData) -> None:
        """Write the configured value into robot sensor data."""
        data.sensors.update(self._field, self._value)
