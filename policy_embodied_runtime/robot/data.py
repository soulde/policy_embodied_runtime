"""Robot-layer data containers."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True, slots=True)
class ServoFeedbackData:
    """Servo position feedback."""

    servo_id: int
    position_rad: float
    raw_position: int
    status_error: int = 0


@dataclass(frozen=True, slots=True)
class ServoCommand:
    """Servo target command."""

    servo_id: int
    target_position_rad: float
    enabled: bool = True


@dataclass(slots=True)
class SensorData:
    """Robot input state keyed by semantic or device field name."""

    fields: dict[str, Any] = field(default_factory=dict)

    def update(self, name: str, value: Any) -> None:
        """Update one sensor field."""
        self.fields[name] = value

    def get(self, name: str, default: Any = None) -> Any:
        """Return one sensor field."""
        return self.fields.get(name, default)

    def as_dict(self) -> dict[str, Any]:
        """Return a shallow copy of sensor fields."""
        return dict(self.fields)

    def clear(self) -> None:
        """Clear all sensor state."""
        self.fields.clear()

    def upsert_servo_feedback(self, feedback: ServoFeedbackData) -> None:
        """Insert or replace servo feedback by servo id."""
        items = list(self.fields.get("servo_feedback", []))
        _upsert_by_id(items, feedback, feedback.servo_id)
        self.fields["servo_feedback"] = items


@dataclass(slots=True)
class ActuatorCommands:
    """Robot output command state keyed by semantic or device field name."""

    fields: dict[str, Any] = field(default_factory=dict)

    def update(self, name: str, value: Any) -> None:
        """Update one actuator command field."""
        self.fields[name] = value

    def get(self, name: str, default: Any = None) -> Any:
        """Return one actuator command field."""
        return self.fields.get(name, default)

    def as_dict(self) -> dict[str, Any]:
        """Return a shallow copy of command fields."""
        return dict(self.fields)

    def clear(self) -> None:
        """Clear all actuator commands."""
        self.fields.clear()

    def upsert_servo_command(self, command: ServoCommand) -> None:
        """Insert or replace servo command by servo id."""
        items = list(self.fields.get("servos", []))
        _upsert_by_id(items, command, command.servo_id)
        self.fields["servos"] = items


@dataclass(frozen=True, slots=True)
class RobotObservation:
    """Canonical observation snapshot used by policy inference."""

    fields: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        """Return a shallow copy of the observation fields."""
        return dict(self.fields)


@dataclass(frozen=True, slots=True)
class RobotAction:
    """Canonical action update produced by policy inference."""

    fields: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        """Return a shallow copy of the action fields."""
        return dict(self.fields)


@dataclass(slots=True)
class RobotData:
    """Robot-layer data exchange object.

    Sensors write into ``sensors``. Policy execution reads an observation
    snapshot and produces ``action``/``commands``. Actuators consume commands.
    """

    observation: RobotObservation = field(default_factory=RobotObservation)
    action: RobotAction | None = None
    sensors: SensorData = field(default_factory=SensorData)
    commands: ActuatorCommands = field(default_factory=ActuatorCommands)

    @classmethod
    def from_observation(cls, observation: dict[str, Any]) -> "RobotData":
        """Create robot data from a canonical observation mapping."""
        return cls(observation=RobotObservation(observation))

    def set_action(self, action: RobotAction) -> None:
        """Set the current policy action and mirror it into command fields."""
        self.action = action
        self.commands = ActuatorCommands(action.as_dict())

    def observation_dict(self) -> dict[str, Any]:
        """Return the current canonical observation as a dictionary."""
        return self.observation.as_dict()

    def upsert_servo_feedback(self, feedback: ServoFeedbackData) -> None:
        """Insert or replace servo feedback."""
        self.sensors.upsert_servo_feedback(feedback)

    def upsert_servo_command(self, command: ServoCommand) -> None:
        """Insert or replace servo command."""
        self.commands.upsert_servo_command(command)


def _upsert_by_id(items: list[Any], value: Any, item_id: int) -> None:
    for index, item in enumerate(items):
        if getattr(item, "servo_id", None) == item_id:
            items[index] = value
            return
    items.append(value)
