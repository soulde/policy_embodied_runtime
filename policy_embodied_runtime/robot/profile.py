"""Robot profile parsing."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True, slots=True)
class DeviceConfig:
    """Robot device configuration."""

    name: str
    device_type: str
    args: dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class RobotProfile:
    """Robot profile describing sensors, actuators, and policies."""

    sensors: list[DeviceConfig] = field(default_factory=list)
    actuators: list[DeviceConfig] = field(default_factory=list)
    policies: list[DeviceConfig] = field(default_factory=list)

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "RobotProfile":
        """Load a robot profile from a JSON-style mapping."""
        sensors = [_device_from_mapping("sensor", item) for item in data.get("sensors", [])]
        actuators = [_device_from_mapping("actuator", item) for item in data.get("actuators", [])]
        policies = [_device_from_mapping("policy", item) for item in data.get("policies", [])]
        _validate_unique_names(sensors, actuators, policies)
        return cls(sensors=sensors, actuators=actuators, policies=policies)

    @classmethod
    def from_json_file(cls, path: str | Path) -> "RobotProfile":
        """Load a robot profile from JSON."""
        return cls.from_mapping(json.loads(Path(path).read_text()))


def _device_from_mapping(kind: str, data: dict[str, Any]) -> DeviceConfig:
    if not isinstance(data, dict):
        raise ValueError(f"{kind} entry must be an object")
    name = str(data.get("name", "")).strip()
    device_type = str(data.get("type", "")).strip()
    if not name:
        raise ValueError(f"{kind} entry is missing name")
    if not device_type:
        raise ValueError(f"{kind}.{name} is missing type")
    args = {
        str(key): _json_arg_value(value)
        for key, value in data.items()
        if key not in {"name", "type"}
    }
    return DeviceConfig(name=name, device_type=device_type, args=args)


def _json_arg_value(value: Any) -> str:
    if isinstance(value, str):
        return value
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _validate_unique_names(
    sensors: list[DeviceConfig],
    actuators: list[DeviceConfig],
    policies: list[DeviceConfig],
) -> None:
    names: dict[str, str] = {}
    for kind, devices in (("sensor", sensors), ("actuator", actuators), ("policy", policies)):
        for device in devices:
            previous = names.setdefault(device.name, kind)
            if previous != kind:
                raise ValueError(f"duplicate device name '{device.name}' in {previous} and {kind}")
