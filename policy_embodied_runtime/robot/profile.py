"""Robot profile parsing."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True, slots=True)
class DeviceLink:
    """Physical device link shared by robot sensors and actuators."""

    type: str
    path: str


@dataclass(frozen=True, slots=True)
class DeviceConfig:
    """Robot sensor or actuator configuration."""

    name: str
    device: DeviceLink
    args: dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class RobotProfile:
    """Robot profile describing robot inputs, outputs, and hardware links."""

    sensors: list[DeviceConfig] = field(default_factory=list)
    actuators: list[DeviceConfig] = field(default_factory=list)

    @classmethod
    def from_mapping(cls, data: dict[str, Any]) -> "RobotProfile":
        """Load a robot profile from a JSON-style mapping."""
        sensors = [_device_from_mapping("sensor", item) for item in data.get("sensors", [])]
        actuators = [_device_from_mapping("actuator", item) for item in data.get("actuators", [])]
        _validate_unique_names(sensors, actuators)
        return cls(sensors=sensors, actuators=actuators)

    @classmethod
    def from_json_file(cls, path: str | Path) -> "RobotProfile":
        """Load a robot profile from JSON."""
        return cls.from_mapping(json.loads(Path(path).read_text()))


def _device_from_mapping(kind: str, data: dict[str, Any]) -> DeviceConfig:
    if not isinstance(data, dict):
        raise ValueError(f"{kind} entry must be an object")
    name = str(data.get("name", "")).strip()
    if not name:
        raise ValueError(f"{kind} entry is missing name")
    device = _device_link_from_mapping(kind, name, data.get("device"))
    raw_args = data.get("args", {})
    if raw_args is None:
        raw_args = {}
    if not isinstance(raw_args, dict):
        raise ValueError(f"{kind}.{name}.args must be an object")
    args = {
        str(key): _json_arg_value(value)
        for key, value in raw_args.items()
    }
    return DeviceConfig(name=name, device=device, args=args)


def _device_link_from_mapping(kind: str, name: str, data: Any) -> DeviceLink:
    if not isinstance(data, dict):
        raise ValueError(f"{kind}.{name}.device must be an object")
    device_type = str(data.get("type", "")).strip()
    path = str(data.get("path", "")).strip()
    if not device_type:
        raise ValueError(f"{kind}.{name}.device is missing type")
    if not path:
        raise ValueError(f"{kind}.{name}.device is missing path")
    return DeviceLink(type=device_type, path=path)


def _json_arg_value(value: Any) -> str:
    if isinstance(value, str):
        return value
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _validate_unique_names(
    sensors: list[DeviceConfig],
    actuators: list[DeviceConfig],
) -> None:
    names: dict[str, str] = {}
    for kind, devices in (("sensor", sensors), ("actuator", actuators)):
        for device in devices:
            previous = names.setdefault(device.name, kind)
            if previous != kind:
                raise ValueError(f"duplicate device name '{device.name}' in {previous} and {kind}")
