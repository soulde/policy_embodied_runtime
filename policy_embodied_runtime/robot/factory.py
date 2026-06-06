"""Factories that build robot objects from profiles."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from policy_embodied_runtime.robot.actuator import Actuator, RecordingActuator
from policy_embodied_runtime.robot.devices.rpc import RpcActionActuator, RpcFeedbackActuator, RpcObservationSensor
from policy_embodied_runtime.robot.devices.st3215 import St3215ServoActuator, St3215ServoConfig, St3215ServoSensor
from policy_embodied_runtime.robot.profile import DeviceConfig, DeviceLink, RobotProfile
from policy_embodied_runtime.robot.sensor import Sensor, StaticSensor
from policy_embodied_runtime.transport import Transport
from policy_embodied_runtime.transport.serial import SerialConfig, SerialTransport


@dataclass(slots=True)
class BuiltRobot:
    """Robot objects created from a profile."""

    sensors: list[Sensor]
    actuators: list[Actuator]


def build_robot(profile: RobotProfile) -> BuiltRobot:
    """Build sensors and actuators from a robot profile."""
    transport_cache: dict[DeviceLink, Transport] = {}
    return BuiltRobot(
        sensors=[build_sensor(device, transport_cache) for device in profile.sensors],
        actuators=[build_actuator(device, transport_cache) for device in profile.actuators],
    )


def build_sensor(device: DeviceConfig, transport_cache: dict[DeviceLink, Transport] | None = None) -> Sensor:
    """Build one sensor from config."""
    if device.device.type == "static":
        return StaticSensor(
            device.name,
            device.name,
            _coerce_arg(device.args.get("value", "")),
        )
    if device.device.type == "rpc":
        return RpcObservationSensor(
            {},
            sensor_name=device.name,
        )
    if device.device.type == "st3215":
        return St3215ServoSensor(device.name, _st3215_config(device, is_sensor=True), _transport(device, transport_cache))
    raise ValueError(f"unsupported sensor device type '{device.device.type}' for '{device.name}'")


def build_actuator(device: DeviceConfig, transport_cache: dict[DeviceLink, Transport] | None = None) -> Actuator:
    """Build one actuator from config."""
    if device.device.type == "recording":
        return RecordingActuator(device.name)
    if device.device.type == "rpc":
        mode = device.args.get("mode", "feedback")
        if mode == "action":
            return RpcActionActuator(actuator_name=device.name)
        if mode == "feedback":
            return RpcFeedbackActuator(actuator_name=device.name)
        raise ValueError(f"unsupported rpc actuator mode '{mode}' for '{device.name}'")
    if device.device.type == "st3215":
        return St3215ServoActuator(device.name, _st3215_config(device, is_sensor=False), _transport(device, transport_cache))
    raise ValueError(f"unsupported actuator device type '{device.device.type}' for '{device.name}'")


def _transport(
    device: DeviceConfig,
    transport_cache: dict[DeviceLink, Transport] | None = None,
) -> Transport:
    if device.device.type == "st3215":
        cache = transport_cache if transport_cache is not None else {}
        if device.device not in cache:
            cache[device.device] = SerialTransport(
                SerialConfig.from_args({"path": device.device.path, **device.args})
            )
        return cache[device.device]
    raise ValueError(f"device '{device.name}' does not use a transport")


def _st3215_config(device: DeviceConfig, *, is_sensor: bool) -> St3215ServoConfig:
    servo_id = _int_arg(device, "servo_id", fallback="device_id")
    device_id = _int_arg(device, "device_id", fallback="servo_id")
    return St3215ServoConfig(
        servo_id=servo_id,
        device_id=device_id,
        feedback_field=device.name if is_sensor else None,
        command_field=device.name if not is_sensor else None,
        max_position_units=_int_arg(device, "max_position_units", default=4095),
        speed_units=_int_arg(device, "speed_units", default=0),
        time_units=_int_arg(device, "time_units", default=0),
    )


def _required(device: DeviceConfig, key: str) -> str:
    try:
        return device.args[key]
    except KeyError as exc:
        raise ValueError(f"device '{device.name}' is missing arg '{key}'") from exc


def _int_arg(
    device: DeviceConfig,
    key: str,
    *,
    fallback: str | None = None,
    default: int | None = None,
) -> int:
    value = device.args.get(key)
    if value is None and fallback is not None:
        value = device.args.get(fallback)
    if value is None:
        if default is not None:
            return default
        raise ValueError(f"device '{device.name}' is missing arg '{key}'")
    try:
        return int(value)
    except ValueError as exc:
        raise ValueError(f"device '{device.name}' has invalid arg '{key}' value '{value}'") from exc


def _coerce_arg(value: str) -> Any:
    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value
