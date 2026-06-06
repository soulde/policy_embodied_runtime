"""Factories that build robot objects from profiles."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from policy_embodied_runtime.robot.actuator import Actuator, RecordingActuator
from policy_embodied_runtime.robot.devices.rpc import RpcActionActuator, RpcObservationSensor
from policy_embodied_runtime.robot.devices.st3215 import St3215ServoActuator, St3215ServoConfig, St3215ServoSensor
from policy_embodied_runtime.robot.policy import DummyPolicy, ModelPolicy, Policy
from policy_embodied_runtime.robot.profile import DeviceConfig, RobotProfile
from policy_embodied_runtime.robot.sensor import Sensor, StaticSensor
from policy_embodied_runtime.transport import Transport
from policy_embodied_runtime.transport.serial import SerialConfig, SerialTransport

if TYPE_CHECKING:
    from policy_embodied_runtime.protocol.policy_profile import PolicyProfile


@dataclass(slots=True)
class BuiltRobot:
    """Robot objects created from a profile."""

    sensors: list[Sensor]
    actuators: list[Actuator]
    policies: list[Policy]


def build_robot(profile: RobotProfile, *, policy_profile: PolicyProfile | None = None) -> BuiltRobot:
    """Build sensors, actuators, and policies from a robot profile."""
    return BuiltRobot(
        sensors=[build_sensor(device) for device in profile.sensors],
        actuators=[build_actuator(device) for device in profile.actuators],
        policies=[build_policy(device, policy_profile=policy_profile) for device in profile.policies],
    )


def build_sensor(device: DeviceConfig) -> Sensor:
    """Build one sensor from config."""
    if device.device_type == "static_sensor":
        return StaticSensor(
            device.name,
            _required(device, "field"),
            _coerce_arg(device.args.get("value", "")),
        )
    if device.device_type == "policy_rpc_observation":
        return RpcObservationSensor(
            {},
            sensor_name=device.name,
            field_name=device.args.get("field", "rpc_observation"),
        )
    if device.device_type == "st3215_servo_sensor":
        return St3215ServoSensor(device.name, _st3215_config(device), _transport(device))
    raise ValueError(f"unsupported sensor type '{device.device_type}' for '{device.name}'")


def build_actuator(device: DeviceConfig) -> Actuator:
    """Build one actuator from config."""
    if device.device_type == "recording_actuator":
        return RecordingActuator(device.name)
    if device.device_type == "policy_rpc_action":
        return RpcActionActuator(
            actuator_name=device.name,
            field_name=device.args.get("field", "rpc_action"),
        )
    if device.device_type == "st3215_servo_actuator":
        return St3215ServoActuator(device.name, _st3215_config(device), _transport(device))
    raise ValueError(f"unsupported actuator type '{device.device_type}' for '{device.name}'")


def build_policy(device: DeviceConfig, *, policy_profile: PolicyProfile | None = None) -> Policy:
    """Build one policy from config."""
    if device.device_type == "dummy_policy":
        return DummyPolicy(device.name)
    if device.device_type == "model_policy":
        if policy_profile is None:
            raise ValueError(f"policy '{device.name}' requires a policy profile")
        adapter_name = device.args.get("model_adapter", policy_profile.model_adapter)
        return ModelPolicy(device.name, _model_adapter(adapter_name, policy_profile=policy_profile))
    raise ValueError(f"unsupported policy type '{device.device_type}' for '{device.name}'")


def _transport(device: DeviceConfig) -> Transport:
    transport_type = device.args.get("transport", "serial")
    if transport_type == "serial":
        return SerialTransport(SerialConfig.from_args(device.args))
    raise ValueError(f"unsupported transport type '{transport_type}' for '{device.name}'")


def _st3215_config(device: DeviceConfig) -> St3215ServoConfig:
    servo_id = _int_arg(device, "servo_id", fallback="device_id")
    device_id = _int_arg(device, "device_id", fallback="servo_id")
    return St3215ServoConfig(
        servo_id=servo_id,
        device_id=device_id,
        feedback_field=device.name if device.device_type.endswith("_sensor") else None,
        command_field=device.name if device.device_type.endswith("_actuator") else None,
        max_position_units=_int_arg(device, "max_position_units", default=4095),
        speed_units=_int_arg(device, "speed_units", default=0),
        time_units=_int_arg(device, "time_units", default=0),
    )


def _model_adapter(name: str, *, policy_profile: PolicyProfile) -> object:
    import policy_embodied_runtime.models  # noqa: F401
    from policy_embodied_runtime.models.base import BaseModelAdapter
    from policy_embodied_runtime.robot.registry import create_registered

    try:
        adapter = create_registered("model_adapter", name, policy_profile=policy_profile)
    except KeyError as exc:
        raise ValueError(f"unknown model adapter: {name}") from exc
    if not isinstance(adapter, BaseModelAdapter):
        raise TypeError(f"model adapter {name} is not a BaseModelAdapter")
    return adapter


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
