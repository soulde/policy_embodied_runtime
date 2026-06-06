"""Robot-layer runtime orchestration."""

from __future__ import annotations

import time
import threading
from pathlib import Path
from typing import Any

from pydantic import ValidationError

import policy_embodied_runtime.models  # noqa: F401

from policy_embodied_runtime.models.policy import BaseInferencePolicy
from policy_embodied_runtime.protocol.messages import (
    ActionPayload,
    ActionResponse,
    ErrorPayload,
    HealthRequest,
    HealthResponse,
    MessageEnvelope,
    ObservationRequest,
    ResetRequest,
    SCHEMA_VERSION,
    ServerInfoResponse,
)
from policy_embodied_runtime.robot.actuator import Actuator
from policy_embodied_runtime.robot.errors import ValidationRuntimeError
from policy_embodied_runtime.robot.factory import BuiltRobot, build_robot
from policy_embodied_runtime.robot.session_manager import SessionManager
from policy_embodied_runtime.robot.data import RobotAction, RobotData, RobotObservation
from policy_embodied_runtime.robot.devices.rpc import RpcActionActuator, RpcFeedbackActuator, RpcObservationSensor
from policy_embodied_runtime.robot.policy import ModelPolicy, Policy
from policy_embodied_runtime.robot.profile import DeviceLink, RobotProfile
from policy_embodied_runtime.robot.registry import create_registered
from policy_embodied_runtime.robot.sensor import Sensor
from policy_embodied_runtime.postprocess.pipeline import build_postprocessors, run_postprocessors
from policy_embodied_runtime.preprocess.pipeline import build_preprocessors, run_preprocessors
from policy_embodied_runtime.safety.checks import validate_required_fields
from policy_embodied_runtime.protocol.policy_profile import PolicyProfile


class Runtime:
    """Main robot runtime entrypoint."""

    def __init__(
        self,
        *,
        policy_profile: PolicyProfile,
        robot_profile: RobotProfile | None = None,
        robot: BuiltRobot | None = None,
        session_manager: SessionManager | None = None,
    ) -> None:
        self._policy_profile = policy_profile
        self._policy = _create_robot_policy(
            "control",
            policy_profile.policy,
            policy_profile=policy_profile,
        )
        if robot is None:
            robot = build_robot(robot_profile or RobotProfile())
        self.robot_loop = RobotLoopRuntime(
            policy_profile=policy_profile,
            policy=self._policy,
            robot=robot,
            session_manager=session_manager,
        )

    @classmethod
    def from_profiles(
        cls,
        *,
        policy_profile: str | Path,
        robot_profile: str | Path | None = None,
        session_manager: SessionManager | None = None,
    ) -> "Runtime":
        """Load profiles and build a runtime."""
        from policy_embodied_runtime.profiles.loader import load_policy_profile, load_robot_profile

        loaded_robot_profile = load_robot_profile(robot_profile) if robot_profile is not None else None
        return cls(
            policy_profile=load_policy_profile(policy_profile),
            robot_profile=loaded_robot_profile,
            session_manager=session_manager,
        )

    def open(self) -> None:
        """Open robot runtime resources."""
        self.robot_loop.open()

    def close(self) -> None:
        """Close robot runtime resources."""
        self.robot_loop.close()

    def health(self) -> HealthResponse:
        """Return current runtime health."""
        return HealthResponse(ok=True, policy_id=self._policy_profile.id)

    def server_info(self) -> ServerInfoResponse:
        """Return runtime capability information."""
        return ServerInfoResponse(
            server_name="policy_embodied_runtime",
            server_version="0.1.0",
            supported_schema=SCHEMA_VERSION,
            transports=["zmq+json"],
        )

    def reset(self, session_id: str, request: ResetRequest) -> dict[str, Any]:
        """Reset a runtime session."""
        self.robot_loop.reset(session_id)
        return {"ok": True, "hard": request.hard}

    def infer(self, session_id: str, request: ObservationRequest) -> ActionResponse:
        """Publish one RPC observation and run one robot control step."""
        robot_data = self.robot_loop.step(
            session_id,
            request.observation.as_policy_input(),
        )
        if robot_data.action is None:
            raise RuntimeError("policy inference did not produce an action")
        return ActionResponse(ok=True, action=ActionPayload.model_validate(robot_data.action.as_dict()))

    def handle_envelope(self, envelope: MessageEnvelope) -> MessageEnvelope:
        """Dispatch an envelope to runtime APIs and wrap the response."""
        try:
            payload = self._handle_request(envelope)
            return MessageEnvelope(
                schema=SCHEMA_VERSION,
                type=self._response_type_for(envelope.type),
                request_id=envelope.request_id,
                session_id=envelope.session_id,
                step_id=envelope.step_id,
                timestamp_ns=time.time_ns(),
                payload=payload,
            )
        except (ValidationRuntimeError, ValidationError, ValueError) as exc:
            return self.error_envelope(
                request_type=envelope.type,
                request_id=envelope.request_id,
                session_id=envelope.session_id,
                step_id=envelope.step_id,
                code="runtime_error",
                message=str(exc),
            )
        except Exception as exc:  # pragma: no cover - defensive fallback
            return self.error_envelope(
                request_type=envelope.type,
                request_id=envelope.request_id,
                session_id=envelope.session_id,
                step_id=envelope.step_id,
                code="internal_error",
                message=str(exc),
            )

    def error_envelope(
        self,
        *,
        request_type: str,
        request_id: str,
        session_id: str,
        step_id: int,
        code: str,
        message: str,
        details: dict[str, Any] | None = None,
    ) -> MessageEnvelope:
        """Build a standard error envelope."""
        return MessageEnvelope(
            schema=SCHEMA_VERSION,
            type=f"{request_type}_error",
            request_id=request_id,
            session_id=session_id,
            step_id=step_id,
            timestamp_ns=time.time_ns(),
            payload={},
            error=ErrorPayload(code=code, message=message, details=details or {}),
        )

    def _handle_request(self, envelope: MessageEnvelope) -> dict[str, Any]:
        if envelope.type == "health_request":
            HealthRequest.model_validate(envelope.payload)
            return self.health().model_dump()
        if envelope.type == "server_info_request":
            return self.server_info().model_dump()
        if envelope.type == "reset_request":
            request = ResetRequest.model_validate(envelope.payload)
            return self.reset(envelope.session_id, request)
        if envelope.type == "observation_request":
            request = ObservationRequest.model_validate(envelope.payload)
            return self.infer(envelope.session_id, request).model_dump()
        raise ValidationRuntimeError(f"unknown request type: {envelope.type}")

    def _response_type_for(self, request_type: str) -> str:
        mapping = {
            "health_request": "health_response",
            "server_info_request": "server_info_response",
            "reset_request": "reset_response",
            "observation_request": "action_response",
        }
        try:
            return mapping[request_type]
        except KeyError as exc:
            raise ValidationRuntimeError(f"unknown request type: {request_type}") from exc


class RobotLoopRuntime:
    """Policy-driven robot loop over all configured sensors and actuators."""

    def __init__(
        self,
        *,
        policy_profile: PolicyProfile,
        policy: Policy,
        robot: BuiltRobot,
        session_manager: SessionManager | None = None,
    ) -> None:
        self._policy_profile = policy_profile
        self._policy = policy
        self._robot = robot
        self._session_manager = session_manager or SessionManager()
        self._policy_preprocessors = build_preprocessors(policy_profile.preprocess)
        self._policy_postprocessors = build_postprocessors(policy_profile.postprocess)
        self._sensor_state = RobotData().sensors
        self._sensor_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._sensor_threads: list[threading.Thread] = []
        self._device_locks: dict[DeviceLink, threading.Lock] = {
            device: threading.Lock()
            for device in {
                *robot.sensor_devices.values(),
                *robot.actuator_devices.values(),
            }
        }

    def open(self) -> None:
        """Open transports and start one sensor worker per physical device."""
        if self._sensor_threads:
            return
        self._open_transports()
        self._stop_event.clear()
        for device, sensors in self._sensors_by_device().items():
            thread = threading.Thread(
                target=self._sensor_worker,
                args=(device, sensors),
                daemon=True,
            )
            thread.start()
            self._sensor_threads.append(thread)

    def close(self) -> None:
        """Stop sensor workers and close transports."""
        self._stop_event.set()
        for thread in self._sensor_threads:
            thread.join(timeout=1.0)
        self._sensor_threads.clear()
        self._close_transports()

    def reset(self, session_id: str) -> dict[str, bool]:
        """Reset robot-layer session state."""
        self._session_manager.reset(session_id)
        self._policy.reset(session_id)
        return {"ok": True}

    def publish_observation(self, observation: dict[str, Any]) -> None:
        """Update RPC observation sensors with externally published robot input."""
        for sensor in self._robot.sensors:
            if isinstance(sensor, RpcObservationSensor):
                sensor.observation = dict(observation)
                self._read_sensor_into_state(sensor)

    def step(self, session_id: str, observation: dict[str, Any] | None = None) -> RobotData:
        """Run one policy/action step from latest sensor state."""
        if observation is not None:
            self.publish_observation(observation)
        robot_data = RobotData()
        if observation is not None:
            robot_data.observation = RobotObservation(dict(observation))
        with self._sensor_lock:
            robot_data.sensors.fields.update(self._sensor_state.as_dict())
        robot_data.observation = self._to_canonical_observation(self._policy_observation(robot_data))
        robot_data.observation = self._preprocess(robot_data.observation)

        session_ctx = self._session_manager.get_or_create(session_id)
        self._policy.infer(robot_data, session_ctx)
        if robot_data.action is None:
            raise RuntimeError("policy inference did not produce an action")
        robot_data.set_action(self._postprocess(robot_data.action))
        self._write_actuators(robot_data)
        self._publish_feedback(robot_data)

        session_ctx.step_id += 1
        return robot_data

    def _to_canonical_observation(self, obs_msg: dict[str, Any]) -> RobotObservation:
        required_fields = [
            field.name
            for field in self._policy_profile.canonical_observation_schema
            if not field.optional
        ]
        validate_required_fields(obs_msg, required_fields)
        return RobotObservation(obs_msg)

    def _preprocess(self, observation: RobotObservation) -> RobotObservation:
        return RobotObservation(run_preprocessors(observation.as_dict(), self._policy_preprocessors))

    def _postprocess(self, action: RobotAction) -> RobotAction:
        return RobotAction(run_postprocessors(action.as_dict(), self._policy_postprocessors))

    def _policy_observation(self, data: RobotData) -> dict[str, Any]:
        sensor_fields = data.sensors.as_dict()
        observation = {}
        for binding in self._policy_profile.inputs:
            if binding.robot_data in sensor_fields:
                observation[binding.canonical_field] = sensor_fields[binding.robot_data]
            elif binding.canonical_field in sensor_fields:
                observation[binding.canonical_field] = sensor_fields[binding.canonical_field]
        if not observation:
            observation = data.observation.as_dict()
        return observation

    def _write_actuators(self, data: RobotData) -> None:
        for device, actuators in self._actuators_by_device().items():
            lock = self._device_locks.setdefault(device, threading.Lock())
            with lock:
                for actuator in actuators:
                    actuator.write(data)
        for actuator in self._robot.actuators:
            if isinstance(actuator, RpcActionActuator):
                actuator.write(data)

    def _publish_feedback(self, data: RobotData) -> None:
        for actuator in self._robot.actuators:
            if isinstance(actuator, RpcFeedbackActuator):
                actuator.write(data)

    def _sensor_worker(self, device: DeviceLink, sensors: list[Sensor]) -> None:
        lock = self._device_locks.setdefault(device, threading.Lock())
        while not self._stop_event.is_set():
            with lock:
                for sensor in sensors:
                    self._read_sensor_into_state(sensor)
            time.sleep(0.01)

    def _read_sensor_into_state(self, sensor: Sensor) -> None:
        data = RobotData()
        sensor.read(data)
        if not data.sensors.fields and not data.observation.fields:
            return
        with self._sensor_lock:
            self._sensor_state.fields.update(data.sensors.as_dict())
            self._sensor_state.fields.update(data.observation.as_dict())

    def _sensors_by_device(self) -> dict[DeviceLink, list[Sensor]]:
        groups: dict[DeviceLink, list[Sensor]] = {}
        for sensor in self._robot.sensors:
            if isinstance(sensor, RpcObservationSensor):
                continue
            device = self._robot.sensor_devices.get(sensor.name())
            if device is not None:
                groups.setdefault(device, []).append(sensor)
        return groups

    def _actuators_by_device(self) -> dict[DeviceLink, list[Actuator]]:
        groups: dict[DeviceLink, list[Actuator]] = {}
        for actuator in self._robot.actuators:
            if isinstance(actuator, (RpcActionActuator, RpcFeedbackActuator)):
                continue
            device = self._robot.actuator_devices.get(actuator.name())
            if device is not None:
                groups.setdefault(device, []).append(actuator)
        return groups

    def _open_transports(self) -> None:
        for endpoint in self._transport_endpoints():
            endpoint.transport.open()

    def _close_transports(self) -> None:
        for endpoint in self._transport_endpoints():
            endpoint.transport.close()

    def _transport_endpoints(self):
        seen = set()
        for item in [*self._robot.sensors, *self._robot.actuators]:
            endpoint = getattr(item, "endpoint", None)
            if endpoint is None or id(endpoint.transport) in seen:
                continue
            seen.add(id(endpoint.transport))
            yield endpoint


def _create_inference_policy(name: str, *, policy_profile: PolicyProfile) -> BaseInferencePolicy:
    try:
        policy = create_registered("policy", name, policy_profile=policy_profile)
    except KeyError as exc:
        raise ValidationRuntimeError(f"unknown policy: {name}") from exc
    if not isinstance(policy, BaseInferencePolicy):
        raise TypeError(f"policy {name} is not a BaseInferencePolicy")
    return policy


def _create_robot_policy(name: str, policy_name: str, *, policy_profile: PolicyProfile) -> Policy:
    return ModelPolicy(
        name=name,
        inference_policy=_create_inference_policy(policy_name, policy_profile=policy_profile),
    )
