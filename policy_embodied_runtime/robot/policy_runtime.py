"""Core embodied policy runtime."""

from __future__ import annotations

import time
from typing import Any

from pydantic import ValidationError

import policy_embodied_runtime.adapters  # noqa: F401
import policy_embodied_runtime.models  # noqa: F401

from policy_embodied_runtime.adapters.base import BaseEmbodimentAdapter
from policy_embodied_runtime.robot.registry import create_registered
from policy_embodied_runtime.robot.errors import ValidationRuntimeError
from policy_embodied_runtime.models.base import BaseModelAdapter
from policy_embodied_runtime.robot.policy import ModelPolicy, Policy
from policy_embodied_runtime.robot.runtime import EmbodiedRobotRuntime
from policy_embodied_runtime.protocol.embodiment_profile import EmbodimentProfile
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
from policy_embodied_runtime.protocol.policy_profile import PolicyProfile


class PolicyRuntime:
    """Embodied policy runtime."""

    def __init__(self, *, policy_profile: PolicyProfile, embodiment_profile: EmbodimentProfile) -> None:
        self._policy_profile = policy_profile
        self._embodiment_profile = embodiment_profile
        self._policy = _create_policy(
            "control",
            policy_profile.model_adapter,
            policy_profile=policy_profile,
        )
        self._adapter = _create_embodiment_adapter(embodiment_profile.adapter)
        self._robot_runtime = EmbodiedRobotRuntime(
            policy_profile=policy_profile,
            embodiment_profile=embodiment_profile,
            policy=self._policy,
            adapter=self._adapter,
        )

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
        """Reset a session."""
        response = self._robot_runtime.reset(session_id)
        response["hard"] = request.hard
        return response

    def infer(self, session_id: str, request: ObservationRequest) -> ActionResponse:
        """Execute one inference step."""
        obs_msg = request.observation.as_adapter_input()
        embodiment_action = self._robot_runtime.infer(session_id, obs_msg)
        return ActionResponse(ok=True, action=ActionPayload.model_validate(embodiment_action))

    def handle_envelope(self, envelope: MessageEnvelope) -> MessageEnvelope:
        """Dispatch an envelope to the relevant runtime API and wrap the response."""
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
            request = HealthRequest.model_validate(envelope.payload)
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


def _create_model_adapter(name: str, *, policy_profile: PolicyProfile) -> BaseModelAdapter:
    try:
        adapter = create_registered("model_adapter", name, policy_profile=policy_profile)
    except KeyError as exc:
        raise ValidationRuntimeError(f"unknown model adapter: {name}") from exc
    if not isinstance(adapter, BaseModelAdapter):
        raise TypeError(f"adapter {name} is not a BaseModelAdapter")
    return adapter


def _create_policy(name: str, model_adapter_name: str, *, policy_profile: PolicyProfile) -> Policy:
    return ModelPolicy(
        name=name,
        model_adapter=_create_model_adapter(model_adapter_name, policy_profile=policy_profile),
    )


def _create_embodiment_adapter(name: str) -> BaseEmbodimentAdapter:
    try:
        adapter = create_registered("embodiment_adapter", name)
    except KeyError as exc:
        raise ValidationRuntimeError(f"unknown embodiment adapter: {name}") from exc
    if not isinstance(adapter, BaseEmbodimentAdapter):
        raise TypeError(f"adapter {name} is not a BaseEmbodimentAdapter")
    return adapter
