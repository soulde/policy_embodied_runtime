"""Policy RPC protocol message schemas."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

SCHEMA_VERSION = "embodied-policy-runtime/v1alpha1"


class ErrorPayload(BaseModel):
    """Structured error payload returned by the server."""

    model_config = ConfigDict(extra="forbid")

    code: str
    message: str
    details: dict[str, Any] = Field(default_factory=dict)


class JointStateValue(BaseModel):
    """Named joint vector value with ordering metadata."""

    model_config = ConfigDict(extra="forbid")

    values: list[float]
    joint_names: list[str]
    unit: str

    @model_validator(mode="after")
    def validate_joint_shape(self) -> "JointStateValue":
        if len(self.values) != len(self.joint_names):
            raise ValueError("values and joint_names must have the same length")
        return self


class GripperValue(BaseModel):
    """Gripper scalar with semantic metadata."""

    model_config = ConfigDict(extra="forbid")

    value: float
    unit: str


class TaskTextValue(BaseModel):
    """Task text payload."""

    model_config = ConfigDict(extra="forbid")

    text: str


class ImageValue(BaseModel):
    """JSON-compatible image placeholder."""

    model_config = ConfigDict(extra="forbid")

    encoding: Literal["base64", "uri", "inline_json"] = "base64"
    data: str
    mime_type: str = "image/jpeg"


class ObservationPayload(BaseModel):
    """Named observation fields accepted by the MVP."""

    model_config = ConfigDict(extra="allow")

    joint_position: JointStateValue | None = None
    gripper_width: GripperValue | None = None
    task_text: TaskTextValue | None = None
    image: ImageValue | None = None
    meta: dict[str, Any] = Field(default_factory=dict)

    def as_adapter_input(self) -> dict[str, Any]:
        """Return the observation payload including adapter-specific extra fields."""
        payload = self.model_dump(exclude_none=True)
        extra_fields = getattr(self, "__pydantic_extra__", None) or {}
        for key, value in extra_fields.items():
            payload.setdefault(key, value)
        return payload


class ActionPayload(BaseModel):
    """Named action fields returned by the server."""

    model_config = ConfigDict(extra="forbid")

    joint_position_delta: JointStateValue | None = None
    gripper_command: GripperValue | None = None
    action_chunk: list[dict[str, Any]] = Field(default_factory=list)
    meta: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_action_modes(self) -> "ActionPayload":
        if not self.action_chunk and self.joint_position_delta is None and self.gripper_command is None:
            raise ValueError("action payload must contain a single-step action or an action_chunk")
        return self


class HealthRequest(BaseModel):
    """Health probe request."""

    model_config = ConfigDict(extra="forbid")

    verbose: bool = False


class HealthResponse(BaseModel):
    """Health probe response."""

    model_config = ConfigDict(extra="forbid")

    ok: bool
    policy_id: str


class ServerInfoResponse(BaseModel):
    """Server capability summary."""

    model_config = ConfigDict(extra="forbid")

    server_name: str
    server_version: str
    supported_schema: str = SCHEMA_VERSION
    transports: list[str] = Field(default_factory=lambda: ["zmq+json"])


class ResetRequest(BaseModel):
    """Reset an existing session."""

    model_config = ConfigDict(extra="forbid")

    hard: bool = False


class ObservationRequest(BaseModel):
    """Inference request containing the current observation."""

    model_config = ConfigDict(extra="forbid")

    observation: ObservationPayload


class ActionResponse(BaseModel):
    """Inference response with a named action payload."""

    model_config = ConfigDict(extra="forbid")

    ok: bool
    action: ActionPayload


class MessageEnvelope(BaseModel):
    """Top-level protocol envelope for all requests and responses."""

    model_config = ConfigDict(extra="forbid", populate_by_name=True)

    schema_: str = Field(default=SCHEMA_VERSION, alias="schema")
    type: str
    request_id: str
    session_id: str
    step_id: int = 0
    timestamp_ns: int
    payload: dict[str, Any] = Field(default_factory=dict)
    error: ErrorPayload | None = None

    @model_validator(mode="after")
    def validate_envelope(self) -> "MessageEnvelope":
        if not self.type.strip():
            raise ValueError("type must be non-empty")
        if not self.request_id.strip():
            raise ValueError("request_id must be non-empty")
        if not self.session_id.strip():
            raise ValueError("session_id must be non-empty")
        if self.timestamp_ns < 0:
            raise ValueError("timestamp_ns must be >= 0")
        return self
