"""Schema for policy profile configuration."""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, ConfigDict, Field, model_validator

from policy_embodied_runtime.protocol.common import ProcessorSpec, SemanticField, TemporalSpec


class RobotPolicyBinding(BaseModel):
    """Binding between one unique robot data field and one policy field."""

    model_config = ConfigDict(extra="forbid")

    name: str
    robot_data: str
    canonical_field: str
    source_path: str | None = None
    target_path: str | None = None
    optional: bool = False
    metadata: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_names(self) -> "RobotPolicyBinding":
        if not self.name.strip():
            raise ValueError("binding name must be non-empty")
        if not self.robot_data.strip():
            raise ValueError("binding robot_data must be non-empty")
        if not self.canonical_field.strip():
            raise ValueError("binding canonical_field must be non-empty")
        return self


class PolicyProfile(BaseModel):
    """Describes which policy to use and its canonical I/O contract."""

    model_config = ConfigDict(extra="forbid")

    id: str
    version: str
    policy: str
    model: dict[str, Any] = Field(default_factory=dict)
    inputs: list[RobotPolicyBinding] = Field(default_factory=list)
    outputs: list[RobotPolicyBinding] = Field(default_factory=list)
    canonical_observation_schema: list[SemanticField]
    canonical_action_schema: list[SemanticField]
    temporal: TemporalSpec = Field(default_factory=TemporalSpec)
    preprocess: list[ProcessorSpec] = Field(default_factory=list)
    postprocess: list[ProcessorSpec] = Field(default_factory=list)
    safety: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_schema_names(self) -> "PolicyProfile":
        obs_names = [field.name for field in self.canonical_observation_schema]
        action_names = [field.name for field in self.canonical_action_schema]
        if len(obs_names) != len(set(obs_names)):
            raise ValueError("canonical_observation_schema contains duplicate field names")
        if len(action_names) != len(set(action_names)):
            raise ValueError("canonical_action_schema contains duplicate field names")
        _validate_unique_bindings(self.inputs, "inputs")
        _validate_unique_bindings(self.outputs, "outputs")
        _validate_bound_fields(self.inputs, set(obs_names), "inputs")
        _validate_bound_fields(self.outputs, set(action_names), "outputs")
        if bool(self.inputs) != bool(self.outputs) or len(self.inputs) != len(self.outputs):
            raise ValueError("policy inputs and outputs must be declared as equal-length pairs")
        if not self.id.strip():
            raise ValueError("id must be non-empty")
        if not self.policy.strip():
            raise ValueError("policy must be non-empty")
        return self


def _validate_unique_bindings(bindings: list[RobotPolicyBinding], section: str) -> None:
    names = [binding.name for binding in bindings]
    robot_data = [binding.robot_data for binding in bindings]
    if len(names) != len(set(names)):
        raise ValueError(f"{section} contains duplicate binding names")
    if len(robot_data) != len(set(robot_data)):
        raise ValueError(f"{section} contains duplicate robot_data names")


def _validate_bound_fields(
    bindings: list[RobotPolicyBinding],
    canonical_fields: set[str],
    section: str,
) -> None:
    for binding in bindings:
        if binding.canonical_field not in canonical_fields:
            raise ValueError(
                f"{section}.{binding.name} references unknown canonical field '{binding.canonical_field}'"
            )
