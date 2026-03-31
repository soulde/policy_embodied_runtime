"""Schema for policy profile configuration."""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, ConfigDict, Field, model_validator

from policy_embodied_runtime.server.schemas.common import ProcessorSpec, SemanticField, TemporalSpec


class PolicyProfile(BaseModel):
    """Describes which model adapter to use and its canonical I/O contract."""

    model_config = ConfigDict(extra="forbid")

    id: str
    version: str
    model_adapter: str
    model: dict[str, Any] = Field(default_factory=dict)
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
        if not self.id.strip():
            raise ValueError("id must be non-empty")
        if not self.model_adapter.strip():
            raise ValueError("model_adapter must be non-empty")
        return self
