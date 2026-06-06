"""Common schema types shared across cards, profiles, and messages."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class SemanticBounds(BaseModel):
    """Closed numeric bounds for scalar or array elements."""

    lower: float | None = None
    upper: float | None = None

    @model_validator(mode="after")
    def validate_range(self) -> "SemanticBounds":
        if self.lower is not None and self.upper is not None and self.lower > self.upper:
            raise ValueError("lower must be <= upper")
        return self


class SemanticField(BaseModel):
    """Schema entry describing a semantically named observation or action field."""

    model_config = ConfigDict(extra="forbid")

    name: str
    semantic_type: str
    kind: Literal["scalar", "vector", "text", "image", "object"] = "scalar"
    unit: str | None = None
    ordering: list[str] = Field(default_factory=list)
    bounds: SemanticBounds | None = None
    frame: str | None = None
    normalized: bool = False
    optional: bool = False
    description: str | None = None

    @model_validator(mode="after")
    def validate_semantic_requirements(self) -> "SemanticField":
        if not self.name.strip():
            raise ValueError("field name must be non-empty")
        if not self.semantic_type.strip():
            raise ValueError("semantic_type must be non-empty")
        if self.kind == "vector" and not self.ordering:
            raise ValueError("vector fields must declare ordering")
        return self


class TemporalSpec(BaseModel):
    """Temporal characteristics of an inference policy."""

    model_config = ConfigDict(extra="forbid")

    mode: Literal["single_step", "chunked"] = "single_step"
    action_horizon: int = 1
    observation_history: int = 1

    @model_validator(mode="after")
    def validate_positive_values(self) -> "TemporalSpec":
        if self.action_horizon < 1:
            raise ValueError("action_horizon must be >= 1")
        if self.observation_history < 1:
            raise ValueError("observation_history must be >= 1")
        return self


class MappingRule(BaseModel):
    """Mapping between external embodiment fields and canonical schema fields."""

    model_config = ConfigDict(extra="forbid")

    canonical_field: str
    source_field: str | None = None
    transform: str | None = None
    unit: str | None = None
    frame: str | None = None
    metadata: dict[str, Any] = Field(default_factory=dict)


class MappingProcessorRule(MappingRule):
    """Mapping rule declared inside a mapping processor."""

    external_name: str

    @model_validator(mode="after")
    def validate_external_name(self) -> "MappingProcessorRule":
        if not self.external_name.strip():
            raise ValueError("external_name must be non-empty")
        return self


class ProcessorSpec(BaseModel):
    """Ordered processor specification with free-form parameters."""

    model_config = ConfigDict(extra="forbid")

    name: str
    params: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_name(self) -> "ProcessorSpec":
        if not self.name.strip():
            raise ValueError("processor name must be non-empty")
        return self
