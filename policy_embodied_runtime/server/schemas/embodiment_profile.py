"""Schema for embodiment adapter configuration."""

from __future__ import annotations

from typing import Any

from pydantic import BaseModel, ConfigDict, Field, model_validator

from policy_embodied_runtime.server.schemas.common import MappingProcessorRule, ProcessorSpec, SemanticBounds


class LimitSpec(BaseModel):
    """Limits associated with a named field."""

    model_config = ConfigDict(extra="forbid")

    bounds: SemanticBounds
    unit: str | None = None


class EmbodimentProfile(BaseModel):
    """Embodiment-specific mapping from external messages to canonical semantics."""

    model_config = ConfigDict(extra="forbid")

    id: str
    adapter: str
    joint_names: list[str] = Field(default_factory=list)
    units: dict[str, str] = Field(default_factory=dict)
    limits: dict[str, LimitSpec] = Field(default_factory=dict)
    frame_info: dict[str, str] = Field(default_factory=dict)
    preprocess: list[ProcessorSpec] = Field(default_factory=list)
    postprocess: list[ProcessorSpec] = Field(default_factory=list)

    @model_validator(mode="after")
    def validate_profile(self) -> "EmbodimentProfile":
        if not self.id.strip():
            raise ValueError("id must be non-empty")
        if not self.adapter.strip():
            raise ValueError("adapter must be non-empty")
        if len(self.joint_names) != len(set(self.joint_names)):
            raise ValueError("joint_names contains duplicates")
        for processor in [*self.preprocess, *self.postprocess]:
            if processor.name == "mapping":
                rules = processor.params.get("rules", [])
                for rule_data in rules:
                    rule = MappingProcessorRule.model_validate(rule_data)
                    if not rule.canonical_field.strip():
                        raise ValueError("canonical_field must be non-empty")
        return self
