"""Dummy embodiment adapter."""

from __future__ import annotations

from typing import Any

from policy_embodied_runtime.adapters.base import BaseEmbodimentAdapter
from policy_embodied_runtime.adapters.pipeline import (
    BaseEmbodimentPostprocessor,
    BaseEmbodimentPreprocessor,
    build_embodiment_postprocessors,
    build_embodiment_preprocessors,
    input_mapping_rules,
    output_mapping_rules,
    run_embodiment_postprocess,
    run_embodiment_preprocess,
)
from policy_embodied_runtime.protocol.embodiment_profile import EmbodimentProfile
from policy_embodied_runtime.protocol.policy_profile import PolicyProfile


class DummyEmbodimentAdapter(BaseEmbodimentAdapter):
    """Pass-through embodiment adapter with mapping validation."""

    __registry_name__ = "dummy"

    def __init__(self) -> None:
        self._profile: EmbodimentProfile | None = None
        self._policy_profile: PolicyProfile | None = None
        self._preprocessors: list[BaseEmbodimentPreprocessor] = []
        self._postprocessors: list[BaseEmbodimentPostprocessor] = []

    def setup(self, profile: EmbodimentProfile, policy_profile: PolicyProfile) -> None:
        self._profile = profile
        self._policy_profile = policy_profile
        self._preprocessors = build_embodiment_preprocessors(profile, policy_profile)
        self._postprocessors = build_embodiment_postprocessors(profile, policy_profile)

    def validate_input(self, obs_msg: dict[str, Any]) -> None:
        if self._profile is None:
            raise RuntimeError("adapter profile is not configured")
        required_fields = [
            self._mapping_source(key, rule)
            for key, rule in input_mapping_rules(self._profile).items()
            if not key.startswith("optional:")
        ]
        missing = [field for field in required_fields if not self._path_exists(obs_msg, field)]
        if missing:
            raise ValueError(f"missing embodiment observation fields: {missing}")

    def to_canonical_obs(self, obs_msg: dict[str, Any]) -> dict[str, Any]:
        if self._profile is None:
            raise RuntimeError("adapter profile is not configured")
        return run_embodiment_preprocess(obs_msg, self._preprocessors)

    def from_canonical_action(self, action_msg: dict[str, Any]) -> dict[str, Any]:
        if self._profile is None:
            raise RuntimeError("adapter profile is not configured")
        return run_embodiment_postprocess(action_msg, self._postprocessors)

    def reset(self, session_id: str) -> None:
        _ = session_id

    def schema(self) -> dict[str, Any]:
        if self._profile is None:
            return {}
        return {
            "input_mapping": {key: rule.model_dump() for key, rule in input_mapping_rules(self._profile).items()},
            "output_mapping": {key: rule.model_dump() for key, rule in output_mapping_rules(self._profile).items()},
            "preprocess": [processor.model_dump() for processor in self._profile.preprocess],
            "postprocess": [processor.model_dump() for processor in self._profile.postprocess],
        }

    @staticmethod
    def _mapping_source(external_name: str, rule: Any) -> str:
        return rule.source_field or external_name.removeprefix("optional:")

    @staticmethod
    def _lookup_path(payload: dict[str, Any], field_path: str) -> Any:
        current: Any = payload
        for segment in field_path.split("."):
            if not isinstance(current, dict) or segment not in current:
                return None
            current = current[segment]
        return current

    @staticmethod
    def _path_exists(payload: dict[str, Any], field_path: str) -> bool:
        current: Any = payload
        for segment in field_path.split("."):
            if not isinstance(current, dict) or segment not in current:
                return False
            current = current[segment]
        return True
