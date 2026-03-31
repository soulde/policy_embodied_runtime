"""Embodiment-side preprocess and postprocess pipeline."""

from __future__ import annotations

from abc import ABC, abstractmethod
from copy import deepcopy
from typing import Any

from policy_embodied_runtime.server.core.auto_registry import AutoRegisteringMeta, create_registered
from policy_embodied_runtime.server.schemas.common import MappingProcessorRule, MappingRule, ProcessorSpec
from policy_embodied_runtime.server.schemas.embodiment_profile import EmbodimentProfile
from policy_embodied_runtime.server.schemas.policy_profile import PolicyProfile


class BaseEmbodimentPreprocessor(ABC, metaclass=AutoRegisteringMeta):
    """Base interface for embodiment-side preprocessors."""

    __registry_category__ = "embodiment_preprocess"

    @abstractmethod
    def process(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Process an embodiment-side observation payload."""


class BaseEmbodimentPostprocessor(ABC, metaclass=AutoRegisteringMeta):
    """Base interface for embodiment-side postprocessors."""

    __registry_category__ = "embodiment_postprocess"

    @abstractmethod
    def process(self, payload: dict[str, Any]) -> dict[str, Any]:
        """Process a canonical or embodiment-side action payload."""


class MappingPreprocessor(BaseEmbodimentPreprocessor):
    """Map embodiment observations into canonical fields."""

    __registry_name__ = "mapping"

    def __init__(self, rules: dict[str, MappingRule]) -> None:
        self._rules = rules

    def process(self, payload: dict[str, Any]) -> dict[str, Any]:
        canonical: dict[str, Any] = {}
        for external_name, rule in self._rules.items():
            source_field = rule.source_field or external_name.removeprefix("optional:")
            if _path_exists(payload, source_field):
                canonical[rule.canonical_field] = _lookup_path(payload, source_field)
        return canonical


class NormalizationPreprocessor(BaseEmbodimentPreprocessor):
    """Normalize canonical observation fields for the model."""

    __registry_name__ = "normalize"

    def __init__(self, profile: EmbodimentProfile, policy_profile: PolicyProfile) -> None:
        self._profile = profile
        self._field_names = {field.name for field in policy_profile.canonical_observation_schema if field.normalized}

    def process(self, payload: dict[str, Any]) -> dict[str, Any]:
        normalized = deepcopy(payload)
        for field_name in self._field_names:
            if field_name in normalized:
                normalized[field_name] = _scale_field(self._profile, field_name, normalized[field_name], normalize=True)
        return normalized


class DenormalizationPostprocessor(BaseEmbodimentPostprocessor):
    """Denormalize canonical action fields back into robot scale."""

    __registry_name__ = "denormalize"

    def __init__(self, profile: EmbodimentProfile, policy_profile: PolicyProfile) -> None:
        self._profile = profile
        self._field_names = {field.name for field in policy_profile.canonical_action_schema if field.normalized}

    def process(self, payload: dict[str, Any]) -> dict[str, Any]:
        denormalized = deepcopy(payload)
        for field_name in self._field_names:
            if field_name in denormalized:
                denormalized[field_name] = _scale_field(
                    self._profile,
                    field_name,
                    denormalized[field_name],
                    normalize=False,
                )
        if "action_chunk" in denormalized:
            denormalized["action_chunk"] = [
                self.process(chunk) if isinstance(chunk, dict) else chunk for chunk in denormalized["action_chunk"]
            ]
        return denormalized


class LimitPostprocessor(BaseEmbodimentPostprocessor):
    """Apply embodiment limits to canonical action fields."""

    __registry_name__ = "limits"

    def __init__(self, profile: EmbodimentProfile) -> None:
        self._profile = profile

    def process(self, payload: dict[str, Any]) -> dict[str, Any]:
        limited = deepcopy(payload)
        for field_name in self._profile.limits:
            if field_name in limited:
                limited[field_name] = _clip_field(self._profile, field_name, limited[field_name])
        if "action_chunk" in limited:
            limited["action_chunk"] = [
                self.process(chunk) if isinstance(chunk, dict) else chunk for chunk in limited["action_chunk"]
            ]
        return limited


class MappingPostprocessor(BaseEmbodimentPostprocessor):
    """Map canonical action fields into embodiment output fields."""

    __registry_name__ = "mapping"

    def __init__(self, rules: dict[str, MappingRule]) -> None:
        self._rules = rules

    def process(self, payload: dict[str, Any]) -> dict[str, Any]:
        embodiment_action: dict[str, Any] = {}
        for external_name, rule in self._rules.items():
            if rule.canonical_field in payload:
                embodiment_action[external_name] = payload[rule.canonical_field]
        if "action_chunk" in payload:
            embodiment_action["action_chunk"] = payload["action_chunk"]
        if "meta" in payload:
            embodiment_action["meta"] = payload["meta"]
        return embodiment_action


def build_embodiment_preprocessors(
    profile: EmbodimentProfile,
    policy_profile: PolicyProfile,
) -> list[BaseEmbodimentPreprocessor]:
    """Build embodiment-side preprocessors."""
    preprocessors: list[BaseEmbodimentPreprocessor] = _build_embodiment_preprocessors_from_specs(profile)
    if any(field.normalized for field in policy_profile.canonical_observation_schema):
        mapping_index = _last_preprocessor_index(preprocessors, MappingPreprocessor)
        if mapping_index is None:
            raise ValueError("normalized observation fields require a mapping preprocessor")
        preprocessors.insert(
            mapping_index + 1,
            create_registered(
                "embodiment_preprocess",
                "normalize",
                profile=profile,
                policy_profile=policy_profile,
            ),
        )
    return preprocessors


def build_embodiment_postprocessors(
    profile: EmbodimentProfile,
    policy_profile: PolicyProfile,
) -> list[BaseEmbodimentPostprocessor]:
    """Build embodiment-side postprocessors."""
    postprocessors: list[BaseEmbodimentPostprocessor] = _build_embodiment_postprocessors_from_specs(profile)
    mapping_index = _first_postprocessor_index(postprocessors, MappingPostprocessor)
    insert_index = mapping_index if mapping_index is not None else len(postprocessors)
    if any(field.normalized for field in policy_profile.canonical_action_schema):
        postprocessors.insert(
            insert_index,
            create_registered(
                "embodiment_postprocess",
                "denormalize",
                profile=profile,
                policy_profile=policy_profile,
            ),
        )
        insert_index += 1
    postprocessors.insert(
        insert_index,
        create_registered("embodiment_postprocess", "limits", profile=profile),
    )
    return postprocessors


def run_embodiment_preprocess(
    payload: dict[str, Any],
    preprocessors: list[BaseEmbodimentPreprocessor],
) -> dict[str, Any]:
    """Run embodiment-side preprocessors over an observation payload."""
    processed = payload
    for preprocessor in preprocessors:
        processed = preprocessor.process(processed)
    return processed


def run_embodiment_postprocess(
    payload: dict[str, Any],
    postprocessors: list[BaseEmbodimentPostprocessor],
) -> dict[str, Any]:
    """Run embodiment-side postprocessors over an action payload."""
    processed = payload
    for postprocessor in postprocessors:
        processed = postprocessor.process(processed)
    return processed


def input_mapping_rules(profile: EmbodimentProfile) -> dict[str, MappingRule]:
    """Return input mapping rules declared in embodiment preprocessors."""
    return _mapping_rules_from_specs(profile.preprocess)


def output_mapping_rules(profile: EmbodimentProfile) -> dict[str, MappingRule]:
    """Return output mapping rules declared in embodiment postprocessors."""
    return _mapping_rules_from_specs(profile.postprocess)


def _build_embodiment_preprocessors_from_specs(
    profile: EmbodimentProfile,
) -> list[BaseEmbodimentPreprocessor]:
    preprocessors: list[BaseEmbodimentPreprocessor] = []
    for spec in profile.preprocess:
        params = _processor_params_from_spec(spec)
        preprocessors.append(
            create_registered("embodiment_preprocess", spec.name, **params),
        )
    return preprocessors


def _build_embodiment_postprocessors_from_specs(
    profile: EmbodimentProfile,
) -> list[BaseEmbodimentPostprocessor]:
    postprocessors: list[BaseEmbodimentPostprocessor] = []
    for spec in profile.postprocess:
        params = _processor_params_from_spec(spec)
        postprocessors.append(
            create_registered("embodiment_postprocess", spec.name, **params),
        )
    return postprocessors


def _mapping_rules_from_specs(specs: list[ProcessorSpec]) -> dict[str, MappingRule]:
    rules: dict[str, MappingRule] = {}
    for spec in specs:
        if spec.name != "mapping":
            continue
        rules.update(_mapping_rules_from_spec(spec))
    return rules


def _mapping_rules_from_spec(spec: ProcessorSpec) -> dict[str, MappingRule]:
    raw_rules = spec.params.get("rules", [])
    mapping_rules: dict[str, MappingRule] = {}
    for rule_data in raw_rules:
        rule = MappingProcessorRule.model_validate(rule_data)
        mapping_rules[rule.external_name] = MappingRule(
            canonical_field=rule.canonical_field,
            source_field=rule.source_field,
            transform=rule.transform,
            unit=rule.unit,
            frame=rule.frame,
            metadata=rule.metadata,
        )
    return mapping_rules


def _processor_params_from_spec(spec: ProcessorSpec) -> dict[str, Any]:
    if spec.name == "mapping":
        return {"rules": _mapping_rules_from_spec(spec)}
    return dict(spec.params)


def _last_preprocessor_index(
    preprocessors: list[BaseEmbodimentPreprocessor],
    expected_type: type[BaseEmbodimentPreprocessor],
) -> int | None:
    indexes = [index for index, processor in enumerate(preprocessors) if isinstance(processor, expected_type)]
    return indexes[-1] if indexes else None


def _first_postprocessor_index(
    postprocessors: list[BaseEmbodimentPostprocessor],
    expected_type: type[BaseEmbodimentPostprocessor],
) -> int | None:
    for index, processor in enumerate(postprocessors):
        if isinstance(processor, expected_type):
            return index
    return None


def _lookup_path(payload: dict[str, Any], field_path: str) -> Any:
    current: Any = payload
    for segment in field_path.split("."):
        if not isinstance(current, dict) or segment not in current:
            return None
        current = current[segment]
    return current


def _path_exists(payload: dict[str, Any], field_path: str) -> bool:
    current: Any = payload
    for segment in field_path.split("."):
        if not isinstance(current, dict) or segment not in current:
            return False
        current = current[segment]
    return True


def _clip_field(profile: EmbodimentProfile, field_name: str, field_value: dict[str, Any]) -> dict[str, Any]:
    limit = profile.limits.get(field_name)
    if limit is None:
        return field_value
    clipped = deepcopy(field_value)
    lower = limit.bounds.lower
    upper = limit.bounds.upper
    if "values" in clipped:
        clipped["values"] = [_clip_value(value, lower, upper) for value in clipped["values"]]
    elif "value" in clipped:
        clipped["value"] = _clip_value(clipped["value"], lower, upper)
    return clipped


def _scale_field(
    profile: EmbodimentProfile,
    field_name: str,
    field_value: dict[str, Any],
    *,
    normalize: bool,
) -> dict[str, Any]:
    limit = profile.limits.get(field_name)
    if limit is None or limit.bounds.lower is None or limit.bounds.upper is None:
        raise ValueError(f"field {field_name} requires finite limits for normalization")
    scaled = deepcopy(field_value)
    lower = limit.bounds.lower
    upper = limit.bounds.upper
    if "values" in scaled:
        scaled["values"] = [
            _normalize_value(value, lower, upper) if normalize else _denormalize_value(value, lower, upper)
            for value in scaled["values"]
        ]
    elif "value" in scaled:
        scaled["value"] = (
            _normalize_value(scaled["value"], lower, upper)
            if normalize
            else _denormalize_value(scaled["value"], lower, upper)
        )
    return scaled


def _clip_value(value: float, lower: float | None, upper: float | None) -> float:
    if lower is not None and value < lower:
        return lower
    if upper is not None and value > upper:
        return upper
    return value


def _normalize_value(value: float, lower: float, upper: float) -> float:
    if lower >= upper:
        raise ValueError("normalization bounds must satisfy lower < upper")
    return ((value - lower) / (upper - lower)) * 2.0 - 1.0


def _denormalize_value(value: float, lower: float, upper: float) -> float:
    if lower >= upper:
        raise ValueError("normalization bounds must satisfy lower < upper")
    denormalized = ((value + 1.0) / 2.0) * (upper - lower) + lower
    return _clip_value(denormalized, lower, upper)
