"""Preprocess pipeline."""

from __future__ import annotations

from policy_embodied_runtime.server.core.auto_registry import create_registered
from policy_embodied_runtime.server.preprocess.base import BasePreprocessor
from policy_embodied_runtime.server.preprocess.noop import NoOpPreprocessor
from policy_embodied_runtime.server.schemas.common import ProcessorSpec


def build_preprocessors(config: list[ProcessorSpec]) -> list[BasePreprocessor]:
    """Build preprocessors from configuration."""
    preprocessors: list[BasePreprocessor] = []
    for processor in config:
        preprocessors.append(create_registered("policy_preprocess", processor.name, **processor.params))
    return preprocessors or [NoOpPreprocessor()]


def run_preprocessors(
    canonical_obs: dict[str, object],
    preprocessors: list[BasePreprocessor],
) -> dict[str, object]:
    """Run a prebuilt preprocessor list over a canonical observation."""
    processed = canonical_obs
    for preprocessor in preprocessors:
        processed = preprocessor.process(processed)
    return processed
