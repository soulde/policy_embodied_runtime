"""Postprocess pipeline."""

from __future__ import annotations

from policy_embodied_runtime.robot.registry import create_registered
from policy_embodied_runtime.postprocess.base import BasePostprocessor
from policy_embodied_runtime.postprocess.noop import NoOpPostprocessor
from policy_embodied_runtime.protocol.common import ProcessorSpec


def build_postprocessors(
    config: list[ProcessorSpec],
) -> list[BasePostprocessor]:
    """Build postprocessors from configuration."""
    postprocessors: list[BasePostprocessor] = []
    for processor in config:
        postprocessors.append(create_registered("policy_postprocess", processor.name, **processor.params))
    return postprocessors or [NoOpPostprocessor()]


def run_postprocessors(
    canonical_action: dict[str, object],
    postprocessors: list[BasePostprocessor],
) -> dict[str, object]:
    """Run a prebuilt postprocessor list over a canonical action."""
    processed = canonical_action
    for postprocessor in postprocessors:
        processed = postprocessor.process(processed)
    return processed
