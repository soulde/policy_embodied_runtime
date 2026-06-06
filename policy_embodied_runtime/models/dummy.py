"""Dummy policy for pass-through smoke tests."""

from __future__ import annotations

from typing import Any

from policy_embodied_runtime.robot.types import SessionContext
from policy_embodied_runtime.models.pipeline_policy import BasePipelinePolicy


class DummyPolicy(BasePipelinePolicy):
    """Pass policy inputs to outputs by binding order."""

    __registry_name__ = "dummy"

    def run_inference(self, canonical_obs: dict[str, Any], session_ctx: SessionContext) -> dict[str, Any]:
        _ = session_ctx
        if self._policy_profile.inputs and len(self._policy_profile.inputs) != len(self._policy_profile.outputs):
            raise ValueError("dummy policy requires the same number of inputs and outputs")
        action: dict[str, Any] = {}
        if self._policy_profile.inputs:
            for input_binding, output_binding in zip(self._policy_profile.inputs, self._policy_profile.outputs):
                action[output_binding.canonical_field] = canonical_obs.get(input_binding.canonical_field)
            return action
        if "joint_position" in canonical_obs:
            action["joint_position_delta"] = dict(canonical_obs["joint_position"])
        if "gripper_width" in canonical_obs:
            action["gripper_command"] = dict(canonical_obs["gripper_width"])
        return action

    def capabilities(self) -> dict[str, Any]:
        return {
            "backend": "dummy",
            "modalities": ["joint_position", "gripper_width", "task_text"],
            "temporal_modes": ["single_step"],
        }
