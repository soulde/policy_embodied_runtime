"""Dummy model adapter for MVP loopback."""

from __future__ import annotations

from typing import Any

from policy_embodied_runtime.robot.types import SessionContext
from policy_embodied_runtime.models.pipeline import BasePipelineModelAdapter


class DummyModelAdapter(BasePipelineModelAdapter):
    """Simple deterministic adapter used for smoke tests."""

    __registry_name__ = "dummy"

    def run_inference(self, canonical_obs: dict[str, Any], session_ctx: SessionContext) -> dict[str, Any]:
        joint_position = canonical_obs.get("joint_position", {})
        joint_values = joint_position.get("values", [])
        joint_names = joint_position.get("joint_names", [])
        step_scale = min(session_ctx.step_id + 1, 5) * 0.01
        deltas = [0.0 if value == 0 else -step_scale for value in joint_values]
        gripper_width = canonical_obs.get("gripper_width", {}).get("value", 0.04)

        return {
            "joint_position_delta": {
                "values": deltas,
                "joint_names": joint_names,
                "unit": joint_position.get("unit", "rad"),
            },
            "gripper_command": {
                "value": gripper_width,
                "unit": "m",
            },
            "meta": {
                "policy_id": self._policy_profile.id,
                "step_id": session_ctx.step_id,
            },
        }

    def capabilities(self) -> dict[str, Any]:
        return {
            "backend": "dummy",
            "modalities": ["joint_position", "gripper_width", "task_text"],
            "temporal_modes": ["single_step"],
        }
