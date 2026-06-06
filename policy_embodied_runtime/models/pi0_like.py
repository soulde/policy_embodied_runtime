"""Pi0-like placeholder policy."""

from __future__ import annotations

from policy_embodied_runtime.robot.types import SessionContext
from policy_embodied_runtime.models.pipeline_policy import BasePipelinePolicy
from policy_embodied_runtime.protocol.policy_profile import PolicyProfile


class Pi0LikePolicy(BasePipelinePolicy):
    """Structured placeholder for a future Pi0-style model backend."""

    __registry_name__ = "pi0_like"

    def build_config(self, policy_profile: PolicyProfile) -> dict[str, str]:
        return {
            "checkpoint": str(policy_profile.model.get("checkpoint", "")),
            "device": str(policy_profile.model.get("device", "cpu")),
            "precision": str(policy_profile.model.get("precision", "fp16")),
        }

    def run_inference(self, canonical_obs: dict[str, object], session_ctx: SessionContext) -> dict[str, object]:
        joint_position = canonical_obs.get("joint_position", {})
        joint_values = list(joint_position.get("values", []))
        joint_names = list(joint_position.get("joint_names", []))
        deltas = [0.02 if index % 2 == 0 else -0.02 for index, _ in enumerate(joint_values)]
        chunk = [
            {
                "joint_position_delta": {
                    "values": deltas,
                    "joint_names": joint_names,
                    "unit": joint_position.get("unit", "rad"),
                },
                "gripper_command": {
                    "value": canonical_obs.get("gripper_width", {}).get("value", 0.04),
                    "unit": "m",
                },
                "meta": {"chunk_index": chunk_index, "step_id": session_ctx.step_id},
            }
            for chunk_index in range(4)
        ]
        return {
            "action_chunk": chunk,
            "meta": {"backend": "pi0_like_placeholder", **self._config},
        }

    def capabilities(self) -> dict[str, object]:
        return {
            "backend": "pytorch_placeholder",
            "modalities": ["image", "joint_position", "task_text"],
            "temporal_modes": ["single_step", "chunked"],
        }
