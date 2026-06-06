"""Thin policy base with shared reset and inference flow."""

from __future__ import annotations

from abc import abstractmethod
from typing import Any

from policy_embodied_runtime.robot.types import SessionContext
from policy_embodied_runtime.models.policy import BaseInferencePolicy
from policy_embodied_runtime.protocol.policy_profile import PolicyProfile


class BasePipelinePolicy(BaseInferencePolicy):
    """Thin policy base that leaves only core inference to subclasses."""

    def __init__(self, policy_profile: PolicyProfile) -> None:
        self._policy_profile = policy_profile
        self._config = self.build_config(policy_profile)

    def reset(self, session_id: str) -> None:
        """Reset per-session state."""
        _ = session_id

    def infer(self, canonical_obs: dict[str, Any], session_ctx: SessionContext) -> dict[str, Any]:
        """Run policy-specific core inference."""
        return self.run_inference(canonical_obs, session_ctx)

    def build_config(self, policy_profile: PolicyProfile) -> dict[str, Any]:
        """Build policy-specific runtime config."""
        _ = policy_profile
        return {}

    @abstractmethod
    def run_inference(self, canonical_obs: dict[str, Any], session_ctx: SessionContext) -> dict[str, Any]:
        """Run the core model inference step."""
