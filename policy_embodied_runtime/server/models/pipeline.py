"""Thin model adapter base with shared reset and inference flow."""

from __future__ import annotations

from abc import abstractmethod
from typing import Any

from policy_embodied_runtime.server.core.types import SessionContext
from policy_embodied_runtime.server.models.base import BaseModelAdapter
from policy_embodied_runtime.server.schemas.policy_profile import PolicyProfile


class BasePipelineModelAdapter(BaseModelAdapter):
    """Thin model adapter base that leaves only core inference to subclasses."""

    def __init__(self, policy_profile: PolicyProfile) -> None:
        self._policy_profile = policy_profile
        self._config = self.build_config(policy_profile)

    def reset(self, session_id: str) -> None:
        """Reset per-session state."""
        _ = session_id

    def infer(self, canonical_obs: dict[str, Any], session_ctx: SessionContext) -> dict[str, Any]:
        """Run adapter-specific core inference."""
        return self.run_inference(canonical_obs, session_ctx)

    def build_config(self, policy_profile: PolicyProfile) -> dict[str, Any]:
        """Build adapter-specific runtime config."""
        _ = policy_profile
        return {}

    @abstractmethod
    def run_inference(self, canonical_obs: dict[str, Any], session_ctx: SessionContext) -> dict[str, Any]:
        """Run the core model inference step."""
