"""Base model adapter interface."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from policy_embodied_runtime.robot.registry import AutoRegisteringMeta
from policy_embodied_runtime.robot.types import SessionContext


class BaseModelAdapter(ABC, metaclass=AutoRegisteringMeta):
    """Base interface for model adapters."""

    __registry_category__ = "model_adapter"

    @abstractmethod
    def reset(self, session_id: str) -> None:
        """Reset per-session state."""

    @abstractmethod
    def infer(self, canonical_obs: dict[str, Any], session_ctx: SessionContext) -> dict[str, Any]:
        """Run inference over a canonical observation."""

    @abstractmethod
    def capabilities(self) -> dict[str, Any]:
        """Describe supported modalities and runtime traits."""
