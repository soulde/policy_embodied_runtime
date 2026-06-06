"""Base embodiment adapter interface."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from policy_embodied_runtime.robot.registry import AutoRegisteringMeta
from policy_embodied_runtime.protocol.embodiment_profile import EmbodimentProfile
from policy_embodied_runtime.protocol.policy_profile import PolicyProfile


class BaseEmbodimentAdapter(ABC, metaclass=AutoRegisteringMeta):
    """Base interface for embodiment mappings."""

    __registry_category__ = "embodiment_adapter"

    @abstractmethod
    def setup(self, profile: EmbodimentProfile, policy_profile: PolicyProfile) -> None:
        """Initialize the adapter with a profile and policy profile."""

    @abstractmethod
    def validate_input(self, obs_msg: dict[str, Any]) -> None:
        """Validate an incoming observation message."""

    @abstractmethod
    def to_canonical_obs(self, obs_msg: dict[str, Any]) -> dict[str, Any]:
        """Convert an incoming observation into canonical semantics."""

    @abstractmethod
    def from_canonical_action(self, action_msg: dict[str, Any]) -> dict[str, Any]:
        """Convert a canonical action into embodiment output semantics."""

    @abstractmethod
    def reset(self, session_id: str) -> None:
        """Reset per-session embodiment state."""

    @abstractmethod
    def schema(self) -> dict[str, Any]:
        """Return adapter schema or capabilities."""
