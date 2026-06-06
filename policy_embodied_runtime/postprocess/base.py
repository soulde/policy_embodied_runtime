"""Base postprocess interface."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from policy_embodied_runtime.robot.registry import AutoRegisteringMeta


class BasePostprocessor(ABC, metaclass=AutoRegisteringMeta):
    """Base interface for canonical action postprocessors."""

    __registry_category__ = "policy_postprocess"

    @abstractmethod
    def process(self, canonical_action: dict[str, Any]) -> dict[str, Any]:
        """Process a canonical action and return the transformed payload."""
