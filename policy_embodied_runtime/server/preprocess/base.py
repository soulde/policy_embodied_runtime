"""Base preprocess interface."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from policy_embodied_runtime.server.core.auto_registry import AutoRegisteringMeta


class BasePreprocessor(ABC, metaclass=AutoRegisteringMeta):
    """Base interface for canonical observation preprocessors."""

    __registry_category__ = "policy_preprocess"

    @abstractmethod
    def process(self, canonical_obs: dict[str, Any]) -> dict[str, Any]:
        """Process a canonical observation and return the transformed payload."""
