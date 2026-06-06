"""No-op preprocessor implementation."""

from __future__ import annotations

from typing import Any

from policy_embodied_runtime.preprocess.base import BasePreprocessor


class NoOpPreprocessor(BasePreprocessor):
    """Pass canonical observations through unchanged."""

    __registry_name__ = "noop"

    def process(self, canonical_obs: dict[str, Any]) -> dict[str, Any]:
        """Return the canonical observation unchanged."""
        return canonical_obs
