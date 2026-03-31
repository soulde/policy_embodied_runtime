"""No-op postprocessor implementation."""

from __future__ import annotations

from typing import Any

from policy_embodied_runtime.server.postprocess.base import BasePostprocessor


class NoOpPostprocessor(BasePostprocessor):
    """Pass canonical actions through unchanged."""

    __registry_name__ = "noop"

    def process(self, canonical_action: dict[str, Any]) -> dict[str, Any]:
        """Return the canonical action unchanged."""
        return canonical_action
