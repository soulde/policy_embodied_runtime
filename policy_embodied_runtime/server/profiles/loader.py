"""Unified loaders for policy and embodiment profiles."""

from __future__ import annotations

import json
from pathlib import Path

from policy_embodied_runtime.server.schemas.embodiment_profile import EmbodimentProfile
from policy_embodied_runtime.server.schemas.policy_profile import PolicyProfile

__all__ = ["load_policy_profile", "load_embodiment_profile"]


def load_policy_profile(path: str | Path) -> PolicyProfile:
    """Load and validate a policy profile from JSON."""
    data = json.loads(Path(path).read_text())
    return PolicyProfile.model_validate(data)


def load_embodiment_profile(path: str | Path) -> EmbodimentProfile:
    """Load and validate an embodiment profile from JSON."""
    data = json.loads(Path(path).read_text())
    return EmbodimentProfile.model_validate(data)
