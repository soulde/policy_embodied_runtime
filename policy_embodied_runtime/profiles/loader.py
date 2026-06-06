"""Unified loaders for policy and embodiment profiles."""

from __future__ import annotations

import json
from pathlib import Path

from policy_embodied_runtime.protocol.embodiment_profile import EmbodimentProfile
from policy_embodied_runtime.protocol.policy_profile import PolicyProfile
from policy_embodied_runtime.robot.profile import RobotProfile

__all__ = ["load_policy_profile", "load_embodiment_profile", "load_robot_profile"]


def load_policy_profile(path: str | Path) -> PolicyProfile:
    """Load and validate a policy profile from JSON."""
    data = json.loads(Path(path).read_text())
    return PolicyProfile.model_validate(data)


def load_embodiment_profile(path: str | Path) -> EmbodimentProfile:
    """Load and validate an embodiment profile from JSON."""
    data = json.loads(Path(path).read_text())
    return EmbodimentProfile.model_validate(data)


def load_robot_profile(path: str | Path) -> RobotProfile:
    """Load a JSON robot profile."""
    return RobotProfile.from_json_file(path)
