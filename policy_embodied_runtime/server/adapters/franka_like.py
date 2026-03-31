"""Franka-like embodiment adapter example."""

from __future__ import annotations

from typing import Any

from policy_embodied_runtime.server.adapters.dummy import DummyEmbodimentAdapter
from policy_embodied_runtime.server.adapters.pipeline import input_mapping_rules


class FrankaLikeEmbodimentAdapter(DummyEmbodimentAdapter):
    """Field-mapping adapter for Franka-like semantic profiles."""

    __registry_name__ = "franka_like"

    def validate_input(self, obs_msg: dict[str, Any]) -> None:
        super().validate_input(obs_msg)
        if self._profile is None:
            raise RuntimeError("adapter profile is not configured")
        joint_rule = input_mapping_rules(self._profile).get("joint_position")
        joint_source = self._mapping_source("joint_position", joint_rule) if joint_rule is not None else "joint_position"
        if not self._path_exists(obs_msg, joint_source):
            raise ValueError(f"{joint_source} is required")
        joint_state = self._lookup_path(obs_msg, joint_source)
        if "joint_names" not in joint_state or "values" not in joint_state:
            raise ValueError(f"{joint_source} must include joint_names and values")
