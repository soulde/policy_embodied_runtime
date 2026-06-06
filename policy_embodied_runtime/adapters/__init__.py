"""Embodiment adapter implementations and registries."""

from policy_embodied_runtime.adapters.base import BaseEmbodimentAdapter
from policy_embodied_runtime.adapters.dummy import DummyEmbodimentAdapter
from policy_embodied_runtime.adapters.franka_like import FrankaLikeEmbodimentAdapter

__all__ = [
    "BaseEmbodimentAdapter",
    "DummyEmbodimentAdapter",
    "FrankaLikeEmbodimentAdapter",
]
