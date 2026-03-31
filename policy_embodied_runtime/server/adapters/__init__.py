"""Embodiment adapter implementations and registries."""

from policy_embodied_runtime.server.adapters.base import BaseEmbodimentAdapter
from policy_embodied_runtime.server.adapters.dummy import DummyEmbodimentAdapter
from policy_embodied_runtime.server.adapters.franka_like import FrankaLikeEmbodimentAdapter

__all__ = [
    "BaseEmbodimentAdapter",
    "DummyEmbodimentAdapter",
    "FrankaLikeEmbodimentAdapter",
]
