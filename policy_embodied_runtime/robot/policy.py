"""Robot-layer policy interfaces."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any

from policy_embodied_runtime.robot.types import SessionContext
from policy_embodied_runtime.robot.data import RobotAction, RobotData


class Policy(ABC):
    """Robot policy that reads robot data and writes robot action commands."""

    @abstractmethod
    def reset(self, session_id: str) -> None:
        """Reset per-session policy state."""

    @abstractmethod
    def infer(self, data: RobotData, session_ctx: SessionContext) -> None:
        """Infer the next action and write it into robot data."""

    @abstractmethod
    def capabilities(self) -> dict[str, Any]:
        """Describe supported runtime traits."""


@dataclass(slots=True)
class DummyPolicy(Policy):
    """No-op policy matching rustyRobot's first policy implementation."""

    name: str = "control"

    def reset(self, session_id: str) -> None:
        _ = session_id

    def infer(self, data: RobotData, session_ctx: SessionContext) -> None:
        _ = data
        _ = session_ctx

    def capabilities(self) -> dict[str, Any]:
        return {"backend": "dummy_policy", "name": self.name}


@dataclass(slots=True)
class ModelPolicy(Policy):
    """Policy implementation backed by the project's model adapter registry."""

    name: str
    model_adapter: Any

    def reset(self, session_id: str) -> None:
        self.model_adapter.reset(session_id)

    def infer(self, data: RobotData, session_ctx: SessionContext) -> None:
        action = self.model_adapter.infer(data.observation.as_dict(), session_ctx)
        data.set_action(RobotAction(action))

    def capabilities(self) -> dict[str, Any]:
        capabilities = self.model_adapter.capabilities()
        return {"name": self.name, **capabilities}
