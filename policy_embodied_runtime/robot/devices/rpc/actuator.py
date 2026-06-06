"""RPC action actuator."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from policy_embodied_runtime.robot.data import RobotData


@dataclass(slots=True)
class RpcActionActuator:
    """Actuator model for one policy RPC action response.

    A network response is output from the robot, so it is modeled as an
    actuator.
    """

    actuator_name: str = "policy_rpc_action"
    field_name: str = "rpc_action"
    response: dict[str, Any] = field(default_factory=dict)

    def name(self) -> str:
        """Return the actuator name."""
        return self.actuator_name

    def write(self, data: RobotData) -> None:
        """Capture action commands for the RPC response."""
        action = data.action.as_dict() if data.action is not None else data.commands.as_dict()
        self.response = dict(action)
