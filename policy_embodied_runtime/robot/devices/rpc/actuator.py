"""RPC action actuator."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, is_dataclass
from typing import Any

from policy_embodied_runtime.robot.data import RobotData


@dataclass(slots=True)
class RpcActionActuator:
    """Actuator model for one policy RPC action response.

    A network response is output from the robot, so it is modeled as an
    actuator.
    """

    actuator_name: str = "policy_rpc_action"
    response: dict[str, Any] = field(default_factory=dict)

    def name(self) -> str:
        """Return the actuator name."""
        return self.actuator_name

    def write(self, data: RobotData) -> None:
        """Capture action commands for the RPC response."""
        action = data.action.as_dict() if data.action is not None else data.commands.as_dict()
        self.response = dict(action)


@dataclass(slots=True)
class RpcFeedbackActuator:
    """Actuator model that publishes robot feedback over RPC."""

    actuator_name: str = "policy_rpc_feedback"
    response: dict[str, Any] = field(default_factory=dict)

    def name(self) -> str:
        """Return the actuator name."""
        return self.actuator_name

    def write(self, data: RobotData) -> None:
        """Capture all current robot data for RPC publication."""
        self.response = {
            "observation": _plain(data.observation.as_dict()),
            "sensors": _plain(data.sensors.as_dict()),
            "action": _plain(data.action.as_dict()) if data.action is not None else None,
            "commands": _plain(data.commands.as_dict()),
        }


def _plain(value: Any) -> Any:
    if is_dataclass(value):
        return {key: _plain(item) for key, item in asdict(value).items()}
    if isinstance(value, dict):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_plain(item) for item in value]
    if isinstance(value, tuple):
        return [_plain(item) for item in value]
    return value
