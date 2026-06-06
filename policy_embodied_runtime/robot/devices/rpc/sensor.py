"""RPC observation sensor."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from policy_embodied_runtime.robot.data import RobotData


@dataclass(slots=True)
class RpcObservationSensor:
    """Sensor model for one policy RPC observation payload.

    A network request is input to the robot, so it is modeled as a sensor. The
    sensor writes the request observation into ``RobotData.sensors`` so the
    runtime can treat it like any other input source.
    """

    observation: dict[str, Any]
    sensor_name: str = "policy_rpc_observation"
    field_name: str = "rpc_observation"

    def name(self) -> str:
        """Return the sensor name."""
        return self.sensor_name

    def read(self, data: RobotData) -> None:
        """Write the RPC observation into robot data."""
        data.sensors.update(self.field_name, dict(self.observation))
        data.observation = data.observation.__class__(dict(self.observation))
