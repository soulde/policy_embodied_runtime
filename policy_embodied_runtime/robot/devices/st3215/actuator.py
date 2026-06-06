"""ST3215 servo actuator."""

from __future__ import annotations

from policy_embodied_runtime.protocol import ProtocolTransport, St3215Protocol
from policy_embodied_runtime.robot.data import RobotData, ServoCommand
from policy_embodied_runtime.robot.devices.st3215.common import St3215ServoConfig, radians_to_position_units
from policy_embodied_runtime.transport import Transport


class St3215ServoActuator:
    """Actuator that writes servo goal position commands over a transport."""

    def __init__(
        self,
        actuator_name: str,
        config: St3215ServoConfig,
        transport: Transport,
    ) -> None:
        self._name = actuator_name
        self.config = config
        self.endpoint = ProtocolTransport(St3215Protocol(), transport)

    def name(self) -> str:
        """Return the actuator name."""
        return self._name

    def write(self, data: RobotData) -> None:
        """Send the current enabled servo command if present."""
        command = servo_goal_command(self.config, data)
        if command is None:
            return
        self.endpoint.send_command(command)
        self.endpoint.receive_reading()


def servo_goal_command(config: St3215ServoConfig, data: RobotData):
    """Build an ST3215 goal command from robot actuator commands."""
    if data.commands.get("emergency_stop", False):
        return None
    command = _configured_servo_command(config, data)
    if command is None or not command.enabled:
        return None
    position_units = radians_to_position_units(
        command.target_position_rad,
        config.max_position_units,
    )
    return St3215Protocol.goal_position_command(
        config.device_id,
        position_units,
        config.time_units,
        config.speed_units,
    )


def _configured_servo_command(config: St3215ServoConfig, data: RobotData) -> ServoCommand | None:
    if not config.command_field:
        raise ValueError(f"ST3215 actuator for servo {config.servo_id} is missing robot data field")
    command = data.commands.get(config.command_field)
    if isinstance(command, ServoCommand):
        return command
    if isinstance(command, dict):
        return ServoCommand(
            servo_id=int(command.get("servo_id", config.servo_id)),
            target_position_rad=float(command["target_position_rad"]),
            enabled=bool(command.get("enabled", True)),
        )
    return None
