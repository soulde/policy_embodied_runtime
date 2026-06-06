"""ST3215 servo sensor."""

from __future__ import annotations

from policy_embodied_runtime.protocol import ProtocolTransport, St3215Protocol
from policy_embodied_runtime.robot.data import RobotData
from policy_embodied_runtime.robot.devices.st3215.common import St3215ServoConfig, servo_feedback_from_status
from policy_embodied_runtime.transport import Transport


class St3215ServoSensor:
    """Sensor that reads servo present position over a transport."""

    def __init__(
        self,
        sensor_name: str,
        config: St3215ServoConfig,
        transport: Transport,
    ) -> None:
        self._name = sensor_name
        self.config = config
        self.endpoint = ProtocolTransport(St3215Protocol(), transport)

    def name(self) -> str:
        """Return the sensor name."""
        return self._name

    def read(self, data: RobotData) -> None:
        """Read present position feedback into robot data."""
        self.endpoint.send_command(
            St3215Protocol.read_present_position_command(self.config.device_id)
        )
        if not self.config.feedback_field:
            raise ValueError(f"ST3215 sensor '{self._name}' is missing robot data field")
        for _ in range(12):
            reading = self.endpoint.receive_reading()
            if reading is None:
                return
            status = St3215Protocol.parse_status(reading)
            if status.device_id != self.config.device_id or len(status.parameters) < 2:
                continue
            data.sensors.update(self.config.feedback_field, servo_feedback_from_status(self.config, status))
            return
