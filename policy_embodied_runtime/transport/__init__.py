"""Transport interfaces and implementations.

This follows the rustyRobot meaning of transport: the low-level communication
implementation such as serial, CAN, USB2CAN, ZMQ, or virtual serial. It does
not decide whether data is input or output; that boundary is expressed by
``robot.Sensor`` and ``robot.Actuator``.
"""

from policy_embodied_runtime.transport.base import Transport, TransportFrame
from policy_embodied_runtime.transport.serial import SerialConfig, SerialTransport

__all__ = [
    "SerialConfig",
    "SerialTransport",
    "Transport",
    "TransportFrame",
]
