from __future__ import annotations

import socket
import struct

from .damiao import DISABLE, ENABLE, ZERO_POSITION, Limits, decode_mit, encode_feedback
from .sim import OpenArmSim

CAN_FRAME = struct.Struct("=IB3x8s")
CAN_SFF_MASK = 0x7FF


class DamiaoSocketCanBus:
    def __init__(self, interface: str, sim: OpenArmSim, limits: Limits = Limits()) -> None:
        self.sim = sim
        self.limits = limits
        self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.socket.setblocking(False)
        self.socket.bind((interface,))

    def close(self) -> None:
        self.socket.close()

    def poll(self) -> int:
        processed = 0
        while True:
            try:
                raw = self.socket.recv(CAN_FRAME.size)
            except BlockingIOError:
                return processed
            can_id, length, payload = CAN_FRAME.unpack(raw)
            motor_id = can_id & CAN_SFF_MASK
            if motor_id not in self.sim.motors or length != 8:
                continue
            payload = payload[:length]
            if payload == ENABLE:
                self.sim.set_enabled(motor_id, True)
            elif payload == DISABLE:
                self.sim.set_enabled(motor_id, False)
            elif payload == ZERO_POSITION:
                self.sim.zero_position(motor_id)
            elif self.sim.enabled[motor_id]:
                self.sim.stage_mit(motor_id, decode_mit(payload, self.limits))
            self._publish_feedback(motor_id)
            processed += 1

    def _publish_feedback(self, motor_id: int) -> None:
        position, velocity, torque = self.sim.state(motor_id)
        payload = encode_feedback(
            motor_id, position, velocity, torque, self.limits,
            error_code=0 if self.sim.enabled[motor_id] else 1,
        )
        self.socket.send(CAN_FRAME.pack(motor_id, len(payload), payload))
