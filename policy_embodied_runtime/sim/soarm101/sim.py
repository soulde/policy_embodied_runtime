from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from .assets import DEFAULT_SCENE


SERVO_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


@dataclass(frozen=True)
class ServoMap:
    device_id: int
    name: str
    actuator_id: int
    joint_id: int
    qpos_addr: int


class SoArm101Sim:
    def __init__(self, scene_path: Path = DEFAULT_SCENE) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.data = mujoco.MjData(self.model)
        self.servos = self._build_servo_map()

        for servo in self.servos.values():
            self.data.ctrl[servo.actuator_id] = self.data.qpos[servo.qpos_addr]

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)

    def set_goal_units(self, device_id: int, position_units: int) -> None:
        servo = self.servos[device_id]
        radians = units_to_radians(position_units, self.ctrlrange(servo))
        self.data.ctrl[servo.actuator_id] = radians

    def get_position_units(self, device_id: int) -> int:
        servo = self.servos[device_id]
        radians = float(self.data.qpos[servo.qpos_addr])
        return radians_to_units(radians, self.ctrlrange(servo))

    def ctrlrange(self, servo: ServoMap) -> tuple[float, float]:
        low, high = self.model.actuator_ctrlrange[servo.actuator_id]
        return float(low), float(high)

    def _build_servo_map(self) -> dict[int, ServoMap]:
        servos = {}
        for index, name in enumerate(SERVO_NAMES, start=1):
            actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if actuator_id < 0 or joint_id < 0:
                raise ValueError(f"SO101 MJCF is missing actuator or joint '{name}'")
            qpos_addr = int(self.model.jnt_qposadr[joint_id])
            servos[index] = ServoMap(index, name, actuator_id, joint_id, qpos_addr)
        return servos


def units_to_radians(position_units: int, ctrlrange: tuple[float, float]) -> float:
    low, high = ctrlrange
    normalized = np.clip(position_units, 0, 4095) / 4095.0
    return float(low + normalized * (high - low))


def radians_to_units(radians: float, ctrlrange: tuple[float, float]) -> int:
    low, high = ctrlrange
    clipped = float(np.clip(radians, low, high))
    normalized = (clipped - low) / (high - low)
    return int(round(normalized * 4095))
