from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco

from .damiao import MitCommand

JOINT_NAMES = tuple(f"openarm_joint{index}" for index in range(1, 8))
ACTUATOR_NAMES = tuple(f"joint{index}_ctrl" for index in range(1, 8))
DEFAULT_SCENE = (
    Path(__file__).resolve().parents[3]
    / "third_party"
    / "openarm_mujoco"
    / "v1"
    / "openarm.xml"
)


@dataclass(frozen=True)
class MotorMap:
    motor_id: int
    joint_id: int
    actuator_id: int
    qpos_addr: int
    qvel_addr: int


class OpenArmSim:
    def __init__(self, scene_path: Path = DEFAULT_SCENE) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.data = mujoco.MjData(self.model)
        self.motors = self._build_motor_map()
        self.enabled = {motor_id: False for motor_id in self.motors}
        self.command = {motor_id: MitCommand(0.0, 0.0, 0.0, 0.0, 0.0)
                        for motor_id in self.motors}

    def set_enabled(self, motor_id: int, enabled: bool) -> None:
        self.enabled[motor_id] = enabled
        if not enabled:
            self.data.ctrl[self.motors[motor_id].actuator_id] = 0.0

    def zero_position(self, motor_id: int) -> None:
        motor = self.motors[motor_id]
        self.data.qpos[motor.qpos_addr] = 0.0
        self.data.qvel[motor.qvel_addr] = 0.0

    def stage_mit(self, motor_id: int, command: MitCommand) -> None:
        self.command[motor_id] = command

    def state(self, motor_id: int) -> tuple[float, float, float]:
        motor = self.motors[motor_id]
        return (
            float(self.data.qpos[motor.qpos_addr]),
            float(self.data.qvel[motor.qvel_addr]),
            float(self.data.ctrl[motor.actuator_id]),
        )

    def step(self) -> None:
        for motor_id, motor in self.motors.items():
            if not self.enabled[motor_id]:
                self.data.ctrl[motor.actuator_id] = 0.0
                continue
            command = self.command[motor_id]
            position = float(self.data.qpos[motor.qpos_addr])
            velocity = float(self.data.qvel[motor.qvel_addr])
            torque = (
                command.kp * (command.position - position)
                + command.kd * (command.velocity - velocity)
                + command.torque
            )
            low, high = self.model.actuator_ctrlrange[motor.actuator_id]
            self.data.ctrl[motor.actuator_id] = min(float(high), max(float(low), torque))
        mujoco.mj_step(self.model, self.data)

    def _build_motor_map(self) -> dict[int, MotorMap]:
        motors: dict[int, MotorMap] = {}
        for motor_id, (joint_name, actuator_name) in enumerate(
            zip(JOINT_NAMES, ACTUATOR_NAMES, strict=True), start=1
        ):
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            actuator_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name
            )
            if joint_id < 0 or actuator_id < 0:
                raise ValueError(f"OpenArm model is missing {joint_name}/{actuator_name}")
            motors[motor_id] = MotorMap(
                motor_id,
                joint_id,
                actuator_id,
                int(self.model.jnt_qposadr[joint_id]),
                int(self.model.jnt_dofadr[joint_id]),
            )
        return motors
