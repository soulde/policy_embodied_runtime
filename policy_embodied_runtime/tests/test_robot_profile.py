from pathlib import Path

from policy_embodied_runtime.profiles.loader import load_policy_profile, load_robot_profile
from policy_embodied_runtime.models.dummy import DummyPolicy
from policy_embodied_runtime.robot import RobotData, ServoFeedbackData
from policy_embodied_runtime.robot.devices.rpc import RpcFeedbackActuator, RpcObservationSensor
from policy_embodied_runtime.robot.factory import build_robot
from policy_embodied_runtime.robot.profile import RobotProfile
from policy_embodied_runtime.robot.devices.st3215 import St3215ServoActuator
from policy_embodied_runtime.robot.devices.st3215 import St3215ServoSensor
from policy_embodied_runtime.robot.types import SessionContext


def test_robot_profile_loads_json_with_unique_robot_data_names() -> None:
    profile = RobotProfile.from_mapping(
        {
            "sensors": [
                {
                    "name": "servo3_position",
                    "device": {"type": "st3215", "path": "/tmp/tty-test"},
                    "args": {
                        "baud_rate": 1000000,
                        "servo_id": 3,
                        "device_id": 1,
                    },
                }
            ],
            "actuators": [
                {
                    "name": "servo3_target",
                    "device": {"type": "st3215", "path": "/tmp/tty-test"},
                    "args": {
                        "baud_rate": 1000000,
                        "servo_id": 3,
                        "device_id": 1,
                        "speed_units": 1000,
                    },
                }
            ],
        }
    )

    assert profile.sensors[0].name == "servo3_position"
    assert profile.sensors[0].device.type == "st3215"
    assert profile.sensors[0].device.path == "/tmp/tty-test"
    assert profile.actuators[0].args["speed_units"] == "1000"


def test_robot_factory_builds_st3215_sensor_and_actuator() -> None:
    profile = RobotProfile.from_mapping(
        {
            "sensors": [
                {
                    "name": "servo3_position",
                    "device": {"type": "st3215", "path": "/tmp/tty-test"},
                    "args": {
                        "baud_rate": 1000000,
                        "servo_id": 3,
                        "device_id": 1,
                    },
                }
            ],
            "actuators": [
                {
                    "name": "servo3_target",
                    "device": {"type": "st3215", "path": "/tmp/tty-test"},
                    "args": {
                        "baud_rate": 1000000,
                        "servo_id": 3,
                        "device_id": 1,
                    },
                }
            ],
        }
    )

    robot = build_robot(profile)

    assert isinstance(robot.sensors[0], St3215ServoSensor)
    assert isinstance(robot.actuators[0], St3215ServoActuator)
    assert robot.sensors[0].config.servo_id == 3
    assert robot.actuators[0].config.device_id == 1
    assert robot.sensors[0].config.feedback_field == "servo3_position"
    assert robot.actuators[0].config.command_field == "servo3_target"
    assert robot.sensors[0].endpoint.transport is robot.actuators[0].endpoint.transport


def test_soarm101_sim_profiles_bind_policy_to_unique_robot_data() -> None:
    robot_profile = load_robot_profile(
        "policy_embodied_runtime/examples/robot_profiles/soarm101_sim_robot_profile.json"
    )
    policy_profile = load_policy_profile(
        Path("policy_embodied_runtime/examples/policy_profiles/soarm101_sim_policy_profile.json")
    )

    robot_data_names = {
        device.name
        for device in [*robot_profile.sensors, *robot_profile.actuators]
    }
    bound_data_names = {
        binding.robot_data
        for binding in [*policy_profile.inputs, *policy_profile.outputs]
    }

    assert len(robot_data_names) == len(robot_profile.sensors) + len(robot_profile.actuators)
    assert bound_data_names.issubset(robot_data_names)
    assert "arm_feedback" in robot_data_names
    assert policy_profile.policy == "dummy"
    assert len(policy_profile.inputs) == len(policy_profile.outputs)
    assert [binding.robot_data for binding in policy_profile.inputs] == [
        "shoulder_pan_command",
        "shoulder_lift_command",
        "elbow_command",
        "wrist_pitch_command",
        "wrist_roll_command",
        "gripper_command",
    ]
    assert [binding.robot_data for binding in policy_profile.outputs] == [
        "shoulder_pan_target",
        "shoulder_lift_target",
        "elbow_target",
        "wrist_pitch_target",
        "wrist_roll_target",
        "gripper_target",
    ]


def test_soarm101_sim_profile_flow_passes_rpc_commands_to_servo_targets() -> None:
    robot_profile = load_robot_profile(
        "policy_embodied_runtime/examples/robot_profiles/soarm101_sim_robot_profile.json"
    )
    policy_profile = load_policy_profile(
        Path("policy_embodied_runtime/examples/policy_profiles/soarm101_sim_policy_profile.json")
    )
    robot = build_robot(robot_profile)

    assert any(isinstance(sensor, RpcObservationSensor) for sensor in robot.sensors)
    assert any(isinstance(actuator, RpcFeedbackActuator) for actuator in robot.actuators)

    command = {"servo_id": 1, "target_position_rad": 0.25, "enabled": True}
    observation = {
        binding.canonical_field: {**command, "servo_id": index}
        for index, binding in enumerate(policy_profile.inputs, start=1)
    }
    policy = DummyPolicy(policy_profile)
    action = policy.infer(observation, SessionContext("soarm101-flow"))

    assert list(action) == [binding.canonical_field for binding in policy_profile.outputs]
    assert action["shoulder_pan_target"] == {"servo_id": 1, "target_position_rad": 0.25, "enabled": True}
    assert action["gripper_target"] == {"servo_id": 6, "target_position_rad": 0.25, "enabled": True}

    feedback = next(actuator for actuator in robot.actuators if isinstance(actuator, RpcFeedbackActuator))
    data = RobotData()
    data.sensors.update(
        "shoulder_pan_position",
        ServoFeedbackData(servo_id=1, position_rad=0.25, raw_position=128),
    )
    feedback.write(data)

    assert feedback.response["sensors"]["shoulder_pan_position"]["servo_id"] == 1
    assert feedback.response["observation"] == {}
    assert feedback.response["commands"] == {}
