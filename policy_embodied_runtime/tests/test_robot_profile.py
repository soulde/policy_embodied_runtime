from pathlib import Path

from policy_embodied_runtime.profiles.loader import load_policy_profile, load_robot_profile
from policy_embodied_runtime.robot.factory import build_robot
from policy_embodied_runtime.robot.profile import RobotProfile
from policy_embodied_runtime.robot.devices.st3215 import St3215ServoActuator
from policy_embodied_runtime.robot.devices.st3215 import St3215ServoSensor


def test_robot_profile_loads_json_with_unique_robot_data_names() -> None:
    profile = RobotProfile.from_mapping(
        {
            "sensors": [
                {
                    "name": "servo3_position",
                    "type": "st3215_servo_sensor",
                    "path": "/tmp/tty-test",
                    "baud_rate": 1000000,
                    "servo_id": 3,
                    "device_id": 1,
                }
            ],
            "actuators": [
                {
                    "name": "servo3_target",
                    "type": "st3215_servo_actuator",
                    "path": "/tmp/tty-test",
                    "baud_rate": 1000000,
                    "servo_id": 3,
                    "device_id": 1,
                    "speed_units": 1000,
                }
            ],
        }
    )

    assert profile.sensors[0].name == "servo3_position"
    assert profile.sensors[0].device_type == "st3215_servo_sensor"
    assert profile.actuators[0].args["speed_units"] == "1000"


def test_robot_factory_builds_st3215_sensor_and_actuator() -> None:
    profile = RobotProfile.from_mapping(
        {
            "sensors": [
                {
                    "name": "servo3_position",
                    "type": "st3215_servo_sensor",
                    "path": "/tmp/tty-test",
                    "baud_rate": 1000000,
                    "servo_id": 3,
                    "device_id": 1,
                }
            ],
            "actuators": [
                {
                    "name": "servo3_target",
                    "type": "st3215_servo_actuator",
                    "path": "/tmp/tty-test",
                    "baud_rate": 1000000,
                    "servo_id": 3,
                    "device_id": 1,
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
    assert bound_data_names == robot_data_names
    assert policy_profile.model_adapter == "dummy"
    assert [binding.robot_data for binding in policy_profile.inputs] == [
        "shoulder_pan_position",
        "shoulder_lift_position",
        "elbow_position",
        "wrist_pitch_position",
        "wrist_roll_position",
        "gripper_position",
    ]
    assert [binding.robot_data for binding in policy_profile.outputs] == [
        "shoulder_pan_target",
        "shoulder_lift_target",
        "elbow_target",
        "wrist_pitch_target",
        "wrist_roll_target",
        "gripper_target",
    ]
