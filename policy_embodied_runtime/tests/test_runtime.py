from pathlib import Path

from policy_embodied_runtime.profiles.loader import load_policy_profile
from policy_embodied_runtime.robot.factory import BuiltRobot
from policy_embodied_runtime.robot.profile import DeviceLink
from policy_embodied_runtime.robot.runtime import Runtime
from policy_embodied_runtime.robot.sensor import StaticSensor
from policy_embodied_runtime.protocol.messages import ObservationRequest


def test_runtime_infer_roundtrip() -> None:
    runtime = Runtime(
        policy_profile=load_policy_profile(Path("policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json")),
    )

    action_response = runtime.infer(
        "session-1",
        ObservationRequest.model_validate(
            {
                "observation": {
                    "joint_position": {
                        "values": [0.0] * 7,
                        "joint_names": [f"joint_{index}" for index in range(1, 8)],
                        "unit": "rad",
                    },
                    "gripper_width": {"value": 0.04, "unit": "m"},
                    "task_text": {"text": "pick the cube"},
                    "meta": {"source": "test"},
                }
            }
        ),
    )
    assert action_response.ok is True
    assert action_response.action.gripper_command is not None
    assert len(action_response.action.joint_position_delta.values) == 7


def test_runtime_health_reports_loaded_policy() -> None:
    runtime = Runtime(
        policy_profile=load_policy_profile(Path("policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json")),
    )
    assert runtime.health().policy_id == "dummy-policy"


def test_runtime_loads_policy_and_robot_profiles() -> None:
    runtime = Runtime.from_profiles(
        policy_profile="policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json",
        robot_profile="policy_embodied_runtime/examples/robot_profiles/default_rpc_robot_profile.json",
    )
    runtime.open()
    try:
        assert runtime.health().policy_id == "dummy-policy"
        assert len(runtime.robot_loop._sensor_threads) == 0
    finally:
        runtime.close()


def test_runtime_starts_one_sensor_worker_per_physical_device() -> None:
    device = DeviceLink(type="static", path="local")
    robot = BuiltRobot(
        sensors=[
            StaticSensor("left_button", "left_button", True),
            StaticSensor("right_button", "right_button", False),
        ],
        actuators=[],
        sensor_devices={"left_button": device, "right_button": device},
        actuator_devices={},
    )
    runtime = Runtime(
        policy_profile=load_policy_profile(Path("policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json")),
        robot=robot,
    )

    runtime.open()
    try:
        assert len(runtime.robot_loop._sensor_threads) == 1
    finally:
        runtime.close()
