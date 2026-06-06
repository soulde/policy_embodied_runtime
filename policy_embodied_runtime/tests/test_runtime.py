from pathlib import Path

from policy_embodied_runtime.profiles.loader import load_policy_profile
from policy_embodied_runtime.robot.policy_runtime import PolicyRuntime
from policy_embodied_runtime.protocol.messages import ObservationRequest


def test_runtime_infer_roundtrip() -> None:
    runtime = PolicyRuntime(
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
    runtime = PolicyRuntime(
        policy_profile=load_policy_profile(Path("policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json")),
    )
    assert runtime.health().policy_id == "dummy-policy"
