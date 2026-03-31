from pathlib import Path

from policy_embodied_runtime.server.profiles.loader import load_embodiment_profile, load_policy_profile
from policy_embodied_runtime.server.core.policy_runtime import PolicyRuntime
from policy_embodied_runtime.server.schemas.messages import ObservationRequest


def test_runtime_infer_roundtrip() -> None:
    runtime = PolicyRuntime(
        policy_profile=load_policy_profile(Path("policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json")),
        embodiment_profile=load_embodiment_profile(Path("policy_embodied_runtime/examples/embodiment_profiles/dummy_embodiment_profile.json")),
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
        embodiment_profile=load_embodiment_profile(Path("policy_embodied_runtime/examples/embodiment_profiles/dummy_embodiment_profile.json")),
    )
    assert runtime.health().policy_id == "dummy-policy"


def test_runtime_accepts_nested_franka_like_observation() -> None:
    runtime = PolicyRuntime(
        policy_profile=load_policy_profile(Path("policy_embodied_runtime/examples/policy_profiles/pi0_like_policy_profile.json")),
        embodiment_profile=load_embodiment_profile(Path("policy_embodied_runtime/examples/embodiment_profiles/franka_like_profile.json")),
    )

    action_response = runtime.infer(
        "session-franka",
        ObservationRequest.model_validate(
            {
                "observation": {
                    "arm": {
                        "joint_position": {
                            "values": [0.0] * 7,
                            "joint_names": [
                                "panda_joint1",
                                "panda_joint2",
                                "panda_joint3",
                                "panda_joint4",
                                "panda_joint5",
                                "panda_joint6",
                                "panda_joint7",
                            ],
                            "unit": "rad",
                        }
                    },
                    "gripper": {"width": {"value": 0.04, "unit": "m"}},
                    "task": {"text": {"text": "move to the cup"}},
                    "camera": {
                        "front_rgb": {
                            "encoding": "uri",
                            "data": "memory://rgb/front",
                            "mime_type": "image/jpeg",
                        }
                    },
                }
            }
        ),
    )
    assert action_response.ok is True
    assert len(action_response.action.action_chunk) == 4
