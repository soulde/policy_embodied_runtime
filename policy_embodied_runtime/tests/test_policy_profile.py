from pathlib import Path

import pytest
from pydantic import ValidationError

from policy_embodied_runtime.profiles.loader import load_policy_profile
from policy_embodied_runtime.protocol.policy_profile import PolicyProfile


def test_load_valid_policy_profile() -> None:
    profile = load_policy_profile(Path("policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json"))
    assert profile.id == "dummy-policy"
    assert profile.temporal.mode == "single_step"
    assert [field.name for field in profile.canonical_observation_schema] == [
        "joint_position",
        "gripper_width",
        "task_text",
    ]


def test_load_valid_pi0_like_policy_profile() -> None:
    profile = load_policy_profile(Path("policy_embodied_runtime/examples/policy_profiles/pi0_like_policy_profile.json"))
    assert profile.id == "pi0-like-policy"
    assert profile.temporal.mode == "chunked"


def test_policy_profile_rejects_duplicate_schema_names() -> None:
    with pytest.raises(ValidationError):
        PolicyProfile.model_validate(
            {
                "id": "bad",
                "version": "0.1.0",
                "model_adapter": "dummy",
                "canonical_observation_schema": [
                    {
                        "name": "joint_position",
                        "semantic_type": "robot_joint_position",
                        "kind": "vector",
                        "ordering": ["joint_1"],
                    },
                    {
                        "name": "joint_position",
                        "semantic_type": "robot_joint_position",
                        "kind": "vector",
                        "ordering": ["joint_1"],
                    },
                ],
                "canonical_action_schema": [
                    {
                        "name": "gripper_command",
                        "semantic_type": "gripper_command",
                    }
                ],
            }
        )
