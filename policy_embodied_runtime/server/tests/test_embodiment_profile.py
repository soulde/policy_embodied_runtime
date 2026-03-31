from pathlib import Path

import pytest
from pydantic import ValidationError

from policy_embodied_runtime.server.profiles.loader import load_embodiment_profile
from policy_embodied_runtime.server.schemas.embodiment_profile import EmbodimentProfile


def test_load_valid_embodiment_profile() -> None:
    profile = load_embodiment_profile(Path("policy_embodied_runtime/examples/embodiment_profiles/dummy_embodiment_profile.json"))
    assert profile.id == "dummy-embodiment"
    assert profile.adapter == "dummy"
    assert len(profile.joint_names) == 7


def test_load_valid_franka_like_profile() -> None:
    profile = load_embodiment_profile(Path("policy_embodied_runtime/examples/embodiment_profiles/franka_like_profile.json"))
    assert profile.id == "franka-like"
    assert profile.adapter == "franka_like"
    assert profile.frame_info["base"] == "franka_base"


def test_embodiment_profile_rejects_duplicate_joint_names() -> None:
    with pytest.raises(ValidationError):
        EmbodimentProfile.model_validate(
            {
                "id": "bad-profile",
                "adapter": "dummy",
                "preprocess": [
                    {
                        "name": "mapping",
                        "params": {
                            "rules": [
                                {
                                    "external_name": "joint_position",
                                    "canonical_field": "joint_position",
                                    "source_field": "joint_position",
                                }
                            ]
                        },
                    }
                ],
                "postprocess": [
                    {
                        "name": "mapping",
                        "params": {
                            "rules": [
                                {
                                    "external_name": "joint_position_delta",
                                    "canonical_field": "joint_position_delta",
                                    "source_field": "joint_position_delta",
                                }
                            ]
                        },
                    }
                ],
                "joint_names": ["joint_1", "joint_1"],
                "units": {},
                "limits": {},
            }
        )
