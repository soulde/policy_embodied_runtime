import pytest

import policy_embodied_runtime.models  # noqa: F401

from policy_embodied_runtime.robot.registry import AutoRegisteringMeta, create_registered, get_registry_entries
from policy_embodied_runtime.models.policy import BaseInferencePolicy
from policy_embodied_runtime.preprocess.base import BasePreprocessor
from policy_embodied_runtime.protocol.policy_profile import PolicyProfile


def test_policy_auto_registry_builds_dummy() -> None:
    policy = create_registered("policy", "dummy", policy_profile=_dummy_policy_profile())
    assert isinstance(policy, BaseInferencePolicy)
    assert policy.capabilities()["backend"] == "dummy"


def test_auto_registry_rejects_unknown_policy() -> None:
    with pytest.raises(KeyError):
        create_registered("policy", "missing", policy_profile=_dummy_policy_profile())


def test_auto_registry_exposes_registered_names() -> None:
    entries = get_registry_entries("policy")
    assert "dummy" in entries
    assert "pi0_like" in entries


def test_auto_registry_defaults_to_snake_case_class_name() -> None:
    class ExampleNormalizer(BasePreprocessor, metaclass=AutoRegisteringMeta):
        def process(self, canonical_obs):
            return canonical_obs

    entries = get_registry_entries("policy_preprocess")
    assert "example_normalizer" in entries


def _dummy_policy_profile() -> PolicyProfile:
    return PolicyProfile.model_validate(
        {
            "id": "dummy-policy",
            "version": "0.1.0",
            "policy": "dummy",
            "canonical_observation_schema": [
                {
                    "name": "joint_position",
                    "semantic_type": "robot_joint_position",
                    "kind": "vector",
                    "ordering": ["joint_1"],
                }
            ],
            "canonical_action_schema": [
                {
                    "name": "joint_position_delta",
                    "semantic_type": "robot_joint_delta",
                    "kind": "vector",
                    "ordering": ["joint_1"],
                }
            ],
        }
    )
