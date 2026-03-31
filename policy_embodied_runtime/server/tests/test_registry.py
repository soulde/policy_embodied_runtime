import pytest

import policy_embodied_runtime.server.adapters  # noqa: F401
import policy_embodied_runtime.server.models  # noqa: F401

from policy_embodied_runtime.server.core.auto_registry import AutoRegisteringMeta, create_registered, get_registry_entries
from policy_embodied_runtime.server.models.base import BaseModelAdapter
from policy_embodied_runtime.server.preprocess.base import BasePreprocessor
from policy_embodied_runtime.server.schemas.policy_profile import PolicyProfile


def test_model_adapter_auto_registry_builds_dummy() -> None:
    adapter = create_registered("model_adapter", "dummy", policy_profile=_dummy_policy_profile())
    assert isinstance(adapter, BaseModelAdapter)
    assert adapter.capabilities()["backend"] == "dummy"


def test_auto_registry_rejects_unknown_adapter() -> None:
    with pytest.raises(KeyError):
        create_registered("embodiment_adapter", "missing")


def test_auto_registry_exposes_registered_names() -> None:
    entries = get_registry_entries("embodiment_adapter")
    assert "dummy" in entries
    assert "franka_like" in entries


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
            "model_adapter": "dummy",
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
