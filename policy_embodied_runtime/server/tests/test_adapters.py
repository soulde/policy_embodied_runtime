from policy_embodied_runtime.server.adapters.dummy import DummyEmbodimentAdapter
from policy_embodied_runtime.server.schemas.common import ProcessorSpec, SemanticBounds, SemanticField
from policy_embodied_runtime.server.schemas.embodiment_profile import EmbodimentProfile, LimitSpec
from policy_embodied_runtime.server.schemas.policy_profile import PolicyProfile


def test_dummy_adapter_normalizes_observation_fields_from_limits() -> None:
    adapter = DummyEmbodimentAdapter()
    adapter.setup(_normalized_profile(), _normalized_policy_profile())

    canonical = adapter.to_canonical_obs(
        {
            "joint_position": {
                "values": [0.0, 1.0],
                "joint_names": ["joint_1", "joint_2"],
                "unit": "rad",
            }
        }
    )

    assert canonical["joint_position"]["values"] == [0.0, 1.0]


def test_dummy_adapter_denormalizes_and_clips_action_fields_from_limits() -> None:
    adapter = DummyEmbodimentAdapter()
    adapter.setup(_normalized_profile(), _normalized_policy_profile())

    embodiment = adapter.from_canonical_action(
        {
            "joint_position_delta": {
                "values": [2.0, -2.0],
                "joint_names": ["joint_1", "joint_2"],
                "unit": "rad",
            }
        }
    )

    assert embodiment["joint_position_delta"]["values"] == [0.1, -0.1]


def _normalized_policy_profile() -> PolicyProfile:
    return PolicyProfile(
        id="normalized-policy",
        version="0.1.0",
        model_adapter="dummy",
        canonical_observation_schema=[
            SemanticField(
                name="joint_position",
                semantic_type="robot_joint_position",
                kind="vector",
                unit="rad",
                ordering=["joint_1", "joint_2"],
                normalized=True,
            )
        ],
        canonical_action_schema=[
            SemanticField(
                name="joint_position_delta",
                semantic_type="robot_joint_position_delta",
                kind="vector",
                unit="rad",
                ordering=["joint_1", "joint_2"],
                normalized=True,
            )
        ],
        preprocess=[],
        postprocess=[],
        safety={},
    )


def _normalized_profile() -> EmbodimentProfile:
    return EmbodimentProfile(
        id="normalized-embodiment",
        adapter="dummy",
        preprocess=[
            ProcessorSpec(
                name="mapping",
                params={
                    "rules": [
                        {
                            "external_name": "joint_position",
                            "canonical_field": "joint_position",
                            "source_field": "joint_position",
                            "unit": "rad",
                        }
                    ]
                },
            )
        ],
        postprocess=[
            ProcessorSpec(
                name="mapping",
                params={
                    "rules": [
                        {
                            "external_name": "joint_position_delta",
                            "canonical_field": "joint_position_delta",
                            "source_field": "joint_position_delta",
                            "unit": "rad",
                        }
                    ]
                },
            )
        ],
        joint_names=["joint_1", "joint_2"],
        units={"joint_position": "rad", "joint_position_delta": "rad"},
        limits={
            "joint_position": LimitSpec(bounds=SemanticBounds(lower=-1.0, upper=1.0), unit="rad"),
            "joint_position_delta": LimitSpec(bounds=SemanticBounds(lower=-0.1, upper=0.1), unit="rad"),
        },
    )
