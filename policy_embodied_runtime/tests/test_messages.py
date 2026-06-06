import pytest
from pydantic import ValidationError

from policy_embodied_runtime.protocol.messages import ActionPayload, JointStateValue, MessageEnvelope, ObservationRequest


def test_action_payload_accepts_single_step_action() -> None:
    payload = ActionPayload(
        joint_position_delta=JointStateValue(
            values=[0.1, -0.1],
            joint_names=["joint_1", "joint_2"],
            unit="rad",
        )
    )
    assert payload.joint_position_delta is not None


def test_message_envelope_requires_non_empty_request_id() -> None:
    with pytest.raises(ValidationError):
        MessageEnvelope.model_validate(
            {
                "type": "health_request",
                "request_id": "",
                "session_id": "session-1",
                "step_id": 0,
                "timestamp_ns": 1,
                "payload": {},
            }
        )


def test_observation_request_accepts_adapter_specific_nested_fields() -> None:
    request = ObservationRequest.model_validate(
        {
            "observation": {
                "arm": {
                    "joint_position": {
                        "values": [0.0, 0.1],
                        "joint_names": ["joint_1", "joint_2"],
                        "unit": "rad",
                    }
                },
                "task": {"text": {"text": "pick"}},
            }
        }
    )
    adapter_input = request.observation.as_adapter_input()
    assert adapter_input["arm"]["joint_position"]["unit"] == "rad"
    assert adapter_input["task"]["text"]["text"] == "pick"
