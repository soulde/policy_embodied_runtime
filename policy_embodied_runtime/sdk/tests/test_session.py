import threading
import uuid
from pathlib import Path

from policy_embodied_runtime.sdk.policy_client.client import PolicyClient
from policy_embodied_runtime.sdk.policy_client.types import EndpointConfig
from policy_embodied_runtime.server.apps.zmq_server import ZmqPolicyServer


def _endpoint_name(name: str) -> str:
    return f"{name}-{uuid.uuid4().hex}"


def test_policy_session_roundtrip() -> None:
    endpoints = EndpointConfig(endpoint=_endpoint_name("sdk-session"))
    server = ZmqPolicyServer(
        endpoint=endpoints.endpoint,
        policy_profile=Path("policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json"),
        embodiment_profile=Path("policy_embodied_runtime/examples/embodiment_profiles/dummy_embodiment_profile.json"),
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    client = PolicyClient.connect(endpoints)
    try:
        session = client.session("sdk-session")
        reset_payload = session.reset()
        action_payload = session.step(
            {
                "joint_position": {
                    "values": [0.0] * 7,
                    "joint_names": [f"joint_{index}" for index in range(1, 8)],
                    "unit": "rad",
                },
                "gripper_width": {"value": 0.04, "unit": "m"},
                "task_text": {"text": "close gripper"},
                "meta": {"source": "sdk-test"},
            }
        )
        assert reset_payload["ok"] is True
        assert "action" in action_payload
    finally:
        client.close()
        server.stop()
        thread.join(timeout=1)
