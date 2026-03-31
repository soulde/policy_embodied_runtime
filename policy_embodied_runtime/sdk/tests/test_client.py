import threading
import uuid
from pathlib import Path

from policy_embodied_runtime.sdk.policy_client.client import PolicyClient
from policy_embodied_runtime.sdk.policy_client.types import EndpointConfig
from policy_embodied_runtime.server.apps.zmq_server import ZmqPolicyServer


def _endpoint_name(name: str) -> str:
    return f"{name}-{uuid.uuid4().hex}"


def test_client_health_over_ipc_transport() -> None:
    endpoints = EndpointConfig(endpoint=_endpoint_name("sdk-client"))
    server = ZmqPolicyServer(
        endpoint=endpoints.endpoint,
        policy_profile=Path("policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json"),
        embodiment_profile=Path("policy_embodied_runtime/examples/embodiment_profiles/dummy_embodiment_profile.json"),
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    client = PolicyClient.connect(endpoints)
    try:
        health = client.health()
        assert health == {"ok": True, "policy_id": "dummy-policy"}
    finally:
        client.close()
        server.stop()
        thread.join(timeout=1)


def test_client_infer_against_preloaded_server() -> None:
    endpoints = EndpointConfig(endpoint=_endpoint_name("sdk-client-preloaded"))
    server = ZmqPolicyServer(
        endpoint=endpoints.endpoint,
        policy_profile=Path("policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json"),
        embodiment_profile=Path("policy_embodied_runtime/examples/embodiment_profiles/dummy_embodiment_profile.json"),
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    client = PolicyClient.connect(endpoints)
    try:
        response = client.infer(
            {
                "observation": {
                    "joint_position": {
                        "values": [0.0] * 7,
                        "joint_names": [f"joint_{index}" for index in range(1, 8)],
                        "unit": "rad",
                    },
                    "gripper_width": {"value": 0.04, "unit": "m"},
                    "task_text": {"text": "move to object"},
                    "meta": {"source": "preloaded-server-test"},
                }
            },
            session_id="preloaded-session",
        )
        assert response["ok"] is True
        assert "joint_position_delta" in response["action"]
    finally:
        client.close()
        server.stop()
        thread.join(timeout=1)
