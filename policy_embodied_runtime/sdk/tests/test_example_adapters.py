import threading
import uuid
from pathlib import Path

from policy_embodied_runtime.sdk.policy_client.client import PolicyClient
from policy_embodied_runtime.sdk.policy_client.types import EndpointConfig
from policy_embodied_runtime.server.apps.zmq_server import ZmqPolicyServer


def _endpoint_name(name: str) -> str:
    return f"{name}-{uuid.uuid4().hex}"


def test_pi0_like_and_franka_like_roundtrip() -> None:
    endpoints = EndpointConfig(endpoint=_endpoint_name("sdk-pi0"))
    server = ZmqPolicyServer(
        endpoint=endpoints.endpoint,
        policy_profile=Path("policy_embodied_runtime/examples/policy_profiles/pi0_like_policy_profile.json"),
        embodiment_profile=Path("policy_embodied_runtime/examples/embodiment_profiles/franka_like_profile.json"),
    )
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()

    client = PolicyClient.connect(endpoints)
    try:
        response = client.infer(
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
                    "meta": {"source": "sdk-example-adapter-test"},
                }
            },
            session_id="pi0-session",
        )
        assert "action" in response
        assert len(response["action"]["action_chunk"]) == 4
    finally:
        client.close()
        server.stop()
        thread.join(timeout=1)
