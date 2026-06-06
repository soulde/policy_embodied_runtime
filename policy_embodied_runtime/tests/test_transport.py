import socket
import threading
import time

import zmq

from policy_embodied_runtime.apps.runtime_host import RuntimeHost
from policy_embodied_runtime.protocol.codec import decode_envelope, encode_envelope
from policy_embodied_runtime.transport.zmq import ZmqReqTransport, resolve_endpoint
from policy_embodied_runtime.protocol.messages import MessageEnvelope


def _endpoint_name(name: str) -> str:
    _ = name
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    return f"tcp://127.0.0.1:{port}"


def test_codec_roundtrip() -> None:
    envelope = MessageEnvelope(
        schema="embodied-policy-runtime/v1alpha1",
        type="health_request",
        request_id="req-1",
        session_id="session-1",
        step_id=0,
        timestamp_ns=1,
        payload={"verbose": False},
    )
    decoded = decode_envelope(encode_envelope(envelope))
    assert decoded.type == "health_request"
    assert decoded.schema_ == "embodied-policy-runtime/v1alpha1"


def test_zmq_rpc_robot_host_handles_health_and_infer() -> None:
    endpoint = _endpoint_name("transport")
    context = zmq.Context()
    host = RuntimeHost(
        endpoint=endpoint,
        policy_profile="policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json",
    )
    thread = threading.Thread(target=host.serve_forever, daemon=True)
    thread.start()
    time.sleep(0.1)

    client = ZmqReqTransport(context, resolve_endpoint(endpoint), timeout_ms=2000)
    try:
        health_response = decode_envelope(
            client.request(
                encode_envelope(
                    MessageEnvelope(
                        schema="embodied-policy-runtime/v1alpha1",
                        type="health_request",
                        request_id="health-1",
                        session_id="session-1",
                        step_id=0,
                        timestamp_ns=1,
                        payload={"verbose": False},
                    )
                )
            )
        )
        assert health_response.type == "health_response"
        assert health_response.payload == {"ok": True, "policy_id": "dummy-policy"}

        infer_response = decode_envelope(
            client.request(
                encode_envelope(
                    MessageEnvelope(
                        schema="embodied-policy-runtime/v1alpha1",
                        type="observation_request",
                        request_id="infer-1",
                        session_id="session-1",
                        step_id=0,
                        timestamp_ns=1,
                        payload={
                            "observation": {
                                "joint_position": {
                                    "values": [0.0] * 7,
                                    "joint_names": [f"joint_{index}" for index in range(1, 8)],
                                    "unit": "rad",
                                },
                                "gripper_width": {"value": 0.04, "unit": "m"},
                                "task_text": {"text": "move to object"},
                                "meta": {"source": "transport-test"},
                            }
                        },
                    )
                )
            )
        )
        assert infer_response.type == "action_response"
        assert "joint_position_delta" in infer_response.payload["action"]
    finally:
        client.close()
        host.stop()
        thread.join(timeout=1)
        context.term()


def test_zmq_rpc_robot_host_handles_vla_observation() -> None:
    endpoint = _endpoint_name("transport-vla")
    context = zmq.Context()
    host = RuntimeHost(
        endpoint=endpoint,
        policy_profile="policy_embodied_runtime/examples/policy_profiles/pi0_like_policy_profile.json",
    )
    thread = threading.Thread(target=host.serve_forever, daemon=True)
    thread.start()
    time.sleep(0.1)

    client = ZmqReqTransport(context, resolve_endpoint(endpoint), timeout_ms=2000)
    try:
        infer_response = decode_envelope(
            client.request(
                encode_envelope(
                    MessageEnvelope(
                        schema="embodied-policy-runtime/v1alpha1",
                        type="observation_request",
                        request_id="infer-vla-1",
                        session_id="session-vla",
                        step_id=0,
                        timestamp_ns=1,
                        payload={
                            "observation": {
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
                                },
                                "gripper_width": {"value": 0.04, "unit": "m"},
                                "task_text": {"text": "move to the cup"},
                                "image": {
                                    "encoding": "uri",
                                    "data": "memory://rgb/front",
                                    "mime_type": "image/jpeg",
                                },
                                "meta": {"source": "transport-vla-test"},
                            }
                        },
                    )
                )
            )
        )
        assert infer_response.type == "action_response"
        assert len(infer_response.payload["action"]["action_chunk"]) == 4
    finally:
        client.close()
        host.stop()
        thread.join(timeout=1)
        context.term()
