from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from policy_embodied_runtime.protocol.codec import decode_envelope, encode_envelope
from policy_embodied_runtime.protocol.errors import ProtocolError
from policy_embodied_runtime.robot.runtime import Runtime


def request(
    message_type: str,
    payload: dict[str, Any] | None = None,
    *,
    request_id: str = "request-1",
    session_id: str = "session-1",
    step_id: int = 0,
) -> bytes:
    return json.dumps(
        {
            "schema": "embodied-policy-runtime/v1alpha1",
            "type": message_type,
            "request_id": request_id,
            "session_id": session_id,
            "step_id": step_id,
            "timestamp_ns": 1,
            "payload": payload or {},
            "error": None,
        },
        separators=(",", ":"),
    ).encode()


def python_responses(profile: Path, messages: list[bytes]) -> list[dict[str, Any]]:
    runtime = Runtime.from_profiles(policy_profile=profile)
    responses: list[dict[str, Any]] = []
    for raw in messages:
        try:
            envelope = decode_envelope(raw)
            response = runtime.handle_envelope(envelope)
        except ProtocolError as error:
            response = runtime.error_envelope(
                request_type="protocol",
                request_id="unknown",
                session_id="unknown",
                step_id=0,
                code="protocol_error",
                message=str(error),
            )
        responses.append(json.loads(encode_envelope(response)))
    return responses


def cpp_responses(
    driver: Path, profile: Path, messages: list[bytes]
) -> list[dict[str, Any]]:
    process = subprocess.run(
        [str(driver), str(profile)],
        input=b"\n".join(messages) + b"\n",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=ROOT,
        timeout=5,
        check=False,
    )
    if process.returncode != 0:
        raise AssertionError(
            f"C++ contract driver failed ({process.returncode}): "
            f"{process.stderr.decode(errors='replace')}"
        )
    return [json.loads(line) for line in process.stdout.splitlines()]


def normalize(response: dict[str, Any]) -> dict[str, Any]:
    normalized = dict(response)
    normalized["timestamp_ns"] = 0
    return normalized


def compare_case(driver: Path, profile: Path, messages: list[bytes]) -> None:
    python = python_responses(profile, messages)
    cpp = cpp_responses(driver, profile, messages)
    assert len(cpp) == len(python)
    assert [normalize(response) for response in cpp] == [
        normalize(response) for response in python
    ]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--driver", type=Path, required=True)
    args = parser.parse_args()

    observation = json.loads((ROOT / "tests/golden/observation_request.json").read_text())
    dummy_messages = [
        request("health_request", {"verbose": True}, session_id="health", step_id=7),
        request("server_info_request"),
        json.dumps(observation, separators=(",", ":")).encode(),
        request("reset_request", {"hard": True}, session_id="session-1", step_id=9),
        request("future_request"),
        b"{not-json",
        request("health_request"),
    ]
    compare_case(
        args.driver,
        ROOT / "policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json",
        dummy_messages,
    )

    pi0_observation = {
        "observation": {
            "joint_position": {
                "values": [0.0, 0.1],
                "joint_names": ["joint_a", "joint_b"],
                "unit": "rad",
            },
            "task_text": {"text": "move"},
            "image": {
                "encoding": "uri",
                "data": "memory://rgb/front",
                "mime_type": "image/jpeg",
            },
        }
    }
    pi0_messages = [
        request("observation_request", pi0_observation, session_id="pi0", step_id=42),
        request("observation_request", pi0_observation, session_id="pi0", step_id=42),
        request("reset_request", session_id="pi0"),
        request("observation_request", pi0_observation, session_id="pi0", step_id=42),
    ]
    compare_case(
        args.driver,
        ROOT / "policy_embodied_runtime/examples/policy_profiles/pi0_like_policy_profile.json",
        pi0_messages,
    )


if __name__ == "__main__":
    main()
