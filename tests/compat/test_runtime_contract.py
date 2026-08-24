from __future__ import annotations

import argparse
import copy
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
        responses.append(typed_json_loads(encode_envelope(response)))
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
    return [typed_json_loads(line) for line in process.stdout.splitlines()]


def typed_json_loads(raw: bytes) -> dict[str, Any]:
    return json.loads(
        raw,
        parse_int=lambda value: ("json-int", value),
        parse_float=lambda value: ("json-float", value),
    )


def normalize(response: dict[str, Any]) -> dict[str, Any]:
    normalized = dict(response)
    normalized["timestamp_ns"] = ("normalized", "timestamp")
    return normalized


def compare_case(driver: Path, profile: Path, messages: list[bytes]) -> None:
    python = python_responses(profile, messages)
    cpp = cpp_responses(driver, profile, messages)
    assert len(cpp) == len(python)
    assert [normalize(response) for response in cpp] == [
        normalize(response) for response in python
    ]


def compare_runtime_validation_errors(
    driver: Path, profile: Path, messages: list[bytes]
) -> None:
    python = python_responses(profile, messages)
    cpp = cpp_responses(driver, profile, messages)

    def normalized_error(response: dict[str, Any]) -> dict[str, Any]:
        value = copy.deepcopy(normalize(response))
        assert value["error"]["code"] == "runtime_error"
        assert value["error"]["message"]
        value["error"]["message"] = "runtime-validation-diagnostic"
        return value

    assert [normalized_error(response) for response in cpp] == [
        normalized_error(response) for response in python
    ]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--driver", type=Path, required=True)
    args = parser.parse_args()

    observation = json.loads((ROOT / "tests/golden/observation_request.json").read_text())
    integer_observation = json.loads(json.dumps(observation))
    integer_observation["request_id"] = "integer-numeric-request"
    integer_observation["payload"]["observation"]["joint_position"]["values"] = [
        1,
        2,
        3,
        4,
        5,
        6,
        7,
    ]
    integer_observation["payload"]["observation"]["gripper_width"]["value"] = 1
    null_task_observation = copy.deepcopy(observation)
    null_task_observation["request_id"] = "null-task-request"
    null_task_observation["payload"]["observation"]["task_text"] = None
    dummy_messages = [
        request("health_request", {"verbose": True}, session_id="health", step_id=7),
        request("server_info_request"),
        json.dumps(observation, separators=(",", ":")).encode(),
        json.dumps(integer_observation, separators=(",", ":")).encode(),
        json.dumps(null_task_observation, separators=(",", ":")).encode(),
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

    invalid_payload_messages = [
        request(
            "health_request",
            {"unexpected": True},
            request_id="invalid-health",
            session_id="invalid-session",
            step_id=3,
        ),
        request(
            "reset_request",
            {"hard": []},
            request_id="invalid-reset",
            session_id="invalid-session",
            step_id=4,
        ),
        request(
            "observation_request",
            {},
            request_id="invalid-observation",
            session_id="invalid-session",
            step_id=5,
        ),
    ]
    compare_runtime_validation_errors(
        args.driver,
        ROOT / "policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json",
        invalid_payload_messages,
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

    metadata_observation = copy.deepcopy(pi0_observation)
    metadata_observation["observation"]["joint_position"]["values"] = [0.0]
    metadata_observation["observation"]["joint_position"]["joint_names"] = [
        "joint_a"
    ]
    compare_case(
        args.driver,
        ROOT / "tests/golden/pi0_metadata_policy_profile.json",
        [request("observation_request", metadata_observation)],
    )


if __name__ == "__main__":
    main()
