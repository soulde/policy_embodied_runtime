"""Type-sensitive Python/C++ runtime migration parity gate.

The module is both a pytest test file and a directly executable CTest helper.
Only response timestamps are normalized. JSON integers and floats retain
distinct tagged representations so numeric wire regressions cannot compare
equal accidentally.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from policy_embodied_runtime.profiles.loader import (  # noqa: E402
    load_policy_profile,
    load_robot_profile,
)
from policy_embodied_runtime.protocol.codec import (  # noqa: E402
    decode_envelope,
    encode_envelope,
)
from policy_embodied_runtime.protocol.errors import ProtocolError  # noqa: E402
from policy_embodied_runtime.robot.runtime import Runtime  # noqa: E402


_DRIVER_OVERRIDE: Path | None = None
POLICY_PROFILES = sorted((ROOT / "tests/golden").glob("*_policy_profile.json"))
ROBOT_PROFILES = sorted((ROOT / "tests/golden").glob("*_robot_profile.json"))


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
            "payload": payload if payload is not None else {},
            "error": None,
        },
        separators=(",", ":"),
    ).encode()


def driver_path() -> Path:
    candidates = []
    if _DRIVER_OVERRIDE is not None:
        candidates.append(_DRIVER_OVERRIDE)
    if configured := os.environ.get("POLICY_RUNTIME_PARITY_DRIVER"):
        candidates.append(Path(configured))
    candidates.extend(
        [
            ROOT / "build-clean/runtime_contract_driver",
            ROOT / "build/runtime_contract_driver",
        ]
    )
    for candidate in candidates:
        resolved = candidate if candidate.is_absolute() else ROOT / candidate
        if resolved.is_file() and os.access(resolved, os.X_OK):
            return resolved
    raise AssertionError(
        "runtime_contract_driver not found; build with "
        "-DPOLICY_RUNTIME_BUILD_TESTS=ON or set POLICY_RUNTIME_PARITY_DRIVER"
    )


def typed_json_loads(raw: bytes) -> dict[str, Any]:
    return json.loads(
        raw,
        parse_int=lambda value: ("json-int", value),
        parse_float=lambda value: ("json-float", value),
    )


def normalize(response: dict[str, Any]) -> dict[str, Any]:
    normalized = copy.deepcopy(response)
    normalized["timestamp_ns"] = ("normalized", "timestamp")
    return normalized


def python_responses(profile: Path, messages: list[bytes]) -> list[dict[str, Any]]:
    runtime = Runtime.from_profiles(policy_profile=profile)
    responses: list[dict[str, Any]] = []
    try:
        for raw in messages:
            try:
                response = runtime.handle_envelope(decode_envelope(raw))
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
    finally:
        runtime.close()
    return responses


def cpp_responses(profile: Path, messages: list[bytes]) -> list[dict[str, Any]]:
    process = subprocess.run(
        [str(driver_path()), str(profile)],
        input=b"\n".join(messages) + b"\n",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=ROOT,
        timeout=5,
        check=False,
    )
    assert process.returncode == 0, (
        f"C++ contract driver failed ({process.returncode}): "
        f"{process.stderr.decode(errors='replace')}"
    )
    responses = [typed_json_loads(line) for line in process.stdout.splitlines()]
    assert len(responses) == len(messages)
    return responses


def compare_runtime(profile: Path, messages: list[bytes]) -> None:
    python = [normalize(item) for item in python_responses(profile, messages)]
    cpp = [normalize(item) for item in cpp_responses(profile, messages)]
    assert cpp == python


def cpp_profile_summary(kind: str, profile: Path) -> dict[str, Any]:
    process = subprocess.run(
        [str(driver_path()), f"--check-{kind}-profile", str(profile)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=ROOT,
        timeout=5,
        check=False,
    )
    assert process.returncode == 0, process.stderr.decode(errors="replace")
    return json.loads(process.stdout)


def device_summary(devices: list[Any]) -> list[dict[str, Any]]:
    return [
        {
            "name": device.name,
            "type": device.device.type,
            "path": device.device.path,
            "args": device.args,
        }
        for device in devices
    ]


def python_policy_summary(profile: Path) -> dict[str, Any]:
    try:
        value = load_policy_profile(profile)
    except Exception:
        return {"accepted": False}
    return {
        "accepted": True,
        "id": value.id,
        "version": value.version,
        "policy": value.policy,
        "observation_fields": [field.name for field in value.canonical_observation_schema],
        "action_fields": [field.name for field in value.canonical_action_schema],
        "input_count": len(value.inputs),
        "output_count": len(value.outputs),
        "action_horizon": value.temporal.action_horizon,
        "observation_history": value.temporal.observation_history,
    }


def python_robot_summary(profile: Path) -> dict[str, Any]:
    try:
        value = load_robot_profile(profile)
    except Exception:
        return {"accepted": False}
    return {
        "accepted": True,
        "sensors": device_summary(value.sensors),
        "actuators": device_summary(value.actuators),
    }


def test_happy_error_null_and_numeric_runtime_parity() -> None:
    observation = json.loads((ROOT / "tests/golden/observation_request.json").read_text())
    integer_observation = copy.deepcopy(observation)
    integer_observation["request_id"] = "integer-numeric-request"
    integer_observation["payload"]["observation"]["joint_position"]["values"] = list(
        range(1, 8)
    )
    integer_observation["payload"]["observation"]["gripper_width"]["value"] = 1
    null_observation = copy.deepcopy(observation)
    null_observation["request_id"] = "null-task-request"
    null_observation["payload"]["observation"]["task_text"] = None
    messages = [
        request("health_request", {"verbose": True}, session_id="health", step_id=7),
        request("server_info_request"),
        json.dumps(observation, separators=(",", ":")).encode(),
        json.dumps(integer_observation, separators=(",", ":")).encode(),
        json.dumps(null_observation, separators=(",", ":")).encode(),
        request("future_request"),
        b"{not-json",
        request("health_request"),
    ]
    compare_runtime(
        ROOT / "policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json",
        messages,
    )


def test_session_and_reset_runtime_parity() -> None:
    observation = {
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
    messages = [
        request("observation_request", observation, session_id="session-a", step_id=42),
        request("observation_request", observation, session_id="session-a", step_id=42),
        request("observation_request", observation, session_id="session-b", step_id=42),
        request("reset_request", {"hard": True}, session_id="session-a", step_id=9),
        request("observation_request", observation, session_id="session-a", step_id=42),
    ]
    compare_runtime(
        ROOT
        / "policy_embodied_runtime/examples/policy_profiles/pi0_like_policy_profile.json",
        messages,
    )


def test_all_golden_profile_summaries_match() -> None:
    assert POLICY_PROFILES
    assert ROBOT_PROFILES
    for profile in POLICY_PROFILES:
        assert cpp_profile_summary("policy", profile) == python_policy_summary(profile), profile
    for profile in ROBOT_PROFILES:
        assert cpp_profile_summary("robot", profile) == python_robot_summary(profile), profile


def test_profile_rejection_parity() -> None:
    policy = json.loads(
        (
            ROOT
            / "policy_embodied_runtime/examples/policy_profiles/dummy_policy_profile.json"
        ).read_text()
    )
    robot = json.loads((ROOT / "tests/golden/st3215_only_robot_profile.json").read_text())
    invalid_profiles: list[tuple[str, dict[str, Any]]] = []

    duplicate_field = copy.deepcopy(policy)
    duplicate_field["canonical_observation_schema"].append(
        copy.deepcopy(duplicate_field["canonical_observation_schema"][0])
    )
    invalid_profiles.append(("policy", duplicate_field))
    zero_horizon = copy.deepcopy(policy)
    zero_horizon["temporal"]["action_horizon"] = 0
    invalid_profiles.append(("policy", zero_horizon))
    missing_path = copy.deepcopy(robot)
    missing_path["sensors"][0]["device"]["path"] = "  "
    invalid_profiles.append(("robot", missing_path))
    duplicate_cross_side = copy.deepcopy(robot)
    duplicate_cross_side["actuators"][0]["name"] = duplicate_cross_side["sensors"][0]["name"]
    invalid_profiles.append(("robot", duplicate_cross_side))

    with tempfile.TemporaryDirectory(prefix="policy-runtime-parity-") as directory:
        for index, (kind, contents) in enumerate(invalid_profiles):
            path = Path(directory) / f"invalid-{kind}-{index}.json"
            path.write_text(json.dumps(contents))
            python = (
                python_policy_summary(path)
                if kind == "policy"
                else python_robot_summary(path)
            )
            assert cpp_profile_summary(kind, path) == python == {"accepted": False}


def main() -> None:
    global _DRIVER_OVERRIDE
    parser = argparse.ArgumentParser()
    parser.add_argument("--driver", type=Path, required=True)
    args = parser.parse_args()
    _DRIVER_OVERRIDE = args.driver
    test_happy_error_null_and_numeric_runtime_parity()
    test_session_and_reset_runtime_parity()
    test_all_golden_profile_summaries_match()
    test_profile_rejection_parity()
    print("runtime parity: 4 scenarios passed")


if __name__ == "__main__":
    main()
