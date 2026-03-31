"""Run a local dummy policy roundtrip through the SDK."""

from __future__ import annotations

import os
from pathlib import Path

from policy_embodied_runtime.sdk.policy_client.client import PolicyClient
from policy_embodied_runtime.sdk.policy_client.types import EndpointConfig


def main() -> None:
    examples_dir = Path(__file__).resolve().parent
    endpoints = EndpointConfig(
        endpoint=os.environ.get("EPR_ENDPOINT", "embodied-policy-runtime"),
    )
    policy_profile = str(examples_dir / "policy_profiles" / "dummy_policy_profile.json")
    embodiment_profile = str(examples_dir / "embodiment_profiles" / "dummy_embodiment_profile.json")
    client = PolicyClient.launch(
        endpoints,
        policy_profile=policy_profile,
        embodiment_profile=embodiment_profile,
    )
    try:
        print(client.health())
        session = client.session("demo-session")
        print(session.reset())
        print(
            session.step(
                {
                    "joint_position": {
                        "values": [0.0] * 7,
                        "joint_names": [f"joint_{index}" for index in range(1, 8)],
                        "unit": "rad",
                    },
                    "gripper_width": {"value": 0.04, "unit": "m"},
                    "task_text": {"text": "pick up the cube"},
                    "meta": {"source": "example"},
                }
            )
        )
    finally:
        client.close()


if __name__ == "__main__":
    main()
