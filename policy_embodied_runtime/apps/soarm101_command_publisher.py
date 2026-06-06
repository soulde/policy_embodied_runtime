"""SO-ARM101 command publisher for the runtime host."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import zmq

from policy_embodied_runtime.profiles.loader import load_policy_profile
from policy_embodied_runtime.protocol.codec import decode_envelope, encode_envelope
from policy_embodied_runtime.protocol.messages import MessageEnvelope, SCHEMA_VERSION
from policy_embodied_runtime.transport.zmq import ZmqReqTransport, resolve_endpoint

DEFAULT_POLICY_PROFILE = Path("policy_embodied_runtime/examples/policy_profiles/soarm101_sim_policy_profile.json")


def publish_commands(
    *,
    endpoint: str = "embodied-policy-runtime",
    policy_profile: str | Path = DEFAULT_POLICY_PROFILE,
    duration_s: float = 5.0,
    hz: float = 20.0,
    amplitude_rad: float = 0.35,
    timeout_ms: int = 2000,
) -> None:
    """Publish SO-ARM101 joint target observations to a runtime host."""
    profile = load_policy_profile(policy_profile)
    context = zmq.Context()
    client = ZmqReqTransport(context, resolve_endpoint(endpoint), timeout_ms=timeout_ms)
    try:
        period = 1.0 / hz
        steps = max(1, int(duration_s * hz))
        for step in range(steps):
            envelope = MessageEnvelope(
                schema=SCHEMA_VERSION,
                type="observation_request",
                request_id=f"soarm101-command-{step}",
                session_id="soarm101-command-publisher",
                step_id=step,
                timestamp_ns=time.time_ns(),
                payload={"observation": _command_observation(profile.inputs, step, steps, amplitude_rad)},
            )
            response = decode_envelope(client.request(encode_envelope(envelope)))
            if response.error is not None:
                raise RuntimeError(f"runtime host error: {response.error.code}: {response.error.message}")
            if response.type != "action_response":
                raise RuntimeError(f"unexpected runtime host response: {response.type}")
            time.sleep(period)
    finally:
        client.close()
        context.term()


def _command_observation(bindings, step: int, steps: int, amplitude_rad: float) -> dict[str, object]:
    phase = step / max(1, steps - 1)
    targets = {
        "shoulder_pan_command": math.pi + amplitude_rad * math.sin(phase * math.tau),
        "shoulder_lift_command": math.pi + amplitude_rad * math.sin(phase * math.tau + 0.6),
        "elbow_command": math.pi + amplitude_rad * math.sin(phase * math.tau + 1.2),
        "wrist_pitch_command": math.pi + amplitude_rad * math.sin(phase * math.tau + 1.8),
        "wrist_roll_command": math.pi + amplitude_rad * math.sin(phase * math.tau + 2.4),
        "gripper_command": math.pi + 0.2 * math.sin(phase * math.tau + 3.0),
    }
    observation = {}
    for servo_id, binding in enumerate(bindings, start=1):
        observation[binding.canonical_field] = {
            "servo_id": servo_id,
            "target_position_rad": targets[binding.canonical_field],
            "enabled": True,
        }
    return observation


def build_arg_parser() -> argparse.ArgumentParser:
    """Build command publisher parser."""
    parser = argparse.ArgumentParser(description="Publish SO-ARM101 control commands to a runtime host.")
    parser.add_argument("--endpoint", default="embodied-policy-runtime")
    parser.add_argument("--policy-profile", type=Path, default=DEFAULT_POLICY_PROFILE)
    parser.add_argument("--duration-s", type=float, default=5.0)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--amplitude-rad", type=float, default=0.35)
    parser.add_argument("--timeout-ms", type=int, default=2000)
    return parser


def main() -> None:
    """CLI entrypoint."""
    args = build_arg_parser().parse_args()
    publish_commands(
        endpoint=args.endpoint,
        policy_profile=args.policy_profile,
        duration_s=args.duration_s,
        hz=args.hz,
        amplitude_rad=args.amplitude_rad,
        timeout_ms=args.timeout_ms,
    )
    print("soarm101 command publishing ok")


if __name__ == "__main__":
    main()
