from __future__ import annotations

import argparse
import json
import signal
import socket
import subprocess
import time
from pathlib import Path

import zmq


ROOT = Path(__file__).resolve().parents[2]


def unused_tcp_endpoint() -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    return f"tcp://127.0.0.1:{port}"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", type=Path, required=True)
    args = parser.parse_args()

    endpoint = unused_tcp_endpoint()
    process = subprocess.Popen(
        [
            str(args.host),
            "--endpoint",
            endpoint,
            "--timeout-ms",
            "50",
            "--policy-profile",
            str(
                ROOT
                / "policy_embodied_runtime/examples/policy_profiles/"
                "dummy_policy_profile.json"
            ),
        ],
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    context = zmq.Context()
    client = context.socket(zmq.REQ)
    client.linger = 0
    client.rcvtimeo = 250
    client.sndtimeo = 250
    client.connect(endpoint)
    try:
        deadline = time.monotonic() + 3
        while True:
            try:
                client.send(b"{not-json")
                malformed = json.loads(client.recv())
                break
            except zmq.Again:
                client.close(0)
                if time.monotonic() >= deadline:
                    raise AssertionError("C++ ZeroMQ host did not become ready")
                time.sleep(0.025)
                client = context.socket(zmq.REQ)
                client.linger = 0
                client.rcvtimeo = 250
                client.sndtimeo = 250
                client.connect(endpoint)
        assert malformed["type"] == "protocol_error"
        assert malformed["error"]["code"] == "protocol_error"

        health = {
            "schema": "embodied-policy-runtime/v1alpha1",
            "type": "health_request",
            "request_id": "health-1",
            "session_id": "session-1",
            "step_id": 0,
            "timestamp_ns": 1,
            "payload": {"verbose": False},
            "error": None,
        }
        client.send(json.dumps(health, separators=(",", ":")).encode())
        response = json.loads(client.recv())
        assert response["type"] == "health_response"
        assert response["payload"] == {"ok": True, "policy_id": "dummy-policy"}
        assert response["error"] is None
    finally:
        client.close(0)
        context.term()
        process.send_signal(signal.SIGTERM)
        try:
            _, stderr = process.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            _, stderr = process.communicate(timeout=1)
            raise AssertionError("C++ ZeroMQ host did not stop after SIGTERM")
        if process.returncode != 0:
            raise AssertionError(
                f"C++ ZeroMQ host failed ({process.returncode}): "
                f"{stderr.decode(errors='replace')}"
            )


if __name__ == "__main__":
    main()
