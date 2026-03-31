import os
import socket
import subprocess
import sys
from pathlib import Path

import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get("EPR_RUN_TCP_SMOKE") != "1",
    reason="TCP CLI smoke test is only run explicitly outside the sandbox.",
)


def test_sdk_dummy_roundtrip_cli_smoke() -> None:
    control_port = _reserve_tcp_port()
    env = os.environ.copy()
    env["EPR_ENDPOINT"] = f"tcp://127.0.0.1:{control_port}"

    completed = subprocess.run(
        [sys.executable, "policy_embodied_runtime/examples/sdk_dummy_roundtrip.py"],
        cwd=Path(__file__).resolve().parents[2],
        env=env,
        capture_output=True,
        text=True,
        timeout=15,
        check=True,
    )

    stdout = completed.stdout
    assert "'policy_id': 'dummy-policy'" in stdout
    assert "'ok': True" in stdout
    assert "'joint_position_delta'" in stdout


def _reserve_tcp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])
