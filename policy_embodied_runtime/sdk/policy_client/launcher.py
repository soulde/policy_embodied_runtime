"""Subprocess launcher for a local policy server."""

from __future__ import annotations

import subprocess
import sys
from dataclasses import dataclass

from policy_embodied_runtime.sdk.policy_client.types import EndpointConfig


@dataclass(slots=True)
class LocalServerHandle:
    """Bookkeeping for a launched local server process."""

    process: subprocess.Popen[bytes]
    endpoints: EndpointConfig

    def terminate(self) -> None:
        """Terminate the launched subprocess."""
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)


def launch_local_server(
    endpoints: EndpointConfig,
    *,
    policy_profile: str,
    embodiment_profile: str,
    python_executable: str | None = None,
) -> LocalServerHandle:
    """Launch the ZMQ policy server as a subprocess."""
    process = subprocess.Popen(
        [
            python_executable or sys.executable,
            "-m",
            "policy_embodied_runtime.server.apps.zmq_server",
            "--endpoint",
            endpoints.endpoint,
            "--policy-profile",
            policy_profile,
            "--embodiment-profile",
            embodiment_profile,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return LocalServerHandle(process=process, endpoints=endpoints)
