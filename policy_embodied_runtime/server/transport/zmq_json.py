"""ZMQ JSON request/response transport."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import zmq


class ZmqRepSocket:
    """Thin wrapper around a REP socket."""

    def __init__(self, context: zmq.Context, endpoint: str, *, rcvtimeo_ms: int | None = None) -> None:
        self._endpoint = endpoint
        self._ipc_path = _ipc_path_from_endpoint(endpoint)
        if self._ipc_path is not None:
            self._ipc_path.parent.mkdir(parents=True, exist_ok=True)
            self._ipc_path.unlink(missing_ok=True)
        self._socket = context.socket(zmq.REP)
        self._socket.linger = 0
        if rcvtimeo_ms is not None:
            self._socket.rcvtimeo = rcvtimeo_ms
            self._socket.sndtimeo = rcvtimeo_ms
        self._socket.bind(endpoint)

    def recv(self) -> bytes:
        """Receive a request frame."""
        return self._socket.recv()

    def send(self, payload: bytes) -> None:
        """Send a response frame."""
        self._socket.send(payload)

    def close(self) -> None:
        """Close the socket."""
        self._socket.close(0)
        if self._ipc_path is not None:
            self._ipc_path.unlink(missing_ok=True)


class ZmqReqSocket:
    """Thin wrapper around a REQ socket for tests and SDK."""

    def __init__(self, context: zmq.Context, endpoint: str, *, timeout_ms: int = 2000) -> None:
        self._socket = context.socket(zmq.REQ)
        self._socket.linger = 0
        self._socket.rcvtimeo = timeout_ms
        self._socket.sndtimeo = timeout_ms
        self._socket.connect(endpoint)

    def request(self, payload: bytes) -> bytes:
        """Send and receive one request/response cycle."""
        self._socket.send(payload)
        return self._socket.recv()

    def close(self) -> None:
        """Close the socket."""
        self._socket.close(0)


def _ipc_path_from_endpoint(endpoint: str) -> Path | None:
    if not endpoint.startswith("ipc://"):
        return None
    return Path(endpoint.removeprefix("ipc://"))
