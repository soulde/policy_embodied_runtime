"""ZMQ transport implementations."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import zmq

from policy_embodied_runtime.transport.base import TransportFrame


def resolve_endpoint(endpoint: str) -> str:
    """Resolve a user-facing endpoint name into a concrete ZMQ endpoint."""
    if "://" in endpoint:
        return endpoint
    return f"ipc://{Path('/tmp') / endpoint}.sock"


class ZmqRepTransport:
    """ZMQ REP transport."""

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

    def open(self) -> None:
        """ZMQ sockets are opened during construction."""

    def receive(self) -> TransportFrame | None:
        """Receive a request frame."""
        return TransportFrame(id=0, payload=self._socket.recv())

    def recv(self) -> bytes:
        """Receive raw request bytes."""
        frame = self.receive()
        return b"" if frame is None else frame.payload

    def send(self, frame: TransportFrame | bytes) -> None:
        """Send a response frame."""
        payload = frame if isinstance(frame, bytes) else frame.payload
        self._socket.send(payload)

    def close(self) -> None:
        """Close the socket."""
        self._socket.close(0)
        if self._ipc_path is not None:
            self._ipc_path.unlink(missing_ok=True)


class ZmqReqTransport:
    """ZMQ REQ transport."""

    def __init__(self, context: zmq.Context, endpoint: str, *, timeout_ms: int = 2000) -> None:
        self._socket = context.socket(zmq.REQ)
        self._socket.linger = 0
        self._socket.rcvtimeo = timeout_ms
        self._socket.sndtimeo = timeout_ms
        self._socket.connect(endpoint)

    def open(self) -> None:
        """ZMQ sockets are opened during construction."""

    def send(self, frame: TransportFrame) -> None:
        """Send one request frame."""
        self._socket.send(frame.payload)

    def receive(self) -> TransportFrame | None:
        """Receive one response frame."""
        return TransportFrame(id=0, payload=self._socket.recv())

    def request(self, payload: bytes) -> bytes:
        """Send and receive one request/response cycle."""
        self.send(TransportFrame(id=0, payload=payload))
        frame = self.receive()
        return b"" if frame is None else frame.payload

    def close(self) -> None:
        """Close the socket."""
        self._socket.close(0)


def _ipc_path_from_endpoint(endpoint: str) -> Path | None:
    if not endpoint.startswith("ipc://"):
        return None
    return Path(endpoint.removeprefix("ipc://"))
