"""ZMQ server app for embodied policy runtime."""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path

import zmq
from zmq.error import Again, ZMQError

from policy_embodied_runtime.server.profiles.loader import load_embodiment_profile, load_policy_profile
from policy_embodied_runtime.server.core.policy_runtime import PolicyRuntime
from policy_embodied_runtime.server.schemas.messages import MessageEnvelope, SCHEMA_VERSION
from policy_embodied_runtime.server.transport.codec import decode_envelope, encode_envelope
from policy_embodied_runtime.server.transport.endpoints import resolve_endpoint
from policy_embodied_runtime.server.transport.errors import TransportError
from policy_embodied_runtime.server.transport.zmq_json import ZmqRepSocket


class ZmqPolicyServer:
    """Run a single REP socket around a PolicyRuntime."""

    def __init__(
        self,
        *,
        endpoint: str = "embodied-policy-runtime",
        timeout_ms: int = 100,
        policy_profile: str | Path,
        embodiment_profile: str | Path,
    ) -> None:
        self.runtime = PolicyRuntime(
            policy_profile=load_policy_profile(policy_profile),
            embodiment_profile=load_embodiment_profile(embodiment_profile),
        )
        self.endpoint = resolve_endpoint(endpoint)
        self.timeout_ms = timeout_ms
        self._context = zmq.Context()
        self._socket = ZmqRepSocket(self._context, self.endpoint, rcvtimeo_ms=timeout_ms)
        self._stop_event = threading.Event()

    def serve_forever(self) -> None:
        """Serve until stop() is called."""
        try:
            while not self._stop_event.is_set():
                try:
                    self._serve_socket()
                except Again:
                    continue
                except ZMQError:
                    if self._stop_event.is_set():
                        break
                    raise
        finally:
            self._socket.close()
            self._context.term()

    def stop(self) -> None:
        """Stop the server and close sockets."""
        self._stop_event.set()
        self._wake_server()

    def _serve_socket(self) -> None:
        try:
            envelope = decode_envelope(self._socket.recv())
            response = self.runtime.handle_envelope(envelope)
        except TransportError as exc:
            response = self.runtime.error_envelope(
                request_type="transport",
                request_id="unknown",
                session_id="unknown",
                step_id=0,
                code="transport_error",
                message=str(exc),
            )
        self._socket.send(encode_envelope(response))

    def _wake_server(self) -> None:
        wake_context = zmq.Context()
        wake_socket = wake_context.socket(zmq.REQ)
        wake_socket.linger = 0
        wake_socket.rcvtimeo = self.timeout_ms
        wake_socket.sndtimeo = self.timeout_ms
        try:
            wake_socket.connect(self.endpoint)
            wake_socket.send(
                encode_envelope(
                    MessageEnvelope(
                        schema=SCHEMA_VERSION,
                        type="health_request",
                        request_id="server-stop",
                        session_id="server-stop",
                        step_id=0,
                        timestamp_ns=time.time_ns(),
                        payload={"verbose": False},
                    )
                )
            )
            try:
                wake_socket.recv()
            except ZMQError:
                pass
        except ZMQError:
            pass
        finally:
            wake_socket.close(0)
            wake_context.term()

def build_arg_parser() -> argparse.ArgumentParser:
    """Build CLI parser for the ZMQ server app."""
    parser = argparse.ArgumentParser(description="Run the embodied policy ZMQ server.")
    parser.add_argument("--endpoint", default="embodied-policy-runtime")
    parser.add_argument("--timeout-ms", type=int, default=100)
    parser.add_argument("--policy-profile", required=True)
    parser.add_argument("--embodiment-profile", required=True)
    return parser


def main() -> None:
    """CLI entrypoint."""
    args = build_arg_parser().parse_args()
    server = ZmqPolicyServer(
        endpoint=args.endpoint,
        timeout_ms=args.timeout_ms,
        policy_profile=args.policy_profile,
        embodiment_profile=args.embodiment_profile,
    )
    try:
        server.serve_forever()
    finally:
        server.stop()


if __name__ == "__main__":
    main()
