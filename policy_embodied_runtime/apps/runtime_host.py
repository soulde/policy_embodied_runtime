"""Robot runtime host."""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path

import zmq
from zmq.error import Again, ZMQError

from policy_embodied_runtime.robot.runtime import Runtime
from policy_embodied_runtime.protocol.messages import MessageEnvelope, SCHEMA_VERSION
from policy_embodied_runtime.protocol.codec import decode_envelope, encode_envelope
from policy_embodied_runtime.protocol.errors import ProtocolError
from policy_embodied_runtime.transport.zmq import ZmqRepTransport, resolve_endpoint


class RuntimeHost:
    """Expose the robot runtime over a message transport."""

    def __init__(
        self,
        *,
        endpoint: str = "embodied-policy-runtime",
        timeout_ms: int = 100,
        policy_profile: str | Path,
        robot_profile: str | Path | None = None,
    ) -> None:
        self.runtime = Runtime.from_profiles(
            policy_profile=policy_profile,
            robot_profile=_default_robot_profile(robot_profile),
        )
        self.runtime.open()
        self.endpoint = resolve_endpoint(endpoint)
        self.timeout_ms = timeout_ms
        self._context = zmq.Context()
        self._socket = ZmqRepTransport(self._context, self.endpoint, rcvtimeo_ms=timeout_ms)
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
        """Stop the host and close runtime resources."""
        self._stop_event.set()
        self._wake_host()
        self.runtime.close()

    def _serve_socket(self) -> None:
        try:
            envelope = decode_envelope(self._socket.recv())
            response = self.runtime.handle_envelope(envelope)
        except ProtocolError as exc:
            response = self.runtime.error_envelope(
                request_type="protocol",
                request_id="unknown",
                session_id="unknown",
                step_id=0,
                code="protocol_error",
                message=str(exc),
            )
        self._socket.send(encode_envelope(response))

    def _wake_host(self) -> None:
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
                        request_id="host-stop",
                        session_id="host-stop",
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
    """Build CLI parser for the robot runtime host."""
    parser = argparse.ArgumentParser(description="Run the robot runtime host.")
    parser.add_argument("--endpoint", default="embodied-policy-runtime")
    parser.add_argument("--timeout-ms", type=int, default=100)
    parser.add_argument("--policy-profile", required=True)
    parser.add_argument("--robot-profile")
    return parser


def main() -> None:
    """CLI entrypoint."""
    args = build_arg_parser().parse_args()
    host = RuntimeHost(
        endpoint=args.endpoint,
        timeout_ms=args.timeout_ms,
        policy_profile=args.policy_profile,
        robot_profile=args.robot_profile,
    )
    try:
        host.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        host.stop()


def _default_robot_profile(path: str | Path | None) -> str | Path:
    if path is not None:
        return path
    return Path("policy_embodied_runtime/examples/robot_profiles/default_rpc_robot_profile.json")


if __name__ == "__main__":
    main()
