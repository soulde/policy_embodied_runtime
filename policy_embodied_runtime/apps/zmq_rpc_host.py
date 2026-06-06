"""ZMQ RPC robot host."""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path

import zmq
from zmq.error import Again, ZMQError

from policy_embodied_runtime.profiles.loader import load_policy_profile, load_robot_profile
from policy_embodied_runtime.robot.policy_runtime import PolicyRuntime
from policy_embodied_runtime.protocol.messages import MessageEnvelope, SCHEMA_VERSION
from policy_embodied_runtime.protocol.codec import decode_envelope, encode_envelope
from policy_embodied_runtime.protocol.errors import ProtocolError
from policy_embodied_runtime.robot.devices.rpc import RpcActionActuator, RpcObservationSensor
from policy_embodied_runtime.robot.factory import BuiltRobot, build_robot
from policy_embodied_runtime.transport.zmq import ZmqRepTransport, resolve_endpoint


class ZmqRpcRobotHost:
    """Expose policy RPC as a robot sensor/actuator pair over ZMQ."""

    def __init__(
        self,
        *,
        endpoint: str = "embodied-policy-runtime",
        timeout_ms: int = 100,
        policy_profile: str | Path,
        robot_profile: str | Path | None = None,
    ) -> None:
        self.runtime = PolicyRuntime(
            policy_profile=load_policy_profile(policy_profile),
        )
        self.robot = _load_or_default_robot(robot_profile)
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
        """Stop the host and close sockets."""
        self._stop_event.set()
        self._wake_host()

    def _serve_socket(self) -> None:
        try:
            envelope = decode_envelope(self._socket.recv())
            response = self._handle_rpc_envelope(envelope)
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

    def _handle_rpc_envelope(self, envelope: MessageEnvelope) -> MessageEnvelope:
        if envelope.type == "observation_request":
            sensor = _rpc_sensor(self.robot)
            sensor.observation = envelope.payload.get("observation", {})
        response = self.runtime.handle_envelope(envelope)
        if response.type == "action_response":
            actuator = _rpc_actuator(self.robot)
            actuator.response = dict(response.payload.get("action", {}))
        return response

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
    """Build CLI parser for the ZMQ RPC robot host."""
    parser = argparse.ArgumentParser(description="Run the embodied policy ZMQ RPC robot host.")
    parser.add_argument("--endpoint", default="embodied-policy-runtime")
    parser.add_argument("--timeout-ms", type=int, default=100)
    parser.add_argument("--policy-profile", required=True)
    parser.add_argument("--robot-profile")
    return parser


def main() -> None:
    """CLI entrypoint."""
    args = build_arg_parser().parse_args()
    host = ZmqRpcRobotHost(
        endpoint=args.endpoint,
        timeout_ms=args.timeout_ms,
        policy_profile=args.policy_profile,
        robot_profile=args.robot_profile,
    )
    try:
        host.serve_forever()
    finally:
        host.stop()

def _load_or_default_robot(path: str | Path | None) -> BuiltRobot:
    if path is None:
        profile = load_robot_profile(
            Path("policy_embodied_runtime/examples/robot_profiles/default_rpc_robot_profile.json")
        )
    else:
        profile = load_robot_profile(path)
    return build_robot(profile)


def _rpc_sensor(robot: BuiltRobot) -> RpcObservationSensor:
    for sensor in robot.sensors:
        if isinstance(sensor, RpcObservationSensor):
            return sensor
    raise RuntimeError("robot profile does not define a policy_rpc sensor")


def _rpc_actuator(robot: BuiltRobot) -> RpcActionActuator:
    for actuator in robot.actuators:
        if isinstance(actuator, RpcActionActuator):
            return actuator
    raise RuntimeError("robot profile does not define a policy_rpc actuator")


if __name__ == "__main__":
    main()
