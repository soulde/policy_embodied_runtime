"""Client SDK for embodied policy runtime."""

from __future__ import annotations

import time
from typing import Any

import zmq

from policy_embodied_runtime.sdk.policy_client.builders import build_envelope
from policy_embodied_runtime.sdk.policy_client.codec import decode_envelope, encode_envelope
from policy_embodied_runtime.sdk.policy_client.errors import PolicyClientTransportError
from policy_embodied_runtime.sdk.policy_client.launcher import LocalServerHandle, launch_local_server
from policy_embodied_runtime.sdk.policy_client.types import EndpointConfig
from policy_embodied_runtime.server.schemas.messages import HealthResponse, MessageEnvelope
from policy_embodied_runtime.server.transport.endpoints import resolve_endpoint
from policy_embodied_runtime.server.transport.zmq_json import ZmqReqSocket


class PolicyClient:
    """Python SDK for connecting to or launching a policy server."""

    def __init__(
        self,
        endpoints: EndpointConfig,
        *,
        timeout_ms: int = 2000,
        context: zmq.Context | None = None,
        server_handle: LocalServerHandle | None = None,
    ) -> None:
        self._endpoints = endpoints
        self._timeout_ms = timeout_ms
        self._context = context or zmq.Context()
        self._owns_context = context is None
        self._server_handle = server_handle
        self._socket = ZmqReqSocket(
            self._context,
            resolve_endpoint(endpoints.endpoint),
            timeout_ms=timeout_ms,
        )

    @classmethod
    def connect(
        cls,
        endpoint: EndpointConfig,
        timeout_ms: int = 2000,
        *,
        context: zmq.Context | None = None,
    ) -> "PolicyClient":
        """Connect to an existing server."""
        return cls(endpoint, timeout_ms=timeout_ms, context=context)

    @classmethod
    def launch(
        cls,
        endpoint: EndpointConfig,
        timeout_ms: int = 2000,
        *,
        policy_profile: str,
        embodiment_profile: str,
        python_executable: str | None = None,
        wait_for_server: float = 3.0,
    ) -> "PolicyClient":
        """Launch a local server subprocess and connect to it."""
        server_handle = launch_local_server(
            endpoint,
            policy_profile=policy_profile,
            embodiment_profile=embodiment_profile,
            python_executable=python_executable,
        )
        client = cls(endpoint, timeout_ms=timeout_ms, server_handle=server_handle)
        deadline = time.monotonic() + wait_for_server
        while True:
            try:
                client.health()
                return client
            except Exception:
                if time.monotonic() >= deadline:
                    client.close()
                    raise
                time.sleep(0.1)

    def health(self) -> dict[str, Any]:
        """Return server health."""
        response = self._send_request("health_request", payload={"verbose": False}, session_id="health")
        return HealthResponse.model_validate(response.payload).model_dump()

    def reset(self, session_id: str) -> dict[str, Any]:
        """Reset a remote session."""
        response = self._send_request("reset_request", payload={"hard": False}, session_id=session_id)
        return response.payload

    def infer(self, request: dict[str, Any], *, session_id: str, step_id: int = 0) -> dict[str, Any]:
        """Send one inference request and return the action response payload."""
        response = self._send_request(
            "observation_request",
            payload=request,
            session_id=session_id,
            step_id=step_id,
        )
        return response.payload

    def close(self) -> None:
        """Close sockets and terminate any launched subprocess."""
        self._socket.close()
        if self._server_handle is not None:
            self._server_handle.terminate()
            self._server_handle = None
        if self._owns_context:
            self._context.term()

    def session(self, session_id: str) -> "PolicySession":
        """Create a session-scoped wrapper."""
        from policy_embodied_runtime.sdk.policy_client.session import PolicySession

        return PolicySession(self, session_id)

    def _send_request(
        self,
        message_type: str,
        *,
        payload: dict[str, Any],
        session_id: str,
        step_id: int = 0,
    ) -> MessageEnvelope:
        envelope = build_envelope(message_type, session_id=session_id, payload=payload, step_id=step_id)
        return self._roundtrip(envelope)

    def _roundtrip(self, envelope: MessageEnvelope) -> MessageEnvelope:
        response = decode_envelope(self._socket.request(encode_envelope(envelope)))
        if response.error is not None:
            raise PolicyClientTransportError(f"{response.error.code}: {response.error.message}")
        return response
