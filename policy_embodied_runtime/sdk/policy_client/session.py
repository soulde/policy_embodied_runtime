"""Session-scoped SDK wrapper."""

from __future__ import annotations

from typing import Any


class PolicySession:
    """Session-oriented client wrapper."""

    def __init__(self, client: "PolicyClient", session_id: str) -> None:
        self._client = client
        self._session_id = session_id
        self._step_id = 0

    @property
    def session_id(self) -> str:
        """Return the session identifier."""
        return self._session_id

    def reset(self) -> dict[str, Any]:
        """Reset the remote session and local step counter."""
        self._step_id = 0
        return self._client.reset(self._session_id)

    def step(self, obs: dict[str, Any]) -> dict[str, Any]:
        """Send one observation to the runtime and return the action payload."""
        response = self._client.infer({"observation": obs}, session_id=self._session_id, step_id=self._step_id)
        self._step_id += 1
        return response


from policy_embodied_runtime.sdk.policy_client.client import PolicyClient  # noqa: E402
