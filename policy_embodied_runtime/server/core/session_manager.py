"""In-memory session manager."""

from __future__ import annotations

from policy_embodied_runtime.server.core.errors import SessionNotFoundError
from policy_embodied_runtime.server.core.session import new_session
from policy_embodied_runtime.server.core.types import SessionContext


class SessionManager:
    """Manage runtime session state."""

    def __init__(self) -> None:
        self._sessions: dict[str, SessionContext] = {}

    def get_or_create(self, session_id: str) -> SessionContext:
        """Get a session or create it if it does not exist."""
        session = self._sessions.get(session_id)
        if session is None:
            session = new_session(session_id)
            self._sessions[session_id] = session
        return session

    def get(self, session_id: str) -> SessionContext:
        """Return an existing session."""
        try:
            return self._sessions[session_id]
        except KeyError as exc:
            raise SessionNotFoundError(f"unknown session_id: {session_id}") from exc

    def reset(self, session_id: str) -> SessionContext:
        """Reset or create a session."""
        session = new_session(session_id)
        self._sessions[session_id] = session
        return session

    def clear(self) -> None:
        """Remove all sessions."""
        self._sessions.clear()
