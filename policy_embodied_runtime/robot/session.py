"""Session helpers."""

from __future__ import annotations

from policy_embodied_runtime.robot.types import SessionContext


def new_session(session_id: str) -> SessionContext:
    """Create a new session context."""
    return SessionContext(session_id=session_id)
