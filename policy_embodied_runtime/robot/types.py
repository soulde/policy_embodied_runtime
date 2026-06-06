"""Typed runtime state containers."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

@dataclass(slots=True)
class SessionContext:
    """Runtime state for a single policy session."""

    session_id: str
    step_id: int = 0
    metadata: dict[str, Any] = field(default_factory=dict)
