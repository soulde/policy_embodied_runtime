"""Runtime-specific exceptions."""

from __future__ import annotations


class PolicyRuntimeError(Exception):
    """Base runtime error."""


class AdapterNotFoundError(PolicyRuntimeError):
    """Raised when a configured adapter is not registered."""


class SessionNotFoundError(PolicyRuntimeError):
    """Raised when a referenced session does not exist."""


class ValidationRuntimeError(PolicyRuntimeError):
    """Raised when request payload validation fails."""
