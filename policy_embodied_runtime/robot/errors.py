"""Runtime-specific exceptions."""

from __future__ import annotations


class RobotRuntimeError(Exception):
    """Base runtime error."""


class PluginNotFoundError(RobotRuntimeError):
    """Raised when a configured plugin is not registered."""


class SessionNotFoundError(RobotRuntimeError):
    """Raised when a referenced session does not exist."""


class ValidationRuntimeError(RobotRuntimeError):
    """Raised when request payload validation fails."""
