"""Client SDK exceptions."""

from __future__ import annotations


class PolicyClientError(Exception):
    """Base client error."""


class PolicyClientTransportError(PolicyClientError):
    """Raised when transport fails or the server returns an error."""
