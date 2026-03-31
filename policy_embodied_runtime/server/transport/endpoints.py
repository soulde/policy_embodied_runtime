"""Endpoint normalization helpers."""

from __future__ import annotations

from pathlib import Path


def resolve_endpoint(endpoint: str) -> str:
    """Resolve a user-facing endpoint name into a concrete ZMQ endpoint."""
    if "://" in endpoint:
        return endpoint
    return f"ipc://{Path('/tmp') / endpoint}.sock"
