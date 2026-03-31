"""SDK type helpers."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class EndpointConfig:
    """Base endpoint for control-plane and inference-plane sockets."""

    endpoint: str
