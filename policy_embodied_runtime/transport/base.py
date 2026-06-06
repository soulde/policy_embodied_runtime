"""Common transport primitives."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True, slots=True)
class TransportFrame:
    """One frame on an external robot I/O channel."""

    id: int
    payload: bytes


class Transport(Protocol):
    """External robot I/O transport."""

    def open(self) -> None:
        """Open the transport."""

    def close(self) -> None:
        """Close the transport."""

    def send(self, frame: TransportFrame) -> None:
        """Send a frame."""

    def receive(self) -> TransportFrame | None:
        """Receive one frame if available."""
