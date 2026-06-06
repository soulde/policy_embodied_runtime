"""Device protocol abstractions."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from policy_embodied_runtime.transport import Transport, TransportFrame


@dataclass(frozen=True, slots=True)
class ProtocolFrame:
    """A protocol-level frame."""

    id: int
    payload: bytes


@dataclass(frozen=True, slots=True)
class DeviceCommand:
    """Command addressed to one device."""

    device_id: int
    payload: bytes


@dataclass(frozen=True, slots=True)
class DeviceReading:
    """Reading returned by one device."""

    device_id: int
    payload: bytes


class ProtocolCodec(Protocol):
    """Encode/decode device protocol frames."""

    def encode_command(self, command: DeviceCommand) -> ProtocolFrame:
        """Encode a device command."""

    def decode_reading(self, frame: ProtocolFrame) -> DeviceReading:
        """Decode a device reading."""


class ProtocolTransport:
    """Bridge a device protocol codec onto a low-level transport."""

    def __init__(self, protocol: ProtocolCodec, transport: Transport) -> None:
        self.protocol = protocol
        self.transport = transport

    def open(self) -> None:
        """Open the underlying transport."""
        self.transport.open()

    def close(self) -> None:
        """Close the underlying transport."""
        self.transport.close()

    def send_command(self, command: DeviceCommand) -> None:
        """Encode and send a device command."""
        frame = self.protocol.encode_command(command)
        self.transport.send(TransportFrame(id=frame.id, payload=frame.payload))

    def receive_reading(self) -> DeviceReading | None:
        """Receive and decode one device reading if available."""
        frame = self.transport.receive()
        if frame is None:
            return None
        return self.protocol.decode_reading(ProtocolFrame(id=frame.id, payload=frame.payload))
