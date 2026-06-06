"""PySerial transport implementation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import serial

from policy_embodied_runtime.transport.base import TransportFrame


@dataclass(frozen=True, slots=True)
class SerialConfig:
    """Serial port configuration."""

    path: str
    baud_rate: int
    read_buffer_len: int = 256
    timeout_s: float = 0.0
    write_timeout_s: float | None = None

    @classmethod
    def from_args(cls, args: dict[str, str]) -> "SerialConfig":
        """Build serial config from robot profile args."""
        path = args.get("path") or args.get("device")
        if path is None:
            raise ValueError("missing serial path/device arg")
        try:
            baud_rate = int(args["baud_rate"])
        except KeyError as exc:
            raise ValueError("missing serial arg 'baud_rate'") from exc
        except ValueError as exc:
            raise ValueError(f"invalid serial arg 'baud_rate' value '{args['baud_rate']}'") from exc
        return cls(
            path=str(Path(path)),
            baud_rate=baud_rate,
            read_buffer_len=_int_arg(args, "read_buffer_len", 256),
            timeout_s=_float_arg(args, "timeout_s", _float_arg(args, "timeout", 0.0)),
            write_timeout_s=_optional_float_arg(args, "write_timeout_s"),
        )


class SerialTransport:
    """Transport backed by a pyserial port."""

    def __init__(self, config: SerialConfig) -> None:
        self.config = config
        self._serial: serial.Serial | None = None

    @property
    def is_open(self) -> bool:
        """Whether the serial port is open."""
        return self._serial is not None and self._serial.is_open

    def open(self) -> None:
        """Open the serial port."""
        if self.is_open:
            return
        self._serial = serial.Serial(
            port=self.config.path,
            baudrate=self.config.baud_rate,
            timeout=self.config.timeout_s,
            write_timeout=self.config.write_timeout_s,
        )

    def close(self) -> None:
        """Close the serial port."""
        if self._serial is not None:
            self._serial.close()
        self._serial = None

    def send(self, frame: TransportFrame) -> None:
        """Write frame payload bytes to the serial port."""
        port = self._require_open()
        port.write(frame.payload)
        port.flush()

    def receive(self) -> TransportFrame | None:
        """Read available bytes from the serial port."""
        port = self._require_open()
        waiting = getattr(port, "in_waiting", 0)
        read_len = waiting if waiting > 0 else self.config.read_buffer_len
        payload = port.read(read_len)
        if not payload:
            return None
        return TransportFrame(id=0, payload=bytes(payload))

    def _require_open(self) -> serial.Serial:
        if self._serial is None or not self._serial.is_open:
            raise RuntimeError("serial port is not open")
        return self._serial


def _int_arg(args: dict[str, str], key: str, default: int) -> int:
    value = args.get(key)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError as exc:
        raise ValueError(f"invalid serial arg '{key}' value '{value}'") from exc


def _float_arg(args: dict[str, str], key: str, default: float) -> float:
    value = args.get(key)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError as exc:
        raise ValueError(f"invalid serial arg '{key}' value '{value}'") from exc


def _optional_float_arg(args: dict[str, str], key: str) -> float | None:
    value = args.get(key)
    if value is None:
        return None
    try:
        return float(value)
    except ValueError as exc:
        raise ValueError(f"invalid serial arg '{key}' value '{value}'") from exc
