"""JSON codec for policy protocol envelopes."""

from __future__ import annotations

import json
from typing import Any

from pydantic import ValidationError

from policy_embodied_runtime.protocol.errors import ProtocolError
from policy_embodied_runtime.protocol.messages import MessageEnvelope


def encode_envelope(envelope: MessageEnvelope) -> bytes:
    """Encode a validated message envelope into UTF-8 JSON bytes."""
    return envelope.model_dump_json(by_alias=True).encode("utf-8")


def decode_envelope(raw_message: bytes) -> MessageEnvelope:
    """Decode UTF-8 JSON bytes into a validated message envelope."""
    try:
        data: dict[str, Any] = json.loads(raw_message.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError("invalid JSON message") from exc
    try:
        return MessageEnvelope.model_validate(data)
    except ValidationError as exc:
        raise ProtocolError("invalid message envelope") from exc
