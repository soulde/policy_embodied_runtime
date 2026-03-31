"""Builders for common protocol messages."""

from __future__ import annotations

import time
import uuid
from typing import Any

from policy_embodied_runtime.server.schemas.messages import MessageEnvelope


def build_envelope(
    message_type: str,
    *,
    session_id: str,
    payload: dict[str, Any],
    request_id: str | None = None,
    step_id: int = 0,
) -> MessageEnvelope:
    """Build a standard request envelope."""
    return MessageEnvelope(
        schema="embodied-policy-runtime/v1alpha1",
        type=message_type,
        request_id=request_id or str(uuid.uuid4()),
        session_id=session_id,
        step_id=step_id,
        timestamp_ns=time.time_ns(),
        payload=payload,
    )
