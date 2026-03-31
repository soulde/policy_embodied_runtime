"""Basic validation checks for canonical observations."""

from __future__ import annotations

from typing import Any


def validate_required_fields(canonical_obs: dict[str, Any], required_fields: list[str]) -> None:
    """Ensure required canonical observation fields are present."""
    missing_fields = [field for field in required_fields if field not in canonical_obs]
    if missing_fields:
        raise ValueError(f"missing required canonical observation fields: {missing_fields}")
