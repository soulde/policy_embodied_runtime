"""Global auto-registration for adapters and processors."""

from __future__ import annotations

import re
from abc import ABCMeta
from collections import defaultdict
from typing import Any


_GLOBAL_REGISTRIES: dict[str, dict[str, type[object]]] = defaultdict(dict)


class AutoRegisteringMeta(ABCMeta):
    """Metaclass that auto-registers concrete subclasses in a global registry."""

    def __new__(
        mcls,
        name: str,
        bases: tuple[type[object], ...],
        namespace: dict[str, Any],
        **kwargs: Any,
    ) -> type[object]:
        cls = super().__new__(mcls, name, bases, namespace, **kwargs)
        category = getattr(cls, "__registry_category__", None)
        entry_name = getattr(cls, "__registry_name__", None) or _to_snake_case(name)
        if category and entry_name and not getattr(cls, "__abstractmethods__", False):
            registry = _GLOBAL_REGISTRIES[category]
            if entry_name in registry and registry[entry_name] is not cls:
                raise ValueError(f"{category} already registered: {entry_name}")
            registry[entry_name] = cls
        return cls


def get_registry_entries(category: str) -> dict[str, type[object]]:
    """Return registered entries for a category."""
    return dict(_GLOBAL_REGISTRIES.get(category, {}))


def create_registered(category: str, name: str, **kwargs: Any) -> object:
    """Instantiate a registered class by category and name."""
    try:
        entry_type = _GLOBAL_REGISTRIES[category][name]
    except KeyError as exc:
        raise KeyError(f"{category} not found: {name}") from exc
    return entry_type(**kwargs)


def _to_snake_case(name: str) -> str:
    """Convert a class name into a default registry key."""
    first_pass = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", first_pass).lower()
