"""Configuration loading with explicit defaults and deep merge."""

from __future__ import annotations

import copy
import os
import re
from pathlib import Path
from typing import Any

import yaml


DEFAULT_CONFIG_PATH = Path(__file__).with_name("config.yaml")
_ENV_PATTERN = re.compile(r"^\$\{([A-Za-z_][A-Za-z0-9_]*)(?::-([^}]*))?\}$")


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _expand_environment(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _expand_environment(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_expand_environment(item) for item in value]
    if isinstance(value, str):
        match = _ENV_PATTERN.match(value)
        if match:
            name, fallback = match.groups()
            return os.environ.get(name, fallback if fallback is not None else "")
    return value


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    """Load the bundled config and optionally overlay another YAML file."""

    with DEFAULT_CONFIG_PATH.open("r", encoding="utf-8") as handle:
        base = yaml.safe_load(handle) or {}
    if path is None or Path(path).resolve() == DEFAULT_CONFIG_PATH.resolve():
        return _expand_environment(base)
    with Path(path).open("r", encoding="utf-8") as handle:
        override = yaml.safe_load(handle) or {}
    return _expand_environment(_deep_merge(base, override))


def project_root() -> Path:
    return Path(__file__).resolve().parent.parent

