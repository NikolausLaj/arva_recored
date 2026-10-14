"""YAML helpers used by the test protocol command-line tools."""

from __future__ import annotations

from pathlib import Path
from typing import Any


class ProtocolError(RuntimeError):
    """Raised when a run configuration or protocol action is invalid."""


def _yaml_module() -> Any:
    try:
        import yaml
    except ImportError as error:
        raise ProtocolError(
            'PyYAML is required. Install python3-yaml in the ROS environment.'
        ) from error
    return yaml


def load_yaml(path: Path) -> dict[str, Any]:
    """Load a mapping from a YAML file."""
    if not path.is_file():
        raise ProtocolError(f'YAML file does not exist: {path}')

    yaml = _yaml_module()
    with path.open(encoding='utf-8') as file_handle:
        data = yaml.safe_load(file_handle)

    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ProtocolError(f'YAML document must contain a mapping: {path}')
    return data


def dump_yaml(path: Path, data: dict[str, Any]) -> None:
    """Write a mapping as readable YAML, creating its parent if needed."""
    yaml = _yaml_module()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as file_handle:
        yaml.safe_dump(data, file_handle, sort_keys=False, default_flow_style=False)


def merge_mappings(
    base: dict[str, Any], override: dict[str, Any]
) -> dict[str, Any]:
    """Recursively merge an override mapping without changing either input."""
    merged = dict(base)
    for key, value in override.items():
        existing_value = merged.get(key)
        if isinstance(existing_value, dict) and isinstance(value, dict):
            merged[key] = merge_mappings(existing_value, value)
        else:
            merged[key] = value
    return merged