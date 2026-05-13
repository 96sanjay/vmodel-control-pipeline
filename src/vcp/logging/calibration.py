from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class CalibrationError(ValueError):
    """Raised when calibration parameters are malformed or incomplete."""


@dataclass(frozen=True)
class CalibrationBundle:
    """YAML-backed parameter bundle used for controller and safety tuning."""

    parameters: dict[str, Any]
    source_path: Path

    def get(self, dotted_key: str, default: Any = None) -> Any:
        """Read a nested calibration value with dot notation."""

        cursor: Any = self.parameters
        for part in dotted_key.split("."):
            if not isinstance(cursor, dict) or part not in cursor:
                return default
            cursor = cursor[part]
        return cursor

    def require(self, dotted_key: str) -> Any:
        """Read a nested value and fail if it is missing."""

        sentinel = object()
        value = self.get(dotted_key, sentinel)
        if value is sentinel:
            raise CalibrationError(f"Missing required calibration key: {dotted_key}")
        return value


def load_calibration(path: Path) -> CalibrationBundle:
    """Load a calibration YAML file."""

    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise CalibrationError(f"Calibration file must contain a mapping: {path}")
    return CalibrationBundle(parameters=payload, source_path=path)


def merge_calibration(
    base: CalibrationBundle,
    override: CalibrationBundle,
) -> CalibrationBundle:
    """Merge two calibration bundles, with override values taking precedence."""

    return CalibrationBundle(
        parameters=_deep_merge(base.parameters, override.parameters),
        source_path=override.source_path,
    )


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


__all__ = [
    "CalibrationBundle",
    "CalibrationError",
    "load_calibration",
    "merge_calibration",
]
