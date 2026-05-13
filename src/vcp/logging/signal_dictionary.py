from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class SignalDefinition:
    """Engineering metadata for one logged controller signal."""

    name: str
    unit: str
    description: str
    sample_rate_hz: float
    required: bool = True

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("signal name must not be empty")
        if self.sample_rate_hz <= 0.0:
            raise ValueError("sample_rate_hz must be positive")


DEFAULT_SIGNAL_DEFINITIONS: tuple[SignalDefinition, ...] = (
    SignalDefinition("time_s", "s", "Experiment time stamp.", 10.0),
    SignalDefinition("px", "m", "Measured ego x position.", 10.0),
    SignalDefinition("py", "m", "Measured ego y position.", 10.0),
    SignalDefinition("yaw", "rad", "Measured ego heading angle.", 10.0),
    SignalDefinition("v", "m/s", "Measured ego speed.", 10.0),
    SignalDefinition("px_est", "m", "Estimated ego x position.", 10.0),
    SignalDefinition("py_est", "m", "Estimated ego y position.", 10.0),
    SignalDefinition("yaw_est", "rad", "Estimated ego heading angle.", 10.0),
    SignalDefinition("v_est", "m/s", "Estimated ego speed.", 10.0),
    SignalDefinition("acceleration_cmd", "m/s^2", "Requested longitudinal acceleration.", 10.0),
    SignalDefinition("steering_cmd", "rad", "Requested steering angle.", 10.0),
    SignalDefinition("controller_mode", "enum", "Controller or safety-supervisor mode.", 10.0),
    SignalDefinition("solver_status", "enum", "Optimization solver status.", 10.0),
    SignalDefinition("solve_time_ms", "ms", "Controller compute time.", 10.0),
    SignalDefinition("fallback_reason", "enum", "Reason code for fallback activation.", 10.0),
    SignalDefinition("lateral_error", "m", "Lateral tracking error.", 10.0),
    SignalDefinition("heading_error", "rad", "Heading tracking error.", 10.0),
)


def default_signal_dictionary() -> dict[str, SignalDefinition]:
    """Return the default signal dictionary keyed by signal name."""

    return {definition.name: definition for definition in DEFAULT_SIGNAL_DEFINITIONS}


def required_signal_names(
    definitions: dict[str, SignalDefinition] | None = None,
) -> tuple[str, ...]:
    """Return all required signal names in dictionary order."""

    definitions = definitions or default_signal_dictionary()
    return tuple(name for name, definition in definitions.items() if definition.required)


def validate_required_signals(
    rows: list[dict[str, Any]],
    definitions: dict[str, SignalDefinition] | None = None,
) -> None:
    """Validate that each row contains all required logging signals."""

    if not rows:
        raise ValueError("rows must not be empty")

    missing_by_row: dict[int, list[str]] = {}
    required_names = required_signal_names(definitions)
    for index, row in enumerate(rows):
        missing = [name for name in required_names if name not in row]
        if missing:
            missing_by_row[index] = missing

    if missing_by_row:
        first_row = min(missing_by_row)
        missing = ", ".join(missing_by_row[first_row])
        raise ValueError(f"log row {first_row} is missing required signals: {missing}")


def load_signal_dictionary(path: Path) -> dict[str, SignalDefinition]:
    """Load signal definitions from a YAML file."""

    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"Signal dictionary must be a mapping: {path}")

    raw_signals = payload.get("signals")
    if not isinstance(raw_signals, list):
        raise ValueError("Signal dictionary YAML must contain a 'signals' list")

    definitions: dict[str, SignalDefinition] = {}
    for raw_signal in raw_signals:
        if not isinstance(raw_signal, dict):
            raise ValueError("Each signal definition must be a mapping")
        definition = SignalDefinition(
            name=str(raw_signal["name"]),
            unit=str(raw_signal.get("unit", "")),
            description=str(raw_signal.get("description", "")),
            sample_rate_hz=float(raw_signal.get("sample_rate_hz", 10.0)),
            required=bool(raw_signal.get("required", True)),
        )
        definitions[definition.name] = definition
    return definitions


def dump_signal_dictionary(definitions: dict[str, SignalDefinition], path: Path) -> None:
    """Write signal definitions to YAML for review or tool handoff."""

    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {"signals": [asdict(definition) for definition in definitions.values()]}
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


__all__ = [
    "DEFAULT_SIGNAL_DEFINITIONS",
    "SignalDefinition",
    "default_signal_dictionary",
    "dump_signal_dictionary",
    "load_signal_dictionary",
    "required_signal_names",
    "validate_required_signals",
]
