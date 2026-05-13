from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass, field
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np

from vcp.logging.signal_dictionary import (
    SignalDefinition,
    default_signal_dictionary,
    validate_required_signals,
)


class MF4ExportUnavailable(RuntimeError):
    """Raised when MF4 export is requested without the optional asammdf package."""


@dataclass(frozen=True)
class SignalLogMetadata:
    """Metadata sidecar for traceable signal logs."""

    stage: str
    run_id: str
    controller: str
    scenario_id: str
    project: str = "vmodel-control-pipeline"
    generated_at_utc: str = field(default_factory=lambda: datetime.now(UTC).isoformat())
    git_commit: str | None = None
    sample_time_s: float | None = None
    extra: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable metadata dictionary."""

        return asdict(self)


@dataclass(frozen=True)
class SignalLogArtifacts:
    """Paths produced by one signal logging call."""

    csv: Path
    metadata_json: Path
    mf4: Path | None = None


class SignalLogWriter:
    """Write controller measurement logs with signal metadata."""

    def __init__(
        self,
        output_dir: Path,
        *,
        signal_dictionary: dict[str, SignalDefinition] | None = None,
        metadata: SignalLogMetadata,
    ) -> None:
        self.output_dir = output_dir
        self.signal_dictionary = signal_dictionary or default_signal_dictionary()
        self.metadata = metadata

    def write(
        self,
        rows: list[dict[str, Any]],
        *,
        stem: str = "controller_signals",
        export_mf4: bool = False,
    ) -> SignalLogArtifacts:
        """Write a CSV log, metadata JSON, and optionally an MF4 log."""

        validate_required_signals(rows, self.signal_dictionary)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = self.output_dir / f"{stem}.csv"
        metadata_path = self.output_dir / f"{stem}_metadata.json"
        write_signal_log_csv(rows, csv_path, self.signal_dictionary)
        metadata_path.write_text(
            json.dumps(self.metadata.to_dict(), indent=2, sort_keys=True),
            encoding="utf-8",
        )
        mf4_path = self.write_mf4(rows, self.output_dir / f"{stem}.mf4") if export_mf4 else None
        return SignalLogArtifacts(csv=csv_path, metadata_json=metadata_path, mf4=mf4_path)

    def write_mf4(self, rows: list[dict[str, Any]], output_path: Path) -> Path:
        """Export numeric signals to ASAM MDF/MF4 if asammdf is installed."""

        try:
            from asammdf import MDF, Signal
        except ImportError as exc:  # pragma: no cover - depends on optional dependency
            raise MF4ExportUnavailable(
                "MF4 export requires optional dependency 'asammdf'. "
                "Install with: pip install asammdf"
            ) from exc

        validate_required_signals(rows, self.signal_dictionary)
        timestamps = np.asarray([float(row["time_s"]) for row in rows], dtype=np.float64)
        mdf = MDF()
        numeric_signals = []
        for name, definition in self.signal_dictionary.items():
            values = _numeric_values(rows, name)
            if values is None:
                continue
            numeric_signals.append(
                Signal(
                    samples=values,
                    timestamps=timestamps,
                    name=name,
                    unit=definition.unit,
                    comment=definition.description,
                )
            )
        if numeric_signals:
            mdf.append(numeric_signals)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        mdf.save(output_path, overwrite=True)
        return output_path


def write_signal_log_csv(
    rows: list[dict[str, Any]],
    output_path: Path,
    signal_dictionary: dict[str, SignalDefinition] | None = None,
) -> None:
    """Write rows to CSV with dictionary signals first and extras afterward."""

    if not rows:
        raise ValueError("rows must not be empty")
    signal_dictionary = signal_dictionary or default_signal_dictionary()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = _fieldnames(rows, signal_dictionary)
    with output_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def read_signal_log_csv(path: Path) -> list[dict[str, Any]]:
    """Read a signal log CSV and coerce numeric values where possible."""

    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        return [{key: _coerce_value(value) for key, value in row.items()} for row in reader]


def _fieldnames(
    rows: list[dict[str, Any]],
    signal_dictionary: dict[str, SignalDefinition],
) -> list[str]:
    ordered = list(signal_dictionary.keys())
    extras = sorted({key for row in rows for key in row if key not in signal_dictionary})
    return ordered + extras


def _numeric_values(rows: list[dict[str, Any]], name: str) -> np.ndarray | None:
    values: list[float] = []
    for row in rows:
        value = row.get(name)
        if isinstance(value, bool) or value is None:
            return None
        try:
            values.append(float(value))
        except (TypeError, ValueError):
            return None
    return np.asarray(values, dtype=np.float64)


def _coerce_value(value: str | None) -> Any:
    if value is None:
        return ""
    if value == "":
        return ""
    lowered = value.lower()
    if lowered in {"true", "false"}:
        return lowered == "true"
    try:
        if any(marker in value for marker in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value


__all__ = [
    "MF4ExportUnavailable",
    "SignalLogArtifacts",
    "SignalLogMetadata",
    "SignalLogWriter",
    "read_signal_log_csv",
    "write_signal_log_csv",
]
