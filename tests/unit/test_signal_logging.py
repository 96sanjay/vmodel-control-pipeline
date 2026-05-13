from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from vcp.logging import (
    CalibrationError,
    MF4ExportUnavailable,
    SignalLogMetadata,
    SignalLogWriter,
    load_calibration,
    load_signal_dictionary,
    read_signal_log_csv,
    validate_required_signals,
)


def test_signal_log_writer_creates_csv_and_metadata(tmp_path: Path) -> None:
    writer = SignalLogWriter(
        tmp_path,
        metadata=SignalLogMetadata(
            stage="MIL",
            run_id="unit-test",
            controller="pid",
            scenario_id="synthetic",
            sample_time_s=0.1,
        ),
    )

    artifacts = writer.write(_sample_rows())
    rows = read_signal_log_csv(artifacts.csv)
    metadata = json.loads(artifacts.metadata_json.read_text(encoding="utf-8"))

    assert artifacts.csv.exists()
    assert artifacts.metadata_json.exists()
    assert rows[0]["time_s"] == 0
    assert rows[1]["controller_mode"] == "NORMAL"
    assert metadata["stage"] == "MIL"
    assert metadata["controller"] == "pid"


def test_missing_required_signal_fails_fast() -> None:
    rows = _sample_rows()
    rows[0].pop("solve_time_ms")

    with pytest.raises(ValueError, match="solve_time_ms"):
        validate_required_signals(rows)


def test_signal_dictionary_yaml_round_trip() -> None:
    definitions = load_signal_dictionary(Path("configs/hardware/signal_dictionary.yaml"))

    assert "time_s" in definitions
    assert definitions["acceleration_cmd"].unit == "m/s^2"
    assert definitions["controller_mode"].required


def test_calibration_loader_reads_nested_values() -> None:
    calibration = load_calibration(Path("configs/controllers/default_calibration.yaml"))

    assert calibration.require("controller.horizon") == 20
    assert calibration.get("safety.max_solve_time_ms") == 80.0
    assert calibration.get("missing.key", "fallback") == "fallback"


def test_calibration_require_raises_for_missing_key() -> None:
    calibration = load_calibration(Path("configs/controllers/default_calibration.yaml"))

    with pytest.raises(CalibrationError, match="Missing required calibration key"):
        calibration.require("controller.not_present")


def test_mf4_export_is_optional(tmp_path: Path) -> None:
    writer = SignalLogWriter(
        tmp_path,
        metadata=SignalLogMetadata(
            stage="MIL",
            run_id="unit-test",
            controller="pid",
            scenario_id="synthetic",
        ),
    )

    try:
        path = writer.write_mf4(_sample_rows(), tmp_path / "signals.mf4")
    except MF4ExportUnavailable:
        return

    assert path.exists()


def test_replay_signal_log_script_generates_summary(tmp_path: Path) -> None:
    writer = SignalLogWriter(
        tmp_path,
        metadata=SignalLogMetadata(
            stage="MIL",
            run_id="replay-test",
            controller="pid",
            scenario_id="synthetic",
        ),
    )
    artifacts = writer.write(_sample_rows(), stem="replay_input")
    output_dir = tmp_path / "replay"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/replay_signal_log.py",
            str(artifacts.csv),
            "--output-dir",
            str(output_dir),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    summary = json.loads((output_dir / "signal_log_summary.json").read_text(encoding="utf-8"))
    assert "Wrote replay summary" in result.stdout
    assert summary["row_count"] == 2
    assert (output_dir / "lateral_error.svg").exists()


def _sample_rows() -> list[dict[str, object]]:
    return [
        {
            "time_s": 0.0,
            "px": 0.0,
            "py": -0.2,
            "yaw": 0.0,
            "v": 1.0,
            "px_est": 0.0,
            "py_est": -0.18,
            "yaw_est": 0.01,
            "v_est": 1.02,
            "acceleration_cmd": 0.5,
            "steering_cmd": 0.1,
            "controller_mode": "NORMAL",
            "solver_status": "optimal",
            "solve_time_ms": 2.4,
            "fallback_reason": "none",
            "lateral_error": -0.2,
            "heading_error": 0.0,
        },
        {
            "time_s": 0.1,
            "px": 0.1,
            "py": -0.15,
            "yaw": 0.01,
            "v": 1.05,
            "px_est": 0.1,
            "py_est": -0.14,
            "yaw_est": 0.01,
            "v_est": 1.06,
            "acceleration_cmd": 0.4,
            "steering_cmd": 0.05,
            "controller_mode": "NORMAL",
            "solver_status": "optimal",
            "solve_time_ms": 2.1,
            "fallback_reason": "none",
            "lateral_error": -0.15,
            "heading_error": 0.01,
        },
    ]
