from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from vcp.logging import SignalLogMetadata, SignalLogWriter
from vcp.logging.virtual_can import (
    VirtualCANDependencyError,
    decode_controller_status_frame,
    encode_controller_status_frame,
    encode_controller_status_with_dbc,
    load_dbc,
    rows_to_controller_status_frames,
    write_can_frames_jsonl,
)


def test_controller_status_can_frame_round_trip() -> None:
    frame = encode_controller_status_frame(_row())
    decoded = decode_controller_status_frame(frame)

    assert frame.arbitration_id == 0x120
    assert len(frame.data) == 8
    assert decoded["acceleration_cmd"] == 1.234
    assert decoded["steering_cmd"] == pytest.approx(-0.1234)
    assert decoded["solve_time_ms"] == 12.34
    assert decoded["controller_mode"] == "NORMAL"
    assert decoded["fallback_reason"] == "no_fault"


def test_rows_to_can_frames_and_jsonl_output(tmp_path: Path) -> None:
    frames = rows_to_controller_status_frames([_row(), {**_row(), "time_s": 0.1}])
    output_path = tmp_path / "frames.jsonl"

    write_can_frames_jsonl(frames, output_path)
    lines = output_path.read_text(encoding="utf-8").splitlines()
    payload = json.loads(lines[0])

    assert len(frames) == 2
    assert len(lines) == 2
    assert payload["arbitration_id"] == 0x120
    assert isinstance(payload["data"], str)


def test_dbc_load_and_encode_is_optional() -> None:
    dbc_path = Path("configs/hardware/vcp_controller.dbc")

    try:
        database = load_dbc(dbc_path)
        frame = encode_controller_status_with_dbc(_row(), dbc_path)
    except VirtualCANDependencyError:
        return

    assert database.get_message_by_name("VCP_ControllerStatus").frame_id == 0x120
    assert frame.arbitration_id == 0x120
    assert len(frame.data) == 8


def test_virtual_can_replay_script_generates_jsonl(tmp_path: Path) -> None:
    writer = SignalLogWriter(
        tmp_path,
        metadata=SignalLogMetadata(
            stage="MIL",
            run_id="virtual-can-test",
            controller="pid",
            scenario_id="synthetic",
        ),
    )
    artifacts = writer.write([_signal_log_row()], stem="signals")
    output_path = tmp_path / "frames.jsonl"

    result = subprocess.run(
        [
            sys.executable,
            "scripts/replay_signal_log_to_virtual_can.py",
            str(artifacts.csv),
            "--output",
            str(output_path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    assert "Wrote 1 virtual CAN frame" in result.stdout
    assert output_path.exists()


def test_decode_rejects_wrong_frame_id() -> None:
    frame = encode_controller_status_frame(_row(), arbitration_id=0x121)

    with pytest.raises(Exception, match="Unexpected controller-status frame ID"):
        decode_controller_status_frame(frame)


def _row() -> dict[str, object]:
    return {
        "time_s": 0.0,
        "acceleration_cmd": 1.234,
        "steering_cmd": -0.1234,
        "solve_time_ms": 12.34,
        "controller_mode": "NORMAL",
        "fallback_reason": "no_fault",
    }


def _signal_log_row() -> dict[str, object]:
    return {
        "time_s": 0.0,
        "px": 0.0,
        "py": 0.0,
        "yaw": 0.0,
        "v": 1.0,
        "px_est": 0.0,
        "py_est": 0.0,
        "yaw_est": 0.0,
        "v_est": 1.0,
        "acceleration_cmd": 1.234,
        "steering_cmd": -0.1234,
        "controller_mode": "NORMAL",
        "solver_status": "optimal",
        "solve_time_ms": 12.34,
        "fallback_reason": "no_fault",
        "lateral_error": 0.0,
        "heading_error": 0.0,
    }
