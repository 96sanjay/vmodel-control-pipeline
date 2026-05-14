from __future__ import annotations

import json
import struct
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

CONTROLLER_STATUS_MESSAGE_ID = 0x120

CONTROLLER_MODE_CODES = {
    "NORMAL": 0,
    "DEGRADED": 1,
    "FALLBACK_BRAKE": 2,
    "EMERGENCY_STOP": 3,
    "ESTIMATOR_FAULT": 4,
    "SOLVER_TIMEOUT": 5,
}

FALLBACK_REASON_CODES = {
    "no_fault": 0,
    "solver_infeasible": 1,
    "solver_timeout": 2,
    "estimator_residual": 3,
    "invalid_command": 4,
    "collision_risk": 5,
    "missing_sensor_message": 6,
    "communication_timeout": 7,
    "invalid_measurement": 8,
}


class VirtualCANError(RuntimeError):
    """Raised when a virtual CAN operation cannot be completed."""


class VirtualCANDependencyError(VirtualCANError):
    """Raised when optional CAN tooling is requested but not installed."""


@dataclass(frozen=True)
class CANFrame:
    """Small dependency-free representation of a CAN data frame."""

    arbitration_id: int
    data: bytes
    timestamp_s: float = 0.0
    is_extended_id: bool = False

    def __post_init__(self) -> None:
        if self.arbitration_id < 0:
            raise ValueError("arbitration_id must be non-negative")
        if len(self.data) > 8 and not self.is_extended_id:
            raise ValueError("classic CAN frame data must be at most 8 bytes")

    def to_json_dict(self) -> dict[str, Any]:
        """Return a JSON-serializable frame dictionary."""

        payload = asdict(self)
        payload["data"] = self.data.hex()
        return payload


def encode_controller_status_frame(
    row: dict[str, Any],
    *,
    arbitration_id: int = CONTROLLER_STATUS_MESSAGE_ID,
) -> CANFrame:
    """Encode controller status signals into one deterministic 8-byte CAN frame."""

    acceleration_raw = _scale_signed(row["acceleration_cmd"], factor=0.001, bits=16)
    steering_raw = _scale_signed(row["steering_cmd"], factor=0.0001, bits=16)
    solve_time_raw = _scale_unsigned(row["solve_time_ms"], factor=0.01, bits=16)
    mode_code = _enum_code(row.get("controller_mode", "NORMAL"), CONTROLLER_MODE_CODES)
    fallback_code = _enum_code(row.get("fallback_reason", "no_fault"), FALLBACK_REASON_CODES)
    data = struct.pack(
        "<hhHBB",
        acceleration_raw,
        steering_raw,
        solve_time_raw,
        mode_code,
        fallback_code,
    )
    return CANFrame(
        arbitration_id=arbitration_id,
        data=data,
        timestamp_s=float(row.get("time_s", 0.0)),
    )


def decode_controller_status_frame(frame: CANFrame) -> dict[str, float | int | str]:
    """Decode a frame produced by encode_controller_status_frame."""

    if frame.arbitration_id != CONTROLLER_STATUS_MESSAGE_ID:
        raise VirtualCANError(f"Unexpected controller-status frame ID: {frame.arbitration_id:#x}")
    if len(frame.data) != 8:
        raise VirtualCANError(
            f"Controller-status frame must contain 8 bytes, got {len(frame.data)}"
        )

    acceleration_raw, steering_raw, solve_time_raw, mode_code, fallback_code = struct.unpack(
        "<hhHBB",
        frame.data,
    )
    return {
        "time_s": frame.timestamp_s,
        "acceleration_cmd": acceleration_raw * 0.001,
        "steering_cmd": steering_raw * 0.0001,
        "solve_time_ms": solve_time_raw * 0.01,
        "controller_mode": _enum_name(mode_code, CONTROLLER_MODE_CODES),
        "fallback_reason": _enum_name(fallback_code, FALLBACK_REASON_CODES),
    }


def rows_to_controller_status_frames(rows: list[dict[str, Any]]) -> tuple[CANFrame, ...]:
    """Encode each signal-log row as a controller-status CAN frame."""

    return tuple(encode_controller_status_frame(row) for row in rows)


def write_can_frames_jsonl(
    frames: list[CANFrame] | tuple[CANFrame, ...],
    output_path: Path,
) -> None:
    """Write virtual CAN frames as JSON lines for replay or inspection."""

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as file:
        for frame in frames:
            file.write(json.dumps(frame.to_json_dict(), sort_keys=True) + "\n")


def load_dbc(path: Path) -> Any:
    """Load a DBC file with cantools when optional CAN dependencies are installed."""

    try:
        import cantools
    except ImportError as exc:  # pragma: no cover - depends on optional dependency
        raise VirtualCANDependencyError(
            "DBC loading requires optional dependency 'cantools'. "
            "Install with: pip install cantools"
        ) from exc

    return cantools.database.load_file(path)


def encode_controller_status_with_dbc(row: dict[str, Any], dbc_path: Path) -> CANFrame:
    """Encode the controller status message using the configured DBC file."""

    database = load_dbc(dbc_path)
    message = database.get_message_by_name("VCP_ControllerStatus")
    data = message.encode(
        {
            "AccelCmd": float(row["acceleration_cmd"]),
            "SteeringCmd": float(row["steering_cmd"]),
            "SolveTimeMs": float(row["solve_time_ms"]),
            "ControllerMode": _enum_code(
                row.get("controller_mode", "NORMAL"),
                CONTROLLER_MODE_CODES,
            ),
            "FallbackReason": _enum_code(
                row.get("fallback_reason", "no_fault"),
                FALLBACK_REASON_CODES,
            ),
        }
    )
    return CANFrame(
        arbitration_id=message.frame_id,
        data=data,
        timestamp_s=float(row.get("time_s", 0.0)),
        is_extended_id=message.is_extended_frame,
    )


def frame_to_python_can(frame: CANFrame) -> Any:
    """Convert a dependency-free frame into a python-can Message."""

    try:
        import can
    except ImportError as exc:  # pragma: no cover - depends on optional dependency
        raise VirtualCANDependencyError(
            "python-can conversion requires optional dependency 'python-can'. "
            "Install with: pip install python-can"
        ) from exc

    return can.Message(
        arbitration_id=frame.arbitration_id,
        data=frame.data,
        is_extended_id=frame.is_extended_id,
        timestamp=frame.timestamp_s,
    )


def _scale_signed(value: Any, *, factor: float, bits: int) -> int:
    raw = round(float(value) / factor)
    lower = -(2 ** (bits - 1))
    upper = 2 ** (bits - 1) - 1
    return min(max(raw, lower), upper)


def _scale_unsigned(value: Any, *, factor: float, bits: int) -> int:
    raw = round(float(value) / factor)
    lower = 0
    upper = 2**bits - 1
    return min(max(raw, lower), upper)


def _enum_code(value: Any, mapping: dict[str, int]) -> int:
    if isinstance(value, int):
        return value
    return mapping.get(str(value), 255)


def _enum_name(value: int, mapping: dict[str, int]) -> str:
    reverse_mapping = {code: name for name, code in mapping.items()}
    return reverse_mapping.get(value, f"UNKNOWN_{value}")


__all__ = [
    "CONTROLLER_MODE_CODES",
    "CONTROLLER_STATUS_MESSAGE_ID",
    "FALLBACK_REASON_CODES",
    "CANFrame",
    "VirtualCANDependencyError",
    "VirtualCANError",
    "decode_controller_status_frame",
    "encode_controller_status_frame",
    "encode_controller_status_with_dbc",
    "frame_to_python_can",
    "load_dbc",
    "rows_to_controller_status_frames",
    "write_can_frames_jsonl",
]
