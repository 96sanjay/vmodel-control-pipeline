from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from typing import Any, Literal

from vcp.controllers import PathTrackingTarget
from vcp.models import VehicleInput, VehicleState
from vcp.validation import ControllerStepInput

MessageKind = Literal["measurement", "command"]


class HILProtocolError(ValueError):
    """Raised when a HIL-lite datagram is malformed."""


@dataclass(frozen=True)
class HILMeasurement:
    """Plant-to-controller measurement packet."""

    sequence_id: int
    timestamp_s: float
    dt: float
    px: float
    py: float
    yaw: float
    v: float
    target_speed: float
    target_lateral_position: float
    target_heading: float
    invalid: bool = False

    def to_step_input(self) -> ControllerStepInput:
        if self.invalid:
            raise HILProtocolError("measurement is marked invalid")
        return ControllerStepInput(
            timestamp_s=self.timestamp_s,
            dt=self.dt,
            state=VehicleState(px=self.px, py=self.py, yaw=self.yaw, v=self.v),
            target=PathTrackingTarget(
                speed=self.target_speed,
                lateral_position=self.target_lateral_position,
                heading=self.target_heading,
            ),
        )


@dataclass(frozen=True)
class HILCommand:
    """Controller-to-plant command packet."""

    sequence_id: int
    timestamp_s: float
    acceleration: float
    steering_angle: float
    controller_mode: str
    solver_status: str
    solve_time_ms: float
    fallback_active: bool
    fallback_reason: str

    def to_vehicle_input(self) -> VehicleInput:
        return VehicleInput(
            acceleration=self.acceleration,
            steering_angle=self.steering_angle,
        )


def encode_message(kind: MessageKind, payload: HILMeasurement | HILCommand) -> bytes:
    """Encode a HIL-lite message as UTF-8 JSON bytes."""

    return json.dumps(
        {
            "kind": kind,
            "payload": asdict(payload),
        },
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def decode_measurement(data: bytes) -> HILMeasurement:
    message = _decode_message(data, expected_kind="measurement")
    return HILMeasurement(**message["payload"])


def decode_command(data: bytes) -> HILCommand:
    message = _decode_message(data, expected_kind="command")
    return HILCommand(**message["payload"])


def _decode_message(data: bytes, *, expected_kind: MessageKind) -> dict[str, Any]:
    try:
        raw = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise HILProtocolError("datagram is not valid UTF-8 JSON") from exc

    if not isinstance(raw, dict):
        raise HILProtocolError("datagram root must be a JSON object")
    if raw.get("kind") != expected_kind:
        raise HILProtocolError(f"expected message kind '{expected_kind}'")
    payload = raw.get("payload")
    if not isinstance(payload, dict):
        raise HILProtocolError("message payload must be a JSON object")
    return {"kind": raw["kind"], "payload": payload}


__all__ = [
    "HILCommand",
    "HILMeasurement",
    "HILProtocolError",
    "decode_command",
    "decode_measurement",
    "encode_message",
]
