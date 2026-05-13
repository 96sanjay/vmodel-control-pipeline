from __future__ import annotations

from pathlib import Path

import pytest

import vcp.hil.plant_client as plant_client_module
from vcp.controllers import PID, PIDLimits, VehiclePIDController
from vcp.hil import (
    FailureInjectionConfig,
    HILControllerServer,
    HILLiteLoopConfig,
    HILMeasurement,
    HILPlantClient,
    HILTimeoutError,
    PlantClientConfig,
    decode_measurement,
    encode_message,
    run_hil_lite_loop,
    write_hil_lite_report,
)
from vcp.models import VehicleConstraints
from vcp.validation import PythonControllerAdapter


def test_hil_protocol_measurement_round_trip() -> None:
    measurement = HILMeasurement(
        sequence_id=7,
        timestamp_s=0.35,
        dt=0.05,
        px=1.0,
        py=-0.2,
        yaw=0.1,
        v=3.0,
        target_speed=4.0,
        target_lateral_position=0.0,
        target_heading=0.0,
    )

    decoded = decode_measurement(encode_message("measurement", measurement))

    assert decoded == measurement


def test_hil_plant_client_timeout_behavior(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(
        plant_client_module.socket,
        "socket",
        lambda *args, **kwargs: _TimeoutSocket(),
    )
    client = HILPlantClient(
        PlantClientConfig(
            server_port=49000,
            timeout_s=0.01,
        )
    )

    try:
        client.request_command(
            HILMeasurement(
                sequence_id=1,
                timestamp_s=0.0,
                dt=0.05,
                px=0.0,
                py=0.0,
                yaw=0.0,
                v=0.0,
                target_speed=4.0,
                target_lateral_position=0.0,
                target_heading=0.0,
            )
        )
    except HILTimeoutError as exc:
        assert "timed out" in str(exc)
    else:
        raise AssertionError("expected HILTimeoutError")


def test_hil_controller_server_invalid_measurement_triggers_fallback() -> None:
    server = HILControllerServer(_pid_adapter())

    command = server.handle_measurement(
        HILMeasurement(
            sequence_id=2,
            timestamp_s=0.1,
            dt=0.05,
            px=0.0,
            py=0.0,
            yaw=0.0,
            v=3.0,
            target_speed=4.0,
            target_lateral_position=0.0,
            target_heading=0.0,
            invalid=True,
        )
    )

    assert command.fallback_active
    assert command.controller_mode == "EMERGENCY_STOP"
    assert command.fallback_reason == "invalid_measurement"
    assert command.acceleration < 0.0


def test_hil_lite_loop_failure_injection_generates_report(tmp_path: Path) -> None:
    server = HILControllerServer(_pid_adapter())

    result = run_hil_lite_loop(
        server,
        config=HILLiteLoopConfig(dt=0.002, steps=6, command_timeout_s=0.002),
        failure_injection=FailureInjectionConfig(
            drop_request_steps=(1,),
            invalid_measurement_steps=(3,),
            delayed_request_steps={4: 0.004},
        ),
    )
    artifacts = write_hil_lite_report(result, tmp_path)

    assert result.step_count == 6
    assert result.timeout_count == 1
    assert result.fallback_count >= 2
    assert result.missed_deadline_count >= 1
    assert artifacts["json"].exists()
    assert artifacts["markdown"].exists()


def _pid_adapter() -> PythonControllerAdapter:
    constraints = VehicleConstraints()
    controller = VehiclePIDController(
        speed_pid=PID(
            kp=1.0,
            ki=0.0,
            kd=0.0,
            output_limits=PIDLimits(constraints.accel_min, constraints.accel_max),
        ),
        lateral_pid=PID(
            kp=0.6,
            ki=0.0,
            kd=0.0,
            output_limits=PIDLimits(constraints.steer_min, constraints.steer_max),
        ),
        constraints=constraints,
    )
    adapter = PythonControllerAdapter("pid", controller)
    adapter.initialize({"test": "hil"})
    return adapter


class _TimeoutSocket:
    def __enter__(self) -> _TimeoutSocket:
        return self

    def __exit__(self, *args: object) -> None:
        return None

    def settimeout(self, timeout_s: float) -> None:
        self.timeout_s = timeout_s

    def sendto(self, data: bytes, address: tuple[str, int]) -> int:
        self.sent = (data, address)
        return len(data)

    def recvfrom(self, max_packet_bytes: int) -> tuple[bytes, tuple[str, int]]:
        raise TimeoutError
