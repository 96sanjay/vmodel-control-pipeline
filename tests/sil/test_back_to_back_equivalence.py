from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

from vcp.controllers import (
    PID,
    LinearMPCConfig,
    LinearMPCController,
    PathTrackingTarget,
    PIDLimits,
    VehiclePIDController,
)
from vcp.models import VehicleConstraints, VehicleState
from vcp.validation import (
    CompiledControllerAdapter,
    ControllerStepInput,
    PythonControllerAdapter,
    SILEquivalenceTolerances,
    run_back_to_back_equivalence,
    write_sil_equivalence_report,
)


def test_tc_sil_if_001_linear_mpc_back_to_back_equivalence(tmp_path: Path) -> None:
    tolerances = SILEquivalenceTolerances(
        acceleration_abs=1e-7,
        steering_abs=1e-7,
        predicted_state_abs=1e-6,
    )
    mil_adapter = PythonControllerAdapter("linear_mpc", _linear_mpc())
    sil_adapter = PythonControllerAdapter("linear_mpc", _linear_mpc())

    report = run_back_to_back_equivalence(
        controller_name="linear_mpc",
        mil_adapter=mil_adapter,
        sil_adapter=sil_adapter,
        input_sequence=_input_sequence(),
        tolerances=tolerances,
    )
    artifacts = write_sil_equivalence_report(report, tmp_path)

    assert report.passed
    assert report.sample_count == 4
    assert report.max_acceleration_error <= tolerances.acceleration_abs
    assert report.max_steering_error <= tolerances.steering_abs
    assert report.max_predicted_state_error is not None
    assert report.max_predicted_state_error <= tolerances.predicted_state_abs
    assert artifacts["json"].exists()
    assert artifacts["markdown"].exists()

    payload = json.loads(artifacts["json"].read_text(encoding="utf-8"))
    assert payload["controller_name"] == "linear_mpc"
    assert payload["passed"] is True


def test_tc_sil_if_001_equivalence_report_fails_for_mismatched_pid() -> None:
    constraints = VehicleConstraints()
    mil_adapter = PythonControllerAdapter("pid", _pid_controller(kp=1.0, constraints=constraints))
    sil_adapter = PythonControllerAdapter("pid", _pid_controller(kp=0.4, constraints=constraints))

    report = run_back_to_back_equivalence(
        controller_name="pid",
        mil_adapter=mil_adapter,
        sil_adapter=sil_adapter,
        input_sequence=_input_sequence(),
        tolerances=SILEquivalenceTolerances(
            acceleration_abs=1e-9,
            steering_abs=1e-9,
        ),
    )

    assert not report.passed
    assert report.max_acceleration_error > 0.0


def test_compiled_controller_adapter_is_optional_until_artifact_exists() -> None:
    artifact_path = os.environ.get("VCP_COMPILED_CONTROLLER")
    if not artifact_path:
        pytest.skip("No compiled controller artifact configured for optional SIL test")

    adapter = CompiledControllerAdapter(artifact_path)
    adapter.initialize({"stage": "SIL"})


def _linear_mpc() -> LinearMPCController:
    return LinearMPCController(
        LinearMPCConfig(
            horizon=5,
            dt=0.1,
            nominal_velocity=4.0,
        )
    )


def _pid_controller(kp: float, constraints: VehicleConstraints) -> VehiclePIDController:
    return VehiclePIDController(
        speed_pid=PID(
            kp=kp,
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


def _input_sequence() -> tuple[ControllerStepInput, ...]:
    target = PathTrackingTarget(speed=4.0, lateral_position=0.0, heading=0.0)
    return tuple(
        ControllerStepInput(
            timestamp_s=index * 0.1,
            dt=0.1,
            state=VehicleState(
                px=0.2 * index,
                py=-0.6 + 0.1 * index,
                yaw=0.02 * index,
                v=2.0 + 0.2 * index,
            ),
            target=target,
        )
        for index in range(4)
    )
