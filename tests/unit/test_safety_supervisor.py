from __future__ import annotations

from vcp.controllers import FallbackBrakeConfig, FallbackBrakeController
from vcp.models import VehicleConstraints, VehicleInput, VehicleState
from vcp.validation import (
    SafetyEvaluationInput,
    SafetyMode,
    SafetySupervisor,
    SafetySupervisorConfig,
)

DEFAULT_COMMAND = VehicleInput(acceleration=0.1, steering_angle=0.0)


def _input(
    *,
    command: VehicleInput | None = DEFAULT_COMMAND,
    timestamp_s: float = 1.0,
    solver_status: str | None = "optimal",
    solver_feasible: bool = True,
    solve_time_ms: float = 20.0,
    estimator_residual: float | None = 0.5,
    collision_risk: bool = False,
    missing_sensor_message: bool = False,
    communication_timeout: bool = False,
    constraint_violation_flags: tuple[str, ...] = (),
) -> SafetyEvaluationInput:
    return SafetyEvaluationInput(
        timestamp_s=timestamp_s,
        command=command,
        solver_status=solver_status,
        solver_feasible=solver_feasible,
        solve_time_ms=solve_time_ms,
        estimator_residual=estimator_residual,
        collision_risk=collision_risk,
        missing_sensor_message=missing_sensor_message,
        communication_timeout=communication_timeout,
        constraint_violation_flags=constraint_violation_flags,
    )


def test_tc_safe_001_normal_command_stays_normal() -> None:
    supervisor = SafetySupervisor()

    decision = supervisor.evaluate(_input())

    assert decision.mode is SafetyMode.NORMAL
    assert decision.reason_code == "no_fault"
    assert not decision.fallback_required
    assert decision.command_is_valid
    assert supervisor.transition_log == ()


def test_tc_safe_001_solver_infeasible_triggers_fallback_brake() -> None:
    supervisor = SafetySupervisor()

    decision = supervisor.evaluate(_input(solver_status="infeasible", solver_feasible=False))

    assert decision.mode is SafetyMode.FALLBACK_BRAKE
    assert decision.reason_code == "solver_infeasible"
    assert decision.fallback_required
    assert "REQ-CR-004" in decision.requirement_ids
    assert len(supervisor.transition_log) == 1
    assert supervisor.transition_log[0].new_mode is SafetyMode.FALLBACK_BRAKE


def test_tc_safe_001_solver_timeout_triggers_timeout_mode() -> None:
    supervisor = SafetySupervisor(SafetySupervisorConfig(max_solve_time_ms=50.0))

    decision = supervisor.evaluate(_input(solve_time_ms=51.0))

    assert decision.mode is SafetyMode.SOLVER_TIMEOUT
    assert decision.reason_code == "solver_timeout"
    assert decision.fallback_required
    assert {"REQ-CR-004", "REQ-CR-005"} <= set(decision.requirement_ids)


def test_tc_safe_001_estimator_fault_triggers_estimator_fault_mode() -> None:
    supervisor = SafetySupervisor(
        SafetySupervisorConfig(
            degraded_residual_threshold=2.0,
            estimator_residual_threshold=3.0,
        )
    )

    decision = supervisor.evaluate(_input(estimator_residual=3.5))

    assert decision.mode is SafetyMode.ESTIMATOR_FAULT
    assert decision.reason_code == "estimator_residual_high"
    assert decision.fallback_required
    assert {"REQ-CR-004", "REQ-CR-007"} <= set(decision.requirement_ids)


def test_tc_safe_001_estimator_warning_enters_degraded_without_fallback() -> None:
    supervisor = SafetySupervisor(
        SafetySupervisorConfig(
            degraded_residual_threshold=2.0,
            estimator_residual_threshold=3.0,
        )
    )

    decision = supervisor.evaluate(_input(estimator_residual=2.5))

    assert decision.mode is SafetyMode.DEGRADED
    assert decision.reason_code == "estimator_residual_degraded"
    assert not decision.fallback_required
    assert "REQ-CR-007" in decision.requirement_ids


def test_tc_safe_001_invalid_command_triggers_fallback_brake() -> None:
    constraints = VehicleConstraints(accel_min=-1.0, accel_max=1.0, steer_min=-0.2, steer_max=0.2)
    supervisor = SafetySupervisor(SafetySupervisorConfig(constraints=constraints))

    decision = supervisor.evaluate(
        _input(command=VehicleInput(acceleration=0.0, steering_angle=0.4))
    )

    assert decision.mode is SafetyMode.FALLBACK_BRAKE
    assert decision.reason_code == "command_limit_violation"
    assert not decision.command_is_valid
    assert {"REQ-CR-003", "REQ-CR-004"} <= set(decision.requirement_ids)


def test_tc_safe_001_collision_risk_has_emergency_stop_priority() -> None:
    supervisor = SafetySupervisor(SafetySupervisorConfig(max_solve_time_ms=50.0))

    decision = supervisor.evaluate(
        _input(collision_risk=True, solver_feasible=False, solve_time_ms=100.0)
    )

    assert decision.mode is SafetyMode.EMERGENCY_STOP
    assert decision.reason_code == "collision_risk"
    assert decision.fallback_required
    assert {"REQ-CR-001", "REQ-CR-004"} <= set(decision.requirement_ids)


def test_tc_safe_001_missing_sensor_triggers_emergency_stop() -> None:
    supervisor = SafetySupervisor()

    decision = supervisor.evaluate(_input(missing_sensor_message=True))

    assert decision.mode is SafetyMode.EMERGENCY_STOP
    assert decision.reason_code == "missing_or_stale_sensor_message"
    assert decision.fallback_required
    assert {"REQ-CR-004", "REQ-CR-010"} <= set(decision.requirement_ids)


def test_tc_safe_001_mode_recovery_is_logged() -> None:
    supervisor = SafetySupervisor()

    supervisor.evaluate(_input(solver_status="infeasible", solver_feasible=False, timestamp_s=1.0))
    supervisor.evaluate(_input(timestamp_s=2.0))

    assert supervisor.mode is SafetyMode.NORMAL
    assert len(supervisor.transition_log) == 2
    assert supervisor.transition_log[0].previous_mode is SafetyMode.NORMAL
    assert supervisor.transition_log[1].previous_mode is SafetyMode.FALLBACK_BRAKE
    assert supervisor.transition_log[1].new_mode is SafetyMode.NORMAL


def test_tc_safe_001_fallback_brake_controller_is_deterministic() -> None:
    constraints = VehicleConstraints(accel_min=-3.0, accel_max=1.0, steer_min=-0.5, steer_max=0.5)
    controller = FallbackBrakeController(
        FallbackBrakeConfig(brake_acceleration=-4.0, constraints=constraints)
    )

    moving_command = controller.compute_control(VehicleState(px=0.0, py=0.0, yaw=0.0, v=5.0))
    stopped_command = controller.compute_control(VehicleState(px=0.0, py=0.0, yaw=0.0, v=0.0))

    assert moving_command == VehicleInput(acceleration=-3.0, steering_angle=0.0)
    assert stopped_command == VehicleInput(acceleration=0.0, steering_angle=0.0)
