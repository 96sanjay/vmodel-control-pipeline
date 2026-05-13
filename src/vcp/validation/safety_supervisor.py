from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

from vcp.models import VehicleConstraints, VehicleInput, check_input_constraints


class SafetyMode(StrEnum):
    """Safety state selected by the validation supervisor."""

    NORMAL = "NORMAL"
    DEGRADED = "DEGRADED"
    FALLBACK_BRAKE = "FALLBACK_BRAKE"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    ESTIMATOR_FAULT = "ESTIMATOR_FAULT"
    SOLVER_TIMEOUT = "SOLVER_TIMEOUT"


@dataclass(frozen=True)
class SafetySupervisorConfig:
    """Thresholds used by the safety supervisor."""

    sample_time_s: float = 0.1
    max_solve_time_ms: float = 80.0
    degraded_residual_threshold: float = 2.0
    estimator_residual_threshold: float = 3.0
    constraints: VehicleConstraints = field(default_factory=VehicleConstraints)
    accepted_solver_statuses: tuple[str, ...] = (
        "optimal",
        "optimal_inaccurate",
        "solve_succeeded",
        "success",
    )

    def __post_init__(self) -> None:
        if self.sample_time_s <= 0.0:
            raise ValueError("sample_time_s must be positive")
        if self.max_solve_time_ms <= 0.0:
            raise ValueError("max_solve_time_ms must be positive")
        if self.degraded_residual_threshold < 0.0:
            raise ValueError("degraded_residual_threshold must be non-negative")
        if self.estimator_residual_threshold <= self.degraded_residual_threshold:
            raise ValueError(
                "estimator_residual_threshold must be greater than "
                "degraded_residual_threshold"
            )


@dataclass(frozen=True)
class SafetyEvaluationInput:
    """Diagnostics inspected before a command is accepted."""

    timestamp_s: float
    command: VehicleInput | None
    solver_status: str | None = None
    solver_feasible: bool = True
    solve_time_ms: float = 0.0
    estimator_residual: float | None = None
    collision_risk: bool = False
    missing_sensor_message: bool = False
    communication_timeout: bool = False
    constraint_violation_flags: tuple[str, ...] = ()


@dataclass(frozen=True)
class SafetyDecision:
    """Supervisor decision for the current control step."""

    mode: SafetyMode
    reason_code: str
    requirement_ids: tuple[str, ...]
    fallback_required: bool
    command_is_valid: bool


@dataclass(frozen=True)
class SafetyTransition:
    """Logged mode transition with traceability metadata."""

    timestamp_s: float
    previous_mode: SafetyMode
    new_mode: SafetyMode
    reason_code: str
    requirement_ids: tuple[str, ...]


class SafetySupervisor:
    """Evaluate controller diagnostics and select a safe operating mode."""

    def __init__(self, config: SafetySupervisorConfig | None = None) -> None:
        self.config = config or SafetySupervisorConfig()
        self._mode = SafetyMode.NORMAL
        self._transitions: list[SafetyTransition] = []

    @property
    def mode(self) -> SafetyMode:
        return self._mode

    @property
    def transition_log(self) -> tuple[SafetyTransition, ...]:
        return tuple(self._transitions)

    def reset(self) -> None:
        self._mode = SafetyMode.NORMAL
        self._transitions.clear()

    def evaluate(self, diagnostics: SafetyEvaluationInput) -> SafetyDecision:
        """Return the highest-priority safety decision for one control step."""

        decision = self._select_decision(diagnostics)
        if decision.mode != self._mode:
            self._transitions.append(
                SafetyTransition(
                    timestamp_s=diagnostics.timestamp_s,
                    previous_mode=self._mode,
                    new_mode=decision.mode,
                    reason_code=decision.reason_code,
                    requirement_ids=decision.requirement_ids,
                )
            )
            self._mode = decision.mode
        return decision

    def _select_decision(self, diagnostics: SafetyEvaluationInput) -> SafetyDecision:
        command_valid = self._command_is_valid(diagnostics.command)

        if diagnostics.collision_risk:
            return _decision(
                SafetyMode.EMERGENCY_STOP,
                "collision_risk",
                ("REQ-CR-001", "REQ-CR-004"),
                command_valid,
            )
        if diagnostics.missing_sensor_message or diagnostics.communication_timeout:
            return _decision(
                SafetyMode.EMERGENCY_STOP,
                "missing_or_stale_sensor_message",
                ("REQ-CR-004", "REQ-CR-010"),
                command_valid,
            )
        if not diagnostics.solver_feasible or not _solver_status_is_accepted(
            diagnostics.solver_status,
            self.config.accepted_solver_statuses,
        ):
            return _decision(
                SafetyMode.FALLBACK_BRAKE,
                "solver_infeasible",
                ("REQ-CR-004",),
                command_valid,
            )
        if diagnostics.solve_time_ms > self.config.max_solve_time_ms:
            return _decision(
                SafetyMode.SOLVER_TIMEOUT,
                "solver_timeout",
                ("REQ-CR-004", "REQ-CR-005"),
                command_valid,
            )
        if diagnostics.estimator_residual is not None:
            if diagnostics.estimator_residual > self.config.estimator_residual_threshold:
                return _decision(
                    SafetyMode.ESTIMATOR_FAULT,
                    "estimator_residual_high",
                    ("REQ-CR-004", "REQ-CR-007"),
                    command_valid,
                )
            if diagnostics.estimator_residual > self.config.degraded_residual_threshold:
                return SafetyDecision(
                    mode=SafetyMode.DEGRADED,
                    reason_code="estimator_residual_degraded",
                    requirement_ids=("REQ-CR-007",),
                    fallback_required=False,
                    command_is_valid=command_valid,
                )
        if diagnostics.constraint_violation_flags or not command_valid:
            return _decision(
                SafetyMode.FALLBACK_BRAKE,
                "command_limit_violation",
                ("REQ-CR-003", "REQ-CR-004"),
                command_valid,
            )
        return SafetyDecision(
            mode=SafetyMode.NORMAL,
            reason_code="no_fault",
            requirement_ids=(),
            fallback_required=False,
            command_is_valid=True,
        )

    def _command_is_valid(self, command: VehicleInput | None) -> bool:
        if command is None:
            return False
        try:
            command.validate()
        except ValueError:
            return False
        return check_input_constraints(command, self.config.constraints).is_valid


def _decision(
    mode: SafetyMode,
    reason_code: str,
    requirement_ids: tuple[str, ...],
    command_is_valid: bool,
) -> SafetyDecision:
    return SafetyDecision(
        mode=mode,
        reason_code=reason_code,
        requirement_ids=requirement_ids,
        fallback_required=True,
        command_is_valid=command_is_valid,
    )


def _solver_status_is_accepted(
    solver_status: str | None,
    accepted_statuses: tuple[str, ...],
) -> bool:
    if solver_status is None:
        return True
    normalized = solver_status.strip().lower().replace(" ", "_")
    return normalized in accepted_statuses


__all__ = [
    "SafetyDecision",
    "SafetyEvaluationInput",
    "SafetyMode",
    "SafetySupervisor",
    "SafetySupervisorConfig",
    "SafetyTransition",
]
