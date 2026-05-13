from __future__ import annotations

from dataclasses import dataclass

from vcp.models.vehicle_state import VehicleInput, VehicleState


@dataclass(frozen=True)
class VehicleConstraints:
    """Configurable state and input limits for the vehicle model."""

    velocity_min: float = 0.0
    velocity_max: float = 15.0
    accel_min: float = -4.0
    accel_max: float = 2.0
    steer_min: float = -0.5
    steer_max: float = 0.5
    steer_rate_max: float = 0.2

    def __post_init__(self) -> None:
        _validate_order("velocity", self.velocity_min, self.velocity_max)
        _validate_order("acceleration", self.accel_min, self.accel_max)
        _validate_order("steering", self.steer_min, self.steer_max)
        if self.steer_rate_max <= 0.0:
            raise ValueError("steer_rate_max must be positive")


@dataclass(frozen=True)
class ConstraintViolation:
    """Single constraint violation with a machine-readable limit name."""

    name: str
    value: float
    lower: float | None = None
    upper: float | None = None


@dataclass(frozen=True)
class ConstraintCheckResult:
    """Result of checking state/input values against configured constraints."""

    is_valid: bool
    violations: tuple[ConstraintViolation, ...]


def check_state_constraints(
    state: VehicleState,
    constraints: VehicleConstraints | None = None,
) -> ConstraintCheckResult:
    """Check vehicle state limits."""

    constraints = constraints or VehicleConstraints()
    violations = _bounded("velocity", state.v, constraints.velocity_min, constraints.velocity_max)
    return ConstraintCheckResult(is_valid=not violations, violations=tuple(violations))


def check_input_constraints(
    command: VehicleInput,
    constraints: VehicleConstraints | None = None,
) -> ConstraintCheckResult:
    """Check vehicle command limits."""

    constraints = constraints or VehicleConstraints()
    violations = [
        *_bounded(
            "acceleration",
            command.acceleration,
            constraints.accel_min,
            constraints.accel_max,
        ),
        *_bounded(
            "steering_angle",
            command.steering_angle,
            constraints.steer_min,
            constraints.steer_max,
        ),
    ]
    return ConstraintCheckResult(is_valid=not violations, violations=tuple(violations))


def check_steer_rate_constraint(
    previous_command: VehicleInput,
    command: VehicleInput,
    dt: float,
    constraints: VehicleConstraints | None = None,
) -> ConstraintCheckResult:
    """Check steering-rate limit between consecutive commands."""

    if dt <= 0.0:
        raise ValueError("dt must be positive")

    constraints = constraints or VehicleConstraints()
    steer_rate = (command.steering_angle - previous_command.steering_angle) / dt
    violations = _bounded(
        "steering_rate",
        steer_rate,
        -constraints.steer_rate_max,
        constraints.steer_rate_max,
    )
    return ConstraintCheckResult(is_valid=not violations, violations=tuple(violations))


def clip_input(
    command: VehicleInput,
    constraints: VehicleConstraints | None = None,
) -> VehicleInput:
    """Clip acceleration and steering angle into valid command bounds."""

    constraints = constraints or VehicleConstraints()
    return VehicleInput(
        acceleration=_clip(command.acceleration, constraints.accel_min, constraints.accel_max),
        steering_angle=_clip(command.steering_angle, constraints.steer_min, constraints.steer_max),
    )


def _bounded(
    name: str,
    value: float,
    lower: float,
    upper: float,
) -> list[ConstraintViolation]:
    if value < lower:
        return [ConstraintViolation(name=name, value=value, lower=lower, upper=upper)]
    if value > upper:
        return [ConstraintViolation(name=name, value=value, lower=lower, upper=upper)]
    return []


def _clip(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _validate_order(name: str, lower: float, upper: float) -> None:
    if lower >= upper:
        raise ValueError(f"{name} lower limit must be less than upper limit")
