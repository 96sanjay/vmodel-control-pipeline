from __future__ import annotations

from dataclasses import dataclass
from math import atan2, isclose, isfinite

from vcp.models import VehicleConstraints, VehicleInput, VehicleState, clip_input


@dataclass(frozen=True)
class PIDLimits:
    """Optional lower and upper limits for PID output or integral state."""

    lower: float | None = None
    upper: float | None = None

    def __post_init__(self) -> None:
        if self.lower is not None and self.upper is not None and self.lower >= self.upper:
            raise ValueError("lower limit must be less than upper limit")


@dataclass(frozen=True)
class PIDDiagnostics:
    """Diagnostics from the most recent PID update."""

    error: float
    integral: float
    derivative: float
    unsaturated_output: float
    output: float
    saturated: bool


class PID:
    """Reusable PID controller with saturation and conditional anti-windup."""

    def __init__(
        self,
        kp: float,
        ki: float = 0.0,
        kd: float = 0.0,
        *,
        output_limits: PIDLimits | None = None,
        integral_limits: PIDLimits | None = None,
        anti_windup: bool = True,
    ) -> None:
        self.kp = _finite(kp, "kp")
        self.ki = _finite(ki, "ki")
        self.kd = _finite(kd, "kd")
        self.output_limits = output_limits or PIDLimits()
        self.integral_limits = integral_limits or PIDLimits()
        self.anti_windup = anti_windup
        self._integral = 0.0
        self._previous_error: float | None = None
        self._last_diagnostics = PIDDiagnostics(
            error=0.0,
            integral=0.0,
            derivative=0.0,
            unsaturated_output=0.0,
            output=0.0,
            saturated=False,
        )

    @property
    def integral(self) -> float:
        return self._integral

    @property
    def diagnostics(self) -> PIDDiagnostics:
        return self._last_diagnostics

    def reset(self) -> None:
        self._integral = 0.0
        self._previous_error = None
        self._last_diagnostics = PIDDiagnostics(
            error=0.0,
            integral=0.0,
            derivative=0.0,
            unsaturated_output=0.0,
            output=0.0,
            saturated=False,
        )

    def update(self, error: float, dt: float) -> float:
        error = _finite(error, "error")
        dt = _positive(dt, "dt")

        derivative = 0.0 if self._previous_error is None else (error - self._previous_error) / dt
        candidate_integral = _clamp(self._integral + error * dt, self.integral_limits)
        unsaturated_output = self._compute_output(error, candidate_integral, derivative)
        output = _clamp(unsaturated_output, self.output_limits)
        saturated = not isclose(output, unsaturated_output, rel_tol=0.0, abs_tol=1e-12)

        if not (self.anti_windup and saturated and _drives_further_into_saturation(error, output)):
            self._integral = candidate_integral
            unsaturated_output = self._compute_output(error, self._integral, derivative)
            output = _clamp(unsaturated_output, self.output_limits)
            saturated = not isclose(output, unsaturated_output, rel_tol=0.0, abs_tol=1e-12)

        self._previous_error = error
        self._last_diagnostics = PIDDiagnostics(
            error=error,
            integral=self._integral,
            derivative=derivative,
            unsaturated_output=unsaturated_output,
            output=output,
            saturated=saturated,
        )
        return output

    def _compute_output(self, error: float, integral: float, derivative: float) -> float:
        return self.kp * error + self.ki * integral + self.kd * derivative


@dataclass(frozen=True)
class PathTrackingTarget:
    """Simple straight-path tracking target for the baseline PID controller."""

    speed: float
    lateral_position: float = 0.0
    heading: float = 0.0


@dataclass(frozen=True)
class VehiclePIDDiagnostics:
    """Diagnostics returned by the vehicle-level PID controller."""

    speed_error: float
    lateral_error: float
    heading_error: float
    steering_error: float
    acceleration_saturated: bool
    steering_saturated: bool
    command_saturated: bool


class VehiclePIDController:
    """Baseline vehicle controller with longitudinal and lateral PID loops."""

    def __init__(
        self,
        speed_pid: PID,
        lateral_pid: PID,
        *,
        constraints: VehicleConstraints | None = None,
        heading_error_weight: float = 1.0,
    ) -> None:
        self.speed_pid = speed_pid
        self.lateral_pid = lateral_pid
        self.constraints = constraints or VehicleConstraints()
        self.heading_error_weight = _finite(heading_error_weight, "heading_error_weight")

    def reset(self) -> None:
        self.speed_pid.reset()
        self.lateral_pid.reset()

    def compute_control(
        self,
        state: VehicleState,
        target: PathTrackingTarget,
        dt: float,
    ) -> tuple[VehicleInput, VehiclePIDDiagnostics]:
        state.validate()
        dt = _positive(dt, "dt")

        speed_error = target.speed - state.v
        lateral_error = target.lateral_position - state.py
        heading_error = normalize_angle(target.heading - state.yaw)
        steering_error = lateral_error + self.heading_error_weight * heading_error

        acceleration = self.speed_pid.update(speed_error, dt)
        steering_angle = self.lateral_pid.update(steering_error, dt)
        raw_command = VehicleInput(acceleration=acceleration, steering_angle=steering_angle)
        command = clip_input(raw_command, self.constraints)

        acceleration_saturated = not isclose(
            command.acceleration,
            raw_command.acceleration,
            rel_tol=0.0,
            abs_tol=1e-12,
        ) or self.speed_pid.diagnostics.saturated
        steering_saturated = not isclose(
            command.steering_angle,
            raw_command.steering_angle,
            rel_tol=0.0,
            abs_tol=1e-12,
        ) or self.lateral_pid.diagnostics.saturated

        diagnostics = VehiclePIDDiagnostics(
            speed_error=speed_error,
            lateral_error=lateral_error,
            heading_error=heading_error,
            steering_error=steering_error,
            acceleration_saturated=acceleration_saturated,
            steering_saturated=steering_saturated,
            command_saturated=acceleration_saturated or steering_saturated,
        )
        return command, diagnostics


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""

    angle = _finite(angle, "angle")
    return atan2(sin_angle(angle), cos_angle(angle))


def sin_angle(angle: float) -> float:
    """Small wrapper kept separate to simplify future vectorization."""

    from math import sin

    return sin(angle)


def cos_angle(angle: float) -> float:
    """Small wrapper kept separate to simplify future vectorization."""

    from math import cos

    return cos(angle)


def create_default_vehicle_pid(
    constraints: VehicleConstraints | None = None,
) -> VehiclePIDController:
    """Create a conservative baseline controller for early closed-loop smoke tests."""

    constraints = constraints or VehicleConstraints()
    return VehiclePIDController(
        speed_pid=PID(
            kp=1.0,
            ki=0.2,
            kd=0.05,
            output_limits=PIDLimits(constraints.accel_min, constraints.accel_max),
            integral_limits=PIDLimits(-5.0, 5.0),
        ),
        lateral_pid=PID(
            kp=0.6,
            ki=0.02,
            kd=0.1,
            output_limits=PIDLimits(constraints.steer_min, constraints.steer_max),
            integral_limits=PIDLimits(-2.0, 2.0),
        ),
        constraints=constraints,
        heading_error_weight=1.5,
    )


def _drives_further_into_saturation(error: float, output: float) -> bool:
    return (output > 0.0 and error > 0.0) or (output < 0.0 and error < 0.0)


def _clamp(value: float, limits: PIDLimits) -> float:
    if limits.lower is not None:
        value = max(value, limits.lower)
    if limits.upper is not None:
        value = min(value, limits.upper)
    return value


def _finite(value: float, name: str) -> float:
    if not isfinite(value):
        raise ValueError(f"{name} must be finite")
    return float(value)


def _positive(value: float, name: str) -> float:
    value = _finite(value, name)
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")
    return value


__all__ = [
    "PID",
    "PIDDiagnostics",
    "PIDLimits",
    "PathTrackingTarget",
    "VehiclePIDController",
    "VehiclePIDDiagnostics",
    "create_default_vehicle_pid",
    "normalize_angle",
]
