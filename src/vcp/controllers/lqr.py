from __future__ import annotations

from dataclasses import dataclass
from math import isclose

import numpy as np

from vcp.controllers.pid import PathTrackingTarget, normalize_angle
from vcp.models import VehicleConstraints, VehicleInput, VehicleState, clip_input
from vcp.models.linearized_bicycle import FloatArray, LinearizedBicycleModel


@dataclass(frozen=True)
class LQRWeights:
    """Diagonal cost weights for the lateral LQR problem."""

    q_lateral: float = 4.0
    q_heading: float = 1.0
    r_steering: float = 0.5

    def q_matrix(self) -> FloatArray:
        _positive(self.q_lateral, "q_lateral")
        _positive(self.q_heading, "q_heading")
        return np.diag([self.q_lateral, self.q_heading]).astype(np.float64)

    def r_matrix(self) -> FloatArray:
        _positive(self.r_steering, "r_steering")
        return np.array([[self.r_steering]], dtype=np.float64)


@dataclass(frozen=True)
class LQRDiagnostics:
    """Diagnostics returned by the vehicle-level LQR baseline."""

    lateral_error: float
    heading_error: float
    speed_error: float
    raw_steering: float
    steering_saturated: bool
    acceleration_saturated: bool
    closed_loop_eigenvalues: tuple[complex, ...]


class LQRController:
    """Lateral LQR baseline with simple proportional longitudinal speed control."""

    def __init__(
        self,
        linear_model: LinearizedBicycleModel,
        *,
        weights: LQRWeights | None = None,
        constraints: VehicleConstraints | None = None,
        speed_gain: float = 1.0,
    ) -> None:
        self.linear_model = linear_model
        self.weights = weights or LQRWeights()
        self.constraints = constraints or VehicleConstraints()
        self.speed_gain = _positive(speed_gain, "speed_gain")

        a_matrix, b_matrix = self.linear_model.discrete_matrices()
        self.gain = solve_discrete_lqr(
            a_matrix,
            b_matrix,
            self.weights.q_matrix(),
            self.weights.r_matrix(),
        )
        self.closed_loop_eigenvalues = tuple(self.linear_model.closed_loop_eigenvalues(self.gain))

    def compute_control(
        self,
        state: VehicleState,
        target: PathTrackingTarget,
    ) -> tuple[VehicleInput, LQRDiagnostics]:
        """Compute acceleration and steering commands for a path-tracking target."""

        state.validate()
        lateral_error = state.py - target.lateral_position
        heading_error = normalize_angle(state.yaw - target.heading)
        speed_error = target.speed - state.v

        lateral_state = np.array([[lateral_error], [heading_error]], dtype=np.float64)
        raw_steering = float((-self.gain @ lateral_state)[0, 0])
        raw_acceleration = self.speed_gain * speed_error
        raw_command = VehicleInput(
            acceleration=raw_acceleration,
            steering_angle=raw_steering,
        )
        command = clip_input(raw_command, self.constraints)

        steering_saturated = not isclose(
            command.steering_angle,
            raw_command.steering_angle,
            rel_tol=0.0,
            abs_tol=1e-12,
        )
        acceleration_saturated = not isclose(
            command.acceleration,
            raw_command.acceleration,
            rel_tol=0.0,
            abs_tol=1e-12,
        )

        return command, LQRDiagnostics(
            lateral_error=lateral_error,
            heading_error=heading_error,
            speed_error=speed_error,
            raw_steering=raw_steering,
            steering_saturated=steering_saturated,
            acceleration_saturated=acceleration_saturated,
            closed_loop_eigenvalues=self.closed_loop_eigenvalues,
        )


def solve_discrete_lqr(
    a_matrix: FloatArray,
    b_matrix: FloatArray,
    q_matrix: FloatArray,
    r_matrix: FloatArray,
    *,
    max_iterations: int = 1000,
    tolerance: float = 1e-10,
) -> FloatArray:
    """Solve the discrete-time LQR problem by Riccati iteration."""

    _validate_lqr_shapes(a_matrix, b_matrix, q_matrix, r_matrix)
    if max_iterations <= 0:
        raise ValueError("max_iterations must be positive")
    if tolerance <= 0.0:
        raise ValueError("tolerance must be positive")

    p_matrix = q_matrix.copy()
    for _ in range(max_iterations):
        gain = _lqr_gain_from_riccati(a_matrix, b_matrix, r_matrix, p_matrix)
        p_next = (
            a_matrix.T @ p_matrix @ a_matrix
            - a_matrix.T @ p_matrix @ b_matrix @ gain
            + q_matrix
        )
        p_next = 0.5 * (p_next + p_next.T)
        if np.max(np.abs(p_next - p_matrix)) < tolerance:
            return _lqr_gain_from_riccati(a_matrix, b_matrix, r_matrix, p_next)
        p_matrix = p_next

    raise RuntimeError("discrete LQR Riccati iteration did not converge")


def _lqr_gain_from_riccati(
    a_matrix: FloatArray,
    b_matrix: FloatArray,
    r_matrix: FloatArray,
    p_matrix: FloatArray,
) -> FloatArray:
    system_matrix = r_matrix + b_matrix.T @ p_matrix @ b_matrix
    rhs = b_matrix.T @ p_matrix @ a_matrix
    return np.linalg.solve(system_matrix, rhs)


def _validate_lqr_shapes(
    a_matrix: FloatArray,
    b_matrix: FloatArray,
    q_matrix: FloatArray,
    r_matrix: FloatArray,
) -> None:
    if a_matrix.ndim != 2 or a_matrix.shape[0] != a_matrix.shape[1]:
        raise ValueError("a_matrix must be square")
    if b_matrix.ndim != 2 or b_matrix.shape[0] != a_matrix.shape[0]:
        raise ValueError("b_matrix row count must match a_matrix")
    if q_matrix.shape != a_matrix.shape:
        raise ValueError("q_matrix shape must match a_matrix")
    if r_matrix.shape != (b_matrix.shape[1], b_matrix.shape[1]):
        raise ValueError("r_matrix shape must match input dimension")


def _positive(value: float, name: str) -> float:
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")
    return float(value)


__all__ = [
    "LQRController",
    "LQRDiagnostics",
    "LQRWeights",
    "solve_discrete_lqr",
]
