from __future__ import annotations

from dataclasses import dataclass, field
from time import perf_counter

import cvxpy as cp
import numpy as np

from vcp.controllers.pid import PathTrackingTarget, normalize_angle
from vcp.models import FloatArray, VehicleConstraints, VehicleInput, VehicleState, clip_input

FEASIBLE_STATUSES = {cp.OPTIMAL, cp.OPTIMAL_INACCURATE}


@dataclass(frozen=True)
class LinearMPCConfig:
    """Configuration for the linear MPC tracking problem."""

    horizon: int = 12
    dt: float = 0.1
    wheelbase: float = 2.8
    nominal_velocity: float = 4.0
    q_lateral: float = 8.0
    q_heading: float = 2.0
    q_speed: float = 1.0
    r_accel: float = 0.2
    r_steer: float = 0.4
    rd_accel: float = 0.05
    rd_steer: float = 0.20
    constraints: VehicleConstraints = field(default_factory=VehicleConstraints)
    solver: str = "OSQP"

    def __post_init__(self) -> None:
        if self.horizon <= 0:
            raise ValueError("horizon must be positive")
        if self.dt <= 0.0:
            raise ValueError("dt must be positive")
        if self.wheelbase <= 0.0:
            raise ValueError("wheelbase must be positive")
        if self.nominal_velocity <= 0.0:
            raise ValueError("nominal_velocity must be positive")
        for name, value in (
            ("q_lateral", self.q_lateral),
            ("q_heading", self.q_heading),
            ("q_speed", self.q_speed),
            ("r_accel", self.r_accel),
            ("r_steer", self.r_steer),
            ("rd_accel", self.rd_accel),
            ("rd_steer", self.rd_steer),
        ):
            if value < 0.0:
                raise ValueError(f"{name} must be non-negative")

    @property
    def q_matrix(self) -> FloatArray:
        return np.diag([self.q_lateral, self.q_heading, self.q_speed]).astype(np.float64)

    @property
    def r_matrix(self) -> FloatArray:
        return np.diag([self.r_accel, self.r_steer]).astype(np.float64)

    @property
    def rd_matrix(self) -> FloatArray:
        return np.diag([self.rd_accel, self.rd_steer]).astype(np.float64)


@dataclass(frozen=True)
class LinearMPCResult:
    """Result returned by one linear MPC solve."""

    command: VehicleInput
    predicted_states: FloatArray
    predicted_inputs: FloatArray
    solver_status: str
    solve_time_ms: float
    objective_value: float | None
    feasible: bool


class LinearMPCController:
    """Finite-horizon linear MPC for path tracking around a nominal speed."""

    def __init__(self, config: LinearMPCConfig | None = None) -> None:
        self.config = config or LinearMPCConfig()
        self._previous_input = np.zeros((2,), dtype=np.float64)

    def reset(self) -> None:
        self._previous_input = np.zeros((2,), dtype=np.float64)

    def compute_control(
        self,
        state: VehicleState,
        target: PathTrackingTarget,
    ) -> LinearMPCResult:
        """Solve the MPC problem and return the first control action."""

        initial_error_state = self._tracking_error_state(state, target)
        start_time = perf_counter()
        result = self._solve(initial_error_state, target.speed)
        solve_time_ms = (perf_counter() - start_time) * 1000.0

        if result.feasible:
            self._previous_input = result.predicted_inputs[:, 0].copy()

        return LinearMPCResult(
            command=result.command,
            predicted_states=result.predicted_states,
            predicted_inputs=result.predicted_inputs,
            solver_status=result.solver_status,
            solve_time_ms=solve_time_ms,
            objective_value=result.objective_value,
            feasible=result.feasible,
        )

    def _solve(self, initial_error_state: FloatArray, target_speed: float) -> LinearMPCResult:
        config = self.config
        horizon = config.horizon
        a_matrix, b_matrix = self._discrete_dynamics()

        x_var = cp.Variable((3, horizon + 1))
        u_var = cp.Variable((2, horizon))
        constraints = [x_var[:, 0] == initial_error_state]
        objective = 0
        previous_input = self._previous_input

        for step in range(horizon):
            objective += cp.quad_form(x_var[:, step], config.q_matrix)
            objective += cp.quad_form(u_var[:, step], config.r_matrix)
            objective += cp.quad_form(u_var[:, step] - previous_input, config.rd_matrix)
            constraints.extend(
                [
                    x_var[:, step + 1] == a_matrix @ x_var[:, step] + b_matrix @ u_var[:, step],
                    u_var[0, step] >= config.constraints.accel_min,
                    u_var[0, step] <= config.constraints.accel_max,
                    u_var[1, step] >= config.constraints.steer_min,
                    u_var[1, step] <= config.constraints.steer_max,
                    target_speed + x_var[2, step + 1] >= config.constraints.velocity_min,
                    target_speed + x_var[2, step + 1] <= config.constraints.velocity_max,
                ]
            )
            previous_input = u_var[:, step]

        objective += cp.quad_form(x_var[:, horizon], config.q_matrix)
        problem = cp.Problem(cp.Minimize(objective), constraints)
        try:
            problem.solve(solver=config.solver, warm_start=True, verbose=False)
        except cp.error.SolverError:
            return _failed_result("solver_error")

        if problem.status not in FEASIBLE_STATUSES or u_var.value is None or x_var.value is None:
            return _failed_result(str(problem.status))

        raw_command = VehicleInput(
            acceleration=float(u_var.value[0, 0]),
            steering_angle=float(u_var.value[1, 0]),
        )
        command = clip_input(raw_command, config.constraints)
        return LinearMPCResult(
            command=command,
            predicted_states=np.asarray(x_var.value, dtype=np.float64),
            predicted_inputs=np.asarray(u_var.value, dtype=np.float64),
            solver_status=str(problem.status),
            solve_time_ms=0.0,
            objective_value=None if problem.value is None else float(problem.value),
            feasible=True,
        )

    def _discrete_dynamics(self) -> tuple[FloatArray, FloatArray]:
        config = self.config
        a_matrix = np.array(
            [
                [1.0, config.nominal_velocity * config.dt, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        b_matrix = np.array(
            [
                [0.0, 0.0],
                [0.0, config.nominal_velocity / config.wheelbase * config.dt],
                [config.dt, 0.0],
            ],
            dtype=np.float64,
        )
        return a_matrix, b_matrix

    @staticmethod
    def _tracking_error_state(state: VehicleState, target: PathTrackingTarget) -> FloatArray:
        return np.array(
            [
                state.py - target.lateral_position,
                normalize_angle(state.yaw - target.heading),
                state.v - target.speed,
            ],
            dtype=np.float64,
        )


def _failed_result(status: str) -> LinearMPCResult:
    return LinearMPCResult(
        command=VehicleInput(acceleration=0.0, steering_angle=0.0),
        predicted_states=np.empty((3, 0), dtype=np.float64),
        predicted_inputs=np.empty((2, 0), dtype=np.float64),
        solver_status=status,
        solve_time_ms=0.0,
        objective_value=None,
        feasible=False,
    )


__all__ = [
    "LinearMPCConfig",
    "LinearMPCController",
    "LinearMPCResult",
]
