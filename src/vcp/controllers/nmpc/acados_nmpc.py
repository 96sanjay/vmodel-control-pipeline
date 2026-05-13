from __future__ import annotations

from dataclasses import dataclass, field
from importlib.util import find_spec
from math import isfinite
from time import perf_counter
from typing import Literal

import casadi as ca
import numpy as np

from vcp.controllers.nmpc.casadi_model import create_casadi_kinematic_bicycle_model
from vcp.controllers.pid import PathTrackingTarget
from vcp.models import FloatArray, VehicleConstraints, VehicleInput, VehicleState, clip_input

NMPCBackend = Literal["casadi_ipopt", "acados_unavailable"]


@dataclass(frozen=True)
class NMPCConfig:
    """Configuration for the nonlinear MPC path-tracking problem."""

    horizon: int = 10
    dt: float = 0.1
    wheelbase: float = 2.8
    q_position: float = 10.0
    q_heading: float = 4.0
    q_velocity: float = 2.0
    r_accel: float = 0.2
    r_steer: float = 0.4
    rd_accel: float = 0.05
    rd_steer: float = 0.10
    max_solver_iterations: int = 100
    constraints: VehicleConstraints = field(default_factory=VehicleConstraints)

    def __post_init__(self) -> None:
        if self.horizon <= 0:
            raise ValueError("horizon must be positive")
        if self.dt <= 0.0:
            raise ValueError("dt must be positive")
        if self.wheelbase <= 0.0:
            raise ValueError("wheelbase must be positive")
        if self.max_solver_iterations <= 0:
            raise ValueError("max_solver_iterations must be positive")
        for name, value in (
            ("q_position", self.q_position),
            ("q_heading", self.q_heading),
            ("q_velocity", self.q_velocity),
            ("r_accel", self.r_accel),
            ("r_steer", self.r_steer),
            ("rd_accel", self.rd_accel),
            ("rd_steer", self.rd_steer),
        ):
            _non_negative(value, name)


@dataclass(frozen=True)
class NMPCResult:
    """Result returned by one nonlinear MPC solve."""

    command: VehicleInput
    predicted_states: FloatArray
    predicted_inputs: FloatArray
    solver_status: str
    solve_time_ms: float
    objective_value: float | None
    feasible: bool
    backend: NMPCBackend
    constraint_violation_flags: tuple[str, ...] = ()


class CasadiNMPCController:
    """Nonlinear MPC using CasADi/IPOPT with a kinematic bicycle model."""

    backend: NMPCBackend = "casadi_ipopt"

    def __init__(self, config: NMPCConfig | None = None) -> None:
        self.config = config or NMPCConfig()
        self._previous_input = np.zeros((2,), dtype=np.float64)
        self._warm_start: FloatArray | None = None
        self._solver, self._variable_bounds, self._constraint_count = self._build_solver()

    def reset(self) -> None:
        self._previous_input = np.zeros((2,), dtype=np.float64)
        self._warm_start = None

    def compute_control(
        self,
        state: VehicleState,
        target: PathTrackingTarget,
        reference_states: FloatArray | None = None,
    ) -> NMPCResult:
        """Solve the NMPC problem and return the first acceleration/steering command."""

        state.validate()
        reference = (
            _validate_reference(reference_states, self.config.horizon)
            if reference_states is not None
            else build_straight_reference(state, target, self.config)
        )
        parameters = self._build_parameter_vector(state, reference)
        initial_guess = self._initial_guess(state, reference)
        lower_bounds, upper_bounds = self._variable_bounds
        start_time = perf_counter()

        try:
            solution = self._solver(
                x0=initial_guess,
                p=parameters,
                lbx=lower_bounds,
                ubx=upper_bounds,
                lbg=np.zeros((self._constraint_count,), dtype=np.float64),
                ubg=np.zeros((self._constraint_count,), dtype=np.float64),
            )
            stats = self._solver.stats()
        except RuntimeError as exc:
            return _failed_result(str(exc), self.backend, (perf_counter() - start_time) * 1000.0)

        solve_time_ms = (perf_counter() - start_time) * 1000.0
        solver_status = str(stats.get("return_status", "unknown"))
        feasible = bool(stats.get("success", False))
        if not feasible:
            return _failed_result(solver_status, self.backend, solve_time_ms)

        decision = np.asarray(solution["x"], dtype=np.float64).reshape(-1)
        predicted_states, predicted_inputs = self._unpack_solution(decision)
        raw_command = VehicleInput(
            acceleration=float(predicted_inputs[0, 0]),
            steering_angle=float(predicted_inputs[1, 0]),
        )
        command = clip_input(raw_command, self.config.constraints)
        flags = _constraint_flags(predicted_states, predicted_inputs, self.config.constraints)
        objective_value = float(solution["f"]) if "f" in solution else None

        self._previous_input = np.array(command.as_tuple(), dtype=np.float64)
        self._warm_start = _shift_warm_start(predicted_states, predicted_inputs)

        return NMPCResult(
            command=command,
            predicted_states=predicted_states,
            predicted_inputs=predicted_inputs,
            solver_status=solver_status,
            solve_time_ms=solve_time_ms,
            objective_value=objective_value,
            feasible=True,
            backend=self.backend,
            constraint_violation_flags=flags,
        )

    def _build_solver(self) -> tuple[ca.Function, tuple[FloatArray, FloatArray], int]:
        config = self.config
        horizon = config.horizon
        model = create_casadi_kinematic_bicycle_model(config.wheelbase)

        states = ca.SX.sym("X", 4, horizon + 1)
        inputs = ca.SX.sym("U", 2, horizon)
        parameter_count = 4 + 4 * (horizon + 1) + 2
        parameters = ca.SX.sym("P", parameter_count)
        initial_state = parameters[0:4]
        reference = ca.reshape(parameters[4 : 4 + 4 * (horizon + 1)], 4, horizon + 1)
        previous_input = parameters[-2:]

        objective = 0
        constraints = [states[:, 0] - initial_state]
        last_input = previous_input

        for step in range(horizon):
            state_error = states[:, step] - reference[:, step]
            input_vector = inputs[:, step]
            input_rate = input_vector - last_input
            objective += _stage_cost(state_error, input_vector, input_rate, config)

            derivative = model.dynamics(states[:, step], input_vector)
            next_state = states[:, step] + config.dt * derivative
            constraints.append(states[:, step + 1] - next_state)
            last_input = input_vector

        terminal_error = states[:, horizon] - reference[:, horizon]
        objective += (
            config.q_position * terminal_error[0] ** 2
            + config.q_position * terminal_error[1] ** 2
            + config.q_heading * terminal_error[2] ** 2
            + config.q_velocity * terminal_error[3] ** 2
        )

        decision_variables = ca.vertcat(
            ca.reshape(states, -1, 1),
            ca.reshape(inputs, -1, 1),
        )
        nlp = {
            "x": decision_variables,
            "f": objective,
            "g": ca.vertcat(*constraints),
            "p": parameters,
        }
        solver = ca.nlpsol(
            "nmpc_solver",
            "ipopt",
            nlp,
            {
                "error_on_fail": False,
                "print_time": False,
                "ipopt.print_level": 0,
                "ipopt.sb": "yes",
                "ipopt.max_iter": config.max_solver_iterations,
            },
        )
        return solver, _build_variable_bounds(config), 4 * (horizon + 1)

    def _build_parameter_vector(self, state: VehicleState, reference: FloatArray) -> FloatArray:
        return np.concatenate(
            [
                np.array(state.as_tuple(), dtype=np.float64),
                reference.reshape(-1, order="F"),
                self._previous_input,
            ],
        )

    def _initial_guess(self, state: VehicleState, reference: FloatArray) -> FloatArray:
        if self._warm_start is not None:
            return self._warm_start.copy()

        state_guess = reference.copy()
        state_guess[:, 0] = np.array(state.as_tuple(), dtype=np.float64)
        input_guess = np.zeros((2, self.config.horizon), dtype=np.float64)
        return _pack_decision(state_guess, input_guess)

    def _unpack_solution(self, decision: FloatArray) -> tuple[FloatArray, FloatArray]:
        horizon = self.config.horizon
        state_count = 4 * (horizon + 1)
        predicted_states = decision[:state_count].reshape((4, horizon + 1), order="F")
        predicted_inputs = decision[state_count:].reshape((2, horizon), order="F")
        return predicted_states, predicted_inputs


class AcadosNMPCController:
    """API-compatible NMPC wrapper with a truthful CasADi fallback.

    The native acados Python stack requires the external acados build and templates. Until that
    toolchain is installed, this wrapper delegates to the working CasADi/IPOPT implementation while
    reporting the backend honestly in each result.
    """

    def __init__(
        self,
        config: NMPCConfig | None = None,
        *,
        allow_casadi_fallback: bool = True,
    ) -> None:
        if acados_available():
            raise NotImplementedError("Native acados backend is not wired yet")
        if not allow_casadi_fallback:
            raise AcadosBackendUnavailable("acados_template is not installed")

        self._controller = CasadiNMPCController(config)

    def reset(self) -> None:
        self._controller.reset()

    def compute_control(
        self,
        state: VehicleState,
        target: PathTrackingTarget,
        reference_states: FloatArray | None = None,
    ) -> NMPCResult:
        return self._controller.compute_control(state, target, reference_states)


class AcadosBackendUnavailable(RuntimeError):
    """Raised when the caller requires native acados but it is unavailable."""


def acados_available() -> bool:
    return find_spec("acados_template") is not None


def build_straight_reference(
    state: VehicleState,
    target: PathTrackingTarget,
    config: NMPCConfig,
) -> FloatArray:
    """Build a simple straight-line reference trajectory for early NMPC smoke tests."""

    state.validate()
    reference = np.zeros((4, config.horizon + 1), dtype=np.float64)
    for step in range(config.horizon + 1):
        time_s = step * config.dt
        reference[:, step] = np.array(
            [
                state.px + target.speed * time_s,
                target.lateral_position,
                target.heading,
                target.speed,
            ],
            dtype=np.float64,
        )
    return reference


def _stage_cost(
    state_error: ca.SX,
    input_vector: ca.SX,
    input_rate: ca.SX,
    config: NMPCConfig,
) -> ca.SX:
    return (
        config.q_position * state_error[0] ** 2
        + config.q_position * state_error[1] ** 2
        + config.q_heading * state_error[2] ** 2
        + config.q_velocity * state_error[3] ** 2
        + config.r_accel * input_vector[0] ** 2
        + config.r_steer * input_vector[1] ** 2
        + config.rd_accel * input_rate[0] ** 2
        + config.rd_steer * input_rate[1] ** 2
    )


def _build_variable_bounds(config: NMPCConfig) -> tuple[FloatArray, FloatArray]:
    horizon = config.horizon
    state_count = 4 * (horizon + 1)
    input_count = 2 * horizon
    lower_bounds = np.full((state_count + input_count,), -np.inf, dtype=np.float64)
    upper_bounds = np.full((state_count + input_count,), np.inf, dtype=np.float64)

    for step in range(horizon + 1):
        velocity_index = 3 + 4 * step
        lower_bounds[velocity_index] = config.constraints.velocity_min
        upper_bounds[velocity_index] = config.constraints.velocity_max

    input_offset = state_count
    for step in range(horizon):
        accel_index = input_offset + 2 * step
        steer_index = accel_index + 1
        lower_bounds[accel_index] = config.constraints.accel_min
        upper_bounds[accel_index] = config.constraints.accel_max
        lower_bounds[steer_index] = config.constraints.steer_min
        upper_bounds[steer_index] = config.constraints.steer_max

    return lower_bounds, upper_bounds


def _validate_reference(reference_states: FloatArray, horizon: int) -> FloatArray:
    reference = np.asarray(reference_states, dtype=np.float64)
    expected_shape = (4, horizon + 1)
    if reference.shape != expected_shape:
        raise ValueError(f"reference_states must have shape {expected_shape}")
    if not np.all(np.isfinite(reference)):
        raise ValueError("reference_states values must be finite")
    return reference


def _constraint_flags(
    states: FloatArray,
    inputs: FloatArray,
    constraints: VehicleConstraints,
    tolerance: float = 1e-5,
) -> tuple[str, ...]:
    flags: list[str] = []
    if np.any(states[3, :] < constraints.velocity_min - tolerance):
        flags.append("velocity_below_min")
    if np.any(states[3, :] > constraints.velocity_max + tolerance):
        flags.append("velocity_above_max")
    if np.any(inputs[0, :] < constraints.accel_min - tolerance):
        flags.append("acceleration_below_min")
    if np.any(inputs[0, :] > constraints.accel_max + tolerance):
        flags.append("acceleration_above_max")
    if np.any(inputs[1, :] < constraints.steer_min - tolerance):
        flags.append("steering_below_min")
    if np.any(inputs[1, :] > constraints.steer_max + tolerance):
        flags.append("steering_above_max")
    return tuple(flags)


def _shift_warm_start(states: FloatArray, inputs: FloatArray) -> FloatArray:
    shifted_states = np.column_stack([states[:, 1:], states[:, -1]])
    shifted_inputs = np.column_stack([inputs[:, 1:], inputs[:, -1]])
    return _pack_decision(shifted_states, shifted_inputs)


def _pack_decision(states: FloatArray, inputs: FloatArray) -> FloatArray:
    return np.concatenate([states.reshape(-1, order="F"), inputs.reshape(-1, order="F")])


def _failed_result(status: str, backend: NMPCBackend, solve_time_ms: float) -> NMPCResult:
    return NMPCResult(
        command=VehicleInput(acceleration=0.0, steering_angle=0.0),
        predicted_states=np.empty((4, 0), dtype=np.float64),
        predicted_inputs=np.empty((2, 0), dtype=np.float64),
        solver_status=status,
        solve_time_ms=solve_time_ms,
        objective_value=None,
        feasible=False,
        backend=backend,
        constraint_violation_flags=("solver_failed",),
    )


def _non_negative(value: float, name: str) -> None:
    if not isfinite(value):
        raise ValueError(f"{name} must be finite")
    if value < 0.0:
        raise ValueError(f"{name} must be non-negative")


__all__ = [
    "AcadosBackendUnavailable",
    "AcadosNMPCController",
    "CasadiNMPCController",
    "NMPCConfig",
    "NMPCResult",
    "acados_available",
    "build_straight_reference",
]
