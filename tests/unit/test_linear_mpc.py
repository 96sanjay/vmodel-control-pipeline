from __future__ import annotations

import numpy as np
import pytest

from vcp.controllers import LinearMPCConfig, LinearMPCController, PathTrackingTarget
from vcp.models import KinematicBicycleModel, VehicleConstraints, VehicleState


def test_linear_mpc_solves_feasible_tracking_case() -> None:
    controller = LinearMPCController()
    state = VehicleState(px=0.0, py=-0.5, yaw=0.0, v=3.0)
    target = PathTrackingTarget(speed=4.0, lateral_position=0.0)

    result = controller.compute_control(state, target)

    assert result.feasible
    assert result.solver_status in {"optimal", "optimal_inaccurate"}
    assert result.predicted_states.shape == (3, controller.config.horizon + 1)
    assert result.predicted_inputs.shape == (2, controller.config.horizon)
    assert result.solve_time_ms >= 0.0


def test_linear_mpc_enforces_input_constraints() -> None:
    constraints = VehicleConstraints(accel_min=-1.0, accel_max=0.5, steer_min=-0.1, steer_max=0.1)
    controller = LinearMPCController(LinearMPCConfig(constraints=constraints, horizon=8))
    state = VehicleState(px=0.0, py=-5.0, yaw=0.0, v=0.0)
    target = PathTrackingTarget(speed=10.0, lateral_position=0.0)

    result = controller.compute_control(state, target)

    assert result.feasible
    assert constraints.accel_min <= result.command.acceleration <= constraints.accel_max
    assert constraints.steer_min <= result.command.steering_angle <= constraints.steer_max
    assert np.all(result.predicted_inputs[0, :] <= constraints.accel_max + 1e-6)
    assert np.all(result.predicted_inputs[1, :] <= constraints.steer_max + 1e-6)


def test_linear_mpc_reports_infeasible_velocity_case() -> None:
    constraints = VehicleConstraints(
        velocity_min=10.0,
        velocity_max=11.0,
        accel_min=-1.0,
        accel_max=1.0,
    )
    controller = LinearMPCController(
        LinearMPCConfig(horizon=1, dt=0.1, constraints=constraints),
    )
    state = VehicleState(px=0.0, py=0.0, yaw=0.0, v=0.0)
    target = PathTrackingTarget(speed=0.0, lateral_position=0.0)

    result = controller.compute_control(state, target)

    assert not result.feasible
    assert result.command.acceleration == 0.0
    assert result.command.steering_angle == 0.0
    assert "infeasible" in result.solver_status


def test_linear_mpc_closed_loop_reduces_lateral_error() -> None:
    controller = LinearMPCController(LinearMPCConfig(horizon=10, nominal_velocity=4.0))
    plant = KinematicBicycleModel()
    target = PathTrackingTarget(speed=4.0, lateral_position=0.0, heading=0.0)
    state = VehicleState(px=0.0, py=-1.0, yaw=0.0, v=4.0)
    initial_error = abs(state.py - target.lateral_position)

    for _ in range(30):
        result = controller.compute_control(state, target)
        assert result.feasible
        state = plant.step(state, result.command, dt=0.1)

    assert abs(state.py - target.lateral_position) < initial_error


def test_linear_mpc_validates_config() -> None:
    with pytest.raises(ValueError, match="horizon must be positive"):
        LinearMPCConfig(horizon=0)

    with pytest.raises(ValueError, match="nominal_velocity must be positive"):
        LinearMPCConfig(nominal_velocity=0.0)

    with pytest.raises(ValueError, match="q_lateral must be non-negative"):
        LinearMPCConfig(q_lateral=-1.0)
