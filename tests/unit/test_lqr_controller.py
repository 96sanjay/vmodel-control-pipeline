from __future__ import annotations

from math import isclose

import numpy as np
import pytest

from vcp.controllers import LQRController, LQRWeights, PathTrackingTarget, solve_discrete_lqr
from vcp.models import KinematicBicycleModel, LinearizedBicycleModel, VehicleState


def test_linearized_bicycle_matrices_match_expected_shape_and_values() -> None:
    model = LinearizedBicycleModel(nominal_velocity=4.0, wheelbase=2.0, dt=0.1)

    a_continuous, b_continuous = model.continuous_matrices()
    a_discrete, b_discrete = model.discrete_matrices()

    assert a_continuous.shape == (2, 2)
    assert b_continuous.shape == (2, 1)
    assert np.allclose(a_continuous, np.array([[0.0, 4.0], [0.0, 0.0]]))
    assert np.allclose(b_continuous, np.array([[0.0], [2.0]]))
    assert np.allclose(a_discrete, np.array([[1.0, 0.4], [0.0, 1.0]]))
    assert np.allclose(b_discrete, np.array([[0.0], [0.2]]))


def test_discrete_lqr_gain_stabilizes_linearized_model() -> None:
    model = LinearizedBicycleModel(nominal_velocity=4.0, wheelbase=2.8, dt=0.1)
    a_matrix, b_matrix = model.discrete_matrices()
    gain = solve_discrete_lqr(
        a_matrix,
        b_matrix,
        np.diag([4.0, 1.0]),
        np.array([[0.5]]),
    )

    eigenvalues = model.closed_loop_eigenvalues(gain)

    assert gain.shape == (1, 2)
    assert np.all(np.isfinite(gain))
    assert np.max(np.abs(eigenvalues)) < 1.0


def test_lqr_controller_steers_back_toward_path() -> None:
    model = LinearizedBicycleModel(nominal_velocity=4.0, wheelbase=2.8, dt=0.1)
    controller = LQRController(model)
    state = VehicleState(px=0.0, py=-1.0, yaw=0.0, v=4.0)
    target = PathTrackingTarget(speed=4.0, lateral_position=0.0)

    command, diagnostics = controller.compute_control(state, target)

    assert command.steering_angle > 0.0
    assert diagnostics.lateral_error < 0.0
    assert not diagnostics.acceleration_saturated
    assert np.max(np.abs(diagnostics.closed_loop_eigenvalues)) < 1.0


def test_lqr_controller_accelerates_when_below_target_speed() -> None:
    model = LinearizedBicycleModel(nominal_velocity=4.0, wheelbase=2.8, dt=0.1)
    controller = LQRController(model)
    state = VehicleState(px=0.0, py=0.0, yaw=0.0, v=1.0)
    target = PathTrackingTarget(speed=4.0, lateral_position=0.0)

    command, diagnostics = controller.compute_control(state, target)

    assert command.acceleration > 0.0
    assert diagnostics.speed_error == 3.0


def test_lqr_closed_loop_reduces_lateral_error_on_nonlinear_model() -> None:
    linear_model = LinearizedBicycleModel(nominal_velocity=4.0, wheelbase=2.8, dt=0.1)
    controller = LQRController(linear_model, weights=LQRWeights(q_lateral=3.0, q_heading=1.0))
    plant = KinematicBicycleModel()
    target = PathTrackingTarget(speed=4.0, lateral_position=0.0, heading=0.0)
    state = VehicleState(px=0.0, py=-1.0, yaw=0.0, v=4.0)
    initial_error = abs(state.py - target.lateral_position)

    for _ in range(40):
        command, _ = controller.compute_control(state, target)
        state = plant.step(state, command, dt=0.1)

    assert abs(state.py - target.lateral_position) < initial_error


def test_lqr_rejects_invalid_shapes_and_parameters() -> None:
    with pytest.raises(ValueError, match="nominal_velocity must be positive"):
        LinearizedBicycleModel(nominal_velocity=0.0, wheelbase=2.8, dt=0.1)

    with pytest.raises(ValueError, match="q_lateral must be positive"):
        LQRWeights(q_lateral=0.0).q_matrix()

    with pytest.raises(ValueError, match="a_matrix must be square"):
        solve_discrete_lqr(
            np.zeros((2, 3)),
            np.zeros((2, 1)),
            np.eye(2),
            np.eye(1),
        )


def test_lqr_gain_is_repeatable_for_same_configuration() -> None:
    model = LinearizedBicycleModel(nominal_velocity=4.0, wheelbase=2.8, dt=0.1)

    first = LQRController(model).gain
    second = LQRController(model).gain

    assert np.allclose(first, second)
    assert isclose(float(first[0, 0]), float(second[0, 0]))
