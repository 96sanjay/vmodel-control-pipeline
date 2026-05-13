from __future__ import annotations

import numpy as np
import pytest

from vcp.controllers import (
    AcadosBackendUnavailable,
    AcadosNMPCController,
    CasadiNMPCController,
    NMPCConfig,
    PathTrackingTarget,
)
from vcp.controllers.nmpc import build_straight_reference, create_casadi_kinematic_bicycle_model
from vcp.controllers.nmpc.acados_nmpc import acados_available
from vcp.models import VehicleConstraints, VehicleState


def test_casadi_kinematic_bicycle_model_derivative() -> None:
    model = create_casadi_kinematic_bicycle_model(wheelbase=2.8)

    derivative = np.asarray(model.dynamics([0.0, 0.0, 0.0, 4.0], [1.0, 0.0])).reshape(-1)

    np.testing.assert_allclose(derivative, [4.0, 0.0, 0.0, 1.0], atol=1e-9)


def test_nmpc_solves_straight_line_tracking_problem() -> None:
    controller = CasadiNMPCController(NMPCConfig(horizon=6, max_solver_iterations=80))
    state = VehicleState(px=0.0, py=-0.4, yaw=0.0, v=3.0)
    target = PathTrackingTarget(speed=4.0, lateral_position=0.0, heading=0.0)

    result = controller.compute_control(state, target)

    assert result.feasible
    assert result.backend == "casadi_ipopt"
    assert result.predicted_states.shape == (4, controller.config.horizon + 1)
    assert result.predicted_inputs.shape == (2, controller.config.horizon)
    assert result.constraint_violation_flags == ()
    assert controller.config.constraints.accel_min <= result.command.acceleration
    assert result.command.acceleration <= controller.config.constraints.accel_max
    assert controller.config.constraints.steer_min <= result.command.steering_angle
    assert result.command.steering_angle <= controller.config.constraints.steer_max
    assert result.solve_time_ms >= 0.0


def test_nmpc_warm_start_supports_repeated_solves() -> None:
    controller = CasadiNMPCController(NMPCConfig(horizon=5, max_solver_iterations=80))
    target = PathTrackingTarget(speed=3.0, lateral_position=0.0, heading=0.0)
    first_state = VehicleState(px=0.0, py=-0.2, yaw=0.02, v=2.5)
    second_state = VehicleState(px=0.25, py=-0.18, yaw=0.01, v=2.7)

    first_result = controller.compute_control(first_state, target)
    second_result = controller.compute_control(second_state, target)

    assert first_result.feasible
    assert second_result.feasible
    assert second_result.predicted_inputs.shape == (2, controller.config.horizon)


def test_nmpc_reports_infeasible_velocity_case() -> None:
    constraints = VehicleConstraints(
        velocity_min=10.0,
        velocity_max=11.0,
        accel_min=-1.0,
        accel_max=1.0,
    )
    controller = CasadiNMPCController(
        NMPCConfig(horizon=1, dt=0.1, constraints=constraints, max_solver_iterations=30),
    )
    state = VehicleState(px=0.0, py=0.0, yaw=0.0, v=0.0)
    target = PathTrackingTarget(speed=0.0, lateral_position=0.0, heading=0.0)

    result = controller.compute_control(state, target)

    assert not result.feasible
    assert result.command.acceleration == 0.0
    assert result.command.steering_angle == 0.0
    assert result.constraint_violation_flags == ("solver_failed",)


def test_nmpc_validates_reference_shape() -> None:
    controller = CasadiNMPCController(NMPCConfig(horizon=3))
    state = VehicleState(px=0.0, py=0.0, yaw=0.0, v=2.0)
    target = PathTrackingTarget(speed=2.0)

    with pytest.raises(ValueError, match="reference_states must have shape"):
        controller.compute_control(state, target, reference_states=np.zeros((4, 3)))


def test_build_straight_reference_shape_and_values() -> None:
    config = NMPCConfig(horizon=4, dt=0.2)
    state = VehicleState(px=1.0, py=2.0, yaw=0.0, v=1.0)
    target = PathTrackingTarget(speed=5.0, lateral_position=0.5, heading=0.1)

    reference = build_straight_reference(state, target, config)

    assert reference.shape == (4, 5)
    np.testing.assert_allclose(reference[:, 0], [1.0, 0.5, 0.1, 5.0])
    np.testing.assert_allclose(reference[:, -1], [5.0, 0.5, 0.1, 5.0])


def test_acados_wrapper_requires_backend_when_fallback_disabled() -> None:
    if acados_available():
        pytest.skip("native acados backend is installed in this environment")

    with pytest.raises(AcadosBackendUnavailable):
        AcadosNMPCController(allow_casadi_fallback=False)
