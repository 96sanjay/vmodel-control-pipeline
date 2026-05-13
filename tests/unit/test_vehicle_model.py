from __future__ import annotations

from math import isclose

import pytest

from vcp.models import (
    KinematicBicycleModel,
    VehicleConstraints,
    VehicleInput,
    VehicleParameters,
    VehicleState,
    check_input_constraints,
    check_state_constraints,
    check_steer_rate_constraint,
    clip_input,
)


def test_straight_motion_advances_position_and_velocity() -> None:
    model = KinematicBicycleModel()
    state = VehicleState(px=0.0, py=0.0, yaw=0.0, v=5.0)
    command = VehicleInput(acceleration=1.0, steering_angle=0.0)

    next_state = model.step(state, command, dt=0.1)

    assert isclose(next_state.px, 0.5)
    assert isclose(next_state.py, 0.0)
    assert isclose(next_state.yaw, 0.0)
    assert isclose(next_state.v, 5.1)


def test_turning_motion_changes_yaw() -> None:
    model = KinematicBicycleModel(VehicleParameters(wheelbase=2.5))
    state = VehicleState(px=0.0, py=0.0, yaw=0.0, v=4.0)
    command = VehicleInput(acceleration=0.0, steering_angle=0.2)

    next_state = model.step(state, command, dt=0.1)

    assert next_state.px > 0.0
    assert isclose(next_state.py, 0.0)
    assert next_state.yaw > 0.0
    assert isclose(next_state.v, 4.0)


def test_zero_velocity_with_acceleration_only_updates_speed() -> None:
    model = KinematicBicycleModel()
    state = VehicleState(px=2.0, py=3.0, yaw=1.0, v=0.0)
    command = VehicleInput(acceleration=1.5, steering_angle=0.3)

    next_state = model.step(state, command, dt=0.2)

    assert isclose(next_state.px, state.px)
    assert isclose(next_state.py, state.py)
    assert isclose(next_state.yaw, state.yaw)
    assert isclose(next_state.v, 0.3)


def test_simulate_returns_initial_and_all_propagated_states() -> None:
    model = KinematicBicycleModel()
    initial_state = VehicleState(px=0.0, py=0.0, yaw=0.0, v=1.0)
    commands = (
        VehicleInput(acceleration=0.0, steering_angle=0.0),
        VehicleInput(acceleration=0.0, steering_angle=0.0),
    )

    states = model.simulate(initial_state, commands, dt=0.5)

    assert len(states) == 3
    assert states[0] == initial_state
    assert isclose(states[-1].px, 1.0)


def test_input_constraint_violation_is_reported() -> None:
    constraints = VehicleConstraints(accel_min=-1.0, accel_max=1.0)
    command = VehicleInput(acceleration=2.0, steering_angle=0.0)

    result = check_input_constraints(command, constraints)

    assert not result.is_valid
    assert result.violations[0].name == "acceleration"


def test_state_constraint_violation_is_reported() -> None:
    constraints = VehicleConstraints(velocity_min=0.0, velocity_max=3.0)
    state = VehicleState(px=0.0, py=0.0, yaw=0.0, v=4.0)

    result = check_state_constraints(state, constraints)

    assert not result.is_valid
    assert result.violations[0].name == "velocity"


def test_model_rejects_command_constraint_violation() -> None:
    model = KinematicBicycleModel()
    state = VehicleState(px=0.0, py=0.0, yaw=0.0, v=1.0)
    command = VehicleInput(acceleration=99.0, steering_angle=0.0)

    with pytest.raises(ValueError, match="VehicleInput violates constraints"):
        model.step(state, command, dt=0.1)


def test_clip_input_saturates_acceleration_and_steering() -> None:
    constraints = VehicleConstraints(accel_min=-2.0, accel_max=1.0, steer_min=-0.2, steer_max=0.2)
    command = VehicleInput(acceleration=5.0, steering_angle=-1.0)

    clipped = clip_input(command, constraints)

    assert clipped == VehicleInput(acceleration=1.0, steering_angle=-0.2)


def test_steer_rate_constraint_detects_large_step() -> None:
    constraints = VehicleConstraints(steer_rate_max=0.5)
    previous_command = VehicleInput(acceleration=0.0, steering_angle=0.0)
    command = VehicleInput(acceleration=0.0, steering_angle=0.2)

    result = check_steer_rate_constraint(previous_command, command, dt=0.1, constraints=constraints)

    assert not result.is_valid
    assert result.violations[0].name == "steering_rate"


def test_invalid_parameters_and_timestep_raise_clear_errors() -> None:
    with pytest.raises(ValueError, match="wheelbase must be positive"):
        VehicleParameters(wheelbase=0.0)

    with pytest.raises(ValueError, match="velocity lower limit"):
        VehicleConstraints(velocity_min=1.0, velocity_max=1.0)

    model = KinematicBicycleModel()
    with pytest.raises(ValueError, match="dt must be positive"):
        model.step(
            VehicleState(px=0.0, py=0.0, yaw=0.0, v=1.0),
            VehicleInput(acceleration=0.0, steering_angle=0.0),
            dt=0.0,
        )
