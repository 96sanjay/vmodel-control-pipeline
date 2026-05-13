from __future__ import annotations

from math import isclose, pi

import pytest

from vcp.controllers import (
    PID,
    PathTrackingTarget,
    PIDLimits,
    VehiclePIDController,
    create_default_vehicle_pid,
    normalize_angle,
)
from vcp.models import KinematicBicycleModel, VehicleInput, VehicleState


def test_pid_proportional_response() -> None:
    pid = PID(kp=2.0)

    output = pid.update(error=3.0, dt=0.1)

    assert isclose(output, 6.0)
    assert isclose(pid.diagnostics.error, 3.0)


def test_pid_integral_accumulation() -> None:
    pid = PID(kp=0.0, ki=2.0)

    first = pid.update(error=1.0, dt=0.5)
    second = pid.update(error=1.0, dt=0.5)

    assert isclose(first, 1.0)
    assert isclose(second, 2.0)
    assert isclose(pid.integral, 1.0)


def test_pid_derivative_response() -> None:
    pid = PID(kp=0.0, kd=2.0)

    first = pid.update(error=0.0, dt=0.5)
    second = pid.update(error=3.0, dt=0.5)

    assert isclose(first, 0.0)
    assert isclose(second, 12.0)
    assert isclose(pid.diagnostics.derivative, 6.0)


def test_pid_output_saturation() -> None:
    pid = PID(kp=10.0, output_limits=PIDLimits(lower=-1.0, upper=1.0))

    output = pid.update(error=1.0, dt=0.1)

    assert isclose(output, 1.0)
    assert pid.diagnostics.saturated
    assert isclose(pid.diagnostics.unsaturated_output, 10.0)


def test_pid_anti_windup_blocks_integral_when_saturated() -> None:
    pid = PID(
        kp=10.0,
        ki=1.0,
        output_limits=PIDLimits(lower=-1.0, upper=1.0),
        anti_windup=True,
    )

    output = pid.update(error=5.0, dt=1.0)

    assert isclose(output, 1.0)
    assert isclose(pid.integral, 0.0)


def test_pid_can_disable_anti_windup() -> None:
    pid = PID(
        kp=10.0,
        ki=1.0,
        output_limits=PIDLimits(lower=-1.0, upper=1.0),
        anti_windup=False,
    )

    output = pid.update(error=5.0, dt=1.0)

    assert isclose(output, 1.0)
    assert isclose(pid.integral, 5.0)


def test_pid_reset_clears_state() -> None:
    pid = PID(kp=1.0, ki=1.0)
    pid.update(error=2.0, dt=1.0)

    pid.reset()

    assert isclose(pid.integral, 0.0)
    assert isclose(pid.diagnostics.output, 0.0)


def test_pid_rejects_invalid_limits_and_timestep() -> None:
    with pytest.raises(ValueError, match="lower limit"):
        PIDLimits(lower=1.0, upper=1.0)

    pid = PID(kp=1.0)
    with pytest.raises(ValueError, match="dt must be positive"):
        pid.update(error=1.0, dt=0.0)


def test_vehicle_pid_outputs_acceleration_for_speed_error() -> None:
    controller = create_default_vehicle_pid()
    state = VehicleState(px=0.0, py=0.0, yaw=0.0, v=2.0)
    target = PathTrackingTarget(speed=5.0)

    command, diagnostics = controller.compute_control(state, target, dt=0.1)

    assert command.acceleration > 0.0
    assert isclose(command.steering_angle, 0.0)
    assert isclose(diagnostics.speed_error, 3.0)


def test_vehicle_pid_outputs_steering_for_lateral_error() -> None:
    controller = create_default_vehicle_pid()
    state = VehicleState(px=0.0, py=-1.0, yaw=0.0, v=4.0)
    target = PathTrackingTarget(speed=4.0, lateral_position=0.0)

    command, diagnostics = controller.compute_control(state, target, dt=0.1)

    assert command.steering_angle > 0.0
    assert diagnostics.lateral_error > 0.0


def test_vehicle_pid_reports_command_saturation() -> None:
    controller = VehiclePIDController(
        speed_pid=PID(kp=10.0, output_limits=PIDLimits(lower=-1.0, upper=1.0)),
        lateral_pid=PID(kp=10.0, output_limits=PIDLimits(lower=-0.1, upper=0.1)),
    )
    state = VehicleState(px=0.0, py=-10.0, yaw=0.0, v=0.0)
    target = PathTrackingTarget(speed=20.0, lateral_position=0.0)

    command, diagnostics = controller.compute_control(state, target, dt=0.1)

    assert command == VehicleInput(acceleration=1.0, steering_angle=0.1)
    assert diagnostics.command_saturated


def test_vehicle_pid_closed_loop_reduces_lateral_error() -> None:
    controller = create_default_vehicle_pid()
    model = KinematicBicycleModel()
    state = VehicleState(px=0.0, py=-1.0, yaw=0.0, v=4.0)
    target = PathTrackingTarget(speed=4.0, lateral_position=0.0)

    initial_abs_error = abs(target.lateral_position - state.py)
    for _ in range(30):
        command, _ = controller.compute_control(state, target, dt=0.1)
        state = model.step(state, command, dt=0.1)

    assert abs(target.lateral_position - state.py) < initial_abs_error


def test_normalize_angle_wraps_to_pi_range() -> None:
    assert isclose(normalize_angle(3.0 * pi), pi, abs_tol=1e-12)
    assert isclose(normalize_angle(-3.0 * pi), -pi, abs_tol=1e-12)
