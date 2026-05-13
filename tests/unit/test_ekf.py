from __future__ import annotations

import numpy as np

from vcp.estimators import ExtendedKalmanFilter
from vcp.models import KinematicBicycleModel, VehicleInput, VehicleState
from vcp.sensors import GaussianStateNoise, StateNoiseConfig


def test_ekf_reduces_noisy_measurement_rmse() -> None:
    true_states, commands = _simulate_truth()
    noise = GaussianStateNoise(
        StateNoiseConfig(px_std=0.25, py_std=0.25, yaw_std=0.08, v_std=0.20),
        seed=11,
    )
    measurements = [noise.sample(state) for state in true_states[1:]]
    ekf = _make_ekf(VehicleState(px=0.2, py=-0.2, yaw=0.05, v=0.1))
    estimates: list[np.ndarray] = []

    for command, measurement in zip(commands, measurements, strict=True):
        diagnostics = ekf.step(command, measurement, dt=0.1)
        estimates.append(np.array(diagnostics.state.as_tuple(), dtype=np.float64))

    truth = np.array([state.as_tuple() for state in true_states[1:]], dtype=np.float64)
    measurement_array = np.array(measurements, dtype=np.float64)
    estimate_array = np.array(estimates, dtype=np.float64)

    assert _state_rmse(estimate_array, truth) < _state_rmse(measurement_array, truth)


def test_ekf_biased_yaw_measurement_increases_residual_metric() -> None:
    true_states, commands = _simulate_truth()
    nominal_noise = GaussianStateNoise(
        StateNoiseConfig(px_std=0.05, py_std=0.05, yaw_std=0.01, v_std=0.05),
        seed=19,
    )
    biased_noise = GaussianStateNoise(
        StateNoiseConfig(px_std=0.05, py_std=0.05, yaw_std=0.01, v_std=0.05, yaw_bias=0.35),
        seed=19,
    )

    nominal_residuals = _run_residual_trace(nominal_noise, true_states, commands)
    biased_residuals = _run_residual_trace(biased_noise, true_states, commands)

    assert np.mean(biased_residuals) > 2.0 * np.mean(nominal_residuals)


def test_state_noise_covariance_and_bias_are_reported() -> None:
    config = StateNoiseConfig(px_std=1.0, py_std=2.0, yaw_std=3.0, v_std=4.0, yaw_bias=0.5)

    assert np.allclose(np.diag(config.covariance()), np.array([1.0, 4.0, 9.0, 16.0]))
    assert np.allclose(config.bias_vector(), np.array([0.0, 0.0, 0.5, 0.0]))


def _make_ekf(initial_state: VehicleState) -> ExtendedKalmanFilter:
    return ExtendedKalmanFilter(
        initial_state=initial_state,
        initial_covariance=np.diag([0.5, 0.5, 0.2, 0.5]),
        process_noise=np.diag([0.002, 0.002, 0.001, 0.005]),
        measurement_noise=np.diag([0.25**2, 0.25**2, 0.08**2, 0.20**2]),
    )


def _simulate_truth() -> tuple[list[VehicleState], list[VehicleInput]]:
    model = KinematicBicycleModel()
    state = VehicleState(px=0.0, py=0.0, yaw=0.0, v=2.0)
    states = [state]
    commands: list[VehicleInput] = []

    for step in range(80):
        command = VehicleInput(
            acceleration=0.2 if step < 30 else 0.0,
            steering_angle=0.08,
        )
        commands.append(command)
        state = model.step(state, command, dt=0.1)
        states.append(state)

    return states, commands


def _run_residual_trace(
    noise: GaussianStateNoise,
    true_states: list[VehicleState],
    commands: list[VehicleInput],
) -> list[float]:
    ekf = _make_ekf(true_states[0])
    residuals: list[float] = []

    for command, state in zip(commands, true_states[1:], strict=True):
        measurement = noise.sample(state)
        diagnostics = ekf.step(command, measurement, dt=0.1)
        residuals.append(abs(float(diagnostics.residual[2, 0])))

    return residuals


def _state_rmse(values: np.ndarray, references: np.ndarray) -> float:
    errors = values - references
    return float(np.sqrt(np.mean(errors * errors)))
