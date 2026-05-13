from __future__ import annotations

import numpy as np
import pytest

from vcp.estimators import LinearKalmanFilter


def test_linear_kalman_filter_reduces_position_rmse() -> None:
    dt = 0.1
    steps = 80
    transition = np.array([[1.0, dt], [0.0, 1.0]], dtype=np.float64)
    measurement_matrix = np.array([[1.0, 0.0]], dtype=np.float64)
    filter_ = LinearKalmanFilter(
        state=np.array([0.0, 0.0], dtype=np.float64),
        covariance=np.eye(2, dtype=np.float64),
        transition_matrix=transition,
        process_noise=np.diag([1e-4, 1e-3]),
        measurement_matrix=measurement_matrix,
        measurement_noise=np.array([[0.25]], dtype=np.float64),
    )
    rng = np.random.default_rng(7)
    true_positions: list[float] = []
    measurements: list[float] = []
    estimates: list[float] = []

    position = 0.0
    velocity = 1.0
    for _ in range(steps):
        position += velocity * dt
        measurement = position + rng.normal(0.0, 0.5)
        filter_.predict()
        filter_.update(np.array([measurement], dtype=np.float64))

        true_positions.append(position)
        measurements.append(measurement)
        estimates.append(float(filter_.state[0, 0]))

    assert _rmse(estimates, true_positions) < _rmse(measurements, true_positions)


def test_linear_kalman_filter_reports_residual_and_nis() -> None:
    filter_ = LinearKalmanFilter(
        state=np.array([0.0], dtype=np.float64),
        covariance=np.array([[1.0]], dtype=np.float64),
        transition_matrix=np.array([[1.0]], dtype=np.float64),
        process_noise=np.array([[0.1]], dtype=np.float64),
        measurement_matrix=np.array([[1.0]], dtype=np.float64),
        measurement_noise=np.array([[0.2]], dtype=np.float64),
    )

    filter_.predict()
    diagnostics = filter_.update(np.array([1.0], dtype=np.float64))

    assert diagnostics.residual.shape == (1, 1)
    assert diagnostics.kalman_gain.shape == (1, 1)
    assert diagnostics.normalized_innovation_squared > 0.0


def test_linear_kalman_filter_validates_shapes_and_control_matrix() -> None:
    with pytest.raises(ValueError, match="covariance must be a square matrix"):
        LinearKalmanFilter(
            state=np.array([0.0, 0.0], dtype=np.float64),
            covariance=np.zeros((2, 3), dtype=np.float64),
            transition_matrix=np.eye(2, dtype=np.float64),
            process_noise=np.eye(2, dtype=np.float64),
            measurement_matrix=np.eye(2, dtype=np.float64),
            measurement_noise=np.eye(2, dtype=np.float64),
        )

    filter_ = LinearKalmanFilter(
        state=np.array([0.0], dtype=np.float64),
        covariance=np.eye(1, dtype=np.float64),
        transition_matrix=np.eye(1, dtype=np.float64),
        process_noise=np.eye(1, dtype=np.float64),
        measurement_matrix=np.eye(1, dtype=np.float64),
        measurement_noise=np.eye(1, dtype=np.float64),
    )
    with pytest.raises(ValueError, match="control_matrix is required"):
        filter_.predict(control=np.array([1.0], dtype=np.float64))


def _rmse(values: list[float], references: list[float]) -> float:
    errors = np.array(values, dtype=np.float64) - np.array(references, dtype=np.float64)
    return float(np.sqrt(np.mean(errors * errors)))
