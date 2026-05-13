from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from vcp.models import FloatArray


@dataclass(frozen=True)
class KalmanDiagnostics:
    """Diagnostics from a Kalman measurement update."""

    residual: FloatArray
    innovation_covariance: FloatArray
    kalman_gain: FloatArray
    normalized_innovation_squared: float

    @property
    def residual_norm(self) -> float:
        return float(np.linalg.norm(self.residual))


class LinearKalmanFilter:
    """Generic discrete-time linear Kalman filter."""

    def __init__(
        self,
        *,
        state: FloatArray,
        covariance: FloatArray,
        transition_matrix: FloatArray,
        process_noise: FloatArray,
        measurement_matrix: FloatArray,
        measurement_noise: FloatArray,
        control_matrix: FloatArray | None = None,
    ) -> None:
        self.state = _as_column(state, "state")
        self.covariance = _as_square(covariance, "covariance")
        self.transition_matrix = _as_square(transition_matrix, "transition_matrix")
        self.process_noise = _as_square(process_noise, "process_noise")
        self.measurement_matrix = np.asarray(measurement_matrix, dtype=np.float64)
        self.measurement_noise = _as_square(measurement_noise, "measurement_noise")
        self.control_matrix = (
            None
            if control_matrix is None
            else np.asarray(control_matrix, dtype=np.float64)
        )
        self._validate_shapes()

    def predict(self, control: FloatArray | None = None) -> FloatArray:
        """Run the prediction step and return the predicted state."""

        if control is not None:
            if self.control_matrix is None:
                raise ValueError("control_matrix is required when control is provided")
            control_column = _as_column(control, "control")
            self.state = self.transition_matrix @ self.state + self.control_matrix @ control_column
        else:
            self.state = self.transition_matrix @ self.state

        self.covariance = (
            self.transition_matrix @ self.covariance @ self.transition_matrix.T
            + self.process_noise
        )
        self.covariance = _symmetrize(self.covariance)
        return self.state.copy()

    def update(self, measurement: FloatArray) -> KalmanDiagnostics:
        """Run the measurement update and return innovation diagnostics."""

        measurement_column = _as_column(measurement, "measurement")
        residual = measurement_column - self.measurement_matrix @ self.state
        return self.update_with_residual(residual)

    def update_with_residual(self, residual: FloatArray) -> KalmanDiagnostics:
        """Run the update with a caller-provided residual.

        EKF uses this to normalize angular residuals before applying the common Kalman update.
        """

        residual_column = _as_column(residual, "residual")
        innovation_covariance = (
            self.measurement_matrix @ self.covariance @ self.measurement_matrix.T
            + self.measurement_noise
        )
        kalman_gain = (
            self.covariance
            @ self.measurement_matrix.T
            @ np.linalg.inv(innovation_covariance)
        )
        self.state = self.state + kalman_gain @ residual_column

        identity = np.eye(self.covariance.shape[0], dtype=np.float64)
        update_matrix = identity - kalman_gain @ self.measurement_matrix
        self.covariance = (
            update_matrix @ self.covariance @ update_matrix.T
            + kalman_gain @ self.measurement_noise @ kalman_gain.T
        )
        self.covariance = _symmetrize(self.covariance)

        nis_matrix = residual_column.T @ np.linalg.inv(innovation_covariance) @ residual_column
        normalized_innovation_squared = float(nis_matrix[0, 0])
        return KalmanDiagnostics(
            residual=residual_column.copy(),
            innovation_covariance=innovation_covariance,
            kalman_gain=kalman_gain,
            normalized_innovation_squared=normalized_innovation_squared,
        )

    def reset(self, state: FloatArray, covariance: FloatArray) -> None:
        self.state = _as_column(state, "state")
        self.covariance = _as_square(covariance, "covariance")
        self._validate_shapes()

    def _validate_shapes(self) -> None:
        state_dim = self.state.shape[0]
        measurement_dim = self.measurement_matrix.shape[0]
        if self.covariance.shape != (state_dim, state_dim):
            raise ValueError("covariance shape must match state dimension")
        if self.transition_matrix.shape != (state_dim, state_dim):
            raise ValueError("transition_matrix shape must match state dimension")
        if self.process_noise.shape != (state_dim, state_dim):
            raise ValueError("process_noise shape must match state dimension")
        if self.measurement_matrix.shape[1] != state_dim:
            raise ValueError("measurement_matrix column count must match state dimension")
        if self.measurement_noise.shape != (measurement_dim, measurement_dim):
            raise ValueError("measurement_noise shape must match measurement dimension")
        if self.control_matrix is not None and self.control_matrix.shape[0] != state_dim:
            raise ValueError("control_matrix row count must match state dimension")


def _as_column(value: FloatArray, name: str) -> FloatArray:
    array = np.asarray(value, dtype=np.float64)
    if array.ndim == 1:
        return array.reshape((-1, 1))
    if array.ndim == 2 and array.shape[1] == 1:
        return array
    raise ValueError(f"{name} must be a vector or column vector")


def _as_square(value: FloatArray, name: str) -> FloatArray:
    array = np.asarray(value, dtype=np.float64)
    if array.ndim != 2 or array.shape[0] != array.shape[1]:
        raise ValueError(f"{name} must be a square matrix")
    return array


def _symmetrize(matrix: FloatArray) -> FloatArray:
    return 0.5 * (matrix + matrix.T)
