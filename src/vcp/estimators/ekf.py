from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, sin, tan

import numpy as np

from vcp.estimators.kalman import KalmanDiagnostics, LinearKalmanFilter
from vcp.models import (
    FloatArray,
    KinematicBicycleModel,
    VehicleInput,
    VehicleParameters,
    VehicleState,
)


@dataclass(frozen=True)
class EKFDiagnostics:
    """Diagnostics from a nonlinear prediction and measurement update."""

    state: VehicleState
    residual: FloatArray
    residual_norm: float
    normalized_innovation_squared: float


class ExtendedKalmanFilter:
    """EKF for the kinematic bicycle state [px, py, yaw, v]."""

    def __init__(
        self,
        *,
        initial_state: VehicleState,
        initial_covariance: FloatArray,
        process_noise: FloatArray,
        measurement_noise: FloatArray,
        parameters: VehicleParameters | None = None,
    ) -> None:
        self.parameters = parameters or VehicleParameters()
        self.model = KinematicBicycleModel(self.parameters)
        self._filter = LinearKalmanFilter(
            state=np.array(initial_state.as_tuple(), dtype=np.float64),
            covariance=initial_covariance,
            transition_matrix=np.eye(4, dtype=np.float64),
            process_noise=process_noise,
            measurement_matrix=np.eye(4, dtype=np.float64),
            measurement_noise=measurement_noise,
        )

    @property
    def state_vector(self) -> FloatArray:
        return self._filter.state.copy()

    @property
    def covariance(self) -> FloatArray:
        return self._filter.covariance.copy()

    @property
    def state(self) -> VehicleState:
        vector = self._filter.state[:, 0]
        return VehicleState(
            px=float(vector[0]),
            py=float(vector[1]),
            yaw=float(vector[2]),
            v=float(vector[3]),
        )

    def predict(self, command: VehicleInput, dt: float) -> VehicleState:
        """Predict vehicle state using nonlinear bicycle dynamics and Jacobian covariance update."""

        current_state = self.state
        predicted_state = self.model.step(current_state, command, dt)
        jacobian = self._state_transition_jacobian(current_state, command, dt)
        self._filter.transition_matrix = jacobian
        self._filter.state = np.array(predicted_state.as_tuple(), dtype=np.float64).reshape((4, 1))
        self._filter.covariance = (
            jacobian @ self._filter.covariance @ jacobian.T + self._filter.process_noise
        )
        self._filter.covariance = 0.5 * (self._filter.covariance + self._filter.covariance.T)
        return predicted_state

    def update(self, measurement: FloatArray) -> EKFDiagnostics:
        """Update from measurement vector [px, py, yaw, v]."""

        measurement_column = np.asarray(measurement, dtype=np.float64).reshape((4, 1))
        residual = measurement_column - self._filter.measurement_matrix @ self._filter.state
        residual[2, 0] = normalize_angle(float(residual[2, 0]))
        kalman_diagnostics = self._filter.update_with_residual(residual)
        self._filter.state[2, 0] = normalize_angle(float(self._filter.state[2, 0]))
        return self._diagnostics(kalman_diagnostics)

    def step(self, command: VehicleInput, measurement: FloatArray, dt: float) -> EKFDiagnostics:
        """Run predict and update in one call."""

        self.predict(command, dt)
        return self.update(measurement)

    def reset(self, state: VehicleState, covariance: FloatArray) -> None:
        self._filter.reset(np.array(state.as_tuple(), dtype=np.float64), covariance)

    def _state_transition_jacobian(
        self,
        state: VehicleState,
        command: VehicleInput,
        dt: float,
    ) -> FloatArray:
        wheelbase = self.parameters.wheelbase
        return np.array(
            [
                [1.0, 0.0, -state.v * sin(state.yaw) * dt, cos(state.yaw) * dt],
                [0.0, 1.0, state.v * cos(state.yaw) * dt, sin(state.yaw) * dt],
                [0.0, 0.0, 1.0, tan(command.steering_angle) / wheelbase * dt],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def _diagnostics(self, diagnostics: KalmanDiagnostics) -> EKFDiagnostics:
        return EKFDiagnostics(
            state=self.state,
            residual=diagnostics.residual,
            residual_norm=diagnostics.residual_norm,
            normalized_innovation_squared=diagnostics.normalized_innovation_squared,
        )


def normalize_angle(angle: float) -> float:
    return atan2(sin(angle), cos(angle))


__all__ = [
    "EKFDiagnostics",
    "ExtendedKalmanFilter",
    "normalize_angle",
]
