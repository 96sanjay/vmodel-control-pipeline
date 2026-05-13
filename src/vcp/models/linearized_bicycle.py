from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class LinearizedBicycleModel:
    """Lateral bicycle model linearized around a constant forward velocity.

    State vector: [lateral_error, heading_error]
    Input vector: [steering_angle]
    """

    nominal_velocity: float
    wheelbase: float
    dt: float

    def __post_init__(self) -> None:
        if self.nominal_velocity <= 0.0:
            raise ValueError("nominal_velocity must be positive")
        if self.wheelbase <= 0.0:
            raise ValueError("wheelbase must be positive")
        if self.dt <= 0.0:
            raise ValueError("dt must be positive")

    def continuous_matrices(self) -> tuple[FloatArray, FloatArray]:
        """Return continuous-time A and B matrices for lateral tracking."""

        a_matrix = np.array(
            [
                [0.0, self.nominal_velocity],
                [0.0, 0.0],
            ],
            dtype=np.float64,
        )
        b_matrix = np.array(
            [
                [0.0],
                [self.nominal_velocity / self.wheelbase],
            ],
            dtype=np.float64,
        )
        return a_matrix, b_matrix

    def discrete_matrices(self) -> tuple[FloatArray, FloatArray]:
        """Return Euler-discretized A and B matrices."""

        a_continuous, b_continuous = self.continuous_matrices()
        identity = np.eye(a_continuous.shape[0], dtype=np.float64)
        return identity + self.dt * a_continuous, self.dt * b_continuous

    def closed_loop_eigenvalues(self, gain: FloatArray) -> FloatArray:
        """Return eigenvalues of the discrete closed-loop lateral system."""

        a_matrix, b_matrix = self.discrete_matrices()
        return np.linalg.eigvals(a_matrix - b_matrix @ gain)
