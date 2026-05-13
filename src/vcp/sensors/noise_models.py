from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from vcp.models import FloatArray, VehicleState


@dataclass(frozen=True)
class StateNoiseConfig:
    """Gaussian measurement-noise configuration for [px, py, yaw, v]."""

    px_std: float = 0.10
    py_std: float = 0.10
    yaw_std: float = 0.02
    v_std: float = 0.10
    px_bias: float = 0.0
    py_bias: float = 0.0
    yaw_bias: float = 0.0
    v_bias: float = 0.0

    def __post_init__(self) -> None:
        for name, value in (
            ("px_std", self.px_std),
            ("py_std", self.py_std),
            ("yaw_std", self.yaw_std),
            ("v_std", self.v_std),
        ):
            if value < 0.0:
                raise ValueError(f"{name} must be non-negative")

    def std_vector(self) -> FloatArray:
        return np.array([self.px_std, self.py_std, self.yaw_std, self.v_std], dtype=np.float64)

    def bias_vector(self) -> FloatArray:
        return np.array([self.px_bias, self.py_bias, self.yaw_bias, self.v_bias], dtype=np.float64)

    def covariance(self) -> FloatArray:
        std = self.std_vector()
        return np.diag(std * std).astype(np.float64)


class GaussianStateNoise:
    """Deterministic-seed Gaussian measurement model for vehicle state."""

    def __init__(self, config: StateNoiseConfig | None = None, *, seed: int | None = None) -> None:
        self.config = config or StateNoiseConfig()
        self._rng = np.random.default_rng(seed)

    def sample(self, state: VehicleState) -> FloatArray:
        """Return noisy measurement vector [px, py, yaw, v]."""

        true_state = np.array(state.as_tuple(), dtype=np.float64)
        noise = self._rng.normal(loc=0.0, scale=self.config.std_vector())
        return true_state + self.config.bias_vector() + noise

    def sample_state(self, state: VehicleState) -> VehicleState:
        """Return a noisy measurement as a VehicleState object."""

        measurement = self.sample(state)
        return VehicleState(
            px=float(measurement[0]),
            py=float(measurement[1]),
            yaw=float(measurement[2]),
            v=float(measurement[3]),
        )
