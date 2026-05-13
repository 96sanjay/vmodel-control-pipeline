from __future__ import annotations

from dataclasses import dataclass
from math import isfinite


@dataclass(frozen=True)
class VehicleState:
    """Planar vehicle state used by the kinematic bicycle model."""

    px: float
    py: float
    yaw: float
    v: float

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (self.px, self.py, self.yaw, self.v)

    def validate(self) -> None:
        values = self.as_tuple()
        if not all(isfinite(value) for value in values):
            raise ValueError(f"VehicleState values must be finite: {values}")


@dataclass(frozen=True)
class VehicleInput:
    """Vehicle command input for longitudinal acceleration and steering."""

    acceleration: float
    steering_angle: float

    def as_tuple(self) -> tuple[float, float]:
        return (self.acceleration, self.steering_angle)

    def validate(self) -> None:
        values = self.as_tuple()
        if not all(isfinite(value) for value in values):
            raise ValueError(f"VehicleInput values must be finite: {values}")
