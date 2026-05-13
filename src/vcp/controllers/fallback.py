from __future__ import annotations

from dataclasses import dataclass, field

from vcp.models import VehicleConstraints, VehicleInput, VehicleState, clip_input


@dataclass(frozen=True)
class FallbackBrakeConfig:
    """Configuration for deterministic fallback braking."""

    brake_acceleration: float = -4.0
    steering_angle: float = 0.0
    stopped_velocity_threshold: float = 0.05
    constraints: VehicleConstraints = field(default_factory=VehicleConstraints)

    def __post_init__(self) -> None:
        if self.stopped_velocity_threshold < 0.0:
            raise ValueError("stopped_velocity_threshold must be non-negative")


class FallbackBrakeController:
    """Deterministic fallback controller used when the primary controller is unsafe."""

    def __init__(self, config: FallbackBrakeConfig | None = None) -> None:
        self.config = config or FallbackBrakeConfig()

    def compute_control(self, state: VehicleState) -> VehicleInput:
        """Return a conservative braking command with neutral steering."""

        state.validate()
        acceleration = (
            0.0
            if state.v <= self.config.stopped_velocity_threshold
            else self.config.brake_acceleration
        )
        return clip_input(
            VehicleInput(
                acceleration=acceleration,
                steering_angle=self.config.steering_angle,
            ),
            self.config.constraints,
        )


__all__ = ["FallbackBrakeConfig", "FallbackBrakeController"]
