from __future__ import annotations

from dataclasses import dataclass, field
from math import cos, sin, tan

from vcp.models.constraints import VehicleConstraints, check_input_constraints
from vcp.models.vehicle_state import VehicleInput, VehicleState


@dataclass(frozen=True)
class VehicleParameters:
    """Physical and limit parameters for the kinematic bicycle model."""

    wheelbase: float = 2.8
    width: float = 1.8
    length: float = 4.5
    constraints: VehicleConstraints = field(default_factory=VehicleConstraints)

    def __post_init__(self) -> None:
        if self.wheelbase <= 0.0:
            raise ValueError("wheelbase must be positive")
        if self.width <= 0.0:
            raise ValueError("width must be positive")
        if self.length <= 0.0:
            raise ValueError("length must be positive")


class KinematicBicycleModel:
    """Simple kinematic bicycle plant model with Euler integration."""

    def __init__(self, parameters: VehicleParameters | None = None) -> None:
        self.parameters = parameters or VehicleParameters()

    def derivative(self, state: VehicleState, command: VehicleInput) -> VehicleState:
        """Compute continuous-time state derivative."""

        state.validate()
        command.validate()

        command_check = check_input_constraints(command, self.parameters.constraints)
        if not command_check.is_valid:
            violation_names = ", ".join(violation.name for violation in command_check.violations)
            raise ValueError(f"VehicleInput violates constraints: {violation_names}")

        return VehicleState(
            px=state.v * cos(state.yaw),
            py=state.v * sin(state.yaw),
            yaw=state.v / self.parameters.wheelbase * tan(command.steering_angle),
            v=command.acceleration,
        )

    def step(
        self,
        state: VehicleState,
        command: VehicleInput,
        dt: float,
    ) -> VehicleState:
        """Propagate the vehicle state forward with explicit Euler integration."""

        if dt <= 0.0:
            raise ValueError("dt must be positive")

        derivative = self.derivative(state, command)
        next_state = VehicleState(
            px=state.px + derivative.px * dt,
            py=state.py + derivative.py * dt,
            yaw=state.yaw + derivative.yaw * dt,
            v=state.v + derivative.v * dt,
        )
        next_state.validate()
        return next_state

    def simulate(
        self,
        initial_state: VehicleState,
        commands: list[VehicleInput] | tuple[VehicleInput, ...],
        dt: float,
    ) -> tuple[VehicleState, ...]:
        """Simulate a sequence of commands and return states including the initial state."""

        states = [initial_state]
        current_state = initial_state
        for command in commands:
            current_state = self.step(current_state, command, dt)
            states.append(current_state)
        return tuple(states)
