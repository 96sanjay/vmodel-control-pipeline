from __future__ import annotations

from dataclasses import dataclass

import casadi as ca


@dataclass(frozen=True)
class CasadiKinematicBicycleModel:
    """CasADi symbolic kinematic bicycle model used by the NMPC problem."""

    state: ca.SX
    control: ca.SX
    dynamics: ca.Function


def create_casadi_kinematic_bicycle_model(wheelbase: float) -> CasadiKinematicBicycleModel:
    """Create symbolic dynamics for states [px, py, yaw, v] and inputs [a, delta]."""

    if wheelbase <= 0.0:
        raise ValueError("wheelbase must be positive")

    state = ca.SX.sym("x", 4)
    control = ca.SX.sym("u", 2)
    yaw, velocity = state[2], state[3]
    acceleration, steering_angle = control[0], control[1]

    derivative = ca.vertcat(
        velocity * ca.cos(yaw),
        velocity * ca.sin(yaw),
        velocity / wheelbase * ca.tan(steering_angle),
        acceleration,
    )
    dynamics = ca.Function(
        "kinematic_bicycle_dynamics",
        [state, control],
        [derivative],
        ["state", "control"],
        ["state_derivative"],
    )
    return CasadiKinematicBicycleModel(state=state, control=control, dynamics=dynamics)


__all__ = ["CasadiKinematicBicycleModel", "create_casadi_kinematic_bicycle_model"]
