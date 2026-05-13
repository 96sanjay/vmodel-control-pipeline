"""Nonlinear MPC controllers and model-building utilities."""

from vcp.controllers.nmpc.acados_nmpc import (
    AcadosBackendUnavailable,
    AcadosNMPCController,
    CasadiNMPCController,
    NMPCConfig,
    NMPCResult,
    acados_available,
    build_straight_reference,
)
from vcp.controllers.nmpc.casadi_model import (
    CasadiKinematicBicycleModel,
    create_casadi_kinematic_bicycle_model,
)

__all__ = [
    "AcadosBackendUnavailable",
    "AcadosNMPCController",
    "CasadiKinematicBicycleModel",
    "CasadiNMPCController",
    "NMPCConfig",
    "NMPCResult",
    "acados_available",
    "build_straight_reference",
    "create_casadi_kinematic_bicycle_model",
]
