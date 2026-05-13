"""Controller implementations and interfaces."""

from vcp.controllers.fallback import FallbackBrakeConfig, FallbackBrakeController
from vcp.controllers.lqr import LQRController, LQRDiagnostics, LQRWeights, solve_discrete_lqr
from vcp.controllers.mpc import LinearMPCConfig, LinearMPCController, LinearMPCResult
from vcp.controllers.nmpc import (
    AcadosBackendUnavailable,
    AcadosNMPCController,
    CasadiNMPCController,
    NMPCConfig,
    NMPCResult,
)
from vcp.controllers.pid import (
    PID,
    PathTrackingTarget,
    PIDDiagnostics,
    PIDLimits,
    VehiclePIDController,
    VehiclePIDDiagnostics,
    create_default_vehicle_pid,
    normalize_angle,
)

__all__ = [
    "AcadosBackendUnavailable",
    "AcadosNMPCController",
    "CasadiNMPCController",
    "FallbackBrakeConfig",
    "FallbackBrakeController",
    "LQRController",
    "LQRDiagnostics",
    "LQRWeights",
    "LinearMPCConfig",
    "LinearMPCController",
    "LinearMPCResult",
    "NMPCConfig",
    "NMPCResult",
    "PID",
    "PIDDiagnostics",
    "PIDLimits",
    "PathTrackingTarget",
    "VehiclePIDController",
    "VehiclePIDDiagnostics",
    "create_default_vehicle_pid",
    "normalize_angle",
    "solve_discrete_lqr",
]
