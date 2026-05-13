"""Controller implementations and interfaces."""

from vcp.controllers.lqr import LQRController, LQRDiagnostics, LQRWeights, solve_discrete_lqr
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
    "LQRController",
    "LQRDiagnostics",
    "LQRWeights",
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
