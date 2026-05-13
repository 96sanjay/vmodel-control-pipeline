from __future__ import annotations

from dataclasses import asdict, dataclass, is_dataclass
from pathlib import Path
from time import perf_counter
from typing import Any, Protocol

from vcp.controllers import (
    CasadiNMPCController,
    LinearMPCController,
    LQRController,
    PathTrackingTarget,
    VehiclePIDController,
)
from vcp.models import FloatArray, VehicleInput, VehicleState


@dataclass(frozen=True)
class ControllerStepInput:
    """Input packet passed through the stable controller interface."""

    timestamp_s: float
    dt: float
    state: VehicleState
    target: PathTrackingTarget
    reference_states: FloatArray | None = None

    def __post_init__(self) -> None:
        if self.dt <= 0.0:
            raise ValueError("dt must be positive")
        self.state.validate()


@dataclass(frozen=True)
class ControllerStepOutput:
    """Output packet returned by a controller adapter."""

    command: VehicleInput
    solver_status: str
    solve_time_ms: float
    feasible: bool
    predicted_states: FloatArray | None = None
    predicted_inputs: FloatArray | None = None
    diagnostics: dict[str, Any] | None = None


class ControllerInterface(Protocol):
    """Stable controller interface reused by MIL, SIL, and HIL-lite stages."""

    def initialize(self, metadata: dict[str, Any] | None = None) -> None:
        """Prepare the controller for a run."""

    def step(self, step_input: ControllerStepInput) -> ControllerStepOutput:
        """Compute one control command from an interface input packet."""

    def reset(self) -> None:
        """Reset controller state."""

    def get_diagnostics(self) -> dict[str, Any]:
        """Return diagnostics from the latest step."""


class PythonControllerAdapter:
    """Adapter that exposes current Python controllers through ControllerInterface."""

    def __init__(
        self,
        controller_name: str,
        controller: VehiclePIDController
        | LQRController
        | LinearMPCController
        | CasadiNMPCController,
    ) -> None:
        self.controller_name = controller_name
        self.controller = controller
        self._metadata: dict[str, Any] = {}
        self._last_diagnostics: dict[str, Any] = {}

    def initialize(self, metadata: dict[str, Any] | None = None) -> None:
        self._metadata = metadata or {}
        self.reset()

    def step(self, step_input: ControllerStepInput) -> ControllerStepOutput:
        if self.controller_name == "pid":
            return self._step_pid(step_input)
        if self.controller_name == "lqr":
            return self._step_lqr(step_input)
        if self.controller_name == "linear_mpc":
            return self._step_linear_mpc(step_input)
        if self.controller_name == "nmpc":
            return self._step_nmpc(step_input)
        raise ValueError(f"Unsupported controller adapter: {self.controller_name}")

    def reset(self) -> None:
        reset = getattr(self.controller, "reset", None)
        if callable(reset):
            reset()
        self._last_diagnostics = {}

    def get_diagnostics(self) -> dict[str, Any]:
        return dict(self._last_diagnostics)

    def _step_pid(self, step_input: ControllerStepInput) -> ControllerStepOutput:
        start_time = perf_counter()
        command, diagnostics = self.controller.compute_control(
            step_input.state,
            step_input.target,
            step_input.dt,
        )
        solve_time_ms = (perf_counter() - start_time) * 1000.0
        self._last_diagnostics = _diagnostics_dict(diagnostics)
        return ControllerStepOutput(
            command=command,
            solver_status="not_applicable",
            solve_time_ms=solve_time_ms,
            feasible=True,
            diagnostics=self._last_diagnostics,
        )

    def _step_lqr(self, step_input: ControllerStepInput) -> ControllerStepOutput:
        start_time = perf_counter()
        command, diagnostics = self.controller.compute_control(step_input.state, step_input.target)
        solve_time_ms = (perf_counter() - start_time) * 1000.0
        self._last_diagnostics = _diagnostics_dict(diagnostics)
        return ControllerStepOutput(
            command=command,
            solver_status="not_applicable",
            solve_time_ms=solve_time_ms,
            feasible=True,
            diagnostics=self._last_diagnostics,
        )

    def _step_linear_mpc(self, step_input: ControllerStepInput) -> ControllerStepOutput:
        result = self.controller.compute_control(step_input.state, step_input.target)
        self._last_diagnostics = {
            "objective_value": result.objective_value,
            "solver_status": result.solver_status,
            "feasible": result.feasible,
        }
        return ControllerStepOutput(
            command=result.command,
            solver_status=result.solver_status,
            solve_time_ms=result.solve_time_ms,
            feasible=result.feasible,
            predicted_states=result.predicted_states,
            predicted_inputs=result.predicted_inputs,
            diagnostics=self._last_diagnostics,
        )

    def _step_nmpc(self, step_input: ControllerStepInput) -> ControllerStepOutput:
        result = self.controller.compute_control(
            step_input.state,
            step_input.target,
            reference_states=step_input.reference_states,
        )
        self._last_diagnostics = {
            "backend": result.backend,
            "objective_value": result.objective_value,
            "solver_status": result.solver_status,
            "feasible": result.feasible,
            "constraint_violation_flags": result.constraint_violation_flags,
        }
        return ControllerStepOutput(
            command=result.command,
            solver_status=result.solver_status,
            solve_time_ms=result.solve_time_ms,
            feasible=result.feasible,
            predicted_states=result.predicted_states,
            predicted_inputs=result.predicted_inputs,
            diagnostics=self._last_diagnostics,
        )


class CompiledControllerUnavailable(RuntimeError):
    """Raised when a compiled SIL controller artifact is requested but unavailable."""


class CompiledControllerAdapter:
    """Placeholder for future generated C/acados controller artifacts."""

    def __init__(self, artifact_path: str | Path | None = None) -> None:
        self.artifact_path = Path(artifact_path) if artifact_path is not None else None
        self._diagnostics: dict[str, Any] = {}

    def initialize(self, metadata: dict[str, Any] | None = None) -> None:
        if self.artifact_path is None or not self.artifact_path.exists():
            raise CompiledControllerUnavailable(
                "Compiled controller artifact is unavailable. Generate/package the controller "
                "before enabling compiled SIL tests."
            )
        self._diagnostics = {"metadata": metadata or {}, "artifact_path": str(self.artifact_path)}

    def step(self, step_input: ControllerStepInput) -> ControllerStepOutput:
        raise CompiledControllerUnavailable(
            "Compiled controller execution is not wired yet; use PythonControllerAdapter for "
            "current SIL-style equivalence tests."
        )

    def reset(self) -> None:
        self._diagnostics = {}

    def get_diagnostics(self) -> dict[str, Any]:
        return dict(self._diagnostics)


def _diagnostics_dict(diagnostics: Any) -> dict[str, Any]:
    if diagnostics is None:
        return {}
    if is_dataclass(diagnostics):
        return asdict(diagnostics)
    if isinstance(diagnostics, dict):
        return dict(diagnostics)
    return {"repr": repr(diagnostics)}


__all__ = [
    "CompiledControllerAdapter",
    "CompiledControllerUnavailable",
    "ControllerInterface",
    "ControllerStepInput",
    "ControllerStepOutput",
    "PythonControllerAdapter",
]
