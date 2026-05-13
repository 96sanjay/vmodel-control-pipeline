"""MIL, SIL, HIL-lite, KPI, and safety validation utilities."""

from vcp.validation.controller_interface import (
    CompiledControllerAdapter,
    CompiledControllerUnavailable,
    ControllerInterface,
    ControllerStepInput,
    ControllerStepOutput,
    PythonControllerAdapter,
)
from vcp.validation.kpis import aggregate_kpis, compute_run_kpis
from vcp.validation.mil_runner import (
    BenchmarkRunner,
    ControllerName,
    MILRunnerConfig,
    MILRunResult,
    MILScenarioSpec,
    write_mil_outputs,
)
from vcp.validation.safety_supervisor import (
    SafetyDecision,
    SafetyEvaluationInput,
    SafetyMode,
    SafetySupervisor,
    SafetySupervisorConfig,
    SafetyTransition,
)
from vcp.validation.sil_runner import (
    SILEquivalenceReport,
    SILEquivalenceSample,
    SILEquivalenceTolerances,
    run_back_to_back_equivalence,
    write_sil_equivalence_report,
)

__all__ = [
    "BenchmarkRunner",
    "CompiledControllerAdapter",
    "CompiledControllerUnavailable",
    "ControllerName",
    "ControllerInterface",
    "ControllerStepInput",
    "ControllerStepOutput",
    "MILRunnerConfig",
    "MILRunResult",
    "MILScenarioSpec",
    "PythonControllerAdapter",
    "SILEquivalenceReport",
    "SILEquivalenceSample",
    "SILEquivalenceTolerances",
    "SafetyDecision",
    "SafetyEvaluationInput",
    "SafetyMode",
    "SafetySupervisor",
    "SafetySupervisorConfig",
    "SafetyTransition",
    "aggregate_kpis",
    "compute_run_kpis",
    "run_back_to_back_equivalence",
    "write_sil_equivalence_report",
    "write_mil_outputs",
]
