from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import numpy as np

from vcp.validation.controller_interface import (
    ControllerInterface,
    ControllerStepInput,
    ControllerStepOutput,
)


@dataclass(frozen=True)
class SILEquivalenceTolerances:
    """Numerical tolerances for MIL/SIL-style back-to-back comparison."""

    acceleration_abs: float = 1e-6
    steering_abs: float = 1e-6
    predicted_state_abs: float = 1e-5

    def __post_init__(self) -> None:
        if self.acceleration_abs < 0.0:
            raise ValueError("acceleration_abs must be non-negative")
        if self.steering_abs < 0.0:
            raise ValueError("steering_abs must be non-negative")
        if self.predicted_state_abs < 0.0:
            raise ValueError("predicted_state_abs must be non-negative")


@dataclass(frozen=True)
class SILEquivalenceSample:
    """Single back-to-back comparison sample."""

    step_index: int
    timestamp_s: float
    acceleration_error: float
    steering_error: float
    predicted_state_error: float | None
    passed: bool
    mil_solver_status: str
    sil_solver_status: str


@dataclass(frozen=True)
class SILEquivalenceReport:
    """Back-to-back equivalence result for a controller interface pair."""

    controller_name: str
    passed: bool
    sample_count: int
    tolerances: SILEquivalenceTolerances
    samples: tuple[SILEquivalenceSample, ...]
    max_acceleration_error: float
    max_steering_error: float
    max_predicted_state_error: float | None


def run_back_to_back_equivalence(
    *,
    controller_name: str,
    mil_adapter: ControllerInterface,
    sil_adapter: ControllerInterface,
    input_sequence: list[ControllerStepInput] | tuple[ControllerStepInput, ...],
    tolerances: SILEquivalenceTolerances | None = None,
) -> SILEquivalenceReport:
    """Run the same input sequence through two adapters and compare outputs."""

    if not input_sequence:
        raise ValueError("input_sequence must not be empty")

    tolerances = tolerances or SILEquivalenceTolerances()
    mil_adapter.initialize({"stage": "MIL", "controller": controller_name})
    sil_adapter.initialize({"stage": "SIL", "controller": controller_name})

    samples: list[SILEquivalenceSample] = []
    for step_index, step_input in enumerate(input_sequence):
        mil_output = mil_adapter.step(step_input)
        sil_output = sil_adapter.step(step_input)
        samples.append(
            _compare_outputs(
                step_index,
                step_input,
                mil_output,
                sil_output,
                tolerances,
            )
        )

    return _build_report(controller_name, tolerances, tuple(samples))


def write_sil_equivalence_report(
    report: SILEquivalenceReport,
    output_dir: Path,
) -> dict[str, Path]:
    """Write SIL equivalence evidence as JSON and Markdown."""

    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / f"{report.controller_name}_sil_equivalence.json"
    markdown_path = output_dir / f"{report.controller_name}_sil_equivalence.md"
    json_path.write_text(json.dumps(_report_to_dict(report), indent=2), encoding="utf-8")
    markdown_path.write_text(_report_to_markdown(report), encoding="utf-8")
    return {"json": json_path, "markdown": markdown_path}


def _compare_outputs(
    step_index: int,
    step_input: ControllerStepInput,
    mil_output: ControllerStepOutput,
    sil_output: ControllerStepOutput,
    tolerances: SILEquivalenceTolerances,
) -> SILEquivalenceSample:
    acceleration_error = abs(
        mil_output.command.acceleration - sil_output.command.acceleration
    )
    steering_error = abs(mil_output.command.steering_angle - sil_output.command.steering_angle)
    predicted_state_error = _predicted_state_error(mil_output, sil_output)
    predicted_pass = (
        True
        if predicted_state_error is None
        else predicted_state_error <= tolerances.predicted_state_abs
    )
    passed = (
        acceleration_error <= tolerances.acceleration_abs
        and steering_error <= tolerances.steering_abs
        and predicted_pass
        and mil_output.feasible == sil_output.feasible
    )
    return SILEquivalenceSample(
        step_index=step_index,
        timestamp_s=step_input.timestamp_s,
        acceleration_error=acceleration_error,
        steering_error=steering_error,
        predicted_state_error=predicted_state_error,
        passed=passed,
        mil_solver_status=mil_output.solver_status,
        sil_solver_status=sil_output.solver_status,
    )


def _predicted_state_error(
    mil_output: ControllerStepOutput,
    sil_output: ControllerStepOutput,
) -> float | None:
    if mil_output.predicted_states is None and sil_output.predicted_states is None:
        return None
    if mil_output.predicted_states is None or sil_output.predicted_states is None:
        return float("inf")
    if mil_output.predicted_states.shape != sil_output.predicted_states.shape:
        return float("inf")
    difference = np.asarray(mil_output.predicted_states - sil_output.predicted_states)
    return float(np.max(np.abs(difference))) if difference.size else 0.0


def _build_report(
    controller_name: str,
    tolerances: SILEquivalenceTolerances,
    samples: tuple[SILEquivalenceSample, ...],
) -> SILEquivalenceReport:
    predicted_errors = [
        sample.predicted_state_error
        for sample in samples
        if sample.predicted_state_error is not None
    ]
    return SILEquivalenceReport(
        controller_name=controller_name,
        passed=all(sample.passed for sample in samples),
        sample_count=len(samples),
        tolerances=tolerances,
        samples=samples,
        max_acceleration_error=max(sample.acceleration_error for sample in samples),
        max_steering_error=max(sample.steering_error for sample in samples),
        max_predicted_state_error=max(predicted_errors) if predicted_errors else None,
    )


def _report_to_dict(report: SILEquivalenceReport) -> dict[str, Any]:
    return {
        "generated_at_utc": datetime.now(UTC).isoformat(),
        "controller_name": report.controller_name,
        "passed": report.passed,
        "sample_count": report.sample_count,
        "tolerances": {
            "acceleration_abs": report.tolerances.acceleration_abs,
            "steering_abs": report.tolerances.steering_abs,
            "predicted_state_abs": report.tolerances.predicted_state_abs,
        },
        "max_acceleration_error": report.max_acceleration_error,
        "max_steering_error": report.max_steering_error,
        "max_predicted_state_error": report.max_predicted_state_error,
        "samples": [
            {
                "step_index": sample.step_index,
                "timestamp_s": sample.timestamp_s,
                "acceleration_error": sample.acceleration_error,
                "steering_error": sample.steering_error,
                "predicted_state_error": sample.predicted_state_error,
                "passed": sample.passed,
                "mil_solver_status": sample.mil_solver_status,
                "sil_solver_status": sample.sil_solver_status,
            }
            for sample in report.samples
        ],
    }


def _report_to_markdown(report: SILEquivalenceReport) -> str:
    lines = [
        f"# SIL Equivalence Report: {report.controller_name}",
        "",
        f"Result: {'PASS' if report.passed else 'FAIL'}",
        f"Samples: {report.sample_count}",
        "",
        "| Metric | Value |",
        "|---|---:|",
        f"| Max acceleration error | {report.max_acceleration_error:.6g} |",
        f"| Max steering error | {report.max_steering_error:.6g} |",
        f"| Max predicted-state error | {_format_optional(report.max_predicted_state_error)} |",
        "",
        "| Step | Time s | Accel error | Steering error | Predicted-state error | Pass |",
        "|---:|---:|---:|---:|---:|---:|",
    ]
    for sample in report.samples:
        lines.append(
            f"| {sample.step_index} | "
            f"{sample.timestamp_s:.3f} | "
            f"{sample.acceleration_error:.6g} | "
            f"{sample.steering_error:.6g} | "
            f"{_format_optional(sample.predicted_state_error)} | "
            f"{int(sample.passed)} |"
        )
    return "\n".join(lines) + "\n"


def _format_optional(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.6g}"


__all__ = [
    "SILEquivalenceReport",
    "SILEquivalenceSample",
    "SILEquivalenceTolerances",
    "run_back_to_back_equivalence",
    "write_sil_equivalence_report",
]
