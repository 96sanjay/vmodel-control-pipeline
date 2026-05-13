from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from time import perf_counter
from typing import Any, Literal

from vcp.benchmarks.commonroad_loader import CommonRoadLoaderError, CommonRoadScenarioLoader
from vcp.benchmarks.scenario_manifest import ScenarioManifestEntry, load_scenario_suite
from vcp.controllers import (
    CasadiNMPCController,
    FallbackBrakeController,
    LinearMPCConfig,
    LinearMPCController,
    LQRController,
    NMPCConfig,
    PathTrackingTarget,
    create_default_vehicle_pid,
    normalize_angle,
)
from vcp.models import KinematicBicycleModel, LinearizedBicycleModel, VehicleInput, VehicleState
from vcp.validation.kpis import aggregate_kpis, compute_run_kpis
from vcp.validation.safety_supervisor import (
    SafetyEvaluationInput,
    SafetySupervisor,
    SafetySupervisorConfig,
)

ControllerName = Literal["pid", "lqr", "linear_mpc", "nmpc", "all"]
ScenarioSource = Literal["commonroad_initial_state", "synthetic_smoke_from_manifest"]


@dataclass(frozen=True)
class MILRunnerConfig:
    """Configuration for CI-friendly model-in-the-loop benchmark runs."""

    suite_path: Path
    output_dir: Path = Path("artifacts/mil")
    steps: int = 25
    max_scenarios: int = 1
    target_speed: float = 4.0
    road_boundary_limit_m: float = 3.5
    max_solve_time_ms: float = 95.0
    nmpc_horizon: int = 5
    linear_mpc_horizon: int = 8

    def __post_init__(self) -> None:
        if self.steps <= 0:
            raise ValueError("steps must be positive")
        if self.max_scenarios < 0:
            raise ValueError("max_scenarios must be non-negative")
        if self.target_speed <= 0.0:
            raise ValueError("target_speed must be positive")
        if self.road_boundary_limit_m <= 0.0:
            raise ValueError("road_boundary_limit_m must be positive")
        if self.max_solve_time_ms <= 0.0:
            raise ValueError("max_solve_time_ms must be positive")
        if self.nmpc_horizon <= 0:
            raise ValueError("nmpc_horizon must be positive")
        if self.linear_mpc_horizon <= 0:
            raise ValueError("linear_mpc_horizon must be positive")


@dataclass(frozen=True)
class MILScenarioSpec:
    """Small closed-loop scenario extracted from the suite manifest."""

    scenario_id: str
    difficulty: str
    scenario_type: str
    dt: float
    initial_state: VehicleState
    target: PathTrackingTarget
    source: ScenarioSource
    note: str


@dataclass(frozen=True)
class MILRunResult:
    """One controller evaluated on one MIL scenario."""

    suite_name: str
    scenario_id: str
    controller: str
    scenario_source: ScenarioSource
    rows: list[dict[str, Any]]
    kpis: dict[str, bool | float | int]
    artifacts: dict[str, str]


class BenchmarkRunner:
    """Run PID, LQR, MPC, and NMPC controllers in a lightweight MIL loop."""

    def __init__(self, config: MILRunnerConfig) -> None:
        self.config = config

    def run(self, controller: ControllerName) -> list[MILRunResult]:
        """Run the requested controller over the configured smoke scenario subset."""

        suite = load_scenario_suite(self.config.suite_path)
        controllers = _controller_names(controller)
        entries = suite.scenarios
        if self.config.max_scenarios:
            entries = entries[: self.config.max_scenarios]

        results: list[MILRunResult] = []
        loader = CommonRoadScenarioLoader(suite.scenario_root)
        for entry in entries:
            spec = self._build_scenario_spec(entry, suite.sample_time, loader)
            for controller_name in controllers:
                rows = self._run_controller(spec, controller_name)
                results.append(
                    MILRunResult(
                        suite_name=suite.suite_name,
                        scenario_id=spec.scenario_id,
                        controller=controller_name,
                        scenario_source=spec.source,
                        rows=rows,
                        kpis=compute_run_kpis(rows),
                        artifacts={},
                    )
                )
        return results

    def _build_scenario_spec(
        self,
        entry: ScenarioManifestEntry,
        suite_dt: float,
        loader: CommonRoadScenarioLoader,
    ) -> MILScenarioSpec:
        try:
            scenario_data = loader.load_scenario(entry.scenario_id)
        except CommonRoadLoaderError as exc:
            initial_state, target = _synthetic_scenario(entry, self.config.target_speed)
            return MILScenarioSpec(
                scenario_id=entry.scenario_id,
                difficulty=entry.difficulty,
                scenario_type=entry.scenario_type,
                dt=suite_dt,
                initial_state=initial_state,
                target=target,
                source="synthetic_smoke_from_manifest",
                note=str(exc).splitlines()[0],
            )

        initial_state = _state_from_commonroad_initial_state(scenario_data.initial_state)
        target = PathTrackingTarget(
            speed=max(initial_state.v, self.config.target_speed),
            lateral_position=initial_state.py,
            heading=initial_state.yaw,
        )
        return MILScenarioSpec(
            scenario_id=entry.scenario_id,
            difficulty=entry.difficulty,
            scenario_type=entry.scenario_type,
            dt=scenario_data.dt,
            initial_state=initial_state,
            target=target,
            source="commonroad_initial_state",
            note="CommonRoad XML loaded; current MIL runner uses initial state only.",
        )

    def _run_controller(self, spec: MILScenarioSpec, controller_name: str) -> list[dict[str, Any]]:
        plant = KinematicBicycleModel()
        state = spec.initial_state
        controller = self._create_controller(controller_name, spec)
        fallback = FallbackBrakeController()
        supervisor = SafetySupervisor(
            SafetySupervisorConfig(
                sample_time_s=spec.dt,
                max_solve_time_ms=self.config.max_solve_time_ms,
            )
        )
        rows: list[dict[str, Any]] = []

        for step_index in range(self.config.steps):
            time_s = step_index * spec.dt
            step = self._controller_step(controller, controller_name, state, spec)
            lateral_error = state.py - spec.target.lateral_position
            heading_error = normalize_angle(state.yaw - spec.target.heading)
            speed_error = state.v - spec.target.speed
            road_boundary_violation = abs(lateral_error) > self.config.road_boundary_limit_m

            decision = supervisor.evaluate(
                SafetyEvaluationInput(
                    timestamp_s=time_s,
                    command=step["command"],
                    solver_status=step["solver_status_for_supervisor"],
                    solver_feasible=bool(step["solver_feasible"]),
                    solve_time_ms=float(step["solve_time_ms"]),
                    estimator_residual=0.0,
                    collision_risk=False,
                    missing_sensor_message=False,
                    communication_timeout=False,
                    constraint_violation_flags=tuple(step["constraint_violation_flags"]),
                )
            )
            applied_command = (
                fallback.compute_control(state) if decision.fallback_required else step["command"]
            )
            rows.append(
                _row(
                    spec=spec,
                    controller_name=controller_name,
                    time_s=time_s,
                    state=state,
                    command=step["command"],
                    applied_command=applied_command,
                    lateral_error=lateral_error,
                    heading_error=heading_error,
                    speed_error=speed_error,
                    road_boundary_violation=road_boundary_violation,
                    step=step,
                    decision=decision,
                )
            )
            state = plant.step(state, applied_command, spec.dt)

        return rows

    def _create_controller(self, controller_name: str, spec: MILScenarioSpec) -> object:
        nominal_velocity = max(spec.target.speed, 0.1)
        if controller_name == "pid":
            return create_default_vehicle_pid()
        if controller_name == "lqr":
            return LQRController(
                LinearizedBicycleModel(
                    nominal_velocity=nominal_velocity,
                    wheelbase=2.8,
                    dt=spec.dt,
                )
            )
        if controller_name == "linear_mpc":
            return LinearMPCController(
                LinearMPCConfig(
                    horizon=self.config.linear_mpc_horizon,
                    dt=spec.dt,
                    nominal_velocity=nominal_velocity,
                )
            )
        if controller_name == "nmpc":
            return CasadiNMPCController(
                NMPCConfig(
                    horizon=self.config.nmpc_horizon,
                    dt=spec.dt,
                    max_solver_iterations=60,
                )
            )
        raise ValueError(f"Unsupported controller: {controller_name}")

    def _controller_step(
        self,
        controller: object,
        controller_name: str,
        state: VehicleState,
        spec: MILScenarioSpec,
    ) -> dict[str, Any]:
        start_time = perf_counter()
        if controller_name == "pid":
            command, diagnostics = controller.compute_control(state, spec.target, spec.dt)
            solve_time_ms = (perf_counter() - start_time) * 1000.0
            return _step_result(
                command=command,
                solve_time_ms=solve_time_ms,
                solver_status="not_applicable",
                solver_status_for_supervisor=None,
                solver_feasible=True,
                fallback_count_hint=int(diagnostics.command_saturated),
            )
        if controller_name == "lqr":
            command, diagnostics = controller.compute_control(state, spec.target)
            solve_time_ms = (perf_counter() - start_time) * 1000.0
            return _step_result(
                command=command,
                solve_time_ms=solve_time_ms,
                solver_status="not_applicable",
                solver_status_for_supervisor=None,
                solver_feasible=True,
                fallback_count_hint=int(
                    diagnostics.steering_saturated or diagnostics.acceleration_saturated
                ),
            )
        if controller_name == "linear_mpc":
            result = controller.compute_control(state, spec.target)
            return _step_result(
                command=result.command,
                solve_time_ms=result.solve_time_ms,
                solver_status=result.solver_status,
                solver_status_for_supervisor=result.solver_status,
                solver_feasible=result.feasible,
            )
        if controller_name == "nmpc":
            result = controller.compute_control(state, spec.target)
            return _step_result(
                command=result.command,
                solve_time_ms=result.solve_time_ms,
                solver_status=result.solver_status,
                solver_status_for_supervisor=result.solver_status,
                solver_feasible=result.feasible,
                constraint_violation_flags=result.constraint_violation_flags,
            )
        raise ValueError(f"Unsupported controller: {controller_name}")


def write_mil_outputs(
    results: list[MILRunResult],
    output_dir: Path,
) -> dict[str, Path]:
    """Write JSON, CSV, SVG, and Markdown evidence for MIL runs."""

    if not results:
        raise ValueError("results must not be empty")

    output_dir.mkdir(parents=True, exist_ok=True)
    run_results: list[MILRunResult] = []
    for result in results:
        stem = _artifact_stem(result.scenario_id, result.controller)
        timeseries_path = output_dir / f"{stem}_timeseries.csv"
        trajectory_path = output_dir / f"{stem}_trajectory.svg"
        tracking_path = output_dir / f"{stem}_tracking_error.svg"
        solver_path = output_dir / f"{stem}_solver_time.svg"
        _write_rows_csv(result.rows, timeseries_path)
        _write_xy_svg(result.rows, "px", "py", trajectory_path, f"{stem} trajectory")
        _write_line_svg(
            result.rows,
            "time_s",
            "lateral_error",
            tracking_path,
            f"{stem} lateral error",
            "m",
        )
        _write_line_svg(
            result.rows,
            "time_s",
            "solve_time_ms",
            solver_path,
            f"{stem} solver time",
            "ms",
        )
        run_results.append(
            MILRunResult(
                suite_name=result.suite_name,
                scenario_id=result.scenario_id,
                controller=result.controller,
                scenario_source=result.scenario_source,
                rows=result.rows,
                kpis=result.kpis,
                artifacts={
                    "timeseries_csv": str(timeseries_path),
                    "trajectory_svg": str(trajectory_path),
                    "tracking_error_svg": str(tracking_path),
                    "solver_time_svg": str(solver_path),
                },
            )
        )

    summary_csv_path = output_dir / "mil_results.csv"
    results_json_path = output_dir / "mil_results.json"
    report_path = output_dir / "mil_report.md"
    _write_summary_csv(run_results, summary_csv_path)
    _write_results_json(run_results, results_json_path)
    _write_report(run_results, report_path)
    return {
        "summary_csv": summary_csv_path,
        "results_json": results_json_path,
        "report_md": report_path,
    }


def _synthetic_scenario(
    entry: ScenarioManifestEntry,
    target_speed: float,
) -> tuple[VehicleState, PathTrackingTarget]:
    yaw_offset = 0.0
    lateral_offset = -1.0
    target_lateral = 0.0
    if entry.scenario_type == "urban":
        yaw_offset = 0.05
        lateral_offset = -0.8
    elif entry.scenario_type == "intersection":
        yaw_offset = -0.05
        lateral_offset = -0.6
    elif entry.scenario_type == "lane_change":
        lateral_offset = -1.2
        target_lateral = 0.8
    elif entry.scenario_type == "highway":
        lateral_offset = -0.5

    return (
        VehicleState(px=0.0, py=lateral_offset, yaw=yaw_offset, v=0.0),
        PathTrackingTarget(speed=target_speed, lateral_position=target_lateral, heading=0.0),
    )


def _state_from_commonroad_initial_state(raw_state: object) -> VehicleState:
    position = getattr(raw_state, "position", (0.0, -1.0))
    px = float(position[0])
    py = float(position[1])
    yaw = float(getattr(raw_state, "orientation", 0.0))
    velocity = float(getattr(raw_state, "velocity", 0.0))
    return VehicleState(px=px, py=py, yaw=yaw, v=velocity)


def _step_result(
    *,
    command: VehicleInput,
    solve_time_ms: float,
    solver_status: str,
    solver_status_for_supervisor: str | None,
    solver_feasible: bool,
    fallback_count_hint: int = 0,
    constraint_violation_flags: tuple[str, ...] = (),
) -> dict[str, Any]:
    return {
        "command": command,
        "solve_time_ms": solve_time_ms,
        "solver_status": solver_status,
        "solver_status_for_supervisor": solver_status_for_supervisor,
        "solver_feasible": solver_feasible,
        "fallback_count_hint": fallback_count_hint,
        "constraint_violation_flags": constraint_violation_flags,
    }


def _row(
    *,
    spec: MILScenarioSpec,
    controller_name: str,
    time_s: float,
    state: VehicleState,
    command: VehicleInput,
    applied_command: VehicleInput,
    lateral_error: float,
    heading_error: float,
    speed_error: float,
    road_boundary_violation: bool,
    step: dict[str, Any],
    decision: Any,
) -> dict[str, Any]:
    return {
        "suite_scenario_id": spec.scenario_id,
        "scenario_source": spec.source,
        "controller": controller_name,
        "time_s": time_s,
        "px": state.px,
        "py": state.py,
        "yaw": state.yaw,
        "v": state.v,
        "acceleration_cmd": command.acceleration,
        "steering_cmd": command.steering_angle,
        "applied_acceleration_cmd": applied_command.acceleration,
        "applied_steering_cmd": applied_command.steering_angle,
        "lateral_error": lateral_error,
        "heading_error": heading_error,
        "speed_error": speed_error,
        "solver_status": step["solver_status"],
        "solver_feasible": bool(step["solver_feasible"]),
        "solve_time_ms": float(step["solve_time_ms"]),
        "safety_mode": decision.mode.value,
        "fallback_active": decision.fallback_required,
        "fallback_reason": decision.reason_code,
        "constraint_violation_count": len(step["constraint_violation_flags"])
        + int(not decision.command_is_valid),
        "collision_flag": False,
        "road_boundary_violation": road_boundary_violation,
    }


def _controller_names(controller: ControllerName) -> tuple[str, ...]:
    if controller == "all":
        return ("pid", "lqr", "linear_mpc", "nmpc")
    return (controller,)


def _write_results_json(results: list[MILRunResult], output_path: Path) -> None:
    payload = {
        "generated_at_utc": datetime.now(UTC).isoformat(),
        "suite_name": results[0].suite_name,
        "summary": aggregate_kpis([result.kpis for result in results]),
        "runs": [
            {
                "scenario_id": result.scenario_id,
                "controller": result.controller,
                "scenario_source": result.scenario_source,
                "kpis": result.kpis,
                "artifacts": result.artifacts,
            }
            for result in results
        ],
    }
    output_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def _write_summary_csv(results: list[MILRunResult], output_path: Path) -> None:
    rows = [
        {
            "suite_name": result.suite_name,
            "scenario_id": result.scenario_id,
            "controller": result.controller,
            "scenario_source": result.scenario_source,
            **result.kpis,
        }
        for result in results
    ]
    _write_rows_csv(rows, output_path)


def _write_report(results: list[MILRunResult], output_path: Path) -> None:
    lines = [
        "# MIL Smoke Report",
        "",
        "This report is generated by the Phase 10 MIL runner.",
        "It is a smoke benchmark unless real CommonRoad scenario files are provided.",
        "",
        "| Scenario | Controller | Source | Success | Lateral RMSE | "
        "p95 solve time ms | Fallbacks |",
        "|---|---|---|---:|---:|---:|---:|",
    ]
    for result in results:
        lines.append(
            "| "
            f"{result.scenario_id} | "
            f"{result.controller} | "
            f"{result.scenario_source} | "
            f"{int(bool(result.kpis['success']))} | "
            f"{float(result.kpis['lateral_rmse']):.4f} | "
            f"{float(result.kpis['p95_solve_time_ms']):.2f} | "
            f"{int(result.kpis['fallback_count'])} |"
        )
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_rows_csv(rows: list[dict[str, Any]], output_path: Path) -> None:
    if not rows:
        raise ValueError("rows must not be empty")

    with output_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _write_xy_svg(
    rows: list[dict[str, Any]],
    x_key: str,
    y_key: str,
    output_path: Path,
    title: str,
) -> None:
    _write_polyline_svg(
        rows,
        x_key=x_key,
        y_key=y_key,
        output_path=output_path,
        title=title,
        x_label="px [m]",
        y_label="py [m]",
    )


def _write_line_svg(
    rows: list[dict[str, Any]],
    x_key: str,
    y_key: str,
    output_path: Path,
    title: str,
    y_unit: str,
) -> None:
    _write_polyline_svg(
        rows,
        x_key=x_key,
        y_key=y_key,
        output_path=output_path,
        title=title,
        x_label="time [s]",
        y_label=f"{y_key} [{y_unit}]",
    )


def _write_polyline_svg(
    rows: list[dict[str, Any]],
    *,
    x_key: str,
    y_key: str,
    output_path: Path,
    title: str,
    x_label: str,
    y_label: str,
) -> None:
    width = 900
    height = 420
    margin_left = 70
    margin_right = 30
    margin_top = 45
    margin_bottom = 55
    points = [(float(row[x_key]), float(row[y_key])) for row in rows]
    x_values = [point[0] for point in points]
    y_values = [point[1] for point in points]
    x_min, x_max = _expand_range(min(x_values), max(x_values))
    y_min, y_max = _expand_range(min(y_values), max(y_values))

    def sx(value: float) -> float:
        return margin_left + (value - x_min) / (x_max - x_min) * (
            width - margin_left - margin_right
        )

    def sy(value: float) -> float:
        return height - margin_bottom - (value - y_min) / (y_max - y_min) * (
            height - margin_top - margin_bottom
        )

    polyline = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in points)
    svg = "\n".join(
        [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
            f'viewBox="0 0 {width} {height}">',
            '  <rect width="100%" height="100%" fill="#f8fafc" />',
            f'  <text x="{width / 2:.2f}" y="26" text-anchor="middle" '
            'font-family="monospace" font-size="18" fill="#0f172a">'
            f"{title}</text>",
            _svg_line(
                margin_left,
                height - margin_bottom,
                width - margin_right,
                height - margin_bottom,
            ),
            _svg_line(margin_left, margin_top, margin_left, height - margin_bottom),
            f'  <polyline points="{polyline}" fill="none" stroke="#0f766e" '
            'stroke-width="3" stroke-linejoin="round" stroke-linecap="round" />',
            f'  <text x="{width / 2:.2f}" y="{height - 16}" text-anchor="middle" '
            f'font-family="monospace" font-size="13" fill="#334155">{x_label}</text>',
            f'  <text x="18" y="{height / 2:.2f}" text-anchor="middle" '
            'font-family="monospace" font-size="13" fill="#334155" '
            f'transform="rotate(-90 18 {height / 2:.0f})">{y_label}</text>',
            "</svg>",
        ]
    )
    output_path.write_text(svg + "\n", encoding="utf-8")


def _svg_line(x1: float, y1: float, x2: float, y2: float) -> str:
    return (
        f'  <line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
        'stroke="#334155" stroke-width="1.5" />'
    )


def _expand_range(lower: float, upper: float) -> tuple[float, float]:
    if lower == upper:
        padding = abs(lower) * 0.1 if lower else 1.0
        return lower - padding, upper + padding
    padding = (upper - lower) * 0.1
    return lower - padding, upper + padding


def _artifact_stem(scenario_id: str, controller: str) -> str:
    safe_scenario = "".join(char if char.isalnum() or char in "-_" else "_" for char in scenario_id)
    return f"{safe_scenario}_{controller}"


__all__ = [
    "BenchmarkRunner",
    "ControllerName",
    "MILRunResult",
    "MILRunnerConfig",
    "MILScenarioSpec",
    "write_mil_outputs",
]
