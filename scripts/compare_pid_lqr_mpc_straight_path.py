from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass
from math import sqrt
from pathlib import Path

from vcp.controllers import (
    LinearMPCConfig,
    LinearMPCController,
    LQRController,
    PathTrackingTarget,
    create_default_vehicle_pid,
)
from vcp.models import KinematicBicycleModel, LinearizedBicycleModel, VehicleInput, VehicleState


@dataclass(frozen=True)
class ControllerRun:
    name: str
    rows: list[dict[str, float | str]]


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare PID, LQR, and linear MPC.")
    parser.add_argument("--dt", type=float, default=0.1, help="Simulation step time in seconds.")
    parser.add_argument("--steps", type=int, default=120, help="Number of simulation steps.")
    parser.add_argument("--target-speed", type=float, default=4.0, help="Target speed in m/s.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts/pid_lqr_mpc_straight_path"),
        help="Directory for CSV and metrics JSON output artifacts.",
    )
    args = parser.parse_args()

    if args.dt <= 0.0:
        raise ValueError("--dt must be positive")
    if args.steps <= 0:
        raise ValueError("--steps must be positive")

    target = PathTrackingTarget(speed=args.target_speed, lateral_position=0.0, heading=0.0)
    runs = [
        _run_pid(target, dt=args.dt, steps=args.steps),
        _run_lqr(target, dt=args.dt, steps=args.steps),
        _run_mpc(target, dt=args.dt, steps=args.steps),
    ]

    args.output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output_dir / "pid_lqr_mpc_straight_path.csv"
    metrics_path = args.output_dir / "pid_lqr_mpc_metrics.json"
    _write_csv(runs, csv_path)
    metrics_path.write_text(
        json.dumps({run.name: _metrics(run.rows) for run in runs}, indent=2, sort_keys=True),
        encoding="utf-8",
    )

    print(f"Wrote comparison CSV to {csv_path}")
    print(f"Wrote comparison metrics to {metrics_path}")
    return 0


def _run_pid(target: PathTrackingTarget, *, dt: float, steps: int) -> ControllerRun:
    controller = create_default_vehicle_pid()

    def controller_step(state: VehicleState) -> tuple[VehicleInput, dict[str, float | str]]:
        command, diagnostics = controller.compute_control(state, target, dt)
        return command, {
            "lateral_error": diagnostics.lateral_error,
            "heading_error": diagnostics.heading_error,
            "solver_status": "not_applicable",
            "solve_time_ms": 0.0,
        }

    return _run_controller("pid", target, dt=dt, steps=steps, controller_step=controller_step)


def _run_lqr(target: PathTrackingTarget, *, dt: float, steps: int) -> ControllerRun:
    linear_model = LinearizedBicycleModel(
        nominal_velocity=max(target.speed, 0.1),
        wheelbase=2.8,
        dt=dt,
    )
    controller = LQRController(linear_model)

    def controller_step(state: VehicleState) -> tuple[VehicleInput, dict[str, float | str]]:
        command, diagnostics = controller.compute_control(state, target)
        return command, {
            "lateral_error": diagnostics.lateral_error,
            "heading_error": diagnostics.heading_error,
            "solver_status": "not_applicable",
            "solve_time_ms": 0.0,
        }

    return _run_controller("lqr", target, dt=dt, steps=steps, controller_step=controller_step)


def _run_mpc(target: PathTrackingTarget, *, dt: float, steps: int) -> ControllerRun:
    controller = LinearMPCController(
        LinearMPCConfig(dt=dt, nominal_velocity=max(target.speed, 0.1)),
    )

    def controller_step(state: VehicleState) -> tuple[VehicleInput, dict[str, float | str]]:
        result = controller.compute_control(state, target)
        return result.command, {
            "lateral_error": state.py - target.lateral_position,
            "heading_error": state.yaw - target.heading,
            "solver_status": result.solver_status,
            "solve_time_ms": result.solve_time_ms,
        }

    return _run_controller(
        "linear_mpc",
        target,
        dt=dt,
        steps=steps,
        controller_step=controller_step,
    )


def _run_controller(
    name: str,
    target: PathTrackingTarget,
    *,
    dt: float,
    steps: int,
    controller_step,
) -> ControllerRun:
    plant = KinematicBicycleModel()
    state = VehicleState(px=0.0, py=-1.0, yaw=0.0, v=0.0)
    rows: list[dict[str, float | str]] = []

    for step_index in range(steps):
        command, diagnostics = controller_step(state)
        rows.append(_row(name, step_index * dt, state, command, diagnostics))
        state = plant.step(state, command, dt)

    return ControllerRun(name=name, rows=rows)


def _row(
    controller_name: str,
    time_s: float,
    state: VehicleState,
    command: VehicleInput,
    diagnostics: dict[str, float | str],
) -> dict[str, float | str]:
    return {
        "controller": controller_name,
        "time_s": time_s,
        "px": state.px,
        "py": state.py,
        "yaw": state.yaw,
        "v": state.v,
        "acceleration_cmd": command.acceleration,
        "steering_cmd": command.steering_angle,
        "lateral_error": diagnostics["lateral_error"],
        "heading_error": diagnostics["heading_error"],
        "solver_status": diagnostics["solver_status"],
        "solve_time_ms": diagnostics["solve_time_ms"],
    }


def _metrics(rows: list[dict[str, float | str]]) -> dict[str, float]:
    lateral_errors = [float(row["lateral_error"]) for row in rows]
    heading_errors = [float(row["heading_error"]) for row in rows]
    steering_commands = [float(row["steering_cmd"]) for row in rows]
    solve_times = [float(row["solve_time_ms"]) for row in rows]
    return {
        "lateral_rmse": _rmse(lateral_errors),
        "heading_rmse": _rmse(heading_errors),
        "steering_effort_sum_abs": sum(abs(value) for value in steering_commands),
        "final_abs_lateral_error": abs(lateral_errors[-1]),
        "mean_solve_time_ms": sum(solve_times) / len(solve_times),
        "max_solve_time_ms": max(solve_times),
    }


def _rmse(values: list[float]) -> float:
    return sqrt(sum(value * value for value in values) / len(values))


def _write_csv(runs: list[ControllerRun], output_path: Path) -> None:
    rows = [row for run in runs for row in run.rows]
    if not rows:
        raise ValueError("rows must not be empty")

    with output_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    raise SystemExit(main())
