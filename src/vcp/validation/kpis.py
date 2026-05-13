from __future__ import annotations

from math import ceil, sqrt
from typing import Any


def compute_run_kpis(
    rows: list[dict[str, Any]],
    *,
    goal_lateral_tolerance: float = 0.5,
    goal_speed_tolerance: float = 1.0,
) -> dict[str, bool | float | int]:
    """Compute MIL KPIs from one controller/scenario time series."""

    if not rows:
        raise ValueError("rows must not be empty")

    lateral_errors = [float(row["lateral_error"]) for row in rows]
    heading_errors = [float(row["heading_error"]) for row in rows]
    speed_errors = [float(row["speed_error"]) for row in rows]
    solve_times = [float(row["solve_time_ms"]) for row in rows]
    fallback_count = sum(int(bool(row["fallback_active"])) for row in rows)
    constraint_violation_count = sum(int(row["constraint_violation_count"]) for row in rows)
    collision_count = sum(int(bool(row["collision_flag"])) for row in rows)
    road_boundary_violation_count = sum(
        int(bool(row["road_boundary_violation"])) for row in rows
    )

    final_lateral_error = abs(lateral_errors[-1])
    final_speed_error = abs(speed_errors[-1])
    goal_reached = (
        final_lateral_error <= goal_lateral_tolerance
        and final_speed_error <= goal_speed_tolerance
    )
    success = collision_count == 0 and road_boundary_violation_count == 0 and goal_reached

    return {
        "success": success,
        "success_rate": 1.0 if success else 0.0,
        "collision_count": collision_count,
        "road_boundary_violation_count": road_boundary_violation_count,
        "goal_reached": goal_reached,
        "lateral_rmse": _rmse(lateral_errors),
        "heading_rmse": _rmse(heading_errors),
        "speed_rmse": _rmse(speed_errors),
        "mean_solve_time_ms": sum(solve_times) / len(solve_times),
        "p95_solve_time_ms": _percentile(solve_times, 95.0),
        "max_solve_time_ms": max(solve_times),
        "fallback_count": fallback_count,
        "constraint_violation_count": constraint_violation_count,
        "final_abs_lateral_error": final_lateral_error,
        "final_abs_speed_error": final_speed_error,
    }


def aggregate_kpis(run_kpis: list[dict[str, bool | float | int]]) -> dict[str, float | int]:
    """Aggregate scenario-level KPI dictionaries into a compact summary."""

    if not run_kpis:
        raise ValueError("run_kpis must not be empty")

    return {
        "run_count": len(run_kpis),
        "success_rate": sum(float(kpi["success_rate"]) for kpi in run_kpis) / len(run_kpis),
        "collision_count": sum(int(kpi["collision_count"]) for kpi in run_kpis),
        "road_boundary_violation_count": sum(
            int(kpi["road_boundary_violation_count"]) for kpi in run_kpis
        ),
        "fallback_count": sum(int(kpi["fallback_count"]) for kpi in run_kpis),
        "constraint_violation_count": sum(
            int(kpi["constraint_violation_count"]) for kpi in run_kpis
        ),
        "mean_lateral_rmse": sum(float(kpi["lateral_rmse"]) for kpi in run_kpis)
        / len(run_kpis),
        "mean_speed_rmse": sum(float(kpi["speed_rmse"]) for kpi in run_kpis) / len(run_kpis),
        "max_p95_solve_time_ms": max(float(kpi["p95_solve_time_ms"]) for kpi in run_kpis),
    }


def _rmse(values: list[float]) -> float:
    return sqrt(sum(value * value for value in values) / len(values))


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        raise ValueError("values must not be empty")
    if not 0.0 <= percentile <= 100.0:
        raise ValueError("percentile must be in [0, 100]")

    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]

    rank = ceil(percentile / 100.0 * len(ordered)) - 1
    return ordered[min(max(rank, 0), len(ordered) - 1)]


__all__ = ["aggregate_kpis", "compute_run_kpis"]
