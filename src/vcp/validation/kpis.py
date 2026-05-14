from __future__ import annotations

from math import ceil, sqrt
from typing import Any


def compute_run_kpis(
    rows: list[dict[str, Any]],
    *,
    goal_lateral_tolerance: float = 0.5,
    goal_speed_tolerance: float = 1.0,
    goal_progress_tolerance_m: float = 2.0,
    safe_stop_speed_tolerance: float = 0.75,
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
    obstacle_risk_count = sum(int(bool(row.get("obstacle_risk_flag", False))) for row in rows)
    protective_fallback_count = sum(
        int(bool(row["fallback_active"]) and row.get("fallback_reason") == "collision_risk")
        for row in rows
    )
    fault_fallback_count = fallback_count - protective_fallback_count

    final_lateral_error = abs(lateral_errors[-1])
    final_speed_error = abs(speed_errors[-1])
    final_reference_remaining_distance_m = _maybe_float(
        rows[-1].get("reference_remaining_distance_m"),
    )
    if final_reference_remaining_distance_m is not None:
        goal_reached = final_reference_remaining_distance_m <= goal_progress_tolerance_m
    else:
        goal_reached = (
            final_lateral_error <= goal_lateral_tolerance
            and final_speed_error <= goal_speed_tolerance
        )
    drivable = (
        collision_count == 0
        and road_boundary_violation_count == 0
        and constraint_violation_count == 0
    )
    stable_execution = fault_fallback_count == 0
    final_speed_mps = _maybe_float(rows[-1].get("v")) or 0.0
    blocked_by_obstacle = (
        drivable
        and not goal_reached
        and protective_fallback_count > 0
        and stable_execution
    )
    safe_stop = (
        blocked_by_obstacle and final_speed_mps <= safe_stop_speed_tolerance
    )
    success = drivable and stable_execution and goal_reached
    if success:
        result_class = "success"
    elif safe_stop:
        result_class = "safe_stop"
    elif blocked_by_obstacle:
        result_class = "blocked"
    else:
        result_class = "failure"

    return {
        "success": success,
        "success_rate": 1.0 if success else 0.0,
        "result_class": result_class,
        "drivable": drivable,
        "drivable_rate": 1.0 if drivable else 0.0,
        "collision_count": collision_count,
        "road_boundary_violation_count": road_boundary_violation_count,
        "goal_reached": goal_reached,
        "goal_reached_rate": 1.0 if goal_reached else 0.0,
        "stable_execution": stable_execution,
        "stable_execution_rate": 1.0 if stable_execution else 0.0,
        "blocked_by_obstacle": blocked_by_obstacle,
        "blocked_by_obstacle_rate": 1.0 if blocked_by_obstacle else 0.0,
        "safe_stop": safe_stop,
        "safe_stop_rate": 1.0 if safe_stop else 0.0,
        "lateral_rmse": _rmse(lateral_errors),
        "heading_rmse": _rmse(heading_errors),
        "speed_rmse": _rmse(speed_errors),
        "mean_solve_time_ms": sum(solve_times) / len(solve_times),
        "p95_solve_time_ms": _percentile(solve_times, 95.0),
        "max_solve_time_ms": max(solve_times),
        "fallback_count": fallback_count,
        "protective_fallback_count": protective_fallback_count,
        "fault_fallback_count": fault_fallback_count,
        "obstacle_risk_count": obstacle_risk_count,
        "constraint_violation_count": constraint_violation_count,
        "final_abs_lateral_error": final_lateral_error,
        "final_abs_speed_error": final_speed_error,
        "final_reference_remaining_distance_m": final_reference_remaining_distance_m,
    }


def aggregate_kpis(run_kpis: list[dict[str, bool | float | int]]) -> dict[str, float | int]:
    """Aggregate scenario-level KPI dictionaries into a compact summary."""

    if not run_kpis:
        raise ValueError("run_kpis must not be empty")

    return {
        "run_count": len(run_kpis),
        "success_rate": sum(float(kpi["success_rate"]) for kpi in run_kpis) / len(run_kpis),
        "drivable_rate": sum(float(kpi["drivable_rate"]) for kpi in run_kpis) / len(run_kpis),
        "goal_reached_rate": sum(float(kpi["goal_reached_rate"]) for kpi in run_kpis)
        / len(run_kpis),
        "stable_execution_rate": sum(
            float(kpi["stable_execution_rate"]) for kpi in run_kpis
        )
        / len(run_kpis),
        "blocked_by_obstacle_count": sum(
            int(bool(kpi["blocked_by_obstacle"])) for kpi in run_kpis
        ),
        "safe_stop_count": sum(int(bool(kpi["safe_stop"])) for kpi in run_kpis),
        "collision_count": sum(int(kpi["collision_count"]) for kpi in run_kpis),
        "road_boundary_violation_count": sum(
            int(kpi["road_boundary_violation_count"]) for kpi in run_kpis
        ),
        "fallback_count": sum(int(kpi["fallback_count"]) for kpi in run_kpis),
        "protective_fallback_count": sum(
            int(kpi["protective_fallback_count"]) for kpi in run_kpis
        ),
        "fault_fallback_count": sum(int(kpi["fault_fallback_count"]) for kpi in run_kpis),
        "obstacle_risk_count": sum(int(kpi["obstacle_risk_count"]) for kpi in run_kpis),
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


def _maybe_float(value: Any) -> float | None:
    if value in ("", None):
        return None
    return float(value)


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
