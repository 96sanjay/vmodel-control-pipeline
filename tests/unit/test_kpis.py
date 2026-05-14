from __future__ import annotations

from vcp.validation.kpis import aggregate_kpis, compute_run_kpis


def test_compute_run_kpis_marks_progress_goal_success() -> None:
    kpis = compute_run_kpis(
        [
            _row(reference_remaining_distance_m=5.0, v=2.0),
            _row(reference_remaining_distance_m=1.0, v=0.5),
        ]
    )

    assert kpis["goal_reached"] is True
    assert kpis["success"] is True
    assert kpis["result_class"] == "success"


def test_compute_run_kpis_distinguishes_safe_stop_from_failure() -> None:
    kpis = compute_run_kpis(
        [
            _row(
                reference_remaining_distance_m=8.0,
                v=0.2,
                obstacle_risk_flag=True,
                fallback_active=True,
                fallback_reason="collision_risk",
            )
        ]
    )

    assert kpis["goal_reached"] is False
    assert kpis["safe_stop"] is True
    assert kpis["success"] is False
    assert kpis["result_class"] == "safe_stop"


def test_compute_run_kpis_marks_blocked_when_drivable_but_not_stopped() -> None:
    kpis = compute_run_kpis(
        [
            _row(
                reference_remaining_distance_m=8.0,
                v=1.5,
                obstacle_risk_flag=True,
                fallback_active=True,
                fallback_reason="collision_risk",
            )
        ]
    )

    assert kpis["blocked_by_obstacle"] is True
    assert kpis["safe_stop"] is False
    assert kpis["result_class"] == "blocked"


def test_aggregate_kpis_reports_safe_stop_count() -> None:
    summary = aggregate_kpis(
        [
            compute_run_kpis([_row(reference_remaining_distance_m=1.0, v=0.2)]),
            compute_run_kpis(
                [
                    _row(
                        reference_remaining_distance_m=8.0,
                        v=0.2,
                        obstacle_risk_flag=True,
                        fallback_active=True,
                        fallback_reason="collision_risk",
                    )
                ]
            ),
        ]
    )

    assert summary["run_count"] == 2
    assert summary["safe_stop_count"] == 1
    assert summary["blocked_by_obstacle_count"] == 1
    assert summary["success_rate"] == 0.5


def _row(
    *,
    lateral_error: float = 0.0,
    heading_error: float = 0.0,
    speed_error: float = 0.0,
    solve_time_ms: float = 1.0,
    fallback_active: bool = False,
    fallback_reason: str = "no_fault",
    constraint_violation_count: int = 0,
    collision_flag: bool = False,
    road_boundary_violation: bool = False,
    obstacle_risk_flag: bool = False,
    reference_remaining_distance_m: float | str = "",
    v: float = 0.0,
) -> dict[str, object]:
    return {
        "lateral_error": lateral_error,
        "heading_error": heading_error,
        "speed_error": speed_error,
        "solve_time_ms": solve_time_ms,
        "fallback_active": fallback_active,
        "fallback_reason": fallback_reason,
        "constraint_violation_count": constraint_violation_count,
        "collision_flag": collision_flag,
        "road_boundary_violation": road_boundary_violation,
        "obstacle_risk_flag": obstacle_risk_flag,
        "reference_remaining_distance_m": reference_remaining_distance_m,
        "v": v,
    }
