from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace

from vcp.benchmarks.commonroad_drivability import (
    CommonRoadDrivabilityConfig,
    annotate_rows_with_commonroad_drivability,
    check_commonroad_drivability,
)
from vcp.models import VehicleConstraints


@dataclass
class FakeLaneletNetwork:
    """Small fake for CommonRoad lanelet membership checks."""

    y_limit: float = 1.0

    def find_lanelet_by_position(self, positions: list[list[float]]) -> list[list[int]]:
        memberships: list[list[int]] = []
        for _, py in positions:
            memberships.append([1] if abs(py) <= self.y_limit else [])
        return memberships


def test_commonroad_drivability_uses_lanelet_membership_without_optional_dc() -> None:
    scenario_data = SimpleNamespace(
        lanelet_network=FakeLaneletNetwork(y_limit=0.5),
        scenario=SimpleNamespace(),
    )

    result = check_commonroad_drivability(
        scenario_data,
        _rows(),
        config=CommonRoadDrivabilityConfig(
            road_boundary_collision_check=False,
            obstacle_collision_check=False,
        ),
    )

    assert result.checked_with_lanelet_network
    assert result.road_boundary_violation_flags == (False, True)
    assert result.road_boundary_violation_count == 1
    assert result.collision_count == 0


def test_commonroad_drivability_detects_kinematic_violations() -> None:
    scenario_data = SimpleNamespace(lanelet_network=FakeLaneletNetwork())
    rows = _rows()
    rows[1]["v"] = 99.0

    result = check_commonroad_drivability(
        scenario_data,
        rows,
        config=CommonRoadDrivabilityConfig(
            constraints=VehicleConstraints(velocity_max=15.0),
            road_boundary_collision_check=False,
            obstacle_collision_check=False,
        ),
    )

    assert result.kinematic_violation_flags == (False, True)
    assert result.kinematic_violation_count == 1


def test_commonroad_drivability_annotation_updates_mil_rows() -> None:
    scenario_data = SimpleNamespace(lanelet_network=FakeLaneletNetwork(y_limit=0.5))
    rows = _rows()

    result = annotate_rows_with_commonroad_drivability(
        scenario_data,
        rows,
        config=CommonRoadDrivabilityConfig(
            road_boundary_collision_check=False,
            obstacle_collision_check=False,
        ),
    )

    assert result.road_boundary_violation_count == 1
    assert rows[0]["commonroad_lanelet_checked"] is True
    assert rows[1]["road_boundary_violation"] is True
    assert "commonroad_check_notes" in rows[1]


def _rows() -> list[dict[str, object]]:
    return [
        {
            "time_s": 0.0,
            "px": 0.0,
            "py": 0.0,
            "yaw": 0.0,
            "v": 2.0,
            "applied_acceleration_cmd": 0.1,
            "applied_steering_cmd": 0.0,
            "collision_flag": False,
            "road_boundary_violation": False,
        },
        {
            "time_s": 0.1,
            "px": 1.0,
            "py": 2.0,
            "yaw": 0.0,
            "v": 2.1,
            "applied_acceleration_cmd": 0.1,
            "applied_steering_cmd": 0.0,
            "collision_flag": False,
            "road_boundary_violation": False,
        },
    ]
