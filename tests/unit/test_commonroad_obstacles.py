from __future__ import annotations

from types import SimpleNamespace

from vcp.benchmarks.commonroad_obstacles import assess_commonroad_obstacles
from vcp.benchmarks.commonroad_reference import CommonRoadReferencePath
from vcp.models import VehicleState


class FakeObstacle:
    def __init__(
        self,
        obstacle_id: int,
        position: tuple[float, float],
        *,
        width: float = 2.0,
        length: float = 4.0,
        speed: float = 0.0,
    ) -> None:
        self.obstacle_id = obstacle_id
        self.obstacle_shape = SimpleNamespace(width=width, length=length)
        self._state = SimpleNamespace(
            position=position,
            orientation=0.0,
            velocity=speed,
            time_step=0,
        )

    def state_at_time(self, time_step: int) -> SimpleNamespace | None:
        return self._state if time_step >= 0 else None


def test_assess_commonroad_obstacles_flags_blocking_obstacle_ahead() -> None:
    assessment = assess_commonroad_obstacles(
        _scenario(FakeObstacle(1, (10.0, 0.5))),
        _reference_path(),
        VehicleState(px=0.0, py=0.0, yaw=0.0, v=8.0),
        time_step=0,
    )

    assert assessment.collision_risk is True
    assert assessment.blocking_obstacle_ids == (1,)
    assert assessment.nearest_obstacle_distance_m is not None


def test_assess_commonroad_obstacles_ignores_far_lateral_obstacle() -> None:
    assessment = assess_commonroad_obstacles(
        _scenario(FakeObstacle(2, (15.0, 6.0))),
        _reference_path(),
        VehicleState(px=0.0, py=0.0, yaw=0.0, v=8.0),
        time_step=0,
    )

    assert assessment.collision_risk is False
    assert assessment.blocking_obstacle_ids == ()


def test_assess_commonroad_obstacles_ignores_same_speed_vehicle_ahead() -> None:
    assessment = assess_commonroad_obstacles(
        _scenario(FakeObstacle(3, (15.0, 0.5), speed=8.0)),
        _reference_path(),
        VehicleState(px=0.0, py=0.0, yaw=0.0, v=8.0),
        time_step=0,
    )

    assert assessment.collision_risk is False
    assert assessment.blocking_obstacle_ids == ()


def _reference_path() -> CommonRoadReferencePath:
    return CommonRoadReferencePath(
        points=((0.0, 0.0), (100.0, 0.0)),
        arc_lengths_m=(0.0, 100.0),
        speed=10.0,
        goal_speed=5.0,
        route_length_m=100.0,
        goal_time_s=None,
        source="test_reference",
        start_lanelet_id=1,
        goal_lanelet_ids=(2,),
    )


def _scenario(*obstacles: FakeObstacle) -> SimpleNamespace:
    return SimpleNamespace(dynamic_obstacles=list(obstacles), static_obstacles=[])
