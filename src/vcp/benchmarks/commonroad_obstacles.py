"""Obstacle extraction and conservative route-occupancy checks for CommonRoad scenarios."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from vcp.benchmarks.commonroad_reference import (
    CommonRoadReferencePath,
    project_reference_path_progress,
    sample_reference_path_at_progress,
)
from vcp.models import VehicleParameters, VehicleState


@dataclass(frozen=True)
class CommonRoadObstacleConfig:
    nearby_radius_m: float = 45.0
    route_lookahead_m: float = 20.0
    time_headway_s: float = 2.0
    ttc_threshold_s: float = 1.5
    minimum_closing_speed_mps: float = 0.5
    route_lateral_margin_m: float = 0.75
    emergency_distance_m: float = 8.0
    vehicle_parameters: VehicleParameters = field(default_factory=VehicleParameters)

    def __post_init__(self) -> None:
        for name, value in (
            ("nearby_radius_m", self.nearby_radius_m),
            ("route_lookahead_m", self.route_lookahead_m),
            ("time_headway_s", self.time_headway_s),
            ("ttc_threshold_s", self.ttc_threshold_s),
            ("minimum_closing_speed_mps", self.minimum_closing_speed_mps),
            ("route_lateral_margin_m", self.route_lateral_margin_m),
            ("emergency_distance_m", self.emergency_distance_m),
        ):
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")


@dataclass(frozen=True)
class CommonRoadObstacleSnapshot:
    obstacle_id: int
    obstacle_kind: str
    px: float
    py: float
    yaw: float
    speed: float
    length_m: float
    width_m: float
    distance_to_ego_m: float
    route_progress_m: float
    ahead_distance_m: float
    route_lateral_offset_m: float


@dataclass(frozen=True)
class CommonRoadObstacleAssessment:
    nearby_obstacles: tuple[CommonRoadObstacleSnapshot, ...]
    blocking_obstacle_ids: tuple[int, ...]
    collision_risk: bool
    nearest_obstacle_distance_m: float | None
    min_blocking_distance_m: float | None


def assess_commonroad_obstacles(
    scenario_data: Any,
    reference_path: CommonRoadReferencePath,
    ego_state: VehicleState,
    *,
    time_step: int,
    config: CommonRoadObstacleConfig | None = None,
) -> CommonRoadObstacleAssessment:
    config = config or CommonRoadObstacleConfig()
    ego_position = np.array([ego_state.px, ego_state.py], dtype=float)
    ego_progress_m = project_reference_path_progress(reference_path, ego_position)
    snapshots: list[CommonRoadObstacleSnapshot] = []
    blocking_ids: list[int] = []

    for obstacle in _iter_obstacles(scenario_data):
        obstacle_state = _obstacle_state_at_time(obstacle, time_step)
        if obstacle_state is None:
            continue
        obstacle_position = np.asarray(
            getattr(obstacle_state, "position", ego_position),
            dtype=float,
        )
        distance_to_ego_m = float(np.linalg.norm(obstacle_position - ego_position))
        if distance_to_ego_m > config.nearby_radius_m:
            continue

        route_progress_m = project_reference_path_progress(reference_path, obstacle_position)
        route_sample = sample_reference_path_at_progress(reference_path, route_progress_m)
        ahead_distance_m = route_progress_m - ego_progress_m
        route_lateral_offset_m = float(
            np.linalg.norm(obstacle_position - np.array([route_sample.px, route_sample.py]))
        )
        length_m, width_m = _shape_dimensions(getattr(obstacle, "obstacle_shape", None))
        snapshot = CommonRoadObstacleSnapshot(
            obstacle_id=int(getattr(obstacle, "obstacle_id", -1)),
            obstacle_kind=type(obstacle).__name__,
            px=float(obstacle_position[0]),
            py=float(obstacle_position[1]),
            yaw=float(getattr(obstacle_state, "orientation", 0.0)),
            speed=float(getattr(obstacle_state, "velocity", 0.0)),
            length_m=length_m,
            width_m=width_m,
            distance_to_ego_m=distance_to_ego_m,
            route_progress_m=route_progress_m,
            ahead_distance_m=ahead_distance_m,
            route_lateral_offset_m=route_lateral_offset_m,
        )
        snapshots.append(snapshot)
        if _is_blocking(snapshot, ego_state, config):
            blocking_ids.append(snapshot.obstacle_id)

    nearest_distance_m = min(
        (snapshot.distance_to_ego_m for snapshot in snapshots),
        default=None,
    )
    min_blocking_distance_m = min(
        (
            snapshot.ahead_distance_m
            for snapshot in snapshots
            if snapshot.obstacle_id in blocking_ids
        ),
        default=None,
    )
    emergency_risk = (
        nearest_distance_m is not None
        and nearest_distance_m <= config.emergency_distance_m
    )
    return CommonRoadObstacleAssessment(
        nearby_obstacles=tuple(sorted(snapshots, key=lambda item: item.distance_to_ego_m)),
        blocking_obstacle_ids=tuple(sorted(set(blocking_ids))),
        collision_risk=bool(blocking_ids) or emergency_risk,
        nearest_obstacle_distance_m=nearest_distance_m,
        min_blocking_distance_m=min_blocking_distance_m,
    )


def _iter_obstacles(scenario_data: Any) -> tuple[Any, ...]:
    dynamic = tuple(getattr(scenario_data, "dynamic_obstacles", ()) or ())
    static = tuple(getattr(scenario_data, "static_obstacles", ()) or ())
    return dynamic + static


def _obstacle_state_at_time(obstacle: Any, time_step: int) -> Any | None:
    state_at_time = getattr(obstacle, "state_at_time", None)
    if callable(state_at_time):
        try:
            state = state_at_time(time_step)
        except Exception:
            state = None
        if state is not None:
            return state

    initial_state = getattr(obstacle, "initial_state", None)
    if initial_state is None:
        return None
    if getattr(initial_state, "time_step", 0) <= time_step:
        return initial_state
    return None


def _shape_dimensions(shape: Any) -> tuple[float, float]:
    length_m = getattr(shape, "length", None)
    width_m = getattr(shape, "width", None)
    if length_m is not None and width_m is not None:
        return float(length_m), float(width_m)
    radius = getattr(shape, "radius", None)
    if radius is not None:
        diameter = float(radius) * 2.0
        return diameter, diameter
    return 0.0, 0.0


def _is_blocking(
    obstacle: CommonRoadObstacleSnapshot,
    ego_state: VehicleState,
    config: CommonRoadObstacleConfig,
) -> bool:
    lookahead_distance_m = max(config.route_lookahead_m, ego_state.v * config.time_headway_s)
    lateral_threshold_m = (
        0.5 * (config.vehicle_parameters.width + obstacle.width_m) + config.route_lateral_margin_m
    )
    if not (
        0.0 <= obstacle.ahead_distance_m <= lookahead_distance_m
        and obstacle.route_lateral_offset_m <= lateral_threshold_m
    ):
        return False
    if obstacle.distance_to_ego_m <= config.emergency_distance_m:
        return True
    closing_speed_mps = max(ego_state.v - obstacle.speed, 0.0)
    if closing_speed_mps < config.minimum_closing_speed_mps:
        return False
    time_to_collision_s = obstacle.ahead_distance_m / max(closing_speed_mps, 1e-9)
    return time_to_collision_s <= config.ttc_threshold_s
