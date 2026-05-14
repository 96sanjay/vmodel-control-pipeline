"""Lanelet-based reference extraction for CommonRoad scenarios."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class ReferencePathSample:
    px: float
    py: float
    heading: float
    speed: float


@dataclass(frozen=True)
class CommonRoadReferencePath:
    points: tuple[tuple[float, float], ...]
    speed: float
    source: str
    start_lanelet_id: int | None
    goal_lanelet_ids: tuple[int, ...]
    notes: tuple[str, ...] = ()


def build_commonroad_reference_path(
    scenario_data: Any,
    *,
    default_speed: float,
) -> CommonRoadReferencePath:
    """Build a simple drivable reference from CommonRoad lanelets and goals."""
    lanelet_network = scenario_data.lanelet_network
    initial_position = np.asarray(scenario_data.initial_state.position, dtype=float)
    goal_centers = _goal_centers(scenario_data.goal_region)

    start_ids = _lanelet_ids_at_position(lanelet_network, initial_position)
    goal_ids = _goal_lanelet_ids(scenario_data.goal_region)
    if not goal_ids:
        for center in goal_centers:
            goal_ids.extend(_lanelet_ids_at_position(lanelet_network, center))
    goal_ids = _unique_ints(goal_ids)

    notes: list[str] = []
    if not start_ids:
        heading = float(getattr(scenario_data.initial_state, "orientation", 0.0))
        fallback = np.vstack(
            [
                initial_position,
                initial_position
                + np.array([np.cos(heading), np.sin(heading)]) * max(default_speed, 1.0) * 3.0,
            ]
        )
        return _as_reference_path(
            fallback,
            default_speed,
            None,
            goal_ids,
            ("No start lanelet found; using initial heading fallback.",),
        )

    start_id = int(start_ids[0])
    route = _find_route(lanelet_network, start_id, goal_ids)
    if not route:
        route = _greedy_successor_route(lanelet_network, start_id)
        notes.append("No route to goal lanelet found; following available successors.")

    points = _points_from_route(
        lanelet_network,
        route,
        initial_position=initial_position,
        goal_center=goal_centers[0] if goal_centers else None,
    )
    return _as_reference_path(points, default_speed, start_id, goal_ids, tuple(notes))


def sample_reference_path_at_time(
    reference_path: CommonRoadReferencePath,
    time_s: float,
) -> ReferencePathSample:
    points = np.asarray(reference_path.points, dtype=float)
    if len(points) == 1:
        return ReferencePathSample(
            px=float(points[0, 0]),
            py=float(points[0, 1]),
            heading=0.0,
            speed=reference_path.speed,
        )

    deltas = np.diff(points, axis=0)
    segment_lengths = np.linalg.norm(deltas, axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(segment_lengths)])
    distance = max(float(time_s), 0.0) * max(reference_path.speed, 0.0)
    distance = min(distance, float(cumulative[-1]))

    segment_index = int(np.searchsorted(cumulative, distance, side="right") - 1)
    segment_index = min(max(segment_index, 0), len(segment_lengths) - 1)
    length = max(float(segment_lengths[segment_index]), 1e-9)
    ratio = (distance - float(cumulative[segment_index])) / length
    point = points[segment_index] + ratio * (points[segment_index + 1] - points[segment_index])
    direction = points[segment_index + 1] - points[segment_index]
    heading = float(np.arctan2(direction[1], direction[0]))
    return ReferencePathSample(
        px=float(point[0]),
        py=float(point[1]),
        heading=heading,
        speed=reference_path.speed,
    )


def _as_reference_path(
    points: np.ndarray,
    speed: float,
    start_lanelet_id: int | None,
    goal_lanelet_ids: list[int],
    notes: tuple[str, ...],
) -> CommonRoadReferencePath:
    cleaned = _deduplicate(np.asarray(points, dtype=float))
    if len(cleaned) == 0:
        cleaned = np.zeros((1, 2), dtype=float)
    return CommonRoadReferencePath(
        points=tuple((float(x), float(y)) for x, y in cleaned),
        speed=max(float(speed), 0.0),
        source="commonroad_lanelet_reference",
        start_lanelet_id=start_lanelet_id,
        goal_lanelet_ids=tuple(goal_lanelet_ids),
        notes=notes,
    )


def _lanelet_ids_at_position(lanelet_network: Any, position: np.ndarray) -> list[int]:
    try:
        result = lanelet_network.find_lanelet_by_position([position])
    except Exception:
        return []
    if not result:
        return []
    return _unique_ints(result[0])


def _goal_lanelet_ids(goal_region: Any) -> list[int]:
    lanelets = getattr(goal_region, "lanelets_of_goal_position", None)
    if not lanelets:
        return []
    ids: list[int] = []
    if isinstance(lanelets, dict):
        values = lanelets.values()
    else:
        values = lanelets
    for value in values:
        if isinstance(value, (list, tuple, set)):
            ids.extend(int(item) for item in value)
        elif value is not None:
            ids.append(int(value))
    return _unique_ints(ids)


def _goal_centers(goal_region: Any) -> list[np.ndarray]:
    centers: list[np.ndarray] = []
    for goal_state in getattr(goal_region, "state_list", []) or []:
        position = getattr(goal_state, "position", None)
        if position is None:
            continue
        centers.extend(_shape_centers(position))
    return centers


def _shape_centers(shape: Any) -> list[np.ndarray]:
    if hasattr(shape, "center"):
        return [np.asarray(shape.center, dtype=float)]
    if hasattr(shape, "shapes"):
        centers: list[np.ndarray] = []
        for child in shape.shapes:
            centers.extend(_shape_centers(child))
        return centers
    if hasattr(shape, "vertices"):
        vertices = np.asarray(shape.vertices, dtype=float)
        if len(vertices) > 0:
            return [np.mean(vertices, axis=0)]
    return []


def _find_route(lanelet_network: Any, start_id: int, goal_ids: list[int]) -> list[int]:
    if not goal_ids:
        return _greedy_successor_route(lanelet_network, start_id)
    goals = set(goal_ids)
    queue: deque[tuple[int, list[int]]] = deque([(start_id, [start_id])])
    visited = {start_id}
    while queue:
        lanelet_id, path = queue.popleft()
        if lanelet_id in goals:
            return path
        lanelet = lanelet_network.find_lanelet_by_id(lanelet_id)
        for neighbor_id in _neighbors(lanelet):
            if neighbor_id not in visited:
                visited.add(neighbor_id)
                queue.append((neighbor_id, [*path, neighbor_id]))
    return []


def _greedy_successor_route(lanelet_network: Any, start_id: int, limit: int = 8) -> list[int]:
    route = [start_id]
    current_id = start_id
    for _ in range(limit - 1):
        lanelet = lanelet_network.find_lanelet_by_id(current_id)
        successors = list(getattr(lanelet, "successor", []) or [])
        if not successors:
            break
        current_id = int(successors[0])
        if current_id in route:
            break
        route.append(current_id)
    return route


def _neighbors(lanelet: Any) -> list[int]:
    neighbors = [int(item) for item in (getattr(lanelet, "successor", []) or [])]
    for side in ("left", "right"):
        adjacent_id = getattr(lanelet, f"adj_{side}", None)
        same_direction = bool(getattr(lanelet, f"adj_{side}_same_direction", False))
        if adjacent_id is not None and same_direction:
            neighbors.append(int(adjacent_id))
    return _unique_ints(neighbors)


def _points_from_route(
    lanelet_network: Any,
    route: list[int],
    *,
    initial_position: np.ndarray,
    goal_center: np.ndarray | None,
) -> np.ndarray:
    if len(route) >= 2 and _is_adjacent_transition(lanelet_network, route[0], route[1]):
        points = _blend_adjacent_lanelets(
            lanelet_network.find_lanelet_by_id(route[0]),
            lanelet_network.find_lanelet_by_id(route[1]),
            initial_position=initial_position,
            goal_center=goal_center,
        )
        anchor = points[-1]
        remaining = route[2:]
    else:
        points = np.empty((0, 2), dtype=float)
        anchor = initial_position
        remaining = route

    for lanelet_id in remaining:
        lanelet = lanelet_network.find_lanelet_by_id(lanelet_id)
        segment = _trimmed_oriented_vertices(
            np.asarray(lanelet.center_vertices, dtype=float),
            start_position=anchor,
            goal_center=goal_center,
        )
        points = _append_segment(points, segment)
        anchor = points[-1]
    return points


def _is_adjacent_transition(lanelet_network: Any, first_id: int, second_id: int) -> bool:
    lanelet = lanelet_network.find_lanelet_by_id(first_id)
    for side in ("left", "right"):
        adjacent_id = getattr(lanelet, f"adj_{side}", None)
        same_direction = bool(getattr(lanelet, f"adj_{side}_same_direction", False))
        if adjacent_id is not None and int(adjacent_id) == int(second_id) and same_direction:
            return True
    return False


def _blend_adjacent_lanelets(
    start_lanelet: Any,
    goal_lanelet: Any,
    *,
    initial_position: np.ndarray,
    goal_center: np.ndarray | None,
) -> np.ndarray:
    start_segment = _trimmed_oriented_vertices(
        np.asarray(start_lanelet.center_vertices, dtype=float),
        start_position=initial_position,
        goal_center=goal_center,
    )
    goal_segment = _trimmed_oriented_vertices(
        np.asarray(goal_lanelet.center_vertices, dtype=float),
        start_position=initial_position,
        goal_center=goal_center,
    )
    count = max(12, len(start_segment), len(goal_segment))
    start_resampled = _resample_polyline(start_segment, count)
    goal_resampled = _resample_polyline(goal_segment, count)
    blend = np.linspace(0.0, 1.0, count)
    blend = blend * blend * (3.0 - 2.0 * blend)
    return (1.0 - blend[:, None]) * start_resampled + blend[:, None] * goal_resampled


def _trimmed_oriented_vertices(
    vertices: np.ndarray,
    *,
    start_position: np.ndarray,
    goal_center: np.ndarray | None,
) -> np.ndarray:
    if len(vertices) <= 1:
        return vertices
    oriented = vertices
    if np.linalg.norm(vertices[-1] - start_position) < np.linalg.norm(vertices[0] - start_position):
        oriented = vertices[::-1]

    start_index = int(np.argmin(np.linalg.norm(oriented - start_position, axis=1)))
    end_index = len(oriented) - 1
    if goal_center is not None:
        candidate = int(np.argmin(np.linalg.norm(oriented - goal_center, axis=1)))
        if candidate > start_index:
            end_index = candidate
    trimmed = oriented[start_index : end_index + 1]
    if len(trimmed) < 2:
        trimmed = oriented[start_index:]
    if len(trimmed) < 2:
        trimmed = oriented[:2]
    return trimmed


def _resample_polyline(points: np.ndarray, count: int) -> np.ndarray:
    if len(points) == 1:
        return np.repeat(points, count, axis=0)
    deltas = np.diff(points, axis=0)
    lengths = np.linalg.norm(deltas, axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(lengths)])
    if cumulative[-1] <= 1e-9:
        return np.repeat(points[:1], count, axis=0)
    targets = np.linspace(0.0, cumulative[-1], count)
    return np.column_stack(
        [
            np.interp(targets, cumulative, points[:, 0]),
            np.interp(targets, cumulative, points[:, 1]),
        ]
    )


def _append_segment(points: np.ndarray, segment: np.ndarray) -> np.ndarray:
    if len(points) == 0:
        return segment
    if np.linalg.norm(points[-1] - segment[0]) < 1e-6:
        segment = segment[1:]
    return np.vstack([points, segment]) if len(segment) else points


def _deduplicate(points: np.ndarray) -> np.ndarray:
    if len(points) <= 1:
        return points
    keep = [0]
    for index in range(1, len(points)):
        if np.linalg.norm(points[index] - points[keep[-1]]) > 1e-6:
            keep.append(index)
    return points[keep]


def _unique_ints(values: Any) -> list[int]:
    unique: list[int] = []
    for value in values or []:
        item = int(value)
        if item not in unique:
            unique.append(item)
    return unique
