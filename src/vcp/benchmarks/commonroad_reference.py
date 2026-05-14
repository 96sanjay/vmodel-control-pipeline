"""Lanelet-based reference extraction and progress tracking for CommonRoad scenarios."""

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
    progress_m: float
    remaining_distance_m: float


@dataclass(frozen=True)
class CommonRoadReferencePath:
    points: tuple[tuple[float, float], ...]
    arc_lengths_m: tuple[float, ...]
    speed: float
    goal_speed: float
    route_length_m: float
    goal_time_s: float | None
    source: str
    start_lanelet_id: int | None
    goal_lanelet_ids: tuple[int, ...]
    notes: tuple[str, ...] = ()
    lookahead_time_s: float = 0.35


def build_commonroad_reference_path(
    scenario_data: Any,
    *,
    default_speed: float,
) -> CommonRoadReferencePath:
    """Build a drivable route reference from lanelets, goals, and simple timing hints."""

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
            scenario_data=scenario_data,
            default_speed=default_speed,
            start_lanelet_id=None,
            goal_lanelet_ids=goal_ids,
            notes=("No start lanelet found; using initial heading fallback.",),
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
    return _as_reference_path(
        points,
        scenario_data=scenario_data,
        default_speed=default_speed,
        start_lanelet_id=start_id,
        goal_lanelet_ids=goal_ids,
        notes=tuple(notes),
    )


def sample_reference_path_at_time(
    reference_path: CommonRoadReferencePath,
    time_s: float,
) -> ReferencePathSample:
    progress_m = max(float(time_s), 0.0) * max(reference_path.speed, 0.0)
    return sample_reference_path_at_progress(reference_path, progress_m)


def sample_reference_path_at_progress(
    reference_path: CommonRoadReferencePath,
    progress_m: float,
) -> ReferencePathSample:
    points = np.asarray(reference_path.points, dtype=float)
    arc_lengths = np.asarray(reference_path.arc_lengths_m, dtype=float)
    route_length = float(reference_path.route_length_m)
    clamped_progress = min(max(float(progress_m), 0.0), route_length)

    if len(points) == 1:
        return ReferencePathSample(
            px=float(points[0, 0]),
            py=float(points[0, 1]),
            heading=0.0,
            speed=_speed_at_progress(reference_path, clamped_progress),
            progress_m=clamped_progress,
            remaining_distance_m=max(route_length - clamped_progress, 0.0),
        )

    segment_index = int(np.searchsorted(arc_lengths, clamped_progress, side="right") - 1)
    segment_index = min(max(segment_index, 0), len(points) - 2)
    segment_start = arc_lengths[segment_index]
    segment_end = arc_lengths[segment_index + 1]
    segment_length = max(float(segment_end - segment_start), 1e-9)
    ratio = (clamped_progress - float(segment_start)) / segment_length
    point = points[segment_index] + ratio * (points[segment_index + 1] - points[segment_index])
    direction = points[segment_index + 1] - points[segment_index]
    heading = float(np.arctan2(direction[1], direction[0]))
    return ReferencePathSample(
        px=float(point[0]),
        py=float(point[1]),
        heading=heading,
        speed=_speed_at_progress(reference_path, clamped_progress),
        progress_m=clamped_progress,
        remaining_distance_m=max(route_length - clamped_progress, 0.0),
    )


def project_reference_path_progress(
    reference_path: CommonRoadReferencePath,
    position: np.ndarray,
) -> float:
    points = np.asarray(reference_path.points, dtype=float)
    arc_lengths = np.asarray(reference_path.arc_lengths_m, dtype=float)
    if len(points) <= 1:
        return 0.0

    best_distance = float("inf")
    best_progress = 0.0
    for index in range(len(points) - 1):
        start = points[index]
        end = points[index + 1]
        delta = end - start
        length_sq = float(np.dot(delta, delta))
        if length_sq <= 1e-12:
            continue
        ratio = float(np.dot(position - start, delta) / length_sq)
        ratio = min(max(ratio, 0.0), 1.0)
        projection = start + ratio * delta
        distance = float(np.linalg.norm(position - projection))
        if distance < best_distance:
            best_distance = distance
            segment_length = float(arc_lengths[index + 1] - arc_lengths[index])
            best_progress = float(arc_lengths[index] + ratio * segment_length)
    return best_progress


def sample_reference_path_for_state(
    reference_path: CommonRoadReferencePath,
    *,
    position: np.ndarray,
    current_speed: float,
) -> ReferencePathSample:
    base_progress = project_reference_path_progress(
        reference_path,
        np.asarray(position, dtype=float),
    )
    base_sample = sample_reference_path_at_progress(reference_path, base_progress)
    lookahead_distance = (
        max(base_sample.speed, float(current_speed), 1.0) * reference_path.lookahead_time_s
    )
    return sample_reference_path_at_progress(reference_path, base_progress + lookahead_distance)


def build_reference_horizon_from_state(
    reference_path: CommonRoadReferencePath,
    *,
    position: np.ndarray,
    current_speed: float,
    dt: float,
    horizon: int,
) -> np.ndarray:
    reference = np.zeros((4, horizon + 1), dtype=np.float64)
    progress_m = project_reference_path_progress(reference_path, np.asarray(position, dtype=float))
    sample = sample_reference_path_at_progress(reference_path, progress_m)
    for step in range(horizon + 1):
        if step > 0:
            progress_m += max(sample.speed, float(current_speed), 0.5) * dt
            sample = sample_reference_path_at_progress(reference_path, progress_m)
        reference[:, step] = np.array(
            [sample.px, sample.py, sample.heading, sample.speed],
            dtype=np.float64,
        )
    return reference


def _as_reference_path(
    points: np.ndarray,
    *,
    scenario_data: Any,
    default_speed: float,
    start_lanelet_id: int | None,
    goal_lanelet_ids: list[int],
    notes: tuple[str, ...],
) -> CommonRoadReferencePath:
    cleaned = _deduplicate(np.asarray(points, dtype=float))
    if len(cleaned) == 0:
        cleaned = np.zeros((1, 2), dtype=float)
    arc_lengths = _arc_lengths(cleaned)
    route_length_m = float(arc_lengths[-1]) if len(arc_lengths) else 0.0
    goal_speed = _goal_speed_mps(scenario_data.goal_region)
    goal_time_s = _goal_time_s(scenario_data.goal_region, float(getattr(scenario_data, "dt", 0.1)))
    nominal_speed = _infer_reference_speed(
        route_length_m=route_length_m,
        default_speed=default_speed,
        goal_speed=goal_speed,
        goal_time_s=goal_time_s,
    )
    terminal_speed = min(goal_speed if goal_speed is not None else nominal_speed, nominal_speed)
    return CommonRoadReferencePath(
        points=tuple((float(x), float(y)) for x, y in cleaned),
        arc_lengths_m=tuple(float(value) for value in arc_lengths),
        speed=nominal_speed,
        goal_speed=max(float(terminal_speed), 0.5),
        route_length_m=route_length_m,
        goal_time_s=goal_time_s,
        source="commonroad_lanelet_reference",
        start_lanelet_id=start_lanelet_id,
        goal_lanelet_ids=tuple(goal_lanelet_ids),
        notes=notes,
    )


def _goal_speed_mps(goal_region: Any) -> float | None:
    speeds: list[float] = []
    for goal_state in getattr(goal_region, "state_list", []) or []:
        midpoint = _interval_midpoint(getattr(goal_state, "velocity", None))
        if midpoint is not None and midpoint > 0.0:
            speeds.append(midpoint)
    return min(speeds) if speeds else None


def _goal_time_s(goal_region: Any, dt: float) -> float | None:
    durations: list[float] = []
    for goal_state in getattr(goal_region, "state_list", []) or []:
        midpoint = _interval_midpoint(getattr(goal_state, "time_step", None))
        if midpoint is not None and midpoint > 0.0:
            durations.append(midpoint * dt)
    return min(durations) if durations else None


def _interval_midpoint(value: Any) -> float | None:
    if value is None:
        return None
    if hasattr(value, "start") and hasattr(value, "end"):
        return 0.5 * (float(value.start) + float(value.end))
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _infer_reference_speed(
    *,
    route_length_m: float,
    default_speed: float,
    goal_speed: float | None,
    goal_time_s: float | None,
) -> float:
    speed = max(float(default_speed), 0.5)
    if goal_speed is not None and goal_speed > 0.0:
        speed = min(speed, goal_speed)
    if goal_time_s is not None and goal_time_s > 0.0 and route_length_m > 0.0:
        time_speed = route_length_m / goal_time_s
        plausible = goal_speed is None or time_speed <= max(goal_speed * 1.6, 2.0)
        if plausible and time_speed < speed:
            speed = max(time_speed, 0.5)
    return speed


def _speed_at_progress(reference_path: CommonRoadReferencePath, progress_m: float) -> float:
    remaining = max(reference_path.route_length_m - progress_m, 0.0)
    slowdown_distance = min(
        reference_path.route_length_m,
        max(8.0, 2.0 * reference_path.speed),
    )
    if slowdown_distance <= 1e-9 or remaining >= slowdown_distance:
        return reference_path.speed
    ratio = min(max(remaining / slowdown_distance, 0.0), 1.0)
    ratio = ratio * ratio * (3.0 - 2.0 * ratio)
    return reference_path.goal_speed + (reference_path.speed - reference_path.goal_speed) * ratio


def _arc_lengths(points: np.ndarray) -> np.ndarray:
    if len(points) <= 1:
        return np.zeros((len(points),), dtype=float)
    deltas = np.diff(points, axis=0)
    lengths = np.linalg.norm(deltas, axis=1)
    return np.concatenate([[0.0], np.cumsum(lengths)])


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
    values = lanelets.values() if isinstance(lanelets, dict) else lanelets
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
        if position is not None:
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
