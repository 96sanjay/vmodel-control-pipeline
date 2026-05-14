from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from vcp.models import (
    VehicleConstraints,
    VehicleInput,
    VehicleParameters,
    VehicleState,
    check_input_constraints,
    check_state_constraints,
)


@dataclass(frozen=True)
class CommonRoadDrivabilityConfig:
    """Configuration for optional CommonRoad-specific validation checks."""

    vehicle_parameters: VehicleParameters = field(default_factory=VehicleParameters)
    constraints: VehicleConstraints = field(default_factory=VehicleConstraints)
    lanelet_membership_check: bool = True
    road_boundary_collision_check: bool = True
    obstacle_collision_check: bool = True


@dataclass(frozen=True)
class CommonRoadDrivabilityResult:
    """Per-run CommonRoad validation evidence for a closed-loop trajectory."""

    collision_flags: tuple[bool, ...]
    road_boundary_violation_flags: tuple[bool, ...]
    kinematic_violation_flags: tuple[bool, ...]
    checked_with_commonroad_dc: bool
    checked_with_lanelet_network: bool
    notes: tuple[str, ...]

    @property
    def collision_count(self) -> int:
        return sum(int(flag) for flag in self.collision_flags)

    @property
    def road_boundary_violation_count(self) -> int:
        return sum(int(flag) for flag in self.road_boundary_violation_flags)

    @property
    def kinematic_violation_count(self) -> int:
        return sum(int(flag) for flag in self.kinematic_violation_flags)


class CommonRoadDrivabilityCheckError(RuntimeError):
    """Raised when CommonRoad-specific validation cannot be completed."""


def check_commonroad_drivability(
    scenario_data: Any,
    rows: list[dict[str, Any]],
    *,
    config: CommonRoadDrivabilityConfig | None = None,
) -> CommonRoadDrivabilityResult:
    """Run available CommonRoad-specific checks over MIL time-series rows."""

    if not rows:
        raise ValueError("rows must not be empty")

    config = config or CommonRoadDrivabilityConfig()
    notes: list[str] = []
    lanelet_flags, used_lanelet_network = _lanelet_boundary_flags(scenario_data, rows, config)
    collision_flags, used_collision_checker, collision_notes = _collision_flags(
        scenario_data,
        rows,
        config,
    )
    road_boundary_flags, used_boundary_checker, boundary_notes = _road_boundary_flags(
        scenario_data,
        rows,
        config,
    )
    kinematic_flags = tuple(_has_kinematic_violation(row, config.constraints) for row in rows)

    notes.extend(collision_notes)
    notes.extend(boundary_notes)
    if not used_lanelet_network:
        notes.append("Lanelet membership check unavailable; no lanelet-network method found.")

    merged_boundary_flags = tuple(
        bool(lanelet_flag or boundary_flag)
        for lanelet_flag, boundary_flag in zip(lanelet_flags, road_boundary_flags, strict=True)
    )

    return CommonRoadDrivabilityResult(
        collision_flags=collision_flags,
        road_boundary_violation_flags=merged_boundary_flags,
        kinematic_violation_flags=kinematic_flags,
        checked_with_commonroad_dc=used_collision_checker or used_boundary_checker,
        checked_with_lanelet_network=used_lanelet_network,
        notes=tuple(dict.fromkeys(notes)),
    )


def annotate_rows_with_commonroad_drivability(
    scenario_data: Any,
    rows: list[dict[str, Any]],
    *,
    config: CommonRoadDrivabilityConfig | None = None,
) -> CommonRoadDrivabilityResult:
    """Annotate MIL rows with CommonRoad-specific checks and return the check result."""

    result = check_commonroad_drivability(scenario_data, rows, config=config)
    note_text = " | ".join(result.notes)
    for index, row in enumerate(rows):
        row["collision_flag"] = bool(row.get("collision_flag", False)) or result.collision_flags[
            index
        ]
        row["road_boundary_violation"] = bool(
            row.get("road_boundary_violation", False)
        ) or result.road_boundary_violation_flags[index]
        row["commonroad_kinematic_violation"] = result.kinematic_violation_flags[index]
        row["commonroad_dc_checked"] = result.checked_with_commonroad_dc
        row["commonroad_lanelet_checked"] = result.checked_with_lanelet_network
        row["commonroad_check_notes"] = note_text
        row["constraint_violation_count"] = int(row.get("constraint_violation_count", 0)) + int(
            result.kinematic_violation_flags[index]
        )
    return result


def _lanelet_boundary_flags(
    scenario_data: Any,
    rows: list[dict[str, Any]],
    config: CommonRoadDrivabilityConfig,
) -> tuple[tuple[bool, ...], bool]:
    if not config.lanelet_membership_check:
        return tuple(False for _ in rows), False

    lanelet_network = getattr(scenario_data, "lanelet_network", None)
    find_lanelet_by_position = getattr(lanelet_network, "find_lanelet_by_position", None)
    if not callable(find_lanelet_by_position):
        return tuple(False for _ in rows), False

    flags: list[bool] = []
    for row in rows:
        position = [float(row["px"]), float(row["py"])]
        try:
            result = find_lanelet_by_position([position])
        except TypeError:
            result = find_lanelet_by_position(position)
        flags.append(not _lanelet_result_contains_lanelet(result))
    return tuple(flags), True


def _collision_flags(
    scenario_data: Any,
    rows: list[dict[str, Any]],
    config: CommonRoadDrivabilityConfig,
) -> tuple[tuple[bool, ...], bool, tuple[str, ...]]:
    if not config.obstacle_collision_check:
        return tuple(False for _ in rows), False, ()

    try:
        import commonroad_dc.pycrcc as pycrcc
        from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import (
            create_collision_checker,
        )
    except ImportError:
        return (
            tuple(False for _ in rows),
            False,
            (
                "Obstacle collision check skipped; install commonroad-drivability-checker "
                "to enable CommonRoad-DC collision objects.",
            ),
        )

    scenario = getattr(scenario_data, "scenario", None)
    if scenario is None:
        return (
            tuple(False for _ in rows),
            False,
            ("Obstacle collision check skipped; no scenario.",),
        )

    try:
        collision_checker = create_collision_checker(scenario)
    except Exception as exc:  # pragma: no cover - depends on optional CommonRoad-DC internals
        return tuple(False for _ in rows), False, (f"Obstacle collision checker failed: {exc}",)

    flags: list[bool] = []
    for step_index, row in enumerate(rows):
        ego = _ego_collision_object(pycrcc, row, config.vehicle_parameters)
        checker = _time_slice(collision_checker, step_index)
        try:
            flags.append(bool(checker.collide(ego)))
        except Exception as exc:  # pragma: no cover - depends on optional CommonRoad-DC internals
            return tuple(False for _ in rows), False, (f"Obstacle collision query failed: {exc}",)
    return tuple(flags), True, ()


def _road_boundary_flags(
    scenario_data: Any,
    rows: list[dict[str, Any]],
    config: CommonRoadDrivabilityConfig,
) -> tuple[tuple[bool, ...], bool, tuple[str, ...]]:
    if not config.road_boundary_collision_check:
        return tuple(False for _ in rows), False, ()

    try:
        import commonroad_dc.pycrcc as pycrcc
        from commonroad_dc.boundary.boundary import create_road_boundary_obstacle
    except ImportError:
        return (
            tuple(False for _ in rows),
            False,
            (
                "Road-boundary collision check skipped; install commonroad-drivability-checker "
                "to enable boundary obstacle generation.",
            ),
        )

    scenario = getattr(scenario_data, "scenario", None)
    if scenario is None:
        return tuple(False for _ in rows), False, ("Road-boundary check skipped; no scenario.",)

    try:
        boundary_obstacles = create_road_boundary_obstacle(
            scenario,
            method="obb_rectangles",
            return_scenario_obstacle=False,
        )
    except Exception as exc:  # pragma: no cover - depends on optional CommonRoad-DC internals
        return tuple(False for _ in rows), False, (f"Road-boundary checker failed: {exc}",)

    flags: list[bool] = []
    for row in rows:
        ego = _ego_collision_object(pycrcc, row, config.vehicle_parameters)
        try:
            flags.append(bool(boundary_obstacles.collide(ego)))
        except Exception as exc:  # pragma: no cover - depends on optional CommonRoad-DC internals
            return tuple(False for _ in rows), False, (f"Road-boundary query failed: {exc}",)
    return tuple(flags), True, ()


def _ego_collision_object(pycrcc: Any, row: dict[str, Any], parameters: VehicleParameters) -> Any:
    return pycrcc.RectOBB(
        parameters.length / 2.0,
        parameters.width / 2.0,
        float(row["yaw"]),
        float(row["px"]),
        float(row["py"]),
    )


def _time_slice(collision_checker: Any, step_index: int) -> Any:
    time_slice = getattr(collision_checker, "time_slice", None)
    if callable(time_slice):
        return time_slice(step_index)
    return collision_checker


def _has_kinematic_violation(row: dict[str, Any], constraints: VehicleConstraints) -> bool:
    state = VehicleState(
        px=float(row["px"]),
        py=float(row["py"]),
        yaw=float(row["yaw"]),
        v=float(row["v"]),
    )
    command = VehicleInput(
        acceleration=float(row["applied_acceleration_cmd"]),
        steering_angle=float(row["applied_steering_cmd"]),
    )
    return not (
        check_state_constraints(state, constraints).is_valid
        and check_input_constraints(command, constraints).is_valid
    )


def _lanelet_result_contains_lanelet(result: Any) -> bool:
    if result is None:
        return False
    if isinstance(result, int):
        return result >= 0
    if isinstance(result, (list, tuple, set)):
        if not result:
            return False
        first = next(iter(result))
        if isinstance(first, (list, tuple, set)):
            return bool(first)
        return True
    return bool(result)


__all__ = [
    "CommonRoadDrivabilityCheckError",
    "CommonRoadDrivabilityConfig",
    "CommonRoadDrivabilityResult",
    "annotate_rows_with_commonroad_drivability",
    "check_commonroad_drivability",
]
