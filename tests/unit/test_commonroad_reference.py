from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from vcp.benchmarks.commonroad_reference import (
    build_commonroad_reference_path,
    build_reference_horizon_from_state,
    sample_reference_path_at_time,
    sample_reference_path_for_state,
)


class FakeLanelet:
    def __init__(
        self,
        lanelet_id: int,
        center_vertices: list[tuple[float, float]],
        *,
        successor: list[int] | None = None,
        adj_left: int | None = None,
        adj_right: int | None = None,
        adj_left_same_direction: bool = False,
        adj_right_same_direction: bool = False,
    ) -> None:
        self.lanelet_id = lanelet_id
        self.center_vertices = np.asarray(center_vertices, dtype=float)
        self.successor = successor or []
        self.adj_left = adj_left
        self.adj_right = adj_right
        self.adj_left_same_direction = adj_left_same_direction
        self.adj_right_same_direction = adj_right_same_direction


class FakeLaneletNetwork:
    def __init__(self, lanelets: list[FakeLanelet]) -> None:
        self._lanelets = {lanelet.lanelet_id: lanelet for lanelet in lanelets}

    def find_lanelet_by_id(self, lanelet_id: int) -> FakeLanelet:
        return self._lanelets[lanelet_id]

    def find_lanelet_by_position(self, positions: list[np.ndarray]) -> list[list[int]]:
        result = []
        for position in positions:
            x, y = position
            if y > 2.0:
                result.append([2])
            elif x >= 50.0 and 2 in self._lanelets:
                result.append([2])
            else:
                result.append([1])
        return result


def test_commonroad_reference_blends_adjacent_lane_change() -> None:
    network = FakeLaneletNetwork(
        [
            FakeLanelet(
                1,
                [(0.0, 0.0), (100.0, 0.0)],
                adj_left=2,
                adj_left_same_direction=True,
            ),
            FakeLanelet(
                2,
                [(0.0, 4.0), (100.0, 4.0)],
                adj_right=1,
                adj_right_same_direction=True,
            ),
        ]
    )
    scenario_data = _scenario_data(network, start=(0.0, 0.0), goal_lanelet_id=2, goal=(100.0, 4.0))

    reference = build_commonroad_reference_path(scenario_data, default_speed=10.0)
    start = sample_reference_path_at_time(reference, 0.0)
    end = sample_reference_path_at_time(reference, 10.0)

    assert reference.start_lanelet_id == 1
    assert reference.goal_lanelet_ids == (2,)
    assert start.py == 0.0
    assert end.py > 3.5
    assert end.px > 95.0


def test_commonroad_reference_follows_successor_lanelets() -> None:
    network = FakeLaneletNetwork(
        [
            FakeLanelet(1, [(0.0, 0.0), (50.0, 0.0)], successor=[2]),
            FakeLanelet(2, [(50.0, 0.0), (100.0, 0.0)]),
        ]
    )
    scenario_data = _scenario_data(network, start=(0.0, 0.0), goal_lanelet_id=2, goal=(100.0, 0.0))

    reference = build_commonroad_reference_path(scenario_data, default_speed=1.0)
    sample = sample_reference_path_at_time(reference, 75.0)

    assert reference.start_lanelet_id == 1
    assert sample.px == 75.0
    assert sample.py == 0.0


def test_commonroad_reference_uses_goal_time_to_slow_short_route() -> None:
    network = FakeLaneletNetwork([FakeLanelet(1, [(0.0, 0.0), (40.0, 0.0)])])
    scenario_data = _scenario_data(
        network,
        start=(0.0, 0.0),
        goal_lanelet_id=1,
        goal=(40.0, 0.0),
        goal_time_steps=(9.0, 11.0),
        goal_velocity_mps=(12.0, 14.0),
        dt=1.0,
    )

    reference = build_commonroad_reference_path(scenario_data, default_speed=13.0)

    assert reference.route_length_m == 40.0
    assert reference.goal_time_s == 10.0
    assert reference.speed == 4.0


def test_commonroad_reference_target_and_horizon_follow_projected_progress() -> None:
    network = FakeLaneletNetwork(
        [
            FakeLanelet(1, [(0.0, 0.0), (50.0, 0.0)], successor=[2]),
            FakeLanelet(2, [(50.0, 0.0), (100.0, 0.0)]),
        ]
    )
    scenario_data = _scenario_data(network, start=(0.0, 0.0), goal_lanelet_id=2, goal=(100.0, 0.0))
    reference = build_commonroad_reference_path(scenario_data, default_speed=5.0)

    target = sample_reference_path_for_state(
        reference,
        position=np.asarray([60.0, 0.5], dtype=float),
        current_speed=5.0,
    )
    horizon = build_reference_horizon_from_state(
        reference,
        position=np.asarray([60.0, 0.5], dtype=float),
        current_speed=5.0,
        dt=1.0,
        horizon=2,
    )

    assert target.progress_m > 60.0
    assert horizon[0, 0] == 60.0
    assert horizon[0, 1] == 65.0
    assert horizon[0, 2] == 70.0


def _scenario_data(
    network: FakeLaneletNetwork,
    *,
    start: tuple[float, float],
    goal_lanelet_id: int,
    goal: tuple[float, float],
    goal_time_steps: tuple[float, float] | None = None,
    goal_velocity_mps: tuple[float, float] | None = None,
    dt: float = 0.1,
) -> SimpleNamespace:
    goal_state = SimpleNamespace(position=SimpleNamespace(center=np.asarray(goal, dtype=float)))
    if goal_time_steps is not None:
        goal_state.time_step = SimpleNamespace(start=goal_time_steps[0], end=goal_time_steps[1])
    if goal_velocity_mps is not None:
        goal_state.velocity = SimpleNamespace(
            start=goal_velocity_mps[0],
            end=goal_velocity_mps[1],
        )
    return SimpleNamespace(
        dt=dt,
        lanelet_network=network,
        initial_state=SimpleNamespace(position=np.asarray(start, dtype=float), orientation=0.0),
        goal_region=SimpleNamespace(
            lanelets_of_goal_position={0: [goal_lanelet_id]},
            state_list=[goal_state],
        ),
    )
