from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import pytest

from vcp.benchmarks.commonroad_loader import (
    CommonRoadScenarioLoader,
    ScenarioFileNotFoundError,
)
from vcp.benchmarks.scenario_manifest import load_scenario_suite


@dataclass
class FakePlanningProblem:
    initial_state: str = "initial-state"
    goal: str = "goal-region"


@dataclass
class FakePlanningProblemSet:
    planning_problem_dict: dict[int, FakePlanningProblem]


@dataclass
class FakeScenario:
    scenario_id: str = "DEU_Test-1_1_T-1"
    dt: float = 0.1
    lanelet_network: str = "lanelet-network"
    dynamic_obstacles: tuple[str, ...] = ("dynamic-obstacle",)
    static_obstacles: tuple[str, ...] = ("static-obstacle",)


def test_scenario_suite_manifest_loads_smoke_config() -> None:
    suite = load_scenario_suite("configs/commonroad/scenario_suite.yaml")

    assert suite.suite_name == "commonroad_smoke"
    assert suite.sample_time == 0.1
    assert "DEU_Aachen-2_1_T-1" in suite.scenario_ids
    assert not suite.scenario_root.is_absolute() or "data/raw/commonroad/scenarios" in str(
        suite.scenario_root
    )


def test_commonroad_loader_reports_missing_scenario(tmp_path: Path) -> None:
    loader = CommonRoadScenarioLoader(scenario_root=tmp_path)

    with pytest.raises(ScenarioFileNotFoundError, match="Place CommonRoad XML scenarios"):
        loader.load_scenario("DEU_Aachen-2_1_T-1")


def test_commonroad_loader_extracts_core_fields(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    scenario_path = tmp_path / "DEU_Test-1_1_T-1.xml"
    scenario_path.write_text("<commonRoad />", encoding="utf-8")

    def fake_read_commonroad_file(path: Path) -> tuple[FakeScenario, FakePlanningProblemSet]:
        assert path == scenario_path
        return FakeScenario(), FakePlanningProblemSet({1: FakePlanningProblem()})

    monkeypatch.setattr(
        "vcp.benchmarks.commonroad_loader._read_commonroad_file",
        fake_read_commonroad_file,
    )

    loader = CommonRoadScenarioLoader(scenario_root=tmp_path)
    scenario_data = loader.load_scenario("DEU_Test-1_1_T-1")

    assert scenario_data.scenario_id == "DEU_Test-1_1_T-1"
    assert scenario_data.dt == 0.1
    assert scenario_data.initial_state == "initial-state"
    assert scenario_data.goal_region == "goal-region"
    assert scenario_data.lanelet_network == "lanelet-network"
    assert scenario_data.dynamic_obstacles == ("dynamic-obstacle",)
    assert scenario_data.static_obstacles == ("static-obstacle",)
    assert scenario_data.planning_problem_id == 1
    assert scenario_data.source_path == scenario_path
