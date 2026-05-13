from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

DEFAULT_SCENARIO_ROOT = Path("data/raw/commonroad/scenarios")


@dataclass(frozen=True)
class ScenarioData:
    """CommonRoad scenario data normalized for the validation framework."""

    scenario_id: str
    dt: float
    initial_state: Any
    goal_region: Any
    lanelet_network: Any
    dynamic_obstacles: tuple[Any, ...]
    static_obstacles: tuple[Any, ...]
    scenario: Any
    planning_problem: Any
    planning_problem_id: int | str | None
    source_path: Path


class CommonRoadLoaderError(RuntimeError):
    """Base class for CommonRoad loader failures."""


class CommonRoadDependencyError(CommonRoadLoaderError):
    """Raised when CommonRoad dependencies are not installed."""


class ScenarioFileNotFoundError(CommonRoadLoaderError):
    """Raised when a requested scenario file is missing."""


class PlanningProblemNotFoundError(CommonRoadLoaderError):
    """Raised when a requested planning problem cannot be found in a scenario."""


class CommonRoadScenarioLoader:
    """Load CommonRoad scenarios from a configurable local scenario root."""

    def __init__(self, scenario_root: str | Path = DEFAULT_SCENARIO_ROOT) -> None:
        self.scenario_root = Path(scenario_root)

    def load_scenario(
        self,
        scenario_id: str,
        *,
        planning_problem_id: int | str | None = None,
    ) -> ScenarioData:
        """Load a scenario by CommonRoad scenario ID."""

        scenario_path = self.find_scenario_file(scenario_id)
        return self.load_file(scenario_path, planning_problem_id=planning_problem_id)

    def load_file(
        self,
        scenario_path: str | Path,
        *,
        planning_problem_id: int | str | None = None,
    ) -> ScenarioData:
        """Load a CommonRoad XML scenario file."""

        path = Path(scenario_path)
        if not path.exists():
            raise self._missing_file_error(path.stem, path)

        scenario, planning_problem_set = _read_commonroad_file(path)
        selected_id, planning_problem = _select_planning_problem(
            planning_problem_set,
            planning_problem_id,
            path,
        )

        return ScenarioData(
            scenario_id=str(getattr(scenario, "scenario_id", path.stem)),
            dt=float(scenario.dt),
            initial_state=getattr(planning_problem, "initial_state", None),
            goal_region=getattr(planning_problem, "goal", None),
            lanelet_network=getattr(scenario, "lanelet_network", None),
            dynamic_obstacles=tuple(getattr(scenario, "dynamic_obstacles", []) or []),
            static_obstacles=tuple(getattr(scenario, "static_obstacles", []) or []),
            scenario=scenario,
            planning_problem=planning_problem,
            planning_problem_id=selected_id,
            source_path=path,
        )

    def find_scenario_file(self, scenario_id: str) -> Path:
        """Find a scenario XML file by ID below the configured scenario root."""

        direct_path = self.scenario_root / f"{scenario_id}.xml"
        if direct_path.exists():
            return direct_path

        matches = sorted(self.scenario_root.glob(f"**/{scenario_id}.xml"))
        if matches:
            return matches[0]

        raise self._missing_file_error(scenario_id, direct_path)

    def _missing_file_error(
        self,
        scenario_id: str,
        expected_path: Path,
    ) -> ScenarioFileNotFoundError:
        message = (
            f"CommonRoad scenario '{scenario_id}' was not found.\n"
            f"Expected file: {expected_path}\n"
            "Place CommonRoad XML scenarios under the configured scenario_root, for example:\n"
            f"  {self.scenario_root}/DEU_Aachen-2_1_T-1.xml\n"
            "Or pass a different root via CommonRoadScenarioLoader(scenario_root=...).\n"
            "The repository intentionally does not vendor benchmark data; track downloaded data "
            "with DVC or a manifest."
        )
        return ScenarioFileNotFoundError(message)


def _read_commonroad_file(path: Path) -> tuple[Any, Any]:
    try:
        from commonroad.common.file_reader import CommonRoadFileReader
    except ImportError as exc:
        raise CommonRoadDependencyError(
            "CommonRoad is not installed. Install the optional benchmark dependencies before "
            "loading real scenarios, for example:\n"
            "  pip install commonroad-io\n"
            "Then place scenario XML files under data/raw/commonroad/scenarios."
        ) from exc

    return CommonRoadFileReader(str(path)).open()


def _select_planning_problem(
    planning_problem_set: Any,
    requested_id: int | str | None,
    scenario_path: Path,
) -> tuple[int | str | None, Any]:
    planning_problem_dict = getattr(planning_problem_set, "planning_problem_dict", None)
    if not planning_problem_dict:
        raise PlanningProblemNotFoundError(
            f"No planning problems were found in CommonRoad scenario: {scenario_path}"
        )

    if requested_id is None:
        selected_id = next(iter(planning_problem_dict))
        return selected_id, planning_problem_dict[selected_id]

    candidate_ids = (requested_id,)
    if isinstance(requested_id, str):
        candidate_ids = (requested_id, _maybe_int(requested_id))

    for candidate_id in candidate_ids:
        if candidate_id in planning_problem_dict:
            return candidate_id, planning_problem_dict[candidate_id]

    available_ids = ", ".join(str(key) for key in planning_problem_dict)
    raise PlanningProblemNotFoundError(
        f"Planning problem '{requested_id}' was not found in {scenario_path}. "
        f"Available planning problem IDs: {available_ids}"
    )


def _maybe_int(value: str) -> int | str:
    try:
        return int(value)
    except ValueError:
        return value
