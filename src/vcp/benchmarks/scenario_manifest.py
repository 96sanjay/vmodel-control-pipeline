from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class ScenarioManifestEntry:
    """One scenario entry from a benchmark suite manifest."""

    scenario_id: str
    difficulty: str
    scenario_type: str
    tags: tuple[str, ...] = field(default_factory=tuple)
    description: str | None = None


@dataclass(frozen=True)
class ScenarioSuite:
    """Configurable CommonRoad scenario suite definition."""

    suite_name: str
    sample_time: float
    scenario_root: Path
    scenarios: tuple[ScenarioManifestEntry, ...]
    source_path: Path
    download_note: str | None = None

    def expected_file_for(self, scenario_id: str) -> Path:
        return self.scenario_root / f"{scenario_id}.xml"

    @property
    def scenario_ids(self) -> tuple[str, ...]:
        return tuple(scenario.scenario_id for scenario in self.scenarios)


class ScenarioManifestError(ValueError):
    """Raised when a scenario suite manifest is invalid."""


def load_scenario_suite(path: str | Path) -> ScenarioSuite:
    """Load and validate a CommonRoad scenario suite manifest."""

    source_path = Path(path)
    with source_path.open(encoding="utf-8") as file:
        raw_data = yaml.safe_load(file) or {}

    if not isinstance(raw_data, dict):
        raise ScenarioManifestError(f"Scenario suite must be a YAML mapping: {source_path}")

    suite_name = _required_string(raw_data, "suite_name", source_path)
    sample_time = _required_positive_float(raw_data, "sample_time", source_path)
    scenario_root = _resolve_scenario_root(raw_data.get("scenario_root"), source_path)
    scenarios = _parse_scenarios(raw_data.get("scenarios"), source_path)

    return ScenarioSuite(
        suite_name=suite_name,
        sample_time=sample_time,
        scenario_root=scenario_root,
        scenarios=scenarios,
        source_path=source_path,
        download_note=_optional_string(raw_data, "download_note", source_path),
    )


def _resolve_scenario_root(value: Any, source_path: Path) -> Path:
    root_value = value if value is not None else "data/raw/commonroad/scenarios"
    if not isinstance(root_value, str) or not root_value.strip():
        raise ScenarioManifestError(f"'scenario_root' must be a non-empty string: {source_path}")

    root = Path(root_value)
    if root.is_absolute():
        return root

    return (source_path.parent / root).resolve()


def _parse_scenarios(value: Any, source_path: Path) -> tuple[ScenarioManifestEntry, ...]:
    if not isinstance(value, list) or not value:
        raise ScenarioManifestError(f"'scenarios' must be a non-empty list: {source_path}")

    entries: list[ScenarioManifestEntry] = []
    seen_ids: set[str] = set()

    for index, raw_entry in enumerate(value):
        if not isinstance(raw_entry, dict):
            raise ScenarioManifestError(
                f"Scenario entry at index {index} must be a mapping: {source_path}"
            )

        scenario_id = _required_string(raw_entry, "id", source_path)
        if scenario_id in seen_ids:
            raise ScenarioManifestError(f"Duplicate scenario id '{scenario_id}': {source_path}")
        seen_ids.add(scenario_id)

        tags = raw_entry.get("tags", [])
        if not isinstance(tags, list) or not all(isinstance(tag, str) for tag in tags):
            raise ScenarioManifestError(
                f"'tags' must be a list of strings for scenario '{scenario_id}': {source_path}"
            )

        entries.append(
            ScenarioManifestEntry(
                scenario_id=scenario_id,
                difficulty=_required_string(raw_entry, "difficulty", source_path),
                scenario_type=_required_string(raw_entry, "type", source_path),
                tags=tuple(tags),
                description=_optional_string(raw_entry, "description", source_path),
            )
        )

    return tuple(entries)


def _required_string(raw_data: dict[str, Any], key: str, source_path: Path) -> str:
    value = raw_data.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ScenarioManifestError(f"'{key}' must be a non-empty string: {source_path}")
    return value


def _optional_string(raw_data: dict[str, Any], key: str, source_path: Path) -> str | None:
    value = raw_data.get(key)
    if value is None:
        return None
    if not isinstance(value, str) or not value.strip():
        raise ScenarioManifestError(f"'{key}' must be a non-empty string when set: {source_path}")
    return value


def _required_positive_float(raw_data: dict[str, Any], key: str, source_path: Path) -> float:
    value = raw_data.get(key)
    if not isinstance(value, int | float) or value <= 0:
        raise ScenarioManifestError(f"'{key}' must be a positive number: {source_path}")
    return float(value)
