from __future__ import annotations

import csv
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]


def _load_yaml(path: str) -> dict:
    with (ROOT / path).open(encoding="utf-8") as file:
        return yaml.safe_load(file)


def test_requirements_hazards_and_traceability_are_linked() -> None:
    system_requirements = _load_yaml("docs/requirements/system_requirements.yaml")[
        "requirements"
    ]
    software_requirements = _load_yaml("docs/requirements/software_requirements.yaml")[
        "requirements"
    ]
    hazards = _load_yaml("docs/hazards/hazard_log.yaml")["hazards"]

    system_ids = {requirement["id"] for requirement in system_requirements}
    software_ids = {requirement["id"] for requirement in software_requirements}

    assert system_ids
    assert software_ids

    for requirement in system_requirements:
        assert requirement["verification"]

    for requirement in software_requirements:
        assert requirement["verification"]
        assert requirement["design_element"]
        assert set(requirement["linked_system_requirements"]) <= system_ids

    for hazard in hazards:
        assert hazard["linked_requirements"]
        assert set(hazard["linked_requirements"]) <= system_ids

    with (ROOT / "docs/requirements/traceability_matrix.csv").open(
        encoding="utf-8", newline=""
    ) as file:
        trace_rows = list(csv.DictReader(file))

    traced_ids = {row["requirement_id"] for row in trace_rows}
    assert traced_ids == system_ids

    for row in trace_rows:
        assert row["verification_test_id"]
        assert row["verification_stage"]
        assert row["evidence_artifact"]
