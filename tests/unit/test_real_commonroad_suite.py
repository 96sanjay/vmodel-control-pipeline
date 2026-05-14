from __future__ import annotations

from pathlib import Path

import yaml


def test_real_commonroad_suite_has_seven_fetchable_scenarios() -> None:
    suite_path = Path("configs/commonroad/real_scenario_suite.yaml")
    suite = yaml.safe_load(suite_path.read_text(encoding="utf-8"))

    assert suite["suite_name"] == "commonroad_real_public_7"
    assert len(suite["scenarios"]) == 7
    assert suite["source_repository"].startswith("https://gitlab.lrz.de/")
    assert suite["source_revision"]
    for scenario in suite["scenarios"]:
        assert scenario["id"].endswith("_T-1")
        assert scenario["source_path"].endswith(f"{scenario['id']}.xml")
        assert "real_xml" in scenario["tags"]
