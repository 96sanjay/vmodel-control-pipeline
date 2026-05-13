from __future__ import annotations

import json
from pathlib import Path

from vcp.validation import BenchmarkRunner, MILRunnerConfig, write_mil_outputs

ROOT = Path(__file__).resolve().parents[2]


def test_commonroad_mil_smoke_generates_evidence(tmp_path: Path) -> None:
    runner = BenchmarkRunner(
        MILRunnerConfig(
            suite_path=ROOT / "configs/commonroad/scenario_suite.yaml",
            output_dir=tmp_path,
            steps=8,
            max_scenarios=1,
        )
    )

    results = runner.run("pid")
    artifacts = write_mil_outputs(results, tmp_path)

    assert len(results) == 1
    assert results[0].scenario_id == "DEU_Aachen-2_1_T-1"
    assert results[0].controller == "pid"
    assert results[0].scenario_source in {
        "synthetic_smoke_from_manifest",
        "commonroad_initial_state",
    }
    assert "lateral_rmse" in results[0].kpis
    assert artifacts["results_json"].exists()
    assert artifacts["summary_csv"].exists()
    assert artifacts["report_md"].exists()

    payload = json.loads(artifacts["results_json"].read_text(encoding="utf-8"))
    assert payload["suite_name"] == "commonroad_smoke"
    assert payload["summary"]["run_count"] == 1
    assert payload["runs"][0]["controller"] == "pid"
