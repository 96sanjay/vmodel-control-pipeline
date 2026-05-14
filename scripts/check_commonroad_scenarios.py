from __future__ import annotations

import argparse
import importlib.util
import os
from pathlib import Path
from typing import Any

import yaml

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")


def main() -> int:
    parser = argparse.ArgumentParser(description="Check local CommonRoad XML scenario readiness.")
    parser.add_argument(
        "--suite",
        type=Path,
        default=Path("configs/commonroad/real_scenario_suite.yaml"),
        help="Scenario suite YAML to check.",
    )
    args = parser.parse_args()

    suite = _load_suite(args.suite)
    scenario_root = _resolve_suite_root(args.suite, suite)
    commonroad_available = importlib.util.find_spec("commonroad") is not None
    commonroad_dc_available = importlib.util.find_spec("commonroad_dc") is not None
    commonroad_dc_functional, commonroad_dc_note = _check_commonroad_dc_functional()

    print(f"Suite: {suite.get('suite_name')}")
    print(f"Scenario root: {scenario_root}")
    print(f"commonroad-io installed: {commonroad_available}")
    print(f"commonroad-dc installed: {commonroad_dc_available}")
    print(f"commonroad-dc collision/boundary APIs import: {commonroad_dc_functional}")
    if commonroad_dc_note:
        print(f"commonroad-dc note: {commonroad_dc_note}")

    missing_count = 0
    for scenario in suite["scenarios"]:
        scenario_id = str(scenario["id"])
        path = scenario_root / f"{scenario_id}.xml"
        exists = path.exists()
        missing_count += int(not exists)
        status = "present" if exists else "missing"
        print(f"{scenario_id}: {status} ({path})")

    if missing_count:
        print(
            "Missing scenarios. Run: "
            f"python scripts/fetch_commonroad_scenarios.py --suite {args.suite}"
        )
        return 1

    if not commonroad_available:
        print("Scenario XML files are present, but commonroad-io is not installed.")
        print('Install with: pip install -e ".[commonroad]"')
        return 2

    print("All scenario XML files are present and commonroad-io is available.")
    return 0


def _check_commonroad_dc_functional() -> tuple[bool, str]:
    if importlib.util.find_spec("commonroad_dc") is None:
        return False, "commonroad_dc package is not installed"
    try:
        from commonroad_dc.boundary.boundary import create_road_boundary_obstacle
        from commonroad_dc.collision.collision_detection.pycrcc_collision_dispatch import (
            create_collision_checker,
        )
    except Exception as exc:
        return False, str(exc)
    if create_collision_checker is None or create_road_boundary_obstacle is None:
        return False, "CommonRoad-DC imports returned empty symbols"
    return True, ""


def _load_suite(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"Suite YAML must contain a mapping: {path}")
    return payload


def _resolve_suite_root(suite_path: Path, suite: dict[str, Any]) -> Path:
    raw_root = suite.get("scenario_root", "data/raw/commonroad/scenarios")
    path = Path(str(raw_root))
    if path.is_absolute():
        return path
    return (suite_path.parent / path).resolve()


if __name__ == "__main__":
    raise SystemExit(main())
