from __future__ import annotations

import argparse
import subprocess
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any

import yaml

DEFAULT_REPOSITORY = "https://gitlab.lrz.de/tum-cps/commonroad-scenarios.git"


def main() -> int:
    parser = argparse.ArgumentParser(description="Fetch real CommonRoad XML scenarios locally.")
    parser.add_argument(
        "--suite",
        type=Path,
        default=Path("configs/commonroad/real_scenario_suite.yaml"),
        help="Scenario suite YAML with source_path entries.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help="Override output directory for XML files.",
    )
    parser.add_argument(
        "--repository",
        default=None,
        help="Override CommonRoad scenario repository URL.",
    )
    parser.add_argument(
        "--revision",
        default=None,
        help="Override git revision to fetch.",
    )
    parser.add_argument(
        "--scenario-id",
        action="append",
        default=None,
        help="Fetch only the selected scenario ID. Repeat for multiple IDs.",
    )
    args = parser.parse_args()

    suite = _load_suite(args.suite)
    repo_url = args.repository or suite.get("source_repository") or DEFAULT_REPOSITORY
    revision = args.revision or suite.get("source_revision") or "HEAD"
    output_root = args.output_root or _resolve_suite_root(args.suite, suite)
    selected_ids = set(args.scenario_id or [])
    scenarios = [
        scenario
        for scenario in suite["scenarios"]
        if not selected_ids or str(scenario["id"]) in selected_ids
    ]
    if not scenarios:
        raise ValueError("No scenarios selected for download")

    output_root.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(prefix="vcp-commonroad-") as temp_dir:
        clone_dir = Path(temp_dir) / "commonroad-scenarios"
        _run(
            [
                "git",
                "clone",
                "--filter=blob:none",
                "--no-checkout",
                repo_url,
                str(clone_dir),
            ]
        )
        _ensure_revision_available(clone_dir, revision)
        for scenario in scenarios:
            scenario_id = str(scenario["id"])
            source_path = str(scenario["source_path"])
            xml_text = _git_show(clone_dir, revision, source_path)
            output_path = output_root / f"{scenario_id}.xml"
            output_path.write_text(xml_text, encoding="utf-8")
            print(f"Fetched {scenario_id} -> {output_path}")

    print(f"Fetched {len(scenarios)} scenario(s) into {output_root}")
    return 0


def _load_suite(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"Suite YAML must contain a mapping: {path}")
    scenarios = payload.get("scenarios")
    if not isinstance(scenarios, list) or not scenarios:
        raise ValueError(f"Suite YAML must contain a non-empty scenarios list: {path}")
    for scenario in scenarios:
        if not isinstance(scenario, dict) or "id" not in scenario or "source_path" not in scenario:
            raise ValueError("Each scenario must define id and source_path")
    return payload


def _resolve_suite_root(suite_path: Path, suite: dict[str, Any]) -> Path:
    raw_root = suite.get("scenario_root")
    if not raw_root:
        return Path("data/raw/commonroad/scenarios")
    path = Path(str(raw_root))
    if path.is_absolute():
        return path
    return (suite_path.parent / path).resolve()


def _git_show(clone_dir: Path, revision: str, source_path: str) -> str:
    result = _run(
        ["git", "-C", str(clone_dir), "show", f"{revision}:{source_path}"],
        capture_output=True,
    )
    return result.stdout


def _ensure_revision_available(clone_dir: Path, revision: str) -> None:
    result = subprocess.run(
        ["git", "-C", str(clone_dir), "cat-file", "-e", f"{revision}^{{commit}}"],
        capture_output=True,
        text=True,
    )
    if result.returncode == 0:
        return
    _run(["git", "-C", str(clone_dir), "fetch", "--depth=1", "origin", revision])


def _run(args: list[str], *, capture_output: bool = False) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        check=True,
        capture_output=capture_output,
        text=True,
    )


if __name__ == "__main__":
    raise SystemExit(main())
