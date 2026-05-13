from __future__ import annotations

import argparse
from pathlib import Path

from vcp.validation.mil_runner import (
    BenchmarkRunner,
    MILRunnerConfig,
    write_mil_outputs,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run a CI-friendly MIL smoke benchmark.")
    parser.add_argument(
        "--suite",
        type=Path,
        required=True,
        help="Path to a CommonRoad scenario suite YAML manifest.",
    )
    parser.add_argument(
        "--controller",
        choices=["pid", "lqr", "linear_mpc", "nmpc", "all"],
        required=True,
        help="Controller to run.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts/mil"),
        help="Directory for JSON, CSV, Markdown, and SVG outputs.",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=25,
        help="Number of closed-loop simulation steps per scenario.",
    )
    parser.add_argument(
        "--max-scenarios",
        type=int,
        default=1,
        help="Maximum scenarios to run; use 0 for every manifest scenario.",
    )
    parser.add_argument(
        "--target-speed",
        type=float,
        default=4.0,
        help="Synthetic smoke target speed in m/s.",
    )
    args = parser.parse_args(argv)

    runner = BenchmarkRunner(
        MILRunnerConfig(
            suite_path=args.suite,
            output_dir=args.output_dir,
            steps=args.steps,
            max_scenarios=args.max_scenarios,
            target_speed=args.target_speed,
        )
    )
    results = runner.run(args.controller)
    artifacts = write_mil_outputs(results, args.output_dir)

    print(f"Completed {len(results)} MIL run(s).")
    print(f"Wrote MIL results JSON to {artifacts['results_json']}")
    print(f"Wrote MIL summary CSV to {artifacts['summary_csv']}")
    print(f"Wrote MIL report to {artifacts['report_md']}")
    for result in results:
        print(
            f"{result.controller} on {result.scenario_id}: "
            f"success={int(bool(result.kpis['success']))}, "
            f"lateral_rmse={float(result.kpis['lateral_rmse']):.4f}, "
            f"fallback_count={int(result.kpis['fallback_count'])}"
        )
    return 0


__all__ = ["main"]


if __name__ == "__main__":
    raise SystemExit(main())
