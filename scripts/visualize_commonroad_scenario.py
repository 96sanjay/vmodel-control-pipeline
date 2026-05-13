from __future__ import annotations

import argparse
from pathlib import Path

from vcp.benchmarks.commonroad_loader import (
    CommonRoadLoaderError,
    CommonRoadScenarioLoader,
)
from vcp.benchmarks.scenario_manifest import load_scenario_suite


def main() -> int:
    parser = argparse.ArgumentParser(description="Visualize a CommonRoad scenario.")
    parser.add_argument(
        "scenario_id",
        help="CommonRoad scenario ID, for example DEU_Aachen-2_1_T-1",
    )
    parser.add_argument(
        "--suite",
        type=Path,
        default=Path("configs/commonroad/scenario_suite.yaml"),
        help="Scenario suite YAML file.",
    )
    parser.add_argument(
        "--scenario-root",
        type=Path,
        default=None,
        help="Override the suite scenario_root.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Optional output image path. If omitted, an interactive window is shown.",
    )
    args = parser.parse_args()

    suite = load_scenario_suite(args.suite)
    scenario_root = args.scenario_root if args.scenario_root is not None else suite.scenario_root
    loader = CommonRoadScenarioLoader(scenario_root=scenario_root)

    try:
        scenario_data = loader.load_scenario(args.scenario_id)
        _render_scenario(scenario_data.scenario, args.output)
    except CommonRoadLoaderError as exc:
        print(exc)
        return 2

    return 0


def _render_scenario(scenario: object, output_path: Path | None) -> None:
    try:
        import matplotlib.pyplot as plt
        from commonroad.visualization.mp_renderer import MPRenderer
    except ImportError as exc:
        raise CommonRoadLoaderError(
            "Visualization requires matplotlib and commonroad-io. Install the optional benchmark "
            "dependencies before rendering scenarios."
        ) from exc

    renderer = MPRenderer()
    scenario.draw(renderer)
    renderer.render()

    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(output_path, bbox_inches="tight")
        print(f"Saved CommonRoad scenario plot to {output_path}")
    else:
        plt.show()


if __name__ == "__main__":
    raise SystemExit(main())
