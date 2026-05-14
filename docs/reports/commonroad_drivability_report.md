# CommonRoad Drivability Report

## Scope

This report records the current real CommonRoad-DC check run over the local seven-scenario public
CommonRoad XML suite. The goal is to verify that the pipeline can load real XML scenarios, extract
a progress-based lanelet/goal route reference, detect nearby obstacles, and produce collision,
road-boundary, lanelet, and kinematic evidence.

This is not yet a strong controller-performance result. The current real-scenario MIL runner still
uses a geometric lanelet route with progress tracking plus a conservative obstacle stop layer, not
a full route planner, behavior planner, or obstacle-aware NMPC.

## Environment

| Item | Value |
|---|---|
| Scenario suite | `configs/commonroad/real_scenario_suite.yaml` |
| Scenario source | Public CommonRoad scenarios GitLab repository |
| Local XML directory | `data/raw/commonroad/scenarios/` |
| `commonroad-io` | `2024.3` |
| `commonroad-drivability-checker` | `2025.4.0` |
| Controller | `nmpc` |
| Steps per scenario | `25` |

## Command

```bash
MPLCONFIGDIR=/tmp/matplotlib PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python -m vcp.validation.run_mil \
  --suite configs/commonroad/real_scenario_suite.yaml \
  --controller nmpc \
  --max-scenarios 0 \
  --steps 25 \
  --output-dir artifacts/step2_obstacle_stop_mil_7_v2
```

## Aggregate Result

| Metric | Value |
|---|---:|
| Run count | 7 |
| Success rate | 0.0000 |
| Collision count | 6 |
| Road-boundary violation count | 44 |
| Constraint violation count | 68 |
| Fallback count | 101 |
| Mean lateral RMSE | 3.2655 m |
| Mean speed RMSE | 3.5621 m/s |
| Max p95 solve time | 6.07 ms |

## Scenario Results

| Scenario | Success | Collisions | Road-boundary violations | Constraint violations | Fallbacks | Lateral RMSE |
|---|---:|---:|---:|---:|---:|---:|
| `USA_US101-1_1_T-1` | 0 | 2 | 0 | 0 | 5 | 0.3100 m |
| `USA_US101-2_1_T-1` | 0 | 0 | 25 | 28 | 5 | 3.8675 m |
| `USA_US101-13_1_T-1` | 0 | 2 | 0 | 0 | 25 | 2.5161 m |
| `USA_Lanker-1_1_T-1` | 0 | 2 | 0 | 14 | 25 | 2.2748 m |
| `USA_Lanker-2_1_T-1` | 0 | 0 | 0 | 0 | 9 | 2.7548 m |
| `USA_Peach-1_1_T-1` | 0 | 0 | 19 | 0 | 14 | 11.0576 m |
| `USA_Peach-3_1_T-1` | 0 | 0 | 0 | 26 | 18 | 0.0774 m |

## What This Proves

The pipeline now performs real CommonRoad XML loading, projects the ego state onto a lanelet/goal
route reference, builds the NMPC horizon from route progress instead of wall-clock time, adds a
nearby-obstacle assessment with TTC-based safety-stop gating, and activates CommonRoad-DC checks.
The row artifacts report `scenario_source=commonroad_reference_path`,
`commonroad_dc_checked=True`, `commonroad_lanelet_checked=True`, `obstacle_risk_flag`,
`nearby_obstacle_count`, and `blocking_obstacle_count`.

This step adds real traffic-awareness plumbing, but it is still a blunt safety layer. The current
thresholds trade tracking quality for conservative fallback behavior, and they do not yet solve the
real-scenario gap.

## Next Engineering Gap

The next technical step is not more reporting. It is to make the new safety signals useful:

- improve the success logic so we can separate safe slowdowns from true failures;
- retune the controller and safety thresholds around the new obstacle signals.
