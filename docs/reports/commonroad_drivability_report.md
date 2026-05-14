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
  --output-dir artifacts/step4_tuned_defaults_mil_7
```

## Aggregate Result

| Metric | Value |
|---|---:|
| Run count | 7 |
| Success rate | 0.0000 |
| Drivable rate | 0.4286 |
| Stable-execution rate | 0.7143 |
| Blocked-by-obstacle count | 3 |
| Collision count | 2 |
| Road-boundary violation count | 44 |
| Constraint violation count | 68 |
| Fallback count | 80 |
| Mean lateral RMSE | 3.3293 m |
| Mean speed RMSE | 2.8944 m/s |
| Max p95 solve time | 9.38 ms |

## Scenario Results

| Scenario | Success | Collisions | Road-boundary violations | Constraint violations | Fallbacks | Lateral RMSE |
|---|---:|---:|---:|---:|---:|---:|
| `USA_US101-1_1_T-1` | 0 | 0 | 0 | 0 | 3 | 0.2917 m |
| `USA_US101-2_1_T-1` | 0 | 0 | 25 | 28 | 5 | 3.8675 m |
| `USA_US101-13_1_T-1` | 0 | 0 | 0 | 0 | 15 | 2.8751 m |
| `USA_Lanker-1_1_T-1` | 0 | 2 | 0 | 14 | 25 | 2.2748 m |
| `USA_Lanker-2_1_T-1` | 0 | 0 | 0 | 0 | 4 | 2.7611 m |
| `USA_Peach-1_1_T-1` | 0 | 0 | 19 | 0 | 12 | 11.0685 m |
| `USA_Peach-3_1_T-1` | 0 | 0 | 0 | 22 | 16 | 0.1733 m |

## What This Proves

The pipeline now performs real CommonRoad XML loading, projects the ego state onto a lanelet/goal
route reference, builds the NMPC horizon from route progress instead of wall-clock time, adds a
nearby-obstacle assessment with TTC-based safety-stop gating, and activates CommonRoad-DC checks.
The KPI layer now distinguishes `success`, `blocked`, `safe_stop`, and `failure` rather than
treating every non-success case the same. The row artifacts report
`scenario_source=commonroad_reference_path`, `commonroad_dc_checked=True`,
`commonroad_lanelet_checked=True`, `obstacle_risk_flag`, `nearby_obstacle_count`, and
`blocking_obstacle_count`.

This step adds real traffic-awareness plumbing and a first tuning pass. The tuned defaults reduce
collisions and classify three scenarios as drivable-but-blocked, but they still do not solve the
real-scenario gap.

## Next Engineering Gap

The next technical step is not more reporting. It is to make the tuned safety signals useful inside
the controller itself:

- use obstacle-aware constraints or a real obstacle cost inside NMPC;
- replace the current route heuristic with a stronger planner or behavior layer for cases like
  `USA_US101-2_1_T-1` and `USA_Peach-1_1_T-1`.
