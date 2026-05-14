# CommonRoad Drivability Report

## Scope

This report records the current real CommonRoad-DC check run over the local seven-scenario public
CommonRoad XML suite. The goal is to verify that the pipeline can load real XML scenarios, extract
a progress-based lanelet/goal route reference, and produce collision, road-boundary, lanelet, and
kinematic evidence.

This is not yet a strong controller-performance result. The current real-scenario MIL runner still
uses a geometric lanelet route with progress tracking and lookahead, not a full route planner,
behavior planner, or obstacle-aware NMPC.

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
  --output-dir artifacts/step1_progress_reference_mil_7
```

## Aggregate Result

| Metric | Value |
|---|---:|
| Run count | 7 |
| Success rate | 0.0000 |
| Collision count | 6 |
| Road-boundary violation count | 32 |
| Constraint violation count | 28 |
| Fallback count | 5 |
| Mean lateral RMSE | 2.0940 m |
| Mean speed RMSE | 1.4529 m/s |
| Max p95 solve time | 5.80 ms |

## Scenario Results

| Scenario | Success | Collisions | Road-boundary violations | Constraint violations | Fallbacks | Lateral RMSE |
|---|---:|---:|---:|---:|---:|---:|
| `USA_US101-1_1_T-1` | 0 | 2 | 0 | 0 | 0 | 0.3111 m |
| `USA_US101-2_1_T-1` | 0 | 0 | 25 | 28 | 5 | 3.8675 m |
| `USA_US101-13_1_T-1` | 0 | 0 | 0 | 0 | 0 | 2.8745 m |
| `USA_Lanker-1_1_T-1` | 0 | 0 | 0 | 0 | 0 | 2.2470 m |
| `USA_Lanker-2_1_T-1` | 0 | 0 | 0 | 0 | 0 | 2.7614 m |
| `USA_Peach-1_1_T-1` | 0 | 0 | 7 | 0 | 0 | 1.7757 m |
| `USA_Peach-3_1_T-1` | 0 | 4 | 0 | 0 | 0 | 0.8210 m |

## What This Proves

The pipeline now performs real CommonRoad XML loading, projects the ego state onto a lanelet/goal
route reference, builds the NMPC horizon from route progress instead of wall-clock time, and
activates CommonRoad-DC checks. The row artifacts report `scenario_source=commonroad_reference_path`,
`commonroad_dc_checked=True`, and
`commonroad_lanelet_checked=True`.

This step improved the real-suite behavior materially, but it did not solve the real-scenario gap.
The controller still does not make behavior decisions around traffic, stop before occupied space,
or enforce obstacle constraints inside the NMPC problem.

## Next Engineering Gap

The next technical step is not more reporting. It is to add traffic awareness:

- expose nearby dynamic obstacles to the controller or safety supervisor;
- then add obstacle-aware constraints or a conservative fallback policy.
