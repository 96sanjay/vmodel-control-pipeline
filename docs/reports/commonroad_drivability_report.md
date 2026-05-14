# CommonRoad Drivability Report

## Scope

This report records the current real CommonRoad-DC check run over the local seven-scenario public
CommonRoad XML suite. The goal is to verify that the pipeline can load real XML scenarios, extract
a lanelet/goal-based reference path, and produce collision, road-boundary, lanelet, and kinematic
evidence.

This is not yet a strong controller-performance result. The current real-scenario MIL runner still
uses a simple lanelet-centerline reference, not a full route planner, behavior planner, or
obstacle-aware NMPC.

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
  --output-dir artifacts/real_commonroad_reference_mil_7
```

## Aggregate Result

| Metric | Value |
|---|---:|
| Run count | 7 |
| Success rate | 0.0000 |
| Collision count | 20 |
| Road-boundary violation count | 70 |
| Constraint violation count | 28 |
| Fallback count | 5 |
| Mean lateral RMSE | 2.7057 m |
| Mean speed RMSE | 1.7533 m/s |
| Max p95 solve time | 6.76 ms |

## Scenario Results

| Scenario | Success | Collisions | Road-boundary violations | Constraint violations | Fallbacks | Lateral RMSE |
|---|---:|---:|---:|---:|---:|---:|
| `USA_US101-1_1_T-1` | 0 | 0 | 1 | 0 | 0 | 0.1046 m |
| `USA_US101-2_1_T-1` | 0 | 0 | 0 | 28 | 5 | 0.8785 m |
| `USA_US101-13_1_T-1` | 0 | 10 | 20 | 0 | 0 | 4.3918 m |
| `USA_Lanker-1_1_T-1` | 0 | 6 | 17 | 0 | 0 | 6.1272 m |
| `USA_Lanker-2_1_T-1` | 0 | 0 | 9 | 0 | 0 | 2.7256 m |
| `USA_Peach-1_1_T-1` | 0 | 0 | 13 | 0 | 0 | 1.6438 m |
| `USA_Peach-3_1_T-1` | 0 | 4 | 10 | 0 | 0 | 3.0683 m |

## What This Proves

The pipeline now performs real CommonRoad XML loading, extracts a first-pass reference from the
lanelet network and planning goal, and activates CommonRoad-DC checks. The row artifacts report
`scenario_source=commonroad_reference_path`, `commonroad_dc_checked=True`, and
`commonroad_lanelet_checked=True`.

The current controller behavior is not sufficient for most real scenarios. This is expected at this
stage because the reference path is only a geometric centerline/lane-change guide. It does not yet
make behavior decisions around traffic, stop before occupied space, or enforce obstacle constraints
inside the NMPC problem.

## Next Engineering Gap

The next technical step is not more reporting. It is to make real scenario interpretation stronger:

- turn the lanelet/goal reference into a proper route with progress tracking and lookahead;
- expose nearby dynamic obstacles to the controller or safety supervisor;
- then add obstacle-aware constraints or a conservative fallback policy.
