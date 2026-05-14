# CommonRoad Drivability Report

## Scope

This report records the first real CommonRoad-DC check run over the local seven-scenario public
CommonRoad XML suite. The goal is to verify that the pipeline can load real XML scenarios and
produce collision, road-boundary, lanelet, and kinematic evidence.

This is not yet a strong controller-performance result. The current real-scenario MIL runner still
uses a simple initial-state tracking target, not a full route planner or obstacle-aware NMPC.

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
  --output-dir artifacts/real_commonroad_dc_mil_7
```

## Aggregate Result

| Metric | Value |
|---|---:|
| Run count | 7 |
| Success rate | 0.1429 |
| Collision count | 8 |
| Road-boundary violation count | 76 |
| Constraint violation count | 28 |
| Fallback count | 5 |
| Mean lateral RMSE | 1.5532 m |
| Mean speed RMSE | 0.7480 m/s |
| Max p95 solve time | 10.81 ms |

## Scenario Results

| Scenario | Success | Collisions | Road-boundary violations | Constraint violations | Fallbacks | Lateral RMSE |
|---|---:|---:|---:|---:|---:|---:|
| `USA_US101-1_1_T-1` | 1 | 0 | 0 | 0 | 0 | 0.0000 m |
| `USA_US101-2_1_T-1` | 0 | 0 | 21 | 28 | 5 | 3.7021 m |
| `USA_US101-13_1_T-1` | 0 | 6 | 13 | 0 | 0 | 0.7493 m |
| `USA_Lanker-1_1_T-1` | 0 | 0 | 9 | 0 | 0 | 1.8538 m |
| `USA_Lanker-2_1_T-1` | 0 | 0 | 14 | 0 | 0 | 1.7952 m |
| `USA_Peach-1_1_T-1` | 0 | 0 | 19 | 0 | 0 | 2.7721 m |
| `USA_Peach-3_1_T-1` | 0 | 2 | 0 | 0 | 0 | 0.0000 m |

## What This Proves

The pipeline now performs real CommonRoad XML loading and activates CommonRoad-DC checks. The row
artifacts report `commonroad_dc_checked=True` and `commonroad_lanelet_checked=True`.

The current controller behavior is not sufficient for most real scenarios. This is expected at this
stage because the real-scenario runner does not yet compute a lanelet-following route, reference
path, or obstacle-aware trajectory.

## Next Engineering Gap

The next technical step is not more reporting. It is to build real scenario interpretation:

- extract a route or centerline reference from the planning problem and lanelet network;
- follow that reference instead of holding the initial lateral position;
- expose nearby dynamic obstacles to the controller or safety supervisor;
- then add obstacle-aware constraints or a conservative fallback policy.
