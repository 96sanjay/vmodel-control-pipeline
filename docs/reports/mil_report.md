# MIL Report

## Scope

This report summarizes the Phase 14 Model-in-the-Loop smoke validation run. Controllers are tested
against the current CommonRoad scenario manifest. Because raw CommonRoad XML files are not included
in the repository, the runner used synthetic smoke references derived from the manifest and labeled
the source accordingly.

## Run Command

```bash
python -m vcp.validation.run_mil \
  --suite configs/commonroad/scenario_suite.yaml \
  --controller all \
  --max-scenarios 0 \
  --steps 120 \
  --output-dir artifacts/phase14_mil_all
```

## Aggregate Results

| Controller | Success rate | Collision count | Road-boundary violations | Mean lateral RMSE | Mean speed RMSE | Max p95 solve time | Fallback count |
|---|---:|---:|---:|---:|---:|---:|---:|
| PID | 0.80 | 0 | 0 | 0.3211 m | 0.9891 m/s | 0.02 ms | 0 |
| LQR | 0.80 | 0 | 0 | 0.3177 m | 0.9845 m/s | 0.01 ms | 0 |
| Linear MPC | 0.80 | 0 | 0 | 0.3027 m | 0.9623 m/s | 62.73 ms | 0 |
| NMPC | 1.00 | 0 | 0 | 0.2285 m | 0.9601 m/s | 2.66 ms | 0 |

## Scenario-Level Tracking Results

| Scenario | Controller | Source | Success | Lateral RMSE | Fallbacks |
|---|---|---|---:|---:|---:|
| DEU_Aachen-2_1_T-1 | PID | synthetic smoke | 1 | 0.3632 m | 0 |
| DEU_Aachen-2_1_T-1 | LQR | synthetic smoke | 1 | 0.3467 m | 0 |
| DEU_Aachen-2_1_T-1 | Linear MPC | synthetic smoke | 1 | 0.3448 m | 0 |
| DEU_Aachen-2_1_T-1 | NMPC | synthetic smoke | 1 | 0.3475 m | 0 |
| DEU_Muc-3_1_T-1 | PID | synthetic smoke | 1 | 0.3399 m | 0 |
| DEU_Muc-3_1_T-1 | LQR | synthetic smoke | 1 | 0.3160 m | 0 |
| DEU_Muc-3_1_T-1 | Linear MPC | synthetic smoke | 1 | 0.3118 m | 0 |
| DEU_Muc-3_1_T-1 | NMPC | synthetic smoke | 1 | 0.3148 m | 0 |
| DEU_Gar-1_1_T-1 | PID | synthetic smoke | 0 | 0.5982 m | 0 |
| DEU_Gar-1_1_T-1 | LQR | synthetic smoke | 0 | 0.6466 m | 0 |
| DEU_Gar-1_1_T-1 | Linear MPC | synthetic smoke | 0 | 0.5906 m | 0 |
| DEU_Gar-1_1_T-1 | NMPC | synthetic smoke | 1 | 0.2752 m | 0 |
| DEU_Ffb-1_1_T-1 | PID | synthetic smoke | 1 | 0.1018 m | 0 |
| DEU_Ffb-1_1_T-1 | LQR | synthetic smoke | 1 | 0.0985 m | 0 |
| DEU_Ffb-1_1_T-1 | Linear MPC | synthetic smoke | 1 | 0.0864 m | 0 |
| DEU_Ffb-1_1_T-1 | NMPC | synthetic smoke | 1 | 0.0183 m | 0 |
| USA_US101-13_1_T-1 | PID | synthetic smoke | 1 | 0.2023 m | 0 |
| USA_US101-13_1_T-1 | LQR | synthetic smoke | 1 | 0.1808 m | 0 |
| USA_US101-13_1_T-1 | Linear MPC | synthetic smoke | 1 | 0.1797 m | 0 |
| USA_US101-13_1_T-1 | NMPC | synthetic smoke | 1 | 0.1870 m | 0 |

## Assessment

NMPC is currently the strongest controller on the synthetic smoke references. The clearest example
is the turn-like `DEU_Gar-1_1_T-1` case, where PID, LQR, and linear MPC fail the current final-error
success criterion while NMPC passes.

## Limitations

The collision counts are not full CommonRoad obstacle-validation evidence yet. They are placeholders
within the smoke KPI structure until the closed-loop runner integrates real CommonRoad obstacle and
drivability checks.
