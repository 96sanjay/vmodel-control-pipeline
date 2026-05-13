# MIL Validation Report Template

## Scope

This report summarizes Model-in-the-Loop smoke validation for controllers running against the
current CommonRoad scenario suite manifest. If raw CommonRoad XML files are unavailable, runs must
be labeled as synthetic smoke runs derived from the manifest.

## Required Evidence

- Scenario suite manifest and controller name
- JSON KPI artifact
- CSV summary artifact
- Per-run trajectory, tracking-error, and solver-time plots
- Controller mode and fallback counts
- Clear note on whether real CommonRoad XML files or synthetic smoke scenarios were used

## KPI Table

| Scenario | Controller | Source | Success | Collision count | Road-boundary violations | Lateral RMSE | p95 solve time ms | Fallback count |
|---|---|---|---:|---:|---:|---:|---:|---:|
| TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD | TBD |

## Limitations

This project is V-model-inspired and industry-inspired. Early MIL smoke results do not establish
production safety, roadworthiness, ISO 26262 compliance, or complete CommonRoad benchmark coverage.
