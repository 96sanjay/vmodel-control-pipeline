# Validation Plan

## Purpose

This document defines the V-model-inspired validation plan for the CommonRoad motion-control case
study. The goal is to preserve a clear digital thread from requirements to implementation, tests,
logs, reports, and portfolio evidence.

This project uses industry-inspired practices. It does not claim ISO 26262, Automotive SPICE, ASAM,
FMI, dSPACE, Speedgoat, or MathWorks compliance.

## Validation Scope

The primary case study is NMPC motion control on CommonRoad benchmark scenarios. The secondary
CityLearn and OpenDSS energy-management case study is intentionally out of scope until the
CommonRoad validation path is strong.

The validation thread is:

```text
Requirement -> design module -> controller or estimator code -> test case -> benchmark run
-> log file -> report -> traceability matrix
```

## Stages

| Stage | Intent | Initial Evidence |
| --- | --- | --- |
| Requirements | Define expected behavior and acceptance criteria | YAML requirements, hazard log, traceability matrix |
| MIL | Run controllers against simulated plant and scenarios | KPI JSON, CSV summaries, plots, MIL report |
| SIL | Compare packaged controller behavior against MIL | Back-to-back equivalence JSON and Markdown report |
| HIL-lite | Run controller loop across process or hardware boundary | Timing JSON, latency logs, timeout/fallback report |
| Reporting | Summarize results and limitations | Final report, README, traceability matrix |

## Test Case Register

| Test ID | Requirement Links | Stage | Planned Test Intent |
| --- | --- | --- | --- |
| TC-CR-MIL-001 | REQ-CR-001 | MIL | Detect collision status in a CommonRoad smoke scenario |
| TC-CR-MIL-002 | REQ-CR-002 | MIL | Check road-boundary or drivable-area compliance |
| TC-CR-MIL-003 | REQ-CR-003 | MIL | Verify command and velocity constraints are enforced |
| TC-CR-MIL-004 | REQ-CR-004 | MIL | Inject solver failure and verify fallback activation |
| TC-CR-MIL-005 | REQ-CR-006 | MIL | Compare estimator RMSE against noisy measurements |
| TC-CR-MIL-006 | REQ-CR-007 | MIL | Inject biased measurement and verify residual growth |
| TC-CR-MIL-007 | REQ-CR-008 | MIL | Check required log signals and run metadata |
| TC-CR-SIL-001 | REQ-CR-005 | SIL | Record solve time and deadline-margin statistics |
| TC-CR-SIL-002 | REQ-CR-009 | SIL | Run MIL/SIL back-to-back equivalence test |
| TC-CR-HIL-001 | REQ-CR-010 | HIL-lite | Record loop timing, latency, timeouts, and fallback activations |
| TC-DOC-001 | REQ-CR-011 | Documentation | Verify traceability rows exist for each requirement |
| TC-CI-001 | REQ-CR-012 | CI | Run lint, pytest, and Docker build smoke checks |

## KPI Set

The CommonRoad MIL runner should report these KPIs first:

| KPI | Description |
| --- | --- |
| success_rate | Share of scenarios that meet the smoke success criteria |
| collision_count | Number of detected collisions |
| road_boundary_violation_count | Number of road-boundary or drivable-area violations |
| goal_reached | Whether the ego vehicle reaches the planning goal |
| lateral_rmse | Root-mean-square lateral tracking error |
| heading_rmse | Root-mean-square heading error |
| speed_rmse | Root-mean-square speed error |
| mean_solve_time_ms | Mean controller solve time |
| p95_solve_time_ms | 95th percentile controller solve time |
| max_solve_time_ms | Maximum controller solve time |
| fallback_count | Number of fallback activations |
| constraint_violation_count | Number of command or model constraint violations |

## Evidence Rules

Each benchmark run should capture the git commit, configuration file path, scenario suite name,
controller name, estimator name, sample time, random seed, test case IDs, and generated artifact
paths.

Logs should start as CSV and JSON. MDF/MF4 export through `asammdf` is planned for the industrial
logging phase.

Reports should include measured results and limitations. They should avoid phrases such as
"certified", "production safe", or "ISO 26262 compliant" unless a real certified workflow is used.

## Exit Criteria For Phase 1

Phase 1 is complete when:

| Criterion | Status |
| --- | --- |
| System requirements exist with verification IDs | Complete |
| Software requirements exist with design-element links | Complete |
| Hazard log links hazards to requirements | Complete |
| Traceability matrix maps requirements to tests and evidence | Complete |
| Architecture document explains the validation pipeline | Complete |
| README points to the documentation set | Complete |
