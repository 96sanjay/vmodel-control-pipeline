# V-Model Validation Pipeline for Optimization-Based Control Algorithms

This repository is the starting point for an industry-inspired validation framework for
optimization-based control algorithms. The primary case study will be NMPC motion control on
CommonRoad benchmark scenarios, with a later secondary case study for CityLearn and OpenDSS energy
management.

The project is intentionally structured around an engineering workflow rather than a single
controller demo:

```text
Requirements -> modeling -> PID/LQR/Kalman baselines -> MPC/NMPC
-> MIL -> SIL -> HIL-lite -> logging -> traceability -> reports
```

## Current Scope

Phase 0 created the production-style repository skeleton. Phase 1 adds the V-model-inspired
requirements, hazard log, traceability matrix, system architecture, and validation plan.

Control algorithms, CommonRoad ingestion, MIL/SIL/HIL-lite runners, and industrial logging will be
added in later phases.

## Repository Layout

```text
docs/                 Requirements, hazards, architecture, reports, and figures
configs/              Scenario, controller, estimator, and hardware configuration
data/                 Raw and processed data manifests; large data should be DVC-tracked
src/vcp/              Python package for the validation control pipeline
tests/                Unit, integration, benchmark, SIL, and HIL-lite tests
scripts/              Developer and workflow helper scripts
docker/               Docker-related support files
.github/workflows/   CI workflow definitions
```

## Quick Start

Create or activate the project-local conda environment:

```bash
conda env create --prefix ./.conda -f environment.yml
conda activate ./.conda
```

Run the starter test suite:

```bash
pytest
```

Build the base container image:

```bash
docker build -t vcp .
```

## Development Commands

```bash
pytest
ruff check src tests
coverage run -m pytest
coverage report
```

## CommonRoad Scenario Data

Phase 2 adds the CommonRoad ingestion layer, but the repository does not vendor raw benchmark
scenarios. Place downloaded CommonRoad XML files under:

```text
data/raw/commonroad/scenarios/
```

The smoke suite manifest is:

```text
configs/commonroad/scenario_suite.yaml
```

Install optional visualization/loading dependencies only when you are ready to use real scenarios:

```bash
pip install -e ".[commonroad]"
python scripts/visualize_commonroad_scenario.py DEU_Aachen-2_1_T-1
```

## Vehicle Model Layer

Phase 3 adds a CommonRoad-independent kinematic bicycle model:

```text
state: [px, py, yaw, v]
input: [acceleration, steering_angle]
```

The model layer includes vehicle parameters, Euler integration, command clipping, velocity/input
constraint checks, and steering-rate checks. Controllers in later phases should depend on this
layer instead of talking directly to CommonRoad.

## PID Baseline

Phase 4 adds the first closed-loop controller baseline:

```bash
python scripts/simulate_pid_straight_path.py
```

The script writes a CSV plus SVG plots for speed error and lateral error under
`artifacts/pid_straight_path/`. These generated artifacts are intentionally ignored by Git.

## LQR Baseline

Phase 5 adds a lateral Linear Quadratic Regulator baseline using a bicycle model linearized around
a nominal velocity. It is a classical optimal-control reference between PID and future MPC/NMPC
controllers.

```bash
python scripts/compare_pid_lqr_straight_path.py
```

The comparison script writes controller trajectories and metrics under
`artifacts/pid_lqr_straight_path/`.

## State Estimation

Phase 6 adds state-estimation baselines:

```bash
python scripts/evaluate_estimators.py
```

The estimator smoke script simulates noisy measurements, runs an EKF on the kinematic bicycle
model, and writes RMSE/residual metrics under `artifacts/estimators/`.

## Linear MPC Baseline

Phase 7 adds the first constrained optimization controller. The linear MPC optimizes acceleration
and steering over a finite horizon while enforcing input and velocity limits.

```bash
python scripts/compare_pid_lqr_mpc_straight_path.py
```

The comparison script writes PID, LQR, and linear MPC trajectories and metrics under
`artifacts/pid_lqr_mpc_straight_path/`.

## NMPC Controller

Phase 8 adds the first nonlinear MPC problem using a CasADi symbolic kinematic bicycle model and
IPOPT as the local nonlinear-program solver. The controller tracks position, heading, and velocity
references while enforcing velocity, acceleration, and steering limits.

The acados-facing module is present as a backend hook, but native acados code generation is not
claimed unless the external acados toolchain is installed and wired later.

## Safety Supervisor

Phase 9 adds deterministic fallback braking and a safety supervisor for solver failure, solver
timeout, estimator residual growth, invalid commands, stale sensor input, and collision-risk flags.
Each transition records a timestamp, reason code, mode, and linked requirement IDs.

## MIL Smoke Validation

Phase 10 adds a CI-friendly Model-in-the-Loop runner. By default it runs one manifest scenario for
25 steps so local execution stays lightweight. Use `--max-scenarios 0` only when you intentionally
want to run every scenario listed in the suite.

```bash
python -m vcp.validation.run_mil \
  --suite configs/commonroad/scenario_suite.yaml \
  --controller nmpc
```

If raw CommonRoad XML files are missing, the runner labels results as synthetic smoke scenarios
derived from the manifest instead of claiming full CommonRoad scenario validation.

Synthetic smoke scenarios now include straight tracking, gentle S-curve tracking, turn-like
reference changes, smooth lane changes, and highway-style gentle curves. Obstacle avoidance is
still future work and should not be claimed until real obstacle constraints are implemented.

## SIL-Style Equivalence

Phase 11 adds a stable `ControllerInterface` with `initialize`, `step`, `reset`, and
`get_diagnostics` operations. Current Python controllers can be wrapped through
`PythonControllerAdapter` and checked back-to-back against another adapter using acceleration,
steering, and predicted-state tolerances.

Compiled controller execution is represented by `CompiledControllerAdapter`, but tests for native
generated artifacts remain optional until an acados-generated C controller exists.

## Project Principles

- Keep requirements, implementation, tests, logs, and reports traceable.
- Use "V-model-inspired", "ISO 26262-inspired", and "HIL-lite" wording unless actual certified
  tools and processes are used.
- Build the CommonRoad control case first before adding the CityLearn/OpenDSS transfer case.
- Prefer reproducible scripts, tests, and reports over notebook-only results.

## Phase 1 Documentation

- [System requirements](docs/requirements/system_requirements.yaml)
- [Software requirements](docs/requirements/software_requirements.yaml)
- [Hazard log](docs/hazards/hazard_log.yaml)
- [Traceability matrix](docs/requirements/traceability_matrix.csv)
- [System architecture](docs/architecture/system_architecture.md)
- [Validation plan](docs/reports/validation_plan.md)
