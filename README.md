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

Phase 0 creates the production-style repository skeleton only. Control algorithms, CommonRoad
ingestion, MIL/SIL/HIL-lite runners, and industrial logging will be added in later phases.

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

## Project Principles

- Keep requirements, implementation, tests, logs, and reports traceable.
- Use "V-model-inspired", "ISO 26262-inspired", and "HIL-lite" wording unless actual certified
  tools and processes are used.
- Build the CommonRoad control case first before adding the CityLearn/OpenDSS transfer case.
- Prefer reproducible scripts, tests, and reports over notebook-only results.
