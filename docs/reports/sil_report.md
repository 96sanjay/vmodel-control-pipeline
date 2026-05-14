# SIL Report

## Scope

This report summarizes the current Software-in-the-Loop-style validation layer. The implemented SIL
evidence is a stable controller interface plus back-to-back equivalence testing. It does not yet
execute generated C code.

## Interface Under Test

Controllers are adapted through:

```text
initialize(metadata)
step(ControllerStepInput) -> ControllerStepOutput
reset()
get_diagnostics()
```

The same interface is reused by MIL, SIL-style equivalence, and HIL-lite components.

## Phase 14 Equivalence Result

Controller: `linear_mpc`

| Metric | Value |
|---|---:|
| Result | PASS |
| Samples | 4 |
| Acceleration tolerance | 1e-7 |
| Steering tolerance | 1e-7 |
| Predicted-state tolerance | 1e-6 |
| Max acceleration error | 0.0 |
| Max steering error | 0.0 |
| Max predicted-state error | 0.0 |

## Current Limitation

The `CompiledControllerAdapter` intentionally raises a clear unavailable-artifact error unless a
compiled controller artifact is configured. This keeps the project honest: the current SIL stage is
adapter/equivalence validation, not generated-code execution.

## Next SIL Improvements

- Generate or package an acados C controller artifact.
- Run the same input sequence through Python and compiled adapters.
- Archive generated-code metadata and equivalence reports with git commit and configuration ID.
