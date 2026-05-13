# SIL Equivalence Report Template

## Scope

This report summarizes Software-in-the-Loop-style back-to-back equivalence between a MIL controller
adapter and a SIL controller adapter.

## Required Evidence

- Controller name and version metadata
- Input sequence description
- Acceleration, steering, and predicted-state tolerances
- Per-step acceleration and steering differences
- Per-step predicted-state difference when the controller exposes predictions
- PASS/FAIL result and maximum observed differences

## Current Limitation

The current implementation uses Python adapters for SIL-style interface validation. Native compiled
controller execution is optional and skipped until an acados-generated or otherwise packaged
controller artifact exists.
