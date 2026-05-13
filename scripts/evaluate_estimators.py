from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from vcp.estimators import ExtendedKalmanFilter
from vcp.models import KinematicBicycleModel, VehicleInput, VehicleState
from vcp.sensors import GaussianStateNoise, StateNoiseConfig


def main() -> int:
    parser = argparse.ArgumentParser(description="Evaluate EKF state-estimation smoke metrics.")
    parser.add_argument("--steps", type=int, default=100, help="Number of simulation steps.")
    parser.add_argument("--dt", type=float, default=0.1, help="Simulation step time in seconds.")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/estimators/estimator_metrics.json"),
        help="Output JSON metrics path.",
    )
    args = parser.parse_args()

    if args.steps <= 0:
        raise ValueError("--steps must be positive")
    if args.dt <= 0.0:
        raise ValueError("--dt must be positive")

    metrics = evaluate_ekf(steps=args.steps, dt=args.dt)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(metrics, indent=2, sort_keys=True), encoding="utf-8")
    print(f"Wrote estimator metrics to {args.output}")
    return 0


def evaluate_ekf(*, steps: int, dt: float) -> dict[str, float]:
    true_states, commands = _simulate_truth(steps=steps, dt=dt)
    noise = GaussianStateNoise(
        StateNoiseConfig(px_std=0.25, py_std=0.25, yaw_std=0.08, v_std=0.20),
        seed=23,
    )
    ekf = ExtendedKalmanFilter(
        initial_state=VehicleState(px=0.2, py=-0.2, yaw=0.05, v=0.0),
        initial_covariance=np.diag([0.5, 0.5, 0.2, 0.5]),
        process_noise=np.diag([0.002, 0.002, 0.001, 0.005]),
        measurement_noise=noise.config.covariance(),
    )

    measurements: list[np.ndarray] = []
    estimates: list[np.ndarray] = []
    residual_norms: list[float] = []
    nis_values: list[float] = []

    for command, true_state in zip(commands, true_states[1:], strict=True):
        measurement = noise.sample(true_state)
        diagnostics = ekf.step(command, measurement, dt=dt)
        measurements.append(measurement)
        estimates.append(np.array(diagnostics.state.as_tuple(), dtype=np.float64))
        residual_norms.append(diagnostics.residual_norm)
        nis_values.append(diagnostics.normalized_innovation_squared)

    truth = np.array([state.as_tuple() for state in true_states[1:]], dtype=np.float64)
    measurement_array = np.array(measurements, dtype=np.float64)
    estimate_array = np.array(estimates, dtype=np.float64)

    return {
        "raw_measurement_rmse": _state_rmse(measurement_array, truth),
        "ekf_estimate_rmse": _state_rmse(estimate_array, truth),
        "mean_residual_norm": float(np.mean(residual_norms)),
        "max_residual_norm": float(np.max(residual_norms)),
        "mean_normalized_innovation_squared": float(np.mean(nis_values)),
    }


def _simulate_truth(*, steps: int, dt: float) -> tuple[list[VehicleState], list[VehicleInput]]:
    model = KinematicBicycleModel()
    state = VehicleState(px=0.0, py=0.0, yaw=0.0, v=2.0)
    states = [state]
    commands: list[VehicleInput] = []

    for step in range(steps):
        command = VehicleInput(
            acceleration=0.2 if step < steps // 3 else 0.0,
            steering_angle=0.08,
        )
        commands.append(command)
        state = model.step(state, command, dt)
        states.append(state)

    return states, commands


def _state_rmse(values: np.ndarray, references: np.ndarray) -> float:
    errors = values - references
    return float(np.sqrt(np.mean(errors * errors)))


if __name__ == "__main__":
    raise SystemExit(main())
