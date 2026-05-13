from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from time import perf_counter, sleep

from vcp.controllers import FallbackBrakeController, PathTrackingTarget
from vcp.hil.controller_server import HILControllerServer
from vcp.hil.protocol import HILCommand, HILMeasurement
from vcp.models import KinematicBicycleModel, VehicleState


@dataclass(frozen=True)
class FailureInjectionConfig:
    """Fault injection schedule for HIL-lite validation."""

    drop_request_steps: tuple[int, ...] = ()
    invalid_measurement_steps: tuple[int, ...] = ()
    delayed_request_steps: dict[int, float] = field(default_factory=dict)


@dataclass(frozen=True)
class HILLiteLoopConfig:
    """Configuration for the repeatable in-process HIL-lite loop."""

    dt: float = 0.05
    steps: int = 20
    target_speed: float = 4.0
    command_timeout_s: float = 0.05

    def __post_init__(self) -> None:
        if self.dt <= 0.0:
            raise ValueError("dt must be positive")
        if self.steps <= 0:
            raise ValueError("steps must be positive")
        if self.target_speed <= 0.0:
            raise ValueError("target_speed must be positive")
        if self.command_timeout_s <= 0.0:
            raise ValueError("command_timeout_s must be positive")


@dataclass(frozen=True)
class HILLiteStepLog:
    """One HIL-lite loop timing and command record."""

    sequence_id: int
    time_s: float
    loop_time_ms: float
    command_latency_ms: float
    missed_deadline: bool
    timeout: bool
    fallback_active: bool
    fallback_reason: str
    acceleration_cmd: float
    steering_cmd: float
    px: float
    py: float
    yaw: float
    v: float


@dataclass(frozen=True)
class HILLiteResult:
    """Summary of a HIL-lite loop run."""

    step_count: int
    missed_deadline_count: int
    timeout_count: int
    fallback_count: int
    logs: tuple[HILLiteStepLog, ...]


def run_hil_lite_loop(
    server: HILControllerServer,
    *,
    config: HILLiteLoopConfig | None = None,
    failure_injection: FailureInjectionConfig | None = None,
) -> HILLiteResult:
    """Run a deterministic HIL-lite loop without needing external hardware."""

    config = config or HILLiteLoopConfig()
    failure_injection = failure_injection or FailureInjectionConfig()
    plant = KinematicBicycleModel()
    fallback = FallbackBrakeController()
    state = VehicleState(px=0.0, py=-0.5, yaw=0.0, v=0.0)
    target = PathTrackingTarget(speed=config.target_speed, lateral_position=0.0, heading=0.0)
    logs: list[HILLiteStepLog] = []

    for sequence_id in range(config.steps):
        loop_start = perf_counter()
        time_s = sequence_id * config.dt
        timeout = sequence_id in failure_injection.drop_request_steps
        measurement = _measurement_from_state(
            sequence_id,
            time_s,
            config.dt,
            state,
            target,
            invalid=sequence_id in failure_injection.invalid_measurement_steps,
        )

        delay_s = failure_injection.delayed_request_steps.get(sequence_id, 0.0)
        if delay_s > 0.0:
            sleep(delay_s)

        if timeout:
            command = _timeout_fallback_command(sequence_id, time_s, state, fallback)
            command_latency_ms = config.command_timeout_s * 1000.0
        else:
            command_start = perf_counter()
            command = server.handle_measurement(measurement)
            command_latency_ms = (perf_counter() - command_start) * 1000.0

        applied_command = command.to_vehicle_input()
        loop_time_ms = (perf_counter() - loop_start) * 1000.0
        missed_deadline = loop_time_ms > config.dt * 1000.0
        logs.append(
            HILLiteStepLog(
                sequence_id=sequence_id,
                time_s=time_s,
                loop_time_ms=loop_time_ms,
                command_latency_ms=command_latency_ms,
                missed_deadline=missed_deadline,
                timeout=timeout,
                fallback_active=command.fallback_active,
                fallback_reason=command.fallback_reason,
                acceleration_cmd=applied_command.acceleration,
                steering_cmd=applied_command.steering_angle,
                px=state.px,
                py=state.py,
                yaw=state.yaw,
                v=state.v,
            )
        )
        state = plant.step(state, applied_command, config.dt)

    return HILLiteResult(
        step_count=len(logs),
        missed_deadline_count=sum(int(log.missed_deadline) for log in logs),
        timeout_count=sum(int(log.timeout) for log in logs),
        fallback_count=sum(int(log.fallback_active) for log in logs),
        logs=tuple(logs),
    )


def write_hil_lite_report(result: HILLiteResult, output_dir: Path) -> dict[str, Path]:
    """Write HIL-lite results as JSON and Markdown evidence."""

    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "hil_lite_results.json"
    markdown_path = output_dir / "hil_lite_report.md"
    payload = {
        "step_count": result.step_count,
        "missed_deadline_count": result.missed_deadline_count,
        "timeout_count": result.timeout_count,
        "fallback_count": result.fallback_count,
        "logs": [asdict(log) for log in result.logs],
    }
    json_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    markdown_path.write_text(_markdown_report(result), encoding="utf-8")
    return {"json": json_path, "markdown": markdown_path}


def _measurement_from_state(
    sequence_id: int,
    time_s: float,
    dt: float,
    state: VehicleState,
    target: PathTrackingTarget,
    *,
    invalid: bool,
) -> HILMeasurement:
    return HILMeasurement(
        sequence_id=sequence_id,
        timestamp_s=time_s,
        dt=dt,
        px=state.px,
        py=state.py,
        yaw=state.yaw,
        v=state.v,
        target_speed=target.speed,
        target_lateral_position=target.lateral_position,
        target_heading=target.heading,
        invalid=invalid,
    )


def _timeout_fallback_command(
    sequence_id: int,
    time_s: float,
    state: VehicleState,
    fallback: FallbackBrakeController,
) -> HILCommand:
    command = fallback.compute_control(state)
    return HILCommand(
        sequence_id=sequence_id,
        timestamp_s=time_s,
        acceleration=command.acceleration,
        steering_angle=command.steering_angle,
        controller_mode="EMERGENCY_STOP",
        solver_status="timeout",
        solve_time_ms=0.0,
        fallback_active=True,
        fallback_reason="communication_timeout",
    )


def _markdown_report(result: HILLiteResult) -> str:
    lines = [
        "# HIL-Lite Report",
        "",
        "| Metric | Value |",
        "|---|---:|",
        f"| Steps | {result.step_count} |",
        f"| Missed deadlines | {result.missed_deadline_count} |",
        f"| Timeouts | {result.timeout_count} |",
        f"| Fallback activations | {result.fallback_count} |",
        "",
        "| Step | Loop time ms | Latency ms | Missed deadline | Timeout | Fallback |",
        "|---:|---:|---:|---:|---:|---:|",
    ]
    for log in result.logs:
        lines.append(
            f"| {log.sequence_id} | "
            f"{log.loop_time_ms:.3f} | "
            f"{log.command_latency_ms:.3f} | "
            f"{int(log.missed_deadline)} | "
            f"{int(log.timeout)} | "
            f"{int(log.fallback_active)} |"
        )
    return "\n".join(lines) + "\n"


__all__ = [
    "FailureInjectionConfig",
    "HILLiteLoopConfig",
    "HILLiteResult",
    "HILLiteStepLog",
    "run_hil_lite_loop",
    "write_hil_lite_report",
]
