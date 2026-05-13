from __future__ import annotations

import socket
from dataclasses import dataclass
from time import perf_counter

from vcp.controllers import FallbackBrakeController
from vcp.hil.protocol import HILCommand, HILMeasurement, decode_measurement, encode_message
from vcp.models import VehicleState
from vcp.validation import (
    ControllerInterface,
    SafetyEvaluationInput,
    SafetyMode,
    SafetySupervisor,
    SafetySupervisorConfig,
)


@dataclass(frozen=True)
class ControllerServerConfig:
    """UDP server settings for HIL-lite controller execution."""

    host: str = "127.0.0.1"
    port: int = 49000
    max_packet_bytes: int = 8192
    socket_timeout_s: float = 0.1

    def __post_init__(self) -> None:
        if self.port <= 0:
            raise ValueError("port must be positive")
        if self.max_packet_bytes <= 0:
            raise ValueError("max_packet_bytes must be positive")
        if self.socket_timeout_s <= 0.0:
            raise ValueError("socket_timeout_s must be positive")


class HILControllerServer:
    """Expose a ControllerInterface over a small UDP-friendly HIL-lite protocol."""

    def __init__(
        self,
        controller: ControllerInterface,
        *,
        config: ControllerServerConfig | None = None,
        safety_supervisor: SafetySupervisor | None = None,
        fallback_controller: FallbackBrakeController | None = None,
    ) -> None:
        self.controller = controller
        self.config = config or ControllerServerConfig()
        self.safety_supervisor = safety_supervisor or SafetySupervisor(
            SafetySupervisorConfig(max_solve_time_ms=self.config.socket_timeout_s * 1000.0)
        )
        self.fallback_controller = fallback_controller or FallbackBrakeController()
        self.controller.initialize({"stage": "HIL-lite"})

    def handle_measurement(self, measurement: HILMeasurement) -> HILCommand:
        """Handle one measurement packet and return one command packet."""

        start_time = perf_counter()
        if measurement.invalid:
            return self._fallback_command(
                measurement,
                reason="invalid_measurement",
                mode=SafetyMode.EMERGENCY_STOP,
                solve_time_ms=(perf_counter() - start_time) * 1000.0,
            )

        step_input = measurement.to_step_input()
        output = self.controller.step(step_input)
        decision = self.safety_supervisor.evaluate(
            SafetyEvaluationInput(
                timestamp_s=measurement.timestamp_s,
                command=output.command,
                solver_status=_solver_status_for_supervisor(output.solver_status),
                solver_feasible=output.feasible,
                solve_time_ms=output.solve_time_ms,
                estimator_residual=0.0,
                collision_risk=False,
                missing_sensor_message=False,
                communication_timeout=False,
            )
        )
        command = (
            self.fallback_controller.compute_control(step_input.state)
            if decision.fallback_required
            else output.command
        )
        return HILCommand(
            sequence_id=measurement.sequence_id,
            timestamp_s=measurement.timestamp_s,
            acceleration=command.acceleration,
            steering_angle=command.steering_angle,
            controller_mode=decision.mode.value,
            solver_status=output.solver_status,
            solve_time_ms=output.solve_time_ms,
            fallback_active=decision.fallback_required,
            fallback_reason=decision.reason_code,
        )

    def serve_once(self, udp_socket: socket.socket) -> None:
        """Receive and respond to one UDP datagram."""

        data, address = udp_socket.recvfrom(self.config.max_packet_bytes)
        measurement = decode_measurement(data)
        command = self.handle_measurement(measurement)
        udp_socket.sendto(encode_message("command", command), address)

    def serve_forever(self, *, max_messages: int | None = None) -> None:
        """Run a blocking UDP server loop."""

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
            udp_socket.bind((self.config.host, self.config.port))
            udp_socket.settimeout(self.config.socket_timeout_s)
            handled = 0
            while max_messages is None or handled < max_messages:
                self.serve_once(udp_socket)
                handled += 1

    def _fallback_command(
        self,
        measurement: HILMeasurement,
        *,
        reason: str,
        mode: SafetyMode,
        solve_time_ms: float,
    ) -> HILCommand:
        command = self.fallback_controller.compute_control(
            VehicleState(px=measurement.px, py=measurement.py, yaw=measurement.yaw, v=measurement.v)
        )
        return HILCommand(
            sequence_id=measurement.sequence_id,
            timestamp_s=measurement.timestamp_s,
            acceleration=command.acceleration,
            steering_angle=command.steering_angle,
            controller_mode=mode.value,
            solver_status="not_applicable",
            solve_time_ms=solve_time_ms,
            fallback_active=True,
            fallback_reason=reason,
        )


def _solver_status_for_supervisor(solver_status: str) -> str | None:
    return None if solver_status == "not_applicable" else solver_status


__all__ = ["ControllerServerConfig", "HILControllerServer"]
