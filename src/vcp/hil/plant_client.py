from __future__ import annotations

import socket
from dataclasses import dataclass

from vcp.hil.protocol import (
    HILCommand,
    HILMeasurement,
    HILProtocolError,
    decode_command,
    encode_message,
)


class HILTimeoutError(TimeoutError):
    """Raised when the plant client does not receive a command in time."""


@dataclass(frozen=True)
class PlantClientConfig:
    """UDP plant-client settings for HIL-lite validation."""

    server_host: str = "127.0.0.1"
    server_port: int = 49000
    timeout_s: float = 0.1
    max_packet_bytes: int = 8192

    def __post_init__(self) -> None:
        if self.server_port <= 0:
            raise ValueError("server_port must be positive")
        if self.timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        if self.max_packet_bytes <= 0:
            raise ValueError("max_packet_bytes must be positive")


class HILPlantClient:
    """UDP plant client that sends measurements and receives commands."""

    def __init__(self, config: PlantClientConfig | None = None) -> None:
        self.config = config or PlantClientConfig()

    def request_command(self, measurement: HILMeasurement) -> HILCommand:
        """Send one measurement and wait for a command datagram."""

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
            udp_socket.settimeout(self.config.timeout_s)
            udp_socket.sendto(
                encode_message("measurement", measurement),
                (self.config.server_host, self.config.server_port),
            )
            try:
                data, _ = udp_socket.recvfrom(self.config.max_packet_bytes)
            except TimeoutError as exc:
                raise HILTimeoutError("timed out waiting for HIL-lite command") from exc

        try:
            return decode_command(data)
        except HILProtocolError as exc:
            raise HILProtocolError("received malformed HIL-lite command") from exc


__all__ = ["HILPlantClient", "HILTimeoutError", "PlantClientConfig"]
