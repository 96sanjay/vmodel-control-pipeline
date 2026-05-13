"""HIL-lite protocol, server, client, and timing loop utilities."""

from vcp.hil.controller_server import ControllerServerConfig, HILControllerServer
from vcp.hil.plant_client import HILPlantClient, HILTimeoutError, PlantClientConfig
from vcp.hil.protocol import (
    HILCommand,
    HILMeasurement,
    HILProtocolError,
    decode_command,
    decode_measurement,
    encode_message,
)
from vcp.hil.realtime_loop import (
    FailureInjectionConfig,
    HILLiteLoopConfig,
    HILLiteResult,
    HILLiteStepLog,
    run_hil_lite_loop,
    write_hil_lite_report,
)

__all__ = [
    "ControllerServerConfig",
    "FailureInjectionConfig",
    "HILCommand",
    "HILControllerServer",
    "HILMeasurement",
    "HILPlantClient",
    "HILProtocolError",
    "HILTimeoutError",
    "HILLiteLoopConfig",
    "HILLiteResult",
    "HILLiteStepLog",
    "PlantClientConfig",
    "decode_command",
    "decode_measurement",
    "encode_message",
    "run_hil_lite_loop",
    "write_hil_lite_report",
]
