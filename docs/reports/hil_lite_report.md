# HIL-Lite Report

## Scope

This report summarizes the current HIL-lite validation stage. The goal is to validate timing,
protocol behavior, timeout handling, and fallback activation across a controller boundary. This is
not full dSPACE, Speedgoat, or production ECU HIL.

## Components

| Component | Location |
|---|---|
| UDP-style message protocol | `src/vcp/hil/protocol.py` |
| Controller server | `src/vcp/hil/controller_server.py` |
| Plant client | `src/vcp/hil/plant_client.py` |
| Deterministic timing loop | `src/vcp/hil/realtime_loop.py` |
| Configuration | `configs/hardware/hil_lite.yaml` |

## Phase 14 Smoke Result

Fault injection:

| Fault | Step |
|---|---:|
| Dropped request / timeout | 2 |
| Invalid measurement | 5 |
| Delayed request | 6 |

Measured result:

| Metric | Value |
|---|---:|
| Steps | 8 |
| Missed deadlines | 1 |
| Timeouts | 1 |
| Fallback activations | 2 |

## Assessment

The HIL-lite loop correctly records communication timeout and invalid-measurement fallback behavior.
The delayed request creates a missed deadline, which is counted in the evidence artifact.

## Limitations

This validates local protocol and timing behavior only. It does not include real-time kernel
scheduling, ECU I/O, XCP/CCP measurement, CAN rest-bus simulation, or certified HIL automation.

## Related Virtual CAN Interface

Post-Phase 14 hardening added a virtual CAN replay path for controller status frames:

```bash
python scripts/replay_signal_log_to_virtual_can.py \
  artifacts/example/controller_signals.csv \
  --output artifacts/virtual_can/controller_status_frames.jsonl
```

This supports interface testing before any SocketCAN or hardware bus setup is introduced.
