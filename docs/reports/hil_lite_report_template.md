# HIL-Lite Report Template

## Scope

This report summarizes HIL-style validation using the project UDP/in-process loop. It is not a
claim of full dSPACE, Speedgoat, or production ECU HIL.

## Required Evidence

- Controller interface and protocol version
- Loop sample time and number of steps
- Command latency statistics
- Missed-deadline count
- Timeout count
- Fallback activation count and reasons
- Fault injection schedule

## Limitations

The current HIL-lite setup validates timing and communication behavior on a local machine. It does
not replace real-time target execution, calibrated ECU I/O, bus simulation, or certified HIL tools.
