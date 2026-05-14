from __future__ import annotations

import argparse
from pathlib import Path

from vcp.logging import read_signal_log_csv
from vcp.logging.virtual_can import rows_to_controller_status_frames, write_can_frames_jsonl


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay a controller signal CSV as virtual CAN.")
    parser.add_argument("log_csv", type=Path, help="CSV signal log to encode.")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/virtual_can/controller_status_frames.jsonl"),
        help="JSONL output path for encoded CAN frames.",
    )
    args = parser.parse_args()

    rows = read_signal_log_csv(args.log_csv)
    if not rows:
        raise ValueError(f"No rows found in {args.log_csv}")

    frames = rows_to_controller_status_frames(rows)
    write_can_frames_jsonl(frames, args.output)
    print(f"Wrote {len(frames)} virtual CAN frame(s) to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
