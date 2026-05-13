from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import mean
from typing import Any

from vcp.logging import read_signal_log_csv


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay a CSV controller signal log.")
    parser.add_argument("log_csv", type=Path, help="CSV signal log to replay.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts/signal_log_replay"),
        help="Directory for replay summary and SVG plots.",
    )
    parser.add_argument(
        "--signal",
        action="append",
        default=None,
        help="Signal to plot against time. Repeat to plot multiple signals.",
    )
    args = parser.parse_args()

    rows = read_signal_log_csv(args.log_csv)
    if not rows:
        raise ValueError(f"No rows found in {args.log_csv}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    signals = args.signal or ["lateral_error", "heading_error", "solve_time_ms"]
    summary = summarize_rows(rows, signals)
    summary_path = args.output_dir / "signal_log_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")

    for signal in signals:
        if signal not in rows[0]:
            continue
        plot_path = args.output_dir / f"{signal}.svg"
        write_svg_plot(rows, "time_s", signal, plot_path, f"{signal} replay")

    print(f"Wrote replay summary to {summary_path}")
    return 0


def summarize_rows(rows: list[dict[str, Any]], signals: list[str]) -> dict[str, Any]:
    """Build compact numeric statistics for selected signals."""

    summary: dict[str, Any] = {
        "row_count": len(rows),
        "time_start_s": rows[0].get("time_s"),
        "time_end_s": rows[-1].get("time_s"),
        "signals": {},
    }
    for signal in signals:
        values = _numeric_values(rows, signal)
        if not values:
            continue
        summary["signals"][signal] = {
            "min": min(values),
            "max": max(values),
            "mean": mean(values),
        }
    return summary


def write_svg_plot(
    rows: list[dict[str, Any]],
    x_key: str,
    y_key: str,
    output_path: Path,
    title: str,
) -> None:
    """Write a small dependency-free SVG line plot."""

    points = [(float(row[x_key]), float(row[y_key])) for row in rows if _is_numeric(row.get(y_key))]
    if not points:
        return

    width = 900
    height = 420
    margin_left = 70
    margin_right = 30
    margin_top = 45
    margin_bottom = 55
    x_values = [point[0] for point in points]
    y_values = [point[1] for point in points]
    x_min, x_max = _expand_range(min(x_values), max(x_values))
    y_min, y_max = _expand_range(min(y_values), max(y_values))

    def sx(value: float) -> float:
        return margin_left + (value - x_min) / (x_max - x_min) * (
            width - margin_left - margin_right
        )

    def sy(value: float) -> float:
        return height - margin_bottom - (value - y_min) / (y_max - y_min) * (
            height - margin_top - margin_bottom
        )

    polyline = " ".join(f"{sx(x):.2f},{sy(y):.2f}" for x, y in points)
    svg = "\n".join(
        [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
            f'viewBox="0 0 {width} {height}">',
            '  <rect width="100%" height="100%" fill="#f8fafc" />',
            f'  <text x="{width / 2:.2f}" y="26" text-anchor="middle" '
            'font-family="monospace" font-size="18" fill="#0f172a">'
            f"{title}</text>",
            _svg_line(
                margin_left,
                height - margin_bottom,
                width - margin_right,
                height - margin_bottom,
            ),
            _svg_line(margin_left, margin_top, margin_left, height - margin_bottom),
            f'  <polyline points="{polyline}" fill="none" stroke="#2563eb" '
            'stroke-width="3" stroke-linejoin="round" stroke-linecap="round" />',
            f'  <text x="{width / 2:.2f}" y="{height - 16}" text-anchor="middle" '
            'font-family="monospace" font-size="13" fill="#334155">time [s]</text>',
            f'  <text x="18" y="{height / 2:.2f}" text-anchor="middle" '
            'font-family="monospace" font-size="13" fill="#334155" '
            f'transform="rotate(-90 18 {height / 2:.0f})">{y_key}</text>',
            "</svg>",
        ]
    )
    output_path.write_text(svg + "\n", encoding="utf-8")


def _numeric_values(rows: list[dict[str, Any]], key: str) -> list[float]:
    return [float(row[key]) for row in rows if _is_numeric(row.get(key))]


def _is_numeric(value: Any) -> bool:
    if isinstance(value, bool) or value is None:
        return False
    try:
        float(value)
    except (TypeError, ValueError):
        return False
    return True


def _svg_line(x1: float, y1: float, x2: float, y2: float) -> str:
    return (
        f'  <line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
        'stroke="#334155" stroke-width="1.5" />'
    )


def _expand_range(lower: float, upper: float) -> tuple[float, float]:
    if lower == upper:
        padding = abs(lower) * 0.1 if lower else 1.0
        return lower - padding, upper + padding
    padding = (upper - lower) * 0.1
    return lower - padding, upper + padding


if __name__ == "__main__":
    raise SystemExit(main())
