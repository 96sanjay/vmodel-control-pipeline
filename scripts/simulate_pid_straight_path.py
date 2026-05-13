from __future__ import annotations

import argparse
import csv
from pathlib import Path

from vcp.controllers import PathTrackingTarget, create_default_vehicle_pid
from vcp.models import KinematicBicycleModel, VehicleState


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a PID baseline on a straight reference path.")
    parser.add_argument("--dt", type=float, default=0.1, help="Simulation step time in seconds.")
    parser.add_argument("--steps", type=int, default=120, help="Number of simulation steps.")
    parser.add_argument("--target-speed", type=float, default=5.0, help="Target speed in m/s.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts/pid_straight_path"),
        help="Directory for CSV and SVG output artifacts.",
    )
    args = parser.parse_args()

    if args.dt <= 0.0:
        raise ValueError("--dt must be positive")
    if args.steps <= 0:
        raise ValueError("--steps must be positive")

    rows = run_simulation(dt=args.dt, steps=args.steps, target_speed=args.target_speed)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    csv_path = args.output_dir / "pid_straight_path.csv"
    speed_plot_path = args.output_dir / "speed_error.svg"
    lateral_plot_path = args.output_dir / "lateral_error.svg"

    _write_csv(rows, csv_path)
    _write_svg_plot(rows, "time_s", "speed_error", speed_plot_path, "PID Speed Error", "m/s")
    _write_svg_plot(rows, "time_s", "lateral_error", lateral_plot_path, "PID Lateral Error", "m")

    print(f"Wrote PID simulation CSV to {csv_path}")
    print(f"Wrote PID speed-error plot to {speed_plot_path}")
    print(f"Wrote PID lateral-error plot to {lateral_plot_path}")
    return 0


def run_simulation(dt: float, steps: int, target_speed: float) -> list[dict[str, float | int]]:
    model = KinematicBicycleModel()
    controller = create_default_vehicle_pid()
    target = PathTrackingTarget(speed=target_speed, lateral_position=0.0, heading=0.0)
    state = VehicleState(px=0.0, py=-1.0, yaw=0.0, v=0.0)
    rows: list[dict[str, float | int]] = []

    for step_index in range(steps):
        time_s = step_index * dt
        command, diagnostics = controller.compute_control(state, target, dt)
        rows.append(
            {
                "time_s": time_s,
                "px": state.px,
                "py": state.py,
                "yaw": state.yaw,
                "v": state.v,
                "acceleration_cmd": command.acceleration,
                "steering_cmd": command.steering_angle,
                "speed_error": diagnostics.speed_error,
                "lateral_error": diagnostics.lateral_error,
                "heading_error": diagnostics.heading_error,
                "saturation_count": int(diagnostics.command_saturated),
            }
        )
        state = model.step(state, command, dt)

    return rows


def _write_csv(rows: list[dict[str, float | int]], output_path: Path) -> None:
    if not rows:
        raise ValueError("rows must not be empty")

    with output_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def _write_svg_plot(
    rows: list[dict[str, float | int]],
    x_key: str,
    y_key: str,
    output_path: Path,
    title: str,
    y_unit: str,
) -> None:
    width = 900
    height = 420
    margin_left = 70
    margin_right = 30
    margin_top = 45
    margin_bottom = 55

    points = [(float(row[x_key]), float(row[y_key])) for row in rows]
    x_values = [point[0] for point in points]
    y_values = [point[1] for point in points]
    x_min, x_max = min(x_values), max(x_values)
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
    zero_line: str | None = None
    if y_min <= 0.0 <= y_max:
        zero_y = sy(0.0)
        zero_line = _element(
            "line",
            [
                f'x1="{margin_left}"',
                f'y1="{zero_y:.2f}"',
                f'x2="{width - margin_right}"',
                f'y2="{zero_y:.2f}"',
                'stroke="#9ca3af"',
                'stroke-width="1"',
                'stroke-dasharray="4 4"',
            ],
        )

    svg_lines = [
        _svg_root(width, height),
        _element("rect", ['width="100%"', 'height="100%"', 'fill="#f8fafc"']),
        _text(width / 2, 26, title, size=18, fill="#0f172a"),
        _line(margin_left, height - margin_bottom, width - margin_right, height - margin_bottom),
        _line(margin_left, margin_top, margin_left, height - margin_bottom),
        zero_line,
        _element(
            "polyline",
            [
                f'points="{polyline}"',
                'fill="none"',
                'stroke="#0f766e"',
                'stroke-width="3"',
                'stroke-linejoin="round"',
                'stroke-linecap="round"',
            ],
        ),
        _text(width / 2, height - 16, "time [s]", size=13, fill="#334155"),
        _text(
            18,
            height / 2,
            f"{y_key} [{y_unit}]",
            size=13,
            fill="#334155",
            transform=f"rotate(-90 18 {height / 2:.0f})",
        ),
        _text(margin_left, height - margin_bottom + 22, f"{x_min:.1f}", size=11),
        _text(width - margin_right, height - margin_bottom + 22, f"{x_max:.1f}", size=11),
        _text(margin_left - 10, sy(y_max), f"{y_max:.2f}", anchor="end", size=11),
        _text(margin_left - 10, sy(y_min), f"{y_min:.2f}", anchor="end", size=11),
        "</svg>",
    ]
    svg = "\n".join(line for line in svg_lines if line is not None) + "\n"
    output_path.write_text(svg, encoding="utf-8")


def _svg_root(width: int, height: int) -> str:
    attrs = [
        'xmlns="http://www.w3.org/2000/svg"',
        f'width="{width}"',
        f'height="{height}"',
        f'viewBox="0 0 {width} {height}"',
    ]
    return f"<svg {' '.join(attrs)}>"


def _line(x1: float, y1: float, x2: float, y2: float) -> str:
    return _element(
        "line",
        [
            f'x1="{x1:.2f}"',
            f'y1="{y1:.2f}"',
            f'x2="{x2:.2f}"',
            f'y2="{y2:.2f}"',
            'stroke="#334155"',
            'stroke-width="1.5"',
        ],
    )


def _text(
    x: float,
    y: float,
    body: str,
    *,
    anchor: str = "middle",
    size: int = 13,
    fill: str = "#475569",
    transform: str | None = None,
) -> str:
    attrs = [
        f'x="{x:.2f}"',
        f'y="{y:.2f}"',
        f'text-anchor="{anchor}"',
        'font-family="monospace"',
        f'font-size="{size}"',
        f'fill="{fill}"',
    ]
    if transform is not None:
        attrs.append(f'transform="{transform}"')
    return _element("text", attrs, body)


def _element(tag: str, attrs: list[str], body: str | None = None) -> str:
    attr_text = " ".join(attrs)
    if body is None:
        return f"  <{tag} {attr_text} />"
    return f"  <{tag} {attr_text}>{body}</{tag}>"


def _expand_range(lower: float, upper: float) -> tuple[float, float]:
    if lower == upper:
        padding = abs(lower) * 0.1 if lower else 1.0
        return lower - padding, upper + padding

    padding = (upper - lower) * 0.1
    return lower - padding, upper + padding


if __name__ == "__main__":
    raise SystemExit(main())
