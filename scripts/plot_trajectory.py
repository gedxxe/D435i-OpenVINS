#!/usr/bin/env python3
"""Plot and summarize an OVRS TUM-syntax trajectory."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path


def load(path: Path) -> list[tuple[float, float, float, float]]:
    rows: list[tuple[float, float, float, float]] = []
    for number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 8:
            raise ValueError(f"{path}:{number}: expected 8 columns")
        values = [float(value) for value in fields]
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{path}:{number}: non-finite value")
        if rows and values[0] <= rows[-1][0]:
            raise ValueError(f"{path}:{number}: timestamp is not increasing")
        rows.append((values[0], values[1], values[2], values[3]))
    if not rows:
        raise ValueError("trajectory has no states")
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Summarize and optionally plot an OVRS trajectory"
    )
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--save", type=Path, help="save PNG instead of showing")
    parser.add_argument(
        "--backend",
        help="Matplotlib GUI backend override (for example TkAgg)",
    )
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="print metrics without importing Matplotlib",
    )
    parser.add_argument(
        "--max-path-length-m",
        type=float,
        help="fail if path length exceeds this operator-supplied test bound",
    )
    parser.add_argument(
        "--max-displacement-m",
        type=float,
        help="fail if displacement exceeds this operator-supplied test bound",
    )
    args = parser.parse_args()
    for name in ("max_path_length_m", "max_displacement_m"):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or value < 0):
            parser.error(f"--{name.replace('_', '-')} must be finite and nonnegative")
    rows = load(args.trajectory)
    duration = rows[-1][0] - rows[0][0]
    path_length = sum(
        math.dist(previous[1:], current[1:])
        for previous, current in zip(rows, rows[1:])
    )
    displacement = math.dist(rows[0][1:], rows[-1][1:])
    print(f"states: {len(rows)}")
    print(f"duration_s: {duration:.6f}")
    print(f"path_length_m: {path_length:.6f}")
    print(f"final_displacement_m: {displacement:.6f}")
    failed_limits: list[str] = []
    if (
        args.max_path_length_m is not None
        and path_length > args.max_path_length_m
    ):
        failed_limits.append(
            f"path length {path_length:.6f} > {args.max_path_length_m:.6f} m"
        )
    if (
        args.max_displacement_m is not None
        and displacement > args.max_displacement_m
    ):
        failed_limits.append(
            f"displacement {displacement:.6f} > "
            f"{args.max_displacement_m:.6f} m"
        )
    if failed_limits:
        print("validation: FAIL")
        for failure in failed_limits:
            print(f"  {failure}")
    elif (
        args.max_path_length_m is not None
        or args.max_displacement_m is not None
    ):
        print("validation: PASS")

    if args.summary_only:
        if args.save:
            parser.error("--save cannot be combined with --summary-only")
        return 5 if failed_limits else 0

    try:
        import matplotlib
    except ModuleNotFoundError as error:
        if error.name != "matplotlib":
            raise
        parser.error(
            "Matplotlib is optional; create .venv and install requirements.txt, "
            "or use --summary-only"
        )

    if args.save:
        if args.backend:
            parser.error("--backend cannot be combined with --save")
        if args.save.exists():
            parser.error(f"refusing to overwrite existing plot: {args.save}")
        matplotlib.use("Agg", force=True)
    else:
        if os.name != "nt" and not (
            os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
        ):
            parser.error(
                "no graphical display is available; run from the Ubuntu "
                "desktop, or use --save OUTPUT.png"
            )
        requested_backend = args.backend or "TkAgg"
        try:
            matplotlib.use(requested_backend, force=True)
        except (ImportError, ModuleNotFoundError) as error:
            parser.error(
                f"cannot load interactive backend {requested_backend}: "
                f"{error}. Install Ubuntu package python3-tk for TkAgg, "
                "or use --save OUTPUT.png"
            )

    try:
        import matplotlib.pyplot as plt
    except (ImportError, ModuleNotFoundError) as error:
        if args.save:
            raise
        parser.error(
            f"interactive Matplotlib backend failed to initialize: {error}. "
            "Install Ubuntu package python3-tk, or use --save OUTPUT.png"
        )

    x = [row[1] for row in rows]
    y = [row[2] for row in rows]
    z = [row[3] for row in rows]
    figure = plt.figure()
    axis = figure.add_subplot(111, projection="3d")
    axis.plot(x, y, z)
    axis.scatter([x[0]], [y[0]], [z[0]], label="start")
    axis.scatter([x[-1]], [y[-1]], [z[-1]], label="end")
    axis.set_xlabel("global X [m]")
    axis.set_ylabel("global Y [m]")
    axis.set_zlabel("global Z [m]")
    centres = (
        0.5 * (min(x) + max(x)),
        0.5 * (min(y) + max(y)),
        0.5 * (min(z) + max(z)),
    )
    maximum_span = max(
        max(x) - min(x),
        max(y) - min(y),
        max(z) - min(z),
        1e-9,
    )
    half_span = 0.5 * maximum_span
    axis.set_xlim(centres[0] - half_span, centres[0] + half_span)
    axis.set_ylim(centres[1] - half_span, centres[1] + half_span)
    axis.set_zlim(centres[2] - half_span, centres[2] + half_span)
    axis.set_box_aspect((1.0, 1.0, 1.0))
    axis.set_title(
        f"path={path_length:.3f} m, displacement={displacement:.3f} m"
    )
    axis.legend()
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.save, dpi=160, bbox_inches="tight")
        print(f"saved_plot: {args.save}")
    else:
        plt.show(block=True)
    return 5 if failed_limits else 0


if __name__ == "__main__":
    raise SystemExit(main())
