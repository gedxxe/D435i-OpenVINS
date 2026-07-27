#!/usr/bin/env python3
"""Check whether a proposed AprilGrid plus white border fits one page."""

from __future__ import annotations

import argparse
import math


def positive_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return value


def nonnegative_float(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value < 0:
        raise argparse.ArgumentTypeError("must be finite and nonnegative")
    return value


def positive_int(text: str) -> int:
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Calculate physical AprilGrid dimensions before generating a PDF. "
            "This does not create AprilTags and does not replace measurement "
            "of the printed target."
        )
    )
    parser.add_argument("--tag-rows", required=True, type=positive_int)
    parser.add_argument("--tag-cols", required=True, type=positive_int)
    parser.add_argument("--tag-size-mm", required=True, type=positive_float)
    parser.add_argument(
        "--tag-spacing-ratio", required=True, type=positive_float
    )
    parser.add_argument("--paper-width-mm", required=True, type=positive_float)
    parser.add_argument("--paper-height-mm", required=True, type=positive_float)
    parser.add_argument(
        "--printer-margin-mm", required=True, type=nonnegative_float
    )
    args = parser.parse_args()

    gap_mm = args.tag_size_mm * args.tag_spacing_ratio
    grid_width_mm = (
        args.tag_cols * args.tag_size_mm
        + (args.tag_cols - 1) * gap_mm
    )
    grid_height_mm = (
        args.tag_rows * args.tag_size_mm
        + (args.tag_rows - 1) * gap_mm
    )
    # Kalibr recommends a surrounding white border at least as large as one
    # grid element. A grid element is one tag plus its adjacent gap.
    border_mm = args.tag_size_mm + gap_mm
    required_width_mm = grid_width_mm + 2 * border_mm
    required_height_mm = grid_height_mm + 2 * border_mm
    printable_width_mm = args.paper_width_mm - 2 * args.printer_margin_mm
    printable_height_mm = args.paper_height_mm - 2 * args.printer_margin_mm
    if printable_width_mm <= 0 or printable_height_mm <= 0:
        parser.error("printer margins leave no printable page area")

    portrait_fits = (
        required_width_mm <= printable_width_mm
        and required_height_mm <= printable_height_mm
    )
    landscape_fits = (
        required_width_mm <= printable_height_mm
        and required_height_mm <= printable_width_mm
    )
    print(f"tag_gap_mm: {gap_mm:.6f}")
    print(f"active_grid_mm: {grid_width_mm:.6f} x {grid_height_mm:.6f}")
    print(f"minimum_white_border_each_side_mm: {border_mm:.6f}")
    print(
        "required_with_border_mm: "
        f"{required_width_mm:.6f} x {required_height_mm:.6f}"
    )
    print(
        "printable_page_mm: "
        f"{printable_width_mm:.6f} x {printable_height_mm:.6f}"
    )
    print(f"portrait_fit: {'PASS' if portrait_fits else 'FAIL'}")
    print(f"landscape_fit: {'PASS' if landscape_fits else 'FAIL'}")
    print(
        "rosrun kalibr kalibr_create_target_pdf --type apriltag "
        f"--nx {args.tag_cols} --ny {args.tag_rows} "
        f"--tsize {args.tag_size_mm / 1000.0:.9g} "
        f"--tspace {args.tag_spacing_ratio:.9g}"
    )
    if not portrait_fits and not landscape_fits:
        print("fit_result: FAIL")
        return 5
    print("fit_result: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
