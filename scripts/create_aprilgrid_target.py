#!/usr/bin/env python3
"""Create a measured Kalibr AprilGrid target YAML without hidden dimensions."""

from __future__ import annotations

import argparse
import sys
from decimal import Decimal, InvalidOperation
from pathlib import Path


def positive_decimal(text: str) -> Decimal:
    try:
        value = Decimal(text)
    except InvalidOperation as exc:
        raise argparse.ArgumentTypeError(
            "must be a positive finite number"
        ) from exc
    if not value.is_finite() or value <= 0:
        raise argparse.ArgumentTypeError("must be a positive finite number")
    return value


def positive_int(text: str) -> int:
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return value


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description=(
            "Create a Kalibr AprilGrid YAML from dimensions measured on the "
            "actual printed target. No board dimensions are assumed."
        )
    )
    result.add_argument("--tag-rows", required=True, type=positive_int)
    result.add_argument("--tag-cols", required=True, type=positive_int)
    size = result.add_mutually_exclusive_group(required=True)
    size.add_argument(
        "--tag-size-m",
        type=positive_decimal,
        help="measured black tag side length in metres",
    )
    size.add_argument(
        "--tag-size-mm",
        type=positive_decimal,
        help="measured black tag side length in millimetres",
    )
    gap = result.add_mutually_exclusive_group(required=True)
    gap.add_argument(
        "--tag-gap-m",
        type=positive_decimal,
        help="measured white gap between adjacent tags in metres",
    )
    gap.add_argument(
        "--tag-gap-mm",
        type=positive_decimal,
        help="measured white gap between adjacent tags in millimetres",
    )
    result.add_argument("--output", required=True, type=Path)
    return result


def main() -> int:
    args = parser().parse_args()
    try:
        if args.output.exists():
            print(f"output already exists: {args.output}", file=sys.stderr)
            return 2
        millimetres_per_metre = Decimal(1000)
        tag_size_m = (
            args.tag_size_m
            if args.tag_size_m is not None
            else args.tag_size_mm / millimetres_per_metre
        )
        tag_gap_m = (
            args.tag_gap_m
            if args.tag_gap_m is not None
            else args.tag_gap_mm / millimetres_per_metre
        )
        spacing = tag_gap_m / tag_size_m
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            "target_type: aprilgrid\n"
            f"tagRows: {args.tag_rows}\n"
            f"tagCols: {args.tag_cols}\n"
            f"tagSize: {tag_size_m.normalize()}\n"
            f"tagSpacing: {spacing.normalize()}\n"
            "# tagSpacing is measured_gap / measured_tag_size.\n",
            encoding="utf-8",
        )
        print(f"AprilGrid target YAML written: {args.output}")
        print(f"Measured gap/size ratio: {spacing.normalize()}")
        return 0
    except OSError as exc:
        print(f"cannot write target YAML: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
