#!/usr/bin/env python3
"""Analyze an operator-designated stationary interval from an OVRS dataset."""

from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path
from statistics import fmean, stdev


FIELDS = (
    "wx_rad_s",
    "wy_rad_s",
    "wz_rad_s",
    "ax_m_s2",
    "ay_m_s2",
    "az_m_s2",
)


def load_interval(
    path: Path, start: float, duration: float
) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    previous_time: float | None = None
    end = start + duration
    with path.open(newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        required = {"timestamp_s", *FIELDS}
        if set(reader.fieldnames or ()) < required:
            raise ValueError(f"{path} is missing required columns")
        for line_number, raw in enumerate(reader, 2):
            try:
                row = {key: float(raw[key]) for key in required}
            except (TypeError, ValueError) as error:
                raise ValueError(f"{path}:{line_number}: invalid numeric value") from error
            if not all(math.isfinite(value) for value in row.values()):
                raise ValueError(f"{path}:{line_number}: non-finite value")
            timestamp = row["timestamp_s"]
            if previous_time is not None and timestamp <= previous_time:
                raise ValueError(f"{path}:{line_number}: timestamp is not increasing")
            previous_time = timestamp
            if timestamp >= end:
                break
            if start <= timestamp:
                rows.append(row)
    if len(rows) < 3:
        raise ValueError("selected interval contains fewer than three IMU samples")
    return rows


def adjacent_noise_density(values: list[float], mean_dt: float) -> float:
    differences = [
        current - previous for previous, current in zip(values, values[1:])
    ]
    sample_white_noise = stdev(differences) / math.sqrt(2.0)
    return sample_white_noise * math.sqrt(mean_dt)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Measure gravity consistency and short-term white noise in an "
            "operator-designated stationary dataset interval. This does not "
            "estimate bias random walk or certify an IMU calibration."
        )
    )
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--start-s", type=float, required=True)
    parser.add_argument("--duration-s", type=float, required=True)
    gravity_source = parser.add_mutually_exclusive_group(required=True)
    gravity_source.add_argument(
        "--gravity-m-s2",
        type=float,
        help="gravity magnitude used by the estimator or measured locally",
    )
    gravity_source.add_argument(
        "--estimator-config",
        type=Path,
        help="read the single gravity_mag scalar from an estimator YAML",
    )
    parser.add_argument(
        "--max-gravity-error-m-s2",
        type=float,
        help="optional operator-supplied pass/fail bound",
    )
    args = parser.parse_args()
    if args.estimator_config:
        if not args.estimator_config.is_file():
            parser.error(
                "--estimator-config is not a readable file: "
                f"{args.estimator_config}"
            )
        try:
            text = args.estimator_config.read_text(encoding="utf-8")
        except OSError as error:
            parser.error(
                f"cannot read --estimator-config {args.estimator_config}: "
                f"{error}"
            )
        matches = re.findall(
            r"(?m)^[ \t]*gravity_mag:[ \t]*"
            r"([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)"
            r"(?:[eE][+-]?[0-9]+)?)[ \t]*(?:#.*)?$",
            text,
        )
        if len(matches) != 1:
            parser.error(
                "--estimator-config must contain exactly one numeric gravity_mag"
            )
        args.gravity_m_s2 = float(matches[0])
    for name in ("start_s", "duration_s", "gravity_m_s2"):
        value = getattr(args, name)
        if not math.isfinite(value) or (name != "start_s" and value <= 0):
            parser.error(f"--{name.replace('_', '-')} is invalid")
    if args.start_s < 0:
        parser.error("--start-s must be nonnegative")
    if args.max_gravity_error_m_s2 is not None and (
        not math.isfinite(args.max_gravity_error_m_s2)
        or args.max_gravity_error_m_s2 < 0
    ):
        parser.error("--max-gravity-error-m-s2 must be finite and nonnegative")

    path = args.dataset / "imu" / "synchronized.csv"
    if not args.dataset.is_dir():
        parser.error(f"dataset directory does not exist: {args.dataset}")
    if not path.is_file():
        parser.error(f"dataset is missing synchronized IMU data: {path}")
    try:
        rows = load_interval(path, args.start_s, args.duration_s)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    timestamps = [row["timestamp_s"] for row in rows]
    intervals = [
        current - previous
        for previous, current in zip(timestamps, timestamps[1:])
    ]
    mean_dt = fmean(intervals)
    means = {field: fmean(row[field] for row in rows) for field in FIELDS}
    deviations = {
        field: stdev(row[field] for row in rows) for field in FIELDS
    }
    densities = {
        field: adjacent_noise_density(
            [row[field] for row in rows], mean_dt
        )
        for field in FIELDS
    }
    acceleration_mean_norm = math.sqrt(
        sum(means[field] ** 2 for field in FIELDS[3:])
    )
    gyro_mean_norm = math.sqrt(
        sum(means[field] ** 2 for field in FIELDS[:3])
    )
    gravity_error = abs(acceleration_mean_norm - args.gravity_m_s2)

    print(f"samples: {len(rows)}")
    print(f"mean_rate_hz: {1.0 / mean_dt:.9f}")
    print(f"acceleration_mean_norm_m_s2: {acceleration_mean_norm:.9f}")
    print(f"configured_gravity_m_s2: {args.gravity_m_s2:.9f}")
    print(f"gravity_magnitude_error_m_s2: {gravity_error:.9f}")
    print(f"gyroscope_mean_norm_rad_s: {gyro_mean_norm:.9f}")
    for field in FIELDS:
        unit = "rad/s/sqrt(Hz)" if field.startswith("w") else "m/s^2/sqrt(Hz)"
        print(f"{field}_mean: {means[field]:.12g}")
        print(f"{field}_sample_std: {deviations[field]:.12g}")
        print(f"{field}_short_term_noise_density_{unit}: {densities[field]:.12g}")
    print(
        "random_walk_status: NOT_ESTIMATED "
        "(requires a long stationary Allan-deviation recording)"
    )

    if args.max_gravity_error_m_s2 is None:
        print("validation: NOT_REQUESTED")
        return 0
    if gravity_error > args.max_gravity_error_m_s2:
        print("validation: FAIL")
        return 5
    print("validation: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
