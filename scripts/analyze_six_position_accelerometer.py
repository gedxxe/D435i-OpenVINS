#!/usr/bin/env python3
"""Fit a diagnostic accelerometer affine model from six static poses."""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
from pathlib import Path
from statistics import fmean, stdev

from calibration_common import CalibrationError, validate_capture


AXES = ("x", "y", "z")
POSES = (
    ("x-positive", 0, 1.0),
    ("x-negative", 0, -1.0),
    ("y-positive", 1, 1.0),
    ("y-negative", 1, -1.0),
    ("z-positive", 2, 1.0),
    ("z-negative", 2, -1.0),
)
ACCEL_FIELDS = ("ax_m_s2", "ay_m_s2", "az_m_s2")
GYRO_FIELDS = ("wx_rad_s", "wy_rad_s", "wz_rad_s")


def vector_norm(vector: list[float]) -> float:
    return math.sqrt(sum(value * value for value in vector))


def matrix_vector(
    matrix: list[list[float]], vector: list[float]
) -> list[float]:
    return [
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    ]


def matrix_inverse(matrix: list[list[float]]) -> list[list[float]]:
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]
    determinant = (
        a * (e * i - f * h)
        - b * (d * i - f * g)
        + c * (d * h - e * g)
    )
    if not math.isfinite(determinant) or abs(determinant) < 1e-9:
        raise ValueError("fitted accelerometer matrix is singular")
    inverse = [
        [
            (e * i - f * h) / determinant,
            (c * h - b * i) / determinant,
            (b * f - c * e) / determinant,
        ],
        [
            (f * g - d * i) / determinant,
            (a * i - c * g) / determinant,
            (c * d - a * f) / determinant,
        ],
        [
            (d * h - e * g) / determinant,
            (b * g - a * h) / determinant,
            (a * e - b * d) / determinant,
        ],
    ]
    return inverse


def frobenius_norm(matrix: list[list[float]]) -> float:
    return math.sqrt(sum(value * value for row in matrix for value in row))


def fit_six_position(
    means: dict[str, list[float]], gravity: float
) -> dict[str, object]:
    missing = {pose for pose, _, _ in POSES} - set(means)
    if missing:
        raise ValueError(
            "missing pose means: " + ", ".join(sorted(missing))
        )
    pair_biases: list[list[float]] = []
    columns: list[list[float]] = []
    for axis in AXES:
        positive = means[f"{axis}-positive"]
        negative = means[f"{axis}-negative"]
        pair_biases.append(
            [(positive[index] + negative[index]) * 0.5 for index in range(3)]
        )
        columns.append(
            [
                (positive[index] - negative[index]) / (2.0 * gravity)
                for index in range(3)
            ]
        )
    bias = [
        fmean(pair[index] for pair in pair_biases) for index in range(3)
    ]
    forward = [
        [columns[column][row] for column in range(3)]
        for row in range(3)
    ]
    correction = matrix_inverse(forward)
    condition_estimate = (
        frobenius_norm(forward) * frobenius_norm(correction)
    )

    residuals: dict[str, list[float]] = {}
    corrected: dict[str, list[float]] = {}
    for pose, axis_index, sign in POSES:
        ideal = [0.0, 0.0, 0.0]
        ideal[axis_index] = sign * gravity
        predicted = matrix_vector(forward, ideal)
        predicted = [predicted[index] + bias[index] for index in range(3)]
        residuals[pose] = [
            means[pose][index] - predicted[index] for index in range(3)
        ]
        unbiased = [
            means[pose][index] - bias[index] for index in range(3)
        ]
        corrected[pose] = matrix_vector(correction, unbiased)

    residual_norms = [vector_norm(value) for value in residuals.values()]
    pair_bias_spread = max(
        vector_norm(
            [pair[index] - bias[index] for index in range(3)]
        )
        for pair in pair_biases
    )
    return {
        "bias": bias,
        "pair_biases": pair_biases,
        "pair_bias_spread": pair_bias_spread,
        "forward": forward,
        "correction": correction,
        "condition_estimate": condition_estimate,
        "residuals": residuals,
        "corrected": corrected,
        "residual_rms": math.sqrt(
            fmean(value * value for value in residual_norms)
        ),
        "residual_max": max(residual_norms),
    }


def load_stationary_rows(
    path: Path, trim_start: float, trim_end: float
) -> list[dict[str, float]]:
    parsed: list[dict[str, float]] = []
    required = {"timestamp_s", *ACCEL_FIELDS, *GYRO_FIELDS}
    with path.open(newline="", encoding="utf-8") as source:
        reader = csv.DictReader(source)
        if set(reader.fieldnames or ()) < required:
            raise CalibrationError(f"{path} is missing required columns")
        previous: float | None = None
        for line_number, raw in enumerate(reader, 2):
            try:
                row = {key: float(raw[key]) for key in required}
            except (TypeError, ValueError) as error:
                raise CalibrationError(
                    f"{path}:{line_number}: invalid numeric value"
                ) from error
            if not all(math.isfinite(value) for value in row.values()):
                raise CalibrationError(
                    f"{path}:{line_number}: non-finite value"
                )
            timestamp = row["timestamp_s"]
            if previous is not None and timestamp <= previous:
                raise CalibrationError(
                    f"{path}:{line_number}: timestamp is not increasing"
                )
            previous = timestamp
            parsed.append(row)
    if len(parsed) < 3:
        raise CalibrationError(f"{path} has fewer than three samples")
    start = parsed[0]["timestamp_s"] + trim_start
    end = parsed[-1]["timestamp_s"] - trim_end
    rows = [row for row in parsed if start <= row["timestamp_s"] <= end]
    if len(rows) < 200 or end <= start:
        raise CalibrationError(
            f"{path}: trimmed stationary interval is too short"
        )
    return rows


def summarize_pose(
    rows: list[dict[str, float]],
    axis_index: int,
    sign: float,
    gravity: float,
    max_pose_tilt_deg: float,
    max_gyro_rms: float,
    max_gyro_peak: float,
    max_accel_norm_std: float,
) -> dict[str, object]:
    accel_mean = [
        fmean(row[field] for row in rows) for field in ACCEL_FIELDS
    ]
    gyro_mean = [
        fmean(row[field] for row in rows) for field in GYRO_FIELDS
    ]
    accel_norms = [
        vector_norm([row[field] for field in ACCEL_FIELDS]) for row in rows
    ]
    gyro_norms = [
        vector_norm([row[field] for field in GYRO_FIELDS]) for row in rows
    ]
    accel_norm = vector_norm(accel_mean)
    direction_cosine = sign * accel_mean[axis_index] / accel_norm
    direction_cosine = max(-1.0, min(1.0, direction_cosine))
    tilt_deg = math.degrees(math.acos(direction_cosine))
    gyro_rms = math.sqrt(fmean(value * value for value in gyro_norms))
    gyro_peak = max(gyro_norms)
    accel_norm_std = stdev(accel_norms)
    failures: list[str] = []
    if tilt_deg > max_pose_tilt_deg:
        failures.append(
            f"pose tilt {tilt_deg:.3f} deg exceeds "
            f"{max_pose_tilt_deg:.3f} deg"
        )
    if gyro_rms > max_gyro_rms:
        failures.append(
            f"gyro RMS {gyro_rms:.6f} rad/s exceeds {max_gyro_rms:.6f}"
        )
    if gyro_peak > max_gyro_peak:
        failures.append(
            f"gyro peak {gyro_peak:.6f} rad/s exceeds {max_gyro_peak:.6f}"
        )
    if accel_norm_std > max_accel_norm_std:
        failures.append(
            f"accel norm std {accel_norm_std:.6f} m/s^2 exceeds "
            f"{max_accel_norm_std:.6f}"
        )
    return {
        "samples": len(rows),
        "accel_mean": accel_mean,
        "accel_norm": accel_norm,
        "gravity_error": accel_norm - gravity,
        "gyro_mean": gyro_mean,
        "gyro_rms": gyro_rms,
        "gyro_peak": gyro_peak,
        "accel_norm_std": accel_norm_std,
        "tilt_deg": tilt_deg,
        "failures": failures,
    }


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        while chunk := source.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def yaml_vector(vector: list[float]) -> str:
    return "[" + ", ".join(f"{value:.12g}" for value in vector) + "]"


def append_matrix(
    lines: list[str], name: str, matrix: list[list[float]]
) -> None:
    indent = name[: len(name) - len(name.lstrip())]
    lines.append(f"{name}:")
    lines.extend(f"{indent}  - {yaml_vector(row)}" for row in matrix)


def build_report(
    serial: str,
    gravity: float,
    pose_paths: dict[str, Path],
    summaries: dict[str, dict[str, object]],
    fit: dict[str, object],
    thresholds: dict[str, float],
    validation: str,
    failure_reasons: list[str],
) -> str:
    lines = [
        "---",
        'format: "ovrs-six-position-accelerometer-diagnostic-v1"',
        'status: "DIAGNOSTIC_ONLY_NOT_KALIBR_PROMOTABLE"',
        f'calibrated_serial: "{serial}"',
        f"gravity_m_s2: {gravity:.12g}",
        "selected_runtime_modified: false",
        "device_eeprom_modified: false",
        "thresholds:",
    ]
    lines.extend(
        f"  {key}: {value:.12g}" for key, value in thresholds.items()
    )
    lines.append("poses:")
    for pose, _, _ in POSES:
        summary = summaries[pose]
        source = pose_paths[pose] / "imu" / "synchronized.csv"
        lines.extend(
            [
                f"  {pose}:",
                f'    dataset: "{pose_paths[pose]}"',
                f'    synchronized_sha256: "{sha256(source)}"',
                f"    samples: {summary['samples']}",
                "    acceleration_mean_m_s2: "
                f"{yaml_vector(summary['accel_mean'])}",
                f"    acceleration_norm_m_s2: {summary['accel_norm']:.12g}",
                f"    gravity_error_m_s2: {summary['gravity_error']:.12g}",
                f"    pose_tilt_deg: {summary['tilt_deg']:.12g}",
                f"    gyro_rms_rad_s: {summary['gyro_rms']:.12g}",
                f"    gyro_peak_rad_s: {summary['gyro_peak']:.12g}",
                "    acceleration_norm_std_m_s2: "
                f"{summary['accel_norm_std']:.12g}",
            ]
        )
    lines.extend(
        [
            "fit:",
            "  measurement_bias_m_s2: "
            f"{yaml_vector(fit['bias'])}",
            f"  pair_bias_spread_m_s2: {fit['pair_bias_spread']:.12g}",
            f"  condition_estimate: {fit['condition_estimate']:.12g}",
            f"  residual_rms_m_s2: {fit['residual_rms']:.12g}",
            f"  residual_max_m_s2: {fit['residual_max']:.12g}",
        ]
    )
    append_matrix(lines, "  measured_forward_matrix", fit["forward"])
    append_matrix(lines, "  correction_matrix", fit["correction"])
    lines.extend(
        [
            "notes:",
            '  - "The forward model is measurement = matrix * ideal + bias."',
            '  - "The correction matrix is the inverse forward matrix."',
            '  - "This report does not satisfy the Kalibr scale-misalignment promotion gate."',
            f'validation: "{validation}"',
        ]
    )
    if failure_reasons:
        lines.append("failure_reasons:")
        lines.extend(f'  - "{reason}"' for reason in failure_reasons)
    return "\n".join(lines) + "\n"


def positive_finite(
    parser: argparse.ArgumentParser, name: str, value: float
) -> None:
    if not math.isfinite(value) or value <= 0.0:
        parser.error(f"{name} must be positive and finite")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Fit a diagnostic affine accelerometer model from six separate "
            "stationary imu-allan captures. This never writes device EEPROM "
            "or promotes a runtime calibration."
        )
    )
    for pose, _, _ in POSES:
        parser.add_argument(f"--{pose}", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gravity-m-s2", type=float, default=9.80665)
    parser.add_argument("--trim-start-s", type=float, default=2.0)
    parser.add_argument("--trim-end-s", type=float, default=2.0)
    parser.add_argument("--max-pose-tilt-deg", type=float, default=10.0)
    parser.add_argument("--max-gyro-rms-rad-s", type=float, default=0.02)
    parser.add_argument("--max-gyro-peak-rad-s", type=float, default=0.15)
    parser.add_argument(
        "--max-accel-norm-std-m-s2", type=float, default=0.08
    )
    parser.add_argument(
        "--max-fit-residual-m-s2", type=float, default=0.08
    )
    parser.add_argument("--max-condition-estimate", type=float, default=4.0)
    args = parser.parse_args()
    positive_finite(parser, "--gravity-m-s2", args.gravity_m_s2)
    for name in (
        "trim_start_s",
        "trim_end_s",
        "max_pose_tilt_deg",
        "max_gyro_rms_rad_s",
        "max_gyro_peak_rad_s",
        "max_accel_norm_std_m_s2",
        "max_fit_residual_m_s2",
        "max_condition_estimate",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0.0:
            parser.error(
                f"--{name.replace('_', '-')} must be finite and nonnegative"
            )
    if args.output.exists():
        parser.error(f"--output already exists: {args.output}")

    pose_paths = {
        pose: getattr(args, pose.replace("-", "_")) for pose, _, _ in POSES
    }
    summaries: dict[str, dict[str, object]] = {}
    serial: str | None = None
    policy: tuple[object, ...] | None = None
    failures: list[str] = []
    for pose, axis_index, sign in POSES:
        path = pose_paths[pose]
        try:
            info = validate_capture(path)
            if info.mode != "imu-allan":
                raise CalibrationError(
                    f"{path}: capture mode must be imu-allan"
                )
            current_policy = (
                info.gyro_rate_hz,
                info.accelerometer_rate_hz,
                info.gyro_sensitivity,
                info.gyro_scale_factor,
                info.motion_correction_active,
                info.global_time_enabled,
            )
            if serial is None:
                serial = info.serial
                policy = current_policy
            elif info.serial != serial:
                raise CalibrationError(
                    f"{path}: serial {info.serial} does not match {serial}"
                )
            elif current_policy != policy:
                raise CalibrationError(
                    f"{path}: stream/motion policy differs from other poses"
                )
            rows = load_stationary_rows(
                path / "imu" / "synchronized.csv",
                args.trim_start_s,
                args.trim_end_s,
            )
            summary = summarize_pose(
                rows,
                axis_index,
                sign,
                args.gravity_m_s2,
                args.max_pose_tilt_deg,
                args.max_gyro_rms_rad_s,
                args.max_gyro_peak_rad_s,
                args.max_accel_norm_std_m_s2,
            )
            summaries[pose] = summary
            failures.extend(
                f"{pose}: {failure}" for failure in summary["failures"]
            )
        except (CalibrationError, OSError, ValueError) as error:
            parser.error(str(error))
    if failures:
        parser.error("; ".join(failures))
    means = {
        pose: summaries[pose]["accel_mean"] for pose, _, _ in POSES
    }
    try:
        fit = fit_six_position(means, args.gravity_m_s2)
    except ValueError as error:
        parser.error(str(error))
    fit_failures: list[str] = []
    if fit["residual_max"] > args.max_fit_residual_m_s2:
        fit_failures.append(
            "fit residual "
            f"{fit['residual_max']:.6f} m/s^2 exceeds "
            f"{args.max_fit_residual_m_s2:.6f}"
        )
    if fit["condition_estimate"] > args.max_condition_estimate:
        fit_failures.append(
            "fit condition estimate "
            f"{fit['condition_estimate']:.6f} exceeds "
            f"{args.max_condition_estimate:.6f}"
        )
    thresholds = {
        "trim_start_s": args.trim_start_s,
        "trim_end_s": args.trim_end_s,
        "max_pose_tilt_deg": args.max_pose_tilt_deg,
        "max_gyro_rms_rad_s": args.max_gyro_rms_rad_s,
        "max_gyro_peak_rad_s": args.max_gyro_peak_rad_s,
        "max_accel_norm_std_m_s2": args.max_accel_norm_std_m_s2,
        "max_fit_residual_m_s2": args.max_fit_residual_m_s2,
        "max_condition_estimate": args.max_condition_estimate,
    }
    report = build_report(
        serial or "",
        args.gravity_m_s2,
        pose_paths,
        summaries,
        fit,
        thresholds,
        "FAIL" if fit_failures else "PASS",
        fit_failures,
    )
    try:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(report, encoding="utf-8")
    except OSError as error:
        parser.error(f"cannot write --output {args.output}: {error}")
    print(f"Six-position diagnostic report: {args.output}")
    print(f"measurement_bias_m_s2: {yaml_vector(fit['bias'])}")
    print(f"fit_residual_max_m_s2: {fit['residual_max']:.9f}")
    print(f"condition_estimate: {fit['condition_estimate']:.9f}")
    if fit_failures:
        for failure in fit_failures:
            print(f"failure: {failure}")
        print("validation: FAIL")
        return 5
    print("validation: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
