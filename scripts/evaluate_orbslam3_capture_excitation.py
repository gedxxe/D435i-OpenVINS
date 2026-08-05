#!/usr/bin/env python3
"""Compare candidate IMU excitation with a provenance-bound ORB-SLAM3 pass."""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from statistics import fmean


class ExcitationError(RuntimeError):
    """Raised when capture evidence is missing or internally inconsistent."""


@dataclass(frozen=True)
class ImuMetrics:
    rows: int
    duration_s: float
    mean_rate_hz: float
    analysis_bins: int
    acceleration_deltas: int
    acceleration_delta_mean_m_s2: float
    acceleration_delta_p95_m_s2: float
    acceleration_deltas_at_threshold: int
    gyro_magnitude_mean_rad_s: float
    gyro_magnitude_p95_rad_s: float


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as source:
            for chunk in iter(lambda: source.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as error:
        raise ExcitationError(f"cannot hash {path}: {error}") from error
    return digest.hexdigest()


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except OSError as error:
        raise ExcitationError(f"cannot read {path}: {error}") from error


def flat_yaml(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line_number, raw_line in enumerate(read_text(path).splitlines(), 1):
        if raw_line[:1].isspace():
            continue
        line = raw_line.strip()
        if not line or line.startswith(("#", "%")) or ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", key):
            continue
        if key in result:
            raise ExcitationError(f"{path}:{line_number}: duplicate key {key}")
        value = value.split("#", 1)[0].strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1]
        result[key] = value
    return result


def require(mapping: dict[str, str], key: str, source: Path) -> str:
    value = mapping.get(key, "")
    if not value:
        raise ExcitationError(f"{source} lacks required {key}")
    return value


def require_bool(mapping: dict[str, str], key: str, source: Path) -> bool:
    value = require(mapping, key, source)
    if value not in {"true", "false"}:
        raise ExcitationError(f"{source} has invalid Boolean {key}")
    return value == "true"


def require_int(mapping: dict[str, str], key: str, source: Path) -> int:
    value = require(mapping, key, source)
    if not re.fullmatch(r"[0-9]+", value):
        raise ExcitationError(f"{source} has invalid nonnegative integer {key}")
    return int(value)


def percentile95(values: list[float]) -> float:
    if not values:
        raise ExcitationError("cannot calculate a percentile from no values")
    ordered = sorted(values)
    return ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)]


def load_metrics(
    imu_path: Path, analysis_rate_hz: float, acceleration_threshold: float
) -> ImuMetrics:
    timestamps_ns: list[int] = []
    gyroscope: list[tuple[float, float, float]] = []
    acceleration: list[tuple[float, float, float]] = []
    required_fields = (
        "#timestamp [ns]",
        "w_RS_S_x [rad s^-1]",
        "w_RS_S_y [rad s^-1]",
        "w_RS_S_z [rad s^-1]",
        "a_RS_S_x [m s^-2]",
        "a_RS_S_y [m s^-2]",
        "a_RS_S_z [m s^-2]",
    )
    try:
        with imu_path.open(newline="", encoding="utf-8") as source:
            reader = csv.DictReader(source)
            if tuple(reader.fieldnames or ()) != required_fields:
                raise ExcitationError(
                    f"{imu_path} does not have the canonical EuRoC IMU header"
                )
            previous: int | None = None
            for line_number, raw in enumerate(reader, 2):
                try:
                    timestamp = int(raw[required_fields[0]])
                    values = tuple(float(raw[field]) for field in required_fields[1:])
                except (TypeError, ValueError) as error:
                    raise ExcitationError(
                        f"{imu_path}:{line_number}: invalid numeric value"
                    ) from error
                if timestamp < 0 or (previous is not None and timestamp <= previous):
                    raise ExcitationError(
                        f"{imu_path}:{line_number}: timestamp is not increasing"
                    )
                if not all(math.isfinite(value) for value in values):
                    raise ExcitationError(
                        f"{imu_path}:{line_number}: non-finite IMU value"
                    )
                previous = timestamp
                timestamps_ns.append(timestamp)
                gyroscope.append(values[:3])
                acceleration.append(values[3:])
    except OSError as error:
        raise ExcitationError(f"cannot read {imu_path}: {error}") from error
    if len(timestamps_ns) < 3:
        raise ExcitationError(f"{imu_path} contains fewer than three rows")

    period_ns = 1_000_000_000.0 / analysis_rate_hz
    origin = timestamps_ns[0]
    bins: dict[int, list[tuple[float, float, float]]] = {}
    for timestamp, vector in zip(timestamps_ns, acceleration):
        index = int(math.floor((timestamp - origin) / period_ns))
        bins.setdefault(index, []).append(vector)
    ordered_indices = sorted(bins)
    if any(
        current != previous + 1
        for previous, current in zip(ordered_indices, ordered_indices[1:])
    ):
        raise ExcitationError(
            f"{imu_path} has an empty {analysis_rate_hz:g} Hz analysis bin"
        )
    averaged_acceleration = [
        tuple(fmean(vector[axis] for vector in bins[index]) for axis in range(3))
        for index in ordered_indices
    ]
    deltas = [
        math.sqrt(sum((current[axis] - previous[axis]) ** 2 for axis in range(3)))
        for previous, current in zip(
            averaged_acceleration, averaged_acceleration[1:]
        )
    ]
    gyro_magnitudes = [
        math.sqrt(sum(component * component for component in vector))
        for vector in gyroscope
    ]
    duration_s = (timestamps_ns[-1] - timestamps_ns[0]) / 1_000_000_000.0
    return ImuMetrics(
        rows=len(timestamps_ns),
        duration_s=duration_s,
        mean_rate_hz=(len(timestamps_ns) - 1) / duration_s,
        analysis_bins=len(averaged_acceleration),
        acceleration_deltas=len(deltas),
        acceleration_delta_mean_m_s2=fmean(deltas),
        acceleration_delta_p95_m_s2=percentile95(deltas),
        acceleration_deltas_at_threshold=sum(
            delta >= acceleration_threshold for delta in deltas
        ),
        gyro_magnitude_mean_rad_s=fmean(gyro_magnitudes),
        gyro_magnitude_p95_rad_s=percentile95(gyro_magnitudes),
    )


def validate_export(directory: Path) -> tuple[Path, dict[str, str]]:
    manifest = directory / "benchmark_manifest.yaml"
    imu = directory / "mav0" / "imu0" / "data.csv"
    if not directory.is_dir() or not manifest.is_file() or not imu.is_file():
        raise ExcitationError(f"incomplete benchmark export: {directory}")
    fields = flat_yaml(manifest)
    if require(fields, "format", manifest) != "ovrs-vislam-benchmark-v1":
        raise ExcitationError(f"{manifest} has an unsupported format")
    if require(fields, "state", manifest) != "EXPORTED_NOT_EVALUATED":
        raise ExcitationError(f"{manifest} is not a complete neutral export")
    if require(fields, "estimation_policy", manifest) != "MARKERLESS_STEREO_INERTIAL":
        raise ExcitationError(f"{manifest} has an incompatible estimation policy")
    expected_hash = require(fields, "imu0_data_csv_sha256", manifest)
    if not re.fullmatch(r"[0-9a-f]{64}", expected_hash) or sha256(imu) != expected_hash:
        raise ExcitationError(f"{imu} does not match its manifest hash")
    if require_int(fields, "synchronized_imu_rows", manifest) < 3:
        raise ExcitationError(f"{manifest} records insufficient IMU rows")
    return imu, fields


def validate_reference_result(reference: Path, result: Path) -> None:
    fields = flat_yaml(result)
    if require(fields, "format", result) != "ovrs-orbslam3-result-v1":
        raise ExcitationError(f"{result} has an unsupported result format")
    if not require_bool(fields, "tracking_gate_passed", result):
        raise ExcitationError(f"{result} is not a passing tracking reference")
    required_zero = ("imu_map_resets", "local_map_tracking_failures")
    if any(require_int(fields, key, result) != 0 for key in required_zero):
        raise ExcitationError(f"{result} has tracking failure events")
    required_positive = ("viba1_completions", "viba2_completions")
    if any(require_int(fields, key, result) < 1 for key in required_positive):
        raise ExcitationError(f"{result} lacks completed inertial BA stages")
    if require_int(fields, "created_maps", result) != 1:
        raise ExcitationError(f"{result} did not create exactly one map")
    if require_int(fields, "final_atlas_maps", result) != 1:
        raise ExcitationError(f"{result} did not finish with exactly one map")

    adapter = result.parent / "adapter_manifest.yaml"
    expected_adapter_hash = require(fields, "adapter_manifest_sha256", result)
    if sha256(adapter) != expected_adapter_hash:
        raise ExcitationError(f"{adapter} does not match the reference result")
    adapter_fields = flat_yaml(adapter)
    benchmark_manifest = reference / "benchmark_manifest.yaml"
    expected_benchmark_hash = require(
        adapter_fields, "source_benchmark_manifest_sha256", adapter
    )
    if sha256(benchmark_manifest) != expected_benchmark_hash:
        raise ExcitationError(
            "reference result is not bound to the supplied reference export"
        )


def yaml_float(value: float) -> str:
    return f"{value:.12g}"


def metric_lines(prefix: str, metrics: ImuMetrics) -> list[str]:
    return [
        f"{prefix}_imu_rows: {metrics.rows}",
        f"{prefix}_duration_s: {yaml_float(metrics.duration_s)}",
        f"{prefix}_mean_imu_rate_hz: {yaml_float(metrics.mean_rate_hz)}",
        f"{prefix}_analysis_bins: {metrics.analysis_bins}",
        f"{prefix}_acceleration_deltas: {metrics.acceleration_deltas}",
        f"{prefix}_acceleration_delta_mean_m_s2: "
        f"{yaml_float(metrics.acceleration_delta_mean_m_s2)}",
        f"{prefix}_acceleration_delta_p95_m_s2: "
        f"{yaml_float(metrics.acceleration_delta_p95_m_s2)}",
        f"{prefix}_acceleration_deltas_at_threshold: "
        f"{metrics.acceleration_deltas_at_threshold}",
        f"{prefix}_gyro_magnitude_mean_rad_s: "
        f"{yaml_float(metrics.gyro_magnitude_mean_rad_s)}",
        f"{prefix}_gyro_magnitude_p95_rad_s: "
        f"{yaml_float(metrics.gyro_magnitude_p95_rad_s)}",
    ]


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Compare a candidate capture's synchronized IMU excitation with "
            "a provenance-bound ORB-SLAM3 tracking pass. This diagnostic does "
            "not change estimator settings or evaluate visual quality or accuracy."
        )
    )
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--reference-result", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--analysis-rate-hz", type=float, default=30.0)
    parser.add_argument(
        "--acceleration-delta-threshold-m-s2", type=float, default=0.5
    )
    parser.add_argument("--minimum-duration-ratio", type=float, required=True)
    parser.add_argument(
        "--minimum-acceleration-delta-count-ratio", type=float, required=True
    )
    parser.add_argument("--maximum-gyro-mean-ratio", type=float, required=True)
    args = parser.parse_args()
    numeric = (
        args.analysis_rate_hz,
        args.acceleration_delta_threshold_m_s2,
        args.minimum_duration_ratio,
        args.minimum_acceleration_delta_count_ratio,
        args.maximum_gyro_mean_ratio,
    )
    if not all(math.isfinite(value) and value > 0 for value in numeric):
        parser.error("all rates, thresholds, and ratios must be finite and positive")
    if args.output.exists():
        parser.error(f"refusing to overwrite existing output: {args.output}")

    try:
        candidate_imu, candidate_manifest = validate_export(args.candidate)
        reference_imu, reference_manifest = validate_export(args.reference)
        validate_reference_result(args.reference, args.reference_result)
        if require(candidate_manifest, "camera_serial", args.candidate / "benchmark_manifest.yaml") != require(
            reference_manifest, "camera_serial", args.reference / "benchmark_manifest.yaml"
        ):
            raise ExcitationError("candidate and reference camera serials differ")
        if require(candidate_manifest, "imu_frame", args.candidate / "benchmark_manifest.yaml") != require(
            reference_manifest, "imu_frame", args.reference / "benchmark_manifest.yaml"
        ):
            raise ExcitationError("candidate and reference IMU frames differ")
        candidate = load_metrics(
            candidate_imu,
            args.analysis_rate_hz,
            args.acceleration_delta_threshold_m_s2,
        )
        reference = load_metrics(
            reference_imu,
            args.analysis_rate_hz,
            args.acceleration_delta_threshold_m_s2,
        )
        if reference.acceleration_deltas_at_threshold == 0:
            raise ExcitationError("reference has zero acceleration events at threshold")
        if reference.gyro_magnitude_mean_rad_s <= 0:
            raise ExcitationError("reference has zero mean gyro magnitude")
        duration_ratio = candidate.duration_s / reference.duration_s
        acceleration_ratio = (
            candidate.acceleration_deltas_at_threshold
            / reference.acceleration_deltas_at_threshold
        )
        gyro_ratio = (
            candidate.gyro_magnitude_mean_rad_s
            / reference.gyro_magnitude_mean_rad_s
        )
        duration_pass = duration_ratio >= args.minimum_duration_ratio
        acceleration_pass = (
            acceleration_ratio >= args.minimum_acceleration_delta_count_ratio
        )
        gyro_pass = gyro_ratio <= args.maximum_gyro_mean_ratio
        passed = duration_pass and acceleration_pass and gyro_pass
        state = (
            "CAPTURE_EXCITATION_GATE_PASS_VISUAL_NOT_EVALUATED"
            if passed
            else "CAPTURE_EXCITATION_GATE_FAILED"
        )
        lines = [
            "%YAML:1.0",
            'format: "ovrs-orbslam3-capture-excitation-v1"',
            f'state: "{state}"',
            "estimator_consumed_metrics: false",
            "visual_quality_evaluated: false",
            "trajectory_accuracy_evaluated: false",
            f"analysis_rate_hz: {yaml_float(args.analysis_rate_hz)}",
            "acceleration_delta_threshold_m_s2: "
            f"{yaml_float(args.acceleration_delta_threshold_m_s2)}",
            f"minimum_duration_ratio: {yaml_float(args.minimum_duration_ratio)}",
            "minimum_acceleration_delta_count_ratio: "
            f"{yaml_float(args.minimum_acceleration_delta_count_ratio)}",
            "maximum_gyro_mean_ratio: "
            f"{yaml_float(args.maximum_gyro_mean_ratio)}",
            f"duration_ratio: {yaml_float(duration_ratio)}",
            f"acceleration_delta_count_ratio: {yaml_float(acceleration_ratio)}",
            f"gyro_mean_ratio: {yaml_float(gyro_ratio)}",
            f"duration_gate_passed: {str(duration_pass).lower()}",
            f"acceleration_gate_passed: {str(acceleration_pass).lower()}",
            f"gyro_gate_passed: {str(gyro_pass).lower()}",
            f'candidate_source_dataset_name: "{require(candidate_manifest, "source_dataset_name", args.candidate / "benchmark_manifest.yaml")}"',
            f'reference_source_dataset_name: "{require(reference_manifest, "source_dataset_name", args.reference / "benchmark_manifest.yaml")}"',
            f'camera_serial: "{require(candidate_manifest, "camera_serial", args.candidate / "benchmark_manifest.yaml")}"',
            f'candidate_manifest_sha256: "{sha256(args.candidate / "benchmark_manifest.yaml")}"',
            f'reference_manifest_sha256: "{sha256(args.reference / "benchmark_manifest.yaml")}"',
            f'reference_result_sha256: "{sha256(args.reference_result)}"',
            *metric_lines("candidate", candidate),
            *metric_lines("reference", reference),
        ]
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    except (ExcitationError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 4

    print(f"Capture excitation manifest: {args.output}")
    print(f"State: {state}")
    return 0 if passed else 5


if __name__ == "__main__":
    raise SystemExit(main())
