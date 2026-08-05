#!/usr/bin/env python3
"""Export one complete OVRS VIO recording as a backend-neutral EuRoC dataset."""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
import os
import re
import shutil
import sys
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation, ROUND_HALF_EVEN
from itertools import zip_longest
from pathlib import Path
from typing import Iterator


class BenchmarkError(RuntimeError):
    """Raised when source evidence is incomplete or internally inconsistent."""


CAMERA_FIELDS = (
    "timestamp_s",
    "raw_timestamp_ms",
    "frameset_number",
    "file",
)
IMU_FIELDS = (
    "timestamp_s",
    "raw_gyro_timestamp_ms",
    "wx_rad_s",
    "wy_rad_s",
    "wz_rad_s",
    "ax_m_s2",
    "ay_m_s2",
    "az_m_s2",
    "interpolation_delay_s",
)
FATAL_CAPTURE_COUNTERS = (
    "dropped_camera_frames",
    "malformed_frames",
    "rejected_timestamps",
    "callback_errors",
    "stereo_queue_drops",
    "gyro_queue_drops",
    "accelerometer_queue_drops",
    "imu_duplicate_timestamps",
    "imu_regressing_timestamps",
    "imu_invalid_values",
    "imu_synchronizer_capacity_drops",
)


@dataclass(frozen=True)
class CameraRow:
    timestamp: Decimal
    raw_timestamp_ms: Decimal
    frameset_number: int
    filename: str
    source_image: Path


@dataclass(frozen=True)
class ExportRange:
    rows: int
    first_timestamp_ns: int
    last_timestamp_ns: int
    source_rows: int
    skipped_leading_rows: int = 0
    skipped_trailing_rows: int = 0


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except OSError as exc:
        raise BenchmarkError(f"cannot read {path}: {exc}") from exc


def simple_yaml_map(path: Path) -> dict[str, str]:
    """Parse the flat top-level scalar subset emitted by OVRS."""
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
            raise BenchmarkError(
                f"{path}:{line_number}: duplicate scalar key {key}"
            )
        value = value.split("#", 1)[0].strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1]
        result[key] = value
    return result


def parse_decimal(value: str, field: str) -> Decimal:
    try:
        parsed = Decimal(value)
    except InvalidOperation as exc:
        raise BenchmarkError(f"{field} is not numeric") from exc
    if not parsed.is_finite():
        raise BenchmarkError(f"{field} is not finite")
    return parsed


def parse_nonnegative_int(value: str, field: str) -> int:
    if not re.fullmatch(r"[0-9]+", value):
        raise BenchmarkError(f"{field} must be a nonnegative integer")
    return int(value)


def seconds_to_nanoseconds(value: Decimal, field: str) -> int:
    if value < 0:
        raise BenchmarkError(f"{field} must not be negative")
    return int(
        (value * Decimal(1_000_000_000)).to_integral_value(
            rounding=ROUND_HALF_EVEN
        )
    )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise BenchmarkError(f"cannot hash {path}: {exc}") from exc
    return digest.hexdigest()


def yaml_quote(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


def csv_rows(path: Path, expected_fields: tuple[str, ...]) -> Iterator[dict[str, str]]:
    try:
        handle = path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise BenchmarkError(f"cannot read {path}: {exc}") from exc
    with handle:
        reader = csv.DictReader(handle)
        if tuple(reader.fieldnames or ()) != expected_fields:
            raise BenchmarkError(
                f"{path}: expected CSV fields {','.join(expected_fields)}"
            )
        for line_number, row in enumerate(reader, 2):
            if None in row or any(value is None for value in row.values()):
                raise BenchmarkError(f"{path}:{line_number}: malformed CSV row")
            yield row


def camera_rows(dataset: Path, camera: str) -> Iterator[CameraRow]:
    csv_path = dataset / camera / "data.csv"
    previous_timestamp: Decimal | None = None
    previous_raw_timestamp: Decimal | None = None
    previous_frameset: int | None = None
    for line_number, row in enumerate(csv_rows(csv_path, CAMERA_FIELDS), 2):
        field = f"{csv_path}:{line_number}"
        timestamp = parse_decimal(row["timestamp_s"], f"{field}:timestamp_s")
        raw_timestamp = parse_decimal(
            row["raw_timestamp_ms"], f"{field}:raw_timestamp_ms"
        )
        frameset = parse_nonnegative_int(
            row["frameset_number"], f"{field}:frameset_number"
        )
        filename = row["file"]
        if Path(filename).name != filename or filename != f"{frameset}.png":
            raise BenchmarkError(f"{field}: unsafe or mismatched image filename")
        if previous_timestamp is not None and timestamp <= previous_timestamp:
            raise BenchmarkError(f"{field}: timestamp is not increasing")
        if (
            previous_raw_timestamp is not None
            and raw_timestamp <= previous_raw_timestamp
        ):
            raise BenchmarkError(f"{field}: raw timestamp is not increasing")
        if previous_frameset is not None and frameset <= previous_frameset:
            raise BenchmarkError(f"{field}: frameset number is not increasing")
        image = dataset / camera / "data" / filename
        if not image.is_file():
            raise BenchmarkError(f"{field}: missing image {image}")
        previous_timestamp = timestamp
        previous_raw_timestamp = raw_timestamp
        previous_frameset = frameset
        yield CameraRow(timestamp, raw_timestamp, frameset, filename, image)


def transfer_image(source: Path, destination: Path, mode: str) -> None:
    if mode == "copy":
        shutil.copy2(source, destination)
        return
    try:
        os.link(source, destination)
    except OSError as exc:
        raise BenchmarkError(
            f"cannot hardlink {source} to {destination}; use --image-mode copy"
        ) from exc


def export_stereo(
    dataset: Path,
    output: Path,
    image_mode: str,
    tolerance_ms: Decimal,
    imu_range: ExportRange,
) -> ExportRange:
    left_csv = output / "mav0" / "cam0" / "data.csv"
    right_csv = output / "mav0" / "cam1" / "data.csv"
    left_data = left_csv.parent / "data"
    right_data = right_csv.parent / "data"
    left_data.mkdir(parents=True)
    right_data.mkdir(parents=True)
    previous_timestamp_ns: int | None = None
    source_row_count = 0
    exported_row_count = 0
    skipped_leading_rows = 0
    skipped_trailing_rows = 0
    first_timestamp_ns: int | None = None
    last_timestamp_ns: int | None = None
    tolerance_s = tolerance_ms / Decimal(1000)

    with left_csv.open("w", encoding="utf-8", newline="") as left_handle, (
        right_csv.open("w", encoding="utf-8", newline="")
    ) as right_handle:
        left_writer = csv.writer(left_handle, lineterminator="\n")
        right_writer = csv.writer(right_handle, lineterminator="\n")
        left_writer.writerow(("#timestamp [ns]", "filename"))
        right_writer.writerow(("#timestamp [ns]", "filename"))
        pairs = zip_longest(
            camera_rows(dataset, "cam0"), camera_rows(dataset, "cam1")
        )
        for source_row_count, pair in enumerate(pairs, 1):
            left, right = pair
            if left is None or right is None:
                raise BenchmarkError("cam0 and cam1 row counts differ")
            if left.frameset_number != right.frameset_number:
                raise BenchmarkError(
                    f"stereo pair {source_row_count}: frameset numbers differ"
                )
            if abs(left.timestamp - right.timestamp) > tolerance_s:
                raise BenchmarkError(
                    f"stereo pair {source_row_count}: timestamps exceed "
                    f"{tolerance_ms} ms tolerance"
                )
            timestamp = (left.timestamp + right.timestamp) / Decimal(2)
            timestamp_ns = seconds_to_nanoseconds(
                timestamp, f"stereo pair {source_row_count}:timestamp"
            )
            if timestamp_ns < imu_range.first_timestamp_ns:
                skipped_leading_rows += 1
                continue
            if timestamp_ns > imu_range.last_timestamp_ns:
                skipped_trailing_rows += 1
                continue
            if (
                previous_timestamp_ns is not None
                and timestamp_ns <= previous_timestamp_ns
            ):
                raise BenchmarkError(
                    f"stereo pair {source_row_count}: nanosecond timestamp collision"
                )
            exported_row_count += 1
            previous_timestamp_ns = timestamp_ns
            first_timestamp_ns = (
                timestamp_ns if first_timestamp_ns is None else first_timestamp_ns
            )
            last_timestamp_ns = timestamp_ns
            filename = f"{timestamp_ns}.png"
            transfer_image(left.source_image, left_data / filename, image_mode)
            transfer_image(right.source_image, right_data / filename, image_mode)
            left_writer.writerow((timestamp_ns, filename))
            right_writer.writerow((timestamp_ns, filename))

    if (
        source_row_count == 0
        or exported_row_count == 0
        or first_timestamp_ns is None
        or last_timestamp_ns is None
    ):
        raise BenchmarkError("dataset has no stereo pairs inside the IMU range")
    return ExportRange(
        exported_row_count,
        first_timestamp_ns,
        last_timestamp_ns,
        source_row_count,
        skipped_leading_rows,
        skipped_trailing_rows,
    )


def export_imu(dataset: Path, output: Path) -> ExportRange:
    source = dataset / "imu" / "synchronized.csv"
    destination = output / "mav0" / "imu0" / "data.csv"
    destination.parent.mkdir(parents=True)
    previous_timestamp_ns: int | None = None
    first_timestamp_ns: int | None = None
    last_timestamp_ns: int | None = None
    row_count = 0

    with destination.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(
            (
                "#timestamp [ns]",
                "w_RS_S_x [rad s^-1]",
                "w_RS_S_y [rad s^-1]",
                "w_RS_S_z [rad s^-1]",
                "a_RS_S_x [m s^-2]",
                "a_RS_S_y [m s^-2]",
                "a_RS_S_z [m s^-2]",
            )
        )
        for row_count, row in enumerate(csv_rows(source, IMU_FIELDS), 1):
            field = f"{source}:{row_count + 1}"
            timestamp = parse_decimal(
                row["timestamp_s"], f"{field}:timestamp_s"
            )
            timestamp_ns = seconds_to_nanoseconds(
                timestamp, f"{field}:timestamp_s"
            )
            if (
                previous_timestamp_ns is not None
                and timestamp_ns <= previous_timestamp_ns
            ):
                raise BenchmarkError(f"{field}: timestamp is not increasing")
            previous_timestamp_ns = timestamp_ns
            first_timestamp_ns = (
                timestamp_ns if first_timestamp_ns is None else first_timestamp_ns
            )
            last_timestamp_ns = timestamp_ns
            values = []
            for name in (
                "wx_rad_s",
                "wy_rad_s",
                "wz_rad_s",
                "ax_m_s2",
                "ay_m_s2",
                "az_m_s2",
                "interpolation_delay_s",
                "raw_gyro_timestamp_ms",
            ):
                value = parse_decimal(row[name], f"{field}:{name}")
                if not math.isfinite(float(value)):
                    raise BenchmarkError(f"{field}:{name} is not finite")
                values.append(value)
            if values[6] < 0:
                raise BenchmarkError(
                    f"{field}:interpolation_delay_s must not be negative"
                )
            writer.writerow(
                (timestamp_ns, *(str(value) for value in values[:6]))
            )

    if row_count == 0 or first_timestamp_ns is None or last_timestamp_ns is None:
        raise BenchmarkError("dataset has no synchronized IMU rows")
    return ExportRange(
        row_count, first_timestamp_ns, last_timestamp_ns, row_count
    )


def validate_source(dataset: Path) -> tuple[dict[str, str], dict[str, str], Decimal]:
    required = (
        "dataset_metadata.yaml",
        "device_report.yaml",
        "resolved_stream_config.yaml",
        "recording_summary.yaml",
        "cam0/data.csv",
        "cam1/data.csv",
        "imu/synchronized.csv",
    )
    if not dataset.is_dir():
        raise BenchmarkError(f"dataset is not a directory: {dataset}")
    if (dataset / "INCOMPLETE").exists():
        raise BenchmarkError("source dataset is marked INCOMPLETE")
    for relative in required:
        if not (dataset / relative).is_file():
            raise BenchmarkError(f"source dataset is missing {relative}")

    metadata = simple_yaml_map(dataset / "dataset_metadata.yaml")
    if metadata.get("format") != "ovrs-euroc-like-v1":
        raise BenchmarkError("source format must be ovrs-euroc-like-v1")
    if metadata.get("capture_mode") != "vio":
        raise BenchmarkError("source capture_mode must be vio")
    if metadata.get("complete") != "true":
        raise BenchmarkError("source dataset does not declare complete: true")
    if metadata.get("replay_compatible") != "true":
        raise BenchmarkError("source dataset is not replay compatible")

    device = simple_yaml_map(dataset / "device_report.yaml")
    if not device.get("serial"):
        raise BenchmarkError("device report is missing the camera serial")
    stream = simple_yaml_map(dataset / "resolved_stream_config.yaml")
    if stream.get("serial") and stream["serial"] != device["serial"]:
        raise BenchmarkError("stream config and device serials differ")
    tolerance = parse_decimal(
        stream.get("stereo_tolerance_ms", ""),
        "resolved_stream_config.yaml:stereo_tolerance_ms",
    )
    if tolerance < 0 or tolerance > 20:
        raise BenchmarkError("stereo_tolerance_ms is outside [0, 20]")

    summary = simple_yaml_map(dataset / "recording_summary.yaml")
    for name in FATAL_CAPTURE_COUNTERS:
        value = parse_nonnegative_int(
            summary.get(name, ""), f"recording_summary.yaml:{name}"
        )
        if value != 0:
            raise BenchmarkError(
                f"recording_summary.yaml:{name} must be zero"
            )
    return metadata, device, tolerance


def prepare_output(path: Path) -> Path:
    output = path.resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise BenchmarkError(f"output exists and is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    (output / "INCOMPLETE").write_text(
        "VSLAM benchmark export did not complete.\n", encoding="utf-8"
    )
    return output


def copy_provenance(dataset: Path, output: Path) -> dict[str, str]:
    provenance = output / "ovrs_metadata"
    provenance.mkdir()
    hashes: dict[str, str] = {}
    for name in (
        "dataset_metadata.yaml",
        "device_report.yaml",
        "resolved_stream_config.yaml",
        "recording_summary.yaml",
    ):
        source = dataset / name
        shutil.copy2(source, provenance / name)
        hashes[name] = sha256_file(source)
    return hashes


def write_manifest(
    output: Path,
    dataset: Path,
    device: dict[str, str],
    stereo: ExportRange,
    imu: ExportRange,
    hashes: dict[str, str],
    image_mode: str,
) -> None:
    lines = [
        '%YAML:1.0',
        'format: "ovrs-vislam-benchmark-v1"',
        'state: "EXPORTED_NOT_EVALUATED"',
        'estimation_policy: "MARKERLESS_STEREO_INERTIAL"',
        'ground_truth_consumed_by_estimator: false',
        f"source_dataset_name: {yaml_quote(dataset.name)}",
        f"camera_serial: {yaml_quote(device['serial'])}",
        f"image_transfer_mode: {yaml_quote(image_mode)}",
        'timestamp_policy: "OVRS_NORMALIZED_SECONDS_TO_INTEGER_NANOSECONDS"',
        'stereo_timestamp_policy: "PAIR_MIDPOINT"',
        'imu_frame: "GYROSCOPE_FRAME"',
        'accelerometer_frame: "ROTATED_TO_GYROSCOPE_FRAME"',
        f"stereo_pairs: {stereo.rows}",
        f"source_stereo_pairs: {stereo.source_rows}",
        f"skipped_leading_stereo_outside_imu_range: {stereo.skipped_leading_rows}",
        f"skipped_trailing_stereo_outside_imu_range: {stereo.skipped_trailing_rows}",
        f"synchronized_imu_rows: {imu.rows}",
        f"first_stereo_timestamp_ns: {stereo.first_timestamp_ns}",
        f"last_stereo_timestamp_ns: {stereo.last_timestamp_ns}",
        f"first_imu_timestamp_ns: {imu.first_timestamp_ns}",
        f"last_imu_timestamp_ns: {imu.last_timestamp_ns}",
        (
            "cam0_data_csv_sha256: "
            f"{yaml_quote(sha256_file(output / 'mav0' / 'cam0' / 'data.csv'))}"
        ),
        (
            "cam1_data_csv_sha256: "
            f"{yaml_quote(sha256_file(output / 'mav0' / 'cam1' / 'data.csv'))}"
        ),
        (
            "imu0_data_csv_sha256: "
            f"{yaml_quote(sha256_file(output / 'mav0' / 'imu0' / 'data.csv'))}"
        ),
    ]
    for name, digest in hashes.items():
        key = name.replace(".", "_") + "_sha256"
        lines.append(f"{key}: {yaml_quote(digest)}")
    (output / "benchmark_manifest.yaml").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )


def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Validate one complete ovrs-euroc-like-v1 VIO recording and "
            "export backend-neutral EuRoC stereo/IMU files. This does not "
            "run, tune, or certify a SLAM backend."
        )
    )
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--image-mode",
        choices=("copy", "hardlink"),
        default="hardlink",
        help="hardlink saves space; copy works across filesystems",
    )
    return parser


def main() -> int:
    args = argument_parser().parse_args()
    output: Path | None = None
    try:
        dataset = args.dataset.resolve()
        _, device, tolerance = validate_source(dataset)
        output = prepare_output(args.output)
        hashes = copy_provenance(dataset, output)
        imu = export_imu(dataset, output)
        stereo = export_stereo(
            dataset, output, args.image_mode, tolerance, imu
        )
        write_manifest(
            output,
            dataset,
            device,
            stereo,
            imu,
            hashes,
            args.image_mode,
        )
        (output / "INCOMPLETE").unlink()
        print(f"VSLAM benchmark export: PASS: {output}")
        print(f"stereo_pairs={stereo.rows}")
        print(f"synchronized_imu_rows={imu.rows}")
        print("state=EXPORTED_NOT_EVALUATED")
        return 0
    except (BenchmarkError, OSError) as exc:
        print(f"VSLAM benchmark export: FAIL: {exc}", file=sys.stderr)
        if output is not None and (output / "INCOMPLETE").exists():
            print(f"Export remains marked INCOMPLETE: {output}", file=sys.stderr)
        return 4


if __name__ == "__main__":
    raise SystemExit(main())
