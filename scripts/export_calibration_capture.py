#!/usr/bin/env python3
"""Validate and stage an OVRS calibration capture without requiring ROS."""

from __future__ import annotations

import argparse
import csv
import math
import os
import shutil
import sys
from pathlib import Path

from calibration_common import (
    CalibrationError,
    iter_csv_rows,
    seconds_to_nanoseconds,
    sha256_file,
    simple_yaml_map,
    validate_capture,
)

KALIBR_TIMESTAMP_WIDTH = 19


def format_kalibr_timestamp(timestamp_ns: int, field: str) -> str:
    if timestamp_ns < 0:
        raise CalibrationError(f"{field} must not be negative")
    return f"{timestamp_ns:0{KALIBR_TIMESTAMP_WIDTH}d}"


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description=(
            "Validate an ovrs-calibration-capture-v1 directory and export a "
            "portable timestamped staging tree. This does not create a ROS "
            "bag and does not run or certify Kalibr."
        )
    )
    result.add_argument("--capture", required=True, type=Path)
    result.add_argument("--output", required=True, type=Path)
    result.add_argument(
        "--image-mode",
        choices=("copy", "hardlink"),
        default="copy",
        help="copy is portable; hardlink saves space but requires one filesystem",
    )
    return result


def prepare_output(path: Path) -> Path:
    output = path.resolve()
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise CalibrationError(f"output exists and is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    (output / "INCOMPLETE").write_text(
        "Calibration export did not complete.\n", encoding="utf-8"
    )
    return output


def transfer_image(source: Path, destination: Path, mode: str) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if mode == "copy":
        shutil.copy2(source, destination)
        return
    try:
        os.link(source, destination)
    except OSError as exc:
        raise CalibrationError(
            f"cannot hardlink {source} to {destination}; use --image-mode copy"
        ) from exc


def stage_calibration_target(source: Path, destination: Path) -> None:
    target = simple_yaml_map(source)
    destination.write_text(
        "target_type: aprilgrid\n"
        f"tagRows: {target['tagRows']}\n"
        f"tagCols: {target['tagCols']}\n"
        f"tagSize: {target['tagSize']}\n"
        f"tagSpacing: {target['tagSpacing']}\n",
        encoding="utf-8",
    )


def export_camera(info, output: Path, mode: str) -> None:
    for camera in ("cam0", "cam1"):
        source_csv = info.root / camera / "data.csv"
        rows = iter_csv_rows(
            source_csv,
            ("timestamp_s", "raw_timestamp_ms", "frameset_number", "file"),
        )
        destination_dir = output / camera
        destination_dir.mkdir(parents=True, exist_ok=True)
        destination_csv = (
            output / "ovrs_metadata" / f"{camera}_index.csv"
        )
        destination_csv.parent.mkdir(parents=True, exist_ok=True)
        previous_timestamp_ns: int | None = None
        with destination_csv.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle, lineterminator="\n")
            writer.writerow(
                (
                    "timestamp_ns",
                    "source_timestamp_s",
                    "raw_timestamp_ms",
                    "frameset_number",
                    "file",
                )
            )
            for index, row in enumerate(rows, 2):
                timestamp_ns = seconds_to_nanoseconds(
                    row["timestamp_s"], f"{source_csv}:{index}:timestamp_s"
                )
                if (
                    previous_timestamp_ns is not None
                    and timestamp_ns <= previous_timestamp_ns
                ):
                    raise CalibrationError(
                        f"{source_csv}:{index}: nanosecond timestamp collision"
                    )
                previous_timestamp_ns = timestamp_ns
                kalibr_timestamp = format_kalibr_timestamp(
                    timestamp_ns, f"{source_csv}:{index}:timestamp_s"
                )
                filename = f"{kalibr_timestamp}.png"
                transfer_image(
                    info.root / camera / "data" / row["file"],
                    destination_dir / filename,
                    mode,
                )
                writer.writerow(
                    (
                        kalibr_timestamp,
                        row["timestamp_s"],
                        row["raw_timestamp_ms"],
                        row["frameset_number"],
                        filename,
                    )
                )


def export_imu(info, output: Path) -> None:
    synchronized_path = info.root / "imu" / "synchronized.csv"
    rows = iter_csv_rows(
        synchronized_path,
        (
            "timestamp_s",
            "raw_gyro_timestamp_ms",
            "wx_rad_s",
            "wy_rad_s",
            "wz_rad_s",
            "ax_m_s2",
            "ay_m_s2",
            "az_m_s2",
            "interpolation_delay_s",
        ),
    )
    with (output / "imu0.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(
            (
                "timestamp",
                "omega_x",
                "omega_y",
                "omega_z",
                "alpha_x",
                "alpha_y",
                "alpha_z",
            )
        )
        for index, row in enumerate(rows, 2):
            timestamp_field = (
                f"{synchronized_path}:{index}:timestamp_s"
            )
            timestamp_ns = seconds_to_nanoseconds(
                row["timestamp_s"], timestamp_field
            )
            writer.writerow(
                (
                    format_kalibr_timestamp(timestamp_ns, timestamp_field),
                    row["wx_rad_s"],
                    row["wy_rad_s"],
                    row["wz_rad_s"],
                    row["ax_m_s2"],
                    row["ay_m_s2"],
                    row["az_m_s2"],
                )
            )
    raw_output = output / "ovrs_metadata" / "imu_raw"
    raw_output.mkdir(parents=True, exist_ok=True)
    shutil.copy2(info.root / "imu" / "gyro.csv", raw_output / "gyro.csv")
    shutil.copy2(
        info.root / "imu" / "accelerometer.csv",
        raw_output / "accelerometer.csv",
    )


def write_manifest(info, output: Path, image_mode: str) -> None:
    provenance_files = [
        "dataset_metadata.yaml",
        "recording_summary.yaml",
        "device_report.yaml",
        "resolved_stream_config.yaml",
    ]
    provenance_output = output / "ovrs_metadata"
    provenance_output.mkdir(parents=True, exist_ok=True)
    provenance_hashes: list[tuple[str, str]] = []
    for relative in provenance_files:
        source = info.root / relative
        destination = provenance_output / f"source_{Path(relative).name}"
        shutil.copy2(source, destination)
        provenance_hashes.append(
            (f"source_{Path(relative).stem}_sha256", sha256_file(source))
        )
    if info.calibration_target is not None:
        source_target = (
            provenance_output / "source_calibration_target.yaml"
        )
        shutil.copy2(info.calibration_target, source_target)
        staged_target = output / "target.yaml"
        stage_calibration_target(info.calibration_target, staged_target)
        provenance_hashes.append(
            (
                "source_calibration_target_sha256",
                sha256_file(info.calibration_target),
            )
        )
        provenance_hashes.append(
            (
                "staged_calibration_target_sha256",
                sha256_file(staged_target),
            )
        )
    lines = [
        "%YAML:1.0",
        'format: "ovrs-calibration-export-v2"',
        'provenance_layout: "ovrs-export-provenance-v1"',
        f'capture_mode: "{info.mode}"',
        f'calibrated_serial: "{info.serial}"',
        f'infrared_profile: "{info.infrared_profile}"',
        "complete: true",
        'calibration_state: "UNVERIFIED_CAPTURE"',
        "ros_bag_created: false",
        "kalibr_executed: false",
        f'image_transfer_mode: "{image_mode}"',
        f"camera_rows_per_camera: {info.camera_rows}",
        f"synchronized_imu_rows: {info.synchronized_imu_rows}",
        f"recording_duration_s: {info.recording_duration_s}",
        f"gyro_rate_hz: {info.gyro_rate_hz}",
        f"accelerometer_rate_hz: {info.accelerometer_rate_hz}",
        "motion_correction_active: "
        + ("true" if info.motion_correction_active else "false"),
        "global_time_enabled: "
        + ("true" if info.global_time_enabled else "false"),
    ]
    if info.gyro_sensitivity is not None:
        lines.append(f"gyro_sensitivity: {info.gyro_sensitivity}")
    if info.gyro_scale_factor is not None:
        lines.append(f"gyro_scale_factor: {info.gyro_scale_factor}")
    for key, digest in provenance_hashes:
        lines.append(f'{key}: "{digest}"')
    (output / "calibration_export_manifest.yaml").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    (output / "README.txt").write_text(
        "This directory is validated calibration staging data, not a ROS bag.\n"
        "It has not run Kalibr and must not be labelled KALIBR_VERIFIED.\n"
        "cam0/cam1 contain normalized nanosecond PNG names and imu0.csv uses\n"
        "the official kalibr_bagcreater column contract. ovrs_metadata keeps\n"
        "timestamp provenance, copied source metadata with SHA-256 checks, "
        "and separately sampled raw IMU streams.\n",
        encoding="utf-8",
    )
    if info.mode == "imu-allan":
        sequence_time = max(1, math.floor(info.recording_duration_s))
        (output / "allan_variance_config.yaml").write_text(
            'imu_topic: "/imu0"\n'
            f"imu_rate: {info.gyro_rate_hz}\n"
            f"measure_rate: {info.gyro_rate_hz}\n"
            f"sequence_time: {sequence_time}\n",
            encoding="utf-8",
        )


def main() -> int:
    args = parser().parse_args()
    output: Path | None = None
    try:
        info = validate_capture(args.capture)
        output = prepare_output(args.output)
        if info.stereo_enabled:
            export_camera(info, output, args.image_mode)
        if info.motion_enabled:
            export_imu(info, output)
        write_manifest(info, output, args.image_mode)
        (output / "INCOMPLETE").unlink()
        print(f"Calibration export complete: {output}")
        print("calibration_state=UNVERIFIED_CAPTURE")
        print("ros_bag_created=false")
        print("kalibr_executed=false")
        return 0
    except (CalibrationError, OSError, csv.Error) as exc:
        print(f"Calibration export failed: {exc}", file=sys.stderr)
        if output is not None:
            print(
                f"Partial export remains marked INCOMPLETE: {output}",
                file=sys.stderr,
            )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
