#!/usr/bin/env python3
"""Validate that three calibration exports belong to one coherent session."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

from calibration_common import (
    CalibrationError,
    export_global_time_enabled,
    iter_csv_rows,
    parse_bool,
    parse_nonnegative_int,
    png_dimensions,
    sha256_file,
    simple_yaml_map,
    validate_export_provenance,
)


MANIFEST_NAME = "calibration_export_manifest.yaml"
SOURCE_METADATA = "ovrs_metadata/source_dataset_metadata.yaml"
SOURCE_STREAM_CONFIG = "ovrs_metadata/source_resolved_stream_config.yaml"

SOURCE_BOUND_FIELDS = (
    "capture_mode",
    "calibrated_serial",
    "infrared_profile",
    "gyro_rate_hz",
    "accelerometer_rate_hz",
    "motion_correction_active",
)
KALIBR_TIMESTAMP_PATTERN = re.compile(r"[0-9]{19}")


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description=(
            "Fail closed unless Allan, stereo, and camera-IMU exports have "
            "matching serial, target, stream profile, and IMU policy."
        )
    )
    result.add_argument("--allan-export", required=True, type=Path)
    result.add_argument("--stereo-export", required=True, type=Path)
    result.add_argument("--imu-camera-export", required=True, type=Path)
    result.add_argument("--output-report", required=True, type=Path)
    return result


def load_manifest(export: Path, expected_mode: str) -> tuple[Path, dict[str, str]]:
    if not export.is_dir():
        raise CalibrationError(f"export directory does not exist: {export}")
    if (export / "INCOMPLETE").exists():
        raise CalibrationError(f"export is marked INCOMPLETE: {export}")
    manifest_path = export / MANIFEST_NAME
    manifest = simple_yaml_map(manifest_path)
    if manifest.get("format") != "ovrs-calibration-export-v2":
        raise CalibrationError(f"{manifest_path}: unsupported export format")
    if manifest.get("complete") != "true":
        raise CalibrationError(f"{manifest_path}: export is not complete")
    if manifest.get("calibration_state") != "UNVERIFIED_CAPTURE":
        raise CalibrationError(
            f"{manifest_path}: expected UNVERIFIED_CAPTURE state"
        )
    if manifest.get("capture_mode") != expected_mode:
        raise CalibrationError(
            f"{manifest_path}: expected capture_mode {expected_mode}"
        )
    serial = manifest.get("calibrated_serial", "")
    if not re.fullmatch(r"[0-9]+", serial):
        raise CalibrationError(
            f"{manifest_path}: calibrated_serial must contain digits only"
        )
    validate_export_provenance(manifest_path, manifest)
    source_metadata_path = export / SOURCE_METADATA
    source_metadata = simple_yaml_map(source_metadata_path)
    for key in SOURCE_BOUND_FIELDS:
        if manifest.get(key, "") != source_metadata.get(key, ""):
            raise CalibrationError(
                f"{manifest_path}: {key} does not match "
                f"{source_metadata_path}"
            )
    validate_staged_layout(export, manifest_path, manifest)
    return manifest_path, manifest


def validate_staged_layout(
    export: Path, manifest_path: Path, manifest: dict[str, str]
) -> None:
    camera_rows = parse_nonnegative_int(
        manifest.get("camera_rows_per_camera", ""),
        f"{manifest_path}:camera_rows_per_camera",
    )
    imu_rows = parse_nonnegative_int(
        manifest.get("synchronized_imu_rows", ""),
        f"{manifest_path}:synchronized_imu_rows",
    )

    if camera_rows == 0:
        for camera in ("cam0", "cam1"):
            camera_dir = export / camera
            if camera_dir.exists() and any(camera_dir.iterdir()):
                raise CalibrationError(
                    f"{manifest_path}: unexpected camera data in {camera_dir}"
                )
    else:
        stream_config_path = export / SOURCE_STREAM_CONFIG
        stream_config = simple_yaml_map(stream_config_path)
        width = require_positive(stream_config, "width", stream_config_path)
        height = require_positive(stream_config, "height", stream_config_path)
        for camera in ("cam0", "cam1"):
            camera_dir = export / camera
            if not camera_dir.is_dir():
                raise CalibrationError(
                    f"{manifest_path}: missing camera directory {camera_dir}"
                )
            images = sorted(
                camera_dir.iterdir(),
                key=lambda path: (
                    int(path.stem) if path.stem.isdigit() else -1,
                    path.name,
                ),
            )
            if len(images) != camera_rows:
                raise CalibrationError(
                    f"{manifest_path}: {camera_dir} has {len(images)} "
                    f"entries, expected {camera_rows}"
                )
            index_path = export / "ovrs_metadata" / f"{camera}_index.csv"
            index_rows = list(
                iter_csv_rows(
                    index_path,
                    (
                        "timestamp_ns",
                        "source_timestamp_s",
                        "raw_timestamp_ms",
                        "frameset_number",
                        "file",
                    ),
                    "timestamp_ns",
                )
            )
            if len(index_rows) != camera_rows:
                raise CalibrationError(
                    f"{manifest_path}: {index_path} has {len(index_rows)} "
                    f"rows, expected {camera_rows}"
                )
            # The equal-length checks above provide Python 3.8-compatible
            # strict zip behavior for the Ubuntu 20.04 calibration image.
            for image_path, row in zip(images, index_rows):
                if (
                    not image_path.is_file()
                    or image_path.suffix != ".png"
                    or not KALIBR_TIMESTAMP_PATTERN.fullmatch(image_path.stem)
                    or row["file"] != image_path.name
                    or row["timestamp_ns"] != image_path.stem
                ):
                    raise CalibrationError(
                        f"{manifest_path}: invalid staged camera entry "
                        f"{image_path}"
                    )
                if png_dimensions(image_path) != (width, height):
                    raise CalibrationError(
                        f"{manifest_path}: unexpected image dimensions in "
                        f"{image_path}"
                    )

    imu_path = export / "imu0.csv"
    if imu_rows == 0:
        if imu_path.exists():
            raise CalibrationError(
                f"{manifest_path}: unexpected synchronized IMU file {imu_path}"
            )
    else:
        actual_imu_rows = 0
        for index, row in enumerate(
            iter_csv_rows(
                imu_path,
                (
                    "timestamp",
                    "omega_x",
                    "omega_y",
                    "omega_z",
                    "alpha_x",
                    "alpha_y",
                    "alpha_z",
                ),
                "timestamp",
            ),
            2,
        ):
            if not KALIBR_TIMESTAMP_PATTERN.fullmatch(row["timestamp"]):
                raise CalibrationError(
                    f"{imu_path}:{index}: timestamp must be a zero-padded "
                    "19-digit nanosecond value for kalibr_bagcreater"
                )
            actual_imu_rows += 1
        if actual_imu_rows != imu_rows:
            raise CalibrationError(
                f"{manifest_path}: {imu_path} has {actual_imu_rows} rows, "
                f"expected {imu_rows}"
            )
        for raw_name in ("gyro.csv", "accelerometer.csv"):
            raw_path = export / "ovrs_metadata" / "imu_raw" / raw_name
            if not raw_path.is_file() or raw_path.stat().st_size == 0:
                raise CalibrationError(
                    f"{manifest_path}: missing raw IMU evidence {raw_path}"
                )


def require_positive(manifest: dict[str, str], key: str, source: Path) -> int:
    value = parse_nonnegative_int(manifest.get(key, ""), f"{source}:{key}")
    if value <= 0:
        raise CalibrationError(f"{source}: {key} must be positive")
    return value


def require_zero(manifest: dict[str, str], key: str, source: Path) -> None:
    value = parse_nonnegative_int(manifest.get(key, ""), f"{source}:{key}")
    if value != 0:
        raise CalibrationError(f"{source}: {key} must be zero")


def require_equal(
    manifests: list[tuple[str, Path, dict[str, str]]], key: str
) -> str:
    values = [(label, manifest.get(key, "")) for label, _, manifest in manifests]
    if any(not value for _, value in values):
        missing = ", ".join(label for label, value in values if not value)
        raise CalibrationError(f"missing {key} in: {missing}")
    if len({value for _, value in values}) != 1:
        details = ", ".join(f"{label}={value}" for label, value in values)
        raise CalibrationError(f"exports disagree on {key}: {details}")
    return values[0][1]


def yaml_quote(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


def write_report(
    output: Path,
    serial: str,
    infrared_profile: str,
    gyro_rate_hz: str,
    accelerometer_rate_hz: str,
    gyro_sensitivity: str | None,
    gyro_scale_factor: str | None,
    target_sha256: str,
    global_time_enabled: bool,
    manifests: list[tuple[str, Path, dict[str, str]]],
) -> None:
    if output.exists():
        raise CalibrationError(f"refusing to overwrite report: {output}")
    if not output.parent.is_dir():
        raise CalibrationError(
            f"report parent directory does not exist: {output.parent}"
        )
    hashes = {
        f"{label}_manifest_sha256": sha256_file(path)
        for label, path, _ in manifests
    }
    lines = [
        "%YAML:1.0",
        'format: "ovrs-calibration-export-set-v1"',
        "complete: true",
        'calibration_state: "UNVERIFIED_EXPORT_SET"',
        f"calibrated_serial: {yaml_quote(serial)}",
        f"infrared_profile: {yaml_quote(infrared_profile)}",
        f"gyro_rate_hz: {gyro_rate_hz}",
        f"accelerometer_rate_hz: {accelerometer_rate_hz}",
        "motion_correction_active: true",
        f"global_time_enabled: {str(global_time_enabled).lower()}",
        f"source_calibration_target_sha256: {yaml_quote(target_sha256)}",
    ]
    if gyro_sensitivity is not None:
        lines.append(f"gyro_sensitivity: {gyro_sensitivity}")
    if gyro_scale_factor is not None:
        lines.append(f"gyro_scale_factor: {gyro_scale_factor}")
    lines.extend(f"{key}: {yaml_quote(value)}" for key, value in hashes.items())
    try:
        with output.open("x", encoding="utf-8", newline="\n") as handle:
            handle.write("\n".join(lines) + "\n")
    except OSError as exc:
        raise CalibrationError(f"cannot write report {output}: {exc}") from exc


def main() -> int:
    args = parser().parse_args()
    try:
        allan_path, allan = load_manifest(args.allan_export, "imu-allan")
        stereo_path, stereo = load_manifest(
            args.stereo_export, "stereo-calibration"
        )
        imucam_path, imucam = load_manifest(
            args.imu_camera_export, "imu-camera-calibration"
        )
        manifests = [
            ("allan", allan_path, allan),
            ("stereo", stereo_path, stereo),
            ("imu_camera", imucam_path, imucam),
        ]
        timestamp_policies = {
            label: export_global_time_enabled(path, manifest)
            for label, path, manifest in manifests
        }
        if len(set(timestamp_policies.values())) != 1:
            details = ", ".join(
                f"{label}={'Global Time' if enabled else 'Hardware Clock'}"
                for label, enabled in timestamp_policies.items()
            )
            raise CalibrationError(
                f"exports disagree on timestamp policy: {details}"
            )
        global_time_enabled = next(iter(timestamp_policies.values()))

        serial = require_equal(manifests, "calibrated_serial")
        infrared_profile = require_equal(
            [
                ("stereo", stereo_path, stereo),
                ("imu_camera", imucam_path, imucam),
            ],
            "infrared_profile",
        )
        if infrared_profile == "disabled":
            raise CalibrationError("camera exports have a disabled IR profile")
        if allan.get("infrared_profile") != "disabled":
            raise CalibrationError("Allan export must have infrared disabled")

        require_zero(allan, "camera_rows_per_camera", allan_path)
        require_positive(allan, "synchronized_imu_rows", allan_path)
        require_positive(stereo, "camera_rows_per_camera", stereo_path)
        require_zero(stereo, "synchronized_imu_rows", stereo_path)
        require_positive(imucam, "camera_rows_per_camera", imucam_path)
        require_positive(imucam, "synchronized_imu_rows", imucam_path)

        for label, path, manifest in (
            ("allan", allan_path, allan),
            ("imu_camera", imucam_path, imucam),
        ):
            if not parse_bool(
                manifest.get("motion_correction_active", ""),
                f"{path}:motion_correction_active",
            ):
                raise CalibrationError(
                    f"{label} export requires active motion correction"
                )

        gyro_rate = require_equal(
            [
                ("allan", allan_path, allan),
                ("imu_camera", imucam_path, imucam),
            ],
            "gyro_rate_hz",
        )
        accel_rate = require_equal(
            [
                ("allan", allan_path, allan),
                ("imu_camera", imucam_path, imucam),
            ],
            "accelerometer_rate_hz",
        )
        require_positive(allan, "gyro_rate_hz", allan_path)
        require_positive(allan, "accelerometer_rate_hz", allan_path)

        sensitivity_values = [
            ("allan", allan.get("gyro_sensitivity", "")),
            ("imu_camera", imucam.get("gyro_sensitivity", "")),
        ]
        if any(value for _, value in sensitivity_values):
            if any(not value for _, value in sensitivity_values):
                missing = ", ".join(
                    label for label, value in sensitivity_values if not value
                )
                raise CalibrationError(
                    f"missing gyro_sensitivity in: {missing}"
                )
            gyro_sensitivity = require_equal(
                [
                    ("allan", allan_path, allan),
                    ("imu_camera", imucam_path, imucam),
                ],
                "gyro_sensitivity",
            )
            parsed_sensitivity = parse_nonnegative_int(
                gyro_sensitivity, "gyro_sensitivity"
            )
            if parsed_sensitivity > 4:
                raise CalibrationError(
                    "gyro_sensitivity must be an index in [0,4]"
                )
        else:
            gyro_sensitivity = None

        scale_values = [
            ("allan", allan.get("gyro_scale_factor", "")),
            ("imu_camera", imucam.get("gyro_scale_factor", "")),
        ]
        if any(value for _, value in scale_values):
            if any(not value for _, value in scale_values):
                missing = ", ".join(
                    label for label, value in scale_values if not value
                )
                raise CalibrationError(
                    f"missing gyro_scale_factor in: {missing}"
                )
            gyro_scale_factor = require_equal(
                [
                    ("allan", allan_path, allan),
                    ("imu_camera", imucam_path, imucam),
                ],
                "gyro_scale_factor",
            )
            parsed_scale = finite_decimal(
                gyro_scale_factor, "gyro_scale_factor"
            )
            if parsed_scale <= 0 or parsed_scale > 100:
                raise CalibrationError(
                    "gyro_scale_factor must be in (0,100]"
                )
        else:
            gyro_scale_factor = None

        target_sha = require_equal(
            [
                ("stereo", stereo_path, stereo),
                ("imu_camera", imucam_path, imucam),
            ],
            "source_calibration_target_sha256",
        )
        write_report(
            args.output_report,
            serial,
            infrared_profile,
            gyro_rate,
            accel_rate,
            gyro_sensitivity,
            gyro_scale_factor,
            target_sha,
            global_time_enabled,
            manifests,
        )
        print("calibration export set: PASS")
        print(f"serial: {serial}")
        print(f"infrared_profile: {infrared_profile}")
        print(
            "timestamp_policy: "
            + ("Global Time" if global_time_enabled else "Hardware Clock")
        )
        print(f"report: {args.output_report}")
        return 0
    except (CalibrationError, OSError) as exc:
        print(f"calibration export set: FAIL: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
