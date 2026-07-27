#!/usr/bin/env python3
"""Shared fail-closed helpers for repository-local calibration tooling."""

from __future__ import annotations

import csv
import hashlib
import math
import re
import struct
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation, ROUND_HALF_EVEN
from itertools import zip_longest
from pathlib import Path
from typing import Iterable, Iterator


class CalibrationError(RuntimeError):
    """Raised when calibration input cannot be trusted."""


CALIBRATION_MODES = {
    "imu-allan",
    "stereo-calibration",
    "imu-camera-calibration",
}


@dataclass(frozen=True)
class CaptureInfo:
    root: Path
    mode: str
    serial: str
    stereo_enabled: bool
    motion_enabled: bool
    synchronized_imu_enabled: bool
    camera_rows: int
    synchronized_imu_rows: int
    gyro_rate_hz: int
    accelerometer_rate_hz: int
    motion_correction_active: bool
    infrared_profile: str
    recording_duration_s: Decimal
    calibration_target: Path | None


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except OSError as exc:
        raise CalibrationError(f"cannot read {path}: {exc}") from exc


def simple_yaml_map(path: Path) -> dict[str, str]:
    """Parse the flat scalar subset used by OVRS metadata and summaries."""
    result: dict[str, str] = {}
    for line_number, raw_line in enumerate(read_text(path).splitlines(), 1):
        # This intentionally is not a general YAML parser. Consumers only use
        # top-level capture-contract scalars; nested RealSense diagnostic maps
        # may legitimately repeat child keys under different parents.
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
            raise CalibrationError(
                f"{path}:{line_number}: duplicate scalar key {key}"
            )
        value = value.split("#", 1)[0].strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "\"'":
            value = value[1:-1]
        result[key] = value
    return result


def require_scalar(values: dict[str, str], key: str, source: Path) -> str:
    value = values.get(key, "")
    if not value:
        raise CalibrationError(f"{source} is missing nonempty {key}")
    return value


def parse_bool(value: str, field: str) -> bool:
    if value == "true":
        return True
    if value == "false":
        return False
    raise CalibrationError(f"{field} must be true or false")


def parse_nonnegative_int(value: str, field: str) -> int:
    if not re.fullmatch(r"[0-9]+", value):
        raise CalibrationError(f"{field} must be a nonnegative integer")
    return int(value)


def finite_decimal(value: str, field: str) -> Decimal:
    try:
        parsed = Decimal(value)
    except InvalidOperation as exc:
        raise CalibrationError(f"{field} is not numeric") from exc
    if not parsed.is_finite():
        raise CalibrationError(f"{field} must be finite")
    return parsed


def seconds_to_nanoseconds(value: str, field: str) -> int:
    scaled = finite_decimal(value, field) * Decimal(1_000_000_000)
    return int(scaled.to_integral_value(rounding=ROUND_HALF_EVEN))


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise CalibrationError(f"cannot hash {path}: {exc}") from exc
    return digest.hexdigest()


def validate_export_provenance(
    manifest_path: Path, manifest: dict[str, str]
) -> None:
    if manifest.get("provenance_layout") != "ovrs-export-provenance-v1":
        raise CalibrationError(
            f"{manifest_path}: unsupported or missing provenance_layout"
        )
    common_files = {
        "source_dataset_metadata_sha256": (
            "ovrs_metadata/source_dataset_metadata.yaml"
        ),
        "source_recording_summary_sha256": (
            "ovrs_metadata/source_recording_summary.yaml"
        ),
        "source_device_report_sha256": (
            "ovrs_metadata/source_device_report.yaml"
        ),
        "source_resolved_stream_config_sha256": (
            "ovrs_metadata/source_resolved_stream_config.yaml"
        ),
    }
    mode = manifest.get("capture_mode", "")
    if mode in {"stereo-calibration", "imu-camera-calibration"}:
        common_files["source_calibration_target_sha256"] = (
            "ovrs_metadata/source_calibration_target.yaml"
        )
        common_files["staged_calibration_target_sha256"] = "target.yaml"
    for key, relative in common_files.items():
        expected = manifest.get(key, "")
        if not re.fullmatch(r"[0-9a-f]{64}", expected):
            raise CalibrationError(
                f"{manifest_path}: {key} must be a lowercase SHA-256"
            )
        source = manifest_path.parent / relative
        actual = sha256_file(source)
        if actual != expected:
            raise CalibrationError(
                f"{manifest_path}: provenance hash mismatch for {relative}"
            )
    if mode in {"stereo-calibration", "imu-camera-calibration"}:
        source_target_path = (
            manifest_path.parent
            / "ovrs_metadata"
            / "source_calibration_target.yaml"
        )
        staged_target_path = manifest_path.parent / "target.yaml"
        source_target = simple_yaml_map(source_target_path)
        staged_target = simple_yaml_map(staged_target_path)
        for key in (
            "target_type",
            "tagRows",
            "tagCols",
            "tagSize",
            "tagSpacing",
        ):
            if source_target.get(key, "") != staged_target.get(key, ""):
                raise CalibrationError(
                    f"{manifest_path}: staged target {key} does not match "
                    f"{source_target_path}"
                )
        try:
            first_line = staged_target_path.read_text(
                encoding="utf-8"
            ).splitlines()[0]
        except (OSError, IndexError) as exc:
            raise CalibrationError(
                f"cannot read staged target {staged_target_path}"
            ) from exc
        if first_line.startswith("%YAML:"):
            raise CalibrationError(
                f"{staged_target_path}: OpenCV YAML directive is not "
                "accepted by the pinned Kalibr/PyYAML toolchain"
            )


def iter_csv_rows(
    path: Path,
    required_fields: Iterable[str],
    timestamp_field: str = "timestamp_s",
) -> Iterator[dict[str, str]]:
    """Yield validated CSV rows without retaining the complete file in RAM."""
    try:
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            fields = reader.fieldnames or []
            missing = [field for field in required_fields if field not in fields]
            if missing:
                raise CalibrationError(
                    f"{path} is missing CSV fields: {', '.join(missing)}"
                )
            previous: Decimal | None = None
            row_count = 0
            for index, row in enumerate(reader, 2):
                timestamp = finite_decimal(
                    row.get(timestamp_field, ""),
                    f"{path}:{index}:{timestamp_field}",
                )
                if previous is not None and timestamp <= previous:
                    raise CalibrationError(
                        f"{path}:{index}: timestamps are not strictly increasing"
                    )
                previous = timestamp
                for field in required_fields:
                    value = row.get(field, "")
                    if field not in {"file", "frameset_number"}:
                        finite_decimal(value, f"{path}:{index}:{field}")
                row_count += 1
                yield row
            if row_count == 0:
                raise CalibrationError(f"{path} has no measurement rows")
    except (OSError, csv.Error) as exc:
        raise CalibrationError(f"cannot read {path}: {exc}") from exc


def read_csv_rows(
    path: Path,
    required_fields: Iterable[str],
    timestamp_field: str = "timestamp_s",
) -> list[dict[str, str]]:
    """Compatibility helper for callers that explicitly need retained rows."""
    return list(iter_csv_rows(path, required_fields, timestamp_field))


def count_csv_rows(
    path: Path,
    required_fields: Iterable[str],
    timestamp_field: str = "timestamp_s",
) -> int:
    """Validate and count a CSV using constant memory."""
    return sum(
        1 for _ in iter_csv_rows(path, required_fields, timestamp_field)
    )


def png_dimensions(path: Path) -> tuple[int, int]:
    try:
        with path.open("rb") as handle:
            header = handle.read(24)
    except OSError as exc:
        raise CalibrationError(f"cannot read image {path}: {exc}") from exc
    if len(header) != 24 or header[:8] != b"\x89PNG\r\n\x1a\n":
        raise CalibrationError(f"{path} is not a valid PNG header")
    if header[12:16] != b"IHDR":
        raise CalibrationError(f"{path} has no PNG IHDR header")
    width, height = struct.unpack(">II", header[16:24])
    if width <= 0 or height <= 0:
        raise CalibrationError(f"{path} has invalid PNG dimensions")
    return width, height


def _required_zero(summary: dict[str, str], key: str, source: Path) -> None:
    value = parse_nonnegative_int(require_scalar(summary, key, source), key)
    if value != 0:
        raise CalibrationError(f"{source}: {key} must be zero, found {value}")


def _required_count(
    summary: dict[str, str],
    key: str,
    expected: int,
    source: Path,
) -> None:
    value = parse_nonnegative_int(require_scalar(summary, key, source), key)
    if value != expected:
        raise CalibrationError(
            f"{source}: {key} reports {value}, but CSV contains {expected} rows"
        )


def validate_capture(root: Path) -> CaptureInfo:
    root = root.resolve()
    if not root.is_dir():
        raise CalibrationError(f"capture directory does not exist: {root}")
    if (root / "INCOMPLETE").exists():
        raise CalibrationError(f"capture is marked INCOMPLETE: {root}")

    metadata_path = root / "dataset_metadata.yaml"
    summary_path = root / "recording_summary.yaml"
    report_path = root / "device_report.yaml"
    stream_path = root / "resolved_stream_config.yaml"
    metadata = simple_yaml_map(metadata_path)
    summary = simple_yaml_map(summary_path)
    report = simple_yaml_map(report_path)
    simple_yaml_map(stream_path)

    if require_scalar(metadata, "format", metadata_path) != (
        "ovrs-calibration-capture-v1"
    ):
        raise CalibrationError(
            "input is not an ovrs-calibration-capture-v1 capture"
        )
    if not parse_bool(
        require_scalar(metadata, "complete", metadata_path), "complete"
    ):
        raise CalibrationError("capture metadata does not declare complete: true")
    if parse_bool(
        require_scalar(metadata, "replay_compatible", metadata_path),
        "replay_compatible",
    ):
        raise CalibrationError("calibration capture must not be replay-compatible")

    mode = require_scalar(metadata, "capture_mode", metadata_path)
    if mode not in CALIBRATION_MODES:
        raise CalibrationError(f"unsupported calibration capture mode: {mode}")
    serial = require_scalar(metadata, "calibrated_serial", metadata_path)
    if not re.fullmatch(r"[0-9]+", serial):
        raise CalibrationError("calibrated_serial must contain digits only")
    if require_scalar(report, "serial", report_path) != serial:
        raise CalibrationError("device report and capture serial do not match")

    stereo_enabled = mode in {
        "stereo-calibration",
        "imu-camera-calibration",
    }
    motion_enabled = mode in {"imu-allan", "imu-camera-calibration"}
    synchronized_imu_enabled = motion_enabled
    if parse_bool(
        require_scalar(report, "stereo_stream_enabled", report_path),
        "stereo_stream_enabled",
    ) != stereo_enabled:
        raise CalibrationError("device report stereo selection conflicts with mode")
    if parse_bool(
        require_scalar(report, "motion_streams_enabled", report_path),
        "motion_streams_enabled",
    ) != motion_enabled:
        raise CalibrationError("device report motion selection conflicts with mode")

    for key in (
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
    ):
        _required_zero(summary, key, summary_path)
    if stereo_enabled:
        _required_zero(summary, "dropped_camera_frames", summary_path)

    camera_rows = 0
    if stereo_enabled:
        camera_fields = (
            "timestamp_s",
            "raw_timestamp_ms",
            "frameset_number",
            "file",
        )
        left = iter_csv_rows(root / "cam0" / "data.csv", camera_fields)
        right = iter_csv_rows(root / "cam1" / "data.csv", camera_fields)
        for index, pair in enumerate(
            zip_longest(left, right, fillvalue=None), 2
        ):
            left_row, right_row = pair
            if left_row is None or right_row is None:
                raise CalibrationError("cam0 and cam1 row counts differ")
            if left_row["frameset_number"] != right_row["frameset_number"]:
                raise CalibrationError(
                    f"camera frameset mismatch at CSV row {index}"
                )
            for camera, row in (("cam0", left_row), ("cam1", right_row)):
                filename = row["file"]
                if Path(filename).name != filename:
                    raise CalibrationError(
                        f"{camera} row {index} contains a nonlocal image path"
                    )
                png_dimensions(root / camera / "data" / filename)
            camera_rows += 1
        _required_count(
            summary, "received_framesets", camera_rows, summary_path
        )
        _required_count(
            summary, "valid_stereo_pairs", camera_rows, summary_path
        )
    else:
        _required_zero(summary, "received_framesets", summary_path)
        _required_zero(summary, "valid_stereo_pairs", summary_path)

    synchronized_imu_rows = 0
    if motion_enabled:
        gyro_rows = count_csv_rows(
            root / "imu" / "gyro.csv",
            (
                "timestamp_s",
                "raw_timestamp_ms",
                "wx_rad_s",
                "wy_rad_s",
                "wz_rad_s",
            ),
        )
        accelerometer_rows = count_csv_rows(
            root / "imu" / "accelerometer.csv",
            (
                "timestamp_s",
                "raw_timestamp_ms",
                "ax_m_s2",
                "ay_m_s2",
                "az_m_s2",
            ),
        )
        synchronized_imu_rows = count_csv_rows(
            root / "imu" / "synchronized.csv",
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
        _required_count(summary, "received_gyro", gyro_rows, summary_path)
        _required_count(
            summary,
            "received_accelerometer",
            accelerometer_rows,
            summary_path,
        )
        _required_count(
            summary,
            "synchronized_imu",
            synchronized_imu_rows,
            summary_path,
        )
    else:
        _required_zero(summary, "received_gyro", summary_path)
        _required_zero(summary, "received_accelerometer", summary_path)
        _required_zero(summary, "synchronized_imu", summary_path)

    if mode == "imu-allan":
        if not parse_bool(
            require_scalar(
                metadata, "operator_confirmed_stationary", metadata_path
            ),
            "operator_confirmed_stationary",
        ):
            raise CalibrationError(
                "imu-allan capture lacks operator stationary confirmation"
            )

    gyro_rate_hz = parse_nonnegative_int(
        require_scalar(metadata, "gyro_rate_hz", metadata_path),
        "gyro_rate_hz",
    )
    accelerometer_rate_hz = parse_nonnegative_int(
        require_scalar(metadata, "accelerometer_rate_hz", metadata_path),
        "accelerometer_rate_hz",
    )
    motion_correction_active = parse_bool(
        require_scalar(metadata, "motion_correction_active", metadata_path),
        "motion_correction_active",
    )
    recording_duration_s = finite_decimal(
        require_scalar(summary, "recording_duration_s", summary_path),
        "recording_duration_s",
    )
    if recording_duration_s <= 0:
        raise CalibrationError("recording_duration_s must be positive")
    if motion_enabled and (gyro_rate_hz <= 0 or accelerometer_rate_hz <= 0):
        raise CalibrationError("enabled motion streams require positive rates")
    if not motion_enabled and (gyro_rate_hz != 0 or accelerometer_rate_hz != 0):
        raise CalibrationError("disabled motion streams must report zero rates")
    infrared_profile = require_scalar(
        metadata, "infrared_profile", metadata_path
    )
    if stereo_enabled and infrared_profile == "disabled":
        raise CalibrationError("enabled stereo stream reports disabled profile")
    if not stereo_enabled and infrared_profile != "disabled":
        raise CalibrationError("disabled stereo stream reports an active profile")
    target_present = parse_bool(
        require_scalar(metadata, "calibration_target_present", metadata_path),
        "calibration_target_present",
    )
    if target_present != stereo_enabled:
        raise CalibrationError(
            "calibration target presence conflicts with capture mode"
        )
    calibration_target: Path | None = None
    if target_present:
        calibration_target = root / "calibration_target.yaml"
        target = simple_yaml_map(calibration_target)
        if require_scalar(
            target, "target_type", calibration_target
        ) != "aprilgrid":
            raise CalibrationError("calibration target must be an AprilGrid")
        if parse_nonnegative_int(
            require_scalar(target, "tagRows", calibration_target), "tagRows"
        ) <= 0 or parse_nonnegative_int(
            require_scalar(target, "tagCols", calibration_target), "tagCols"
        ) <= 0:
            raise CalibrationError("AprilGrid rows and columns must be positive")
        if finite_decimal(
            require_scalar(target, "tagSize", calibration_target), "tagSize"
        ) <= 0 or finite_decimal(
            require_scalar(target, "tagSpacing", calibration_target),
            "tagSpacing",
        ) <= 0:
            raise CalibrationError(
                "AprilGrid tag size and spacing ratio must be positive"
            )

    return CaptureInfo(
        root=root,
        mode=mode,
        serial=serial,
        stereo_enabled=stereo_enabled,
        motion_enabled=motion_enabled,
        synchronized_imu_enabled=synchronized_imu_enabled,
        camera_rows=camera_rows,
        synchronized_imu_rows=synchronized_imu_rows,
        gyro_rate_hz=gyro_rate_hz,
        accelerometer_rate_hz=accelerometer_rate_hz,
        motion_correction_active=motion_correction_active,
        infrared_profile=infrared_profile,
        recording_duration_s=recording_duration_s,
        calibration_target=calibration_target,
    )


def finite_float(value: object, field: str) -> float:
    if isinstance(value, bool):
        raise CalibrationError(f"{field} must be numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise CalibrationError(f"{field} must be numeric") from exc
    if not math.isfinite(result):
        raise CalibrationError(f"{field} must be finite")
    return result
