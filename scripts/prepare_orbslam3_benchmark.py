#!/usr/bin/env python3
"""Prepare a neutral OVRS benchmark for the pinned ORB-SLAM3 EuRoC runner."""

from __future__ import annotations

import argparse
import ast
import csv
import math
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation, ROUND_HALF_EVEN
from pathlib import Path
from typing import Iterable

from export_vislam_benchmark import (
    BenchmarkError,
    sha256_file,
    simple_yaml_map,
    yaml_quote,
)


EXPECTED_CAMERA_HEADER = ("#timestamp [ns]", "filename")
EXPECTED_IMU_HEADER = (
    "#timestamp [ns]",
    "w_RS_S_x [rad s^-1]",
    "w_RS_S_y [rad s^-1]",
    "w_RS_S_z [rad s^-1]",
    "a_RS_S_x [m s^-2]",
    "a_RS_S_y [m s^-2]",
    "a_RS_S_z [m s^-2]",
)


@dataclass(frozen=True)
class CameraCalibration:
    transform_imu_camera: tuple[tuple[float, ...], ...]
    intrinsics: tuple[float, ...]
    distortion: tuple[float, ...]
    resolution: tuple[int, int]
    time_offset_s: Decimal


@dataclass(frozen=True)
class AdaptedRange:
    source_rows: int
    rows: int
    skipped_stride_rows: int
    skipped_leading_rows: int
    skipped_trailing_rows: int
    first_timestamp_ns: int
    last_timestamp_ns: int


def atlas_basename(value: str) -> str:
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", value):
        raise argparse.ArgumentTypeError(
            "atlas name must be 1-64 letters, digits, underscores, or hyphens"
        )
    return value


def parse_decimal(value: str, field: str) -> Decimal:
    try:
        parsed = Decimal(value)
    except InvalidOperation as exc:
        raise BenchmarkError(f"{field} is not numeric") from exc
    if not parsed.is_finite():
        raise BenchmarkError(f"{field} is not finite")
    return parsed


def parse_int(value: str, field: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise BenchmarkError(f"{field} is not an integer") from exc
    if str(parsed) != value.strip() or parsed < 0:
        raise BenchmarkError(f"{field} must be a nonnegative integer")
    return parsed


def decimal_seconds_to_ns(value: Decimal, field: str) -> int:
    result = int(
        (value * Decimal(1_000_000_000)).to_integral_value(
            rounding=ROUND_HALF_EVEN
        )
    )
    if result < 0:
        raise BenchmarkError(f"{field} produces a negative timestamp")
    return result


def strip_comment(value: str) -> str:
    return value.split("#", 1)[0].strip()


def parse_literal(value: str, field: str) -> object:
    try:
        return ast.literal_eval(strip_comment(value))
    except (SyntaxError, ValueError) as exc:
        raise BenchmarkError(f"{field} has invalid YAML-list syntax") from exc


def section_lines(path: Path, name: str) -> list[tuple[int, str]]:
    lines = path.read_text(encoding="utf-8").splitlines()
    start: int | None = None
    result: list[tuple[int, str]] = []
    for index, line in enumerate(lines, 1):
        if line == f"{name}:":
            if start is not None:
                raise BenchmarkError(f"{path}: duplicate section {name}")
            start = index
            continue
        if start is None:
            continue
        if line and not line[0].isspace() and not line.lstrip().startswith("#"):
            break
        result.append((index, line))
    if start is None:
        raise BenchmarkError(f"{path}: missing section {name}")
    return result


def nested_scalars(path: Path, name: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for line_number, line in section_lines(path, name):
        if not line.startswith("  ") or line.startswith("    "):
            continue
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or ":" not in stripped:
            continue
        key, value = stripped.split(":", 1)
        value = strip_comment(value)
        if not value:
            continue
        if key in result:
            raise BenchmarkError(
                f"{path}:{line_number}: duplicate {name}.{key}"
            )
        if (
            len(value) >= 2
            and value[0] == value[-1]
            and value[0] in "\"'"
        ):
            value = value[1:-1]
        result[key] = value
    return result


def finite_float(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise BenchmarkError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise BenchmarkError(f"{field} must be finite")
    return result


def numeric_tuple(
    value: object, length: int, field: str
) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise BenchmarkError(f"{field} must contain {length} values")
    return tuple(finite_float(item, field) for item in value)


def validate_rigid_transform(
    rows: Iterable[Iterable[float]], field: str
) -> tuple[tuple[float, ...], ...]:
    matrix = tuple(tuple(row) for row in rows)
    if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        raise BenchmarkError(f"{field} must be 4x4")
    if any(not math.isfinite(value) for row in matrix for value in row):
        raise BenchmarkError(f"{field} contains a non-finite value")
    for actual, expected in zip(matrix[3], (0.0, 0.0, 0.0, 1.0)):
        if abs(actual - expected) > 1e-9:
            raise BenchmarkError(f"{field} has an invalid homogeneous row")
    rotation = tuple(row[:3] for row in matrix[:3])
    for row in range(3):
        for column in range(3):
            dot = sum(
                rotation[row][index] * rotation[column][index]
                for index in range(3)
            )
            expected = 1.0 if row == column else 0.0
            if abs(dot - expected) > 1e-5:
                raise BenchmarkError(f"{field} rotation is not orthonormal")
    determinant = (
        rotation[0][0]
        * (
            rotation[1][1] * rotation[2][2]
            - rotation[1][2] * rotation[2][1]
        )
        - rotation[0][1]
        * (
            rotation[1][0] * rotation[2][2]
            - rotation[1][2] * rotation[2][0]
        )
        + rotation[0][2]
        * (
            rotation[1][0] * rotation[2][1]
            - rotation[1][1] * rotation[2][0]
        )
    )
    if abs(determinant - 1.0) > 1e-5:
        raise BenchmarkError(f"{field} rotation determinant is not +1")
    return matrix


def parse_camera_calibration(path: Path, camera: str) -> CameraCalibration:
    lines = section_lines(path, camera)
    scalars = nested_scalars(path, camera)
    transform_rows: list[tuple[float, ...]] = []
    reading_transform = False
    for line_number, line in lines:
        if line.strip().startswith("T_imu_cam:"):
            if reading_transform or transform_rows:
                raise BenchmarkError(
                    f"{path}:{line_number}: duplicate {camera}.T_imu_cam"
                )
            reading_transform = True
            continue
        if reading_transform and line.startswith("    - "):
            row = numeric_tuple(
                parse_literal(line.strip()[2:], f"{path}:{line_number}"),
                4,
                f"{path}:{line_number}",
            )
            transform_rows.append(row)
            if len(transform_rows) == 4:
                reading_transform = False
    transform = validate_rigid_transform(
        transform_rows, f"{path}:{camera}.T_imu_cam"
    )
    if scalars.get("camera_model") != "pinhole":
        raise BenchmarkError(f"{path}:{camera}.camera_model must be pinhole")
    if scalars.get("distortion_model") != "radtan":
        raise BenchmarkError(f"{path}:{camera}.distortion_model must be radtan")
    required = (
        "intrinsics",
        "distortion_coeffs",
        "resolution",
        "timeshift_cam_imu",
    )
    missing = [key for key in required if key not in scalars]
    if missing:
        raise BenchmarkError(f"{path}:{camera} missing {', '.join(missing)}")
    intrinsics = numeric_tuple(
        parse_literal(scalars["intrinsics"], f"{path}:{camera}.intrinsics"),
        4,
        f"{path}:{camera}.intrinsics",
    )
    distortion = numeric_tuple(
        parse_literal(
            scalars["distortion_coeffs"],
            f"{path}:{camera}.distortion_coeffs",
        ),
        4,
        f"{path}:{camera}.distortion_coeffs",
    )
    resolution_values = parse_literal(
        scalars["resolution"], f"{path}:{camera}.resolution"
    )
    if (
        not isinstance(resolution_values, (list, tuple))
        or len(resolution_values) != 2
        or any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or value <= 0
            for value in resolution_values
        )
    ):
        raise BenchmarkError(
            f"{path}:{camera}.resolution must contain two positive integers"
        )
    time_offset = parse_decimal(
        scalars["timeshift_cam_imu"],
        f"{path}:{camera}.timeshift_cam_imu",
    )
    return CameraCalibration(
        transform,
        intrinsics,
        distortion,
        (resolution_values[0], resolution_values[1]),
        time_offset,
    )


def invert_rigid(
    matrix: tuple[tuple[float, ...], ...]
) -> tuple[tuple[float, ...], ...]:
    rotation_t = tuple(
        tuple(matrix[column][row] for column in range(3))
        for row in range(3)
    )
    translation = tuple(matrix[row][3] for row in range(3))
    inverse_translation = tuple(
        -sum(
            rotation_t[row][column] * translation[column]
            for column in range(3)
        )
        for row in range(3)
    )
    return tuple(
        tuple((*rotation_t[row], inverse_translation[row]))
        for row in range(3)
    ) + ((0.0, 0.0, 0.0, 1.0),)


def matrix_multiply(
    left: tuple[tuple[float, ...], ...],
    right: tuple[tuple[float, ...], ...],
) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(
            sum(
                left[row][index] * right[index][column]
                for index in range(4)
            )
            for column in range(4)
        )
        for row in range(4)
    )


def read_camera_index(path: Path) -> list[tuple[int, str]]:
    result: list[tuple[int, str]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.reader(handle)
        header = tuple(next(reader, ()))
        if header != EXPECTED_CAMERA_HEADER:
            raise BenchmarkError(f"{path}: invalid camera CSV header")
        previous = -1
        for line_number, row in enumerate(reader, 2):
            if len(row) != 2:
                raise BenchmarkError(f"{path}:{line_number}: malformed row")
            timestamp = parse_int(row[0], f"{path}:{line_number}:timestamp")
            if timestamp <= previous:
                raise BenchmarkError(
                    f"{path}:{line_number}: timestamp is not increasing"
                )
            if row[1] != f"{timestamp}.png":
                raise BenchmarkError(
                    f"{path}:{line_number}: filename does not match timestamp"
                )
            previous = timestamp
            result.append((timestamp, row[1]))
    if not result:
        raise BenchmarkError(f"{path}: camera index is empty")
    return result


def validate_imu_csv(path: Path) -> tuple[int, int, int]:
    rows = 0
    first: int | None = None
    previous = -1
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.reader(handle)
        if tuple(next(reader, ())) != EXPECTED_IMU_HEADER:
            raise BenchmarkError(f"{path}: invalid IMU CSV header")
        for line_number, row in enumerate(reader, 2):
            if len(row) != 7:
                raise BenchmarkError(f"{path}:{line_number}: malformed row")
            timestamp = parse_int(row[0], f"{path}:{line_number}:timestamp")
            if timestamp <= previous:
                raise BenchmarkError(
                    f"{path}:{line_number}: timestamp is not increasing"
                )
            try:
                values = tuple(float(value) for value in row[1:])
            except ValueError as exc:
                raise BenchmarkError(
                    f"{path}:{line_number}: nonnumeric IMU value"
                ) from exc
            if any(not math.isfinite(value) for value in values):
                raise BenchmarkError(
                    f"{path}:{line_number}: non-finite IMU value"
                )
            if first is None:
                first = timestamp
            previous = timestamp
            rows += 1
    if rows < 2 or first is None:
        raise BenchmarkError(f"{path}: at least two IMU rows are required")
    return rows, first, previous


def transfer_file(source: Path, destination: Path, mode: str) -> None:
    if mode == "copy":
        shutil.copy2(source, destination)
        return
    try:
        os.link(source, destination)
    except OSError as exc:
        raise BenchmarkError(
            f"cannot hardlink {source} to {destination}; use --file-mode copy"
        ) from exc


def format_matrix(matrix: tuple[tuple[float, ...], ...]) -> str:
    return ",\n         ".join(
        ",".join(f"{value:.17g}" for value in row) for row in matrix
    )


def write_settings(
    path: Path,
    cam0: CameraCalibration,
    cam1: CameraCalibration,
    stereo: tuple[tuple[float, ...], ...],
    imu: dict[str, str],
    fps: int,
    n_features: int,
    initial_fast: int,
    minimum_fast: int,
    imu_init_acceleration_threshold: float,
    atlas_load_name: str | None,
    atlas_save_name: str | None,
    provenance_lines: tuple[str, ...] = (),
) -> None:
    required_imu = (
        "accelerometer_noise_density",
        "accelerometer_random_walk",
        "gyroscope_noise_density",
        "gyroscope_random_walk",
        "update_rate",
    )
    missing = [key for key in required_imu if key not in imu]
    if missing:
        raise BenchmarkError(f"IMU calibration missing {', '.join(missing)}")
    try:
        values = {
            key: finite_float(float(imu[key]), f"IMU calibration {key}")
            for key in required_imu
        }
    except ValueError as exc:
        raise BenchmarkError("IMU calibration contains a nonnumeric value") from exc
    if values["update_rate"] <= 0:
        raise BenchmarkError("IMU update_rate must be positive")
    if abs(values["update_rate"] - round(values["update_rate"])) > 1e-9:
        raise BenchmarkError("IMU update_rate must be an integer frequency")
    baseline = math.sqrt(sum(stereo[row][3] ** 2 for row in range(3)))
    if baseline <= 0.01:
        raise BenchmarkError("stereo baseline is degenerate")
    camera_lines: list[str] = []
    for index, calibration in enumerate((cam0, cam1), 1):
        fx, fy, cx, cy = calibration.intrinsics
        k1, k2, p1, p2 = calibration.distortion
        camera_lines.extend(
            (
                f"Camera{index}.fx: {fx:.17g}",
                f"Camera{index}.fy: {fy:.17g}",
                f"Camera{index}.cx: {cx:.17g}",
                f"Camera{index}.cy: {cy:.17g}",
                f"Camera{index}.k1: {k1:.17g}",
                f"Camera{index}.k2: {k2:.17g}",
                f"Camera{index}.p1: {p1:.17g}",
                f"Camera{index}.p2: {p2:.17g}",
                "",
            )
        )
    atlas_lines: list[str] = []
    if atlas_load_name is not None:
        atlas_lines.append(
            f'System.LoadAtlasFromFile: "{atlas_load_name}"'
        )
    if atlas_save_name is not None:
        atlas_lines.append(
            f'System.SaveAtlasToFile: "{atlas_save_name}"'
        )
    if atlas_lines:
        atlas_lines.append("")

    content = "\n".join(
        (
            "%YAML:1.0",
            "# Generated by prepare_orbslam3_benchmark.py.",
            '# Upstream ORB-SLAM3 "1.0" settings format.',
            'File.version: "1.0"',
            *provenance_lines,
            "",
            *atlas_lines,
            'Camera.type: "PinHole"',
            "",
            *camera_lines,
            f"Camera.width: {cam0.resolution[0]}",
            f"Camera.height: {cam0.resolution[1]}",
            f"Camera.fps: {fps}",
            "Camera.RGB: 0",
            "Stereo.ThDepth: 60.0",
            "Stereo.T_c1_c2: !!opencv-matrix",
            "  rows: 4",
            "  cols: 4",
            "  dt: f",
            f"  data: [{format_matrix(stereo)}]",
            "",
            "# T_imu_cam0 maps left-camera coordinates into the IMU body frame.",
            "IMU.T_b_c1: !!opencv-matrix",
            "  rows: 4",
            "  cols: 4",
            "  dt: f",
            f"  data: [{format_matrix(cam0.transform_imu_camera)}]",
            f"IMU.NoiseGyro: {values['gyroscope_noise_density']:.17g}",
            f"IMU.NoiseAcc: {values['accelerometer_noise_density']:.17g}",
            f"IMU.GyroWalk: {values['gyroscope_random_walk']:.17g}",
            f"IMU.AccWalk: {values['accelerometer_random_walk']:.17g}",
            f"IMU.Frequency: {int(round(values['update_rate']))}.0",
            "IMU.InitAccelerationThreshold: "
            f"{imu_init_acceleration_threshold:.17g}",
            "",
            "# Untuned upstream EuRoC extractor baseline.",
            f"ORBextractor.nFeatures: {n_features}",
            "ORBextractor.scaleFactor: 1.2",
            "ORBextractor.nLevels: 8",
            f"ORBextractor.iniThFAST: {initial_fast}",
            f"ORBextractor.minThFAST: {minimum_fast}",
            "",
            "Viewer.KeyFrameSize: 0.05",
            "Viewer.KeyFrameLineWidth: 1.0",
            "Viewer.GraphLineWidth: 0.9",
            "Viewer.PointSize: 2.0",
            "Viewer.CameraSize: 0.08",
            "Viewer.CameraLineWidth: 3.0",
            "Viewer.ViewpointX: 0.0",
            "Viewer.ViewpointY: -0.7",
            "Viewer.ViewpointZ: -1.8",
            "Viewer.ViewpointF: 500.0",
            "Viewer.imageViewScale: 1.0",
            "",
        )
    )
    path.write_text(content, encoding="utf-8")


def repository_state(root: Path) -> tuple[str, str]:
    try:
        commit = subprocess.run(
            ("git", "rev-parse", "HEAD"),
            cwd=root,
            check=True,
            text=True,
            capture_output=True,
        ).stdout.strip()
        status = subprocess.run(
            ("git", "status", "--porcelain"),
            cwd=root,
            check=True,
            text=True,
            capture_output=True,
        ).stdout
    except (OSError, subprocess.CalledProcessError) as exc:
        raise BenchmarkError("cannot determine repository state") from exc
    if len(commit) != 40:
        raise BenchmarkError("repository HEAD is not a full Git commit")
    return commit, "DIRTY" if status else "CLEAN"


def validate_backend_patch(pin: dict[str, str], repository_root: Path) -> str:
    patch_value = pin.get("patch", "")
    relative = Path(patch_value)
    if (
        not patch_value
        or relative.is_absolute()
        or ".." in relative.parts
        or relative.suffix != ".patch"
    ):
        raise BenchmarkError(
            "backend pin patch must be a repository-relative .patch path"
        )
    patch_path = (repository_root / relative).resolve()
    if not patch_path.is_relative_to(repository_root) or not patch_path.is_file():
        raise BenchmarkError(f"backend patch does not exist: {patch_value}")
    expected_hash = pin.get("patch_sha256", "")
    if not re.fullmatch(r"[0-9a-f]{64}", expected_hash):
        raise BenchmarkError(
            "backend pin patch_sha256 must be a lowercase SHA-256"
        )
    actual_hash = sha256_file(patch_path)
    if actual_hash != expected_hash:
        raise BenchmarkError(
            f"backend patch hash differs: expected {expected_hash}, "
            f"found {actual_hash}"
        )
    return actual_hash


def validate_source_atlas_manifest(
    manifest_path: Path,
    atlas_path: Path,
    *,
    serial: str,
    calibration_state: str,
    backend_commit: str,
    backend_patch_hash: str,
    backend_pin_hash: str,
    imucam_hash: str,
    imu_hash: str,
    source_fps: int,
    adapted_fps: int,
    camera_stride: int,
    camera_time_offset_policy: str,
    calibrated_offset_ns: int,
    applied_offset_ns: int,
) -> dict[str, str]:
    if not manifest_path.is_file():
        raise BenchmarkError(
            "input atlas companion manifest is missing: "
            f"{manifest_path}"
        )
    manifest = simple_yaml_map(manifest_path)
    expected = {
        "format": "ovrs-orbslam3-atlas-manifest-v1",
        "state": "TRACKING_GATE_PASS_ATLAS_RELOAD_UNVERIFIED",
        "atlas_file": atlas_path.name,
        "atlas_sha256": sha256_file(atlas_path),
        "backend_name": "ORB_SLAM3",
        "backend_commit": backend_commit,
        "backend_patch_sha256": backend_patch_hash,
        "backend_pin_sha256": backend_pin_hash,
        "camera_serial": serial,
        "calibration_state": calibration_state,
        "imucam_config_sha256": imucam_hash,
        "imu_config_sha256": imu_hash,
        "source_camera_fps": str(source_fps),
        "adapted_camera_fps": str(adapted_fps),
        "camera_stride": str(camera_stride),
        "camera_time_offset_policy": camera_time_offset_policy,
        "calibrated_camera_imu_time_offset_ns": str(calibrated_offset_ns),
        "applied_camera_imu_time_offset_ns": str(applied_offset_ns),
        "coordinate_frame_policy": "ORB_SLAM3_ATLAS_WORLD",
        "ground_truth_consumed_by_estimator": "false",
    }
    for key, value in expected.items():
        if manifest.get(key) != value:
            raise BenchmarkError(
                f"input atlas manifest {key} must be {value}"
            )
    for key in (
        "result_manifest_sha256",
        "backend_runner_sha256",
        "backend_library_sha256",
        "vocabulary_sha256",
    ):
        if not re.fullmatch(r"[0-9a-f]{64}", manifest.get(key, "")):
            raise BenchmarkError(
                f"input atlas manifest {key} must be a lowercase SHA-256"
            )
    return manifest


def prepare(args: argparse.Namespace) -> None:
    benchmark = args.benchmark.resolve()
    output = args.output.resolve()
    imucam_path = args.imucam_config.resolve()
    imu_path = args.imu_config.resolve()
    pin_path = args.backend_pin.resolve()
    atlas_load_path = (
        args.load_atlas.resolve() if args.load_atlas is not None else None
    )
    atlas_load_manifest_path = (
        atlas_load_path.with_suffix(".osa.manifest.yaml")
        if atlas_load_path is not None
        else None
    )
    if output.exists():
        raise BenchmarkError(f"output already exists: {output}")
    for path in (benchmark, imucam_path, imu_path, pin_path):
        if not path.exists():
            raise BenchmarkError(f"required input does not exist: {path}")
    if atlas_load_path is not None:
        if not atlas_load_path.is_file():
            raise BenchmarkError(
                f"input atlas does not exist: {atlas_load_path}"
            )
        if atlas_load_path.suffix != ".osa":
            raise BenchmarkError("input atlas must use the .osa extension")
        if atlas_load_path.stat().st_size == 0:
            raise BenchmarkError("input atlas is empty")
    if not benchmark.is_dir() or (benchmark / "INCOMPLETE").exists():
        raise BenchmarkError("benchmark must be a complete directory")

    manifest_path = benchmark / "benchmark_manifest.yaml"
    manifest = simple_yaml_map(manifest_path)
    expected_manifest = {
        "format": "ovrs-vislam-benchmark-v1",
        "state": "EXPORTED_NOT_EVALUATED",
        "estimation_policy": "MARKERLESS_STEREO_INERTIAL",
        "ground_truth_consumed_by_estimator": "false",
    }
    for key, expected in expected_manifest.items():
        if manifest.get(key) != expected:
            raise BenchmarkError(
                f"benchmark manifest {key} must be {expected}"
            )
    serial = manifest.get("camera_serial", "")
    if not serial:
        raise BenchmarkError("benchmark manifest camera_serial is missing")

    pin = simple_yaml_map(pin_path)
    expected_pin = {
        "format": "ovrs-orbslam3-backend-pin-v1",
        "backend_name": "ORB_SLAM3",
        "license": "GPL-3.0-or-later",
        "runner": "Examples/Stereo-Inertial/stereo_inertial_euroc",
        "vocabulary": "Vocabulary/ORBvoc.txt",
    }
    for key, expected in expected_pin.items():
        if pin.get(key) != expected:
            raise BenchmarkError(f"backend pin {key} must be {expected}")
    repository_root = Path(__file__).resolve().parents[1]
    backend_patch_hash = validate_backend_patch(pin, repository_root)
    imu_init_acceleration_threshold = parse_decimal(
        pin.get("imu_init_acceleration_threshold_m_s2", ""),
        "backend pin imu_init_acceleration_threshold_m_s2",
    )
    if imu_init_acceleration_threshold <= 0:
        raise BenchmarkError(
            "backend pin imu_init_acceleration_threshold_m_s2 must be positive"
        )
    backend_commit = pin.get("commit", "")
    if len(backend_commit) != 40 or any(
        character not in "0123456789abcdef" for character in backend_commit
    ):
        raise BenchmarkError("backend pin commit must be a lowercase SHA-1")

    imucam_root = simple_yaml_map(imucam_path)
    imu_root = simple_yaml_map(imu_path)
    for path, root in ((imucam_path, imucam_root), (imu_path, imu_root)):
        if root.get("calibrated_serial") != serial:
            raise BenchmarkError(f"{path}: calibrated_serial does not match")
    calibration_state = imucam_root.get("calibration_state", "")
    if calibration_state not in (
        "BOOTSTRAP_UNVERIFIED",
        "KALIBR_VERIFIED",
    ):
        raise BenchmarkError(
            "camera calibration_state must be BOOTSTRAP_UNVERIFIED or "
            "KALIBR_VERIFIED"
        )
    if imu_root.get("calibration_state") != calibration_state:
        raise BenchmarkError("camera and IMU calibration states differ")

    cam0 = parse_camera_calibration(imucam_path, "cam0")
    cam1 = parse_camera_calibration(imucam_path, "cam1")
    if cam0.resolution != cam1.resolution:
        raise BenchmarkError("camera calibration resolutions differ")
    if cam0.time_offset_s != cam1.time_offset_s:
        raise BenchmarkError("camera time offsets differ")
    stereo = validate_rigid_transform(
        matrix_multiply(
            invert_rigid(cam0.transform_imu_camera),
            cam1.transform_imu_camera,
        ),
        "derived Stereo.T_c1_c2",
    )

    stream_path = benchmark / "ovrs_metadata" / "resolved_stream_config.yaml"
    if (
        manifest.get("resolved_stream_config_yaml_sha256")
        != sha256_file(stream_path)
    ):
        raise BenchmarkError("resolved stream configuration hash differs")
    stream = simple_yaml_map(stream_path)
    if stream.get("serial") != serial:
        raise BenchmarkError("resolved stream serial differs")
    width = parse_int(stream.get("width", ""), "stream width")
    height = parse_int(stream.get("height", ""), "stream height")
    source_fps = parse_int(
        stream.get("camera_fps", ""), "stream camera_fps"
    )
    if (width, height) != cam0.resolution:
        raise BenchmarkError("stream and calibration resolutions differ")
    if source_fps <= 0:
        raise BenchmarkError("stream camera_fps must be positive")
    if source_fps % args.camera_stride != 0:
        raise BenchmarkError(
            "stream camera_fps must be divisible by --camera-stride"
        )
    adapted_fps = source_fps // args.camera_stride
    gyro_scale = parse_decimal(
        stream.get("gyro_scale_factor", ""), "stream gyro_scale_factor"
    )
    if gyro_scale != Decimal(1):
        raise BenchmarkError(
            "ORB-SLAM3 comparison requires recorded gyro_scale_factor 1"
        )
    if stream.get("motion_correction_enabled") != "true":
        raise BenchmarkError("recording must enable SDK motion correction")
    if stream.get("global_time_enabled") != "true":
        raise BenchmarkError("recording must enable RealSense global time")

    imu_values = nested_scalars(imu_path, "imu0")
    if imu_values.get("imu_intrinsic_method") != (
        "REALSENSE_DEVICE_TABLE_WITH_SDK_CORRECTION"
    ):
        raise BenchmarkError("unexpected IMU intrinsic correction policy")
    if imu_values.get("realsense_motion_correction_enabled") != "true":
        raise BenchmarkError("SDK IMU motion correction must be enabled")
    if imu_values.get("realsense_global_time_enabled") != "true":
        raise BenchmarkError("RealSense global time must be enabled")

    left_index_path = benchmark / "mav0" / "cam0" / "data.csv"
    right_index_path = benchmark / "mav0" / "cam1" / "data.csv"
    imu_csv_path = benchmark / "mav0" / "imu0" / "data.csv"
    for key, path in (
        ("cam0_data_csv_sha256", left_index_path),
        ("cam1_data_csv_sha256", right_index_path),
        ("imu0_data_csv_sha256", imu_csv_path),
    ):
        expected_hash = manifest.get(key)
        if not expected_hash:
            raise BenchmarkError(
                f"benchmark manifest lacks {key}; rerun the v0.6 exporter"
            )
        if expected_hash != sha256_file(path):
            raise BenchmarkError(f"benchmark {key} differs")
    left = read_camera_index(left_index_path)
    right = read_camera_index(right_index_path)
    if left != right:
        raise BenchmarkError("cam0 and cam1 indexes differ")
    imu_rows, first_imu_ns, last_imu_ns = validate_imu_csv(imu_csv_path)
    if imu_rows != parse_int(
        manifest.get("synchronized_imu_rows", ""),
        "manifest synchronized_imu_rows",
    ):
        raise BenchmarkError("manifest and IMU row counts differ")

    calibrated_offset_ns = decimal_seconds_to_ns(
        abs(cam0.time_offset_s), "absolute camera-IMU offset"
    )
    calibrated_signed_offset_ns = (
        -calibrated_offset_ns
        if cam0.time_offset_s < 0
        else calibrated_offset_ns
    )
    applied_offset_ns = (
        calibrated_signed_offset_ns
        if args.camera_time_offset_policy == "calibrated"
        else 0
    )
    atlas_load_manifest: dict[str, str] | None = None
    if atlas_load_path is not None and atlas_load_manifest_path is not None:
        atlas_load_manifest = validate_source_atlas_manifest(
            atlas_load_manifest_path,
            atlas_load_path,
            serial=serial,
            calibration_state=calibration_state,
            backend_commit=backend_commit,
            backend_patch_hash=backend_patch_hash,
            backend_pin_hash=sha256_file(pin_path),
            imucam_hash=sha256_file(imucam_path),
            imu_hash=sha256_file(imu_path),
            source_fps=source_fps,
            adapted_fps=adapted_fps,
            camera_stride=args.camera_stride,
            camera_time_offset_policy=args.camera_time_offset_policy,
            calibrated_offset_ns=calibrated_signed_offset_ns,
            applied_offset_ns=applied_offset_ns,
        )

    output.mkdir(parents=True)
    incomplete = output / "INCOMPLETE"
    incomplete.write_text(
        "ORB-SLAM3 benchmark preparation did not complete.\n",
        encoding="utf-8",
    )
    sequence = output / "sequence" / "mav0"
    left_output = sequence / "cam0" / "data"
    right_output = sequence / "cam1" / "data"
    imu_output = sequence / "imu0"
    left_output.mkdir(parents=True)
    right_output.mkdir(parents=True)
    imu_output.mkdir(parents=True)
    atlas_input_name: str | None = None
    atlas_input_path: Path | None = None
    atlas_input_manifest_path: Path | None = None
    if atlas_load_path is not None:
        atlas_input_name = "input_atlas"
        atlas_input_path = output / f"{atlas_input_name}.osa"
        transfer_file(atlas_load_path, atlas_input_path, args.file_mode)
        if atlas_load_manifest_path is None or atlas_load_manifest is None:
            raise BenchmarkError("input atlas manifest was not validated")
        atlas_input_manifest_path = output / "input_atlas.osa.manifest.yaml"
        transfer_file(
            atlas_load_manifest_path,
            atlas_input_manifest_path,
            args.file_mode,
        )

    adapted: list[tuple[int, str]] = []
    skipped_stride = 0
    skipped_leading = 0
    skipped_trailing = 0
    seen_inside = False
    for source_index, (source_timestamp, source_filename) in enumerate(left):
        if source_index % args.camera_stride != 0:
            skipped_stride += 1
            continue
        adjusted = source_timestamp + applied_offset_ns
        if adjusted <= first_imu_ns:
            if seen_inside:
                raise BenchmarkError("adjusted camera range is not contiguous")
            skipped_leading += 1
            continue
        if adjusted >= last_imu_ns:
            seen_inside = True
            skipped_trailing += 1
            continue
        seen_inside = True
        adjusted_filename = f"{adjusted}.png"
        transfer_file(
            benchmark / "mav0" / "cam0" / "data" / source_filename,
            left_output / adjusted_filename,
            args.file_mode,
        )
        transfer_file(
            benchmark / "mav0" / "cam1" / "data" / source_filename,
            right_output / adjusted_filename,
            args.file_mode,
        )
        adapted.append((adjusted, adjusted_filename))
    if len(adapted) < 2:
        raise BenchmarkError("fewer than two camera pairs remain after offset")

    for camera in ("cam0", "cam1"):
        index_path = sequence / camera / "data.csv"
        with index_path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle, lineterminator="\n")
            writer.writerow(EXPECTED_CAMERA_HEADER)
            writer.writerows(adapted)
    timestamps_path = output / "timestamps.txt"
    timestamps_path.write_text(
        "".join(f"{timestamp}\n" for timestamp, _ in adapted),
        encoding="utf-8",
    )
    transfer_file(imu_csv_path, imu_output / "data.csv", args.file_mode)

    settings_path = output / "orbslam3_settings.yaml"
    write_settings(
        settings_path,
        cam0,
        cam1,
        stereo,
        imu_values,
        adapted_fps,
        args.n_features,
        args.initial_fast_threshold,
        args.minimum_fast_threshold,
        float(imu_init_acceleration_threshold),
        atlas_input_name,
        args.save_atlas_name,
    )
    repo_root = Path(__file__).resolve().parents[1]
    repository_commit, repository_worktree = repository_state(repo_root)
    adapted_range = AdaptedRange(
        len(left),
        len(adapted),
        skipped_stride,
        skipped_leading,
        skipped_trailing,
        adapted[0][0],
        adapted[-1][0],
    )
    adapter_manifest = output / "adapter_manifest.yaml"
    if atlas_load_path is not None:
        atlas_mode = "MULTI_SESSION_MERGE"
    elif args.save_atlas_name is not None:
        atlas_mode = "MAP_BUILD"
    else:
        atlas_mode = "NONE"
    atlas_manifest_lines = [
        f"atlas_mode: {yaml_quote(atlas_mode)}",
        'atlas_localization_policy: "UPSTREAM_MULTI_SESSION_MAP_MERGE"',
        "localization_only_mode_enabled: false",
    ]
    if atlas_input_path is not None:
        if atlas_input_manifest_path is None:
            raise BenchmarkError("staged input atlas manifest is missing")
        atlas_manifest_lines.extend(
            (
                f'atlas_input_file: "{atlas_input_path.name}"',
                (
                    "atlas_input_sha256: "
                    f"{yaml_quote(sha256_file(atlas_input_path))}"
                ),
                (
                    "atlas_input_manifest_file: "
                    f'{yaml_quote(atlas_input_manifest_path.name)}'
                ),
                (
                    "atlas_input_manifest_sha256: "
                    f"{yaml_quote(sha256_file(atlas_input_manifest_path))}"
                ),
            )
        )
    if args.save_atlas_name is not None:
        atlas_manifest_lines.append(
            f'atlas_output_file: "{args.save_atlas_name}.osa"'
        )
    adapter_manifest.write_text(
        "\n".join(
            (
                "%YAML:1.0",
                'format: "ovrs-orbslam3-adapter-v1"',
                'state: "PREPARED_NOT_RUN"',
                'estimation_policy: "MARKERLESS_STEREO_INERTIAL"',
                "ground_truth_consumed_by_estimator: false",
                f"camera_serial: {yaml_quote(serial)}",
                f"calibration_state: {yaml_quote(calibration_state)}",
                f"repository_commit: {yaml_quote(repository_commit)}",
                f"repository_worktree: {yaml_quote(repository_worktree)}",
                f"backend_name: {yaml_quote(pin['backend_name'])}",
                f"backend_commit: {yaml_quote(backend_commit)}",
                f"backend_license: {yaml_quote(pin['license'])}",
                f"backend_patch_sha256: {yaml_quote(backend_patch_hash)}",
                *atlas_manifest_lines,
                f"file_transfer_mode: {yaml_quote(args.file_mode)}",
                'timestamp_adapter: "CAMERA_LABELLED_IN_IMU_CLOCK"',
                'time_offset_convention: "t_imu=t_cam+timeshift_cam_imu"',
                (
                    "camera_time_offset_policy: "
                    f"{yaml_quote(args.camera_time_offset_policy)}"
                ),
                (
                    "calibrated_camera_imu_time_offset_s: "
                    f"{cam0.time_offset_s}"
                ),
                (
                    "calibrated_camera_imu_time_offset_ns: "
                    f"{calibrated_signed_offset_ns}"
                ),
                f"applied_camera_imu_time_offset_ns: {applied_offset_ns}",
                'imu_timestamp_policy: "UNCHANGED"',
                'imu_frame: "GYROSCOPE_BODY_FRAME"',
                'imu_correction_policy: "SDK_DEVICE_TABLE_ALREADY_APPLIED"',
                'imu_transform_contract: "T_b_c1=T_imu_cam0"',
                (
                    'stereo_transform_contract: '
                    '"T_c1_c2=inverse(T_imu_cam0)*T_imu_cam1"'
                ),
                f"source_stereo_pairs: {adapted_range.source_rows}",
                f"adapted_stereo_pairs: {adapted_range.rows}",
                f"camera_stride: {args.camera_stride}",
                (
                    "skipped_pairs_by_camera_stride: "
                    f"{adapted_range.skipped_stride_rows}"
                ),
                (
                    "skipped_leading_pairs_after_time_offset: "
                    f"{adapted_range.skipped_leading_rows}"
                ),
                (
                    "skipped_trailing_pairs_after_time_offset: "
                    f"{adapted_range.skipped_trailing_rows}"
                ),
                (
                    "first_adapted_stereo_timestamp_ns: "
                    f"{adapted_range.first_timestamp_ns}"
                ),
                (
                    "last_adapted_stereo_timestamp_ns: "
                    f"{adapted_range.last_timestamp_ns}"
                ),
                f"imu_rows: {imu_rows}",
                f"first_imu_timestamp_ns: {first_imu_ns}",
                f"last_imu_timestamp_ns: {last_imu_ns}",
                f"source_camera_fps: {source_fps}",
                f"adapted_camera_fps: {adapted_fps}",
                f"orb_n_features: {args.n_features}",
                (
                    "orb_initial_fast_threshold: "
                    f"{args.initial_fast_threshold}"
                ),
                (
                    "orb_minimum_fast_threshold: "
                    f"{args.minimum_fast_threshold}"
                ),
                (
                    "source_benchmark_manifest_sha256: "
                    f"{yaml_quote(sha256_file(manifest_path))}"
                ),
                f"backend_pin_sha256: {yaml_quote(sha256_file(pin_path))}",
                f"imucam_config_sha256: {yaml_quote(sha256_file(imucam_path))}",
                f"imu_config_sha256: {yaml_quote(sha256_file(imu_path))}",
                f"settings_sha256: {yaml_quote(sha256_file(settings_path))}",
                f"timestamps_sha256: {yaml_quote(sha256_file(timestamps_path))}",
                (
                    "adapted_cam0_data_csv_sha256: "
                    f"{yaml_quote(sha256_file(sequence / 'cam0' / 'data.csv'))}"
                ),
                (
                    "adapted_cam1_data_csv_sha256: "
                    f"{yaml_quote(sha256_file(sequence / 'cam1' / 'data.csv'))}"
                ),
                (
                    "imu0_data_csv_sha256: "
                    f"{yaml_quote(sha256_file(imu_output / 'data.csv'))}"
                ),
                "",
            )
        ),
        encoding="utf-8",
    )
    incomplete.unlink()
    print(f"Prepared ORB-SLAM3 benchmark: {output}")
    print(
        "State: PREPARED_NOT_RUN "
        f"(stereo={adapted_range.rows}, imu={imu_rows}, "
        f"camera_offset_ns={applied_offset_ns})"
    )


def build_parser() -> argparse.ArgumentParser:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description=(
            "Prepare a complete neutral benchmark for the pinned upstream "
            "ORB-SLAM3 stereo-inertial EuRoC runner."
        )
    )
    parser.add_argument("--benchmark", required=True, type=Path)
    parser.add_argument("--imucam-config", required=True, type=Path)
    parser.add_argument("--imu-config", required=True, type=Path)
    parser.add_argument(
        "--backend-pin",
        type=Path,
        default=root / "config" / "research" / "orbslam3_backend.yaml",
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--file-mode", choices=("copy", "hardlink"), default="hardlink"
    )
    parser.add_argument(
        "--camera-stride",
        type=int,
        default=1,
        help=(
            "keep every Nth stereo pair while retaining every IMU row "
            "(default: 1)"
        ),
    )
    parser.add_argument(
        "--camera-time-offset-policy",
        choices=("calibrated", "zero"),
        default="calibrated",
        help=(
            "apply the calibrated camera-to-IMU offset or zero it for a "
            "controlled diagnostic A/B (default: calibrated)"
        ),
    )
    parser.add_argument("--n-features", type=int, default=1200)
    parser.add_argument("--initial-fast-threshold", type=int, default=20)
    parser.add_argument("--minimum-fast-threshold", type=int, default=7)
    parser.add_argument(
        "--load-atlas",
        type=Path,
        help=(
            "stage an existing .osa atlas and its adjacent "
            ".osa.manifest.yaml for an upstream multi-session map-merge "
            "experiment"
        ),
    )
    parser.add_argument(
        "--save-atlas-name",
        type=atlas_basename,
        help=(
            "save the resulting atlas under this basename in the adapter "
            "directory (ORB-SLAM3 appends .osa)"
        ),
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.camera_stride <= 0:
        parser.error("--camera-stride must be positive")
    if args.n_features <= 0:
        parser.error("--n-features must be positive")
    if args.initial_fast_threshold <= 0:
        parser.error("--initial-fast-threshold must be positive")
    if args.minimum_fast_threshold <= 0:
        parser.error("--minimum-fast-threshold must be positive")
    if args.minimum_fast_threshold > args.initial_fast_threshold:
        parser.error(
            "--minimum-fast-threshold must not exceed "
            "--initial-fast-threshold"
        )
    if args.load_atlas is not None and args.save_atlas_name is None:
        parser.error("--load-atlas requires --save-atlas-name")
    try:
        prepare(args)
    except (BenchmarkError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 4
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
