#!/usr/bin/env python3
"""Structurally validate reviewed Kalibr inputs without promoting calibration."""

from __future__ import annotations

import argparse
import hashlib
import math
import re
import sys
from pathlib import Path
from typing import Any

from calibration_common import (  # noqa: E402
    CalibrationError,
    finite_float,
    read_text,
    sha256_file,
    simple_yaml_map,
    validate_export_provenance,
)


def load_yaml(path: Path) -> dict[str, Any]:
    try:
        import yaml
    except ModuleNotFoundError as exc:
        raise CalibrationError(
            "PyYAML is required only for calibration review. Create .venv "
            "and install requirements.txt; do not install it globally."
        ) from exc

    class KalibrLoader(yaml.SafeLoader):
        """Safe loader that accepts Kalibr/OpenCV tags as plain values."""

    def unknown_tag(loader: KalibrLoader, node: yaml.Node) -> Any:
        if isinstance(node, yaml.MappingNode):
            return loader.construct_mapping(node)
        if isinstance(node, yaml.SequenceNode):
            return loader.construct_sequence(node)
        return loader.construct_scalar(node)

    KalibrLoader.add_constructor(None, unknown_tag)
    text = read_text(path)
    if text.startswith("%YAML:1.0"):
        text = "\n".join(text.splitlines()[1:])
    try:
        value = yaml.load(text, Loader=KalibrLoader)
    except yaml.YAMLError as exc:
        raise CalibrationError(f"cannot parse {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise CalibrationError(f"{path} must contain a YAML mapping")
    return value


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description=(
            "Validate Kalibr camchain/IMU YAML and report provenance. A pass "
            "means structural checks passed; human report review is still "
            "mandatory and no KALIBR_VERIFIED files are created."
        )
    )
    result.add_argument("--export-manifest", required=True, type=Path)
    result.add_argument("--camchain", required=True, type=Path)
    result.add_argument("--imu", required=True, type=Path)
    result.add_argument("--camera-report", required=True, type=Path)
    result.add_argument("--imu-camera-report", required=True, type=Path)
    result.add_argument("--expected-serial", required=True)
    result.add_argument("--output-report", required=True, type=Path)
    result.add_argument(
        "--max-time-offset-disagreement-us",
        required=True,
        type=float,
        help=(
            "operator-reviewed maximum disagreement between cam0/cam1 "
            "timeshift estimates; no hidden project default is used"
        ),
    )
    result.add_argument(
        "--matrix-tolerance",
        type=float,
        default=1e-6,
        help="numeric tolerance for rigid-transform and zero-matrix checks",
    )
    return result


def require_mapping(value: Any, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise CalibrationError(f"{field} must be a mapping")
    return value


def require_sequence(value: Any, length: int, field: str) -> list[Any]:
    if not isinstance(value, list) or len(value) != length:
        raise CalibrationError(f"{field} must contain {length} values")
    return value


def numeric_vector(value: Any, length: int, field: str) -> list[float]:
    return [
        finite_float(item, f"{field}[{index}]")
        for index, item in enumerate(require_sequence(value, length, field))
    ]


def numeric_matrix(value: Any, rows: int, columns: int, field: str) -> list[list[float]]:
    matrix = require_sequence(value, rows, field)
    return [
        numeric_vector(row, columns, f"{field}[{index}]")
        for index, row in enumerate(matrix)
    ]


def determinant3(matrix: list[list[float]]) -> float:
    return (
        matrix[0][0]
        * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1]
        * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2]
        * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def validate_rotation(matrix: list[list[float]], field: str, tolerance: float) -> None:
    determinant = determinant3(matrix)
    if abs(determinant - 1.0) > tolerance:
        raise CalibrationError(
            f"{field} rotation determinant is {determinant}, expected 1"
        )
    for row in range(3):
        for column in range(3):
            dot = sum(matrix[index][row] * matrix[index][column] for index in range(3))
            expected = 1.0 if row == column else 0.0
            if abs(dot - expected) > tolerance:
                raise CalibrationError(f"{field} rotation is not orthonormal")


def validate_transform(value: Any, field: str, tolerance: float) -> list[list[float]]:
    matrix = numeric_matrix(value, 4, 4, field)
    if any(
        abs(matrix[3][index] - expected) > tolerance
        for index, expected in enumerate((0.0, 0.0, 0.0, 1.0))
    ):
        raise CalibrationError(f"{field} bottom row is not [0,0,0,1]")
    validate_rotation([row[:3] for row in matrix[:3]], field, tolerance)
    return matrix


def ensure_pdf(path: Path, field: str) -> None:
    try:
        with path.open("rb") as handle:
            header = handle.read(5)
    except OSError as exc:
        raise CalibrationError(f"cannot read {field}: {path}: {exc}") from exc
    if header != b"%PDF-":
        raise CalibrationError(f"{field} is not a PDF file: {path}")


def validate_camera(
    camera: dict[str, Any],
    name: str,
    expected_resolution: list[int],
    tolerance: float,
) -> tuple[list[list[float]], float]:
    if camera.get("camera_model") != "pinhole":
        raise CalibrationError(f"{name}.camera_model must be pinhole")
    if camera.get("distortion_model") != "radtan":
        raise CalibrationError(f"{name}.distortion_model must be radtan")
    intrinsics = numeric_vector(camera.get("intrinsics"), 4, f"{name}.intrinsics")
    if intrinsics[0] <= 0.0 or intrinsics[1] <= 0.0:
        raise CalibrationError(f"{name} focal lengths must be positive")
    numeric_vector(
        camera.get("distortion_coeffs"), 4, f"{name}.distortion_coeffs"
    )
    resolution_values = numeric_vector(
        camera.get("resolution"), 2, f"{name}.resolution"
    )
    resolution = [int(value) for value in resolution_values]
    if any(float(integer) != value for integer, value in zip(resolution, resolution_values)):
        raise CalibrationError(f"{name}.resolution must contain integers")
    if resolution != expected_resolution:
        raise CalibrationError(
            f"{name}.resolution {resolution} does not match capture "
            f"{expected_resolution}"
        )
    transform = validate_transform(camera.get("T_cam_imu"), f"{name}.T_cam_imu", tolerance)
    timeshift = finite_float(
        camera.get("timeshift_cam_imu"), f"{name}.timeshift_cam_imu"
    )
    return transform, timeshift


def validate_imu(
    document: dict[str, Any],
    expected_rate: int,
    motion_correction_active: bool,
    tolerance: float,
) -> None:
    imu = require_mapping(document.get("imu0"), "imu0")
    if imu.get("allan_sample_status") != "CHARACTERIZATION_CANDIDATE":
        raise CalibrationError(
            "imu0.allan_sample_status is not a reviewed characterization "
            "candidate"
        )
    if imu.get("imu_intrinsic_status") != "MULTI_ORIENTATION_REVIEWED":
        raise CalibrationError(
            "imu0.imu_intrinsic_status must be MULTI_ORIENTATION_REVIEWED; "
            "active RealSense motion correction alone does not validate "
            "identity IMU intrinsic matrices"
        )
    if imu.get("model") != "kalibr":
        raise CalibrationError("imu0.model must be kalibr")
    rate = finite_float(imu.get("update_rate"), "imu0.update_rate")
    if abs(rate - expected_rate) > tolerance:
        raise CalibrationError(
            f"imu0.update_rate {rate} does not match capture {expected_rate}"
        )
    for key in (
        "accelerometer_noise_density",
        "accelerometer_random_walk",
        "gyroscope_noise_density",
        "gyroscope_random_walk",
    ):
        if finite_float(imu.get(key), f"imu0.{key}") <= 0.0:
            raise CalibrationError(f"imu0.{key} must be positive")
    declared_policy = imu.get("realsense_motion_correction_enabled")
    if not isinstance(declared_policy, bool):
        raise CalibrationError(
            "imu0.realsense_motion_correction_enabled must be boolean"
        )
    if declared_policy != motion_correction_active:
        raise CalibrationError(
            "IMU YAML motion-correction policy does not match capture"
        )
    validate_transform(imu.get("T_i_b"), "imu0.T_i_b", tolerance)
    for key in ("Tw", "R_IMUtoGYRO", "Ta", "R_IMUtoACC", "Tg"):
        matrix = numeric_matrix(imu.get(key), 3, 3, f"imu0.{key}")
        if key in ("Tw", "Ta") and abs(determinant3(matrix)) <= tolerance:
            raise CalibrationError(
                f"imu0.{key} must be nonsingular"
            )
        if key in ("R_IMUtoGYRO", "R_IMUtoACC"):
            validate_rotation(matrix, f"imu0.{key}", tolerance)


def file_fingerprint(path: Path) -> str:
    return f"{path.name}: {sha256_file(path)}"


def main() -> int:
    args = parser().parse_args()
    try:
        if not re.fullmatch(r"[0-9]+", args.expected_serial):
            raise CalibrationError("--expected-serial must contain digits only")
        if (
            not math.isfinite(args.max_time_offset_disagreement_us)
            or args.max_time_offset_disagreement_us < 0.0
        ):
            raise CalibrationError(
                "--max-time-offset-disagreement-us must be finite and nonnegative"
            )
        if (
            not math.isfinite(args.matrix_tolerance)
            or args.matrix_tolerance <= 0.0
        ):
            raise CalibrationError("--matrix-tolerance must be positive and finite")
        if args.output_report.exists():
            raise CalibrationError(
                f"output report already exists: {args.output_report}"
            )

        manifest = simple_yaml_map(args.export_manifest)
        if manifest.get("format") != "ovrs-calibration-export-v2":
            raise CalibrationError("export manifest format is unsupported")
        if manifest.get("complete") != "true":
            raise CalibrationError("export manifest is not complete")
        if manifest.get("calibration_state") != "UNVERIFIED_CAPTURE":
            raise CalibrationError(
                "export manifest must remain UNVERIFIED_CAPTURE"
            )
        if manifest.get("calibrated_serial") != args.expected_serial:
            raise CalibrationError("export manifest serial mismatch")
        if manifest.get("capture_mode") != "imu-camera-calibration":
            raise CalibrationError(
                "Kalibr camera-IMU review requires imu-camera-calibration capture"
            )
        validate_export_provenance(args.export_manifest, manifest)
        expected_rate = int(manifest.get("gyro_rate_hz", "0"))
        if expected_rate <= 0:
            raise CalibrationError("export manifest gyro_rate_hz must be positive")
        motion_policy = manifest.get("motion_correction_active")
        if motion_policy not in {"true", "false"}:
            raise CalibrationError(
                "export manifest motion_correction_active must be boolean"
            )

        ensure_pdf(args.camera_report, "camera report")
        ensure_pdf(args.imu_camera_report, "IMU-camera report")
        camchain = load_yaml(args.camchain)
        stream = simple_yaml_map(
            args.export_manifest.parent
            / "ovrs_metadata"
            / "source_resolved_stream_config.yaml"
        )
        expected_resolution = [
            int(stream.get("width", "0")),
            int(stream.get("height", "0")),
        ]
        if any(value <= 0 for value in expected_resolution):
            raise CalibrationError(
                "source resolved stream configuration has invalid resolution"
            )

        cam0 = require_mapping(camchain.get("cam0"), "cam0")
        cam1 = require_mapping(camchain.get("cam1"), "cam1")
        _, shift0 = validate_camera(
            cam0, "cam0", expected_resolution, args.matrix_tolerance
        )
        _, shift1 = validate_camera(
            cam1, "cam1", expected_resolution, args.matrix_tolerance
        )
        stereo = validate_transform(
            cam1.get("T_cn_cnm1"), "cam1.T_cn_cnm1", args.matrix_tolerance
        )
        baseline = math.sqrt(sum(stereo[index][3] ** 2 for index in range(3)))
        if baseline <= args.matrix_tolerance:
            raise CalibrationError("stereo baseline is zero or unresolved")

        disagreement_us = abs(shift0 - shift1) * 1_000_000.0
        if disagreement_us > args.max_time_offset_disagreement_us:
            raise CalibrationError(
                "cam0/cam1 time-offset disagreement "
                f"{disagreement_us:.9g} us exceeds operator limit "
                f"{args.max_time_offset_disagreement_us:.9g} us"
            )
        validate_imu(
            load_yaml(args.imu),
            expected_rate,
            motion_policy == "true",
            args.matrix_tolerance,
        )

        report_lines = [
            "# Kalibr structural review",
            "",
            "Verdict: `STRUCTURAL_PASS_MANUAL_REVIEW_REQUIRED`",
            "",
            f"- D435i serial: `{args.expected_serial}`",
            f"- Resolution: `{expected_resolution[0]}x{expected_resolution[1]}`",
            f"- Gyroscope/synchronized IMU rate: `{expected_rate} Hz`",
            f"- Stereo baseline magnitude: `{baseline:.12g} m`",
            f"- cam0 time shift: `{shift0:.12g} s`",
            f"- cam1 time shift: `{shift1:.12g} s`",
            f"- time-shift disagreement: `{disagreement_us:.12g} us`",
            (
                "- operator-supplied disagreement limit: "
                f"`{args.max_time_offset_disagreement_us:.12g} us`"
            ),
            "",
            "## Provenance",
            "",
            *[
                f"- `{file_fingerprint(path)}`"
                for path in (
                    args.export_manifest,
                    args.camchain,
                    args.imu,
                    args.camera_report,
                    args.imu_camera_report,
                )
            ],
            "",
            "## Mandatory manual review still pending",
            "",
            "- [ ] Camera reprojection residual plots were inspected.",
            "- [ ] IMU residuals and biases stay within reviewed 3-sigma bounds.",
            "- [ ] IMU timestamp-delta plots show no batching or gaps.",
            "- [ ] AprilGrid dimensions match the printed target measurements.",
            "- [ ] Transform directions and physical stereo baseline were checked.",
            "- [ ] Camera-to-IMU time-offset sign was checked.",
            "- [ ] Allan-deviation fit and any noise inflation were documented.",
            "",
            "This report does not create or authorize `KALIBR_VERIFIED`.",
            "",
        ]
        args.output_report.parent.mkdir(parents=True, exist_ok=True)
        args.output_report.write_text("\n".join(report_lines), encoding="utf-8")
        print(f"Structural Kalibr review written: {args.output_report}")
        print("KALIBR_RESULT=MANUAL_REVIEW_REQUIRED")
        return 0
    except (CalibrationError, OSError, ValueError) as exc:
        print(f"Kalibr structural validation failed: {exc}", file=sys.stderr)
        print("KALIBR_RESULT=FAIL", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
