#!/usr/bin/env python3
"""Bind Allan noise results to matching D435i calibration captures."""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Any

from calibration_common import (
    CalibrationError,
    finite_float,
    sha256_file,
    simple_yaml_map,
    validate_export_provenance,
)
from prepare_verified_calibration import dump_yaml
from validate_kalibr_outputs import (
    determinant3,
    load_yaml,
    numeric_matrix,
    validate_rotation,
)


NOISE_KEYS = (
    "accelerometer_noise_density",
    "accelerometer_random_walk",
    "gyroscope_noise_density",
    "gyroscope_random_walk",
)


def dump_kalibr_yaml(document: dict[str, Any]) -> str:
    rendered = dump_yaml(document)
    opencv_directive = "%YAML:1.0\n"
    if not rendered.startswith(opencv_directive):
        raise CalibrationError("unexpected calibration YAML serialization")
    return rendered[len(opencv_directive):]


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description=(
            "Validate Allan noise output against matching imu-allan and "
            "imu-camera calibration exports, then create Kalibr input and an "
            "unverified OpenVINS IMU YAML. This does not run Allan analysis, "
            "Kalibr, or certify the values."
        )
    )
    result.add_argument("--allan-yaml", required=True, type=Path)
    result.add_argument(
        "--allan-export-manifest", required=True, type=Path
    )
    result.add_argument(
        "--imu-camera-export-manifest", required=True, type=Path
    )
    result.add_argument(
        "--imu-intrinsics-yaml",
        type=Path,
        help=(
            "reviewed multi-orientation IMU intrinsic YAML containing imu0 "
            "Tw, Ta, R_IMUtoGYRO, R_IMUtoACC, and Tg matrices"
        ),
    )
    result.add_argument(
        "--acknowledge-multi-orientation-intrinsics-reviewed",
        action="store_true",
        help=(
            "required with --imu-intrinsics-yaml after reviewing its method, "
            "axis convention, motion-correction policy, and fit residuals"
        ),
    )
    result.add_argument("--output-dir", required=True, type=Path)
    return result


def validate_manifest(path: Path, expected_mode: str) -> dict[str, str]:
    manifest = simple_yaml_map(path)
    if manifest.get("format") != "ovrs-calibration-export-v2":
        raise CalibrationError(f"{path}: unsupported export format")
    if manifest.get("complete") != "true":
        raise CalibrationError(f"{path}: export is not complete")
    if manifest.get("calibration_state") != "UNVERIFIED_CAPTURE":
        raise CalibrationError(
            f"{path}: calibration_state must be UNVERIFIED_CAPTURE"
        )
    if manifest.get("capture_mode") != expected_mode:
        raise CalibrationError(
            f"{path}: expected capture_mode {expected_mode}"
        )
    validate_export_provenance(path, manifest)
    return manifest


def allan_source(document: dict[str, Any]) -> dict[str, Any]:
    source: object = document.get("imu0", document)
    if not isinstance(source, dict):
        raise CalibrationError("Allan YAML root/imu0 must be a mapping")
    return source


def positive_noise_values(source: dict[str, Any]) -> dict[str, float]:
    values: dict[str, float] = {}
    for key in NOISE_KEYS:
        value = finite_float(source.get(key), key)
        if value <= 0.0:
            raise CalibrationError(f"{key} must be positive")
        values[key] = value
    return values


def main() -> int:
    args = parser().parse_args()
    output: Path | None = None
    try:
        allan_manifest = validate_manifest(
            args.allan_export_manifest, "imu-allan"
        )
        imucam_manifest = validate_manifest(
            args.imu_camera_export_manifest, "imu-camera-calibration"
        )
        for key in (
            "calibrated_serial",
            "gyro_rate_hz",
            "motion_correction_active",
        ):
            if allan_manifest.get(key) != imucam_manifest.get(key):
                raise CalibrationError(
                    f"Allan and IMU-camera exports disagree on {key}"
                )
        serial = allan_manifest.get("calibrated_serial", "")
        if not serial.isdigit():
            raise CalibrationError(
                "export calibrated_serial must contain digits only"
            )
        try:
            update_rate = int(allan_manifest.get("gyro_rate_hz", ""))
        except ValueError as exc:
            raise CalibrationError(
                "export gyro_rate_hz must be an integer"
            ) from exc
        if update_rate <= 0:
            raise CalibrationError("export gyro_rate_hz must be positive")
        allan_duration_s = finite_float(
            allan_manifest.get("recording_duration_s"),
            "Allan export recording_duration_s",
        )
        if allan_duration_s <= 0.0:
            raise CalibrationError(
                "Allan export recording_duration_s must be positive"
            )
        # Duration is provenance, not a quality certificate. A longer capture
        # provides lower-frequency Allan coverage, but cannot compensate for
        # bad timestamps, wrong units/axes, motion, or a poor fitted region.
        allan_sample_status = "CHARACTERIZATION_CANDIDATE"
        motion_correction_active = (
            allan_manifest.get("motion_correction_active") == "true"
        )
        if bool(args.imu_intrinsics_yaml) != bool(
            args.acknowledge_multi_orientation_intrinsics_reviewed
        ):
            raise CalibrationError(
                "--imu-intrinsics-yaml and "
                "--acknowledge-multi-orientation-intrinsics-reviewed must be "
                "supplied together"
            )

        source = allan_source(load_yaml(args.allan_yaml))
        if source.get("rostopic") != "/imu0":
            raise CalibrationError(
                "Allan YAML rostopic must be /imu0 from the exported capture"
            )
        allan_rate = finite_float(source.get("update_rate"), "update_rate")
        if allan_rate != float(update_rate):
            raise CalibrationError(
                f"Allan YAML update_rate {allan_rate} does not match "
                f"capture gyro rate {update_rate}"
            )
        noise = positive_noise_values(source)
        kalibr_document: dict[str, Any] = {
            **noise,
            "rostopic": "/imu0",
            "update_rate": update_rate,
        }
        identity3 = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        identity4 = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        zero3 = [[0.0, 0.0, 0.0] for _ in range(3)]
        intrinsic_status = (
            "IDENTITY_ASSUMPTION_REQUIRES_MULTI_ORIENTATION_REVIEW"
        )
        intrinsic_source_sha256 = "none"
        tw = identity3
        gyro_rotation = identity3
        ta = identity3
        accel_rotation = identity3
        tg = zero3
        if args.imu_intrinsics_yaml:
            intrinsic_document = load_yaml(args.imu_intrinsics_yaml)
            intrinsic_source = intrinsic_document.get(
                "imu0", intrinsic_document
            )
            if not isinstance(intrinsic_source, dict):
                raise CalibrationError(
                    "IMU intrinsic YAML root/imu0 must be a mapping"
                )
            intrinsic_motion_policy = intrinsic_source.get(
                "realsense_motion_correction_enabled"
            )
            if not isinstance(intrinsic_motion_policy, bool):
                raise CalibrationError(
                    "IMU intrinsic YAML must declare boolean "
                    "realsense_motion_correction_enabled"
                )
            if intrinsic_motion_policy != motion_correction_active:
                raise CalibrationError(
                    "IMU intrinsic YAML motion-correction policy does not "
                    "match the captures"
                )
            tw = numeric_matrix(intrinsic_source.get("Tw"), 3, 3, "imu0.Tw")
            gyro_rotation = numeric_matrix(
                intrinsic_source.get("R_IMUtoGYRO"),
                3,
                3,
                "imu0.R_IMUtoGYRO",
            )
            ta = numeric_matrix(intrinsic_source.get("Ta"), 3, 3, "imu0.Ta")
            accel_rotation = numeric_matrix(
                intrinsic_source.get("R_IMUtoACC"),
                3,
                3,
                "imu0.R_IMUtoACC",
            )
            tg = numeric_matrix(intrinsic_source.get("Tg"), 3, 3, "imu0.Tg")
            validate_rotation(
                gyro_rotation, "imu0.R_IMUtoGYRO", 1e-6
            )
            validate_rotation(
                accel_rotation, "imu0.R_IMUtoACC", 1e-6
            )
            if abs(determinant3(tw)) <= 1e-9:
                raise CalibrationError("imu0.Tw must be nonsingular")
            if abs(determinant3(ta)) <= 1e-9:
                raise CalibrationError("imu0.Ta must be nonsingular")
            intrinsic_status = "MULTI_ORIENTATION_REVIEWED"
            intrinsic_source_sha256 = sha256_file(args.imu_intrinsics_yaml)

        openvins_document: dict[str, Any] = {
            "calibration_state": "UNVERIFIED_KALIBR_INPUT",
            "calibrated_serial": serial,
            "imu0": {
                "realsense_motion_correction_enabled": (
                    motion_correction_active
                ),
                "allan_sample_status": allan_sample_status,
                "imu_intrinsic_status": intrinsic_status,
                "T_i_b": identity4,
                **noise,
                "time_offset": 0.0,
                "update_rate": update_rate,
                "model": "kalibr",
                "Tw": tw,
                "R_IMUtoGYRO": gyro_rotation,
                "Ta": ta,
                "R_IMUtoACC": accel_rotation,
                "Tg": tg,
            },
        }

        output = args.output_dir.resolve()
        if output.exists():
            raise CalibrationError(
                f"output already exists; choose a new directory: {output}"
            )
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = Path(
            tempfile.mkdtemp(prefix=".imu-yaml-", dir=output.parent)
        )
        try:
            (temporary / "INCOMPLETE").write_text(
                "IMU YAML preparation did not complete.\n",
                encoding="utf-8",
            )
            (temporary / "kalibr_imu.yaml").write_text(
                dump_kalibr_yaml(kalibr_document), encoding="utf-8"
            )
            (temporary / "openvins_imu.yaml").write_text(
                dump_yaml(openvins_document), encoding="utf-8"
            )
            (temporary / "imu_yaml_manifest.yaml").write_text(
                "%YAML:1.0\n"
                'format: "ovrs-imu-yaml-preparation-v1"\n'
                'calibration_state: "UNVERIFIED_KALIBR_INPUT"\n'
                f'calibrated_serial: "{serial}"\n'
                f"update_rate: {update_rate}\n"
                f"motion_correction_active: "
                f"{str(motion_correction_active).lower()}\n"
                f"allan_recording_duration_s: {allan_duration_s:.17g}\n"
                f'allan_sample_status: "{allan_sample_status}"\n'
                "imu_intrinsic_policy: "
                f'"{intrinsic_status}"\n'
                f'imu_intrinsics_source_sha256: "{intrinsic_source_sha256}"\n'
                'Tg_policy: "zero-g-sensitivity-not-estimated"\n'
                f'allan_yaml_sha256: "{sha256_file(args.allan_yaml)}"\n'
                "allan_export_manifest_sha256: "
                f'"{sha256_file(args.allan_export_manifest)}"\n'
                "imu_camera_export_manifest_sha256: "
                f'"{sha256_file(args.imu_camera_export_manifest)}"\n',
                encoding="utf-8",
            )
            (temporary / "INCOMPLETE").unlink()
            os.replace(temporary, output)
        except Exception:
            shutil.rmtree(temporary, ignore_errors=True)
            raise

        print(f"Prepared unverified IMU YAML bundle: {output}")
        print("calibration_state=UNVERIFIED_KALIBR_INPUT")
        return 0
    except (CalibrationError, OSError) as exc:
        print(f"IMU YAML preparation failed: {exc}", file=sys.stderr)
        if output is not None:
            print("calibration_state=NOT_PREPARED", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
