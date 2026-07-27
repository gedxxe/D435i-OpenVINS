#!/usr/bin/env python3
"""Migrate one legacy bootstrap bundle to OpenVINS v2.7 transform naming."""

from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
import tempfile
from pathlib import Path

from calibration_common import CalibrationError
from prepare_verified_calibration import dump_yaml, invert_rigid_transform
from validate_kalibr_outputs import load_yaml


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description=(
            "Convert legacy T_cam_imu matrices in one BOOTSTRAP_UNVERIFIED "
            "local bundle to the T_imu_cam direction consumed by pinned "
            "OpenVINS v2.7. A pre-v0.5.0 backup is retained."
        )
    )
    result.add_argument("--bundle", required=True, type=Path)
    result.add_argument(
        "--acknowledge-transform-direction",
        action="store_true",
        required=True,
        help=(
            "confirm that the input came from the repository's legacy "
            "ovrs_inspect factory export"
        ),
    )
    return result


def main() -> int:
    args = parser().parse_args()
    repository = Path(__file__).resolve().parents[1]
    local_root = (repository / "config" / "local").resolve()
    try:
        bundle = args.bundle.resolve()
        if bundle.parent != local_root:
            raise CalibrationError(
                f"--bundle must be one direct child of {local_root}"
            )
        serial_match = re.fullmatch(r"d435i-([0-9]+)", bundle.name)
        if not serial_match:
            raise CalibrationError(
                "bundle name must be d435i-NUMERIC_SERIAL"
            )
        camera_path = bundle / "d435i_factory_imucam.yaml"
        backup_path = bundle / "d435i_factory_imucam.pre-v0.5.0.yaml"
        if backup_path.exists():
            raise CalibrationError(
                f"backup already exists; refusing a second migration: {backup_path}"
            )
        document = load_yaml(camera_path)
        if document.get("calibration_state") != "BOOTSTRAP_UNVERIFIED":
            raise CalibrationError(
                "only BOOTSTRAP_UNVERIFIED factory bundles can be migrated"
            )
        serial = serial_match.group(1)
        if str(document.get("calibrated_serial", "")) != serial:
            raise CalibrationError(
                "camera calibration serial does not match bundle directory"
            )
        for camera_name in ("cam0", "cam1"):
            camera = document.get(camera_name)
            if not isinstance(camera, dict):
                raise CalibrationError(f"missing mapping {camera_name}")
            if "T_imu_cam" in camera:
                raise CalibrationError(
                    f"{camera_name} already contains T_imu_cam"
                )
            if "T_cam_imu" not in camera:
                raise CalibrationError(
                    f"{camera_name} lacks legacy T_cam_imu"
                )
            camera["T_imu_cam"] = invert_rigid_transform(
                camera.pop("T_cam_imu"), f"{camera_name}.T_cam_imu"
            )

        rendered = dump_yaml(document)
        temporary_handle = tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            newline="\n",
            prefix=".imucam-v050-",
            suffix=".yaml",
            dir=bundle,
            delete=False,
        )
        temporary = Path(temporary_handle.name)
        try:
            with temporary_handle:
                temporary_handle.write(rendered)
                temporary_handle.flush()
                os.fsync(temporary_handle.fileno())
            shutil.copy2(camera_path, backup_path)
            os.replace(temporary, camera_path)
        except Exception:
            temporary.unlink(missing_ok=True)
            if backup_path.exists() and camera_path.exists():
                backup_path.unlink(missing_ok=True)
            raise

        print(f"Migrated OpenVINS transform contract: {camera_path}")
        print(f"Preserved legacy source: {backup_path}")
        print("calibration_state=BOOTSTRAP_UNVERIFIED")
        return 0
    except (CalibrationError, OSError) as exc:
        print(f"Transform migration failed: {exc}", file=sys.stderr)
        print("calibration_state=NOT_MIGRATED", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
