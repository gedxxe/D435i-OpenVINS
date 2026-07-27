#!/usr/bin/env python3
"""Create a local KALIBR_VERIFIED bundle after explicit human review."""

from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Any

from calibration_common import (
    CalibrationError,
    finite_float,
    read_text,
    sha256_file,
)
from validate_kalibr_outputs import load_yaml, validate_transform


ACKNOWLEDGEMENT_FLAGS = (
    "--acknowledge-camera-report-reviewed",
    "--acknowledge-imu-report-reviewed",
    "--acknowledge-transform-direction-reviewed",
    "--acknowledge-allan-fit-reviewed",
)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description=(
            "Create a serial-specific local KALIBR_VERIFIED bundle only after "
            "the structural report and all human-review gates are complete."
        )
    )
    result.add_argument("--review-report", required=True, type=Path)
    result.add_argument("--camchain", required=True, type=Path)
    result.add_argument("--imu", required=True, type=Path)
    result.add_argument("--serial", required=True)
    result.add_argument(
        "--shared-time-offset-source",
        required=True,
        choices=("cam0", "cam1"),
        help=(
            "pinned OpenVINS v2.7 uses one camera-IMU offset; choose the "
            "reviewed Kalibr camera estimate explicitly"
        ),
    )
    result.add_argument(
        "--output-dir",
        required=True,
        type=Path,
        help=(
            "must be config/local/d435i-SERIAL/kalibr inside this repository; "
            "the existing bootstrap bundle is preserved"
        ),
    )
    for flag in ACKNOWLEDGEMENT_FLAGS:
        result.add_argument(flag, action="store_true", required=True)
    return result


def dump_yaml(document: dict[str, Any]) -> str:
    try:
        import yaml
    except ModuleNotFoundError as exc:
        raise CalibrationError(
            "PyYAML is required only for calibration promotion. Create "
            ".venv and install requirements.txt; do not install it globally."
        ) from exc
    class OpenVinsDumper(yaml.SafeDumper):
        def ignore_aliases(self, data: object) -> bool:
            return True

        def increase_indent(
            self, flow: bool = False, indentless: bool = False
        ) -> object:
            # OpenCV FileStorage rejects PyYAML's otherwise-valid indentless
            # block sequences, so indent each matrix row beneath its key.
            return super().increase_indent(flow, False)

    try:
        rendered = yaml.dump(
            document,
            Dumper=OpenVinsDumper,
            sort_keys=False,
            # OpenVINS/OpenCV and the project fail-closed validator consume
            # vectors and matrix rows in flow form: [x, y, ...].
            default_flow_style=None,
            allow_unicode=False,
            width=1_000_000,
        )
    except yaml.YAMLError as exc:
        raise CalibrationError(f"cannot serialize calibration YAML: {exc}") from exc
    return "%YAML:1.0\n" + rendered


def verify_review_provenance(
    report: str, camchain: Path, imu: Path, serial: str
) -> None:
    if "Verdict: `STRUCTURAL_PASS_MANUAL_REVIEW_REQUIRED`" not in report:
        raise CalibrationError("review report lacks the structural-pass verdict")
    if f"D435i serial: `{serial}`" not in report:
        raise CalibrationError("review report serial does not match")
    for path in (camchain, imu):
        fingerprint = f"{path.name}: {sha256_file(path)}"
        if fingerprint not in report:
            raise CalibrationError(
                f"review report does not match current {path.name}"
            )
    unchecked = re.findall(r"^- \[ \] ", report, flags=re.MULTILINE)
    if unchecked:
        raise CalibrationError(
            "review report still contains unchecked manual-review items"
        )
    checked = re.findall(r"^- \[x\] ", report, flags=re.MULTILINE)
    if len(checked) != 7:
        raise CalibrationError(
            "review report must contain exactly seven completed checklist items"
        )


def replace_exact(text: str, old: str, new: str, field: str) -> str:
    if text.count(old) != 1:
        raise CalibrationError(
            f"bootstrap template does not contain exactly one {field}"
        )
    return text.replace(old, new)


def invert_rigid_transform(value: object, field: str) -> list[list[float]]:
    """Convert Kalibr T_cam_imu (IMU->camera) to OpenVINS T_imu_cam."""
    transform = validate_transform(value, field, 1e-6)
    rotation = [row[:3] for row in transform[:3]]
    translation = [transform[row][3] for row in range(3)]
    rotation_transpose = [
        [rotation[column][row] for column in range(3)]
        for row in range(3)
    ]
    inverse_translation = [
        -sum(rotation_transpose[row][column] * translation[column]
             for column in range(3))
        for row in range(3)
    ]
    return [
        rotation_transpose[row] + [inverse_translation[row]]
        for row in range(3)
    ] + [[0.0, 0.0, 0.0, 1.0]]


def main() -> int:
    args = parser().parse_args()
    repository = Path(__file__).resolve().parents[1]
    try:
        if not re.fullmatch(r"[0-9]+", args.serial):
            raise CalibrationError("--serial must contain digits only")
        output = args.output_dir.resolve()
        expected = (
            repository
            / "config"
            / "local"
            / f"d435i-{args.serial}"
            / "kalibr"
        ).resolve()
        if output != expected:
            raise CalibrationError(
                f"--output-dir must be exactly {expected}"
            )
        if output.exists():
            raise CalibrationError(
                f"output already exists; preserve and review it: {output}"
            )

        review = read_text(args.review_report)
        verify_review_provenance(
            review, args.camchain, args.imu, args.serial
        )
        camchain = load_yaml(args.camchain)
        imu = load_yaml(args.imu)
        for camera_name in ("cam0", "cam1"):
            if not isinstance(camchain.get(camera_name), dict):
                raise CalibrationError(
                    f"camchain is missing mapping {camera_name}"
                )
            if "T_imu_cam" in camchain[camera_name]:
                raise CalibrationError(
                    f"{camera_name} unexpectedly contains T_imu_cam; input "
                    "must be an unmodified Kalibr T_cam_imu result"
                )
            if "T_cam_imu" not in camchain[camera_name]:
                raise CalibrationError(
                    f"{camera_name} lacks Kalibr T_cam_imu"
                )
            if "timeshift_cam_imu" not in camchain[camera_name]:
                raise CalibrationError(
                    f"{camera_name} lacks timeshift_cam_imu"
                )
        chosen_offset = finite_float(
            camchain[args.shared_time_offset_source]["timeshift_cam_imu"],
            (
                f"{args.shared_time_offset_source}."
                "timeshift_cam_imu"
            ),
        )
        camchain["cam0"]["timeshift_cam_imu"] = chosen_offset
        camchain["cam1"]["timeshift_cam_imu"] = chosen_offset
        for camera_name in ("cam0", "cam1"):
            camera = camchain[camera_name]
            camera["T_imu_cam"] = invert_rigid_transform(
                camera.pop("T_cam_imu"),
                f"{camera_name}.T_cam_imu",
            )
        camera_document: dict[str, Any] = {
            "calibration_state": "KALIBR_VERIFIED",
            "calibrated_serial": args.serial,
            "cam0": camchain["cam0"],
            "cam1": camchain["cam1"],
        }

        if not isinstance(imu.get("imu0"), dict):
            raise CalibrationError("IMU YAML is missing mapping imu0")
        if (
            imu["imu0"].get("allan_sample_status")
            != "CHARACTERIZATION_CANDIDATE"
        ):
            raise CalibrationError(
                "IMU YAML Allan result is not a promotion candidate"
            )
        if (
            imu["imu0"].get("imu_intrinsic_status")
            != "MULTI_ORIENTATION_REVIEWED"
        ):
            raise CalibrationError(
                "IMU YAML lacks reviewed multi-orientation intrinsics"
            )
        imu_document: dict[str, Any] = {
            "calibration_state": "KALIBR_VERIFIED",
            "calibrated_serial": args.serial,
            "imu0": imu["imu0"],
        }

        template_path = repository / "config" / "sensors" / "d435i_bootstrap.yaml"
        main = read_text(template_path)
        main = replace_exact(
            main,
            'calibration_state: "BOOTSTRAP_UNVERIFIED"',
            'calibration_state: "KALIBR_VERIFIED"',
            "calibration state",
        )
        main = replace_exact(
            main,
            'calibrated_serial: "REPLACE_WITH_DEVICE_SERIAL"',
            f'calibrated_serial: "{args.serial}"',
            "serial placeholder",
        )
        main = replace_exact(
            main,
            'relative_config_imu: "d435i_factory_imu.yaml"',
            'relative_config_imu: "kalibr_imu_chain.yaml"',
            "IMU relative path",
        )
        main = replace_exact(
            main,
            'relative_config_imucam: "d435i_factory_imucam.yaml"',
            'relative_config_imucam: "kalibr_imucam_chain.yaml"',
            "camera relative path",
        )

        completed_review = review
        completed_review += (
            "\nPromotion acknowledgements were supplied explicitly on the "
            "command line.\n"
        )

        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = Path(
            tempfile.mkdtemp(
                prefix=f".d435i-{args.serial}-", dir=output.parent
            )
        )
        try:
            write_pairs = {
                "estimator.yaml": main,
                "kalibr_imu_chain.yaml": dump_yaml(imu_document),
                "kalibr_imucam_chain.yaml": dump_yaml(camera_document),
                "calibration_promotion_manifest.yaml": (
                    "%YAML:1.0\n"
                    'calibration_state: "KALIBR_VERIFIED"\n'
                    f'calibrated_serial: "{args.serial}"\n'
                    f'shared_time_offset_source: "{args.shared_time_offset_source}"\n'
                    f'camchain_sha256: "{sha256_file(args.camchain)}"\n'
                    f'imu_sha256: "{sha256_file(args.imu)}"\n'
                    f'review_report_sha256: "{sha256_file(args.review_report)}"\n'
                    "camera_report_review_acknowledged: true\n"
                    "imu_report_review_acknowledged: true\n"
                    "transform_direction_review_acknowledged: true\n"
                    "allan_fit_review_acknowledged: true\n"
                ),
                "MANUAL_REVIEW_RECORD.md": completed_review,
            }
            for name, content in write_pairs.items():
                (temporary / name).write_text(content, encoding="utf-8")
            os.replace(temporary, output)
        except Exception:
            shutil.rmtree(temporary, ignore_errors=True)
            raise

        print(f"Prepared reviewed local calibration bundle: {output}")
        print(f"Estimator config: {output / 'estimator.yaml'}")
        print("CALIBRATION_STATE=KALIBR_VERIFIED")
        return 0
    except (CalibrationError, OSError) as exc:
        print(f"Calibration promotion failed: {exc}", file=sys.stderr)
        print("CALIBRATION_STATE=NOT_PROMOTED", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
