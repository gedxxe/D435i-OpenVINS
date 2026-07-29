#!/usr/bin/env python3
"""Dependency-free tests for calibration capture validation/export."""

from __future__ import annotations

import csv
import hashlib
import importlib.util
import re
import struct
import subprocess
import sys
import tempfile
import unittest
from decimal import Decimal
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[1]
SCRIPTS = REPOSITORY / "scripts"
sys.path.insert(0, str(SCRIPTS))

from calibration_common import (  # noqa: E402
    CalibrationError,
    count_csv_rows,
    validate_capture,
)
from analyze_six_position_accelerometer import (  # noqa: E402
    fit_six_position,
)


def write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def write_export_manifest(path: Path, text: str, mode: str) -> None:
    if mode == "imu-allan" and "recording_duration_s:" not in text:
        text += "recording_duration_s: 10800\n"
    provenance = path.parent / "ovrs_metadata"
    scalar_values: dict[str, str] = {}
    for raw_line in text.splitlines():
        if ":" not in raw_line or raw_line[:1].isspace():
            continue
        key, value = raw_line.split(":", 1)
        scalar_values[key.strip()] = value.strip().strip('"')
    source_files = {
        "dataset_metadata": provenance / "source_dataset_metadata.yaml",
        "recording_summary": provenance / "source_recording_summary.yaml",
        "device_report": provenance / "source_device_report.yaml",
        "resolved_stream_config": (
            provenance / "source_resolved_stream_config.yaml"
        ),
    }
    for name, source in source_files.items():
        if name == "dataset_metadata":
            content = "".join(
                f'{key}: "{value}"\n'
                if key in {"capture_mode", "calibrated_serial", "infrared_profile"}
                else f"{key}: {value}\n"
                for key in (
                    "capture_mode",
                    "calibrated_serial",
                    "infrared_profile",
                    "gyro_rate_hz",
                    "accelerometer_rate_hz",
                    "motion_correction_active",
                )
                if (value := scalar_values.get(key)) is not None
            )
        elif name == "resolved_stream_config":
            content = (
                "width: 848\nheight: 480\n"
                "global_time_enabled: true\n"
            )
        elif name == "device_report":
            content = (
                "global_time_requested: true\n"
                "global_time_available: true\n"
                "global_time_active: true\n"
                "timestamp_domains:\n"
                '  accelerometer: "Global Time"\n'
                '  gyro: "Global Time"\n'
                '  infrared_1: "Global Time"\n'
                '  infrared_2: "Global Time"\n'
            )
        else:
            content = f"fixture: {name}\n"
        write(source, content)
    hashes = [
        'provenance_layout: "ovrs-export-provenance-v1"',
        *[
            f'source_{name}_sha256: "'
            f'{hashlib.sha256(source.read_bytes()).hexdigest()}"'
            for name, source in source_files.items()
        ],
    ]
    if mode in {"stereo-calibration", "imu-camera-calibration"}:
        target = path.parent / "target.yaml"
        target_text = (
            "target_type: aprilgrid\ntagRows: 6\ntagCols: 6\n"
            "tagSize: 0.04\ntagSpacing: 0.3\n"
        )
        write(target, target_text)
        source_target = provenance / "source_calibration_target.yaml"
        write(source_target, target_text)
        hashes.append(
            "source_calibration_target_sha256: "
            f'"{hashlib.sha256(source_target.read_bytes()).hexdigest()}"'
        )
        hashes.append(
            "staged_calibration_target_sha256: "
            f'"{hashlib.sha256(target.read_bytes()).hexdigest()}"'
        )
    camera_rows = int(scalar_values.get("camera_rows_per_camera", "0"))
    for camera in ("cam0", "cam1"):
        for index in range(camera_rows):
            timestamp = f"{1_000_000_000 + index:019d}"
            image = path.parent / camera / f"{timestamp}.png"
            image.parent.mkdir(parents=True, exist_ok=True)
            image.write_bytes(
                b"\x89PNG\r\n\x1a\n"
                b"\x00\x00\x00\rIHDR"
                + struct.pack(">II", 848, 480)
            )
        if camera_rows:
            write(
                provenance / f"{camera}_index.csv",
                "timestamp_ns,source_timestamp_s,raw_timestamp_ms,"
                "frameset_number,file\n"
                + "".join(
                    f"{1_000_000_000 + index:019d},{1 + index / 1000},"
                    f"{1000 + index},{index},"
                    f"{1_000_000_000 + index:019d}.png\n"
                    for index in range(camera_rows)
                ),
            )
    imu_rows = int(scalar_values.get("synchronized_imu_rows", "0"))
    if imu_rows:
        write(
            path.parent / "imu0.csv",
            "timestamp,omega_x,omega_y,omega_z,"
            "alpha_x,alpha_y,alpha_z\n"
            + "".join(
                f"{1_000_000_000 + index:019d},0,0,0,0,0,9.81\n"
                for index in range(imu_rows)
            ),
        )
        write(provenance / "imu_raw" / "gyro.csv", "fixture\n")
        write(provenance / "imu_raw" / "accelerometer.csv", "fixture\n")
    write(path, text + "\n".join(hashes) + "\n")


def make_imu_capture(root: Path, bad_queue_drop: bool = False) -> None:
    write(
        root / "dataset_metadata.yaml",
        "%YAML:1.0\n"
        'format: "ovrs-calibration-capture-v1"\n'
        'capture_mode: "imu-allan"\n'
        "complete: true\n"
        "replay_compatible: false\n"
        "requires_stationary_sensor: true\n"
        "operator_confirmed_stationary: true\n"
        "calibration_target_present: false\n"
        'calibration_state: "BOOTSTRAP_UNVERIFIED"\n'
        'calibrated_serial: "123456"\n'
        'infrared_profile: "disabled"\n'
        "gyro_rate_hz: 200\n"
        "accelerometer_rate_hz: 250\n"
        "motion_correction_active: true\n"
        "global_time_active: true\n",
    )
    write(
        root / "device_report.yaml",
        "%YAML:1.0\n"
        'serial: "123456"\n'
        "stereo_stream_enabled: false\n"
        "motion_streams_enabled: true\n"
        "global_time_requested: true\n"
        "global_time_available: true\n"
        "global_time_active: true\n"
        "timestamp_domains:\n"
        '  accelerometer: "Global Time"\n'
        '  gyro: "Global Time"\n'
        '  infrared_1: "Global Time"\n'
        '  infrared_2: "Global Time"\n',
    )
    zero_or_one = "1" if bad_queue_drop else "0"
    write(
        root / "recording_summary.yaml",
        "%YAML:1.0\n"
        "received_framesets: 0\n"
        "valid_stereo_pairs: 0\n"
        "dropped_camera_frames: 0\n"
        "malformed_frames: 0\n"
        "rejected_timestamps: 0\n"
        "callback_errors: 0\n"
        "received_gyro: 2\n"
        "received_accelerometer: 3\n"
        "stereo_queue_drops: 0\n"
        f"gyro_queue_drops: {zero_or_one}\n"
        "accelerometer_queue_drops: 0\n"
        "synchronized_imu: 2\n"
        "imu_duplicate_timestamps: 0\n"
        "imu_regressing_timestamps: 0\n"
        "imu_invalid_values: 0\n"
        "imu_synchronizer_capacity_drops: 0\n"
        "recording_duration_s: 10.25\n",
    )
    write(
        root / "resolved_stream_config.yaml",
        "%YAML:1.0\n"
        "width: 848\nheight: 480\ncamera_fps: 30\n"
        "gyro_fps: 200\naccelerometer_fps: 250\n"
        "global_time_enabled: true\n",
    )
    write(
        root / "imu" / "gyro.csv",
        "timestamp_s,raw_timestamp_ms,wx_rad_s,wy_rad_s,wz_rad_s\n"
        "0.0,1000.0,0.1,0.2,0.3\n"
        "0.005,1005.0,0.2,0.3,0.4\n",
    )
    write(
        root / "imu" / "accelerometer.csv",
        "timestamp_s,raw_timestamp_ms,ax_m_s2,ay_m_s2,az_m_s2\n"
        "0.0,1000.0,0.0,9.8,0.0\n"
        "0.004,1004.0,0.0,9.8,0.0\n"
        "0.008,1008.0,0.0,9.8,0.0\n",
    )
    write(
        root / "imu" / "synchronized.csv",
        "timestamp_s,raw_gyro_timestamp_ms,wx_rad_s,wy_rad_s,wz_rad_s,"
        "ax_m_s2,ay_m_s2,az_m_s2,interpolation_delay_s\n"
        "0.0,1000.0,0.1,0.2,0.3,0.0,9.8,0.0,0.0\n"
        "0.005,1005.0,0.2,0.3,0.4,0.0,9.8,0.0,0.001\n",
    )


def add_stereo_to_imu_capture(root: Path) -> None:
    metadata = (root / "dataset_metadata.yaml").read_text(encoding="utf-8")
    replacements = {
        'capture_mode: "imu-allan"':
            'capture_mode: "imu-camera-calibration"',
        "requires_stationary_sensor: true":
            "requires_stationary_sensor: false",
        "operator_confirmed_stationary: true":
            "operator_confirmed_stationary: false",
        "calibration_target_present: false":
            "calibration_target_present: true",
        'infrared_profile: "disabled"':
            'infrared_profile: "848x480 Y8 @30"',
    }
    for old, new in replacements.items():
        metadata = metadata.replace(old, new)
    write(root / "dataset_metadata.yaml", metadata)
    report = (root / "device_report.yaml").read_text(encoding="utf-8")
    write(
        root / "device_report.yaml",
        report.replace(
            "stereo_stream_enabled: false",
            "stereo_stream_enabled: true",
        ),
    )
    summary = (root / "recording_summary.yaml").read_text(encoding="utf-8")
    summary = summary.replace(
        "received_framesets: 0", "received_framesets: 2"
    ).replace("valid_stereo_pairs: 0", "valid_stereo_pairs: 2")
    write(root / "recording_summary.yaml", summary)
    write(
        root / "calibration_target.yaml",
        "%YAML:1.0\n"
        "target_type: aprilgrid\n"
        "tagRows: 6\n"
        "tagCols: 6\n"
        "tagSize: 0.04\n"
        "tagSpacing: 0.25\n",
    )
    png_header = (
        b"\x89PNG\r\n\x1a\n"
        + struct.pack(">I", 13)
        + b"IHDR"
        + struct.pack(">II", 848, 480)
    )
    for camera in ("cam0", "cam1"):
        write(
            root / camera / "data.csv",
            "timestamp_s,raw_timestamp_ms,frameset_number,file\n"
            "0.0,1000.0,1,1.png\n"
            "0.033333333,1033.333333,2,2.png\n",
        )
        for filename in ("1.png", "2.png"):
            image = root / camera / "data" / filename
            image.parent.mkdir(parents=True, exist_ok=True)
            image.write_bytes(png_header)


class CalibrationScriptTests(unittest.TestCase):
    @unittest.skipUnless(
        importlib.util.find_spec("yaml") is not None,
        "PyYAML is optional and not installed",
    )
    def test_promoted_yaml_uses_openvins_sequence_layout(self) -> None:
        from prepare_verified_calibration import dump_yaml

        shared_identity = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        rendered = dump_yaml(
            {
                "cam0": {
                    "intrinsics": [
                        430.7511714933689,
                        431.0621766049679,
                        428.0704768813045,
                        242.71937930514002,
                    ],
                    "T_imu_cam": [
                        [1.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0],
                        [0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0],
                    ],
                },
                "Tw": shared_identity,
                "R_IMUtoGYRO": shared_identity,
            }
        )

        self.assertIn(
            "intrinsics: [430.7511714933689, 431.0621766049679, "
            "428.0704768813045, 242.71937930514002]",
            rendered,
        )
        self.assertIn(
            "T_imu_cam:\n    - [1.0, 0.0, 0.0, 0.0]", rendered
        )
        self.assertNotIn("- - ", rendered)
        self.assertNotIn("&id", rendered)
        self.assertNotIn("*id", rendered)

    def test_kalibr_transform_is_inverted_for_openvins(self) -> None:
        from prepare_verified_calibration import invert_rigid_transform

        kalibr_transform = [
            [0.0, -1.0, 0.0, 1.0],
            [1.0, 0.0, 0.0, 2.0],
            [0.0, 0.0, 1.0, 3.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        openvins_transform = invert_rigid_transform(
            kalibr_transform, "cam0.T_cam_imu"
        )
        self.assertEqual(
            openvins_transform,
            [
                [0.0, 1.0, 0.0, -2.0],
                [-1.0, 0.0, 0.0, 1.0],
                [0.0, 0.0, 1.0, -3.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        )

    @unittest.skipUnless(
        importlib.util.find_spec("yaml") is not None,
        "PyYAML is optional and not installed",
    )
    def test_imu_yaml_preparation_binds_matching_capture_policy(self) -> None:
        import yaml

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            common = (
                "%YAML:1.0\n"
                'format: "ovrs-calibration-export-v2"\n'
                "complete: true\n"
                'calibration_state: "UNVERIFIED_CAPTURE"\n'
                'calibrated_serial: "123456"\n'
                "gyro_rate_hz: 200\n"
                "motion_correction_active: true\n"
            )
            allan_manifest = root / "allan" / "calibration_export_manifest.yaml"
            imucam_manifest = (
                root / "imucam" / "calibration_export_manifest.yaml"
            )
            write_export_manifest(
                allan_manifest,
                common + 'capture_mode: "imu-allan"\n',
                "imu-allan",
            )
            write_export_manifest(
                imucam_manifest,
                common + 'capture_mode: "imu-camera-calibration"\n',
                "imu-camera-calibration",
            )
            allan_yaml = root / "allan.yaml"
            write(
                allan_yaml,
                "accelerometer_noise_density: 0.01\n"
                "accelerometer_random_walk: 0.001\n"
                "gyroscope_noise_density: 0.0001\n"
                "gyroscope_random_walk: 0.00001\n"
                "rostopic: /imu0\n"
                "update_rate: 200\n",
            )
            output = root / "imu_yaml"
            completed = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPTS / "prepare_imu_calibration_yaml.py"),
                    "--allan-yaml",
                    str(allan_yaml),
                    "--allan-export-manifest",
                    str(allan_manifest),
                    "--imu-camera-export-manifest",
                    str(imucam_manifest),
                    "--output-dir",
                    str(output),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            kalibr_generated = (output / "kalibr_imu.yaml").read_text(
                encoding="utf-8"
            )
            self.assertFalse(kalibr_generated.startswith("%YAML"))
            self.assertEqual(
                yaml.safe_load(kalibr_generated)["rostopic"], "/imu0"
            )
            generated = (output / "openvins_imu.yaml").read_text(
                encoding="utf-8"
            )
            self.assertIn("Tg:", generated)
            self.assertIn("realsense_motion_correction_enabled: true", generated)
            self.assertIn("realsense_global_time_enabled: true", generated)
            self.assertIn(
                "allan_sample_status: CHARACTERIZATION_CANDIDATE",
                generated,
            )
            self.assertIn(
                "IDENTITY_ASSUMPTION_REQUIRES_MULTI_ORIENTATION_REVIEW",
                generated,
            )
            self.assertNotIn("INCOMPLETE", [item.name for item in output.iterdir()])

            short_manifest = root / "short" / "calibration_export_manifest.yaml"
            write_export_manifest(
                short_manifest,
                common
                + 'capture_mode: "imu-allan"\n'
                + "recording_duration_s: 3600\n",
                "imu-allan",
            )
            short_output = root / "short_output"
            short_command = [
                sys.executable,
                str(SCRIPTS / "prepare_imu_calibration_yaml.py"),
                "--allan-yaml",
                str(allan_yaml),
                "--allan-export-manifest",
                str(short_manifest),
                "--imu-camera-export-manifest",
                str(imucam_manifest),
                "--output-dir",
                str(short_output),
            ]
            short_completed = subprocess.run(
                short_command, check=False, capture_output=True, text=True
            )
            self.assertEqual(
                short_completed.returncode, 0, short_completed.stderr
            )
            self.assertIn(
                "allan_sample_status: CHARACTERIZATION_CANDIDATE",
                (short_output / "openvins_imu.yaml").read_text(
                    encoding="utf-8"
                ),
            )

            intrinsics = root / "kalibr_intrinsics.yaml"
            write(
                intrinsics,
                "imu0:\n"
                "  model: scale-misalignment\n"
                "  rostopic: /imu0\n"
                "  update_rate: 200\n"
                "  accelerometer_noise_density: 0.01\n"
                "  accelerometer_random_walk: 0.001\n"
                "  gyroscope_noise_density: 0.0001\n"
                "  gyroscope_random_walk: 0.00001\n"
                "  accelerometers:\n"
                "    M: [[1.01, 0.0, 0.0], [0.01, 0.99, 0.0], "
                "[0.02, -0.01, 1.02]]\n"
                "  gyroscopes:\n"
                "    M: [[1.02, 0.0, 0.0], [0.01, 0.98, 0.0], "
                "[-0.01, 0.02, 1.01]]\n"
                "    C_gyro_i: [[0.0, -1.0, 0.0], "
                "[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]\n"
                "    A: [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6], "
                "[0.7, 0.8, 0.9]]\n",
            )
            reviewed_output = root / "reviewed_output"
            reviewed = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPTS / "prepare_imu_calibration_yaml.py"),
                    "--allan-yaml",
                    str(allan_yaml),
                    "--allan-export-manifest",
                    str(allan_manifest),
                    "--imu-camera-export-manifest",
                    str(imucam_manifest),
                    "--kalibr-intrinsics-yaml",
                    str(intrinsics),
                    "--acknowledge-kalibr-scale-misalignment-reviewed",
                    "--output-dir",
                    str(reviewed_output),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(reviewed.returncode, 0, reviewed.stderr)
            reviewed_text = (
                reviewed_output / "openvins_imu.yaml"
            ).read_text(encoding="utf-8")
            self.assertIn(
                "imu_intrinsic_status: MULTI_ORIENTATION_REVIEWED",
                reviewed_text,
            )
            self.assertIn("[1.01, 0.0, 0.0]", reviewed_text)
            self.assertIn(
                "imu_intrinsic_method: KALIBR_SCALE_MISALIGNMENT",
                reviewed_text,
            )
            # Tg is Kalibr A*C_gyro_i, not a blind copy of A.
            self.assertIn("Tg:\n    - [0.2, -0.1, 0.3]", reviewed_text)

            calibrated_intrinsics = root / "calibrated_intrinsics.yaml"
            write(
                calibrated_intrinsics,
                intrinsics.read_text(encoding="utf-8").replace(
                    "model: scale-misalignment", "model: calibrated"
                ),
            )
            calibrated_rejected = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPTS / "prepare_imu_calibration_yaml.py"),
                    "--allan-yaml",
                    str(allan_yaml),
                    "--allan-export-manifest",
                    str(allan_manifest),
                    "--imu-camera-export-manifest",
                    str(imucam_manifest),
                    "--kalibr-intrinsics-yaml",
                    str(calibrated_intrinsics),
                    "--acknowledge-kalibr-scale-misalignment-reviewed",
                    "--output-dir",
                    str(root / "calibrated_rejected"),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(calibrated_rejected.returncode, 2)
            self.assertIn(
                "model must be scale-misalignment",
                calibrated_rejected.stderr,
            )

            wrong_rate_yaml = root / "wrong_rate_allan.yaml"
            write(
                wrong_rate_yaml,
                allan_yaml.read_text(encoding="utf-8").replace(
                    "update_rate: 200", "update_rate: 400"
                ),
            )
            wrong_rate = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPTS / "prepare_imu_calibration_yaml.py"),
                    "--allan-yaml",
                    str(wrong_rate_yaml),
                    "--allan-export-manifest",
                    str(allan_manifest),
                    "--imu-camera-export-manifest",
                    str(imucam_manifest),
                    "--output-dir",
                    str(root / "wrong_rate"),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(wrong_rate.returncode, 0)
            self.assertIn("does not match capture gyro rate", wrong_rate.stderr)

            provenance = (
                allan_manifest.parent
                / "ovrs_metadata"
                / "source_device_report.yaml"
            )
            original_provenance = provenance.read_text(encoding="utf-8")
            write(provenance, original_provenance + "tampered: true\n")
            tampered = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPTS / "prepare_imu_calibration_yaml.py"),
                    "--allan-yaml",
                    str(allan_yaml),
                    "--allan-export-manifest",
                    str(allan_manifest),
                    "--imu-camera-export-manifest",
                    str(imucam_manifest),
                    "--output-dir",
                    str(root / "tampered"),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(tampered.returncode, 0)
            self.assertIn("provenance hash mismatch", tampered.stderr)
            write(provenance, original_provenance)

            mismatch = common.replace(
                "motion_correction_active: true",
                "motion_correction_active: false",
            )
            write_export_manifest(
                imucam_manifest,
                mismatch + 'capture_mode: "imu-camera-calibration"\n',
                "imu-camera-calibration",
            )
            rejected = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPTS / "prepare_imu_calibration_yaml.py"),
                    "--allan-yaml",
                    str(allan_yaml),
                    "--allan-export-manifest",
                    str(allan_manifest),
                    "--imu-camera-export-manifest",
                    str(imucam_manifest),
                    "--output-dir",
                    str(root / "rejected"),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(rejected.returncode, 0)
            self.assertIn("disagree on motion_correction_active", rejected.stderr)

    def test_calibration_export_set_requires_coherent_capture_state(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            common = (
                "%YAML:1.0\n"
                'format: "ovrs-calibration-export-v2"\n'
                "complete: true\n"
                'calibration_state: "UNVERIFIED_CAPTURE"\n'
                'calibrated_serial: "123456"\n'
            )
            allan = root / "allan" / "calibration_export_manifest.yaml"
            stereo = root / "stereo" / "calibration_export_manifest.yaml"
            imucam = root / "imucam" / "calibration_export_manifest.yaml"
            write_export_manifest(
                allan,
                common
                + 'capture_mode: "imu-allan"\n'
                + 'infrared_profile: "disabled"\n'
                + "camera_rows_per_camera: 0\n"
                + "synchronized_imu_rows: 2\n"
                + "gyro_rate_hz: 200\n"
                + "accelerometer_rate_hz: 250\n"
                + "motion_correction_active: true\n",
                "imu-allan",
            )
            write_export_manifest(
                stereo,
                common
                + 'capture_mode: "stereo-calibration"\n'
                + 'infrared_profile: "848x480 Y8 @30"\n'
                + "camera_rows_per_camera: 10\n"
                + "synchronized_imu_rows: 0\n"
                + "gyro_rate_hz: 0\n"
                + "accelerometer_rate_hz: 0\n"
                + "motion_correction_active: false\n",
                "stereo-calibration",
            )
            write_export_manifest(
                imucam,
                common
                + 'capture_mode: "imu-camera-calibration"\n'
                + 'infrared_profile: "848x480 Y8 @30"\n'
                + "camera_rows_per_camera: 10\n"
                + "synchronized_imu_rows: 2\n"
                + "gyro_rate_hz: 200\n"
                + "accelerometer_rate_hz: 250\n"
                + "motion_correction_active: true\n",
                "imu-camera-calibration",
            )
            report = root / "calibration_set.yaml"
            command = [
                sys.executable,
                str(SCRIPTS / "validate_calibration_export_set.py"),
                "--allan-export",
                str(allan.parent),
                "--stereo-export",
                str(stereo.parent),
                "--imu-camera-export",
                str(imucam.parent),
                "--output-report",
                str(report),
            ]
            completed = subprocess.run(
                command, check=False, capture_output=True, text=True
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            report_text = report.read_text(encoding="utf-8")
            self.assertIn("UNVERIFIED_EXPORT_SET", report_text)
            self.assertIn('calibrated_serial: "123456"', report_text)
            self.assertIn("global_time_enabled: true", report_text)

            stereo_stream = (
                stereo.parent
                / "ovrs_metadata"
                / "source_resolved_stream_config.yaml"
            )
            stereo_device = (
                stereo.parent / "ovrs_metadata" / "source_device_report.yaml"
            )
            write(
                stereo_stream,
                stereo_stream.read_text(encoding="utf-8").replace(
                    "global_time_enabled: true",
                    "global_time_enabled: false",
                ),
            )
            write(
                stereo_device,
                stereo_device.read_text(encoding="utf-8")
                .replace("global_time_requested: true", "global_time_requested: false")
                .replace("global_time_active: true", "global_time_active: false")
                .replace("Global Time", "Hardware Clock"),
            )
            stereo_text = stereo.read_text(encoding="utf-8")
            for key, source in (
                ("source_device_report_sha256", stereo_device),
                ("source_resolved_stream_config_sha256", stereo_stream),
            ):
                stereo_text = re.sub(
                    rf'({key}: ")[0-9a-f]{{64}}(")',
                    rf"\g<1>{hashlib.sha256(source.read_bytes()).hexdigest()}\2",
                    stereo_text,
                )
            write(stereo, stereo_text)
            mixed_clock = subprocess.run(
                command[:-1] + [str(root / "mixed_clock.yaml")],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(mixed_clock.returncode, 0)
            self.assertIn(
                "exports disagree on timestamp policy", mixed_clock.stderr
            )

            write_export_manifest(
                stereo,
                common
                + 'capture_mode: "stereo-calibration"\n'
                + 'infrared_profile: "848x480 Y8 @30"\n'
                + "camera_rows_per_camera: 10\n"
                + "synchronized_imu_rows: 0\n"
                + "gyro_rate_hz: 0\n"
                + "accelerometer_rate_hz: 0\n"
                + "motion_correction_active: false\n",
                "stereo-calibration",
            )
            valid_imucam_manifest = imucam.read_text(encoding="utf-8")
            write(
                imucam,
                valid_imucam_manifest.replace(
                    'calibrated_serial: "123456"',
                    'calibrated_serial: "654321"',
                ),
            )
            manifest_tampered = subprocess.run(
                command[:-1] + [str(root / "manifest_tampered.yaml")],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(manifest_tampered.returncode, 0)
            self.assertIn(
                "calibrated_serial does not match",
                manifest_tampered.stderr,
            )
            write(imucam, valid_imucam_manifest)

            missing_image = (
                imucam.parent / "cam0" / "0000000001000000000.png"
            )
            missing_image_bytes = missing_image.read_bytes()
            missing_image.unlink()
            layout_rejected = subprocess.run(
                command[:-1] + [str(root / "layout_mismatch.yaml")],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(layout_rejected.returncode, 0)
            self.assertIn("entries, expected", layout_rejected.stderr)
            missing_image.write_bytes(missing_image_bytes)

            source_metadata = (
                imucam.parent
                / "ovrs_metadata"
                / "source_dataset_metadata.yaml"
            )
            valid_source_metadata = source_metadata.read_text(encoding="utf-8")
            old_source_hash = hashlib.sha256(
                source_metadata.read_bytes()
            ).hexdigest()
            write(
                source_metadata,
                valid_source_metadata.replace(
                    'calibrated_serial: "123456"',
                    'calibrated_serial: "654321"',
                ),
            )
            new_source_hash = hashlib.sha256(
                source_metadata.read_bytes()
            ).hexdigest()
            mismatch_text = valid_imucam_manifest.replace(
                'calibrated_serial: "123456"',
                'calibrated_serial: "654321"',
            ).replace(old_source_hash, new_source_hash)
            write(imucam, mismatch_text)
            rejected = subprocess.run(
                command[:-1] + [str(root / "mismatch.yaml")],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(rejected.returncode, 0)
            self.assertIn("disagree on calibrated_serial", rejected.stderr)

            write(imucam, valid_imucam_manifest)
            write(source_metadata, valid_source_metadata)
            imucam_target = imucam.parent / "target.yaml"
            valid_imucam_target = imucam_target.read_text(encoding="utf-8")
            write(
                imucam_target,
                valid_imucam_target.replace(
                    "tagSize: 0.04", "tagSize: 0.041"
                ),
            )
            staged_target_rejected = subprocess.run(
                command[:-1] + [str(root / "staged_target_tampered.yaml")],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(staged_target_rejected.returncode, 0)
            self.assertIn(
                "provenance hash mismatch for target.yaml",
                staged_target_rejected.stderr,
            )
            write(imucam_target, valid_imucam_target)

            source_target = (
                imucam.parent
                / "ovrs_metadata"
                / "source_calibration_target.yaml"
            )
            old_target_hash = hashlib.sha256(
                source_target.read_bytes()
            ).hexdigest()
            write(
                source_target,
                source_target.read_text(encoding="utf-8").replace(
                    "tagSize: 0.04", "tagSize: 0.041"
                ),
            )
            write(
                imucam_target,
                valid_imucam_target.replace(
                    "tagSize: 0.04", "tagSize: 0.041"
                ),
            )
            new_target_hash = hashlib.sha256(
                source_target.read_bytes()
            ).hexdigest()
            write(
                imucam,
                valid_imucam_manifest.replace(old_target_hash, new_target_hash),
            )
            target_rejected = subprocess.run(
                command[:-1] + [str(root / "target_mismatch.yaml")],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertNotEqual(target_rejected.returncode, 0)
            self.assertIn(
                "disagree on source_calibration_target_sha256",
                target_rejected.stderr,
            )

    def test_aprilgrid_generator_requires_measured_dimensions(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "target.yaml"
            command = [
                sys.executable,
                str(SCRIPTS / "create_aprilgrid_target.py"),
                "--tag-rows",
                "6",
                "--tag-cols",
                "6",
                "--tag-size-m",
                "0.04",
                "--tag-gap-m",
                "0.012",
                "--output",
                str(output),
            ]
            completed = subprocess.run(
                command, check=False, capture_output=True, text=True
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            target = output.read_text(encoding="utf-8")
            self.assertFalse(target.startswith("%YAML"))
            self.assertIn("tagSize: 0.04", target)
            self.assertIn("tagSpacing: 0.3", target)
            repeated = subprocess.run(
                command, check=False, capture_output=True, text=True
            )
            self.assertEqual(repeated.returncode, 2)

    def test_aprilgrid_a4_planner_checks_border_and_page_fit(self) -> None:
        base = [
            sys.executable,
            str(SCRIPTS / "plan_aprilgrid_target.py"),
            "--tag-rows",
            "6",
            "--tag-cols",
            "6",
            "--tag-size-mm",
            "18",
            "--tag-spacing-ratio",
            "0.3",
            "--paper-width-mm",
            "210",
            "--paper-height-mm",
            "297",
            "--printer-margin-mm",
            "5",
        ]
        accepted = subprocess.run(
            base, check=False, capture_output=True, text=True
        )
        self.assertEqual(accepted.returncode, 0, accepted.stderr)
        self.assertIn("portrait_fit: PASS", accepted.stdout)
        self.assertIn("fit_result: PASS", accepted.stdout)
        self.assertIn(
            "rosrun kalibr kalibr_create_target_pdf",
            accepted.stdout,
        )
        self.assertIn("--nx 6 --ny 6", accepted.stdout)

        rejected = subprocess.run(
            [*base, "--tag-size-mm", "40"],
            check=False,
            capture_output=True,
            text=True,
        )
        # argparse intentionally accepts the last repeated option.
        self.assertEqual(rejected.returncode, 5, rejected.stderr)
        self.assertIn("fit_result: FAIL", rejected.stdout)

    def test_operator_runbook_uses_rosrun_for_catkin_kalibr_tools(
        self,
    ) -> None:
        runbook = (
            REPOSITORY / "docs" / "operator_runbook.md"
        ).read_text(encoding="utf-8")
        for tool in (
            "kalibr_create_target_pdf",
            "kalibr_bagcreater",
            "kalibr_calibrate_cameras",
            "kalibr_calibrate_imu_camera",
        ):
            self.assertIn(f"rosrun kalibr {tool}", runbook)
        self.assertNotIn("exec kalibr_", runbook)
        self.assertNotRegex(
            runbook,
            r"(?m)^  kalibr_(?:bag|calibrate)[A-Za-z0-9_]*[ \t]+",
        )
        self.assertIn("--imu-models scale-misalignment", runbook)
        self.assertNotIn("--imu-intrinsics-yaml", runbook)

    def test_d435i_template_retains_short_baseline_estimator_limits(
        self,
    ) -> None:
        template = (
            REPOSITORY / "config" / "sensors" / "d435i_bootstrap.yaml"
        ).read_text(encoding="utf-8")
        expected_lines = (
            "init_window_time: 2.0",
            "num_pts: 200",
            'histogram_method: "HISTOGRAM"',
            "fi_max_cond_number: 60000",
            "fi_max_baseline: 100",
            "zupt_chi2_multipler: 1",
            "zupt_only_at_beginning: true",
            "zupt_constrain_velocity: true",
            "zupt_velocity_noise: 0.05",
            "zupt_min_stationary_time: 0.25",
            "max_estimated_speed_m_s: 3.0",
            "max_accel_bias_m_s2: 2.0",
        )
        for line in expected_lines:
            self.assertEqual(template.count(line), 1)

    def test_selected_runtime_documentation_uses_one_policy(self) -> None:
        readme = (REPOSITORY / "README.md").read_text(encoding="utf-8")
        selected = (
            REPOSITORY / "docs" / "selected_runtime.md"
        ).read_text(encoding="utf-8")
        estimator_doc = (
            REPOSITORY / "config" / "estimator" / "README.md"
        ).read_text(encoding="utf-8")
        preflight = (
            REPOSITORY / "scripts" / "preflight_ubuntu.sh"
        ).read_text(encoding="utf-8")

        selected_path = (
            "config/local/d435i-843212070146/"
            "selected_runtime/estimator.yaml"
        )
        for document in (readme, selected, estimator_doc):
            self.assertIn(selected_path, document)

        for document in (readme, selected):
            self.assertIn("--online-time-offset off", document)
            self.assertNotIn("--online-time-offset on", document)

        self.assertIn("zupt_only_at_beginning: false", selected)
        self.assertIn("zupt_min_stationary_time: 1.0", selected)
        self.assertIn("visually gated continuous ZUPT", selected)
        self.assertIn("zupt_only_at_beginning: false", estimator_doc)
        self.assertIn("consecutive per-frame disparity", estimator_doc)
        self.assertIn(
            'config/local/d435i-*/selected_runtime/*_imucam.yaml',
            preflight,
        )

    def test_interactive_viewer_contract_is_documented(self) -> None:
        readme = (REPOSITORY / "README.md").read_text(encoding="utf-8")
        selected = (
            REPOSITORY / "docs" / "selected_runtime.md"
        ).read_text(encoding="utf-8")
        manual = (
            REPOSITORY / "docs" / "manual_test.md"
        ).read_text(encoding="utf-8")
        live = (REPOSITORY / "apps" / "ovrs_live.cpp").read_text(
            encoding="utf-8"
        )
        replay = (REPOSITORY / "apps" / "ovrs_replay.cpp").read_text(
            encoding="utf-8"
        )

        for document in (readme, selected, manual):
            normalized = document.lower()
            self.assertIn("left-drag", normalized)
            self.assertIn("middle/right-drag", normalized)
            self.assertIn("wheel", normalized)

        for source in (live, replay):
            self.assertIn("global XYZ", source)
            self.assertIn("interactive", source)
            self.assertIn("left-drag orbits", source)
            self.assertIn("middle/right-drag", source)
            self.assertIn("pans, the ", source)
            self.assertIn("wheel zooms", source)

    @unittest.skipUnless(
        (
            REPOSITORY
            / "third_party"
            / "open_vins"
            / "ov_msckf"
            / "src"
            / "core"
            / "VioManager.h"
        ).is_file(),
        "OpenVINS submodule is not initialized",
    )
    def test_visual_diagnostics_use_current_tracks_and_msckf_batches(
        self,
    ) -> None:
        manager_header = (
            REPOSITORY
            / "third_party"
            / "open_vins"
            / "ov_msckf"
            / "src"
            / "core"
            / "VioManager.h"
        ).read_text(encoding="utf-8")
        manager_source = (
            REPOSITORY
            / "third_party"
            / "open_vins"
            / "ov_msckf"
            / "src"
            / "core"
            / "VioManager.cpp"
        ).read_text(encoding="utf-8")
        adapter = (REPOSITORY / "src" / "openvins_estimator.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("get_active_track_count(size_t camera_id)", manager_header)
        self.assertIn("struct MsckfUpdateStats", manager_header)
        self.assertIn("get_last_msckf_update_stats()", manager_header)
        self.assertIn("trackFEATS->get_last_ids()", manager_source)
        self.assertIn(
            "last_msckf_update_stats.accepted_features = "
            "featsup_MSCKF.size()",
            manager_source,
        )
        self.assertIn("get_active_track_count(0)", adapter)
        self.assertIn("get_last_msckf_update_stats()", adapter)
        self.assertIn("msckf_update_quality_available = true", adapter)

    @unittest.skipUnless(
        (
            REPOSITORY
            / "third_party"
            / "open_vins"
            / "ov_msckf"
            / "src"
            / "update"
            / "UpdaterZeroVelocity.cpp"
        ).is_file(),
        "OpenVINS submodule is not initialized",
    )
    def test_zupt_patch_requires_visual_and_inertial_gates(self) -> None:
        source = (
            REPOSITORY
            / "third_party"
            / "open_vins"
            / "ov_msckf"
            / "src"
            / "update"
            / "UpdaterZeroVelocity.cpp"
        ).read_text(encoding="utf-8")
        self.assertNotIn("override_with_disparity_check", source)
        self.assertIn("const bool frame_is_stationary", source)
        self.assertIn(
            "missing tracks are unknown, not\n  // proof that the rig is stationary",
            source,
        )
        self.assertIn(
            "chi2 > _options.chi2_multipler * chi2_check || "
            "state->_imu->vel().norm() > _zupt_max_velocity",
            source,
        )
        self.assertNotIn("if (!disparity_passed &&", source)
        self.assertIn(
            "if (_constrain_velocity && visually_confirmed_stationary)",
            source,
        )

    def test_realsense_motion_profiles_never_silently_fallback(self) -> None:
        source = (
            REPOSITORY / "src" / "realsense_source.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("profile.fps() == requested", source)
        self.assertIn("rate fallback is disabled", source)
        self.assertNotIn("std::abs(a - requested)", source)
        self.assertIn(
            "configured scale factor is applied to the",
            source,
        )
        self.assertIn(
            '\\"m/s^2 from librealsense motion API\\"',
            source,
        )

    def test_review_checklist_must_be_completed_by_operator(self) -> None:
        from prepare_verified_calibration import verify_review_provenance

        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            camchain = root / "camchain.yaml"
            imu = root / "imu.yaml"
            write(camchain, "cam0: {}\n")
            write(imu, "imu0: {}\n")
            header = (
                "Verdict: `STRUCTURAL_PASS_MANUAL_REVIEW_REQUIRED`\n"
                "D435i serial: `123456`\n"
                f"{camchain.name}: "
                f"{hashlib.sha256(camchain.read_bytes()).hexdigest()}\n"
                f"{imu.name}: "
                f"{hashlib.sha256(imu.read_bytes()).hexdigest()}\n"
            )
            unchecked = header + "".join(
                f"- [ ] review item {index}\n" for index in range(8)
            )
            with self.assertRaisesRegex(
                CalibrationError, "still contains unchecked"
            ):
                verify_review_provenance(
                    unchecked, camchain, imu, "123456"
                )
            completed = unchecked.replace("- [ ] ", "- [x] ")
            verify_review_provenance(completed, camchain, imu, "123456")

    def test_aprilgrid_generator_accepts_measured_millimetres(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "target.yaml"
            completed = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPTS / "create_aprilgrid_target.py"),
                    "--tag-rows",
                    "6",
                    "--tag-cols",
                    "6",
                    "--tag-size-mm",
                    "18",
                    "--tag-gap-mm",
                    "5.4",
                    "--output",
                    str(output),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            text = output.read_text(encoding="utf-8")
            self.assertIn("tagSize: 0.018", text)
            self.assertIn("tagSpacing: 0.3", text)

    def test_valid_imu_capture_and_export(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            capture = root / "capture"
            output = root / "export"
            make_imu_capture(capture)
            info = validate_capture(capture)
            self.assertEqual(info.mode, "imu-allan")
            self.assertFalse(info.stereo_enabled)
            self.assertEqual(info.synchronized_imu_rows, 2)
            self.assertIsNone(info.gyro_sensitivity)
            self.assertIsNone(info.gyro_scale_factor)

            completed = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPTS / "export_calibration_capture.py"),
                    "--capture",
                    str(capture),
                    "--output",
                    str(output),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertFalse((output / "INCOMPLETE").exists())
            manifest = (output / "calibration_export_manifest.yaml").read_text(
                encoding="utf-8"
            )
            self.assertIn('capture_mode: "imu-allan"', manifest)
            self.assertIn('format: "ovrs-calibration-export-v2"', manifest)
            self.assertIn("kalibr_executed: false", manifest)
            self.assertTrue(
                (
                    output
                    / "ovrs_metadata"
                    / "source_resolved_stream_config.yaml"
                ).is_file()
            )
            with (output / "imu0.csv").open(
                encoding="utf-8", newline=""
            ) as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[1]["timestamp"], "0000000000005000000")
            self.assertEqual(
                (output / "allan_variance_config.yaml").read_text(
                    encoding="utf-8"
                ),
                'imu_topic: "/imu0"\n'
                "imu_rate: 200\n"
                "measure_rate: 200\n"
                "sequence_time: 10\n",
            )

    def test_capture_binds_explicit_gyro_sensitivity(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            capture = root / "capture"
            output = root / "export"
            make_imu_capture(capture)
            metadata = capture / "dataset_metadata.yaml"
            report = capture / "device_report.yaml"
            stream = capture / "resolved_stream_config.yaml"
            write(
                metadata,
                metadata.read_text(encoding="utf-8")
                + "gyro_sensitivity_active: 1\n"
                + "gyro_scale_factor_applied: 0.5\n",
            )
            write(
                report,
                report.read_text(encoding="utf-8")
                + "gyro_sensitivity_requested: 1\n"
                + "gyro_sensitivity_available: true\n"
                + "gyro_sensitivity_active: 1\n"
                + "gyro_scale_factor_configured: 0.5\n"
                + "gyro_scale_factor_applied: 0.5\n",
            )
            write(
                stream,
                stream.read_text(encoding="utf-8")
                + "gyro_sensitivity: 1\n"
                + "gyro_scale_factor: 0.5\n",
            )

            info = validate_capture(capture)
            self.assertEqual(info.gyro_sensitivity, 1)
            self.assertEqual(info.gyro_scale_factor, Decimal("0.5"))
            completed = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPTS / "export_calibration_capture.py"),
                    "--capture",
                    str(capture),
                    "--output",
                    str(output),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            manifest = (
                output / "calibration_export_manifest.yaml"
            ).read_text(encoding="utf-8")
            self.assertIn("gyro_sensitivity: 1", manifest)
            self.assertIn("gyro_scale_factor: 0.5", manifest)

            write(
                report,
                report.read_text(encoding="utf-8").replace(
                    "gyro_sensitivity_active: 1",
                    "gyro_sensitivity_active: 0",
                ),
            )
            with self.assertRaisesRegex(
                CalibrationError,
                "configuration, request, readback, and capture metadata "
                "disagree",
            ):
                validate_capture(capture)

    def test_capture_validator_streams_and_ignores_nested_report_keys(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            make_imu_capture(root)
            report = root / "device_report.yaml"
            write(
                report,
                report.read_text(encoding="utf-8")
                + "gyroscope_factory_intrinsics:\n"
                + "  scale_cross_axis_bias:\n"
                + "    - [1, 0, 0, 0]\n"
                + "accelerometer_factory_intrinsics:\n"
                + "  scale_cross_axis_bias:\n"
                + "    - [1, 0, 0, 0]\n",
            )
            info = validate_capture(root)
            self.assertEqual(info.synchronized_imu_rows, 2)
            self.assertEqual(
                count_csv_rows(
                    root / "imu" / "gyro.csv",
                    (
                        "timestamp_s",
                        "raw_timestamp_ms",
                        "wx_rad_s",
                        "wy_rad_s",
                        "wz_rad_s",
                    ),
                ),
                2,
            )
            completed = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPTS / "validate_calibration_capture.py"),
                    "--capture",
                    str(root),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn("validation: PASS", completed.stdout)
            self.assertIn("capture_modified: false", completed.stdout)

            device_report = root / "device_report.yaml"
            valid_device_report = device_report.read_text(encoding="utf-8")
            write(
                device_report,
                valid_device_report.replace(
                    'gyro: "Global Time"', 'gyro: "Hardware Clock"'
                ),
            )
            with self.assertRaisesRegex(
                CalibrationError,
                "gyro timestamp domain is Hardware Clock, expected Global Time",
            ):
                validate_capture(root)
            write(device_report, valid_device_report)

            summary = root / "recording_summary.yaml"
            write(
                summary,
                summary.read_text(encoding="utf-8").replace(
                    "received_gyro: 2", "received_gyro: 3"
                ),
            )
            rejected = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPTS / "validate_calibration_capture.py"),
                    "--capture",
                    str(root),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(rejected.returncode, 2)
            self.assertIn(
                "received_gyro reports 3, but CSV contains 2 rows",
                rejected.stderr,
            )

    def test_imucam_capture_exports_images_target_and_imu(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            capture = root / "capture"
            output = root / "export"
            make_imu_capture(capture)
            add_stereo_to_imu_capture(capture)
            info = validate_capture(capture)
            self.assertEqual(info.mode, "imu-camera-calibration")
            self.assertEqual(info.camera_rows, 2)
            completed = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPTS / "export_calibration_capture.py"),
                    "--capture",
                    str(capture),
                    "--output",
                    str(output),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertTrue((output / "target.yaml").is_file())
            self.assertFalse(
                (output / "target.yaml")
                .read_text(encoding="utf-8")
                .startswith("%YAML")
            )
            self.assertTrue(
                (
                    output
                    / "ovrs_metadata"
                    / "source_calibration_target.yaml"
                )
                .read_text(encoding="utf-8")
                .startswith("%YAML:1.0")
            )
            self.assertTrue(
                (output / "cam0" / "0000000000000000000.png").is_file()
            )
            self.assertTrue(
                (
                    output
                    / "cam1"
                    / "0000000000033333333.png"
                ).is_file()
            )
            self.assertTrue((output / "imu0.csv").is_file())
            self.assertFalse(
                (output / "allan_variance_config.yaml").exists()
            )

    def test_drop_and_incomplete_capture_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            make_imu_capture(root, bad_queue_drop=True)
            with self.assertRaises(CalibrationError):
                validate_capture(root)
            (root / "INCOMPLETE").write_text("partial\n", encoding="utf-8")
            with self.assertRaises(CalibrationError):
                validate_capture(root)

    def test_validator_help_does_not_require_pyyaml(self) -> None:
        completed = subprocess.run(
            [
                sys.executable,
                str(SCRIPTS / "validate_kalibr_outputs.py"),
                "--help",
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("--max-time-offset-disagreement-us", completed.stdout)

    @unittest.skipUnless(
        importlib.util.find_spec("yaml") is not None,
        "PyYAML is optional and not installed",
    )
    def test_kalibr_structural_validator_passes_then_rejects_singular_intrinsics(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            manifest = root / "calibration_export_manifest.yaml"
            camchain = root / "camchain.yaml"
            imu = root / "imu.yaml"
            camera_report = root / "camera-report.pdf"
            imucam_report = root / "imucam-report.pdf"
            report = root / "review.md"
            write_export_manifest(
                manifest,
                "%YAML:1.0\n"
                'format: "ovrs-calibration-export-v2"\n'
                'capture_mode: "imu-camera-calibration"\n'
                'calibrated_serial: "123456"\n'
                'infrared_profile: "848x480 Y8 @30"\n'
                "complete: true\n"
                'calibration_state: "UNVERIFIED_CAPTURE"\n'
                "gyro_rate_hz: 200\n"
                "motion_correction_active: true\n",
                "imu-camera-calibration",
            )
            identity4 = (
                "[[1.0, 0.0, 0.0, 0.0], "
                "[0.0, 1.0, 0.0, 0.0], "
                "[0.0, 0.0, 1.0, 0.0], "
                "[0.0, 0.0, 0.0, 1.0]]"
            )
            write(
                camchain,
                "%YAML:1.0\n"
                "cam0:\n"
                "  camera_model: pinhole\n"
                "  distortion_model: radtan\n"
                "  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]\n"
                "  intrinsics: [420.0, 420.0, 424.0, 240.0]\n"
                "  resolution: [848, 480]\n"
                f"  T_cam_imu: {identity4}\n"
                "  timeshift_cam_imu: 0.0002\n"
                "cam1:\n"
                "  camera_model: pinhole\n"
                "  distortion_model: radtan\n"
                "  distortion_coeffs: [0.0, 0.0, 0.0, 0.0]\n"
                "  intrinsics: [420.0, 420.0, 424.0, 240.0]\n"
                "  resolution: [848, 480]\n"
                "  T_cam_imu: [[1.0, 0.0, 0.0, -0.05], "
                "[0.0, 1.0, 0.0, 0.0], "
                "[0.0, 0.0, 1.0, 0.0], "
                "[0.0, 0.0, 0.0, 1.0]]\n"
                "  T_cn_cnm1: [[1.0, 0.0, 0.0, -0.05], "
                "[0.0, 1.0, 0.0, 0.0], "
                "[0.0, 0.0, 1.0, 0.0], "
                "[0.0, 0.0, 0.0, 1.0]]\n"
                "  timeshift_cam_imu: 0.0002004\n",
            )
            zero3 = (
                "[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], "
                "[0.0, 0.0, 0.0]]"
            )
            identity3 = (
                "[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], "
                "[0.0, 0.0, 1.0]]"
            )
            imu_text = (
                "%YAML:1.0\n"
                "imu0:\n"
                "  allan_sample_status: CHARACTERIZATION_CANDIDATE\n"
                "  imu_intrinsic_status: MULTI_ORIENTATION_REVIEWED\n"
                "  imu_intrinsic_method: KALIBR_SCALE_MISALIGNMENT\n"
                "  imu_intrinsic_mapping: ovrs-kalibr-openvins-imu-v1\n"
                f"  imu_intrinsic_source_sha256: {'a' * 64}\n"
                "  model: kalibr\n"
                "  update_rate: 200\n"
                "  realsense_motion_correction_enabled: true\n"
                "  realsense_global_time_enabled: true\n"
                "  accelerometer_noise_density: 0.001\n"
                "  accelerometer_random_walk: 0.0001\n"
                "  gyroscope_noise_density: 0.0001\n"
                "  gyroscope_random_walk: 0.00001\n"
                f"  T_i_b: {identity4}\n"
                f"  Tw: {identity3}\n"
                f"  R_IMUtoGYRO: {identity3}\n"
                f"  Ta: {identity3}\n"
                f"  R_IMUtoACC: {identity3}\n"
                f"  Tg: {zero3}\n"
                f"  kalibr_gyroscope_A: {zero3}\n"
            )
            write(imu, imu_text)
            camera_report.write_bytes(b"%PDF-1.4\nsynthetic\n")
            imucam_report.write_bytes(b"%PDF-1.4\nsynthetic\n")
            command = [
                sys.executable,
                str(SCRIPTS / "validate_kalibr_outputs.py"),
                "--export-manifest",
                str(manifest),
                "--camchain",
                str(camchain),
                "--imu",
                str(imu),
                "--camera-report",
                str(camera_report),
                "--imu-camera-report",
                str(imucam_report),
                "--expected-serial",
                "123456",
                "--max-time-offset-disagreement-us",
                "1.0",
                "--output-report",
                str(report),
            ]
            completed = subprocess.run(
                command, check=False, capture_output=True, text=True
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)
            self.assertIn(
                "STRUCTURAL_PASS_MANUAL_REVIEW_REQUIRED",
                report.read_text(encoding="utf-8"),
            )

            report.unlink()
            write(
                imu,
                imu_text.replace(
                    "allan_sample_status: CHARACTERIZATION_CANDIDATE",
                    "allan_sample_status: UNREVIEWED_CHARACTERIZATION",
                ),
            )
            status_rejected = subprocess.run(
                command, check=False, capture_output=True, text=True
            )
            self.assertEqual(status_rejected.returncode, 2)
            self.assertIn(
                "not a reviewed characterization candidate",
                status_rejected.stderr,
            )

            write(imu, imu_text.replace(f"Ta: {identity3}", f"Ta: {zero3}"))
            rejected = subprocess.run(
                command, check=False, capture_output=True, text=True
            )
            self.assertEqual(rejected.returncode, 2)
            self.assertIn(
                "Ta diagonal entries must be positive", rejected.stderr
            )

    def test_stationary_gravity_gate_passes_and_rejects_bound(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write(
                root / "imu" / "synchronized.csv",
                "timestamp_s,raw_gyro_timestamp_ms,"
                "raw_accel_before_timestamp_ms,"
                "raw_accel_after_timestamp_ms,"
                "wx_rad_s,wy_rad_s,wz_rad_s,"
                "ax_m_s2,ay_m_s2,az_m_s2\n"
                "0.00,0,0,0,0,0,0,0,0,9.8\n"
                "0.01,10,10,10,0,0,0,0,0,9.8\n"
                "0.02,20,20,20,0,0,0,0,0,9.8\n"
                "0.03,30,30,30,0,0,0,0,0,9.8\n",
            )
            estimator = root / "estimator.yaml"
            write(estimator, "%YAML:1.0\ngravity_mag: 9.81\n")
            command = [
                sys.executable,
                str(SCRIPTS / "analyze_stationary_imu.py"),
                str(root),
                "--start-s",
                "0",
                "--duration-s",
                "0.04",
                "--estimator-config",
                str(estimator),
                "--max-gravity-error-m-s2",
                "0.02",
            ]
            accepted = subprocess.run(
                command, check=False, capture_output=True, text=True
            )
            self.assertEqual(accepted.returncode, 0, accepted.stderr)
            self.assertIn("validation: PASS", accepted.stdout)

            command[-1] = "0.001"
            rejected = subprocess.run(
                command, check=False, capture_output=True, text=True
            )
            self.assertEqual(rejected.returncode, 5, rejected.stderr)
            self.assertIn("validation: FAIL", rejected.stdout)

    def test_stationary_analyzer_reports_missing_paths_without_traceback(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            missing_config = root / "missing" / "bootstrap.yaml"
            completed = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPTS / "analyze_stationary_imu.py"),
                    str(root),
                    "--start-s",
                    "5",
                    "--duration-s",
                    "60",
                    "--estimator-config",
                    str(missing_config),
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(completed.returncode, 2)
            self.assertIn(
                "--estimator-config is not a readable file",
                completed.stderr,
            )
            self.assertNotIn("Traceback", completed.stderr)

    def test_six_position_accelerometer_fit_recovers_affine_model(
        self,
    ) -> None:
        gravity = 9.80665
        forward = [
            [1.01, 0.02, -0.01],
            [-0.01, 0.99, 0.015],
            [0.005, -0.02, 1.02],
        ]
        bias = [0.12, -0.08, 0.04]
        means: dict[str, list[float]] = {}
        for name, axis, sign in (
            ("x-positive", 0, 1.0),
            ("x-negative", 0, -1.0),
            ("y-positive", 1, 1.0),
            ("y-negative", 1, -1.0),
            ("z-positive", 2, 1.0),
            ("z-negative", 2, -1.0),
        ):
            ideal = [0.0, 0.0, 0.0]
            ideal[axis] = sign * gravity
            means[name] = [
                sum(forward[row][column] * ideal[column]
                    for column in range(3))
                + bias[row]
                for row in range(3)
            ]

        fit = fit_six_position(means, gravity)
        for actual, expected in zip(fit["bias"], bias):
            self.assertAlmostEqual(actual, expected, places=12)
        for actual_row, expected_row in zip(fit["forward"], forward):
            for actual, expected in zip(actual_row, expected_row):
                self.assertAlmostEqual(actual, expected, places=12)
        self.assertAlmostEqual(fit["residual_max"], 0.0, places=12)
        for name, axis, sign in (
            ("x-positive", 0, 1.0),
            ("x-negative", 0, -1.0),
            ("y-positive", 1, 1.0),
            ("y-negative", 1, -1.0),
            ("z-positive", 2, 1.0),
            ("z-negative", 2, -1.0),
        ):
            corrected = fit["corrected"][name]
            for index, value in enumerate(corrected):
                expected = sign * gravity if index == axis else 0.0
                self.assertAlmostEqual(value, expected, places=10)

    def test_six_position_accelerometer_fit_rejects_singular_model(
        self,
    ) -> None:
        means = {
            f"{axis}-{sign}": [0.0, 0.0, 9.80665]
            for axis in ("x", "y", "z")
            for sign in ("positive", "negative")
        }
        with self.assertRaisesRegex(ValueError, "singular"):
            fit_six_position(means, 9.80665)


if __name__ == "__main__":
    unittest.main()
