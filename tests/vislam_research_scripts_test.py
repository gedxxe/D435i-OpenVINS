#!/usr/bin/env python3
"""Dependency-free tests for the markerless VSLAM benchmark workflow."""

from __future__ import annotations

import csv
import hashlib
import math
import subprocess
import sys
import tempfile
import unittest
from decimal import Decimal
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EXPORTER = ROOT / "scripts" / "export_vislam_benchmark.py"
ORB_ADAPTER = ROOT / "scripts" / "prepare_orbslam3_benchmark.py"
ORB_LIVE_PREPARER = ROOT / "scripts" / "prepare_orbslam3_live.py"
ORB_EVALUATOR = ROOT / "scripts" / "evaluate_orbslam3_run.py"
ORB_LIVE_EVALUATOR = ROOT / "scripts" / "evaluate_orbslam3_live_run.py"
ORB_RUNNER = ROOT / "scripts" / "run_orbslam3_benchmark.py"
ORB_LIVE_LAUNCHER = ROOT / "scripts" / "run_orbslam3_live.sh"
ORB_CAPTURE_EXCITATION = (
    ROOT / "scripts" / "evaluate_orbslam3_capture_excitation.py"
)
ORB_BACKEND_PIN = ROOT / "config" / "research" / "orbslam3_backend.yaml"
ORB_PATCH = ROOT / "patches" / "orbslam3-atlas-serialization-integrity.patch"


def backend_pin_scalar(key: str) -> str:
    prefix = f"{key}: "
    for line in ORB_BACKEND_PIN.read_text(encoding="utf-8").splitlines():
        if line.startswith(prefix):
            return line[len(prefix):].strip().strip("\"'")
    raise AssertionError(f"ORB-SLAM3 backend pin lacks {key}")


ORB_BACKEND_COMMIT = backend_pin_scalar("commit")
ORB_PATCH_SHA256 = backend_pin_scalar("patch_sha256")


def write(path: Path, content: str | bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(content, bytes):
        path.write_bytes(content)
    else:
        path.write_text(content, encoding="utf-8")


def make_dataset(root: Path) -> Path:
    dataset = root / "dataset"
    write(
        dataset / "dataset_metadata.yaml",
        '%YAML:1.0\n'
        'format: "ovrs-euroc-like-v1"\n'
        'capture_mode: "vio"\n'
        "complete: true\n"
        "replay_compatible: true\n",
    )
    write(
        dataset / "device_report.yaml",
        '%YAML:1.0\nserial: "test-d435i"\n',
    )
    write(
        dataset / "resolved_stream_config.yaml",
        '%YAML:1.0\n'
        'serial: "test-d435i"\n'
        "width: 848\n"
        "height: 480\n"
        "camera_fps: 90\n"
        "gyro_scale_factor: 1\n"
        "motion_correction_enabled: true\n"
        "global_time_enabled: true\n"
        "stereo_tolerance_ms: 2.0\n",
    )
    zero_counters = (
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
    write(
        dataset / "recording_summary.yaml",
        "%YAML:1.0\n"
        + "".join(f"{name}: 0\n" for name in zero_counters)
        + "imu_missing_interpolation_brackets: 1\n",
    )
    camera_header = (
        "timestamp_s,raw_timestamp_ms,frameset_number,file\n"
    )
    write(
        dataset / "cam0" / "data.csv",
        camera_header
        + "1.000000000,1000.0,10,10.png\n"
        + "1.100000000,1100.0,11,11.png\n",
    )
    write(
        dataset / "cam1" / "data.csv",
        camera_header
        + "1.000200000,1000.2,10,10.png\n"
        + "1.100200000,1100.2,11,11.png\n",
    )
    for camera in ("cam0", "cam1"):
        write(dataset / camera / "data" / "10.png", b"fixture-10")
        write(dataset / camera / "data" / "11.png", b"fixture-11")
    write(
        dataset / "imu" / "synchronized.csv",
        "timestamp_s,raw_gyro_timestamp_ms,wx_rad_s,wy_rad_s,wz_rad_s,"
        "ax_m_s2,ay_m_s2,az_m_s2,interpolation_delay_s\n"
        "0.990,990.0,0.1,0.2,0.3,1.0,2.0,9.7,0.001\n"
        "1.050,1050.0,0.2,0.3,0.4,1.1,2.1,9.6,0.001\n"
        "1.101,1101.0,0.3,0.4,0.5,1.2,2.2,9.5,0.001\n",
    )
    return dataset


def run_export(dataset: Path, output: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        (
            sys.executable,
            str(EXPORTER),
            "--dataset",
            str(dataset),
            "--output",
            str(output),
            "--image-mode",
            "copy",
        ),
        check=False,
        text=True,
        capture_output=True,
    )


def make_calibration(root: Path) -> tuple[Path, Path]:
    imucam = root / "imucam.yaml"
    imu = root / "imu.yaml"
    camera_common = (
        "  camera_model: pinhole\n"
        "  distortion_model: radtan\n"
        "  distortion_coeffs: [0.01, -0.001, 0.0002, -0.0003]\n"
        "  intrinsics: [429.0, 430.0, 424.0, 240.0]\n"
        "  resolution: [848, 480]\n"
        "  timeshift_cam_imu: -0.0049\n"
    )
    write(
        imucam,
        "%YAML:1.0\n"
        'calibration_state: "BOOTSTRAP_UNVERIFIED"\n'
        'calibrated_serial: "test-d435i"\n'
        "cam0:\n"
        "  T_imu_cam:\n"
        "    - [1.0, 0.0, 0.0, 0.0]\n"
        "    - [0.0, 1.0, 0.0, 0.0]\n"
        "    - [0.0, 0.0, 1.0, 0.0]\n"
        "    - [0.0, 0.0, 0.0, 1.0]\n"
        + camera_common
        + "cam1:\n"
        "  T_imu_cam:\n"
        "    - [1.0, 0.0, 0.0, 0.095]\n"
        "    - [0.0, 1.0, 0.0, 0.0]\n"
        "    - [0.0, 0.0, 1.0, 0.0]\n"
        "    - [0.0, 0.0, 0.0, 1.0]\n"
        + camera_common,
    )
    write(
        imu,
        "%YAML:1.0\n"
        'calibration_state: "BOOTSTRAP_UNVERIFIED"\n'
        'calibrated_serial: "test-d435i"\n'
        "imu0:\n"
        "  realsense_motion_correction_enabled: true\n"
        "  realsense_global_time_enabled: true\n"
        '  imu_intrinsic_method: "REALSENSE_DEVICE_TABLE_WITH_SDK_CORRECTION"\n'
        "  accelerometer_noise_density: 0.0016\n"
        "  accelerometer_random_walk: 0.00016\n"
        "  gyroscope_noise_density: 0.00015\n"
        "  gyroscope_random_walk: 0.0000047\n"
        "  update_rate: 200\n",
    )
    return imucam, imu


def run_adapter(
    benchmark: Path,
    imucam: Path,
    imu: Path,
    output: Path,
    *extra_args: str,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        (
            sys.executable,
            str(ORB_ADAPTER),
            "--benchmark",
            str(benchmark),
            "--imucam-config",
            str(imucam),
            "--imu-config",
            str(imu),
            "--output",
            str(output),
            "--file-mode",
            "copy",
            *extra_args,
        ),
        check=False,
        text=True,
        capture_output=True,
    )


def write_source_atlas_manifest(
    atlas: Path, imucam: Path, imu: Path
) -> Path:
    manifest = atlas.with_suffix(".osa.manifest.yaml")
    write(
        manifest,
        "%YAML:1.0\n"
        'format: "ovrs-orbslam3-atlas-manifest-v1"\n'
        'state: "TRACKING_GATE_PASS_ATLAS_RELOAD_UNVERIFIED"\n'
        f'atlas_file: "{atlas.name}"\n'
        f'atlas_sha256: "{hashlib.sha256(atlas.read_bytes()).hexdigest()}"\n'
        'backend_name: "ORB_SLAM3"\n'
        f'backend_commit: "{ORB_BACKEND_COMMIT}"\n'
        f'backend_patch_sha256: "{ORB_PATCH_SHA256}"\n'
        "backend_pin_sha256: "
        f'"{hashlib.sha256(ORB_BACKEND_PIN.read_bytes()).hexdigest()}"\n'
        'camera_serial: "test-d435i"\n'
        'calibration_state: "BOOTSTRAP_UNVERIFIED"\n'
        f'imucam_config_sha256: "{hashlib.sha256(imucam.read_bytes()).hexdigest()}"\n'
        f'imu_config_sha256: "{hashlib.sha256(imu.read_bytes()).hexdigest()}"\n'
        'source_camera_fps: "90"\n'
        'adapted_camera_fps: "90"\n'
        'camera_stride: "1"\n'
        'camera_time_offset_policy: "calibrated"\n'
        'calibrated_camera_imu_time_offset_ns: "-4900000"\n'
        'applied_camera_imu_time_offset_ns: "-4900000"\n'
        f'result_manifest_sha256: "{"1" * 64}"\n'
        f'backend_runner_sha256: "{"2" * 64}"\n'
        f'backend_library_sha256: "{"3" * 64}"\n'
        f'vocabulary_sha256: "{"4" * 64}"\n'
        'coordinate_frame_policy: "ORB_SLAM3_ATLAS_WORLD"\n'
        "ground_truth_consumed_by_estimator: false\n",
    )
    return manifest


def make_orb_run(root: Path, reset: bool = False) -> tuple[Path, ...]:
    adapter = root / "adapter_manifest.yaml"
    log = root / "backend.log"
    frame = root / "frames.txt"
    keyframe = root / "keyframes.txt"
    reference = root / "reference.yaml"
    write(
        adapter,
        "%YAML:1.0\n"
        'format: "ovrs-orbslam3-adapter-v1"\n'
        'state: "PREPARED_NOT_RUN"\n'
        'backend_name: "ORB_SLAM3"\n'
        "ground_truth_consumed_by_estimator: false\n"
        "adapted_stereo_pairs: 3\n"
        "adapted_camera_fps: 30\n"
        "first_adapted_stereo_timestamp_ns: 1000000000\n"
        "last_adapted_stereo_timestamp_ns: 1066666666\n",
    )
    reset_line = (
        "TRACK: Reset map because local mapper set the bad imu flag\n"
        if reset
        else ""
    )
    write(
        log,
        "New Map created with 100 points\n"
        "end VIBA 1\n"
        "end VIBA 2\n"
        "*Loop detected\n"
        "BAD LOOP!!!\n"
        "*Loop detected\n"
        + reset_line
        + "Shutdown\n"
        "Saving trajectory to frames.txt ...\n"
        "There are 1 maps in the atlas\n"
        "  Map 0 has 2 KFs\n"
        "End of saving trajectory to frames.txt ...\n"
        "Saving keyframe trajectory to keyframes.txt ...\n",
    )
    rows = (
        "1000000000 0 0 0 0 0 0 1\n"
        "1033333333 0.1 0 0 0 0 0 1\n"
        "1066666666 0.02 0 0 0 0 0 1\n"
    )
    write(frame, rows)
    write(
        keyframe,
        "1000000000 0 0 0 0 0 0 1\n"
        "1066666666 0.02 0 0 0 0 0 1\n",
    )
    write(
        reference,
        "%YAML:1.0\n"
        'format: "ovrs-closed-loop-reference-v1"\n'
        'reference_type: "COLOCATED_START_END"\n'
        "estimator_consumed_reference: false\n"
        "physical_path_completed: true\n"
        'method: "camera returned against the same rigid stop"\n'
        "position_tolerance_m: 0.03\n"
        "orientation_tolerance_deg: 2\n",
    )
    return adapter, log, frame, keyframe, reference


def ensure_orb_evaluator_inputs(adapter: Path) -> tuple[Path, Path, Path, Path]:
    backend_runner = adapter.parent / "backend-runner"
    backend_library = adapter.parent / "libORB_SLAM3.so"
    vocabulary = adapter.parent / "vocabulary.txt"
    settings = adapter.parent / "orbslam3_settings.yaml"
    if not backend_runner.exists():
        write(backend_runner, b"fixture backend")
    if not vocabulary.exists():
        write(vocabulary, "fixture vocabulary\n")
    if not backend_library.exists():
        write(backend_library, b"fixture backend library")
    if not settings.exists():
        write(settings, "%YAML:1.0\n")
    return backend_runner, backend_library, vocabulary, settings


def add_evaluator_input_atlas_provenance(adapter: Path, atlas: Path) -> None:
    backend_runner, backend_library, vocabulary, _ = (
        ensure_orb_evaluator_inputs(adapter)
    )
    manifest = adapter.parent / "input_atlas.osa.manifest.yaml"
    write(
        manifest,
        "%YAML:1.0\n"
        'format: "ovrs-orbslam3-atlas-manifest-v1"\n'
        'state: "TRACKING_GATE_PASS_ATLAS_RELOAD_UNVERIFIED"\n'
        f'atlas_file: "{atlas.name}"\n'
        f'atlas_sha256: "{hashlib.sha256(atlas.read_bytes()).hexdigest()}"\n'
        'backend_name: "ORB_SLAM3"\n'
        f'backend_commit: "{ORB_BACKEND_COMMIT}"\n'
        f'backend_patch_sha256: "{ORB_PATCH_SHA256}"\n'
        f'backend_pin_sha256: "{"5" * 64}"\n'
        'camera_serial: "test-d435i"\n'
        'calibration_state: "BOOTSTRAP_UNVERIFIED"\n'
        f'imucam_config_sha256: "{"6" * 64}"\n'
        f'imu_config_sha256: "{"7" * 64}"\n'
        'source_camera_fps: "30"\n'
        'adapted_camera_fps: "30"\n'
        'camera_stride: "1"\n'
        'camera_time_offset_policy: "calibrated"\n'
        'calibrated_camera_imu_time_offset_ns: "0"\n'
        'applied_camera_imu_time_offset_ns: "0"\n'
        f'backend_runner_sha256: "{hashlib.sha256(backend_runner.read_bytes()).hexdigest()}"\n'
        f'backend_library_sha256: "{hashlib.sha256(backend_library.read_bytes()).hexdigest()}"\n'
        f'vocabulary_sha256: "{hashlib.sha256(vocabulary.read_bytes()).hexdigest()}"\n'
        'coordinate_frame_policy: "ORB_SLAM3_ATLAS_WORLD"\n'
        "ground_truth_consumed_by_estimator: false\n",
    )
    with adapter.open("a", encoding="utf-8") as handle:
        handle.write(
            f'backend_commit: "{ORB_BACKEND_COMMIT}"\n'
            f'backend_patch_sha256: "{ORB_PATCH_SHA256}"\n'
            f'backend_pin_sha256: "{"5" * 64}"\n'
            'camera_serial: "test-d435i"\n'
            'calibration_state: "BOOTSTRAP_UNVERIFIED"\n'
            f'imucam_config_sha256: "{"6" * 64}"\n'
            f'imu_config_sha256: "{"7" * 64}"\n'
            'source_camera_fps: "30"\n'
            'camera_stride: "1"\n'
            'camera_time_offset_policy: "calibrated"\n'
            'calibrated_camera_imu_time_offset_ns: "0"\n'
            'applied_camera_imu_time_offset_ns: "0"\n'
            'atlas_input_manifest_file: "input_atlas.osa.manifest.yaml"\n'
            "atlas_input_manifest_sha256: "
            f'"{hashlib.sha256(manifest.read_bytes()).hexdigest()}"\n'
        )


def run_evaluator(
    inputs: tuple[Path, ...],
    output: Path,
    *extra_args: str,
) -> subprocess.CompletedProcess[str]:
    adapter, log, frame, keyframe, _ = inputs
    backend_runner, backend_library, vocabulary, settings = (
        ensure_orb_evaluator_inputs(adapter)
    )
    return subprocess.run(
        (
            sys.executable,
            str(ORB_EVALUATOR),
            "--adapter-manifest",
            str(adapter),
            "--settings",
            str(settings),
            "--backend-log",
            str(log),
            "--frame-trajectory",
            str(frame),
            "--keyframe-trajectory",
            str(keyframe),
            "--backend-runner",
            str(backend_runner),
            "--backend-library",
            str(backend_library),
            "--vocabulary",
            str(vocabulary),
            "--backend-exit-status",
            "0",
            "--output",
            str(output),
            *extra_args,
        ),
        check=False,
        text=True,
        capture_output=True,
    )


def make_orb_live_run(
    root: Path,
    reset: bool = False,
    tracking_loss: bool = False,
    tracking_gap: bool = False,
    visual_support_loss: bool = False,
    pose_rate_loss: bool = False,
    legacy_schema: bool = False,
    legacy_visual_support_schema: bool = False,
) -> tuple[Path, Path, Path, Path, Path, Path, Path]:
    if sum(
        (
            reset,
            tracking_loss,
            tracking_gap,
            visual_support_loss,
            pose_rate_loss,
        )
    ) > 1:
        raise ValueError("live failure fixtures are mutually exclusive")
    if legacy_schema and legacy_visual_support_schema:
        raise ValueError("legacy live schemas are mutually exclusive")
    run = root / "live-run"
    run.mkdir()
    settings = run / "resolved_orbslam3_settings.yaml"
    patch = root / "backend.patch"
    backend_library = root / "libORB_SLAM3.so"
    live_executable = root / "ovrs_orbslam3_live"
    vocabulary = root / "ORBvoc.txt"
    write(settings, '%YAML:1.0\nOVRS.Mode: "fixture"\n')
    write(patch, "fixture patch\n")
    write(backend_library, b"fixture backend library")
    write(live_executable, b"fixture live executable")
    write(vocabulary, "fixture vocabulary\n")
    patch_hash = hashlib.sha256(patch.read_bytes()).hexdigest()
    library_hash = hashlib.sha256(backend_library.read_bytes()).hexdigest()
    executable_hash = hashlib.sha256(live_executable.read_bytes()).hexdigest()
    vocabulary_hash = hashlib.sha256(vocabulary.read_bytes()).hexdigest()
    settings_hash = hashlib.sha256(settings.read_bytes()).hexdigest()
    backend_commit = "1" * 40
    bundle_format = (
        "ovrs-orbslam3-live-bundle-v4"
        if legacy_schema
        else "ovrs-orbslam3-live-bundle-v5"
        if legacy_visual_support_schema
        else "ovrs-orbslam3-live-bundle-v6"
    )
    runtime_provenance_format = (
        "ovrs-orbslam3-live-runtime-provenance-v5"
        if legacy_schema
        else "ovrs-orbslam3-live-runtime-provenance-v6"
        if legacy_visual_support_schema
        else "ovrs-orbslam3-live-runtime-provenance-v7"
    )
    minimum_visual_support_line = (
        "" if legacy_schema else "minimum_tracked_map_points: 50\n"
    )
    pose_rate_lines = (
        ""
        if legacy_schema or legacy_visual_support_schema
        else
        "maximum_pose_linear_speed_m_s: 2.000000\n"
        "maximum_pose_angular_speed_rad_s: 6.000000\n"
    )
    trajectory_policy = (
        "startup_imu_pass_post_inertial_ba2_stable_tracking_continuous_"
        "bounded_preacceptance_resets_zero_postacceptance_resets_"
        "no_postacceptance_map_correction"
        if legacy_schema
        else (
            "startup_imu_pass_post_inertial_ba2_stable_tracking_minimum_"
            "visual_support_continuous_bounded_preacceptance_resets_"
            "zero_postacceptance_resets_no_postacceptance_map_correction"
            if legacy_visual_support_schema
            else
            "startup_imu_pass_post_inertial_ba2_stable_tracking_minimum_"
            "visual_support_bounded_pose_rates_continuous_"
            "bounded_preacceptance_resets_zero_postacceptance_resets_"
            "no_postacceptance_map_correction"
        )
    )
    backend_pin = root / "backend.yaml"
    write(
        backend_pin,
        "%YAML:1.0\n"
        'format: "ovrs-orbslam3-backend-pin-v1"\n'
        'backend_name: "ORB_SLAM3"\n'
        f'commit: "{backend_commit}"\n'
        f'patch_sha256: "{patch_hash}"\n'
        "live_minimum_tracked_map_points: 50\n"
        "live_maximum_pose_linear_speed_m_s: 2.0\n"
        "live_maximum_pose_angular_speed_rad_s: 6.0\n"
        "live_maximum_preacceptance_map_resets: 5\n"
        "live_startup_stationary_seconds: 1.0\n"
        "live_startup_stationary_timeout_seconds: 10.0\n"
        "live_startup_maximum_acceleration_stddev_m_s2: 0.25\n"
        "live_startup_maximum_gyro_magnitude_rad_s: 0.1\n"
        "live_maximum_input_stall_seconds: 1.0\n",
    )
    live_manifest = root / "live_manifest.yaml"
    write(
        live_manifest,
        "%YAML:1.0\n"
        f'format: "{bundle_format}"\n'
        'state: "PREPARED_NOT_RUN"\n'
        'integration: "PURE_ORB_SLAM3_STEREO_INERTIAL"\n'
        "openvins_pose_consumed: false\n"
        "global_correction_fed_to_openvins: false\n"
        'camera_serial: "test-d435i"\n'
        'calibration_state: "BOOTSTRAP_UNVERIFIED"\n'
        f'backend_commit: "{backend_commit}"\n'
        f'backend_patch_sha256: "{patch_hash}"\n'
        "camera_stride: 3\n"
        "orb_camera_fps: 3\n"
        "imu_init_acceleration_threshold_m_s2: 0.5\n"
        "minimum_stable_inertial_seconds: 1.0\n"
        "maximum_tracking_interval_factor: 3.0\n"
        "maximum_tracking_interval_seconds: 1.0\n"
        f"{minimum_visual_support_line}"
        f"{pose_rate_lines}"
        "maximum_preacceptance_map_resets: 5\n"
        "gravity_m_s2: 9.80665\n"
        "startup_maximum_gravity_error_m_s2: 2.0\n"
        "startup_stationary_seconds: 1.0\n"
        "startup_stationary_timeout_seconds: 10.0\n"
        "startup_maximum_acceleration_stddev_m_s2: 0.25\n"
        "startup_maximum_gyro_magnitude_rad_s: 0.1\n"
        "maximum_input_stall_seconds: 1.0\n"
        "camera_imu_time_offset_s: -0.0049\n"
        f'settings_sha256: "{settings_hash}"\n',
    )
    write(
        run / "source_live_manifest.yaml",
        live_manifest.read_bytes(),
    )
    manifest_hash = hashlib.sha256(live_manifest.read_bytes()).hexdigest()
    launch_provenance = run / "launch_provenance.yaml"
    write(
        launch_provenance,
        "%YAML:1.0\n"
        'format: "ovrs-orbslam3-live-launch-provenance-v1"\n'
        'state: "LAUNCHED_NOT_CAPTURE_VALIDATED"\n'
        'integration: "PURE_ORB_SLAM3_STEREO_INERTIAL"\n'
        "openvins_pose_consumed: false\n"
        "global_correction_fed_to_openvins: false\n"
        f'source_fingerprint: "{"2" * 64}"\n'
        f'live_executable_sha256_at_start: "{executable_hash}"\n'
        f'backend_library_sha256_at_start: "{library_hash}"\n'
        f'vocabulary_sha256_at_start: "{vocabulary_hash}"\n'
        f'settings_sha256_at_start: "{settings_hash}"\n'
        f'live_bundle_manifest_sha256_at_start: "{manifest_hash}"\n',
    )
    launch_hash = hashlib.sha256(
        launch_provenance.read_bytes()
    ).hexdigest()
    write(
        run / "run_metadata.yaml",
        "%YAML:1.0\n"
        'mode: "experimental_pure_orbslam3_live"\n'
        "runtime_provenance_format: "
        f'"{runtime_provenance_format}"\n'
        "openvins_pose_consumed: false\n"
        "global_correction_fed_to_openvins: false\n"
        'calibration_state: "BOOTSTRAP_UNVERIFIED"\n'
        'camera_serial: "test-d435i"\n'
        f'backend_commit: "{backend_commit}"\n'
        f'backend_patch_sha256: "{patch_hash}"\n'
        f'backend_library_sha256_at_build: "{library_hash}"\n'
        f'backend_library_sha256_at_start: "{library_hash}"\n'
        f'live_executable_sha256_at_start: "{executable_hash}"\n'
        f'vocabulary_sha256_at_start: "{vocabulary_hash}"\n'
        f'settings_sha256_at_start: "{settings_hash}"\n'
        f'live_bundle_manifest_sha256_at_start: "{manifest_hash}"\n'
        f'launch_provenance_sha256: "{launch_hash}"\n'
        f'source_fingerprint_at_start: "{"2" * 64}"\n'
        "camera_stride: 3\n"
        "camera_imu_time_offset_s: -0.004900\n"
        "imu_init_acceleration_threshold_m_s2: 0.500000\n"
        "minimum_stable_inertial_seconds: 1.000000\n"
        "maximum_tracking_interval_factor: 3.000000\n"
        "maximum_tracking_interval_seconds: 1.000000\n"
        f"{minimum_visual_support_line}"
        f"{pose_rate_lines}"
        "maximum_preacceptance_map_resets: 5\n"
        "gravity_m_s2: 9.806650\n"
        "startup_maximum_gravity_error_m_s2: 2.000000\n"
        "startup_stationary_seconds: 1.000000\n"
        "startup_stationary_timeout_seconds: 10.000000\n"
        "startup_maximum_acceleration_stddev_m_s2: 0.250000\n"
        "startup_maximum_gyro_magnitude_rad_s: 0.100000\n"
        "maximum_input_stall_seconds: 1.000000\n"
        "trajectory_acceptance_policy: "
        f'"{trajectory_policy}"\n'
        "closed_loop_reference_start_policy: "
        '"post_acceptance_gate_open_operator_cue"\n'
        "visual_tracking_trajectory_is_diagnostic_only: true\n",
    )
    write(
        run / "device_report.yaml",
        "%YAML:1.0\n"
        'device_name: "Intel RealSense D435I"\n'
        'serial: "test-d435i"\n'
        'firmware: "fixture"\n'
        'usb_type: "3.2"\n',
    )
    for name in (
        "resolved_estimator_config.yaml",
        "requested_stream_config.yaml",
        "resolved_stream_config.yaml",
    ):
        write(run / name, "%YAML:1.0\nfixture: true\n")

    tracking_fields = [
            "timestamp_s",
            "state",
            "tracked_keypoints",
            "tracked_map_points",
            "tracking_latency_ms",
            "imu_batch",
            "startup_imu_gate_passed",
            "inertial_initialized",
            "inertial_ba2_finished",
            "active_map_reset_count",
            "active_map_change_index",
            "reset_pending",
    ]
    if not legacy_schema and not legacy_visual_support_schema:
        tracking_fields.extend(
            (
                "pose_tx_m",
                "pose_ty_m",
                "pose_tz_m",
                "pose_qx",
                "pose_qy",
                "pose_qz",
                "pose_qw",
            )
        )
    tracking_fields.extend(
        (
            "stable_gate_elapsed_s",
            "trajectory_candidate_accepted",
        )
    )
    tracking_header = ",".join(tracking_fields)
    if reset:
        tracking_rows = (
            "0.1,NOT_INITIALIZED,100,0,2.0,5,1,0,0,0,0,0,0.0,0\n"
            "0.2,OK,100,50,2.0,5,1,1,1,0,0,0,0.0,0\n"
            "0.3,OK,100,50,2.0,5,1,0,0,1,0,1,0.0,0\n"
            "0.4,OK,100,50,2.0,5,1,1,0,1,0,0,0.0,0\n"
        )
    elif tracking_loss:
        tracking_rows = (
            "0.1,NOT_INITIALIZED,100,0,2.0,5,1,0,0,0,0,0,0.0,0\n"
            "0.2,OK,100,50,2.0,5,1,1,1,0,0,0,0.0,0\n"
            "1.2,OK,100,50,2.0,5,1,1,1,0,0,0,1.0,1\n"
            "1.3,LOST,0,0,2.0,5,1,1,1,0,0,0,0.0,0\n"
        )
    elif tracking_gap:
        tracking_rows = (
            "0.1,NOT_INITIALIZED,100,0,2.0,5,1,0,0,0,0,0,0.0,0\n"
            "0.2,OK,100,50,2.0,5,1,1,1,0,0,0,0.0,0\n"
            "1.2,OK,100,50,2.0,5,1,1,1,0,0,0,1.0,1\n"
            "2.4,OK,100,50,2.0,5,1,1,1,0,0,0,0.0,0\n"
        )
    elif visual_support_loss:
        tracking_rows = (
            "0.1,NOT_INITIALIZED,100,0,2.0,5,1,0,0,0,0,0,0.0,0\n"
            "0.2,OK,100,50,2.0,5,1,1,1,0,0,0,0.0,0\n"
            "1.2,OK,100,50,2.0,5,1,1,1,0,0,0,1.0,1\n"
            "1.3,OK,100,49,2.0,5,1,1,1,0,0,0,0.0,0\n"
        )
    elif pose_rate_loss:
        tracking_rows = (
            "0.1,NOT_INITIALIZED,100,0,2.0,5,1,0,0,0,0,0,0.0,0\n"
            "0.2,OK,100,50,2.0,5,1,1,1,0,0,0,0.0,0\n"
            "1.2,OK,100,50,2.0,5,1,1,1,0,0,0,1.0,1\n"
            "1.3,OK,100,50,2.0,5,1,1,1,0,0,0,0.0,0\n"
        )
    else:
        tracking_rows = (
            "0.1,NOT_INITIALIZED,100,0,2.0,5,1,0,0,0,0,0,0.0,0\n"
            "0.2,OK,100,50,2.0,5,1,1,1,0,0,0,0.0,0\n"
            "1.2,OK,100,50,2.0,5,1,1,1,0,0,0,1.0,1\n"
            "1.3,OK,100,50,2.0,5,1,1,1,0,0,0,1.1,1\n"
            "1.4,OK,100,50,2.0,5,1,1,1,0,0,0,1.2,1\n"
            "2.2,OK,100,50,2.0,5,1,1,1,0,0,0,2.0,1\n"
            "2.3,OK,100,50,2.0,5,1,1,1,0,0,0,2.1,1\n"
            "2.4,OK,100,50,2.0,5,1,1,1,0,0,0,2.2,1\n"
        )
    positions = {
        "0.2": 0.0,
        "0.3": 0.02,
        "0.4": 0.02,
        "1.2": 0.0,
        "1.3": (
            0.3
            if pose_rate_loss
            else 0.02
            if visual_support_loss
            else 0.001
        ),
        "1.4": 0.0,
        "2.2": 0.019,
        "2.3": 0.021,
        "2.4": 0.020,
    }
    position_text = {
        timestamp: f"{position:g}"
        for timestamp, position in positions.items()
    }
    position_text.update(
        {
            "1.3": (
                "0.3"
                if pose_rate_loss
                else "0.02"
                if visual_support_loss
                else "0.001"
            ),
            "2.2": "0.019",
            "2.3": "0.021",
            "2.4": "0.020",
        }
    )
    raw_tracking_values = [
        row.split(",") for row in tracking_rows.strip().splitlines()
    ]
    if not legacy_schema and not legacy_visual_support_schema:
        for row in raw_tracking_values:
            pose_fields = (
                [
                    f"{positions[row[0]]:.9f}",
                    "0.000000000",
                    "0.000000000",
                    "0.000000000",
                    "0.000000000",
                    "0.000000000",
                    "1.000000000",
                ]
                if row[1] in ("OK", "OK_KLT")
                else [""] * 7
            )
            row[12:12] = pose_fields
        tracking_rows = "\n".join(
            ",".join(row) for row in raw_tracking_values
        ) + "\n"
    write(
        run / "live_tracking_states.csv",
        tracking_header + "\n" + tracking_rows,
    )
    imu_header = ",".join(
        (
            "timestamp_s",
            "imu_batch",
            "mean_ax_m_s2",
            "mean_ay_m_s2",
            "mean_az_m_s2",
            "mean_accel_magnitude_m_s2",
            "mean_gyro_magnitude_rad_s",
            "max_gyro_magnitude_rad_s",
            "orb_init_accel_delta_m_s2",
            "orb_init_accel_threshold_m_s2",
            "orb_init_accel_gate_passed",
            "inertial_initialized",
            "inertial_ba2_finished",
            "active_map_reset_count",
            "active_map_change_index",
            "reset_pending",
        )
    )
    tracking_values = [
        row.split(",") for row in tracking_rows.strip().splitlines()
    ]
    maximum_observed_interval = max(
        float(current[0]) - float(previous[0])
        for current, previous in zip(
            tracking_values[1:], tracking_values[:-1]
        )
    )
    imu_rows = []
    for row in tracking_values:
        imu_rows.append(
            ",".join(
                (
                    row[0],
                    row[5],
                    "0.0",
                    "0.0",
                    "9.8",
                    "9.8",
                    "0.1",
                    "0.2",
                    "",
                    "",
                    "",
                    row[7],
                    row[8],
                    row[9],
                    row[10],
                    row[11],
                )
            )
        )
    write(
        run / "live_imu_excitation.csv",
        imu_header + "\n" + "\n".join(imu_rows) + "\n",
    )
    visual_rows = "".join(
        f"{row[0]} {position_text[row[0]]} 0 0 0 0 0 1\n"
        for row in tracking_values
        if row[1] in ("OK", "OK_KLT")
    )
    write(run / "live_visual_tracking_trajectory_tum.txt", visual_rows)
    visual_support_failure_after_acceptance_count = 0
    pose_rate_failure_count = 0
    pose_rate_failure_after_acceptance_count = 0
    maximum_observed_pose_linear_speed_m_s = 0.0
    if reset:
        write(run / "live_camera_trajectory_candidate_tum.txt", "")
        write(run / "INCOMPLETE", "ORB-SLAM3 live run incomplete.\n")
        run_state = "EXPERIMENTAL_RUN_FAILED"
        accepted_count = 0
        candidate_count = 0
        ever_ba2 = "true"
        final_ba2 = "false"
        reset_count = 1
        pending_observed = "true"
        inertial_regressions = 1
        ba2_regressions = 1
        accepted_file = ""
        rejected_file = "live_camera_trajectory_candidate_tum.txt"
        runtime_failure = "ORB-SLAM3 active-map reset was requested"
        visual_count = 3
        lost_count = 0
        tracking_loss_after_acceptance_count = 0
        tracking_gap_after_acceptance_count = 0
        discontinuity = "false"
        acceptance_started = "false"
        last_tracking_state = "OK"
    elif tracking_loss:
        write(
            run / "live_camera_trajectory_candidate_tum.txt",
            "1.2 0 0 0 0 0 0 1\n",
        )
        write(run / "INCOMPLETE", "ORB-SLAM3 live run incomplete.\n")
        run_state = "EXPERIMENTAL_RUN_FAILED"
        accepted_count = 0
        candidate_count = 1
        ever_ba2 = "true"
        final_ba2 = "true"
        reset_count = 0
        pending_observed = "false"
        inertial_regressions = 0
        ba2_regressions = 0
        accepted_file = ""
        rejected_file = "live_camera_trajectory_candidate_tum.txt"
        runtime_failure = (
            "ORB-SLAM3 tracking was lost after trajectory acceptance"
        )
        visual_count = 2
        lost_count = 1
        tracking_loss_after_acceptance_count = 1
        tracking_gap_after_acceptance_count = 0
        discontinuity = "true"
        acceptance_started = "true"
        last_tracking_state = "LOST"
    elif tracking_gap:
        write(
            run / "live_camera_trajectory_candidate_tum.txt",
            "1.2 0 0 0 0 0 0 1\n",
        )
        write(run / "INCOMPLETE", "ORB-SLAM3 live run incomplete.\n")
        run_state = "EXPERIMENTAL_RUN_FAILED"
        accepted_count = 0
        candidate_count = 1
        ever_ba2 = "true"
        final_ba2 = "true"
        reset_count = 0
        pending_observed = "false"
        inertial_regressions = 0
        ba2_regressions = 0
        accepted_file = ""
        rejected_file = "live_camera_trajectory_candidate_tum.txt"
        runtime_failure = (
            "ORB-SLAM3 frame interval exceeded the continuity limit "
            "after trajectory acceptance"
        )
        visual_count = 3
        lost_count = 0
        tracking_loss_after_acceptance_count = 0
        tracking_gap_after_acceptance_count = 1
        discontinuity = "true"
        acceptance_started = "true"
        last_tracking_state = "OK"
        visual_support_failure_after_acceptance_count = 0
    elif visual_support_loss:
        write(
            run / "live_camera_trajectory_candidate_tum.txt",
            "1.2 0 0 0 0 0 0 1\n",
        )
        write(run / "INCOMPLETE", "ORB-SLAM3 live run incomplete.\n")
        run_state = "EXPERIMENTAL_RUN_FAILED"
        accepted_count = 0
        candidate_count = 1
        ever_ba2 = "true"
        final_ba2 = "true"
        reset_count = 0
        pending_observed = "false"
        inertial_regressions = 0
        ba2_regressions = 0
        accepted_file = ""
        rejected_file = "live_camera_trajectory_candidate_tum.txt"
        runtime_failure = (
            "ORB-SLAM3 visual map support fell below the canonical "
            "continuity floor after trajectory acceptance"
        )
        visual_count = 3
        lost_count = 0
        tracking_loss_after_acceptance_count = 0
        tracking_gap_after_acceptance_count = 0
        discontinuity = "true"
        acceptance_started = "true"
        last_tracking_state = "OK"
        visual_support_failure_after_acceptance_count = 1
        maximum_observed_pose_linear_speed_m_s = 0.2
    elif pose_rate_loss:
        write(
            run / "live_camera_trajectory_candidate_tum.txt",
            "1.2 0 0 0 0 0 0 1\n",
        )
        write(run / "INCOMPLETE", "ORB-SLAM3 live run incomplete.\n")
        run_state = "EXPERIMENTAL_RUN_FAILED"
        accepted_count = 0
        candidate_count = 1
        ever_ba2 = "true"
        final_ba2 = "true"
        reset_count = 0
        pending_observed = "false"
        inertial_regressions = 0
        ba2_regressions = 0
        accepted_file = ""
        rejected_file = "live_camera_trajectory_candidate_tum.txt"
        runtime_failure = (
            "ORB-SLAM3 pose rate exceeded the canonical continuity "
            "envelope after trajectory acceptance"
        )
        visual_count = 3
        lost_count = 0
        tracking_loss_after_acceptance_count = 0
        tracking_gap_after_acceptance_count = 0
        discontinuity = "true"
        acceptance_started = "true"
        last_tracking_state = "OK"
        pose_rate_failure_count = 1
        pose_rate_failure_after_acceptance_count = 1
        maximum_observed_pose_linear_speed_m_s = 3.0
    else:
        write(
            run / "live_camera_trajectory_tum.txt",
            "1.2 0 0 0 0 0 0 1\n"
            "1.3 0.001 0 0 0 0 0 1\n"
            "1.4 0 0 0 0 0 0 1\n"
            "2.2 0.019 0 0 0 0 0 1\n"
            "2.3 0.021 0 0 0 0 0 1\n"
            "2.4 0.020 0 0 0 0 0 1\n",
        )
        run_state = "EXPERIMENTAL_RUN_COMPLETE"
        accepted_count = 6
        candidate_count = 6
        ever_ba2 = "true"
        final_ba2 = "true"
        reset_count = 0
        pending_observed = "false"
        inertial_regressions = 0
        ba2_regressions = 0
        accepted_file = "live_camera_trajectory_tum.txt"
        rejected_file = ""
        runtime_failure = ""
        visual_count = 7
        lost_count = 0
        tracking_loss_after_acceptance_count = 0
        tracking_gap_after_acceptance_count = 0
        discontinuity = "false"
        acceptance_started = "true"
        last_tracking_state = "OK"
        visual_support_failure_after_acceptance_count = 0
        maximum_observed_pose_linear_speed_m_s = 0.02375
    write(
        run / "run_summary.yaml",
        "%YAML:1.0\n"
        f'state: "{run_state}"\n'
        f"submitted_stereo: {len(tracking_values)}\n"
        f"visual_pose_count: {visual_count}\n"
        f"candidate_pose_count: {candidate_count}\n"
        f"accepted_pose_count: {accepted_count}\n"
        f"lost_frame_count: {lost_count}\n"
        "capture_duration_s: 3.000000\n"
        "shutdown_duration_s: 0.100000\n"
        "maximum_input_stall_seconds: 1.000000\n"
        "maximum_observed_stereo_wall_gap_seconds: 0.100000\n"
        "maximum_observed_imu_wall_gap_seconds: 0.050000\n"
        "input_stall_detected: false\n"
        "tracking_loss_after_acceptance_count: "
        f"{tracking_loss_after_acceptance_count}\n"
        "tracking_gap_after_acceptance_count: "
        f"{tracking_gap_after_acceptance_count}\n"
        "minimum_tracked_map_points: 50\n"
        "visual_support_failure_after_acceptance_count: "
        f"{visual_support_failure_after_acceptance_count}\n"
        "maximum_pose_linear_speed_m_s: 2.000000\n"
        "maximum_observed_pose_linear_speed_m_s: "
        f"{maximum_observed_pose_linear_speed_m_s:.9f}\n"
        "maximum_pose_angular_speed_rad_s: 6.000000\n"
        "maximum_observed_pose_angular_speed_rad_s: 0.000000000\n"
        f"pose_rate_gate_failure_count: {pose_rate_failure_count}\n"
        "pose_rate_failure_after_acceptance_count: "
        f"{pose_rate_failure_after_acceptance_count}\n"
        "maximum_tracking_interval_seconds: 1.000000\n"
        "maximum_observed_tracking_interval_seconds: "
        f"{maximum_observed_interval:.6f}\n"
        "tracking_latency_samples: "
        f"{len(tracking_values)}\n"
        "tracking_latency_mean_ms: 2.000000\n"
        "tracking_latency_maximum_ms: 2.000000\n"
        "tracking_frame_budget_ms: 333.333333\n"
        "tracking_latency_frame_budget_miss_count: 0\n"
        "tracking_latency_frame_budget_miss_ratio: 0.000000\n"
        "dropped_imu: 0\n"
        "dropped_stereo: 0\n"
        "rejected_nonmonotonic: 0\n"
        "stereo_without_imu_coverage: 0\n"
        "stereo_discarded_on_shutdown: 0\n"
        'startup_imu_gate_state: "PASSED"\n'
        "startup_imu_gate_passed: true\n"
        "startup_imu_gate_samples: 201\n"
        "startup_imu_gate_rejected_dynamic_windows: 0\n"
        "startup_imu_gate_window_duration_s: 1.000000\n"
        "startup_imu_acceleration_magnitude_mean_m_s2: 9.800000\n"
        "startup_imu_acceleration_magnitude_stddev_m_s2: 0.010000\n"
        "startup_imu_maximum_gyro_magnitude_rad_s: 0.010000\n"
        "startup_imu_gravity_error_m_s2: 0.006650\n"
        "gravity_m_s2: 9.806650\n"
        "startup_maximum_gravity_error_m_s2: 2.000000\n"
        "startup_stationary_seconds: 1.000000\n"
        "startup_stationary_timeout_seconds: 10.000000\n"
        "startup_maximum_acceleration_stddev_m_s2: 0.250000\n"
        "startup_maximum_gyro_magnitude_rad_s: 0.100000\n"
        "ever_inertial_initialized: true\n"
        "inertial_initialized: true\n"
        f"ever_inertial_ba2_finished: {ever_ba2}\n"
        f"inertial_ba2_finished: {final_ba2}\n"
        f"active_map_reset_count: {reset_count}\n"
        "maximum_preacceptance_map_resets: 5\n"
        f"preacceptance_map_reset_count: {reset_count}\n"
        "postacceptance_map_reset_count: 0\n"
        "preacceptance_reset_limit_exceeded: false\n"
        "active_map_change_index: 0\n"
        "map_change_after_acceptance: false\n"
        f"pending_reset_observed: {pending_observed}\n"
        "pending_reset_after_acceptance_observed: false\n"
        "reset_pending_at_shutdown: false\n"
        f"inertial_regression_count: {inertial_regressions}\n"
        f"inertial_ba2_regression_count: {ba2_regressions}\n"
        f"trajectory_acceptance_started: {acceptance_started}\n"
        f"trajectory_discontinuity_detected: {discontinuity}\n"
        f'accepted_trajectory_file: "{accepted_file}"\n'
        f'rejected_candidate_trajectory_file: "{rejected_file}"\n'
        f'last_tracking_state: "{last_tracking_state}"\n'
        f'runtime_failure: "{runtime_failure}"\n',
    )
    return (
        run,
        live_manifest,
        backend_pin,
        patch,
        backend_library,
        live_executable,
        vocabulary,
    )


def run_live_evaluator(
    inputs: tuple[Path, Path, Path, Path, Path, Path, Path],
    output: Path,
    *extra_args: str,
) -> subprocess.CompletedProcess[str]:
    run, manifest, pin, patch, library, executable, vocabulary = inputs
    return subprocess.run(
        (
            sys.executable,
            str(ORB_LIVE_EVALUATOR),
            "--run-dir",
            str(run),
            "--live-bundle-manifest",
            str(manifest),
            "--backend-pin",
            str(pin),
            "--backend-patch",
            str(patch),
            "--backend-library",
            str(library),
            "--live-executable",
            str(executable),
            "--vocabulary",
            str(vocabulary),
            "--output",
            str(output),
            *extra_args,
        ),
        check=False,
        text=True,
        capture_output=True,
    )


def update_tracking_pose(
    run: Path,
    timestamp_s: str,
    *,
    translation: tuple[str, str, str] | None = None,
    quaternion: tuple[str, str, str, str] | None = None,
) -> None:
    path = run / "live_tracking_states.csv"
    rows = list(
        csv.DictReader(path.read_text(encoding="utf-8").splitlines())
    )
    matches = [row for row in rows if row["timestamp_s"] == timestamp_s]
    if len(matches) != 1:
        raise ValueError(
            f"tracking fixture lacks unique timestamp {timestamp_s}"
        )
    row = matches[0]
    if translation is not None:
        for field, value in zip(
            ("pose_tx_m", "pose_ty_m", "pose_tz_m"), translation
        ):
            row[field] = value
    if quaternion is not None:
        for field, value in zip(
            ("pose_qx", "pose_qy", "pose_qz", "pose_qw"), quaternion
        ):
            row[field] = value
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def update_summary_scalar(run: Path, key: str, value: str) -> None:
    path = run / "run_summary.yaml"
    prefix = f"{key}: "
    lines = path.read_text(encoding="utf-8").splitlines()
    matches = [
        index for index, line in enumerate(lines) if line.startswith(prefix)
    ]
    if len(matches) != 1:
        raise ValueError(f"run summary fixture lacks unique key {key}")
    lines[matches[0]] = prefix + value
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_live_closed_loop_reference(
    path: Path,
    minimum_endpoint_samples: int = 3,
    endpoint_window_seconds: float = 0.2,
    minimum_path_duration_seconds: float = 1.0,
    minimum_path_excursion_m: float = 0.015,
) -> None:
    write(
        path,
        "%YAML:1.0\n"
        'format: "ovrs-closed-loop-reference-v2"\n'
        'reference_type: "COLOCATED_START_END"\n'
        "estimator_consumed_reference: false\n"
        "physical_path_completed: true\n"
        'method: "camera returned against the same rigid stop"\n'
        "position_tolerance_m: 0.03\n"
        "orientation_tolerance_deg: 2\n"
        f"endpoint_window_seconds: {endpoint_window_seconds}\n"
        f"minimum_endpoint_samples: {minimum_endpoint_samples}\n"
        "maximum_endpoint_position_spread_m: 0.005\n"
        "maximum_endpoint_orientation_spread_deg: 0.5\n"
        f"minimum_path_duration_seconds: {minimum_path_duration_seconds}\n"
        f"minimum_path_excursion_m: {minimum_path_excursion_m}\n",
    )


def make_excitation_export(
    root: Path, name: str, gyro: float, excited: bool
) -> Path:
    export = root / name
    imu = export / "mav0" / "imu0" / "data.csv"
    header = (
        "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],"
        "w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],"
        "a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n"
    )
    rows = []
    for index in range(60):
        timestamp = index * 50_000_000
        analysis_bin = index // 2
        ax = float(analysis_bin % 2) if excited else 0.0
        rows.append(f"{timestamp},{gyro},0,0,{ax},0,9.8\n")
    write(imu, header + "".join(rows))
    manifest = export / "benchmark_manifest.yaml"
    write(
        manifest,
        "%YAML:1.0\n"
        'format: "ovrs-vislam-benchmark-v1"\n'
        'state: "EXPORTED_NOT_EVALUATED"\n'
        'estimation_policy: "MARKERLESS_STEREO_INERTIAL"\n'
        f'source_dataset_name: "{name}"\n'
        'camera_serial: "test-d435i"\n'
        'imu_frame: "GYROSCOPE_FRAME"\n'
        "synchronized_imu_rows: 60\n"
        f'imu0_data_csv_sha256: "{hashlib.sha256(imu.read_bytes()).hexdigest()}"\n',
    )
    return export


def make_excitation_reference_result(root: Path, reference: Path) -> Path:
    adapter = root / "reference_adapter" / "adapter_manifest.yaml"
    reference_manifest = reference / "benchmark_manifest.yaml"
    write(
        adapter,
        "%YAML:1.0\n"
        f'source_benchmark_manifest_sha256: "{hashlib.sha256(reference_manifest.read_bytes()).hexdigest()}"\n',
    )
    result = adapter.parent / "experiment_pass.yaml"
    write(
        result,
        "%YAML:1.0\n"
        'format: "ovrs-orbslam3-result-v1"\n'
        "tracking_gate_passed: true\n"
        "imu_map_resets: 0\n"
        "local_map_tracking_failures: 0\n"
        "created_maps: 1\n"
        "final_atlas_maps: 1\n"
        "viba1_completions: 1\n"
        "viba2_completions: 1\n"
        f'adapter_manifest_sha256: "{hashlib.sha256(adapter.read_bytes()).hexdigest()}"\n',
    )
    return result


def run_excitation_evaluator(
    candidate: Path,
    reference: Path,
    reference_result: Path,
    output: Path,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        (
            sys.executable,
            str(ORB_CAPTURE_EXCITATION),
            "--candidate",
            str(candidate),
            "--reference",
            str(reference),
            "--reference-result",
            str(reference_result),
            "--output",
            str(output),
            "--analysis-rate-hz",
            "10",
            "--minimum-duration-ratio",
            "0.95",
            "--minimum-acceleration-delta-count-ratio",
            "0.85",
            "--maximum-gyro-mean-ratio",
            "1.25",
        ),
        check=False,
        text=True,
        capture_output=True,
    )


class EvaluateOrbslam3CaptureExcitationTests(unittest.TestCase):
    def test_passes_candidate_matching_provenance_bound_reference(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            reference = make_excitation_export(root, "reference", 0.1, True)
            candidate = make_excitation_export(root, "candidate", 0.1, True)
            result = make_excitation_reference_result(root, reference)
            output = root / "excitation.yaml"

            completed = run_excitation_evaluator(
                candidate, reference, result, output
            )

            self.assertEqual(completed.returncode, 0, completed.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn(
                'state: "CAPTURE_EXCITATION_GATE_PASS_VISUAL_NOT_EVALUATED"',
                manifest,
            )
            self.assertIn("acceleration_gate_passed: true", manifest)
            self.assertIn("gyro_gate_passed: true", manifest)
            self.assertIn("visual_quality_evaluated: false", manifest)
            self.assertIn("reference_result_sha256:", manifest)

    def test_rotation_heavy_low_excitation_candidate_fails(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            reference = make_excitation_export(root, "reference", 0.1, True)
            candidate = make_excitation_export(root, "candidate", 0.3, False)
            result = make_excitation_reference_result(root, reference)
            output = root / "excitation.yaml"

            completed = run_excitation_evaluator(
                candidate, reference, result, output
            )

            self.assertEqual(completed.returncode, 5, completed.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn(
                'state: "CAPTURE_EXCITATION_GATE_FAILED"', manifest
            )
            self.assertIn("acceleration_gate_passed: false", manifest)
            self.assertIn("gyro_gate_passed: false", manifest)

    def test_rejects_failed_reference_result(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            reference = make_excitation_export(root, "reference", 0.1, True)
            candidate = make_excitation_export(root, "candidate", 0.1, True)
            result = make_excitation_reference_result(root, reference)
            result.write_text(
                result.read_text(encoding="utf-8").replace(
                    "tracking_gate_passed: true",
                    "tracking_gate_passed: false",
                ),
                encoding="utf-8",
            )

            completed = run_excitation_evaluator(
                candidate, reference, result, root / "excitation.yaml"
            )

            self.assertEqual(completed.returncode, 4)
            self.assertIn("not a passing tracking reference", completed.stderr)


class ExportVislamBenchmarkTests(unittest.TestCase):
    def test_exports_canonical_stereo_imu_and_provenance(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset = make_dataset(root)
            output = root / "export"

            result = run_export(dataset, output)

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertFalse((output / "INCOMPLETE").exists())
            manifest = (output / "benchmark_manifest.yaml").read_text(
                encoding="utf-8"
            )
            self.assertIn('state: "EXPORTED_NOT_EVALUATED"', manifest)
            self.assertIn(
                'estimation_policy: "MARKERLESS_STEREO_INERTIAL"', manifest
            )
            self.assertIn("stereo_pairs: 2", manifest)
            self.assertIn("source_stereo_pairs: 2", manifest)
            self.assertIn(
                "skipped_leading_stereo_outside_imu_range: 0", manifest
            )
            self.assertIn("synchronized_imu_rows: 3", manifest)
            camera_index = (
                output / "mav0" / "cam0" / "data.csv"
            ).read_text(encoding="utf-8")
            self.assertIn("1000100000,1000100000.png", camera_index)
            self.assertTrue(
                (
                    output
                    / "mav0"
                    / "cam1"
                    / "data"
                    / "1100100000.png"
                ).is_file()
            )
            imu = (output / "mav0" / "imu0" / "data.csv").read_text(
                encoding="utf-8"
            )
            self.assertIn("1050000000,0.2,0.3,0.4,1.1,2.1,9.6", imu)
            self.assertTrue(
                (output / "ovrs_metadata" / "device_report.yaml").is_file()
            )
            self.assertIn("cam0_data_csv_sha256:", manifest)
            self.assertIn("imu0_data_csv_sha256:", manifest)

    def test_rejects_nonzero_capture_integrity_counter(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset = make_dataset(root)
            summary = dataset / "recording_summary.yaml"
            summary.write_text(
                summary.read_text(encoding="utf-8").replace(
                    "stereo_queue_drops: 0", "stereo_queue_drops: 1"
                ),
                encoding="utf-8",
            )
            output = root / "export"

            result = run_export(dataset, output)

            self.assertEqual(result.returncode, 4)
            self.assertIn("stereo_queue_drops must be zero", result.stderr)
            self.assertFalse(output.exists())

    def test_skips_stereo_outside_synchronized_imu_range(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset = make_dataset(root)
            imu_path = dataset / "imu" / "synchronized.csv"
            imu_path.write_text(
                imu_path.read_text(encoding="utf-8").replace(
                    "0.990,990.0", "1.050,990.0"
                ).replace(
                    "1.050,1050.0", "1.075,1050.0"
                ),
                encoding="utf-8",
            )
            output = root / "export"

            result = run_export(dataset, output)

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = (output / "benchmark_manifest.yaml").read_text(
                encoding="utf-8"
            )
            self.assertIn("source_stereo_pairs: 2", manifest)
            self.assertIn("stereo_pairs: 1", manifest)
            self.assertIn(
                "skipped_leading_stereo_outside_imu_range: 1", manifest
            )

    def test_failure_after_output_creation_keeps_incomplete_marker(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset = make_dataset(root)
            camera_csv = dataset / "cam1" / "data.csv"
            camera_csv.write_text(
                camera_csv.read_text(encoding="utf-8").replace(
                    "1.100200000", "1.200200000"
                ),
                encoding="utf-8",
            )
            output = root / "export"

            result = run_export(dataset, output)

            self.assertEqual(result.returncode, 4)
            self.assertTrue((output / "INCOMPLETE").is_file())
            self.assertIn("timestamps exceed 2.0 ms tolerance", result.stderr)


class PrepareOrbslam3BenchmarkTests(unittest.TestCase):
    def test_pinned_patch_guards_pre_keyframe_imu_query(self) -> None:
        patch = ORB_PATCH.read_text(encoding="utf-8")
        self.assertEqual(
            hashlib.sha256(ORB_PATCH.read_bytes()).hexdigest(),
            ORB_PATCH_SHA256,
        )
        self.assertIn("mpCurrentKeyFrame(nullptr)", patch)
        self.assertIn("bool System::IsImuInitialized()", patch)
        self.assertIn("return mpAtlas && mpAtlas->isImuInitialized();", patch)
        self.assertIn("GetActiveMapResetCount()", patch)
        self.assertIn("bool System::IsResetPending()", patch)
        self.assertIn("IMU.InitAccelerationThreshold", patch)
        self.assertIn("mbShutdownInProgress", patch)
        self.assertIn("mShutdownCondition.wait", patch)
        self.assertIn(
            "mptViewer->get_id() == std::this_thread::get_id()", patch
        )
        self.assertIn("circular wait", patch)
        self.assertIn("mptViewer = nullptr", patch)
        self.assertIn("diff --git a/src/Frame.cc b/src/Frame.cc", patch)
        self.assertIn("Establish the mutex", patch)
        self.assertIn("invariant before any such return", patch)
        self.assertEqual(patch.count("+    mpMutexImu = new std::mutex();"), 5)

    def test_pinned_realsense_patch_fails_before_null_usb_context(self) -> None:
        patch = (
            ROOT / "patches" / "librealsense-rsusb-gyro-sensitivity.patch"
        ).read_text(encoding="utf-8")
        self.assertIn("if(sts != LIBUSB_SUCCESS)", patch)
        self.assertIn("libusb_init failed:", patch)
        self.assertIn("if(count < 0)", patch)
        self.assertIn("libusb_get_device_list failed:", patch)

    def test_live_bundle_binds_serial_rate_offset_and_backend(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            imucam, imu = make_calibration(root)
            for path in (imucam, imu):
                path.write_text(
                    path.read_text(encoding="utf-8").replace(
                        "test-d435i", "123456"
                    ),
                    encoding="utf-8",
                )
            estimator = root / "estimator.yaml"
            write(
                estimator,
                "%YAML:1.0\n"
                'calibration_state: "BOOTSTRAP_UNVERIFIED"\n'
                'calibrated_serial: "123456"\n'
                "gravity_mag: 9.80665\n"
                "max_accel_bias_m_s2: 2.0\n"
                'relative_config_imucam: "imucam.yaml"\n'
                'relative_config_imu: "imu.yaml"\n',
            )
            stream = root / "streams.yaml"
            write(
                stream,
                "%YAML:1.0\n"
                "width: 848\nheight: 480\ncamera_fps: 90\n"
                "gyro_fps: 200\naccelerometer_fps: 250\n"
                "gyro_scale_factor: 1\n"
                "motion_correction_enabled: true\n"
                "global_time_enabled: true\n",
            )
            output = root / "live_bundle"
            result = subprocess.run(
                (
                    sys.executable,
                    str(ORB_LIVE_PREPARER),
                    "--estimator-config",
                    str(estimator),
                    "--stream-config",
                    str(stream),
                    "--output",
                    str(output),
                ),
                check=False,
                text=True,
                capture_output=True,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            settings = (output / "orbslam3_live_settings.yaml").read_text(
                encoding="utf-8"
            )
            manifest = (output / "live_manifest.yaml").read_text(
                encoding="utf-8"
            )
            self.assertIn('OVRS.CalibratedSerial: "123456"', settings)
            self.assertIn("OVRS.CameraStride: 3", settings)
            self.assertIn(
                "OVRS.CameraImuTimeOffsetSeconds: -0.0049", settings
            )
            self.assertIn("Camera.fps: 30", settings)
            self.assertIn("IMU.InitAccelerationThreshold: 0.5", settings)
            self.assertIn(
                "OVRS.MinimumStableInertialSeconds: 3.0", settings
            )
            self.assertIn(
                "OVRS.MaximumTrackingIntervalSeconds: 0.1", settings
            )
            self.assertIn(
                "OVRS.MaximumTrackingIntervalFactor: 3.0", settings
            )
            self.assertIn("OVRS.GravityMagnitudeMps2: 9.80665", settings)
            self.assertIn(
                "OVRS.StartupMaximumGravityErrorMps2: 2.0", settings
            )
            self.assertIn("OVRS.StartupStationarySeconds: 1.0", settings)
            self.assertIn(
                "OVRS.StartupStationaryTimeoutSeconds: 10.0", settings
            )
            self.assertIn(
                "OVRS.MaximumInputStallSeconds: 1.0", settings
            )
            self.assertIn(
                f'OVRS.BackendCommit: "{ORB_BACKEND_COMMIT}"', settings
            )
            self.assertIn(
                'integration: "PURE_ORB_SLAM3_STEREO_INERTIAL"', manifest
            )
            self.assertIn(
                "minimum_stable_inertial_seconds: 3.0", manifest
            )
            self.assertIn(
                "maximum_tracking_interval_factor: 3.0", manifest
            )
            self.assertIn(
                "maximum_tracking_interval_seconds: 0.1", manifest
            )
            self.assertFalse((output / "INCOMPLETE").exists())

    def test_writes_explicit_transforms_and_shifted_camera_clock(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            benchmark = root / "benchmark"
            export_result = run_export(make_dataset(root), benchmark)
            self.assertEqual(export_result.returncode, 0, export_result.stderr)
            imucam, imu = make_calibration(root)
            output = root / "orb"

            result = run_adapter(benchmark, imucam, imu, output)

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertFalse((output / "INCOMPLETE").exists())
            manifest = (output / "adapter_manifest.yaml").read_text(
                encoding="utf-8"
            )
            self.assertIn('state: "PREPARED_NOT_RUN"', manifest)
            self.assertIn(
                'camera_time_offset_policy: "calibrated"', manifest
            )
            self.assertIn(
                "calibrated_camera_imu_time_offset_ns: -4900000", manifest
            )
            self.assertIn(
                "applied_camera_imu_time_offset_ns: -4900000", manifest
            )
            self.assertIn("camera_stride: 1", manifest)
            self.assertIn("source_camera_fps: 90", manifest)
            self.assertIn("adapted_camera_fps: 90", manifest)
            self.assertIn(
                'imu_transform_contract: "T_b_c1=T_imu_cam0"', manifest
            )
            timestamps = (output / "timestamps.txt").read_text(
                encoding="utf-8"
            )
            self.assertEqual(
                timestamps.splitlines(),
                ["995200000", "1095200000"],
            )
            self.assertTrue(
                (
                    output
                    / "sequence"
                    / "mav0"
                    / "cam0"
                    / "data"
                    / "995200000.png"
                ).is_file()
            )
            settings = (output / "orbslam3_settings.yaml").read_text(
                encoding="utf-8"
            )
            self.assertIn("Camera.fps: 90", settings)
            self.assertIn("0.095", settings)

    def test_zero_time_offset_is_explicit_diagnostic_policy(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            benchmark = root / "benchmark"
            export_result = run_export(make_dataset(root), benchmark)
            self.assertEqual(export_result.returncode, 0, export_result.stderr)
            imucam, imu = make_calibration(root)
            output = root / "orb"

            result = run_adapter(
                benchmark,
                imucam,
                imu,
                output,
                "--camera-time-offset-policy",
                "zero",
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = (output / "adapter_manifest.yaml").read_text(
                encoding="utf-8"
            )
            self.assertIn('camera_time_offset_policy: "zero"', manifest)
            self.assertIn(
                "calibrated_camera_imu_time_offset_ns: -4900000", manifest
            )
            self.assertIn(
                "applied_camera_imu_time_offset_ns: 0", manifest
            )
            self.assertEqual(
                (output / "timestamps.txt").read_text(
                    encoding="utf-8"
                ).splitlines(),
                ["1000100000", "1100100000"],
            )

    def test_camera_stride_retains_all_imu_rows(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            dataset = make_dataset(root)
            for camera, timestamp_offset in (
                ("cam0", Decimal("0")),
                ("cam1", Decimal("0.0002")),
            ):
                index = dataset / camera / "data.csv"
                with index.open("a", encoding="utf-8") as handle:
                    for frame, timestamp in ((12, "1.200"), (13, "1.300")):
                        adjusted = Decimal(timestamp) + timestamp_offset
                        handle.write(
                            f"{adjusted:.9f},{adjusted * 1000:.1f},"
                            f"{frame},{frame}.png\n"
                        )
                        write(
                            dataset / camera / "data" / f"{frame}.png",
                            f"fixture-{frame}".encode(),
                        )
            imu_path = dataset / "imu" / "synchronized.csv"
            with imu_path.open("a", encoding="utf-8") as handle:
                handle.write(
                    "1.200,1200.0,0.4,0.5,0.6,1.3,2.3,9.4,0.001\n"
                    "1.301,1301.0,0.5,0.6,0.7,1.4,2.4,9.3,0.001\n"
                )
            benchmark = root / "benchmark"
            export_result = run_export(dataset, benchmark)
            self.assertEqual(export_result.returncode, 0, export_result.stderr)
            imucam, imu = make_calibration(root)
            output = root / "orb"

            result = run_adapter(
                benchmark,
                imucam,
                imu,
                output,
                "--camera-stride",
                "3",
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = (output / "adapter_manifest.yaml").read_text(
                encoding="utf-8"
            )
            self.assertIn("source_stereo_pairs: 4", manifest)
            self.assertIn("adapted_stereo_pairs: 2", manifest)
            self.assertIn("camera_stride: 3", manifest)
            self.assertIn("skipped_pairs_by_camera_stride: 2", manifest)
            self.assertIn("source_camera_fps: 90", manifest)
            self.assertIn("adapted_camera_fps: 30", manifest)
            self.assertIn(
                f'backend_patch_sha256: "{ORB_PATCH_SHA256}"',
                manifest,
            )
            settings = (output / "orbslam3_settings.yaml").read_text(
                encoding="utf-8"
            )
            self.assertIn("Camera.fps: 30", settings)
            self.assertEqual(
                (output / "timestamps.txt").read_text(
                    encoding="utf-8"
                ).splitlines(),
                ["995200000", "1295200000"],
            )
            source_imu = (
                benchmark / "mav0" / "imu0" / "data.csv"
            ).read_bytes()
            adapted_imu = (
                output / "sequence" / "mav0" / "imu0" / "data.csv"
            ).read_bytes()
            self.assertEqual(adapted_imu, source_imu)

    def test_rejects_changed_orbslam3_patch_pin(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            benchmark = root / "benchmark"
            export_result = run_export(make_dataset(root), benchmark)
            self.assertEqual(export_result.returncode, 0, export_result.stderr)
            imucam, imu = make_calibration(root)
            backend_pin = root / "orbslam3_backend.yaml"
            pin_text = (
                ROOT / "config" / "research" / "orbslam3_backend.yaml"
            ).read_text(encoding="utf-8")
            write(
                backend_pin,
                pin_text.replace(
                    ORB_PATCH_SHA256,
                    "0" * 64,
                ),
            )

            result = run_adapter(
                benchmark,
                imucam,
                imu,
                root / "orb",
                "--backend-pin",
                str(backend_pin),
            )

            self.assertEqual(result.returncode, 4)
            self.assertIn("backend patch hash differs", result.stderr)

    def test_rejects_mutated_neutral_index(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            benchmark = root / "benchmark"
            export_result = run_export(make_dataset(root), benchmark)
            self.assertEqual(export_result.returncode, 0, export_result.stderr)
            index = benchmark / "mav0" / "cam0" / "data.csv"
            index.write_text(
                index.read_text(encoding="utf-8").replace(
                    "1000100000", "1000100001", 1
                ),
                encoding="utf-8",
            )
            imucam, imu = make_calibration(root)

            result = run_adapter(benchmark, imucam, imu, root / "orb")

            self.assertEqual(result.returncode, 4)
            self.assertIn("cam0_data_csv_sha256 differs", result.stderr)

    def test_prepares_map_build_and_multi_session_atlas_settings(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            benchmark = root / "benchmark"
            export_result = run_export(make_dataset(root), benchmark)
            self.assertEqual(export_result.returncode, 0, export_result.stderr)
            imucam, imu = make_calibration(root)
            map_build = root / "map-build"

            build_result = run_adapter(
                benchmark,
                imucam,
                imu,
                map_build,
                "--save-atlas-name",
                "room_revision_1",
            )

            self.assertEqual(build_result.returncode, 0, build_result.stderr)
            build_settings = (
                map_build / "orbslam3_settings.yaml"
            ).read_text(encoding="utf-8")
            self.assertIn(
                'System.SaveAtlasToFile: "room_revision_1"',
                build_settings,
            )
            build_manifest = (
                map_build / "adapter_manifest.yaml"
            ).read_text(encoding="utf-8")
            self.assertIn('atlas_mode: "MAP_BUILD"', build_manifest)
            self.assertIn(
                'atlas_output_file: "room_revision_1.osa"',
                build_manifest,
            )

            source_atlas = root / "source.osa"
            write(source_atlas, b"fixture atlas")
            source_manifest = write_source_atlas_manifest(
                source_atlas, imucam, imu
            )
            merge = root / "merge"
            merge_result = run_adapter(
                benchmark,
                imucam,
                imu,
                merge,
                "--load-atlas",
                str(source_atlas),
                "--save-atlas-name",
                "room_revision_2",
            )

            self.assertEqual(merge_result.returncode, 0, merge_result.stderr)
            self.assertEqual(
                (merge / "input_atlas.osa").read_bytes(), b"fixture atlas"
            )
            self.assertEqual(
                (merge / "input_atlas.osa.manifest.yaml").read_bytes(),
                source_manifest.read_bytes(),
            )
            merge_settings = (
                merge / "orbslam3_settings.yaml"
            ).read_text(encoding="utf-8")
            self.assertIn(
                'System.LoadAtlasFromFile: "input_atlas"',
                merge_settings,
            )
            self.assertIn(
                'System.SaveAtlasToFile: "room_revision_2"',
                merge_settings,
            )
            merge_manifest = (
                merge / "adapter_manifest.yaml"
            ).read_text(encoding="utf-8")
            self.assertIn(
                'atlas_mode: "MULTI_SESSION_MERGE"', merge_manifest
            )
            self.assertIn("atlas_input_sha256:", merge_manifest)
            self.assertIn("atlas_input_manifest_sha256:", merge_manifest)

    def test_rejects_atlas_without_companion_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            benchmark = root / "benchmark"
            export_result = run_export(make_dataset(root), benchmark)
            self.assertEqual(export_result.returncode, 0, export_result.stderr)
            imucam, imu = make_calibration(root)
            source_atlas = root / "source.osa"
            write(source_atlas, b"fixture atlas")

            result = run_adapter(
                benchmark,
                imucam,
                imu,
                root / "merge",
                "--load-atlas",
                str(source_atlas),
                "--save-atlas-name",
                "room_revision_2",
            )

            self.assertEqual(result.returncode, 4)
            self.assertIn("companion manifest is missing", result.stderr)

    def test_rejects_atlas_manifest_from_another_camera(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            benchmark = root / "benchmark"
            export_result = run_export(make_dataset(root), benchmark)
            self.assertEqual(export_result.returncode, 0, export_result.stderr)
            imucam, imu = make_calibration(root)
            source_atlas = root / "source.osa"
            write(source_atlas, b"fixture atlas")
            source_manifest = write_source_atlas_manifest(
                source_atlas, imucam, imu
            )
            source_manifest.write_text(
                source_manifest.read_text(encoding="utf-8").replace(
                    'camera_serial: "test-d435i"',
                    'camera_serial: "different-d435i"',
                ),
                encoding="utf-8",
            )

            result = run_adapter(
                benchmark,
                imucam,
                imu,
                root / "merge",
                "--load-atlas",
                str(source_atlas),
                "--save-atlas-name",
                "room_revision_2",
            )

            self.assertEqual(result.returncode, 4)
            self.assertIn(
                "input atlas manifest camera_serial must be test-d435i",
                result.stderr,
            )

    def test_rejects_atlas_load_without_new_revision_name(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            benchmark = root / "benchmark"
            export_result = run_export(make_dataset(root), benchmark)
            self.assertEqual(export_result.returncode, 0, export_result.stderr)
            imucam, imu = make_calibration(root)
            source_atlas = root / "source.osa"
            write(source_atlas, b"fixture atlas")

            result = run_adapter(
                benchmark,
                imucam,
                imu,
                root / "merge",
                "--load-atlas",
                str(source_atlas),
            )

            self.assertEqual(result.returncode, 2)
            self.assertIn(
                "--load-atlas requires --save-atlas-name", result.stderr
            )


class EvaluateOrbslam3RunTests(unittest.TestCase):
    def test_records_tracking_loop_and_independent_return_reference(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_run(root)
            output = root / "result.yaml"

            result = run_evaluator(
                inputs,
                output,
                "--closed-loop-reference",
                str(inputs[-1]),
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn(
                'state: "TRACKING_PASS_LOOP_CORRECTION_NOT_REFERENCE_VALIDATED"',
                manifest,
            )
            self.assertIn("tracking_gate_passed: true", manifest)
            self.assertIn("loop_candidates: 2", manifest)
            self.assertIn("rejected_loop_candidates: 1", manifest)
            self.assertIn("applied_loop_corrections: 1", manifest)
            self.assertIn("independent_reference_present: true", manifest)
            self.assertIn(
                'reference_evaluation_state: '
                '"RETURN_RESIDUAL_WITHIN_RECORDED_PLACEMENT_TOLERANCE"',
                manifest,
            )
            self.assertIn(
                "return_position_consistent_with_reference_tolerance: true",
                manifest,
            )
            self.assertIn(
                "return_orientation_consistent_with_reference_tolerance: true",
                manifest,
            )
            self.assertIn("estimated_return_displacement_m: 0.02", manifest)
            self.assertIn("backend_log_sha256:", manifest)

    def test_reset_fails_tracking_gate_without_hiding_loop_count(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_run(root, reset=True)
            output = root / "result.yaml"

            result = run_evaluator(inputs, output)

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn('state: "TRACKING_GATE_FAILED"', manifest)
            self.assertIn("tracking_gate_passed: false", manifest)
            self.assertIn("imu_map_resets: 1", manifest)
            self.assertIn("applied_loop_corrections: 1", manifest)

    def test_records_loaded_atlas_merge_without_claiming_false_merge_rate(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_run(root)
            adapter = inputs[0]
            with adapter.open("a", encoding="utf-8") as handle:
                handle.write(
                    'atlas_mode: "MULTI_SESSION_MERGE"\n'
                    'atlas_input_file: "input_atlas.osa"\n'
                    "atlas_input_sha256: "
                    '"4f9e74c8b5e5d0e3c3aedf1a3d4393b6dbf99cbc'
                    'aaef9c609ead65a9054ae6fc"\n'
                    'atlas_output_file: "revision_2.osa"\n'
                )
            write(root / "input_atlas.osa", b"fixture atlas")
            write(root / "revision_2.osa", b"merged atlas")
            add_evaluator_input_atlas_provenance(
                adapter, root / "input_atlas.osa"
            )
            write(
                inputs[3],
                "1066666666 0.02 0 0 0 0 0 1\n"
                "1000000000 0 0 0 0 0 0 1\n",
            )
            with inputs[1].open("a", encoding="utf-8") as handle:
                handle.write(
                    "Initialization of Atlas from file: input_atlas\n"
                    "End to load the save binary file\n"
                    "*Merge detected\n"
                    "Merge finished!\n"
                    "End to write save binary file\n"
                )

            result = run_evaluator(inputs, root / "result.yaml")

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = (root / "result.yaml").read_text(encoding="utf-8")
            self.assertIn(
                'atlas_mode: "MULTI_SESSION_MERGE"', manifest
            )
            self.assertIn("atlas_load_completed: true", manifest)
            self.assertIn("atlas_save_completed: true", manifest)
            self.assertIn("atlas_merge_established: true", manifest)
            self.assertIn("map_merge_completions: 1", manifest)
            self.assertIn(
                'keyframe_timestamp_order_policy: '
                '"MULTI_SESSION_MAP_ID_ORDER"',
                manifest,
            )
            self.assertIn(
                'false_map_merge_evaluation_state: '
                '"NOT_EVALUATED_WITHOUT_REFERENCE"',
                manifest,
            )
            self.assertIn(
                "parent_atlas_reload_verified_by_this_run: true", manifest
            )

    def test_multi_session_keyframes_may_exceed_current_session_frames(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_run(root)
            adapter = inputs[0]
            with adapter.open("a", encoding="utf-8") as handle:
                handle.write(
                    'atlas_mode: "MULTI_SESSION_MERGE"\n'
                    'atlas_input_file: "input_atlas.osa"\n'
                    "atlas_input_sha256: "
                    '"4f9e74c8b5e5d0e3c3aedf1a3d4393b6dbf99cbc'
                    'aaef9c609ead65a9054ae6fc"\n'
                    'atlas_output_file: "revision_2.osa"\n'
                )
            write(root / "input_atlas.osa", b"fixture atlas")
            write(root / "revision_2.osa", b"merged atlas")
            add_evaluator_input_atlas_provenance(
                adapter, root / "input_atlas.osa"
            )
            write(
                inputs[3],
                "1000000000 0 0 0 0 0 0 1\n"
                "1033333333 0.1 0 0 0 0 0 1\n"
                "1066666666 0.02 0 0 0 0 0 1\n"
                "1000000000 0 0 0 0 0 0 1\n",
            )
            with inputs[1].open("a", encoding="utf-8") as handle:
                handle.write(
                    "Initialization of Atlas from file: input_atlas\n"
                    "End to load the save binary file\n"
                    "*Merge detected\n"
                    "Merge finished!\n"
                    "End to write save binary file\n"
                )
            log_text = inputs[1].read_text(encoding="utf-8").replace(
                "Map 0 has 2 KFs", "Map 0 has 4 KFs"
            )
            write(inputs[1], log_text)

            result = run_evaluator(inputs, root / "result.yaml")

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn(
                "keyframe_trajectory_rows: 4",
                (root / "result.yaml").read_text(encoding="utf-8"),
            )

    def test_rejects_non_normalized_trajectory_quaternion(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_run(root)
            frame = inputs[2]
            frame.write_text(
                frame.read_text(encoding="utf-8").replace(
                    "0 0 0 1\n", "0 0 0 2\n", 1
                ),
                encoding="utf-8",
            )

            result = run_evaluator(inputs, root / "result.yaml")

            self.assertEqual(result.returncode, 4)
            self.assertIn("quaternion is not normalized", result.stderr)
            self.assertFalse((root / "result.yaml").exists())

    def test_runner_captures_process_status_before_evaluation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            adapter = root / "adapter"
            adapter.mkdir()
            make_orb_run(adapter)
            with (adapter / "adapter_manifest.yaml").open(
                "a", encoding="utf-8"
            ) as handle:
                backend_pin = (
                    ROOT / "config" / "research" / "orbslam3_backend.yaml"
                )
                handle.write(
                    'atlas_mode: "MAP_BUILD"\n'
                    'atlas_output_file: "room_revision_1.osa"\n'
                    f'backend_commit: "{ORB_BACKEND_COMMIT}"\n'
                    f'backend_patch_sha256: "{ORB_PATCH_SHA256}"\n'
                    "backend_pin_sha256: "
                    f'"{hashlib.sha256(backend_pin.read_bytes()).hexdigest()}"\n'
                )
            write(adapter / "orbslam3_settings.yaml", "%YAML:1.0\n")
            write(adapter / "timestamps.txt", "1000000000\n")
            (adapter / "sequence").mkdir()
            adapted_cam0 = adapter / "sequence" / "mav0" / "cam0" / "data.csv"
            adapted_cam1 = adapter / "sequence" / "mav0" / "cam1" / "data.csv"
            adapted_imu = adapter / "sequence" / "mav0" / "imu0" / "data.csv"
            write(adapted_cam0, "#timestamp [ns],filename\n")
            write(adapted_cam1, "#timestamp [ns],filename\n")
            write(adapted_imu, "#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z\n")
            with (adapter / "adapter_manifest.yaml").open(
                "a", encoding="utf-8"
            ) as handle:
                handle.write(
                    "settings_sha256: "
                    f'"{hashlib.sha256((adapter / "orbslam3_settings.yaml").read_bytes()).hexdigest()}"\n'
                    "timestamps_sha256: "
                    f'"{hashlib.sha256((adapter / "timestamps.txt").read_bytes()).hexdigest()}"\n'
                    "adapted_cam0_data_csv_sha256: "
                    f'"{hashlib.sha256(adapted_cam0.read_bytes()).hexdigest()}"\n'
                    "adapted_cam1_data_csv_sha256: "
                    f'"{hashlib.sha256(adapted_cam1.read_bytes()).hexdigest()}"\n'
                    "imu0_data_csv_sha256: "
                    f'"{hashlib.sha256(adapted_imu.read_bytes()).hexdigest()}"\n'
                )
            vocabulary = root / "vocabulary.txt"
            write(vocabulary, "fixture vocabulary\n")
            fake_runner = root / "fake_orbslam3.py"
            write(
                fake_runner,
                "#!/usr/bin/env python3\n"
                "import sys\n"
                "from pathlib import Path\n"
                "run_id = sys.argv[-1]\n"
                "Path('room_revision_1.osa').write_bytes(b'atlas')\n"
                "Path(f'f_{run_id}.txt').write_text(\n"
                "    '1000000000 0 0 0 0 0 0 1\\n'\n"
                "    '1033333333 0.1 0 0 0 0 0 1\\n'\n"
                "    '1066666666 0.02 0 0 0 0 0 1\\n'\n"
                ")\n"
                "Path(f'kf_{run_id}.txt').write_text(\n"
                "    '1000000000 0 0 0 0 0 0 1\\n'\n"
                "    '1066666666 0.02 0 0 0 0 0 1\\n'\n"
                ")\n"
                "print('New Map created with 100 points')\n"
                "print('end VIBA 1')\n"
                "print('end VIBA 2')\n"
                "print('Shutdown')\n"
                "print('End to write save binary file')\n"
                "print('Saving trajectory to frames.txt ...')\n"
                "print('There are 1 maps in the atlas')\n"
                "print('  Map 0 has 2 KFs')\n"
                "print('End of saving trajectory to frames.txt ...')\n"
                "print('Saving keyframe trajectory to keyframes.txt ...')\n",
            )
            fake_runner.chmod(0o755)
            backend_library = root / "libORB_SLAM3.so"
            write(backend_library, b"fixture backend library")

            result = subprocess.run(
                (
                    sys.executable,
                    str(ORB_RUNNER),
                    "--adapter-dir",
                    str(adapter),
                    "--runner",
                    str(fake_runner),
                    "--backend-library",
                    str(backend_library),
                    "--vocabulary",
                    str(vocabulary),
                    "--run-id",
                    "fixture",
                ),
                check=False,
                text=True,
                capture_output=True,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(
                (adapter / "backend_fixture.status").read_text(
                    encoding="utf-8"
                ),
                "0\n",
            )
            manifest = (
                adapter / "experiment_fixture.yaml"
            ).read_text(encoding="utf-8")
            self.assertIn(
                'state: "TRACKING_PASS_NO_LOOP_CORRECTION"', manifest
            )
            self.assertIn(
                'backend_exit_status_source: "CAPTURED_FILE"', manifest
            )
            self.assertIn("backend_exit_status_file_sha256:", manifest)
            self.assertIn(
                'backend_library_binding_state: '
                '"NON_ELF_RUNNER_NOT_LINK_ATTESTED"',
                manifest,
            )
            atlas_manifest = adapter / "room_revision_1.osa.manifest.yaml"
            self.assertTrue(atlas_manifest.is_file())
            atlas_manifest_text = atlas_manifest.read_text(encoding="utf-8")
            self.assertIn(
                'format: "ovrs-orbslam3-atlas-manifest-v1"',
                atlas_manifest_text,
            )
            self.assertIn(
                'state: "TRACKING_GATE_PASS_ATLAS_RELOAD_UNVERIFIED"',
                atlas_manifest_text,
            )
            self.assertIn("vocabulary_sha256:", atlas_manifest_text)


class RunOrbslam3LiveLauncherTests(unittest.TestCase):
    def test_help_documents_parent_shell_isolation(self) -> None:
        result = subprocess.run(
            ("bash", str(ORB_LIVE_LAUNCHER), "--help"),
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("failed SLAM gate cannot enable errexit", result.stdout)
        self.assertIn("--allow-unverified-calibration", result.stdout)

    def test_invalid_serial_fails_before_hardware_access(self) -> None:
        result = subprocess.run(
            (
                "bash",
                str(ORB_LIVE_LAUNCHER),
                "--serial",
                "not-a-serial",
                "--allow-unverified-calibration",
            ),
            check=False,
            capture_output=True,
            text=True,
        )

        self.assertEqual(result.returncode, 2)
        self.assertIn("--serial must be a numeric", result.stderr)


class EvaluateOrbslam3LiveRunTests(unittest.TestCase):
    def test_input_stall_is_a_continuity_gate_failure(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            summary = inputs[0] / "run_summary.yaml"
            summary.write_text(
                summary.read_text(encoding="utf-8")
                .replace(
                    "maximum_observed_stereo_wall_gap_seconds: 0.100000",
                    "maximum_observed_stereo_wall_gap_seconds: 1.100000",
                )
                .replace(
                    "input_stall_detected: false",
                    "input_stall_detected: true",
                ),
                encoding="utf-8",
            )
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn('state: "LIVE_GATE_FAILED"', manifest)
            self.assertIn("INPUT_STALL_DETECTED", manifest)
            self.assertIn("input_stall_detected: true", manifest)

    def test_recomputes_live_gate_and_closed_loop_return(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            reference = root / "closed_loop_reference.yaml"
            write_live_closed_loop_reference(reference)
            output = root / "evaluation.yaml"

            result = run_live_evaluator(
                inputs,
                output,
                "--closed-loop-reference",
                str(reference),
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn(
                'state: "LIVE_GATE_PASS_CLOSED_LOOP_REFERENCE_'
                'CONSISTENT_NOT_ACCURACY_VALIDATED"',
                manifest,
            )
            self.assertIn("live_gate_passed: true", manifest)
            self.assertIn("live_continuity_gate_passed: true", manifest)
            self.assertIn(
                'format: "ovrs-orbslam3-live-evaluation-v9"', manifest
            )
            self.assertIn(
                'pose_rate_contract_state: "PINNED_LIVE_AND_RECOMPUTED"',
                manifest,
            )
            self.assertIn(
                'pose_artifact_binding_state: "TRACKING_TO_VISUAL_BOUND"',
                manifest,
            )
            self.assertIn("canonical_trajectory_rows: 6", manifest)
            self.assertIn(
                "canonical_endpoint_displacement_m: 0.020000000",
                manifest,
            )
            self.assertIn("closed_loop_reference_passed: true", manifest)
            self.assertIn(
                "closed_loop_start_window_samples: 3", manifest
            )
            self.assertIn(
                "closed_loop_end_window_samples: 3", manifest
            )
            self.assertIn(
                "closed_loop_robust_endpoint_displacement_m: 0.020000000",
                manifest,
            )
            self.assertIn(
                "closed_loop_maximum_estimated_path_excursion_m: "
                "0.021000000",
                manifest,
            )
            self.assertIn("openvins_pose_consumed: false", manifest)
            self.assertIn(
                'flight_control_integration_state: '
                '"NOT_IMPLEMENTED_OUT_OF_SCOPE"',
                manifest,
            )
            self.assertIn("live_tracking_states_sha256:", manifest)
            self.assertIn(
                'live_executable_binding_state: "CAPTURE_TIME_ATTESTED"',
                manifest,
            )
            self.assertIn("vocabulary_sha256:", manifest)

    def test_legacy_v4_live_bundle_remains_re_evaluable(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root, legacy_schema=True)
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn(
                'state: "LIVE_GATE_PASS_CONTINUITY_NOT_ACCURACY_VALIDATED"',
                manifest,
            )
            self.assertIn("minimum_tracked_map_points: 0", manifest)
            self.assertIn(
                "visual_support_failure_after_acceptance_count: 0",
                manifest,
            )
            self.assertIn(
                'pose_rate_contract_state: "LEGACY_NOT_EVALUATED"',
                manifest,
            )
            self.assertIn(
                'pose_artifact_binding_state: '
                '"LEGACY_TRACKING_POSE_UNAVAILABLE"',
                manifest,
            )

    def test_legacy_v5_visual_support_bundle_remains_re_evaluable(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(
                root, legacy_visual_support_schema=True
            )
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn(
                'state: "LIVE_GATE_PASS_CONTINUITY_NOT_ACCURACY_VALIDATED"',
                manifest,
            )
            self.assertIn("minimum_tracked_map_points: 50", manifest)
            self.assertIn(
                'pose_rate_contract_state: "LEGACY_NOT_EVALUATED"',
                manifest,
            )

    def test_bundle_v6_tracking_pose_mismatch_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            update_tracking_pose(
                inputs[0], "1.3", translation=("0.002", "0", "0")
            )
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 4)
            self.assertIn(
                "tracking versus visual pose 3 translation component 1 "
                "differs",
                result.stderr,
            )
            self.assertFalse(output.exists())

    def test_bundle_v6_tracking_orientation_mismatch_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            run = inputs[0]
            quaternion = (
                "0",
                "0",
                f"{math.sin(0.01):.9f}",
                f"{math.cos(0.01):.9f}",
            )
            update_tracking_pose(run, "2.4", quaternion=quaternion)
            observed_rate = (
                2.0
                * math.acos(float(quaternion[3]))
                / 0.1
            )
            update_summary_scalar(
                run,
                "maximum_observed_pose_angular_speed_rad_s",
                f"{observed_rate:.9f}",
            )
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 4)
            self.assertIn(
                "tracking versus visual pose 7 orientation differs",
                result.stderr,
            )
            self.assertFalse(output.exists())

    def test_bundle_v6_tracking_timestamp_mismatch_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            visual = inputs[0] / "live_visual_tracking_trajectory_tum.txt"
            visual.write_text(
                visual.read_text(encoding="utf-8").replace(
                    "1.3 0.001", "1.3001 0.001"
                ),
                encoding="utf-8",
            )
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 4)
            self.assertIn(
                "tracking versus visual timestamp 3 differs",
                result.stderr,
            )
            self.assertFalse(output.exists())

    def test_endpoint_windows_reject_single_frame_false_return(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            run = inputs[0]
            for name in (
                "live_camera_trajectory_tum.txt",
                "live_visual_tracking_trajectory_tum.txt",
            ):
                path = run / name
                content = path.read_text(encoding="utf-8")
                content = content.replace("2.2 0.019", "2.2 0.100")
                content = content.replace("2.3 0.021", "2.3 0.100")
                content = content.replace("2.4 0.020", "2.4 0.000")
                path.write_text(content, encoding="utf-8")
            update_tracking_pose(
                run, "2.2", translation=("0.100", "0", "0")
            )
            update_tracking_pose(
                run, "2.3", translation=("0.100", "0", "0")
            )
            update_tracking_pose(
                run, "2.4", translation=("0.000", "0", "0")
            )
            update_summary_scalar(
                run,
                "maximum_observed_pose_linear_speed_m_s",
                "1.000000000",
            )
            reference = root / "closed_loop_reference.yaml"
            write_live_closed_loop_reference(reference)
            output = root / "evaluation.yaml"

            result = run_live_evaluator(
                inputs,
                output,
                "--closed-loop-reference",
                str(reference),
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn(
                'state: "LIVE_GATE_PASS_CLOSED_LOOP_REFERENCE_FAILED"',
                manifest,
            )
            self.assertIn(
                "live_continuity_gate_passed: true", manifest
            )
            self.assertIn(
                "CLOSED_LOOP_END_POSITION_SPREAD_EXCEEDED", manifest
            )
            self.assertIn(
                "canonical_endpoint_displacement_m: 0.000000000",
                manifest,
            )
            self.assertIn(
                "closed_loop_robust_endpoint_displacement_m: 0.100000000",
                manifest,
            )
            self.assertIn("closed_loop_reference_passed: false", manifest)

    def test_endpoint_quaternion_average_handles_equivalent_signs(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            run = inputs[0]
            for name in (
                "live_camera_trajectory_tum.txt",
                "live_visual_tracking_trajectory_tum.txt",
            ):
                path = run / name
                path.write_text(
                    path.read_text(encoding="utf-8").replace(
                        "1.3 0.001 0 0 0 0 0 1",
                        "1.3 0.001 0 0 0 0 0 -1",
                    ),
                    encoding="utf-8",
                )
            # q and -q are the same orientation; leave the tracking CSV at
            # +q to prove cross-artifact matching is sign-invariant.
            reference = root / "closed_loop_reference.yaml"
            write_live_closed_loop_reference(reference)
            output = root / "evaluation.yaml"

            result = run_live_evaluator(
                inputs,
                output,
                "--closed-loop-reference",
                str(reference),
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn("live_gate_passed: true", manifest)
            self.assertIn("closed_loop_reference_passed: true", manifest)
            self.assertIn(
                "closed_loop_start_maximum_orientation_spread_deg: "
                "0.000000000",
                manifest,
            )

    def test_endpoint_windows_reject_orientation_dispersion(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            run = inputs[0]
            for name in (
                "live_camera_trajectory_tum.txt",
                "live_visual_tracking_trajectory_tum.txt",
            ):
                path = run / name
                path.write_text(
                    path.read_text(encoding="utf-8").replace(
                        "2.3 0.021 0 0 0 0 0 1",
                        "2.3 0.021 0 0 0 0 0.087155743 0.996194698",
                    ),
                    encoding="utf-8",
                )
            update_tracking_pose(
                run,
                "2.3",
                quaternion=(
                    "0",
                    "0",
                    "0.087155743",
                    "0.996194698",
                ),
            )
            update_summary_scalar(
                run,
                "maximum_observed_pose_angular_speed_rad_s",
                "1.745329252",
            )
            reference = root / "closed_loop_reference.yaml"
            write_live_closed_loop_reference(reference)
            output = root / "evaluation.yaml"

            result = run_live_evaluator(
                inputs,
                output,
                "--closed-loop-reference",
                str(reference),
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn(
                'state: "LIVE_GATE_PASS_CLOSED_LOOP_REFERENCE_FAILED"',
                manifest,
            )
            self.assertIn(
                "live_continuity_gate_passed: true", manifest
            )
            self.assertIn(
                "CLOSED_LOOP_END_ORIENTATION_SPREAD_EXCEEDED", manifest
            )
            self.assertIn("closed_loop_reference_passed: false", manifest)

    def test_endpoint_windows_require_configured_hold_samples(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            reference = root / "closed_loop_reference.yaml"
            write_live_closed_loop_reference(
                reference, minimum_endpoint_samples=4
            )
            output = root / "evaluation.yaml"

            result = run_live_evaluator(
                inputs,
                output,
                "--closed-loop-reference",
                str(reference),
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn(
                'state: "LIVE_GATE_PASS_CLOSED_LOOP_REFERENCE_FAILED"',
                manifest,
            )
            self.assertIn(
                "live_continuity_gate_passed: true", manifest
            )
            self.assertIn("CLOSED_LOOP_START_HOLD_INSUFFICIENT", manifest)
            self.assertIn("CLOSED_LOOP_END_HOLD_INSUFFICIENT", manifest)
            self.assertIn(
                'closed_loop_reference_evaluation_state: '
                '"ENDPOINT_HOLD_REQUIREMENTS_NOT_MET"',
                manifest,
            )

    def test_endpoint_windows_must_not_overlap(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            reference = root / "closed_loop_reference.yaml"
            write_live_closed_loop_reference(
                reference, endpoint_window_seconds=0.7
            )
            output = root / "evaluation.yaml"

            result = run_live_evaluator(
                inputs,
                output,
                "--closed-loop-reference",
                str(reference),
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn(
                'state: "LIVE_GATE_PASS_CLOSED_LOOP_REFERENCE_FAILED"',
                manifest,
            )
            self.assertIn(
                "live_continuity_gate_passed: true", manifest
            )
            self.assertIn("CLOSED_LOOP_ENDPOINT_WINDOWS_OVERLAP", manifest)
            self.assertIn("closed_loop_reference_passed: false", manifest)

    def test_reference_requires_declared_path_duration_and_excursion(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            reference = root / "closed_loop_reference.yaml"
            write_live_closed_loop_reference(
                reference,
                minimum_path_duration_seconds=2.0,
                minimum_path_excursion_m=0.5,
            )
            output = root / "evaluation.yaml"

            result = run_live_evaluator(
                inputs,
                output,
                "--closed-loop-reference",
                str(reference),
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn(
                'state: "LIVE_GATE_PASS_CLOSED_LOOP_REFERENCE_FAILED"',
                manifest,
            )
            self.assertIn(
                "CLOSED_LOOP_PATH_DURATION_BELOW_REFERENCE_MINIMUM",
                manifest,
            )
            self.assertIn(
                "CLOSED_LOOP_PATH_EXCURSION_BELOW_REFERENCE_MINIMUM",
                manifest,
            )
            self.assertIn(
                "live_continuity_gate_passed: true", manifest
            )

    def test_live_reference_rejects_legacy_single_pose_schema(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            reference = root / "closed_loop_reference.yaml"
            write_live_closed_loop_reference(reference)
            reference.write_text(
                reference.read_text(encoding="utf-8").replace(
                    "ovrs-closed-loop-reference-v2",
                    "ovrs-closed-loop-reference-v1",
                ),
                encoding="utf-8",
            )
            output = root / "evaluation.yaml"

            result = run_live_evaluator(
                inputs,
                output,
                "--closed-loop-reference",
                str(reference),
            )

            self.assertEqual(result.returncode, 4)
            self.assertIn(
                "closed-loop reference format differs", result.stderr
            )
            self.assertFalse(output.exists())

    def test_reset_run_is_preserved_as_coherent_gate_failure(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root, reset=True)
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn('state: "LIVE_GATE_FAILED"', manifest)
            self.assertIn("live_gate_passed: false", manifest)
            self.assertIn("preacceptance_map_reset_count: 1", manifest)
            self.assertIn(
                "preacceptance_reset_limit_exceeded: false", manifest
            )
            self.assertIn("CANONICAL_TRAJECTORY_ABSENT", manifest)

    def test_post_acceptance_tracking_loss_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root, tracking_loss=True)
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn('state: "LIVE_GATE_FAILED"', manifest)
            self.assertIn("TRACKING_LOSS_AFTER_ACCEPTANCE", manifest)
            self.assertIn(
                "tracking_loss_after_acceptance_count: 1", manifest
            )
            self.assertIn("TRAJECTORY_ACCEPTANCE_DISCONTINUITY", manifest)
            self.assertIn("CANONICAL_TRAJECTORY_ABSENT", manifest)

    def test_post_acceptance_tracking_gap_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root, tracking_gap=True)
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn('state: "LIVE_GATE_FAILED"', manifest)
            self.assertIn("TRACKING_GAP_AFTER_ACCEPTANCE", manifest)
            self.assertIn(
                "tracking_gap_after_acceptance_count: 1", manifest
            )
            self.assertIn(
                "maximum_observed_tracking_interval_seconds: 1.200000000",
                manifest,
            )
            self.assertIn("TRAJECTORY_ACCEPTANCE_DISCONTINUITY", manifest)
            self.assertIn("CANONICAL_TRAJECTORY_ABSENT", manifest)

    def test_post_acceptance_visual_support_loss_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root, visual_support_loss=True)
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn('state: "LIVE_GATE_FAILED"', manifest)
            self.assertIn(
                "VISUAL_SUPPORT_LOST_AFTER_ACCEPTANCE", manifest
            )
            self.assertIn("minimum_tracked_map_points: 50", manifest)
            self.assertIn(
                "visual_support_failure_after_acceptance_count: 1",
                manifest,
            )
            self.assertIn(
                "TRAJECTORY_ACCEPTANCE_DISCONTINUITY", manifest
            )
            self.assertIn("CANONICAL_TRAJECTORY_ABSENT", manifest)

    def test_post_acceptance_pose_rate_jump_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root, pose_rate_loss=True)
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn('state: "LIVE_GATE_FAILED"', manifest)
            self.assertIn(
                "POSE_RATE_LIMIT_EXCEEDED_AFTER_ACCEPTANCE", manifest
            )
            self.assertIn(
                "maximum_observed_pose_linear_speed_m_s: 3.000000000",
                manifest,
            )
            self.assertIn("pose_rate_gate_failure_count: 1", manifest)
            self.assertIn(
                "pose_rate_failure_after_acceptance_count: 1", manifest
            )
            self.assertIn(
                "TRAJECTORY_ACCEPTANCE_DISCONTINUITY", manifest
            )
            self.assertIn("CANONICAL_TRAJECTORY_ABSENT", manifest)

    def test_pre_acceptance_lost_state_does_not_fail_continuity(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            run = inputs[0]
            tracking = run / "live_tracking_states.csv"
            tracking.write_text(
                tracking.read_text(encoding="utf-8").replace(
                    "0.1,NOT_INITIALIZED,", "0.1,LOST,"
                ),
                encoding="utf-8",
            )
            summary = run / "run_summary.yaml"
            summary.write_text(
                summary.read_text(encoding="utf-8").replace(
                    "lost_frame_count: 0", "lost_frame_count: 1"
                ),
                encoding="utf-8",
            )
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn("live_gate_passed: true", manifest)
            self.assertIn("lost_frame_count: 1", manifest)
            self.assertNotIn("TRACKING_LOSS_AFTER_ACCEPTANCE", manifest)

    def test_rejects_fabricated_stable_tracking_elapsed_time(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            tracking = inputs[0] / "live_tracking_states.csv"
            tracking.write_text(
                tracking.read_text(encoding="utf-8").replace(
                    "1.000000000,0.0,0\n",
                    "1.000000000,0.5,0\n",
                    1,
                ),
                encoding="utf-8",
            )
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 4)
            self.assertIn("stable gate elapsed time differs", result.stderr)
            self.assertFalse(output.exists())

    def test_pending_reset_cannot_preload_stability_window(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            tracking = inputs[0] / "live_tracking_states.csv"
            rows = list(
                csv.DictReader(
                    tracking.read_text(encoding="utf-8").splitlines()
                )
            )
            rows[1]["reset_pending"] = "1"
            with tracking.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
                writer.writeheader()
                writer.writerows(rows)
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 4)
            self.assertIn("stable gate elapsed time differs", result.stderr)
            self.assertFalse(output.exists())

    def test_pre_pose_failure_with_empty_visual_file_is_classified(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root, reset=True)
            run = inputs[0]
            tracking = run / "live_tracking_states.csv"
            tracking_rows = list(
                csv.DictReader(
                    tracking.read_text(encoding="utf-8").splitlines()
                )
            )
            for row in tracking_rows:
                if row["state"] in ("OK", "OK_KLT"):
                    row["state"] = "NOT_INITIALIZED"
                    for field in (
                        "pose_tx_m",
                        "pose_ty_m",
                        "pose_tz_m",
                        "pose_qx",
                        "pose_qy",
                        "pose_qz",
                        "pose_qw",
                    ):
                        row[field] = ""
            with tracking.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(
                    handle, fieldnames=tracking_rows[0].keys()
                )
                writer.writeheader()
                writer.writerows(tracking_rows)
            (run / "live_visual_tracking_trajectory_tum.txt").write_text(
                "", encoding="utf-8"
            )
            summary = run / "run_summary.yaml"
            summary.write_text(
                summary.read_text(encoding="utf-8")
                .replace("visual_pose_count: 3", "visual_pose_count: 0")
                .replace(
                    'last_tracking_state: "OK"',
                    'last_tracking_state: "NOT_INITIALIZED"',
                ),
                encoding="utf-8",
            )
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn('state: "LIVE_GATE_FAILED"', manifest)
            self.assertIn("visual_pose_count: 0", manifest)
            self.assertIn("CANONICAL_TRAJECTORY_ABSENT", manifest)
            self.assertIn(
                "live_visual_tracking_trajectory_tum_sha256:", manifest
            )

    def test_rejects_trajectory_timestamp_not_bound_to_tracking(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            trajectory = inputs[0] / "live_camera_trajectory_tum.txt"
            trajectory.write_text(
                trajectory.read_text(encoding="utf-8").replace(
                    "1.2 0 0", "1.21 0 0"
                ),
                encoding="utf-8",
            )
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 4)
            self.assertIn(
                "candidate trajectory timestamp 1 differs", result.stderr
            )
            self.assertFalse(output.exists())

    def test_caller_supplied_continuity_envelope_fails_closed(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            output = root / "evaluation.yaml"

            result = run_live_evaluator(
                inputs,
                output,
                "--maximum-adjacent-translation-m",
                "0.01",
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn('state: "LIVE_GATE_FAILED"', manifest)
            self.assertIn(
                "MAXIMUM_ADJACENT_TRANSLATION_LIMIT_EXCEEDED",
                manifest,
            )
            self.assertIn(
                'continuity_envelope_state: '
                '"EVALUATED_CALLER_SUPPLIED_LIMIT"',
                manifest,
            )

    def test_rejects_executable_changed_after_capture_start(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            inputs[5].write_bytes(b"mutated executable")
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 4)
            self.assertIn(
                "live_executable_sha256_at_start differs", result.stderr
            )
            self.assertFalse(output.exists())

    def test_legacy_run_remains_explicitly_unattested(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            inputs = make_orb_live_run(root)
            metadata = inputs[0] / "run_metadata.yaml"
            legacy_prefixes = (
                "runtime_provenance_format:",
                "backend_library_sha256_at_start:",
                "live_executable_sha256_at_start:",
                "vocabulary_sha256_at_start:",
                "settings_sha256_at_start:",
                "live_bundle_manifest_sha256_at_start:",
                "launch_provenance_sha256:",
                "source_fingerprint_at_start:",
            )
            metadata.write_text(
                "\n".join(
                    line
                    for line in metadata.read_text(
                        encoding="utf-8"
                    ).splitlines()
                    if not line.startswith(legacy_prefixes)
                )
                + "\n",
                encoding="utf-8",
            )
            (inputs[0] / "launch_provenance.yaml").unlink()
            (inputs[0] / "source_live_manifest.yaml").unlink()
            output = root / "evaluation.yaml"

            result = run_live_evaluator(inputs, output)

            self.assertEqual(result.returncode, 0, result.stderr)
            manifest = output.read_text(encoding="utf-8")
            self.assertIn("live_gate_passed: true", manifest)
            self.assertIn(
                '"LEGACY_RUN_CALLER_SUPPLIED_BINARY_'
                'NOT_CAPTURE_TIME_ATTESTED"',
                manifest,
            )


if __name__ == "__main__":
    unittest.main()
