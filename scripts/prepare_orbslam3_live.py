#!/usr/bin/env python3
"""Generate a fail-closed ORB-SLAM3 live settings bundle."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

from export_vislam_benchmark import BenchmarkError, sha256_file, simple_yaml_map
from prepare_orbslam3_benchmark import (
    invert_rigid,
    matrix_multiply,
    nested_scalars,
    parse_camera_calibration,
    parse_decimal,
    parse_int,
    validate_backend_patch,
    validate_rigid_transform,
    write_settings,
)


def safe_dependency(parent: Path, value: str, field: str) -> Path:
    relative = Path(value)
    if not value or relative.is_absolute() or ".." in relative.parts:
        raise BenchmarkError(f"{field} must be a safe relative path")
    resolved_parent = parent.resolve()
    resolved = (resolved_parent / relative).resolve()
    if not resolved.is_relative_to(resolved_parent) or not resolved.is_file():
        raise BenchmarkError(f"{field} does not resolve to a local file")
    return resolved


def prepare(args: argparse.Namespace) -> None:
    root = Path(__file__).resolve().parents[1]
    estimator = args.estimator_config.resolve()
    stream_path = args.stream_config.resolve()
    pin_path = args.backend_pin.resolve()
    output = args.output.resolve()
    if output.exists():
        raise BenchmarkError(f"output already exists: {output}")
    for path in (estimator, stream_path, pin_path):
        if not path.is_file():
            raise BenchmarkError(f"required input is not a file: {path}")

    main = simple_yaml_map(estimator)
    serial = main.get("calibrated_serial", "")
    calibration_state = main.get("calibration_state", "")
    if not re.fullmatch(r"[0-9]+", serial):
        raise BenchmarkError("calibrated_serial must be numeric")
    if calibration_state not in (
        "BOOTSTRAP_UNVERIFIED",
        "KALIBR_VERIFIED",
    ):
        raise BenchmarkError("unsupported calibration_state")
    gravity_m_s2 = parse_decimal(main.get("gravity_mag", ""), "gravity_mag")
    maximum_startup_gravity_error_m_s2 = parse_decimal(
        main.get("max_accel_bias_m_s2", ""), "max_accel_bias_m_s2"
    )
    if gravity_m_s2 <= 0 or maximum_startup_gravity_error_m_s2 <= 0:
        raise BenchmarkError(
            "gravity_mag and max_accel_bias_m_s2 must be positive"
        )
    if maximum_startup_gravity_error_m_s2 >= gravity_m_s2:
        raise BenchmarkError("max_accel_bias_m_s2 must be below gravity_mag")
    imucam_path = safe_dependency(
        estimator.parent,
        main.get("relative_config_imucam", ""),
        "relative_config_imucam",
    )
    imu_path = safe_dependency(
        estimator.parent,
        main.get("relative_config_imu", ""),
        "relative_config_imu",
    )
    imucam_root = simple_yaml_map(imucam_path)
    imu_root = simple_yaml_map(imu_path)
    for path, values in ((imucam_path, imucam_root), (imu_path, imu_root)):
        if values.get("calibrated_serial") != serial:
            raise BenchmarkError(f"{path}: calibrated_serial differs")
        if values.get("calibration_state") != calibration_state:
            raise BenchmarkError(f"{path}: calibration_state differs")

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

    stream = simple_yaml_map(stream_path)
    width = parse_int(stream.get("width", ""), "stream width")
    height = parse_int(stream.get("height", ""), "stream height")
    source_fps = parse_int(stream.get("camera_fps", ""), "camera_fps")
    gyro_fps = parse_int(stream.get("gyro_fps", ""), "gyro_fps")
    if (width, height) != cam0.resolution:
        raise BenchmarkError("stream and calibration resolutions differ")
    if source_fps <= 0 or source_fps % args.camera_stride:
        raise BenchmarkError("camera_fps must be divisible by camera_stride")
    if parse_decimal(
        stream.get("gyro_scale_factor", ""), "gyro_scale_factor"
    ) != 1:
        raise BenchmarkError("live ORB-SLAM3 requires gyro_scale_factor 1")
    if stream.get("motion_correction_enabled") != "true":
        raise BenchmarkError("motion correction must be enabled")
    if stream.get("global_time_enabled") != "true":
        raise BenchmarkError("global time must be enabled")

    imu_values = nested_scalars(imu_path, "imu0")
    if imu_values.get("imu_intrinsic_method") != (
        "REALSENSE_DEVICE_TABLE_WITH_SDK_CORRECTION"
    ):
        raise BenchmarkError("unexpected IMU intrinsic correction policy")
    if imu_values.get("realsense_motion_correction_enabled") != "true":
        raise BenchmarkError("IMU calibration requires motion correction")
    if imu_values.get("realsense_global_time_enabled") != "true":
        raise BenchmarkError("IMU calibration requires global time")
    if parse_int(imu_values.get("update_rate", ""), "IMU update_rate") != gyro_fps:
        raise BenchmarkError("stream gyro_fps and IMU update_rate differ")

    pin = simple_yaml_map(pin_path)
    expected = {
        "format": "ovrs-orbslam3-backend-pin-v1",
        "backend_name": "ORB_SLAM3",
        "license": "GPL-3.0-or-later",
    }
    for key, value in expected.items():
        if pin.get(key) != value:
            raise BenchmarkError(f"backend pin {key} must be {value}")
    commit = pin.get("commit", "")
    if not re.fullmatch(r"[0-9a-f]{40}", commit):
        raise BenchmarkError("backend commit must be a lowercase SHA-1")
    patch_hash = validate_backend_patch(pin, root)
    imu_init_acceleration_threshold = parse_decimal(
        pin.get("imu_init_acceleration_threshold_m_s2", ""),
        "backend pin imu_init_acceleration_threshold_m_s2",
    )
    if imu_init_acceleration_threshold <= 0:
        raise BenchmarkError(
            "backend pin imu_init_acceleration_threshold_m_s2 must be positive"
        )
    minimum_stable_inertial_seconds = parse_decimal(
        pin.get("live_minimum_stable_inertial_seconds", ""),
        "backend pin live_minimum_stable_inertial_seconds",
    )
    if (
        minimum_stable_inertial_seconds <= 0
        or minimum_stable_inertial_seconds > 60
    ):
        raise BenchmarkError(
            "backend pin live_minimum_stable_inertial_seconds must be in (0, 60]"
        )
    maximum_tracking_interval_factor = parse_decimal(
        pin.get("live_maximum_tracking_interval_factor", ""),
        "backend pin live_maximum_tracking_interval_factor",
    )
    if (
        maximum_tracking_interval_factor <= 1
        or maximum_tracking_interval_factor > 10
    ):
        raise BenchmarkError(
            "backend pin live_maximum_tracking_interval_factor "
            "must be in (1, 10]"
        )
    maximum_preacceptance_map_resets = parse_int(
        pin.get("live_maximum_preacceptance_map_resets", ""),
        "backend pin live_maximum_preacceptance_map_resets",
    )
    if maximum_preacceptance_map_resets < 0 or maximum_preacceptance_map_resets > 20:
        raise BenchmarkError(
            "backend pin live_maximum_preacceptance_map_resets must be in [0, 20]"
        )
    startup_stationary_seconds = parse_decimal(
        pin.get("live_startup_stationary_seconds", ""),
        "backend pin live_startup_stationary_seconds",
    )
    startup_stationary_timeout_seconds = parse_decimal(
        pin.get("live_startup_stationary_timeout_seconds", ""),
        "backend pin live_startup_stationary_timeout_seconds",
    )
    startup_maximum_acceleration_stddev_m_s2 = parse_decimal(
        pin.get("live_startup_maximum_acceleration_stddev_m_s2", ""),
        "backend pin live_startup_maximum_acceleration_stddev_m_s2",
    )
    startup_maximum_gyro_magnitude_rad_s = parse_decimal(
        pin.get("live_startup_maximum_gyro_magnitude_rad_s", ""),
        "backend pin live_startup_maximum_gyro_magnitude_rad_s",
    )
    maximum_input_stall_seconds = parse_decimal(
        pin.get("live_maximum_input_stall_seconds", ""),
        "backend pin live_maximum_input_stall_seconds",
    )
    if (
        startup_stationary_seconds <= 0
        or startup_stationary_timeout_seconds < startup_stationary_seconds
        or startup_stationary_timeout_seconds > 60
        or startup_maximum_acceleration_stddev_m_s2 <= 0
        or startup_maximum_gyro_magnitude_rad_s <= 0
        or maximum_input_stall_seconds <= 0
        or maximum_input_stall_seconds > 10
    ):
        raise BenchmarkError("invalid live startup stationary IMU thresholds")

    feed_fps = source_fps // args.camera_stride
    maximum_tracking_interval_seconds = (
        maximum_tracking_interval_factor / feed_fps
    )
    provenance = (
        'OVRS.Mode: "EXPERIMENTAL_PURE_ORB_SLAM3_LIVE"',
        f'OVRS.CalibratedSerial: "{serial}"',
        f'OVRS.CalibrationState: "{calibration_state}"',
        f'OVRS.BackendCommit: "{commit}"',
        f'OVRS.BackendPatchSHA256: "{patch_hash}"',
        f'OVRS.EstimatorConfigSHA256: "{sha256_file(estimator)}"',
        f'OVRS.ImuCamConfigSHA256: "{sha256_file(imucam_path)}"',
        f'OVRS.ImuConfigSHA256: "{sha256_file(imu_path)}"',
        f'OVRS.StreamConfigSHA256: "{sha256_file(stream_path)}"',
        f"OVRS.SourceCameraFPS: {source_fps}",
        f"OVRS.CameraStride: {args.camera_stride}",
        f"OVRS.CameraImuTimeOffsetSeconds: {cam0.time_offset_s}",
        f"OVRS.GravityMagnitudeMps2: {gravity_m_s2}",
        "OVRS.StartupMaximumGravityErrorMps2: "
        f"{maximum_startup_gravity_error_m_s2}",
        f"OVRS.StartupStationarySeconds: {startup_stationary_seconds}",
        "OVRS.StartupStationaryTimeoutSeconds: "
        f"{startup_stationary_timeout_seconds}",
        "OVRS.StartupMaximumAccelerationStddevMps2: "
        f"{startup_maximum_acceleration_stddev_m_s2}",
        "OVRS.StartupMaximumGyroMagnitudeRadps: "
        f"{startup_maximum_gyro_magnitude_rad_s}",
        f"OVRS.MaximumInputStallSeconds: {maximum_input_stall_seconds}",
        "OVRS.MinimumStableInertialSeconds: "
        f"{minimum_stable_inertial_seconds}",
        "OVRS.MaximumTrackingIntervalSeconds: "
        f"{maximum_tracking_interval_seconds}",
        "OVRS.MaximumTrackingIntervalFactor: "
        f"{maximum_tracking_interval_factor}",
        "OVRS.MaximumPreacceptanceMapResets: "
        f"{maximum_preacceptance_map_resets}",
        'OVRS.TimeOffsetConvention: "t_imu=t_cam+timeshift_cam_imu"',
        'OVRS.OutputPose: "T_atlas_world_camera0"',
    )
    output.mkdir(parents=True)
    incomplete = output / "INCOMPLETE"
    incomplete.write_text("ORB-SLAM3 live preparation incomplete.\n", encoding="utf-8")
    settings = output / "orbslam3_live_settings.yaml"
    write_settings(
        settings,
        cam0,
        cam1,
        stereo,
        imu_values,
        feed_fps,
        args.n_features,
        args.initial_fast_threshold,
        args.minimum_fast_threshold,
        float(imu_init_acceleration_threshold),
        None,
        args.save_atlas_name,
        provenance,
    )
    manifest = output / "live_manifest.yaml"
    manifest.write_text(
        "%YAML:1.0\n"
        'format: "ovrs-orbslam3-live-bundle-v4"\n'
        'state: "PREPARED_NOT_RUN"\n'
        'integration: "PURE_ORB_SLAM3_STEREO_INERTIAL"\n'
        "openvins_pose_consumed: false\n"
        "global_correction_fed_to_openvins: false\n"
        f'camera_serial: "{serial}"\n'
        f'calibration_state: "{calibration_state}"\n'
        f'backend_commit: "{commit}"\n'
        f'backend_patch_sha256: "{patch_hash}"\n'
        f"source_camera_fps: {source_fps}\n"
        f"camera_stride: {args.camera_stride}\n"
        f"orb_camera_fps: {feed_fps}\n"
        "imu_init_acceleration_threshold_m_s2: "
        f"{imu_init_acceleration_threshold}\n"
        "minimum_stable_inertial_seconds: "
        f"{minimum_stable_inertial_seconds}\n"
        "maximum_tracking_interval_factor: "
        f"{maximum_tracking_interval_factor}\n"
        "maximum_tracking_interval_seconds: "
        f"{maximum_tracking_interval_seconds}\n"
        "maximum_preacceptance_map_resets: "
        f"{maximum_preacceptance_map_resets}\n"
        f"gravity_m_s2: {gravity_m_s2}\n"
        "startup_maximum_gravity_error_m_s2: "
        f"{maximum_startup_gravity_error_m_s2}\n"
        "startup_stationary_seconds: "
        f"{startup_stationary_seconds}\n"
        "startup_stationary_timeout_seconds: "
        f"{startup_stationary_timeout_seconds}\n"
        "startup_maximum_acceleration_stddev_m_s2: "
        f"{startup_maximum_acceleration_stddev_m_s2}\n"
        "startup_maximum_gyro_magnitude_rad_s: "
        f"{startup_maximum_gyro_magnitude_rad_s}\n"
        "maximum_input_stall_seconds: "
        f"{maximum_input_stall_seconds}\n"
        f"camera_imu_time_offset_s: {cam0.time_offset_s}\n"
        f'settings_sha256: "{sha256_file(settings)}"\n',
        encoding="utf-8",
    )
    incomplete.unlink()
    print(f"Prepared ORB-SLAM3 live bundle: {output}")
    print(
        f"State: PREPARED_NOT_RUN (serial={serial}, "
        f"camera={source_fps}/{args.camera_stride}={feed_fps} Hz)"
    )


def parser() -> argparse.ArgumentParser:
    root = Path(__file__).resolve().parents[1]
    result = argparse.ArgumentParser(
        description="Generate a serial-bound ORB-SLAM3 live settings bundle."
    )
    result.add_argument("--estimator-config", required=True, type=Path)
    result.add_argument("--stream-config", required=True, type=Path)
    result.add_argument(
        "--backend-pin",
        type=Path,
        default=root / "config" / "research" / "orbslam3_backend.yaml",
    )
    result.add_argument("--output", required=True, type=Path)
    result.add_argument("--camera-stride", type=int, default=3)
    result.add_argument("--n-features", type=int, default=1200)
    result.add_argument("--initial-fast-threshold", type=int, default=20)
    result.add_argument("--minimum-fast-threshold", type=int, default=7)
    result.add_argument("--save-atlas-name")
    return result


def main() -> int:
    args = parser().parse_args()
    if args.camera_stride <= 0:
        print("ERROR: camera_stride must be positive", file=sys.stderr)
        return 2
    if args.n_features <= 0:
        print("ERROR: n_features must be positive", file=sys.stderr)
        return 2
    if (
        args.minimum_fast_threshold <= 0
        or args.initial_fast_threshold <= 0
        or args.minimum_fast_threshold > args.initial_fast_threshold
    ):
        print("ERROR: invalid FAST thresholds", file=sys.stderr)
        return 2
    if args.save_atlas_name is not None and not re.fullmatch(
        r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", args.save_atlas_name
    ):
        print("ERROR: invalid atlas name", file=sys.stderr)
        return 2
    try:
        prepare(args)
    except (BenchmarkError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 4
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
