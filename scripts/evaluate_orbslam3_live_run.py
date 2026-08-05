#!/usr/bin/env python3
"""Independently evaluate one pure ORB-SLAM3 live run fail-closed."""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

from export_vislam_benchmark import (
    BenchmarkError,
    sha256_file,
    simple_yaml_map,
    yaml_quote,
)


TRACKING_FIELDS = (
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
    "stable_gate_elapsed_s",
    "trajectory_candidate_accepted",
)
IMU_FIELDS = (
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
TRACKING_STATES = {
    "SYSTEM_NOT_READY",
    "NO_IMAGES_YET",
    "NOT_INITIALIZED",
    "OK",
    "RECENTLY_LOST",
    "LOST",
    "OK_KLT",
}
POSE_STATES = {"OK", "OK_KLT"}
LOST_STATES = {"RECENTLY_LOST", "LOST"}
TIMESTAMP_TOLERANCE_S = 5e-9
FLOAT_TOLERANCE = 1e-6


@dataclass(frozen=True)
class TrackingRow:
    timestamp_s: float
    state: str
    tracked_map_points: int
    tracking_latency_ms: float
    imu_batch: int
    startup_imu_gate_passed: bool
    inertial_initialized: bool
    inertial_ba2_finished: bool
    reset_count: int
    map_change_index: int
    reset_pending: bool
    stable_gate_elapsed_s: float
    accepted: bool


@dataclass(frozen=True)
class TrackingStats:
    rows: tuple[TrackingRow, ...]
    visual_pose_count: int
    lost_frame_count: int
    tracking_loss_after_acceptance_count: int
    tracking_gap_after_acceptance_count: int
    visual_support_failure_after_acceptance_count: int
    maximum_observed_tracking_interval_s: float
    accepted_timestamps_s: tuple[float, ...]
    ever_inertial_initialized: bool
    ever_inertial_ba2_finished: bool
    pending_reset_observed: bool
    pending_reset_after_acceptance_observed: bool
    preacceptance_map_reset_count: int
    postacceptance_map_reset_count: int
    inertial_regressions: int
    ba2_regressions: int
    acceptance_discontinuities: int
    map_change_after_acceptance: bool


@dataclass(frozen=True)
class TrajectoryStats:
    rows: int
    timestamps_s: tuple[float, ...]
    poses: tuple[tuple[float, ...], ...]
    duration_s: float
    endpoint_displacement_m: float
    endpoint_rotation_deg: float
    path_length_m: float
    maximum_adjacent_translation_m: float
    maximum_adjacent_speed_m_s: float
    bounding_box_x_m: float
    bounding_box_y_m: float
    bounding_box_z_m: float
    maximum_quaternion_norm_error: float


@dataclass(frozen=True)
class ClosedLoopReference:
    method: str
    position_tolerance_m: float
    orientation_tolerance_deg: float
    endpoint_window_seconds: float
    minimum_endpoint_samples: int
    maximum_endpoint_position_spread_m: float
    maximum_endpoint_orientation_spread_deg: float
    minimum_path_duration_seconds: float
    minimum_path_excursion_m: float


@dataclass(frozen=True)
class EndpointWindowStats:
    samples: int
    duration_s: float
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    maximum_position_spread_m: float
    maximum_orientation_spread_deg: float


@dataclass(frozen=True)
class ClosedLoopWindowStats:
    start: EndpointWindowStats
    end: EndpointWindowStats
    position_residual_m: float
    orientation_residual_deg: float


def require_file(path: Path, label: str, allow_empty: bool = True) -> None:
    if not path.is_file():
        raise BenchmarkError(f"{label} does not exist: {path}")
    if not allow_empty and path.stat().st_size == 0:
        raise BenchmarkError(f"{label} is empty: {path}")


def parse_bool(value: str, field: str) -> bool:
    if value == "true":
        return True
    if value == "false":
        return False
    raise BenchmarkError(f"{field} must be true or false")


def parse_csv_bool(value: str, field: str) -> bool:
    if value == "0":
        return False
    if value == "1":
        return True
    raise BenchmarkError(f"{field} must be 0 or 1")


def parse_int(value: str, field: str, minimum: int = 0) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise BenchmarkError(f"{field} is not an integer") from exc
    if parsed < minimum or str(parsed) != value.strip():
        raise BenchmarkError(f"{field} must be an integer >= {minimum}")
    return parsed


def parse_float(
    value: str,
    field: str,
    minimum: float | None = None,
    strictly_positive: bool = False,
) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise BenchmarkError(f"{field} is not numeric") from exc
    if not math.isfinite(parsed):
        raise BenchmarkError(f"{field} is not finite")
    if minimum is not None and parsed < minimum:
        raise BenchmarkError(f"{field} must be >= {minimum}")
    if strictly_positive and parsed <= 0:
        raise BenchmarkError(f"{field} must be positive")
    return parsed


def require_scalar(
    values: dict[str, str], key: str, source: str
) -> str:
    if key not in values or values[key] == "":
        raise BenchmarkError(f"{source} lacks {key}")
    return values[key]


def require_equal(
    actual: object, expected: object, field: str
) -> None:
    if actual != expected:
        raise BenchmarkError(
            f"{field} differs: expected {expected!r}, got {actual!r}"
        )


def require_close(
    actual: float,
    expected: float,
    field: str,
    tolerance: float = FLOAT_TOLERANCE,
) -> None:
    if not math.isclose(actual, expected, rel_tol=tolerance, abs_tol=tolerance):
        raise BenchmarkError(
            f"{field} differs: expected {expected:.12g}, got {actual:.12g}"
        )


def read_csv(path: Path, fields: tuple[str, ...]) -> list[dict[str, str]]:
    require_file(path, path.name, allow_empty=False)
    try:
        handle = path.open("r", encoding="utf-8", newline="")
    except OSError as exc:
        raise BenchmarkError(f"cannot read {path}: {exc}") from exc
    with handle:
        reader = csv.DictReader(handle)
        if tuple(reader.fieldnames or ()) != fields:
            raise BenchmarkError(
                f"{path}: expected CSV fields {','.join(fields)}"
            )
        result: list[dict[str, str]] = []
        for line_number, row in enumerate(reader, 2):
            if None in row or any(value is None for value in row.values()):
                raise BenchmarkError(f"{path}:{line_number}: malformed CSV row")
            result.append(row)
    if not result:
        raise BenchmarkError(f"{path}: no data rows")
    return result


def parse_tracking(
    path: Path,
    minimum_stable_s: float,
    maximum_tracking_interval_s: float,
    maximum_preacceptance_map_resets: int,
    minimum_tracked_map_points: int,
) -> TrackingStats:
    raw_rows = read_csv(path, TRACKING_FIELDS)
    rows: list[TrackingRow] = []
    accepted_timestamps: list[float] = []
    previous_timestamp = -math.inf
    previous_reset_count = 0
    previous_map_change = 0
    previous_initialized = False
    previous_ba2 = False
    previous_startup_passed = False
    acceptance_started = False
    visual_pose_count = 0
    lost_frame_count = 0
    tracking_loss_after_acceptance_count = 0
    tracking_gap_after_acceptance_count = 0
    visual_support_failure_after_acceptance_count = 0
    maximum_observed_tracking_interval_s = 0.0
    pending_observed = False
    pending_after_acceptance_observed = False
    reset_count_at_acceptance: int | None = None
    inertial_regressions = 0
    ba2_regressions = 0
    acceptance_discontinuities = 0
    map_change_after_acceptance = False
    stable_gate_started_at: float | None = None

    for index, raw in enumerate(raw_rows, 2):
        prefix = f"{path}:{index}"
        timestamp = parse_float(
            raw["timestamp_s"], f"{prefix} timestamp_s", minimum=0.0
        )
        if timestamp <= previous_timestamp:
            raise BenchmarkError(f"{prefix}: timestamps are not increasing")
        tracking_interval = (
            0.0
            if previous_timestamp == -math.inf
            else timestamp - previous_timestamp
        )
        maximum_observed_tracking_interval_s = max(
            maximum_observed_tracking_interval_s, tracking_interval
        )
        tracking_gap = (
            previous_timestamp != -math.inf
            and tracking_interval > maximum_tracking_interval_s
        )
        state = raw["state"]
        if state not in TRACKING_STATES:
            raise BenchmarkError(f"{prefix}: unsupported tracking state {state}")
        parse_int(raw["tracked_keypoints"], f"{prefix} tracked_keypoints")
        tracked_map_points = parse_int(
            raw["tracked_map_points"], f"{prefix} tracked_map_points"
        )
        tracking_latency_ms = parse_float(
            raw["tracking_latency_ms"],
            f"{prefix} tracking_latency_ms",
            minimum=0.0,
        )
        imu_batch = parse_int(raw["imu_batch"], f"{prefix} imu_batch")
        startup_passed = parse_csv_bool(
            raw["startup_imu_gate_passed"],
            f"{prefix} startup_imu_gate_passed",
        )
        initialized = parse_csv_bool(
            raw["inertial_initialized"],
            f"{prefix} inertial_initialized",
        )
        ba2 = parse_csv_bool(
            raw["inertial_ba2_finished"],
            f"{prefix} inertial_ba2_finished",
        )
        reset_count = parse_int(
            raw["active_map_reset_count"],
            f"{prefix} active_map_reset_count",
        )
        map_change = parse_int(
            raw["active_map_change_index"],
            f"{prefix} active_map_change_index",
        )
        reset_pending = parse_csv_bool(
            raw["reset_pending"], f"{prefix} reset_pending"
        )
        stable_elapsed = parse_float(
            raw["stable_gate_elapsed_s"],
            f"{prefix} stable_gate_elapsed_s",
            minimum=0.0,
        )
        accepted = parse_csv_bool(
            raw["trajectory_candidate_accepted"],
            f"{prefix} trajectory_candidate_accepted",
        )
        visual_support_sufficient = (
            tracked_map_points >= minimum_tracked_map_points
        )
        gate_ready = (
            startup_passed
            and initialized
            and ba2
            and state in POSE_STATES
            and visual_support_sufficient
        )
        if gate_ready:
            if stable_gate_started_at is None or tracking_gap:
                stable_gate_started_at = timestamp
            expected_stable_elapsed = timestamp - stable_gate_started_at
        else:
            stable_gate_started_at = None
            expected_stable_elapsed = 0.0
        require_close(
            stable_elapsed,
            expected_stable_elapsed,
            f"{prefix} stable gate elapsed time",
        )

        if ba2 and not initialized:
            raise BenchmarkError(f"{prefix}: BA2 true while inertial is false")
        if previous_startup_passed and not startup_passed:
            raise BenchmarkError(f"{prefix}: startup IMU gate regressed")
        if reset_count < previous_reset_count:
            raise BenchmarkError(f"{prefix}: reset count regressed")
        if map_change < previous_map_change and reset_count == previous_reset_count:
            raise BenchmarkError(
                f"{prefix}: map change index regressed without a reset"
            )
        if previous_initialized and not initialized:
            inertial_regressions += 1
        if previous_ba2 and not ba2:
            ba2_regressions += 1
        if acceptance_started and map_change != previous_map_change:
            map_change_after_acceptance = True
        if accepted:
            if (
                state not in POSE_STATES
                or not startup_passed
                or not initialized
                or not ba2
                or reset_count > maximum_preacceptance_map_resets
                or reset_pending
                or not visual_support_sufficient
                or stable_elapsed + FLOAT_TOLERANCE < minimum_stable_s
            ):
                raise BenchmarkError(
                    f"{prefix}: accepted trajectory row violates live gate"
                )
            if not acceptance_started:
                reset_count_at_acceptance = reset_count
            acceptance_started = True
            accepted_timestamps.append(timestamp)
        elif acceptance_started:
            acceptance_discontinuities += 1
            if state in LOST_STATES:
                tracking_loss_after_acceptance_count += 1
            if tracking_gap:
                tracking_gap_after_acceptance_count += 1
            if state in POSE_STATES and not visual_support_sufficient:
                visual_support_failure_after_acceptance_count += 1

        if state in POSE_STATES:
            visual_pose_count += 1
        if state in LOST_STATES:
            lost_frame_count += 1
        pending_observed = pending_observed or reset_pending
        pending_after_acceptance_observed = (
            pending_after_acceptance_observed
            or (acceptance_started and reset_pending)
        )
        rows.append(
            TrackingRow(
                timestamp_s=timestamp,
                state=state,
                tracked_map_points=tracked_map_points,
                tracking_latency_ms=tracking_latency_ms,
                imu_batch=imu_batch,
                startup_imu_gate_passed=startup_passed,
                inertial_initialized=initialized,
                inertial_ba2_finished=ba2,
                reset_count=reset_count,
                map_change_index=map_change,
                reset_pending=reset_pending,
                stable_gate_elapsed_s=stable_elapsed,
                accepted=accepted,
            )
        )
        previous_timestamp = timestamp
        previous_reset_count = reset_count
        previous_map_change = map_change
        previous_initialized = initialized
        previous_ba2 = ba2
        previous_startup_passed = startup_passed

    preacceptance_map_reset_count = (
        rows[-1].reset_count
        if reset_count_at_acceptance is None
        else reset_count_at_acceptance
    )
    postacceptance_map_reset_count = (
        rows[-1].reset_count - preacceptance_map_reset_count
    )
    return TrackingStats(
        rows=tuple(rows),
        visual_pose_count=visual_pose_count,
        lost_frame_count=lost_frame_count,
        tracking_loss_after_acceptance_count=(
            tracking_loss_after_acceptance_count
        ),
        tracking_gap_after_acceptance_count=(
            tracking_gap_after_acceptance_count
        ),
        visual_support_failure_after_acceptance_count=(
            visual_support_failure_after_acceptance_count
        ),
        maximum_observed_tracking_interval_s=(
            maximum_observed_tracking_interval_s
        ),
        accepted_timestamps_s=tuple(accepted_timestamps),
        ever_inertial_initialized=any(
            row.inertial_initialized for row in rows
        ),
        ever_inertial_ba2_finished=any(
            row.inertial_ba2_finished for row in rows
        ),
        pending_reset_observed=pending_observed,
        pending_reset_after_acceptance_observed=(
            pending_after_acceptance_observed
        ),
        preacceptance_map_reset_count=preacceptance_map_reset_count,
        postacceptance_map_reset_count=postacceptance_map_reset_count,
        inertial_regressions=inertial_regressions,
        ba2_regressions=ba2_regressions,
        acceptance_discontinuities=acceptance_discontinuities,
        map_change_after_acceptance=map_change_after_acceptance,
    )


def parse_imu(path: Path, tracking: TrackingStats) -> None:
    raw_rows = read_csv(path, IMU_FIELDS)
    if len(raw_rows) != len(tracking.rows):
        raise BenchmarkError(
            "live IMU row count differs from tracking row count"
        )
    continuous_float_fields = (
        "mean_ax_m_s2",
        "mean_ay_m_s2",
        "mean_az_m_s2",
        "mean_accel_magnitude_m_s2",
        "mean_gyro_magnitude_rad_s",
        "max_gyro_magnitude_rad_s",
    )
    optional_fields = (
        "orb_init_accel_delta_m_s2",
        "orb_init_accel_threshold_m_s2",
        "orb_init_accel_gate_passed",
    )
    for index, (raw, state) in enumerate(zip(raw_rows, tracking.rows), 2):
        prefix = f"{path}:{index}"
        timestamp = parse_float(
            raw["timestamp_s"], f"{prefix} timestamp_s", minimum=0.0
        )
        require_close(
            timestamp,
            state.timestamp_s,
            f"{prefix} timestamp versus tracking",
            TIMESTAMP_TOLERANCE_S,
        )
        require_equal(
            parse_int(raw["imu_batch"], f"{prefix} imu_batch"),
            state.imu_batch,
            f"{prefix} imu_batch versus tracking",
        )
        continuous_present = tuple(
            raw[field] != "" for field in continuous_float_fields
        )
        if state.imu_batch == 0:
            if any(continuous_present):
                raise BenchmarkError(
                    f"{prefix}: zero IMU batch has aggregate values"
                )
        else:
            if not all(continuous_present):
                raise BenchmarkError(
                    f"{prefix}: nonzero IMU batch lacks aggregate values"
                )
            for field in continuous_float_fields:
                parse_float(raw[field], f"{prefix} {field}")
        optional_present = tuple(raw[field] != "" for field in optional_fields)
        if any(optional_present) and not all(optional_present):
            raise BenchmarkError(
                f"{prefix}: ORB initialization diagnostics are partial"
            )
        if all(optional_present):
            parse_float(
                raw["orb_init_accel_delta_m_s2"],
                f"{prefix} orb_init_accel_delta_m_s2",
                minimum=0.0,
            )
            parse_float(
                raw["orb_init_accel_threshold_m_s2"],
                f"{prefix} orb_init_accel_threshold_m_s2",
                strictly_positive=True,
            )
            parse_csv_bool(
                raw["orb_init_accel_gate_passed"],
                f"{prefix} orb_init_accel_gate_passed",
            )
        comparisons = (
            (
                parse_csv_bool(
                    raw["inertial_initialized"],
                    f"{prefix} inertial_initialized",
                ),
                state.inertial_initialized,
                "inertial_initialized",
            ),
            (
                parse_csv_bool(
                    raw["inertial_ba2_finished"],
                    f"{prefix} inertial_ba2_finished",
                ),
                state.inertial_ba2_finished,
                "inertial_ba2_finished",
            ),
            (
                parse_int(
                    raw["active_map_reset_count"],
                    f"{prefix} active_map_reset_count",
                ),
                state.reset_count,
                "active_map_reset_count",
            ),
            (
                parse_int(
                    raw["active_map_change_index"],
                    f"{prefix} active_map_change_index",
                ),
                state.map_change_index,
                "active_map_change_index",
            ),
            (
                parse_csv_bool(
                    raw["reset_pending"], f"{prefix} reset_pending"
                ),
                state.reset_pending,
                "reset_pending",
            ),
        )
        for actual, expected, field in comparisons:
            require_equal(
                actual, expected, f"{prefix} {field} versus tracking"
            )


def quaternion_return_angle_deg(
    first: tuple[float, ...], last: tuple[float, ...]
) -> float:
    dot = abs(sum(left * right for left, right in zip(first, last)))
    return math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))


def parse_trajectory(
    path: Path,
    label: str,
    allow_empty: bool = False,
    minimum_rows: int = 2,
) -> TrajectoryStats | None:
    require_file(path, label)
    timestamps: list[float] = []
    positions: list[tuple[float, float, float]] = []
    quaternions: list[tuple[float, float, float, float]] = []
    maximum_norm_error = 0.0
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as exc:
        raise BenchmarkError(f"cannot read {path}: {exc}") from exc
    for line_number, line in enumerate(lines, 1):
        fields = line.split()
        if not fields:
            continue
        if len(fields) != 8:
            raise BenchmarkError(
                f"{path}:{line_number}: expected 8 trajectory fields"
            )
        try:
            values = tuple(float(field) for field in fields)
        except ValueError as exc:
            raise BenchmarkError(
                f"{path}:{line_number}: trajectory field is not numeric"
            ) from exc
        if not all(math.isfinite(value) for value in values):
            raise BenchmarkError(
                f"{path}:{line_number}: trajectory contains non-finite data"
            )
        timestamp = values[0]
        if timestamp < 0 or (timestamps and timestamp <= timestamps[-1]):
            raise BenchmarkError(
                f"{path}:{line_number}: timestamps are not increasing"
            )
        quaternion = values[4:8]
        norm_error = abs(
            math.sqrt(sum(value * value for value in quaternion)) - 1.0
        )
        if norm_error > 1e-3:
            raise BenchmarkError(
                f"{path}:{line_number}: quaternion is not normalized"
            )
        timestamps.append(timestamp)
        positions.append(values[1:4])
        quaternions.append(quaternion)
        maximum_norm_error = max(maximum_norm_error, norm_error)
    if not timestamps:
        if allow_empty:
            return None
        raise BenchmarkError(f"{label} is empty: {path}")
    if len(timestamps) < minimum_rows:
        raise BenchmarkError(
            f"{label} requires at least {minimum_rows} trajectory rows"
        )

    adjacent_distances = [
        math.dist(previous, current)
        for previous, current in zip(positions, positions[1:])
    ]
    adjacent_speeds = [
        distance / (current - previous)
        for distance, previous, current in zip(
            adjacent_distances, timestamps, timestamps[1:]
        )
    ]
    axes = tuple(zip(*positions))
    return TrajectoryStats(
        rows=len(timestamps),
        timestamps_s=tuple(timestamps),
        poses=tuple(
            position + quaternion
            for position, quaternion in zip(positions, quaternions)
        ),
        duration_s=timestamps[-1] - timestamps[0],
        endpoint_displacement_m=math.dist(positions[0], positions[-1]),
        endpoint_rotation_deg=quaternion_return_angle_deg(
            quaternions[0], quaternions[-1]
        ),
        path_length_m=sum(adjacent_distances),
        maximum_adjacent_translation_m=max(adjacent_distances, default=0.0),
        maximum_adjacent_speed_m_s=max(adjacent_speeds, default=0.0),
        bounding_box_x_m=max(axes[0]) - min(axes[0]),
        bounding_box_y_m=max(axes[1]) - min(axes[1]),
        bounding_box_z_m=max(axes[2]) - min(axes[2]),
        maximum_quaternion_norm_error=maximum_norm_error,
    )


def parse_closed_loop_reference(path: Path | None) -> ClosedLoopReference | None:
    if path is None:
        return None
    require_file(path, "closed-loop reference", allow_empty=False)
    values = simple_yaml_map(path)
    expected = {
        "format": "ovrs-closed-loop-reference-v2",
        "reference_type": "COLOCATED_START_END",
        "estimator_consumed_reference": "false",
        "physical_path_completed": "true",
    }
    for key, expected_value in expected.items():
        require_equal(
            values.get(key, ""),
            expected_value,
            f"closed-loop reference {key}",
        )
    method = require_scalar(values, "method", "closed-loop reference")
    position_tolerance = parse_float(
        require_scalar(
            values, "position_tolerance_m", "closed-loop reference"
        ),
        "closed-loop reference position_tolerance_m",
        strictly_positive=True,
    )
    orientation_tolerance = parse_float(
        require_scalar(
            values, "orientation_tolerance_deg", "closed-loop reference"
        ),
        "closed-loop reference orientation_tolerance_deg",
        strictly_positive=True,
    )
    if orientation_tolerance > 180:
        raise BenchmarkError(
            "closed-loop reference orientation_tolerance_deg exceeds 180"
        )
    endpoint_window_seconds = parse_float(
        require_scalar(
            values, "endpoint_window_seconds", "closed-loop reference"
        ),
        "closed-loop reference endpoint_window_seconds",
        strictly_positive=True,
    )
    minimum_endpoint_samples = parse_int(
        require_scalar(
            values, "minimum_endpoint_samples", "closed-loop reference"
        ),
        "closed-loop reference minimum_endpoint_samples",
        minimum=2,
    )
    maximum_endpoint_position_spread_m = parse_float(
        require_scalar(
            values,
            "maximum_endpoint_position_spread_m",
            "closed-loop reference",
        ),
        "closed-loop reference maximum_endpoint_position_spread_m",
        strictly_positive=True,
    )
    maximum_endpoint_orientation_spread_deg = parse_float(
        require_scalar(
            values,
            "maximum_endpoint_orientation_spread_deg",
            "closed-loop reference",
        ),
        "closed-loop reference maximum_endpoint_orientation_spread_deg",
        strictly_positive=True,
    )
    if maximum_endpoint_orientation_spread_deg > 180:
        raise BenchmarkError(
            "closed-loop reference "
            "maximum_endpoint_orientation_spread_deg exceeds 180"
        )
    minimum_path_duration_seconds = parse_float(
        require_scalar(
            values,
            "minimum_path_duration_seconds",
            "closed-loop reference",
        ),
        "closed-loop reference minimum_path_duration_seconds",
        strictly_positive=True,
    )
    minimum_path_excursion_m = parse_float(
        require_scalar(
            values,
            "minimum_path_excursion_m",
            "closed-loop reference",
        ),
        "closed-loop reference minimum_path_excursion_m",
        strictly_positive=True,
    )
    return ClosedLoopReference(
        method=method,
        position_tolerance_m=position_tolerance,
        orientation_tolerance_deg=orientation_tolerance,
        endpoint_window_seconds=endpoint_window_seconds,
        minimum_endpoint_samples=minimum_endpoint_samples,
        maximum_endpoint_position_spread_m=(
            maximum_endpoint_position_spread_m
        ),
        maximum_endpoint_orientation_spread_deg=(
            maximum_endpoint_orientation_spread_deg
        ),
        minimum_path_duration_seconds=minimum_path_duration_seconds,
        minimum_path_excursion_m=minimum_path_excursion_m,
    )


def endpoint_window_stats(
    trajectory: TrajectoryStats,
    indices: tuple[int, ...],
) -> EndpointWindowStats:
    if not indices:
        raise BenchmarkError("endpoint window contains no trajectory samples")
    positions = tuple(
        trajectory.poses[index][:3] for index in indices
    )
    quaternions = tuple(
        trajectory.poses[index][3:] for index in indices
    )
    position = tuple(
        statistics.median(axis) for axis in zip(*positions)
    )
    reference_quaternion = quaternions[0]
    aligned_quaternions = []
    for quaternion in quaternions:
        dot = sum(
            left * right
            for left, right in zip(quaternion, reference_quaternion)
        )
        sign = -1.0 if dot < 0.0 else 1.0
        aligned_quaternions.append(
            tuple(sign * component for component in quaternion)
        )
    quaternion_sum = tuple(
        sum(axis) for axis in zip(*aligned_quaternions)
    )
    quaternion_norm = math.sqrt(
        sum(component * component for component in quaternion_sum)
    )
    if quaternion_norm <= 1e-12:
        raise BenchmarkError(
            "endpoint window quaternion average is degenerate"
        )
    quaternion = tuple(
        component / quaternion_norm for component in quaternion_sum
    )
    timestamps = tuple(
        trajectory.timestamps_s[index] for index in indices
    )
    return EndpointWindowStats(
        samples=len(indices),
        duration_s=timestamps[-1] - timestamps[0],
        position=position,
        quaternion=quaternion,
        maximum_position_spread_m=max(
            math.dist(sample, position) for sample in positions
        ),
        maximum_orientation_spread_deg=max(
            quaternion_return_angle_deg(sample, quaternion)
            for sample in quaternions
        ),
    )


def closed_loop_window_stats(
    trajectory: TrajectoryStats,
    endpoint_window_seconds: float,
) -> ClosedLoopWindowStats:
    first_timestamp = trajectory.timestamps_s[0]
    last_timestamp = trajectory.timestamps_s[-1]
    start_indices = tuple(
        index
        for index, timestamp in enumerate(trajectory.timestamps_s)
        if timestamp
        <= first_timestamp + endpoint_window_seconds
        + TIMESTAMP_TOLERANCE_S
    )
    end_indices = tuple(
        index
        for index, timestamp in enumerate(trajectory.timestamps_s)
        if timestamp
        >= last_timestamp - endpoint_window_seconds
        - TIMESTAMP_TOLERANCE_S
    )
    start = endpoint_window_stats(trajectory, start_indices)
    end = endpoint_window_stats(trajectory, end_indices)
    return ClosedLoopWindowStats(
        start=start,
        end=end,
        position_residual_m=math.dist(start.position, end.position),
        orientation_residual_deg=quaternion_return_angle_deg(
            start.quaternion, end.quaternion
        ),
    )


def validate_provenance(
    run_dir: Path,
    live_manifest_path: Path,
    backend_pin_path: Path,
    backend_patch_path: Path,
    backend_library_path: Path,
    live_executable_path: Path,
    vocabulary_path: Path,
) -> tuple[
    dict[str, str],
    dict[str, str],
    dict[str, str],
    dict[str, str],
    str,
]:
    for path, label in (
        (live_manifest_path, "live bundle manifest"),
        (backend_pin_path, "backend pin"),
        (backend_patch_path, "backend patch"),
        (backend_library_path, "backend library"),
        (live_executable_path, "live executable"),
        (vocabulary_path, "ORB vocabulary"),
    ):
        require_file(path, label, allow_empty=False)
    metadata_path = run_dir / "run_metadata.yaml"
    settings_path = run_dir / "resolved_orbslam3_settings.yaml"
    device_path = run_dir / "device_report.yaml"
    for path, label in (
        (metadata_path, "run metadata"),
        (settings_path, "resolved ORB settings"),
        (device_path, "device report"),
    ):
        require_file(path, label, allow_empty=False)

    metadata = simple_yaml_map(metadata_path)
    manifest = simple_yaml_map(live_manifest_path)
    pin = simple_yaml_map(backend_pin_path)
    device = simple_yaml_map(device_path)
    bundle_format = manifest.get("format", "")
    if bundle_format not in (
        "ovrs-orbslam3-live-bundle-v4",
        "ovrs-orbslam3-live-bundle-v5",
    ):
        raise BenchmarkError(
            f"unsupported live bundle format: {bundle_format}"
        )
    visual_support_contract = (
        bundle_format == "ovrs-orbslam3-live-bundle-v5"
    )
    trajectory_policy = (
        "startup_imu_pass_post_inertial_ba2_stable_tracking_minimum_"
        "visual_support_continuous_bounded_preacceptance_resets_"
        "zero_postacceptance_resets_no_postacceptance_map_correction"
        if visual_support_contract
        else
        "startup_imu_pass_post_inertial_ba2_stable_tracking_continuous_"
        "bounded_preacceptance_resets_zero_postacceptance_resets_"
        "no_postacceptance_map_correction"
    )
    expected_metadata = {
        "mode": "experimental_pure_orbslam3_live",
        "openvins_pose_consumed": "false",
        "global_correction_fed_to_openvins": "false",
        "trajectory_acceptance_policy": trajectory_policy,
        "visual_tracking_trajectory_is_diagnostic_only": "true",
    }
    expected_manifest = {
        "format": bundle_format,
        "state": "PREPARED_NOT_RUN",
        "integration": "PURE_ORB_SLAM3_STEREO_INERTIAL",
        "openvins_pose_consumed": "false",
        "global_correction_fed_to_openvins": "false",
    }
    expected_pin = {
        "format": "ovrs-orbslam3-backend-pin-v1",
        "backend_name": "ORB_SLAM3",
    }
    for values, expected, label in (
        (metadata, expected_metadata, "run metadata"),
        (manifest, expected_manifest, "live bundle manifest"),
        (pin, expected_pin, "backend pin"),
    ):
        for key, value in expected.items():
            require_equal(values.get(key, ""), value, f"{label} {key}")

    patch_hash = sha256_file(backend_patch_path)
    require_equal(
        patch_hash,
        require_scalar(pin, "patch_sha256", "backend pin"),
        "backend patch hash versus pin",
    )
    for values, label in (
        (metadata, "run metadata"),
        (manifest, "live bundle manifest"),
    ):
        require_equal(
            require_scalar(values, "backend_commit", label),
            require_scalar(pin, "commit", "backend pin"),
            f"{label} backend commit versus pin",
        )
        require_equal(
            require_scalar(values, "backend_patch_sha256", label),
            patch_hash,
            f"{label} backend patch hash",
        )
    require_equal(
        sha256_file(run_dir / "resolved_orbslam3_settings.yaml"),
        require_scalar(manifest, "settings_sha256", "live bundle manifest"),
        "resolved ORB settings hash versus bundle",
    )
    require_equal(
        sha256_file(backend_library_path),
        require_scalar(
            metadata, "backend_library_sha256_at_build", "run metadata"
        ),
        "backend library hash versus build identity",
    )
    runtime_provenance_format = metadata.get(
        "runtime_provenance_format", ""
    )
    expected_runtime_provenance_format = (
        "ovrs-orbslam3-live-runtime-provenance-v6"
        if visual_support_contract
        else "ovrs-orbslam3-live-runtime-provenance-v5"
    )
    if runtime_provenance_format == expected_runtime_provenance_format:
        launch_path = run_dir / "launch_provenance.yaml"
        captured_manifest_path = run_dir / "source_live_manifest.yaml"
        require_file(launch_path, "launch provenance", allow_empty=False)
        require_file(
            captured_manifest_path,
            "captured live bundle manifest",
            allow_empty=False,
        )
        launch = simple_yaml_map(launch_path)
        for key, expected in {
            "format": "ovrs-orbslam3-live-launch-provenance-v1",
            "state": "LAUNCHED_NOT_CAPTURE_VALIDATED",
            "integration": "PURE_ORB_SLAM3_STEREO_INERTIAL",
            "openvins_pose_consumed": "false",
            "global_correction_fed_to_openvins": "false",
        }.items():
            require_equal(
                launch.get(key, ""), expected, f"launch provenance {key}"
            )
        require_equal(
            sha256_file(launch_path),
            require_scalar(
                metadata, "launch_provenance_sha256", "run metadata"
            ),
            "launch provenance hash",
        )
        require_equal(
            sha256_file(captured_manifest_path),
            sha256_file(live_manifest_path),
            "captured versus supplied live bundle manifest",
        )
        runtime_hashes = {
            "live_executable_sha256_at_start":
                sha256_file(live_executable_path),
            "backend_library_sha256_at_start":
                sha256_file(backend_library_path),
            "vocabulary_sha256_at_start": sha256_file(vocabulary_path),
            "settings_sha256_at_start":
                sha256_file(run_dir / "resolved_orbslam3_settings.yaml"),
            "live_bundle_manifest_sha256_at_start":
                sha256_file(live_manifest_path),
        }
        for key, actual_hash in runtime_hashes.items():
            require_equal(
                require_scalar(metadata, key, "run metadata"),
                actual_hash,
                f"run metadata {key}",
            )
            require_equal(
                require_scalar(launch, key, "launch provenance"),
                actual_hash,
                f"launch provenance {key}",
            )
        source_fingerprint = require_scalar(
            metadata, "source_fingerprint_at_start", "run metadata"
        )
        if (
            len(source_fingerprint) != 64
            or any(character not in "0123456789abcdef"
                   for character in source_fingerprint)
        ):
            raise BenchmarkError(
                "run metadata source_fingerprint_at_start is not SHA-256"
            )
        require_equal(
            require_scalar(
                launch, "source_fingerprint", "launch provenance"
            ),
            source_fingerprint,
            "launch versus runtime source fingerprint",
        )
        executable_binding_state = "CAPTURE_TIME_ATTESTED"
    elif runtime_provenance_format:
        raise BenchmarkError(
            "unsupported runtime provenance format: "
            f"{runtime_provenance_format}"
        )
    else:
        executable_binding_state = (
            "LEGACY_RUN_CALLER_SUPPLIED_BINARY_NOT_CAPTURE_TIME_ATTESTED"
        )
    for key in ("camera_serial", "calibration_state", "camera_stride"):
        require_equal(
            require_scalar(metadata, key, "run metadata"),
            require_scalar(manifest, key, "live bundle manifest"),
            f"{key} metadata versus bundle",
        )
    require_equal(
        require_scalar(device, "serial", "device report"),
        require_scalar(metadata, "camera_serial", "run metadata"),
        "device serial versus run metadata",
    )
    for key in (
        "camera_imu_time_offset_s",
        "imu_init_acceleration_threshold_m_s2",
        "minimum_stable_inertial_seconds",
        "maximum_tracking_interval_seconds",
        "maximum_tracking_interval_factor",
        "maximum_preacceptance_map_resets",
        "gravity_m_s2",
        "startup_maximum_gravity_error_m_s2",
        "startup_stationary_seconds",
        "startup_stationary_timeout_seconds",
        "startup_maximum_acceleration_stddev_m_s2",
        "startup_maximum_gyro_magnitude_rad_s",
        "maximum_input_stall_seconds",
    ):
        require_close(
            parse_float(require_scalar(metadata, key, "run metadata"), key),
            parse_float(
                require_scalar(manifest, key, "live bundle manifest"), key
            ),
            f"{key} metadata versus bundle",
        )
    if visual_support_contract:
        require_equal(
            parse_int(
                require_scalar(
                    metadata, "minimum_tracked_map_points", "run metadata"
                ),
                "minimum_tracked_map_points",
            ),
            parse_int(
                require_scalar(
                    manifest,
                    "minimum_tracked_map_points",
                    "live bundle manifest",
                ),
                "minimum_tracked_map_points",
            ),
            "minimum_tracked_map_points metadata versus bundle",
        )
    for pin_key, manifest_key in (
        (
            "live_minimum_tracked_map_points",
            "minimum_tracked_map_points",
        ),
        (
            "live_maximum_preacceptance_map_resets",
            "maximum_preacceptance_map_resets",
        ),
        ("live_startup_stationary_seconds", "startup_stationary_seconds"),
        (
            "live_startup_stationary_timeout_seconds",
            "startup_stationary_timeout_seconds",
        ),
        (
            "live_startup_maximum_acceleration_stddev_m_s2",
            "startup_maximum_acceleration_stddev_m_s2",
        ),
        (
            "live_startup_maximum_gyro_magnitude_rad_s",
            "startup_maximum_gyro_magnitude_rad_s",
        ),
        (
            "live_maximum_input_stall_seconds",
            "maximum_input_stall_seconds",
        ),
    ):
        if (
            manifest_key == "minimum_tracked_map_points"
            and not visual_support_contract
        ):
            continue
        require_close(
            parse_float(
                require_scalar(pin, pin_key, "backend pin"), pin_key,
                strictly_positive=True,
            ),
            parse_float(
                require_scalar(manifest, manifest_key, "live bundle manifest"),
                manifest_key,
                strictly_positive=True,
            ),
            f"{manifest_key} versus backend pin",
        )
    orb_camera_fps = parse_int(
        require_scalar(manifest, "orb_camera_fps", "live bundle manifest"),
        "live bundle orb_camera_fps",
        minimum=1,
    )
    maximum_tracking_interval_factor = parse_float(
        require_scalar(
            metadata,
            "maximum_tracking_interval_factor",
            "run metadata",
        ),
        "maximum_tracking_interval_factor",
        strictly_positive=True,
    )
    if not 1.0 < maximum_tracking_interval_factor <= 10.0:
        raise BenchmarkError(
            "maximum_tracking_interval_factor must be in (1, 10]"
        )
    maximum_tracking_interval_seconds = parse_float(
        require_scalar(
            metadata,
            "maximum_tracking_interval_seconds",
            "run metadata",
        ),
        "maximum_tracking_interval_seconds",
        strictly_positive=True,
    )
    require_close(
        maximum_tracking_interval_seconds,
        maximum_tracking_interval_factor / orb_camera_fps,
        "maximum tracking interval derivation",
    )
    return metadata, manifest, pin, device, executable_binding_state


def summary_bool(
    summary: dict[str, str], key: str
) -> bool:
    return parse_bool(require_scalar(summary, key, "run summary"), key)


def summary_int(summary: dict[str, str], key: str) -> int:
    return parse_int(require_scalar(summary, key, "run summary"), key)


def bool_yaml(value: bool) -> str:
    return "true" if value else "false"


def evaluate(args: argparse.Namespace) -> None:
    run_dir = args.run_dir.resolve()
    if not run_dir.is_dir():
        raise BenchmarkError(f"run directory does not exist: {run_dir}")
    output = (
        args.output.resolve()
        if args.output is not None
        else run_dir / "live_evaluation.yaml"
    )
    if output.exists():
        raise BenchmarkError(f"output already exists: {output}")
    if output.parent != run_dir and not output.parent.is_dir():
        raise BenchmarkError(f"output parent does not exist: {output.parent}")

    metadata, manifest, pin, device, executable_binding_state = (
        validate_provenance(
        run_dir,
        args.live_bundle_manifest.resolve(),
        args.backend_pin.resolve(),
        args.backend_patch.resolve(),
        args.backend_library.resolve(),
        args.live_executable.resolve(),
        args.vocabulary.resolve(),
        )
    )
    summary_path = run_dir / "run_summary.yaml"
    require_file(summary_path, "run summary", allow_empty=False)
    summary = simple_yaml_map(summary_path)
    summary_state = require_scalar(summary, "state", "run summary")
    if summary_state not in (
        "EXPERIMENTAL_RUN_COMPLETE",
        "EXPERIMENTAL_RUN_FAILED",
    ):
        raise BenchmarkError(f"unsupported run summary state: {summary_state}")

    minimum_stable_s = parse_float(
        require_scalar(
            metadata, "minimum_stable_inertial_seconds", "run metadata"
        ),
        "minimum_stable_inertial_seconds",
        strictly_positive=True,
    )
    maximum_tracking_interval_s = parse_float(
        require_scalar(
            metadata, "maximum_tracking_interval_seconds", "run metadata"
        ),
        "maximum_tracking_interval_seconds",
        strictly_positive=True,
    )
    maximum_preacceptance_map_resets = parse_int(
        require_scalar(
            metadata, "maximum_preacceptance_map_resets", "run metadata"
        ),
        "maximum_preacceptance_map_resets",
    )
    if maximum_preacceptance_map_resets > 20:
        raise BenchmarkError(
            "maximum_preacceptance_map_resets exceeds supported bound"
        )
    visual_support_contract = (
        manifest.get("format") == "ovrs-orbslam3-live-bundle-v5"
    )
    minimum_tracked_map_points = (
        parse_int(
            require_scalar(
                metadata, "minimum_tracked_map_points", "run metadata"
            ),
            "minimum_tracked_map_points",
            minimum=1,
        )
        if visual_support_contract
        else 0
    )
    tracking_path = run_dir / "live_tracking_states.csv"
    imu_path = run_dir / "live_imu_excitation.csv"
    visual_path = run_dir / "live_visual_tracking_trajectory_tum.txt"
    tracking = parse_tracking(
        tracking_path,
        minimum_stable_s,
        maximum_tracking_interval_s,
        maximum_preacceptance_map_resets,
        minimum_tracked_map_points,
    )
    parse_imu(imu_path, tracking)
    visual = parse_trajectory(
        visual_path,
        "visual diagnostic trajectory",
        allow_empty=True,
        minimum_rows=1,
    )

    accepted_path = run_dir / "live_camera_trajectory_tum.txt"
    candidate_path = run_dir / "live_camera_trajectory_candidate_tum.txt"
    accepted: TrajectoryStats | None = None
    candidate: TrajectoryStats | None = None
    if accepted_path.exists():
        accepted = parse_trajectory(accepted_path, "accepted trajectory")
    if candidate_path.exists():
        candidate = parse_trajectory(
            candidate_path,
            "rejected candidate trajectory",
            allow_empty=True,
            minimum_rows=1,
        )
    if accepted is not None and candidate_path.exists():
        raise BenchmarkError(
            "accepted and rejected candidate trajectory files coexist"
        )
    if accepted is None and not candidate_path.exists():
        raise BenchmarkError(
            "run lacks both accepted and rejected candidate trajectory files"
        )
    recorded_candidate = accepted if accepted is not None else candidate

    require_equal(
        len(tracking.rows),
        summary_int(summary, "submitted_stereo"),
        "tracking rows versus submitted_stereo",
    )
    require_equal(
        tracking.visual_pose_count,
        summary_int(summary, "visual_pose_count"),
        "visual pose count",
    )
    require_equal(
        0 if visual is None else visual.rows,
        summary_int(summary, "visual_pose_count"),
        "visual trajectory rows",
    )
    require_equal(
        tracking.lost_frame_count,
        summary_int(summary, "lost_frame_count"),
        "lost frame count",
    )
    require_equal(
        tracking.tracking_loss_after_acceptance_count,
        summary_int(summary, "tracking_loss_after_acceptance_count"),
        "tracking loss after acceptance count",
    )
    require_equal(
        tracking.tracking_gap_after_acceptance_count,
        summary_int(summary, "tracking_gap_after_acceptance_count"),
        "tracking gap after acceptance count",
    )
    if visual_support_contract:
        require_equal(
            minimum_tracked_map_points,
            summary_int(summary, "minimum_tracked_map_points"),
            "minimum tracked map points",
        )
        require_equal(
            tracking.visual_support_failure_after_acceptance_count,
            summary_int(
                summary,
                "visual_support_failure_after_acceptance_count",
            ),
            "visual support failure after acceptance count",
        )
    require_close(
        tracking.maximum_observed_tracking_interval_s,
        parse_float(
            require_scalar(
                summary,
                "maximum_observed_tracking_interval_seconds",
                "run summary",
            ),
            "maximum_observed_tracking_interval_seconds",
            minimum=0.0,
        ),
        "maximum observed tracking interval",
    )
    require_equal(
        len(tracking.accepted_timestamps_s),
        summary_int(summary, "candidate_pose_count"),
        "candidate tracking rows",
    )
    require_equal(
        summary_int(summary, "candidate_pose_count"),
        0 if recorded_candidate is None else recorded_candidate.rows,
        "candidate pose count",
    )
    require_equal(
        summary_int(summary, "accepted_pose_count"),
        (
            summary_int(summary, "candidate_pose_count")
            if summary_state == "EXPERIMENTAL_RUN_COMPLETE"
            else 0
        ),
        "accepted pose count versus terminal state",
    )
    require_equal(
        tracking.rows[-1].state,
        require_scalar(summary, "last_tracking_state", "run summary"),
        "last tracking state",
    )
    startup_passed = summary_bool(summary, "startup_imu_gate_passed")
    require_equal(
        tracking.rows[-1].startup_imu_gate_passed,
        startup_passed,
        "final startup IMU gate state",
    )
    startup_state = require_scalar(summary, "startup_imu_gate_state", "run summary")
    if startup_state not in (
        "COLLECTING",
        "PASSED",
        "GRAVITY_MISMATCH",
        "STATIONARY_TIMEOUT",
    ):
        raise BenchmarkError(f"unsupported startup IMU gate state: {startup_state}")
    require_equal(startup_state == "PASSED", startup_passed, "startup gate state")
    startup_threshold_fields = (
        "gravity_m_s2",
        "startup_maximum_gravity_error_m_s2",
        "startup_stationary_seconds",
        "startup_stationary_timeout_seconds",
        "startup_maximum_acceleration_stddev_m_s2",
        "startup_maximum_gyro_magnitude_rad_s",
    )
    for field in startup_threshold_fields:
        require_close(
            parse_float(require_scalar(summary, field, "run summary"), field),
            parse_float(require_scalar(metadata, field, "run metadata"), field),
            f"{field} summary versus metadata",
        )
    startup_samples = summary_int(summary, "startup_imu_gate_samples")
    startup_window_duration = parse_float(
        require_scalar(summary, "startup_imu_gate_window_duration_s", "run summary"),
        "startup_imu_gate_window_duration_s",
        minimum=0.0,
    )
    startup_accel_stddev = parse_float(
        require_scalar(
            summary,
            "startup_imu_acceleration_magnitude_stddev_m_s2",
            "run summary",
        ),
        "startup_imu_acceleration_magnitude_stddev_m_s2",
        minimum=0.0,
    )
    startup_maximum_gyro = parse_float(
        require_scalar(
            summary, "startup_imu_maximum_gyro_magnitude_rad_s", "run summary"
        ),
        "startup_imu_maximum_gyro_magnitude_rad_s",
        minimum=0.0,
    )
    startup_gravity_error = parse_float(
        require_scalar(summary, "startup_imu_gravity_error_m_s2", "run summary"),
        "startup_imu_gravity_error_m_s2",
        minimum=0.0,
    )
    if startup_passed:
        if startup_samples < 2:
            raise BenchmarkError("passing startup IMU gate has fewer than 2 samples")
        if startup_window_duration + FLOAT_TOLERANCE < parse_float(
            metadata["startup_stationary_seconds"], "startup_stationary_seconds"
        ):
            raise BenchmarkError("passing startup IMU window is too short")
        if startup_accel_stddev > parse_float(
            metadata["startup_maximum_acceleration_stddev_m_s2"],
            "startup_maximum_acceleration_stddev_m_s2",
        ) + FLOAT_TOLERANCE:
            raise BenchmarkError("passing startup IMU acceleration is too dynamic")
        if startup_maximum_gyro > parse_float(
            metadata["startup_maximum_gyro_magnitude_rad_s"],
            "startup_maximum_gyro_magnitude_rad_s",
        ) + FLOAT_TOLERANCE:
            raise BenchmarkError("passing startup IMU gyro is too dynamic")
        if startup_gravity_error > parse_float(
            metadata["startup_maximum_gravity_error_m_s2"],
            "startup_maximum_gravity_error_m_s2",
        ) + FLOAT_TOLERANCE:
            raise BenchmarkError("passing startup IMU gravity error is excessive")

    maximum_input_stall_seconds = parse_float(
        require_scalar(
            summary, "maximum_input_stall_seconds", "run summary"
        ),
        "maximum_input_stall_seconds",
        strictly_positive=True,
    )
    require_close(
        maximum_input_stall_seconds,
        parse_float(
            require_scalar(
                metadata, "maximum_input_stall_seconds", "run metadata"
            ),
            "maximum_input_stall_seconds",
            strictly_positive=True,
        ),
        "maximum input stall threshold summary versus metadata",
    )
    maximum_stereo_wall_gap_seconds = parse_float(
        require_scalar(
            summary,
            "maximum_observed_stereo_wall_gap_seconds",
            "run summary",
        ),
        "maximum_observed_stereo_wall_gap_seconds",
        minimum=0.0,
    )
    maximum_imu_wall_gap_seconds = parse_float(
        require_scalar(
            summary,
            "maximum_observed_imu_wall_gap_seconds",
            "run summary",
        ),
        "maximum_observed_imu_wall_gap_seconds",
        minimum=0.0,
    )
    input_stall_detected = summary_bool(summary, "input_stall_detected")
    maximum_input_wall_gap_seconds = max(
        maximum_stereo_wall_gap_seconds, maximum_imu_wall_gap_seconds
    )
    # The C++ summary currently serializes these values to six decimal
    # places. Preserve fail-closed checking without rejecting a legitimate
    # just-over-threshold detection that rounds to the threshold itself.
    if input_stall_detected:
        if (
            maximum_input_wall_gap_seconds + FLOAT_TOLERANCE
            < maximum_input_stall_seconds
        ):
            raise BenchmarkError(
                "input stall flag is not supported by the observed wall gaps"
            )
    elif (
        maximum_input_wall_gap_seconds
        > maximum_input_stall_seconds + FLOAT_TOLERANCE
    ):
        raise BenchmarkError(
            "over-limit input wall gap is missing the terminal stall flag"
        )
    parse_float(
        require_scalar(summary, "capture_duration_s", "run summary"),
        "capture_duration_s",
        strictly_positive=True,
    )
    parse_float(
        require_scalar(summary, "shutdown_duration_s", "run summary"),
        "shutdown_duration_s",
        minimum=0.0,
    )

    tracking_latencies = [row.tracking_latency_ms for row in tracking.rows]
    tracking_frame_budget_ms = 1000.0 / parse_int(
        require_scalar(manifest, "orb_camera_fps", "live bundle manifest"),
        "orb_camera_fps",
        minimum=1,
    )
    tracking_latency_misses = sum(
        latency > tracking_frame_budget_ms for latency in tracking_latencies
    )
    require_equal(
        len(tracking_latencies),
        summary_int(summary, "tracking_latency_samples"),
        "tracking latency samples",
    )
    require_close(
        statistics.fmean(tracking_latencies),
        parse_float(
            require_scalar(summary, "tracking_latency_mean_ms", "run summary"),
            "tracking_latency_mean_ms",
            minimum=0.0,
        ),
        "tracking latency mean",
    )
    require_close(
        max(tracking_latencies),
        parse_float(
            require_scalar(summary, "tracking_latency_maximum_ms", "run summary"),
            "tracking_latency_maximum_ms",
            minimum=0.0,
        ),
        "tracking latency maximum",
    )
    require_close(
        tracking_frame_budget_ms,
        parse_float(
            require_scalar(summary, "tracking_frame_budget_ms", "run summary"),
            "tracking_frame_budget_ms",
            strictly_positive=True,
        ),
        "tracking frame budget",
    )
    require_equal(
        tracking_latency_misses,
        summary_int(summary, "tracking_latency_frame_budget_miss_count"),
        "tracking latency frame-budget misses",
    )
    require_equal(
        tracking.ever_inertial_initialized,
        summary_bool(summary, "ever_inertial_initialized"),
        "ever inertial initialized",
    )
    require_equal(
        tracking.rows[-1].inertial_initialized,
        summary_bool(summary, "inertial_initialized"),
        "final inertial initialized",
    )
    require_equal(
        tracking.ever_inertial_ba2_finished,
        summary_bool(summary, "ever_inertial_ba2_finished"),
        "ever BA2 finished",
    )
    require_equal(
        tracking.rows[-1].inertial_ba2_finished,
        summary_bool(summary, "inertial_ba2_finished"),
        "final BA2 finished",
    )
    require_equal(
        tracking.rows[-1].reset_count,
        summary_int(summary, "active_map_reset_count"),
        "final reset count",
    )
    require_equal(
        tracking.rows[-1].map_change_index,
        summary_int(summary, "active_map_change_index"),
        "final map change index",
    )
    require_equal(
        tracking.pending_reset_observed,
        summary_bool(summary, "pending_reset_observed"),
        "pending reset observation",
    )
    require_equal(
        tracking.pending_reset_after_acceptance_observed,
        summary_bool(summary, "pending_reset_after_acceptance_observed"),
        "pending reset after acceptance observation",
    )
    require_equal(
        tracking.preacceptance_map_reset_count,
        summary_int(summary, "preacceptance_map_reset_count"),
        "preacceptance map reset count",
    )
    require_equal(
        tracking.postacceptance_map_reset_count,
        summary_int(summary, "postacceptance_map_reset_count"),
        "postacceptance map reset count",
    )
    require_equal(
        tracking.preacceptance_map_reset_count
        > maximum_preacceptance_map_resets,
        summary_bool(summary, "preacceptance_reset_limit_exceeded"),
        "preacceptance reset limit state",
    )
    require_equal(
        tracking.rows[-1].reset_pending,
        summary_bool(summary, "reset_pending_at_shutdown"),
        "pending reset at shutdown",
    )
    require_equal(
        tracking.inertial_regressions,
        summary_int(summary, "inertial_regression_count"),
        "inertial regression count",
    )
    require_equal(
        tracking.ba2_regressions,
        summary_int(summary, "inertial_ba2_regression_count"),
        "BA2 regression count",
    )
    require_equal(
        tracking.map_change_after_acceptance,
        summary_bool(summary, "map_change_after_acceptance"),
        "map change after acceptance",
    )
    require_equal(
        tracking.acceptance_discontinuities > 0,
        summary_bool(summary, "trajectory_discontinuity_detected"),
        "trajectory discontinuity",
    )
    require_equal(
        bool(tracking.accepted_timestamps_s),
        summary_bool(summary, "trajectory_acceptance_started"),
        "trajectory acceptance started",
    )

    accepted_name = summary.get("accepted_trajectory_file", "")
    rejected_name = summary.get("rejected_candidate_trajectory_file", "")
    require_equal(
        accepted_name,
        accepted_path.name if accepted is not None else "",
        "accepted trajectory filename",
    )
    require_equal(
        rejected_name,
        candidate_path.name if candidate_path.exists() else "",
        "rejected trajectory filename",
    )
    if recorded_candidate is not None:
        require_equal(
            recorded_candidate.rows,
            len(tracking.accepted_timestamps_s),
            "candidate trajectory rows versus tracking acceptance",
        )
        for index, (pose_timestamp, state_timestamp) in enumerate(
            zip(
                recorded_candidate.timestamps_s,
                tracking.accepted_timestamps_s,
            ),
            1,
        ):
            require_close(
                pose_timestamp,
                state_timestamp,
                f"candidate trajectory timestamp {index}",
                TIMESTAMP_TOLERANCE_S,
            )
        visual_index = 0
        for candidate_index, (
            candidate_timestamp,
            candidate_pose,
        ) in enumerate(
            zip(
                recorded_candidate.timestamps_s,
                recorded_candidate.poses,
            ),
            1,
        ):
            if visual is None:
                raise BenchmarkError(
                    "candidate trajectory exists without a visual "
                    "diagnostic trajectory"
                )
            while (
                visual_index < visual.rows
                and visual.timestamps_s[visual_index]
                < candidate_timestamp - TIMESTAMP_TOLERANCE_S
            ):
                visual_index += 1
            if visual_index >= visual.rows:
                raise BenchmarkError(
                    f"candidate trajectory row {candidate_index} has no "
                    "visual diagnostic row"
                )
            require_close(
                visual.timestamps_s[visual_index],
                candidate_timestamp,
                f"candidate versus visual timestamp {candidate_index}",
                TIMESTAMP_TOLERANCE_S,
            )
            for component, (candidate_value, visual_value) in enumerate(
                zip(candidate_pose, visual.poses[visual_index]), 1
            ):
                require_close(
                    candidate_value,
                    visual_value,
                    "candidate versus visual pose "
                    f"{candidate_index} component {component}",
                    FLOAT_TOLERANCE,
                )

    clean_transport_fields = (
        "dropped_imu",
        "dropped_stereo",
        "rejected_nonmonotonic",
        "stereo_without_imu_coverage",
        "stereo_discarded_on_shutdown",
    )
    gate_failures: list[str] = []
    if summary_state != "EXPERIMENTAL_RUN_COMPLETE":
        gate_failures.append("RUN_NOT_COMPLETE")
    if (run_dir / "INCOMPLETE").exists():
        gate_failures.append("INCOMPLETE_MARKER_PRESENT")
    runtime_failure = summary.get("runtime_failure", "")
    if runtime_failure:
        gate_failures.append("RUNTIME_FAILURE_REPORTED")
    if accepted is None:
        gate_failures.append("CANONICAL_TRAJECTORY_ABSENT")
    if not tracking.rows[-1].startup_imu_gate_passed:
        gate_failures.append("STARTUP_IMU_GATE_NOT_PASSED")
    if input_stall_detected:
        gate_failures.append("INPUT_STALL_DETECTED")
    if not tracking.rows[-1].inertial_initialized:
        gate_failures.append("INERTIAL_NOT_INITIALIZED_AT_SHUTDOWN")
    if not tracking.rows[-1].inertial_ba2_finished:
        gate_failures.append("INERTIAL_BA2_NOT_FINISHED_AT_SHUTDOWN")
    if (
        tracking.preacceptance_map_reset_count
        > maximum_preacceptance_map_resets
    ):
        gate_failures.append("PREACCEPTANCE_MAP_RESET_LIMIT_EXCEEDED")
    if tracking.postacceptance_map_reset_count:
        gate_failures.append("POSTACCEPTANCE_MAP_RESET_OBSERVED")
    if tracking.pending_reset_after_acceptance_observed:
        gate_failures.append("POSTACCEPTANCE_PENDING_RESET_OBSERVED")
    if tracking.rows[-1].reset_pending:
        gate_failures.append("RESET_PENDING_AT_SHUTDOWN")
    if tracking.map_change_after_acceptance:
        gate_failures.append("MAP_CHANGE_AFTER_ACCEPTANCE")
    if tracking.acceptance_discontinuities:
        gate_failures.append("TRAJECTORY_ACCEPTANCE_DISCONTINUITY")
    if tracking.tracking_loss_after_acceptance_count:
        gate_failures.append("TRACKING_LOSS_AFTER_ACCEPTANCE")
    if tracking.tracking_gap_after_acceptance_count:
        gate_failures.append("TRACKING_GAP_AFTER_ACCEPTANCE")
    if tracking.visual_support_failure_after_acceptance_count:
        gate_failures.append("VISUAL_SUPPORT_LOST_AFTER_ACCEPTANCE")
    if not tracking.accepted_timestamps_s:
        gate_failures.append("NO_ACCEPTED_TRAJECTORY_POSES")
    for field in clean_transport_fields:
        if summary_int(summary, field) != 0:
            gate_failures.append(f"NONZERO_{field.upper()}")

    maximum_adjacent_limit = args.maximum_adjacent_translation_m
    if maximum_adjacent_limit is not None:
        if not math.isfinite(maximum_adjacent_limit) or maximum_adjacent_limit <= 0:
            raise BenchmarkError(
                "--maximum-adjacent-translation-m must be finite and positive"
            )
        continuity_envelope_state = "EVALUATED_CALLER_SUPPLIED_LIMIT"
        if (
            accepted is not None
            and accepted.maximum_adjacent_translation_m
            > maximum_adjacent_limit
        ):
            gate_failures.append(
                "MAXIMUM_ADJACENT_TRANSLATION_LIMIT_EXCEEDED"
            )
    else:
        continuity_envelope_state = (
            "NOT_EVALUATED_NO_OPERATIONAL_ENVELOPE"
        )

    continuity_gate_failures = list(dict.fromkeys(gate_failures))
    reference = parse_closed_loop_reference(args.closed_loop_reference)
    reference_passed = False
    reference_window_stats: ClosedLoopWindowStats | None = None
    reference_failures: list[str] = []
    if reference is None:
        reference_state = "NOT_EVALUATED_NO_INDEPENDENT_REFERENCE"
    elif metadata.get("closed_loop_reference_start_policy", "") != (
        "post_acceptance_gate_open_operator_cue"
    ):
        raise BenchmarkError(
            "closed-loop reference requires a run that records the "
            "post-acceptance gate-open operator-cue start policy"
        )
    elif accepted is None:
        reference_state = "NOT_EVALUATED_NO_CANONICAL_TRAJECTORY"
        reference_failures.append("CLOSED_LOOP_REFERENCE_NOT_EVALUABLE")
    else:
        reference_window_stats = closed_loop_window_stats(
            accepted, reference.endpoint_window_seconds
        )
        reference_quality_failures: list[str] = []
        maximum_path_excursion_m = max(
            math.dist(
                pose[:3], reference_window_stats.start.position
            )
            for pose in accepted.poses
        )
        if accepted.duration_s < reference.minimum_path_duration_seconds:
            reference_quality_failures.append(
                "CLOSED_LOOP_PATH_DURATION_BELOW_REFERENCE_MINIMUM"
            )
        if maximum_path_excursion_m < reference.minimum_path_excursion_m:
            reference_quality_failures.append(
                "CLOSED_LOOP_PATH_EXCURSION_BELOW_REFERENCE_MINIMUM"
            )
        if accepted.duration_s <= 2.0 * reference.endpoint_window_seconds:
            reference_quality_failures.append(
                "CLOSED_LOOP_ENDPOINT_WINDOWS_OVERLAP"
            )
        required_window_span_s = max(
            0.0,
            reference.endpoint_window_seconds
            - maximum_tracking_interval_s,
        )
        for label, window in (
            ("START", reference_window_stats.start),
            ("END", reference_window_stats.end),
        ):
            if (
                window.samples < reference.minimum_endpoint_samples
                or window.duration_s + TIMESTAMP_TOLERANCE_S
                < required_window_span_s
            ):
                reference_quality_failures.append(
                    f"CLOSED_LOOP_{label}_HOLD_INSUFFICIENT"
                )
            if (
                window.maximum_position_spread_m
                > reference.maximum_endpoint_position_spread_m
            ):
                reference_quality_failures.append(
                    f"CLOSED_LOOP_{label}_POSITION_SPREAD_EXCEEDED"
                )
            if (
                window.maximum_orientation_spread_deg
                > reference.maximum_endpoint_orientation_spread_deg
            ):
                reference_quality_failures.append(
                    f"CLOSED_LOOP_{label}_ORIENTATION_SPREAD_EXCEEDED"
                )
        reference_failures.extend(reference_quality_failures)
        reference_passed = (
            not reference_quality_failures
            and reference_window_stats.position_residual_m
            <= reference.position_tolerance_m
            and reference_window_stats.orientation_residual_deg
            <= reference.orientation_tolerance_deg
        )
        if reference_quality_failures:
            reference_state = "ENDPOINT_HOLD_REQUIREMENTS_NOT_MET"
        elif reference_passed:
            reference_state = (
                "ROBUST_RETURN_RESIDUAL_WITHIN_RECORDED_"
                "PLACEMENT_TOLERANCE"
            )
        else:
            reference_state = (
                "ROBUST_RETURN_RESIDUAL_EXCEEDS_RECORDED_"
                "PLACEMENT_TOLERANCE"
            )
            reference_failures.append(
                "CLOSED_LOOP_RETURN_OUTSIDE_TOLERANCE"
            )

    reference_failures = list(dict.fromkeys(reference_failures))
    gate_failures = list(
        dict.fromkeys(continuity_gate_failures + reference_failures)
    )
    continuity_gate_passed = not continuity_gate_failures
    evaluation_passed = (
        continuity_gate_passed
        and (reference is None or reference_passed)
    )
    if continuity_gate_passed and reference_passed:
        state = (
            "LIVE_GATE_PASS_CLOSED_LOOP_REFERENCE_CONSISTENT_"
            "NOT_ACCURACY_VALIDATED"
        )
    elif continuity_gate_passed and reference is not None:
        state = "LIVE_GATE_PASS_CLOSED_LOOP_REFERENCE_FAILED"
    elif continuity_gate_passed:
        state = "LIVE_GATE_PASS_CONTINUITY_NOT_ACCURACY_VALIDATED"
    else:
        state = "LIVE_GATE_FAILED"

    artifact_paths = [
        run_dir / "run_metadata.yaml",
        summary_path,
        run_dir / "resolved_orbslam3_settings.yaml",
        run_dir / "resolved_estimator_config.yaml",
        run_dir / "requested_stream_config.yaml",
        run_dir / "resolved_stream_config.yaml",
        run_dir / "device_report.yaml",
        tracking_path,
        imu_path,
    ]
    if executable_binding_state == "CAPTURE_TIME_ATTESTED":
        artifact_paths.extend(
            (
                run_dir / "launch_provenance.yaml",
                run_dir / "source_live_manifest.yaml",
            )
        )
    for path in artifact_paths:
        require_file(path, path.name, allow_empty=False)
    # A structurally valid run can fail before ORB produces its first visual
    # pose (for example, a stationary rig that never initializes). Parsing and
    # summary cross-checks above prove that an empty diagnostic trajectory is
    # coherent; only a run claiming visual poses requires nonempty content.
    require_file(
        visual_path,
        visual_path.name,
        allow_empty=summary_int(summary, "visual_pose_count") == 0,
    )
    artifact_paths.append(visual_path)

    canonical_rows = 0 if accepted is None else accepted.rows
    canonical_duration = 0.0 if accepted is None else accepted.duration_s
    canonical_path_length = (
        0.0 if accepted is None else accepted.path_length_m
    )
    canonical_endpoint = (
        0.0 if accepted is None else accepted.endpoint_displacement_m
    )
    canonical_rotation = (
        0.0 if accepted is None else accepted.endpoint_rotation_deg
    )
    maximum_adjacent = (
        0.0 if accepted is None
        else accepted.maximum_adjacent_translation_m
    )
    maximum_speed = (
        0.0 if accepted is None else accepted.maximum_adjacent_speed_m_s
    )
    box = (
        (0.0, 0.0, 0.0)
        if accepted is None
        else (
            accepted.bounding_box_x_m,
            accepted.bounding_box_y_m,
            accepted.bounding_box_z_m,
        )
    )
    quaternion_error = (
        0.0 if accepted is None
        else accepted.maximum_quaternion_norm_error
    )
    reference_start = (
        None if reference_window_stats is None
        else reference_window_stats.start
    )
    reference_end = (
        None if reference_window_stats is None
        else reference_window_stats.end
    )
    maximum_path_excursion_m = (
        0.0
        if reference_window_stats is None or accepted is None
        else max(
            math.dist(
                pose[:3], reference_window_stats.start.position
            )
            for pose in accepted.poses
        )
    )
    result_lines = [
        "%YAML:1.0",
        'format: "ovrs-orbslam3-live-evaluation-v7"',
        f"state: {yaml_quote(state)}",
        f"live_gate_passed: {bool_yaml(evaluation_passed)}",
        "live_continuity_gate_passed: "
        f"{bool_yaml(continuity_gate_passed)}",
        "continuity_gate_failure_count: "
        f"{len(continuity_gate_failures)}",
        "continuity_gate_failures: "
        f"{yaml_quote(','.join(continuity_gate_failures))}",
        f"reference_gate_failure_count: {len(reference_failures)}",
        "reference_gate_failures: "
        f"{yaml_quote(','.join(reference_failures))}",
        "gate_failure_count: " + str(len(gate_failures)),
        "gate_failures: " + yaml_quote(",".join(gate_failures)),
        'integration: "PURE_ORB_SLAM3_STEREO_INERTIAL"',
        "openvins_pose_consumed: false",
        "global_correction_fed_to_openvins: false",
        'flight_control_integration_state: "NOT_IMPLEMENTED_OUT_OF_SCOPE"',
        'accuracy_evaluation_state: '
        '"NOT_EVALUATED_NO_INDEPENDENT_GROUND_TRUTH"',
        f"calibration_state: {yaml_quote(metadata['calibration_state'])}",
        f"camera_serial: {yaml_quote(metadata['camera_serial'])}",
        f"hardware_device_name: {yaml_quote(device.get('device_name', ''))}",
        f"hardware_firmware: {yaml_quote(device.get('firmware', ''))}",
        f"hardware_usb_type: {yaml_quote(device.get('usb_type', ''))}",
        f"startup_imu_gate_state: {yaml_quote(startup_state)}",
        f"startup_imu_gate_passed: {bool_yaml(startup_passed)}",
        f"startup_imu_gate_samples: {startup_samples}",
        f"startup_imu_gate_window_duration_s: {startup_window_duration:.9f}",
        "startup_imu_acceleration_magnitude_stddev_m_s2: "
        f"{startup_accel_stddev:.9f}",
        "startup_imu_maximum_gyro_magnitude_rad_s: "
        f"{startup_maximum_gyro:.9f}",
        f"startup_imu_gravity_error_m_s2: {startup_gravity_error:.9f}",
        "maximum_input_stall_seconds: "
        f"{maximum_input_stall_seconds:.9f}",
        "maximum_observed_stereo_wall_gap_seconds: "
        f"{maximum_stereo_wall_gap_seconds:.9f}",
        "maximum_observed_imu_wall_gap_seconds: "
        f"{maximum_imu_wall_gap_seconds:.9f}",
        f"input_stall_detected: {bool_yaml(input_stall_detected)}",
        f"submitted_stereo: {len(tracking.rows)}",
        f"visual_pose_count: {tracking.visual_pose_count}",
        f"lost_frame_count: {tracking.lost_frame_count}",
        "tracking_loss_after_acceptance_count: "
        f"{tracking.tracking_loss_after_acceptance_count}",
        "tracking_gap_after_acceptance_count: "
        f"{tracking.tracking_gap_after_acceptance_count}",
        f"minimum_tracked_map_points: {minimum_tracked_map_points}",
        "visual_support_failure_after_acceptance_count: "
        f"{tracking.visual_support_failure_after_acceptance_count}",
        "maximum_tracking_interval_seconds: "
        f"{maximum_tracking_interval_s:.9f}",
        "maximum_observed_tracking_interval_seconds: "
        f"{tracking.maximum_observed_tracking_interval_s:.9f}",
        f"tracking_latency_mean_ms: {statistics.fmean(tracking_latencies):.9f}",
        f"tracking_latency_maximum_ms: {max(tracking_latencies):.9f}",
        f"tracking_frame_budget_ms: {tracking_frame_budget_ms:.9f}",
        "tracking_latency_frame_budget_miss_count: "
        f"{tracking_latency_misses}",
        "tracking_latency_frame_budget_miss_ratio: "
        f"{tracking_latency_misses / len(tracking_latencies):.9f}",
        f"trajectory_candidate_rows: {len(tracking.accepted_timestamps_s)}",
        f"terminal_accepted_pose_count: "
        f"{summary_int(summary, 'accepted_pose_count')}",
        f"rejected_candidate_trajectory_rows: "
        f"{0 if candidate is None else candidate.rows}",
        f"canonical_trajectory_rows: {canonical_rows}",
        f"canonical_trajectory_duration_s: {canonical_duration:.9f}",
        f"canonical_path_length_m: {canonical_path_length:.9f}",
        f"canonical_endpoint_displacement_m: {canonical_endpoint:.9f}",
        f"canonical_endpoint_rotation_deg: {canonical_rotation:.9f}",
        f"canonical_maximum_adjacent_translation_m: {maximum_adjacent:.9f}",
        f"canonical_maximum_adjacent_speed_m_s: {maximum_speed:.9f}",
        f"canonical_bounding_box_x_m: {box[0]:.9f}",
        f"canonical_bounding_box_y_m: {box[1]:.9f}",
        f"canonical_bounding_box_z_m: {box[2]:.9f}",
        f"canonical_maximum_quaternion_norm_error: {quaternion_error:.9g}",
        f"ever_inertial_initialized: "
        f"{bool_yaml(tracking.ever_inertial_initialized)}",
        f"ever_inertial_ba2_finished: "
        f"{bool_yaml(tracking.ever_inertial_ba2_finished)}",
        f"active_map_reset_count: {tracking.rows[-1].reset_count}",
        "maximum_preacceptance_map_resets: "
        f"{maximum_preacceptance_map_resets}",
        "preacceptance_map_reset_count: "
        f"{tracking.preacceptance_map_reset_count}",
        "postacceptance_map_reset_count: "
        f"{tracking.postacceptance_map_reset_count}",
        "preacceptance_reset_limit_exceeded: "
        f"{bool_yaml(tracking.preacceptance_map_reset_count > maximum_preacceptance_map_resets)}",
        f"active_map_change_index: {tracking.rows[-1].map_change_index}",
        f"pending_reset_observed: "
        f"{bool_yaml(tracking.pending_reset_observed)}",
        "pending_reset_after_acceptance_observed: "
        f"{bool_yaml(tracking.pending_reset_after_acceptance_observed)}",
        f"inertial_regression_count: {tracking.inertial_regressions}",
        f"inertial_ba2_regression_count: {tracking.ba2_regressions}",
        f"trajectory_acceptance_discontinuity_count: "
        f"{tracking.acceptance_discontinuities}",
        f"map_change_after_acceptance: "
        f"{bool_yaml(tracking.map_change_after_acceptance)}",
        f"continuity_envelope_state: "
        f"{yaml_quote(continuity_envelope_state)}",
        "maximum_adjacent_translation_limit_m: "
        + (
            '""'
            if maximum_adjacent_limit is None
            else f"{maximum_adjacent_limit:.9g}"
        ),
        f"independent_closed_loop_reference_present: "
        f"{bool_yaml(reference is not None)}",
        "closed_loop_reference_format: "
        + (
            '""'
            if reference is None
            else '"ovrs-closed-loop-reference-v2"'
        ),
        "closed_loop_reference_start_policy: "
        + yaml_quote(
            metadata.get(
                "closed_loop_reference_start_policy",
                "LEGACY_RUN_WITHOUT_OPERATOR_CUE_CONTRACT",
            )
        ),
        f"closed_loop_reference_evaluation_state: "
        f"{yaml_quote(reference_state)}",
        f"closed_loop_reference_passed: {bool_yaml(reference_passed)}",
        "closed_loop_reference_method: "
        + yaml_quote("" if reference is None else reference.method),
        "closed_loop_position_tolerance_m: "
        + (
            '""'
            if reference is None
            else f"{reference.position_tolerance_m:.9g}"
        ),
        "closed_loop_orientation_tolerance_deg: "
        + (
            '""'
            if reference is None
            else f"{reference.orientation_tolerance_deg:.9g}"
        ),
        "closed_loop_endpoint_window_seconds: "
        + (
            '""'
            if reference is None
            else f"{reference.endpoint_window_seconds:.9g}"
        ),
        "closed_loop_minimum_endpoint_samples: "
        + (
            '""'
            if reference is None
            else str(reference.minimum_endpoint_samples)
        ),
        "closed_loop_maximum_endpoint_position_spread_m: "
        + (
            '""'
            if reference is None
            else f"{reference.maximum_endpoint_position_spread_m:.9g}"
        ),
        "closed_loop_maximum_endpoint_orientation_spread_deg: "
        + (
            '""'
            if reference is None
            else (
                f"{reference.maximum_endpoint_orientation_spread_deg:.9g}"
            )
        ),
        "closed_loop_minimum_path_duration_seconds: "
        + (
            '""'
            if reference is None
            else f"{reference.minimum_path_duration_seconds:.9g}"
        ),
        "closed_loop_minimum_path_excursion_m: "
        + (
            '""'
            if reference is None
            else f"{reference.minimum_path_excursion_m:.9g}"
        ),
        "closed_loop_maximum_estimated_path_excursion_m: "
        f"{maximum_path_excursion_m:.9f}",
        "closed_loop_start_window_samples: "
        f"{0 if reference_start is None else reference_start.samples}",
        "closed_loop_start_window_duration_s: "
        f"{0.0 if reference_start is None else reference_start.duration_s:.9f}",
        "closed_loop_start_maximum_position_spread_m: "
        + (
            "0.000000000"
            if reference_start is None
            else f"{reference_start.maximum_position_spread_m:.9f}"
        ),
        "closed_loop_start_maximum_orientation_spread_deg: "
        + (
            "0.000000000"
            if reference_start is None
            else f"{reference_start.maximum_orientation_spread_deg:.9f}"
        ),
        "closed_loop_end_window_samples: "
        f"{0 if reference_end is None else reference_end.samples}",
        "closed_loop_end_window_duration_s: "
        f"{0.0 if reference_end is None else reference_end.duration_s:.9f}",
        "closed_loop_end_maximum_position_spread_m: "
        + (
            "0.000000000"
            if reference_end is None
            else f"{reference_end.maximum_position_spread_m:.9f}"
        ),
        "closed_loop_end_maximum_orientation_spread_deg: "
        + (
            "0.000000000"
            if reference_end is None
            else f"{reference_end.maximum_orientation_spread_deg:.9f}"
        ),
        "closed_loop_robust_endpoint_displacement_m: "
        + (
            "0.000000000"
            if reference_window_stats is None
            else f"{reference_window_stats.position_residual_m:.9f}"
        ),
        "closed_loop_robust_endpoint_rotation_deg: "
        + (
            "0.000000000"
            if reference_window_stats is None
            else f"{reference_window_stats.orientation_residual_deg:.9f}"
        ),
        f"backend_commit: {yaml_quote(pin['commit'])}",
        f"backend_patch_sha256: "
        f"{yaml_quote(sha256_file(args.backend_patch.resolve()))}",
        f"backend_library_sha256: "
        f"{yaml_quote(sha256_file(args.backend_library.resolve()))}",
        f"live_executable_sha256: "
        f"{yaml_quote(sha256_file(args.live_executable.resolve()))}",
        f"live_executable_binding_state: "
        f"{yaml_quote(executable_binding_state)}",
        f"vocabulary_sha256: "
        f"{yaml_quote(sha256_file(args.vocabulary.resolve()))}",
        f"live_bundle_manifest_sha256: "
        f"{yaml_quote(sha256_file(args.live_bundle_manifest.resolve()))}",
        f"backend_pin_sha256: "
        f"{yaml_quote(sha256_file(args.backend_pin.resolve()))}",
        f"evaluator_source_sha256: "
        f"{yaml_quote(sha256_file(Path(__file__).resolve()))}",
    ]
    for path in artifact_paths:
        result_lines.append(
            f"{path.stem.replace('.', '_')}_sha256: "
            f"{yaml_quote(sha256_file(path))}"
        )
    if accepted_path.exists():
        result_lines.append(
            "accepted_trajectory_sha256: "
            f"{yaml_quote(sha256_file(accepted_path))}"
        )
    if candidate_path.exists():
        result_lines.append(
            "rejected_candidate_trajectory_sha256: "
            f"{yaml_quote(sha256_file(candidate_path))}"
        )
    if args.closed_loop_reference is not None:
        result_lines.append(
            "closed_loop_reference_sha256: "
            f"{yaml_quote(sha256_file(args.closed_loop_reference.resolve()))}"
        )

    temporary = output.with_name(f".{output.name}.tmp-{os.getpid()}")
    try:
        temporary.write_text("\n".join(result_lines) + "\n", encoding="utf-8")
        os.replace(temporary, output)
    except OSError as exc:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        raise BenchmarkError(f"cannot write evaluation manifest: {exc}") from exc
    print(f"ORB-SLAM3 live evaluation: {state}")
    print(f"Manifest: {output}")


def parser() -> argparse.ArgumentParser:
    root = Path(__file__).resolve().parents[1]
    result = argparse.ArgumentParser(
        description=(
            "Independently recompute the pure ORB-SLAM3 live continuity gate. "
            "This does not connect ORB poses or corrections to OpenVINS/EKF."
        )
    )
    result.add_argument("--run-dir", required=True, type=Path)
    result.add_argument(
        "--live-bundle-manifest", required=True, type=Path
    )
    result.add_argument(
        "--backend-pin",
        type=Path,
        default=root / "config" / "research" / "orbslam3_backend.yaml",
    )
    result.add_argument(
        "--backend-patch",
        type=Path,
        default=root / "patches" / "orbslam3-atlas-serialization-integrity.patch",
    )
    result.add_argument("--backend-library", required=True, type=Path)
    result.add_argument("--live-executable", required=True, type=Path)
    result.add_argument("--vocabulary", required=True, type=Path)
    result.add_argument(
        "--closed-loop-reference",
        type=Path,
        help=(
            "Optional independently recorded v2 colocated start/end "
            "endpoint-window reference; it is never consumed by the "
            "estimator."
        ),
    )
    result.add_argument(
        "--maximum-adjacent-translation-m",
        type=float,
        help=(
            "Optional caller-supplied operational continuity limit. No "
            "project-wide motion envelope is assumed by default."
        ),
    )
    result.add_argument("--output", type=Path)
    return result


def main() -> int:
    args = parser().parse_args()
    try:
        evaluate(args)
    except BenchmarkError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 4
    return 0


if __name__ == "__main__":
    sys.exit(main())
