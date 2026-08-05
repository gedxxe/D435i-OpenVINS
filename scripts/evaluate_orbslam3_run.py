#!/usr/bin/env python3
"""Create a fail-closed result manifest for one pinned ORB-SLAM3 run."""

from __future__ import annotations

import argparse
import math
import re
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from export_vislam_benchmark import (
    BenchmarkError,
    sha256_file,
    simple_yaml_map,
    yaml_quote,
)


@dataclass(frozen=True)
class TrajectoryStats:
    rows: int
    first_timestamp_ns: float
    last_timestamp_ns: float
    duration_s: float
    endpoint_displacement_m: float
    endpoint_rotation_deg: float
    maximum_displacement_m: float
    path_length_m: float


def parse_nonnegative_int(value: str, field: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise BenchmarkError(f"{field} is not an integer") from exc
    if parsed < 0 or str(parsed) != value.strip():
        raise BenchmarkError(f"{field} must be a nonnegative integer")
    return parsed


def parse_positive_float(value: str, field: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise BenchmarkError(f"{field} is not numeric") from exc
    if not math.isfinite(parsed) or parsed <= 0:
        raise BenchmarkError(f"{field} must be finite and positive")
    return parsed


def quaternion_return_angle_deg(
    first: tuple[float, ...], last: tuple[float, ...]
) -> float:
    dot = abs(sum(left * right for left, right in zip(first, last)))
    return math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot))))


def parse_trajectory(
    path: Path, label: str, require_increasing_timestamps: bool = True
) -> TrajectoryStats:
    if not path.is_file():
        raise BenchmarkError(f"{label} trajectory does not exist: {path}")
    rows = 0
    first_timestamp = 0.0
    previous_timestamp = -math.inf
    first_position: tuple[float, ...] = ()
    previous_position: tuple[float, ...] = ()
    first_quaternion: tuple[float, ...] = ()
    last_position: tuple[float, ...] = ()
    last_quaternion: tuple[float, ...] = ()
    path_length = 0.0
    maximum_displacement = 0.0
    minimum_timestamp = math.inf
    maximum_timestamp = -math.inf
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
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
            position = values[1:4]
            quaternion = values[4:8]
            if (
                require_increasing_timestamps
                and timestamp <= previous_timestamp
            ):
                raise BenchmarkError(
                    f"{path}:{line_number}: timestamps are not increasing"
                )
            minimum_timestamp = min(minimum_timestamp, timestamp)
            maximum_timestamp = max(maximum_timestamp, timestamp)
            quaternion_norm = math.sqrt(
                sum(value * value for value in quaternion)
            )
            if abs(quaternion_norm - 1.0) > 1e-3:
                raise BenchmarkError(
                    f"{path}:{line_number}: quaternion is not normalized"
                )
            if rows == 0:
                first_timestamp = timestamp
                first_position = position
                first_quaternion = quaternion
            else:
                path_length += math.dist(previous_position, position)
            maximum_displacement = max(
                maximum_displacement, math.dist(first_position, position)
            )
            previous_timestamp = timestamp
            previous_position = position
            last_position = position
            last_quaternion = quaternion
            rows += 1
    if rows < 2:
        raise BenchmarkError(f"{path}: at least two trajectory rows required")
    return TrajectoryStats(
        rows=rows,
        first_timestamp_ns=first_timestamp,
        last_timestamp_ns=previous_timestamp,
        duration_s=(maximum_timestamp - minimum_timestamp) / 1e9,
        endpoint_displacement_m=math.dist(first_position, last_position),
        endpoint_rotation_deg=quaternion_return_angle_deg(
            first_quaternion, last_quaternion
        ),
        maximum_displacement_m=maximum_displacement,
        path_length_m=path_length,
    )


def count_occurrences(text: str, pattern: str) -> int:
    return text.count(pattern)


def last_integer_match(text: str, pattern: str, field: str) -> int:
    matches = re.findall(pattern, text)
    if not matches:
        raise BenchmarkError(f"backend log lacks {field}")
    return int(matches[-1])


def adapter_child(adapter_manifest: Path, value: str, field: str) -> Path:
    relative = Path(value)
    if (
        not value
        or relative.is_absolute()
        or len(relative.parts) != 1
        or relative.name != value
    ):
        raise BenchmarkError(f"adapter {field} must be a plain filename")
    return adapter_manifest.parent / relative


def backend_library_binding_state(runner: Path, library: Path) -> str:
    with runner.open("rb") as handle:
        if handle.read(4) != b"\x7fELF":
            return "NON_ELF_RUNNER_NOT_LINK_ATTESTED"
    completed = subprocess.run(
        ("ldd", str(runner)),
        check=False,
        text=True,
        capture_output=True,
    )
    if completed.returncode != 0:
        raise BenchmarkError(
            "could not resolve backend runner shared-library dependencies"
        )
    matches = re.findall(
        r"^\s*libORB_SLAM3\.so\s*=>\s*(\S+)",
        completed.stdout,
        flags=re.MULTILINE,
    )
    if len(matches) != 1 or matches[0] == "not":
        raise BenchmarkError(
            "backend runner does not resolve exactly one libORB_SLAM3.so"
        )
    resolved = Path(matches[0]).resolve()
    if resolved != library.resolve():
        raise BenchmarkError(
            "backend runner resolves a different libORB_SLAM3.so: "
            f"{resolved}"
        )
    return "ELF_LDD_MATCHED_REQUESTED_LIBRARY"


def validate_input_atlas_manifest(
    adapter_manifest: Path,
    adapter: dict[str, str],
    atlas_input: Path,
    backend_runner: Path,
    backend_library: Path,
    vocabulary: Path,
) -> Path:
    manifest_path = adapter_child(
        adapter_manifest,
        adapter.get("atlas_input_manifest_file", ""),
        "atlas_input_manifest_file",
    )
    if not manifest_path.is_file():
        raise BenchmarkError("staged input atlas manifest is missing")
    if sha256_file(manifest_path) != adapter.get(
        "atlas_input_manifest_sha256", ""
    ):
        raise BenchmarkError("staged input atlas manifest hash differs")
    manifest = simple_yaml_map(manifest_path)
    expected = {
        "format": "ovrs-orbslam3-atlas-manifest-v1",
        "state": "TRACKING_GATE_PASS_ATLAS_RELOAD_UNVERIFIED",
        "atlas_sha256": sha256_file(atlas_input),
        "backend_name": "ORB_SLAM3",
        "backend_commit": adapter.get("backend_commit", ""),
        "backend_patch_sha256": adapter.get("backend_patch_sha256", ""),
        "backend_pin_sha256": adapter.get("backend_pin_sha256", ""),
        "camera_serial": adapter.get("camera_serial", ""),
        "calibration_state": adapter.get("calibration_state", ""),
        "imucam_config_sha256": adapter.get("imucam_config_sha256", ""),
        "imu_config_sha256": adapter.get("imu_config_sha256", ""),
        "source_camera_fps": adapter.get("source_camera_fps", ""),
        "adapted_camera_fps": adapter.get("adapted_camera_fps", ""),
        "camera_stride": adapter.get("camera_stride", ""),
        "camera_time_offset_policy": adapter.get(
            "camera_time_offset_policy", ""
        ),
        "calibrated_camera_imu_time_offset_ns": adapter.get(
            "calibrated_camera_imu_time_offset_ns", ""
        ),
        "applied_camera_imu_time_offset_ns": adapter.get(
            "applied_camera_imu_time_offset_ns", ""
        ),
        "backend_runner_sha256": sha256_file(backend_runner),
        "backend_library_sha256": sha256_file(backend_library),
        "vocabulary_sha256": sha256_file(vocabulary),
        "coordinate_frame_policy": "ORB_SLAM3_ATLAS_WORLD",
        "ground_truth_consumed_by_estimator": "false",
    }
    for key, value in expected.items():
        if not value or manifest.get(key) != value:
            raise BenchmarkError(
                f"input atlas manifest {key} differs from the current run"
            )
    return manifest_path


def parse_reference(path: Path | None) -> dict[str, str]:
    if path is None:
        return {}
    reference = simple_yaml_map(path)
    expected = {
        "format": "ovrs-closed-loop-reference-v1",
        "reference_type": "COLOCATED_START_END",
        "estimator_consumed_reference": "false",
        "physical_path_completed": "true",
    }
    for key, value in expected.items():
        if reference.get(key) != value:
            raise BenchmarkError(f"closed-loop reference {key} must be {value}")
    method = reference.get("method", "")
    if not method:
        raise BenchmarkError("closed-loop reference method is missing")
    position_tolerance = parse_positive_float(
        reference.get("position_tolerance_m", ""),
        "closed-loop reference position_tolerance_m",
    )
    orientation_tolerance = parse_positive_float(
        reference.get("orientation_tolerance_deg", ""),
        "closed-loop reference orientation_tolerance_deg",
    )
    if orientation_tolerance > 180.0:
        raise BenchmarkError(
            "closed-loop reference orientation_tolerance_deg exceeds 180"
        )
    return {
        "method": method,
        "position_tolerance_m": f"{position_tolerance:.9g}",
        "orientation_tolerance_deg": f"{orientation_tolerance:.9g}",
    }


def evaluate(args: argparse.Namespace) -> None:
    output = args.output.resolve()
    if output.exists():
        raise BenchmarkError(f"output already exists: {output}")
    for path in (
        args.adapter_manifest,
        args.settings,
        args.backend_log,
        args.frame_trajectory,
        args.keyframe_trajectory,
        args.backend_runner,
        args.backend_library,
        args.vocabulary,
    ):
        if not path.is_file():
            raise BenchmarkError(f"required input does not exist: {path}")

    adapter = simple_yaml_map(args.adapter_manifest)
    expected_adapter = {
        "format": "ovrs-orbslam3-adapter-v1",
        "state": "PREPARED_NOT_RUN",
        "backend_name": "ORB_SLAM3",
        "ground_truth_consumed_by_estimator": "false",
    }
    for key, value in expected_adapter.items():
        if adapter.get(key) != value:
            raise BenchmarkError(f"adapter manifest {key} must be {value}")
    expected_settings_hash = adapter.get("settings_sha256", "")
    if (
        expected_settings_hash
        and sha256_file(args.settings) != expected_settings_hash
    ):
        raise BenchmarkError("ORB-SLAM3 settings differ from adapter manifest")
    atlas_mode = adapter.get("atlas_mode", "NONE")
    if atlas_mode not in ("NONE", "MAP_BUILD", "MULTI_SESSION_MERGE"):
        raise BenchmarkError(f"unsupported adapter atlas_mode: {atlas_mode}")
    atlas_input: Path | None = None
    atlas_input_manifest: Path | None = None
    atlas_output: Path | None = None
    if atlas_mode == "MULTI_SESSION_MERGE":
        atlas_input = adapter_child(
            args.adapter_manifest,
            adapter.get("atlas_input_file", ""),
            "atlas_input_file",
        )
        if not atlas_input.is_file() or atlas_input.stat().st_size == 0:
            raise BenchmarkError("staged input atlas is missing or empty")
        if sha256_file(atlas_input) != adapter.get("atlas_input_sha256", ""):
            raise BenchmarkError("staged input atlas hash differs")
        atlas_input_manifest = validate_input_atlas_manifest(
            args.adapter_manifest,
            adapter,
            atlas_input,
            args.backend_runner,
            args.backend_library,
            args.vocabulary,
        )
    if atlas_mode in ("MAP_BUILD", "MULTI_SESSION_MERGE"):
        atlas_output = adapter_child(
            args.adapter_manifest,
            adapter.get("atlas_output_file", ""),
            "atlas_output_file",
        )
    adapted_pairs = parse_nonnegative_int(
        adapter.get("adapted_stereo_pairs", ""),
        "adapter adapted_stereo_pairs",
    )
    adapted_fps = parse_nonnegative_int(
        adapter.get("adapted_camera_fps", ""),
        "adapter adapted_camera_fps",
    )
    first_input_ns = parse_nonnegative_int(
        adapter.get("first_adapted_stereo_timestamp_ns", ""),
        "adapter first_adapted_stereo_timestamp_ns",
    )
    last_input_ns = parse_nonnegative_int(
        adapter.get("last_adapted_stereo_timestamp_ns", ""),
        "adapter last_adapted_stereo_timestamp_ns",
    )
    if adapted_pairs < 2 or adapted_fps <= 0 or last_input_ns <= first_input_ns:
        raise BenchmarkError("adapter camera range is invalid")

    log_text = args.backend_log.read_text(encoding="utf-8")
    if not log_text.strip():
        raise BenchmarkError("backend log is empty")
    frame = parse_trajectory(args.frame_trajectory, "frame")
    # A merged atlas writes KeyFrames in map/KeyFrame-ID order. Timestamp
    # ranges can therefore restart at a later session boundary even though
    # every active-session frame remains strictly ordered.
    keyframe = parse_trajectory(
        args.keyframe_trajectory,
        "keyframe",
        require_increasing_timestamps=(atlas_mode != "MULTI_SESSION_MERGE"),
    )
    if frame.rows > adapted_pairs:
        raise BenchmarkError("frame trajectory exceeds adapted stereo count")
    if atlas_mode != "MULTI_SESSION_MERGE" and keyframe.rows > frame.rows:
        raise BenchmarkError("keyframe trajectory exceeds frame trajectory")
    timestamp_tolerance_ns = 2e9 / adapted_fps
    if (
        frame.first_timestamp_ns < first_input_ns - timestamp_tolerance_ns
        or frame.last_timestamp_ns > last_input_ns + timestamp_tolerance_ns
    ):
        raise BenchmarkError("frame trajectory lies outside adapted time range")

    resets = count_occurrences(
        log_text, "Reset map because local mapper set the bad imu flag"
    )
    tracking_failures = count_occurrences(
        log_text, "Fail to track local map"
    )
    viba1_completed = count_occurrences(log_text, "end VIBA 1")
    viba2_completed = count_occurrences(log_text, "end VIBA 2")
    loop_candidates = count_occurrences(log_text, "*Loop detected")
    rejected_loops = count_occurrences(log_text, "BAD LOOP!!!")
    if rejected_loops > loop_candidates:
        raise BenchmarkError("backend log rejects more loops than it detects")
    applied_loop_corrections = loop_candidates - rejected_loops
    merge_detections = count_occurrences(log_text, "*Merge detected")
    merge_completions = count_occurrences(log_text, "Merge finished!")
    if merge_completions > merge_detections:
        raise BenchmarkError("backend log completes more merges than it detects")
    backend_binding_state = backend_library_binding_state(
        args.backend_runner, args.backend_library
    )
    created_maps = count_occurrences(log_text, "New Map created")
    atlas_maps = last_integer_match(
        log_text, r"There are ([0-9]+) maps in the atlas", "atlas map count"
    )
    map_keyframe_counts = [
        int(value)
        for value in re.findall(
            r"Map [0-9]+ has ([0-9]+) KFs", log_text
        )
    ]
    if not map_keyframe_counts:
        raise BenchmarkError("backend log lacks final map keyframe count")
    final_map_keyframes = max(map_keyframe_counts)
    if final_map_keyframes != keyframe.rows:
        raise BenchmarkError(
            "backend log and keyframe trajectory row counts differ"
        )

    backend_completed = (
        args.backend_exit_status == 0
        and "Shutdown" in log_text
        and "End of saving trajectory" in log_text
        and "Saving keyframe trajectory" in log_text
    )
    terminal_coverage = (
        last_input_ns - frame.last_timestamp_ns <= timestamp_tolerance_ns
    )
    atlas_load_completed = atlas_mode != "MULTI_SESSION_MERGE" or (
        "Initialization of Atlas from file: input_atlas" in log_text
        and "End to load the save binary file" in log_text
    )
    atlas_save_completed = atlas_mode == "NONE" or (
        atlas_output is not None
        and atlas_output.is_file()
        and atlas_output.stat().st_size > 0
        and "End to write save binary file" in log_text
    )
    atlas_merge_established = (
        atlas_mode != "MULTI_SESSION_MERGE"
        or (
            merge_detections > 0
            and merge_completions == merge_detections
            and atlas_maps == 1
        )
    )
    tracking_gate_passed = (
        backend_completed
        and resets == 0
        and tracking_failures == 0
        and viba2_completed > 0
        and atlas_maps == 1
        and terminal_coverage
        and atlas_load_completed
        and atlas_save_completed
        and atlas_merge_established
    )
    if not backend_completed:
        state = "PROCESS_FAILED"
    elif atlas_mode == "MULTI_SESSION_MERGE" and not atlas_merge_established:
        state = "ATLAS_MERGE_NOT_ESTABLISHED"
    elif not tracking_gate_passed:
        state = "TRACKING_GATE_FAILED"
    elif applied_loop_corrections > 0:
        state = "TRACKING_PASS_LOOP_CORRECTION_NOT_REFERENCE_VALIDATED"
    else:
        state = "TRACKING_PASS_NO_LOOP_CORRECTION"

    reference = parse_reference(args.closed_loop_reference)
    if reference:
        position_consistent = frame.endpoint_displacement_m <= float(
            reference["position_tolerance_m"]
        )
        orientation_consistent = frame.endpoint_rotation_deg <= float(
            reference["orientation_tolerance_deg"]
        )
        if position_consistent and orientation_consistent:
            reference_state = (
                "RETURN_RESIDUAL_WITHIN_RECORDED_PLACEMENT_TOLERANCE"
            )
        else:
            reference_state = (
                "RETURN_RESIDUAL_EXCEEDS_RECORDED_PLACEMENT_TOLERANCE"
            )
    else:
        position_consistent = False
        orientation_consistent = False
        reference_state = "NOT_EVALUATED"

    output.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "%YAML:1.0",
        'format: "ovrs-orbslam3-result-v1"',
        f"state: {yaml_quote(state)}",
        (
            "created_utc: "
            f"{yaml_quote(datetime.now(timezone.utc).isoformat())}"
        ),
        f"backend_exit_status: {args.backend_exit_status}",
        f"backend_exit_status_source: {yaml_quote(args.exit_status_source)}",
        f"backend_completed: {str(backend_completed).lower()}",
        f"tracking_gate_passed: {str(tracking_gate_passed).lower()}",
        f"terminal_input_coverage: {str(terminal_coverage).lower()}",
        f"imu_map_resets: {resets}",
        f"local_map_tracking_failures: {tracking_failures}",
        f"created_maps: {created_maps}",
        f"final_atlas_maps: {atlas_maps}",
        f"atlas_mode: {yaml_quote(atlas_mode)}",
        f"atlas_load_completed: {str(atlas_load_completed).lower()}",
        f"atlas_save_completed: {str(atlas_save_completed).lower()}",
        (
            "atlas_merge_established: "
            f"{str(atlas_merge_established).lower()}"
        ),
        f"map_merge_detections: {merge_detections}",
        f"map_merge_completions: {merge_completions}",
        f"viba1_completions: {viba1_completed}",
        f"viba2_completions: {viba2_completed}",
        f"loop_candidates: {loop_candidates}",
        f"rejected_loop_candidates: {rejected_loops}",
        f"applied_loop_corrections: {applied_loop_corrections}",
        (
            'loop_count_policy: "PINNED_ORB_CANDIDATES_MINUS_BAD_LOOP_REJECTIONS"'
        ),
        'false_loop_evaluation_state: "NOT_EVALUATED_WITHOUT_REFERENCE"',
        'false_map_merge_evaluation_state: "NOT_EVALUATED_WITHOUT_REFERENCE"',
        f"adapted_stereo_pairs: {adapted_pairs}",
        f"frame_trajectory_rows: {frame.rows}",
        f"keyframe_trajectory_rows: {keyframe.rows}",
        (
            'keyframe_timestamp_order_policy: "STRICTLY_INCREASING"'
            if atlas_mode != "MULTI_SESSION_MERGE"
            else
            'keyframe_timestamp_order_policy: "MULTI_SESSION_MAP_ID_ORDER"'
        ),
        f"initialization_prefix_frames: {adapted_pairs - frame.rows}",
        f"trajectory_coverage_ratio: {frame.rows / adapted_pairs:.9g}",
        f"trajectory_duration_s: {frame.duration_s:.9g}",
        (
            "estimated_return_displacement_m: "
            f"{frame.endpoint_displacement_m:.9g}"
        ),
        f"estimated_return_rotation_deg: {frame.endpoint_rotation_deg:.9g}",
        f"estimated_maximum_displacement_m: {frame.maximum_displacement_m:.9g}",
        f"estimated_path_length_m: {frame.path_length_m:.9g}",
        f"independent_reference_present: {str(bool(reference)).lower()}",
        "ground_truth_consumed_by_estimator: false",
        f"backend_library_binding_state: {yaml_quote(backend_binding_state)}",
        f"reference_evaluation_state: {yaml_quote(reference_state)}",
    ]
    if reference:
        lines.extend(
            (
                f"reference_method: {yaml_quote(reference['method'])}",
                (
                    "reference_position_tolerance_m: "
                    f"{reference['position_tolerance_m']}"
                ),
                (
                    "reference_orientation_tolerance_deg: "
                    f"{reference['orientation_tolerance_deg']}"
                ),
                (
                    "return_position_consistent_with_reference_tolerance: "
                    f"{str(position_consistent).lower()}"
                ),
                (
                    "return_orientation_consistent_with_reference_tolerance: "
                    f"{str(orientation_consistent).lower()}"
                ),
            )
        )
    if args.backend_exit_status_file is not None:
        lines.append(
            "backend_exit_status_file_sha256: "
            f"{yaml_quote(sha256_file(args.backend_exit_status_file))}"
        )
    if atlas_input is not None:
        lines.append(
            f"atlas_input_sha256: {yaml_quote(sha256_file(atlas_input))}"
        )
        if atlas_input_manifest is None:
            raise BenchmarkError("input atlas manifest was not validated")
        lines.extend(
            (
                "atlas_input_manifest_sha256: "
                f"{yaml_quote(sha256_file(atlas_input_manifest))}",
                "parent_atlas_reload_verified_by_this_run: "
                f"{str(tracking_gate_passed).lower()}",
            )
        )
    if atlas_output is not None and atlas_output.is_file():
        lines.append(
            f"atlas_output_sha256: {yaml_quote(sha256_file(atlas_output))}"
        )
    lines.extend(
        (
            (
                "adapter_manifest_sha256: "
                f"{yaml_quote(sha256_file(args.adapter_manifest))}"
            ),
            f"settings_sha256: {yaml_quote(sha256_file(args.settings))}",
            f"backend_log_sha256: {yaml_quote(sha256_file(args.backend_log))}",
            (
                "frame_trajectory_sha256: "
                f"{yaml_quote(sha256_file(args.frame_trajectory))}"
            ),
            (
                "keyframe_trajectory_sha256: "
                f"{yaml_quote(sha256_file(args.keyframe_trajectory))}"
            ),
            (
                "backend_runner_sha256: "
                f"{yaml_quote(sha256_file(args.backend_runner))}"
            ),
            (
                "backend_library_sha256: "
                f"{yaml_quote(sha256_file(args.backend_library))}"
            ),
            f"vocabulary_sha256: {yaml_quote(sha256_file(args.vocabulary))}",
            "",
        )
    )
    output.write_text("\n".join(lines), encoding="utf-8")
    print(f"ORB-SLAM3 result manifest: {output}")
    print(
        f"State: {state} "
        f"(frames={frame.rows}/{adapted_pairs}, resets={resets}, "
        f"loop_corrections={applied_loop_corrections})"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Evaluate one pinned ORB-SLAM3 run without claiming accuracy "
            "when no independent reference is present."
        )
    )
    parser.add_argument("--adapter-manifest", required=True, type=Path)
    parser.add_argument("--settings", required=True, type=Path)
    parser.add_argument("--backend-log", required=True, type=Path)
    parser.add_argument("--frame-trajectory", required=True, type=Path)
    parser.add_argument("--keyframe-trajectory", required=True, type=Path)
    parser.add_argument("--backend-runner", required=True, type=Path)
    parser.add_argument("--backend-library", required=True, type=Path)
    parser.add_argument("--vocabulary", required=True, type=Path)
    status = parser.add_mutually_exclusive_group(required=True)
    status.add_argument("--backend-exit-status", type=int)
    status.add_argument("--backend-exit-status-file", type=Path)
    parser.add_argument("--closed-loop-reference", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.backend_exit_status_file is not None:
        try:
            status_text = args.backend_exit_status_file.read_text(
                encoding="utf-8"
            ).strip()
            args.backend_exit_status = parse_nonnegative_int(
                status_text, "backend exit-status file"
            )
        except (BenchmarkError, OSError) as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 4
        args.exit_status_source = "CAPTURED_FILE"
    else:
        args.exit_status_source = "CALLER_REPORTED"
    if args.backend_exit_status < 0:
        print("ERROR: --backend-exit-status must be nonnegative", file=sys.stderr)
        return 4
    try:
        evaluate(args)
    except (BenchmarkError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 4
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
