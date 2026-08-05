#!/usr/bin/env python3
"""Run and evaluate one prepared ORB-SLAM3 stereo-inertial benchmark."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

from export_vislam_benchmark import (
    BenchmarkError,
    sha256_file,
    simple_yaml_map,
    yaml_quote,
)
from evaluate_orbslam3_run import (
    backend_library_binding_state,
    validate_input_atlas_manifest,
)


def validate_run_id(value: str) -> str:
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}", value):
        raise argparse.ArgumentTypeError(
            "run id must be 1-64 safe filename characters"
        )
    return value


def adapter_child(adapter_dir: Path, value: str, field: str) -> Path:
    relative = Path(value)
    if (
        not value
        or relative.is_absolute()
        or len(relative.parts) != 1
        or relative.name != value
    ):
        raise BenchmarkError(f"adapter {field} must be a plain filename")
    return adapter_dir / relative


def write_atlas_manifest(
    path: Path,
    atlas: Path,
    adapter_manifest: Path,
    adapter: dict[str, str],
    result: Path,
    runner: Path,
    backend_library: Path,
    vocabulary: Path,
) -> None:
    copied_fields = (
        "atlas_mode",
        "camera_serial",
        "calibration_state",
        "repository_commit",
        "repository_worktree",
        "backend_name",
        "backend_commit",
        "backend_license",
        "source_camera_fps",
        "adapted_camera_fps",
        "camera_stride",
        "camera_time_offset_policy",
        "calibrated_camera_imu_time_offset_ns",
        "applied_camera_imu_time_offset_ns",
        "source_benchmark_manifest_sha256",
        "backend_pin_sha256",
        "backend_patch_sha256",
        "imucam_config_sha256",
        "imu_config_sha256",
        "settings_sha256",
    )
    lines = [
        "%YAML:1.0",
        'format: "ovrs-orbslam3-atlas-manifest-v1"',
        'state: "TRACKING_GATE_PASS_ATLAS_RELOAD_UNVERIFIED"',
        (
            "created_utc: "
            f"{yaml_quote(datetime.now(timezone.utc).isoformat())}"
        ),
        f"atlas_file: {yaml_quote(atlas.name)}",
        f"atlas_sha256: {yaml_quote(sha256_file(atlas))}",
    ]
    for field in copied_fields:
        if field in adapter:
            lines.append(f"{field}: {yaml_quote(adapter[field])}")
    if "atlas_input_sha256" in adapter:
        lines.append(
            f"parent_atlas_sha256: {yaml_quote(adapter['atlas_input_sha256'])}"
        )
    if "atlas_input_manifest_sha256" in adapter:
        lines.append(
            "parent_atlas_manifest_sha256: "
            f"{yaml_quote(adapter['atlas_input_manifest_sha256'])}"
        )
    lines.extend(
        (
            (
                "adapter_manifest_sha256: "
                f"{yaml_quote(sha256_file(adapter_manifest))}"
            ),
            f"result_manifest_sha256: {yaml_quote(sha256_file(result))}",
            f"backend_runner_sha256: {yaml_quote(sha256_file(runner))}",
            (
                "backend_library_sha256: "
                f"{yaml_quote(sha256_file(backend_library))}"
            ),
            f"vocabulary_sha256: {yaml_quote(sha256_file(vocabulary))}",
            'coordinate_frame_policy: "ORB_SLAM3_ATLAS_WORLD"',
            "ground_truth_consumed_by_estimator: false",
            "",
        )
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def run(args: argparse.Namespace) -> int:
    adapter_dir = args.adapter_dir.resolve()
    runner = args.runner.resolve()
    backend_library = args.backend_library.resolve()
    backend_pin = args.backend_pin.resolve()
    vocabulary = args.vocabulary.resolve()
    evaluator = Path(__file__).with_name("evaluate_orbslam3_run.py")
    if not adapter_dir.is_dir():
        raise BenchmarkError(f"adapter directory does not exist: {adapter_dir}")
    if not runner.is_file():
        raise BenchmarkError(f"runner does not exist: {runner}")
    if (
        not backend_library.is_file()
        or backend_library.name != "libORB_SLAM3.so"
    ):
        raise BenchmarkError(
            "backend library must name an existing libORB_SLAM3.so"
        )
    if not backend_pin.is_file():
        raise BenchmarkError(f"backend pin does not exist: {backend_pin}")
    if not vocabulary.is_file():
        raise BenchmarkError(f"vocabulary does not exist: {vocabulary}")

    adapter_manifest = adapter_dir / "adapter_manifest.yaml"
    adapter = simple_yaml_map(adapter_manifest)
    expected = {
        "format": "ovrs-orbslam3-adapter-v1",
        "state": "PREPARED_NOT_RUN",
        "backend_name": "ORB_SLAM3",
    }
    for key, value in expected.items():
        if adapter.get(key) != value:
            raise BenchmarkError(f"adapter manifest {key} must be {value}")
    if sha256_file(backend_pin) != adapter.get("backend_pin_sha256", ""):
        raise BenchmarkError("backend pin differs from adapter manifest")
    pin = simple_yaml_map(backend_pin)
    if (
        pin.get("commit") != adapter.get("backend_commit")
        or pin.get("patch_sha256") != adapter.get("backend_patch_sha256")
    ):
        raise BenchmarkError("backend source or patch pin differs from adapter")

    settings = adapter_dir / "orbslam3_settings.yaml"
    sequence = adapter_dir / "sequence"
    timestamps = adapter_dir / "timestamps.txt"
    for path in (settings, sequence, timestamps):
        if not path.exists():
            raise BenchmarkError(f"prepared adapter input is missing: {path}")
    expected_settings_hash = adapter.get("settings_sha256", "")
    if (
        expected_settings_hash
        and sha256_file(settings) != expected_settings_hash
    ):
        raise BenchmarkError("ORB-SLAM3 settings differ from adapter manifest")
    hashed_inputs = (
        (timestamps, "timestamps_sha256"),
        (
            sequence / "mav0" / "cam0" / "data.csv",
            "adapted_cam0_data_csv_sha256",
        ),
        (
            sequence / "mav0" / "cam1" / "data.csv",
            "adapted_cam1_data_csv_sha256",
        ),
        (
            sequence / "mav0" / "imu0" / "data.csv",
            "imu0_data_csv_sha256",
        ),
    )
    for path, field in hashed_inputs:
        expected_hash = adapter.get(field, "")
        if not path.is_file() or not expected_hash:
            raise BenchmarkError(f"prepared adapter input lacks {field}")
        if sha256_file(path) != expected_hash:
            raise BenchmarkError(f"prepared adapter input {field} differs")

    atlas_mode = adapter.get("atlas_mode", "NONE")
    if atlas_mode not in ("NONE", "MAP_BUILD", "MULTI_SESSION_MERGE"):
        raise BenchmarkError(f"unsupported adapter atlas_mode: {atlas_mode}")
    atlas_output: Path | None = None
    atlas_manifest: Path | None = None
    atlas_input: Path | None = None
    atlas_input_manifest: Path | None = None
    if atlas_mode == "MULTI_SESSION_MERGE":
        atlas_input = adapter_child(
            adapter_dir,
            adapter.get("atlas_input_file", ""),
            "atlas_input_file",
        )
        if not atlas_input.is_file() or atlas_input.stat().st_size == 0:
            raise BenchmarkError("staged input atlas is missing or empty")
        if sha256_file(atlas_input) != adapter.get("atlas_input_sha256", ""):
            raise BenchmarkError("staged input atlas hash differs")
        atlas_input_manifest = validate_input_atlas_manifest(
            adapter_manifest,
            adapter,
            atlas_input,
            runner,
            backend_library,
            vocabulary,
        )
    if atlas_mode in ("MAP_BUILD", "MULTI_SESSION_MERGE"):
        atlas_output = adapter_child(
            adapter_dir,
            adapter.get("atlas_output_file", ""),
            "atlas_output_file",
        )
        atlas_manifest = atlas_output.with_suffix(".osa.manifest.yaml")
        for path in (atlas_output, atlas_manifest):
            if path.exists():
                raise BenchmarkError(f"atlas output already exists: {path}")

    log = adapter_dir / f"backend_{args.run_id}.log"
    status = adapter_dir / f"backend_{args.run_id}.status"
    frame = adapter_dir / f"f_{args.run_id}.txt"
    keyframe = adapter_dir / f"kf_{args.run_id}.txt"
    result = adapter_dir / f"experiment_{args.run_id}.yaml"
    for path in (log, status, frame, keyframe, result):
        if path.exists():
            raise BenchmarkError(f"run output already exists: {path}")

    command = (
        str(runner),
        str(vocabulary),
        str(settings),
        str(sequence),
        str(timestamps),
        args.run_id,
    )
    backend_library_binding_state(runner, backend_library)
    immutable_paths = [
        adapter_manifest,
        settings,
        *(path for path, _ in hashed_inputs),
        runner,
        backend_library,
        vocabulary,
    ]
    if atlas_input is not None and atlas_input_manifest is not None:
        immutable_paths.extend((atlas_input, atlas_input_manifest))
    immutable_inputs = {path: sha256_file(path) for path in immutable_paths}
    with log.open("w", encoding="utf-8") as handle:
        completed = subprocess.run(
            command,
            cwd=adapter_dir,
            stdout=handle,
            stderr=subprocess.STDOUT,
            check=False,
        )
    captured_status = (
        completed.returncode
        if completed.returncode >= 0
        else 128 - completed.returncode
    )
    status.write_text(f"{captured_status}\n", encoding="utf-8")
    for path, expected_hash in immutable_inputs.items():
        if sha256_file(path) != expected_hash:
            raise BenchmarkError(f"run input changed during execution: {path}")

    evaluate_command = [
        sys.executable,
        str(evaluator),
        "--adapter-manifest",
        str(adapter_manifest),
        "--settings",
        str(settings),
        "--backend-log",
        str(log),
        "--frame-trajectory",
        str(frame),
        "--keyframe-trajectory",
        str(keyframe),
        "--backend-runner",
        str(runner),
        "--backend-library",
        str(backend_library),
        "--vocabulary",
        str(vocabulary),
        "--backend-exit-status-file",
        str(status),
        "--output",
        str(result),
    ]
    if args.closed_loop_reference is not None:
        evaluate_command.extend(
            (
                "--closed-loop-reference",
                str(args.closed_loop_reference.resolve()),
            )
        )
    evaluation = subprocess.run(evaluate_command, check=False)
    if evaluation.returncode != 0:
        return evaluation.returncode
    if captured_status != 0:
        return captured_status
    evaluated = simple_yaml_map(result)
    if evaluated.get("tracking_gate_passed") != "true":
        print(
            f"ERROR: backend completed but tracking gate failed: "
            f"{evaluated.get('state', 'UNKNOWN')}",
            file=sys.stderr,
        )
        return 5
    if atlas_output is not None:
        if (
            not atlas_output.is_file()
            or atlas_output.stat().st_size == 0
            or atlas_manifest is None
        ):
            raise BenchmarkError("accepted atlas output is missing or empty")
        write_atlas_manifest(
            atlas_manifest,
            atlas_output,
            adapter_manifest,
            adapter,
            result,
            runner,
            backend_library,
            vocabulary,
        )
        print(f"Provisional atlas manifest: {atlas_manifest}")
    print(f"ORB-SLAM3 benchmark run complete: {args.run_id}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description=(
            "Run the pinned ORB-SLAM3 EuRoC executable and create a hashed "
            "result manifest with an automatically captured exit status."
        )
    )
    parser.add_argument("--adapter-dir", required=True, type=Path)
    parser.add_argument("--runner", required=True, type=Path)
    parser.add_argument("--backend-library", required=True, type=Path)
    parser.add_argument(
        "--backend-pin",
        type=Path,
        default=root / "config" / "research" / "orbslam3_backend.yaml",
    )
    parser.add_argument("--vocabulary", required=True, type=Path)
    parser.add_argument("--run-id", required=True, type=validate_run_id)
    parser.add_argument("--closed-loop-reference", type=Path)
    return parser


def main() -> int:
    try:
        return run(build_parser().parse_args())
    except (BenchmarkError, OSError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 4


if __name__ == "__main__":
    raise SystemExit(main())
