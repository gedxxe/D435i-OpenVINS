#!/usr/bin/env python3
"""Read-only validation for an OVRS calibration capture directory."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from calibration_common import CalibrationError, validate_capture


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description=(
            "Validate capture metadata, integrity counters, CSV contents, "
            "row counts, timestamps, and referenced stereo PNG headers. "
            "The capture is not modified."
        )
    )
    result.add_argument("--capture", required=True, type=Path)
    return result


def main() -> int:
    args = parser().parse_args()
    try:
        print(f"Scanning capture read-only: {args.capture}", flush=True)
        info = validate_capture(args.capture)
    except CalibrationError as exc:
        print(f"Calibration capture validation failed: {exc}", file=sys.stderr)
        return 2

    print("validation: PASS")
    print(f"capture_mode: {info.mode}")
    print(f"calibrated_serial: {info.serial}")
    print(f"recording_duration_s: {info.recording_duration_s}")
    print(f"camera_rows_per_camera: {info.camera_rows}")
    print(f"synchronized_imu_rows: {info.synchronized_imu_rows}")
    print("capture_modified: false")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
