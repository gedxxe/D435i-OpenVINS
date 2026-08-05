# Repository guide

## Scope

This branch is v0.6.0 markerless stereo visual-inertial SLAM research for an
Intel RealSense D435i, with the reviewed v0.5.2 OpenVINS v2.7 runtime retained
as the odometry baseline. Marker-based estimation is out of scope. Do not add
ROS, ROS2, MAVLink, ArduPilot, Pixhawk, GPS, Webots, navigation, depth
processing, RGB processing, or flight control.

Mapping, loop closure, and relocalization work must begin as offline,
backend-neutral benchmark adapters. Do not feed global corrections into
OpenVINS or change the live v0.5.2 estimator path until identical-data
benchmarks and frame contracts justify a reviewed integration.

Hardware tests must never be reported as successful unless they were actually
run with a connected D435i. A build without hardware dependencies validates
only the portable core and stub CLI behavior.

## Architecture

- `include/ovrs`, `src`: project-owned synchronization, dispatch, capture,
  estimator adapter, and logging libraries.
- `apps`: inspector, recorder, live runner, and dataset replay executables.
- `config`: stream, calibration, estimator, and logging configuration.
- `third_party/open_vins`: pinned Git submodule; do not edit it casually.
- `tests`: dependency-light unit and synthetic replay tests.
- `scripts`: supported Ubuntu dependency, build, run, and plotting entry points.
- `docs`: timing, calibration, architecture, Windows, and manual test contracts.
- `scripts/export_vislam_benchmark.py`: fail-closed EuRoC staging for offline
  markerless backend comparison.

The RealSense callback copies each bounded Y8 frame once into owned memory and
only enqueues measurements. The ordered dispatcher is the sole owner of
OpenVINS ingestion.

## Build and test

```bash
./scripts/install_ubuntu_dependencies.sh
./scripts/build_ubuntu.sh
./scripts/preflight_ubuntu.sh
```

`build_ubuntu.sh` runs the registered tests. To repeat only those tests:

```bash
ctest --test-dir build/linux-release --output-on-failure --no-tests=error
```

Portable core only:

```bash
cmake --preset portable-core
cmake --build --preset portable-core
ctest --preset portable-core --output-on-failure
```

## Conventions

- C++17, RAII, deterministic ownership, bounded queues, no detached threads.
- Project code uses `-Wall -Wextra -Wpedantic`; do not impose `-Werror` on
  upstream code.
- SI units internally: seconds, metres, radians, m/s, rad/s, m/s^2.
- Preserve raw device timestamps beside normalized timestamps.
- Camera IDs are fixed: IR1 is 0 and IR2 is 1.
- Format C++ with the repository `.clang-format` before review.
- Shell scripts must use `set -euo pipefail` and quote paths.
- Python is optional for the C++ runtime but required for the supported
  calibration-validation/promotion workflow and plotting; keep third-party
  packages in `.venv`, never system Python.
- Never write calibration or firmware to device EEPROM.
- Keep local odometry and globally corrected map poses in distinct frames.
- Pin external SLAM revisions and review their licenses before integration;
  do not casually vendor another large source tree.

## Definition of done

Changes are done when the cleanest available build and all non-hardware tests
pass, CLI help is consistent with README commands, timestamp and frame
conventions are documented, OpenVINS remains pinned with `ENABLE_ROS=OFF`, the
diff contains no personal paths or fabricated measurements, and unexecuted
hardware/Linux validation is stated explicitly.

Research results additionally require identical source data, immutable
provenance hashes, external ground truth for accuracy claims, false-loop
closure accounting, and measured Raspberry Pi 5 resource results before any
real-time claim.
