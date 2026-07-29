# D435i OpenVINS Standalone

<p align="center">
  <a href="#project-status"><img alt="Project status: research prototype" src="https://img.shields.io/badge/status-research%20prototype-d99b2b?style=flat-square"></a>
  <a href="LICENSE"><img alt="License: GNU GPL version 3" src="https://img.shields.io/badge/license-GPLv3-2f7d32?style=flat-square"></a>
  <a href="CHANGELOG.md"><img alt="Release: version 0.5.2" src="https://img.shields.io/badge/release-v0.5.2-3367a8?style=flat-square"></a>
  <a href="https://isocpp.org/"><img alt="Language: C++17" src="https://img.shields.io/badge/C%2B%2B-17-00599c?style=flat-square&amp;logo=cplusplus&amp;logoColor=white"></a>
  <a href="https://cmake.org/"><img alt="Build system: CMake" src="https://img.shields.io/badge/build-CMake-064f8c?style=flat-square&amp;logo=cmake&amp;logoColor=white"></a>
  <a href="https://ubuntu.com/"><img alt="Platform: Ubuntu" src="https://img.shields.io/badge/platform-Ubuntu-e95420?style=flat-square&amp;logo=ubuntu&amp;logoColor=white"></a>
</p>

<p align="center">
  <a href="https://github.com/realsenseai/librealsense"><img alt="Camera: RealSense D435i" src="https://img.shields.io/badge/camera-RealSense%20D435i-0071c5?style=flat-square&amp;logo=intel&amp;logoColor=white"></a>
  <a href="https://docs.openvins.com/"><img alt="Estimator: OpenVINS 2.7" src="https://img.shields.io/badge/estimator-OpenVINS%202.7-5849be?style=flat-square"></a>
  <a href="https://opencv.org/"><img alt="Computer vision: OpenCV" src="https://img.shields.io/badge/vision-OpenCV-5c3ee8?style=flat-square&amp;logo=opencv&amp;logoColor=white"></a>
  <a href="docs/dependencies.md"><img alt="Tests: 4 of 4 passing" src="https://img.shields.io/badge/tests-4%2F4%20passing-2f8f46?style=flat-square"></a>
</p>

<p align="center">
  <img src="docs/assets/hero.svg" alt="A RealSense D435i sends stereo infrared images and IMU measurements to OpenVINS, which estimates a three-dimensional path." width="100%">
</p>

This project runs stereo visual-inertial odometry on an Intel RealSense D435i
without ROS. It reads the two infrared cameras and the built-in IMU, sends the
measurements to OpenVINS, and saves the estimated pose, velocity, bias, timing,
and camera information.

It includes tools for live viewing, recording a dataset, replaying a dataset,
checking a camera, and reviewing the saved results.

> [!IMPORTANT]
> This is an odometry project, not a mapping system. It does not use loop
> closure, so position error can build up over time. A zero-velocity update can
> stop false motion when the camera is still, but it cannot move an already
> drifted position back to the correct place.

## Project status

| Item | Current setup |
| --- | --- |
| Release | v0.5.2 |
| Status | Research prototype |
| Host | Ubuntu 22.04 or 24.04 |
| Camera | Intel RealSense D435i |
| Live stereo | 848×480 Y8 at 90 Hz |
| IMU | Gyroscope at 200 Hz, accelerometer at 250 Hz |
| Estimator | OpenVINS v2.7, built with ROS disabled |
| Language | C++17 |
| License | GNU GPLv3 |

The selected camera setup belongs to D435i serial `843212070146`:

```text
config/local/d435i-843212070146/selected_runtime/estimator.yaml
```

That setup is still marked `BOOTSTRAP_UNVERIFIED`. It is suitable for testing
and comparison, but it is not a certified accuracy profile. Tests with the
physical camera found an RSUSB gyro-sensitivity encoding error in the pinned
librealsense source. The supported build applies a reviewed host-side patch and
keeps the runtime gyro scale at `1.0`. Marked-position tests still showed some
endpoint drift, especially in poor light. See
[the selected runtime notes](docs/selected_runtime.md) for the measurements
and the limits of the current 90 Hz setup.

## How it works

<p align="center">
  <a href="docs/architecture.md">
    <img src="docs/assets/runtime-pipeline.svg" alt="Data flow from the D435i cameras and IMU through bounded queues, time synchronization, OpenVINS, and the saved state." width="100%">
  </a>
</p>

The RealSense callback copies each Y8 image once and places it in a bounded
queue. Accelerometer samples are rotated into the gyroscope frame and
interpolated at gyroscope timestamps. One ordered dispatcher sends the
synchronized IMU samples and stereo pairs to OpenVINS.

The program stops with an error if it sees a queue overflow, a timestamp going
backwards, a malformed frame, a calibration mismatch, a non-finite state, an
estimated speed above the safety limit, or an accelerometer bias above the
configured limit.

## Live viewer

<p align="center">
  <a href="docs/manual_test.md">
    <img src="docs/assets/live-viewer-guide.svg" alt="The live viewer has a stereo infrared preview, a fixed global XYZ trajectory view, and a small health and motion status panel." width="100%">
  </a>
</p>

The image window shows both infrared cameras without stretching their aspect
ratio. The trajectory window keeps the grid and global XYZ axes fixed while
the estimated camera pose moves through the scene. Click the picture above to
open the manual test guide.

<details>
<summary><strong>Viewer controls</strong></summary>

- Left-drag: orbit around the global frame.
- Middle/right-drag: pan.
- Mouse wheel: zoom around the pointer.
- `F`: fit the recorded path in the window.
- `R`, `0`, or double-left-click: reset the view.
- `q` or Escape: close the viewer cleanly.

</details>

The health label describes recent feature tracking. It does not prove that the
estimated position is correct.

## Build

Install the supported Ubuntu packages, build the project, and run the tests:

```bash
./scripts/install_ubuntu_dependencies.sh
./scripts/build_ubuntu.sh
./scripts/preflight_ubuntu.sh
```

Run the full test set again:

```bash
ctest --test-dir build/linux-release \
  --output-on-failure \
  --no-tests=error
```

For the C++ core and command-line stubs without camera or OpenVINS support:

```bash
cmake --preset portable-core
cmake --build --preset portable-core
ctest --preset portable-core --output-on-failure
```

The portable build checks only the portable code. It does not test
librealsense, OpenVINS, or a physical D435i.

The full Ubuntu build always uses the pinned, repository-local librealsense
with the reviewed RSUSB gyro patch. A same-version library under `/usr/local`
or another system path is deliberately ignored. Dependency patch hashes are
pinned in `cmake/DependencyVersions.cmake`; the registered repository test
runs the selected-runtime verifier and rejects a stream scale other than
`1.0`.

## Run the live viewer

Connect the selected D435i directly to USB 3 and run this from the repository
root in a graphical Ubuntu session:

```bash
(
  set -euo pipefail

  D435I_SERIAL="843212070146"
  SELECTED_DIR="config/local/d435i-${D435I_SERIAL}/selected_runtime"
  ESTIMATOR_CONFIG="${SELECTED_DIR}/estimator.yaml"
  STREAM_CONFIG="config/sensors/realsense_streams_vio_90hz.yaml"
  RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
  LIVE_RUN="runs/live_diagnostic_${RUN_ID}"

  ./scripts/verify_selected_runtime.sh --serial "${D435I_SERIAL}"
  ./scripts/preflight_ubuntu.sh --require-build
  ./scripts/preflight_ubuntu.sh \
    --require-camera \
    --serial "${D435I_SERIAL}" \
    --stream-config "${STREAM_CONFIG}"

  ./build/linux-release/ovrs_live \
    --config "${ESTIMATOR_CONFIG}" \
    --stream-config "${STREAM_CONFIG}" \
    --serial "${D435I_SERIAL}" \
    --viewer \
    --viewer-history 5000 \
    --allow-unverified-calibration \
    --online-time-offset off \
    --output "${LIVE_RUN}"

  test ! -e "${LIVE_RUN}/INCOMPLETE"
  printf 'Completed live run: %s\n' "${LIVE_RUN}"
)
```

Keep the camera still while the estimator starts. Use a bright room with
static objects and clear texture visible in both infrared images. Stop the run
and keep its output if the estimated movement is clearly different from the
physical movement.

## Programs

| Program | Purpose |
| --- | --- |
| `ovrs_inspect` | Check the connected camera and its stream profiles |
| `ovrs_record` | Record synchronized stereo and IMU data |
| `ovrs_replay` | Run a saved dataset through the estimator |
| `ovrs_live` | Run the camera, estimator, logger, and optional viewer |

Each completed run stores configuration copies and CSV files under its output
directory. The files include raw device timestamps so timing problems can be
checked later.

## Repository layout

```text
apps/                 command-line programs
include/ovrs/, src/   capture, synchronization, estimator adapter, and logging
config/               stream and estimator settings
docs/                 guides, design notes, and test records
patches/              reviewed changes for pinned third-party dependencies
scripts/              build, validation, calibration, and plotting tools
tests/                unit tests, synthetic replay tests, and repository checks
third_party/open_vins pinned OpenVINS submodule
```

## Documentation

- [Operator and calibration runbook](docs/operator_runbook.md)
- [Manual test checklist](docs/manual_test.md)
- [Selected runtime and test results](docs/selected_runtime.md)
- [Architecture](docs/architecture.md)
- [Mathematical background](docs/mathematical_foundation.md)
- [Calibration guide](docs/calibration.md)
- [Dataset format](docs/dataset.md)
- [Timestamp handling](docs/timestamps.md)
- [Dependencies](docs/dependencies.md)
- [Audit summary](AUDIT_REPORT.md)
- [Change history](CHANGELOG.md)
- [Third-party notices](THIRD_PARTY_NOTICES.md)

## Scope and safety

- Do not rename `T_cam_imu` to `T_imu_cam` without inverting the transform.
- Do not promote a calibration by changing only its state label.
- Do not write calibration or firmware to the camera EEPROM from this project.
- Do not mix recordings from different cameras or stream settings.
- Do not report a hardware test as passed unless a D435i was connected.
- Keep the OpenVINS submodule pinned and rebuild it after changing its patch.

This repository does not include ROS, ROS2, MAVLink, ArduPilot, Pixhawk, GPS,
mapping, loop closure, navigation, depth processing, RGB processing,
simulation, or flight control.

## Author

I Gede Bagus Jayendra

## License

This project is licensed under the
[GNU General Public License version 3](LICENSE). Third-party components keep
their own copyright and license terms; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
