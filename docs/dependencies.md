# Ubuntu dependencies

Review privileged commands before execution. The repository installer prompts
before calling `sudo apt-get` and never removes packages.

## Optional official librealsense utilities

The official Ubuntu 20/22/24 instructions currently use:

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc |
  gpg --dearmor |
  sudo tee /etc/apt/keyrings/librealsenseai.gpg >/dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] \
https://librealsense.realsenseai.com/Debian/apt-repo \
$(lsb_release -cs) main" |
  sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get install librealsense2-utils librealsense2-dev
```

These commands add a persistent package source and key, so run them only after
review and explicit approval. Confirm the resolved version:

```bash
pkg-config --modversion realsense2
```

The supported project build does not link this package, even when it reports
version `2.57.3`; it always links the reviewed repository-local build below.
The package can still provide diagnostic utilities and persistent udev rules.
For the repository-local RSUSB build,
installing an equivalent rule under `/etc/udev/rules.d` is a separate
privileged, persistent host change: inspect the pinned
`.deps/src/librealsense/config/99-realsense-libusb.rules` file first (after
the pinned source has been cloned) and obtain explicit approval before copying
it. Without an installed rule, do not run normal VIO as root merely to hide a
permissions problem.

## Required repository-local source

`scripts/build_ubuntu.sh` always clones tag `v2.57.3`, verifies commit
`5e046e509995cda79b42d89fa95ab65f90678641`, and builds with
`FORCE_RSUSB_BACKEND=ON` under `.deps`. It applies and verifies
`patches/librealsense-rsusb-gyro-sensitivity.patch`, which preserves dynamic
gyro-sensitivity values when the RSUSB backend encodes the unsigned feature
report and converts failed libusb initialization or enumeration into a clean
backend error instead of a null-context crash. The patch SHA-256 is pinned
beside the dependency commit in
`cmake/DependencyVersions.cmake` and is checked before its content can enter a
build. Optional tools, examples, tests, graphical and Python extensions,
update checks, the all-in-one static bundle, and firmware downloads are
disabled. Librealsense v2.57.3 unconditionally invokes a bare `ldconfig` from
its install rules even for a custom prefix. The build creates a private no-op
executable under `.deps/build` and prepends that one directory to `PATH` only
for `cmake --install`; it does not modify the persistent environment or
`/etc/ld.so.cache`. It does not run upstream helper scripts, execute firmware
tools, patch kernel modules, install under `/usr/local`, or alter firmware.
Project CMake uses `NO_DEFAULT_PATH`, and preflight checks the loaded library
path for every hardware executable.

Reconfiguration does not delete prior build outputs. If an earlier run used
librealsense's default `BUILD_TOOLS=ON`, stale optional executables may remain
under the ignored `.deps` tree. They are not targets of the current build, are
not placed on the application runtime `PATH`, and must not be used to update a
camera.

## Ceres and OpenVINS

Ceres is always built at tag 2.1.0, commit
`f68321e7de8929fbcdb95dd42877531e64f72f66`, under `.deps/install/ceres`.
The build disables Ceres `SUITESPARSE` and `CXSPARSE`. Ubuntu 24.04's
SuiteSparse 7 package exposed CXSparse to Ceres 2.1.0 without the imported
`CXSparse::CXSparse` target expected by that release, which made CMake
generation fail. OpenVINS v2.7's dynamic initializer selects
`ceres::DENSE_SCHUR`, so these optional sparse backends are not used by the
configured estimator path. Eigen sparse support remains available in Ceres.
OpenVINS v2.7 is then configured against only that Ceres prefix with
`ENABLE_ROS=OFF`; CMake discovery of both `catkin` and `ament_cmake` is also
disabled so an existing ROS installation cannot leak link dependencies into
the standalone library. A system Ceres 2.2 installation is neither uninstalled
nor overwritten, and project CMake uses `NO_DEFAULT_PATH` so it cannot be
selected.

The pinned `third_party/open_vins` submodule remains clean. The build creates
an ignored local clone at `.deps/src/open_vins`, checks out the exact submodule
commit, applies the reviewed ZUPT patch there, and requires the OpenVINS CMake
cache to name that disposable source. This prevents a normal build from
leaving the superproject dirty.

## Optional Python plotting and calibration tools

Ubuntu's system Python is not modified. The C++ runtime has no Python
dependency. For plots only:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --requirement requirements.txt
```

`requirements.txt` pins Matplotlib, its direct runtime dependency set, and
PyYAML for CPython 3.11-3.13. Summary-only plotting, capture export, and basic
metadata validation use the standard library. The markerless VSLAM benchmark
exporter is also standard-library-only; external SLAM backends are not Python
dependencies of this repository. The ORB-SLAM3 adapter is likewise
standard-library-only. Its pinned external source and build remain in the
ignored `.deps/` tree and are covered in
[the offline backend notes](orbslam3_offline.md). Atlas-enabled experiments
also require the reviewed serialization-integrity patch pinned in
`config/research/orbslam3_backend.yaml`; adapter preparation verifies its
SHA-256 before creating an experiment directory. Kalibr YAML validation,
promotion, and legacy transform migration require PyYAML. `.venv/` is ignored
by Git.

When the optional live ORB-SLAM3 executable is enabled, it links only the
system OpenSSL `Crypto` component from the already required `libssl-dev`
package. This records capture-time SHA-256 provenance for the executable,
backend library, vocabulary, settings, and bundle; it is not used for
networking or estimator logic.

Kalibr and `allan_variance_ros` are external calibration tools, not runtime
dependencies. They require a separate supported ROS1 environment. Docker is
optional and not installed or assumed. The root README pins exact Kalibr and
Allan commits and describes an isolated Ubuntu 20.04/ROS Noetic image; no
repository script installs Docker, ROS, Kalibr, or Allan tools globally on
Ubuntu 24.04.
