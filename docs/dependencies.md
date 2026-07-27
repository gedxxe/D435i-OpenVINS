# Ubuntu dependencies

Review privileged commands before execution. The repository installer prompts
before calling `sudo apt-get` and never removes packages.

## Official librealsense package route

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

The reproducible build accepts the package only at exactly `2.56.5`.
The package also installs persistent udev rules. For a local RSUSB build,
installing an equivalent rule under `/etc/udev/rules.d` is a separate
privileged, persistent host change: inspect the pinned
`.deps/src/librealsense/config/99-realsense-libusb.rules` file first (after
the pinned source has been cloned) and obtain explicit approval before copying
it. Without an installed rule, do not run normal VIO as root merely to hide a
permissions problem.

## Repository-local source fallback

If the exact package is unavailable or kernel integration is problematic,
`scripts/build_ubuntu.sh` clones tag `v2.56.5`, verifies commit
`38a41441971387197193ad3aeae3cefe6a11f2cb`, and builds with
`FORCE_RSUSB_BACKEND=ON` under `.deps`. Optional tools, examples, tests,
graphical and Python extensions, update checks, the all-in-one static bundle,
and firmware downloads are disabled. Librealsense v2.56.5 unconditionally
invokes a bare `ldconfig` from its install rules even for a custom prefix. The
build creates a private no-op executable under `.deps/build` and prepends that
one directory to `PATH` only for `cmake --install`; it does not modify the
persistent environment or `/etc/ld.so.cache`. It does not run upstream helper
scripts, execute firmware tools, patch kernel modules, install under
`/usr/local`, or alter firmware.

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

## Optional Python plotting and calibration tools

Ubuntu's system Python is not modified. The C++ runtime has no Python
dependency. For plots only:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --requirement requirements.txt
```

`requirements.txt` pins Matplotlib, its direct runtime dependency set, and
PyYAML for CPython 3.11-3.13. Summary-only plotting, capture export, and basic
metadata validation use the standard library; Kalibr YAML validation,
promotion, and legacy transform migration require PyYAML. `.venv/` is ignored
by Git.

Kalibr and `allan_variance_ros` are external calibration tools, not runtime
dependencies. They require a separate supported ROS1 environment. Docker is
optional and not installed or assumed. The root README pins exact Kalibr and
Allan commits and describes an isolated Ubuntu 20.04/ROS Noetic image; no
repository script installs Docker, ROS, Kalibr, or Allan tools globally on
Ubuntu 24.04.
