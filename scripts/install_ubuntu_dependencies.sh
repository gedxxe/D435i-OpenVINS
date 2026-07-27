#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "This installer supports Ubuntu Linux only." >&2
  exit 2
fi

source /etc/os-release
if [[ "${ID}" != "ubuntu" ||
      ("${VERSION_ID}" != "22.04" && "${VERSION_ID}" != "24.04") ]]; then
  echo "Supported releases are Ubuntu 22.04 and 24.04; detected ${PRETTY_NAME}." >&2
  exit 2
fi

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dependency_pin_file="${repo_dir}/cmake/DependencyVersions.cmake"
pin_value() {
  if [[ -r "${dependency_pin_file}" ]]; then
    sed -n "s/^set($1 \"\\([^\"]*\\)\").*/\\1/p" \
      "${dependency_pin_file}" || true
  fi
}
ceres_version="$(pin_value OVRS_CERES_VERSION)"
openvins_tag="$(pin_value OVRS_OPENVINS_TAG)"
realsense_version="$(pin_value OVRS_LIBREALSENSE_VERSION)"
if [[ -z "${ceres_version}" || -z "${openvins_tag}" ||
      -z "${realsense_version}" ]]; then
  echo "Cannot read dependency versions from the repository." >&2
  exit 2
fi

echo "Detected: ${PRETTY_NAME}"
echo "This command will use sudo apt-get to install build dependencies."
read -r -p "Continue? [y/N] " answer
if [[ ! "${answer}" =~ ^[Yy]$ ]]; then
  echo "No changes made."
  exit 0
fi

sudo apt-get update
sudo apt-get install --no-install-recommends \
  build-essential cmake ninja-build git pkg-config \
  libeigen3-dev libboost-all-dev libopencv-dev \
  libgoogle-glog-dev libgflags-dev \
  libblas-dev liblapack-dev libusb-1.0-0-dev libudev-dev libssl-dev \
  python3 python3-venv python3-tk ca-certificates

cat <<EOF

Ceres ${ceres_version} is built repository-locally by scripts/build_ubuntu.sh.
The system Ceres package is intentionally not installed or removed.
SuiteSparse and CXSparse are intentionally disabled for this Ceres build;
OpenVINS ${openvins_tag} uses its dense Schur solver path.

librealsense2 options:
1. Preferred packaged installation: follow the official Ubuntu 20/22/24
   instructions in docs/dependencies.md, then verify
   'pkg-config --modversion realsense2' reports exactly ${realsense_version}.
2. If that exact package is unavailable or kernel integration is problematic,
   scripts/build_ubuntu.sh builds pinned ${realsense_version}
   repository-locally with the RSUSB backend. It does not patch kernel modules
   or install under /usr/local.

Python is isolated from the C++ runtime. It is optional for recording/runtime
mechanics, but required for the supported calibration validation, promotion,
and plotting workflow. Create the repository-local environment with:
  python3 -m venv .venv
  .venv/bin/python -m pip install --requirement requirements.txt
TkAgg uses the distribution-provided python3-tk package and opens only from a
graphical desktop session. Headless systems can save a PNG with --save.
EOF
