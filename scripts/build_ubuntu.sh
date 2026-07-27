#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "Ubuntu/Linux is required for the supported full build." >&2
  exit 2
fi

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
deps_dir="${repo_dir}/.deps"
src_dir="${deps_dir}/src"
build_dir="${deps_dir}/build"
install_dir="${deps_dir}/install"
# shellcheck source=scripts/lib/cmake_cache_checks.sh
source "${repo_dir}/scripts/lib/cmake_cache_checks.sh"
# shellcheck source=scripts/lib/repository_checks.sh
source "${repo_dir}/scripts/lib/repository_checks.sh"

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 2
  fi
}

verify_pinned_checkout() {
  local label="$1"
  local checkout_dir="$2"
  local expected_commit="$3"
  local actual_commit
  actual_commit="$(git -C "${checkout_dir}" rev-parse HEAD)"
  if [[ "${actual_commit}" != "${expected_commit}" ]]; then
    echo "${label} is at ${actual_commit}, expected ${expected_commit}." >&2
    exit 3
  fi
  if ! ovrs_git_tracked_content_is_clean "${checkout_dir}"; then
    echo "${label} has tracked content changes; refusing to build it." >&2
    ovrs_git_print_tracked_content_changes "${checkout_dir}"
    exit 3
  fi
}

for command_name in git cmake ctest ninja pkg-config gcc g++ grep sed awk tee \
                    nproc install mktemp rm cmp sha256sum; do
  require_command "${command_name}"
done
jobs="${OVRS_JOBS:-$(nproc)}"
if [[ ! "${jobs}" =~ ^[1-9][0-9]*$ ]]; then
  echo "OVRS_JOBS must be a positive integer; received: ${jobs}" >&2
  exit 2
fi
source /etc/os-release
if [[ "${ID}" != "ubuntu" ]]; then
  echo "Supported full builds require Ubuntu 22.04 or 24.04; detected ${PRETTY_NAME}." >&2
  exit 2
fi
if [[ "${VERSION_ID}" != "22.04" && "${VERSION_ID}" != "24.04" ]]; then
  echo "Supported full builds require Ubuntu 22.04 or 24.04; detected ${PRETTY_NAME}." >&2
  exit 2
fi

pin_value() {
  sed -n "s/^set($1 \"\\([^\"]*\\)\").*/\\1/p" "${dependency_pin_file}"
}

dependency_pin_file="${repo_dir}/cmake/DependencyVersions.cmake"
version_file="${repo_dir}/VERSION"
if [[ ! -r "${dependency_pin_file}" || ! -r "${version_file}" ]]; then
  echo "VERSION or cmake/DependencyVersions.cmake is unreadable." >&2
  exit 2
fi
ceres_version="$(pin_value OVRS_CERES_VERSION)"
ceres_tag="$(pin_value OVRS_CERES_TAG)"
ceres_commit="$(pin_value OVRS_CERES_COMMIT)"
realsense_version="$(pin_value OVRS_LIBREALSENSE_VERSION)"
realsense_tag="$(pin_value OVRS_LIBREALSENSE_TAG)"
realsense_commit="$(pin_value OVRS_LIBREALSENSE_COMMIT)"
openvins_tag="$(pin_value OVRS_OPENVINS_TAG)"
openvins_commit="$(pin_value OVRS_OPENVINS_COMMIT)"
project_version="$(sed -n '1p' "${version_file}")"
if [[ ! "${project_version}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "VERSION must contain a MAJOR.MINOR.PATCH semantic version." >&2
  exit 2
fi
for pin in ceres_version ceres_tag ceres_commit realsense_version \
           realsense_tag realsense_commit openvins_tag openvins_commit; do
  if [[ -z "${!pin}" ]]; then
    echo "Cannot read ${pin} from cmake/DependencyVersions.cmake" >&2
    exit 2
  fi
done

mkdir -p "${src_dir}" "${build_dir}" "${install_dir}"
{
  echo "date_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "os=${PRETTY_NAME}"
  echo "kernel=$(uname -r)"
  echo "gcc=$(gcc -dumpfullversion -dumpversion)"
  echo "cmake=$(cmake --version | sed -n '1p')"
  echo "opencv=$(pkg-config --modversion opencv4)"
  echo "eigen=$(pkg-config --modversion eigen3)"
  echo "ceres_before_build=$(pkg-config --modversion ceres-solver 2>/dev/null || echo not-installed)"
  echo "librealsense_before_build=$(pkg-config --modversion realsense2 2>/dev/null || echo not-installed)"
  echo "project_version=${project_version}"
  echo "openvins_tag=${openvins_tag}"
  echo "openvins_commit=${openvins_commit}"
  echo "ceres_tag=${ceres_tag}"
  echo "ceres_commit=${ceres_commit}"
  echo "librealsense_tag=${realsense_tag}"
  echo "librealsense_commit=${realsense_commit}"
  echo "ceres_suitesparse=OFF"
  echo "ceres_cxsparse=OFF"
} | tee "${deps_dir}/environment.txt"

if [[ ! -d "${src_dir}/ceres-solver/.git" ]]; then
  git clone --branch "${ceres_tag}" --depth 1 \
    https://github.com/ceres-solver/ceres-solver.git \
    "${src_dir}/ceres-solver"
fi
verify_pinned_checkout \
  "Ceres" "${src_dir}/ceres-solver" "${ceres_commit}"
cmake -S "${src_dir}/ceres-solver" -B "${build_dir}/ceres" -G Ninja \
  -DCMAKE_BUILD_TYPE:STRING=Release \
  -DCMAKE_INSTALL_PREFIX:PATH="${install_dir}/ceres" \
  -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON \
  -DBUILD_TESTING:BOOL=OFF -DBUILD_EXAMPLES:BOOL=OFF \
  -DBUILD_BENCHMARKS:BOOL=OFF \
  -DSUITESPARSE:BOOL=OFF -DCXSPARSE:BOOL=OFF \
  -DMINIGLOG:BOOL=OFF
ceres_cache="${build_dir}/ceres/CMakeCache.txt"
for disabled_backend in SUITESPARSE CXSPARSE; do
  ovrs_cmake_cache_value_equals \
    "${ceres_cache}" "${disabled_backend}" "OFF" || {
    echo "Ceres cache did not disable ${disabled_backend}: ${ceres_cache}" >&2
    exit 3
  }
done
cmake --build "${build_dir}/ceres" --parallel "${jobs}"
cmake --install "${build_dir}/ceres"

realsense_prefix=""
if [[ "${OVRS_FORCE_LOCAL_REALSENSE:-0}" != "1" ]] &&
   pkg-config --exists realsense2 &&
   [[ "$(pkg-config --modversion realsense2)" = "${realsense_version}" ]]; then
  echo "Using packaged librealsense2 ${realsense_version}."
else
  if [[ ! -d "${src_dir}/librealsense/.git" ]]; then
    git clone --branch "${realsense_tag}" --depth 1 \
      https://github.com/realsenseai/librealsense.git \
      "${src_dir}/librealsense"
  fi
  verify_pinned_checkout \
    "librealsense" "${src_dir}/librealsense" "${realsense_commit}"
  cmake -S "${src_dir}/librealsense" -B "${build_dir}/librealsense" -G Ninja \
    -DCMAKE_BUILD_TYPE:STRING=Release \
    -DCMAKE_INSTALL_PREFIX:PATH="${install_dir}/librealsense" \
    -DFORCE_RSUSB_BACKEND:BOOL=ON \
    -DBUILD_SHARED_LIBS:BOOL=ON \
    -DBUILD_EXAMPLES:BOOL=OFF -DBUILD_GRAPHICAL_EXAMPLES:BOOL=OFF \
    -DBUILD_TOOLS:BOOL=OFF -DBUILD_UNIT_TESTS:BOOL=OFF \
    -DBUILD_PYTHON_BINDINGS:BOOL=OFF -DBUILD_GLSL_EXTENSIONS:BOOL=OFF \
    -DBUILD_RS2_ALL:BOOL=OFF -DCHECK_FOR_UPDATES:BOOL=OFF \
    -DIMPORT_DEPTH_CAM_FW:BOOL=OFF
  realsense_cache="${build_dir}/librealsense/CMakeCache.txt"
  for disabled_option in BUILD_EXAMPLES BUILD_GRAPHICAL_EXAMPLES BUILD_TOOLS \
                         BUILD_UNIT_TESTS BUILD_PYTHON_BINDINGS \
                         BUILD_GLSL_EXTENSIONS BUILD_RS2_ALL \
                         CHECK_FOR_UPDATES IMPORT_DEPTH_CAM_FW; do
    ovrs_cmake_cache_value_equals \
      "${realsense_cache}" "${disabled_option}" "OFF" || {
      echo "librealsense cache did not disable ${disabled_option}." >&2
      exit 3
    }
  done
  ovrs_cmake_cache_value_equals \
    "${realsense_cache}" FORCE_RSUSB_BACKEND "ON" || {
    echo "librealsense cache did not enable the repository-local RSUSB backend." >&2
    exit 3
  }
  ovrs_cmake_cache_value_equals \
    "${realsense_cache}" BUILD_SHARED_LIBS "ON" || {
    echo "librealsense cache did not enable its required shared library." >&2
    exit 3
  }
  ovrs_cmake_cache_value_equals \
    "${realsense_cache}" CMAKE_INSTALL_PREFIX \
    "${install_dir}/librealsense" || {
    echo "librealsense cache does not use the repository-local install prefix." >&2
    exit 3
  }
  cmake --build "${build_dir}/librealsense" --parallel "${jobs}"
  ldconfig_shim_dir="${build_dir}/libexec"
  mkdir -p "${ldconfig_shim_dir}"
  install -m 0755 "${repo_dir}/scripts/lib/ldconfig_noop.sh" \
    "${ldconfig_shim_dir}/ldconfig"
  PATH="${ldconfig_shim_dir}:${PATH}" \
    cmake --install "${build_dir}/librealsense"
  local_realsense_pkgconfig="${install_dir}/librealsense/lib/pkgconfig"
  local_realsense_pkgconfig="${local_realsense_pkgconfig}:${install_dir}/librealsense/lib64/pkgconfig"
  resolved_local_realsense_version="$(
    PKG_CONFIG_PATH="${local_realsense_pkgconfig}" \
      pkg-config --modversion realsense2 2>/dev/null || true
  )"
  if [[ "${resolved_local_realsense_version}" != "${realsense_version}" ]]; then
    echo "Repository-local librealsense reports ${resolved_local_realsense_version:-no version}, expected ${realsense_version}." >&2
    exit 3
  fi
  realsense_prefix="${install_dir}/librealsense"
fi

openvins_dir="${repo_dir}/third_party/open_vins"
openvins_patch="${repo_dir}/patches/openvins-zupt-velocity-constraint.patch"
if [[ ! -d "${openvins_dir}/.git" && ! -f "${openvins_dir}/.git" ]]; then
  git -C "${repo_dir}" submodule update --init --recursive
fi
actual_openvins_commit="$(git -C "${openvins_dir}" rev-parse HEAD)"
if [[ "${actual_openvins_commit}" != "${openvins_commit}" ]]; then
  echo "OpenVINS is at ${actual_openvins_commit}, expected ${openvins_commit}." >&2
  exit 3
fi
if ovrs_git_tracked_content_is_clean "${openvins_dir}"; then
  git -C "${openvins_dir}" apply --ignore-space-change --ignore-whitespace \
    "${openvins_patch}"
fi
if ! ovrs_git_tracked_content_matches_patch \
     "${openvins_dir}" "${openvins_patch}"; then
  echo "OpenVINS does not exactly match the reviewed project patch." >&2
  ovrs_git_print_tracked_content_changes "${openvins_dir}"
  exit 3
fi
cmake -S "${repo_dir}/third_party/open_vins/ov_msckf" \
  -B "${build_dir}/open_vins" -G Ninja \
  -DCMAKE_BUILD_TYPE:STRING=Release \
  -DCMAKE_INSTALL_PREFIX:PATH="${install_dir}/open_vins" \
  -DCMAKE_PREFIX_PATH:STRING="${install_dir}/ceres" \
  -DCeres_DIR:PATH="${install_dir}/ceres/lib/cmake/Ceres" \
  -DENABLE_ROS:BOOL=OFF -DENABLE_ARUCO_TAGS:BOOL=OFF \
  -DCMAKE_DISABLE_FIND_PACKAGE_catkin:BOOL=TRUE \
  -DCMAKE_DISABLE_FIND_PACKAGE_ament_cmake:BOOL=TRUE
openvins_cache="${build_dir}/open_vins/CMakeCache.txt"
ovrs_cmake_cache_value_equals \
  "${openvins_cache}" Ceres_DIR \
  "${install_dir}/ceres/lib/cmake/Ceres" || {
    echo "OpenVINS did not resolve repository-local Ceres." >&2
    exit 3
  }
for cache_key in ENABLE_ROS ENABLE_ARUCO_TAGS; do
  ovrs_cmake_cache_value_equals \
    "${openvins_cache}" "${cache_key}" "OFF" || {
    echo "OpenVINS cache did not disable ${cache_key}." >&2
    exit 3
  }
done
for cache_key in CMAKE_DISABLE_FIND_PACKAGE_catkin \
                 CMAKE_DISABLE_FIND_PACKAGE_ament_cmake; do
  ovrs_cmake_cache_value_equals \
    "${openvins_cache}" "${cache_key}" "TRUE" || {
    echo "OpenVINS cache did not enforce ${cache_key}=TRUE." >&2
    exit 3
  }
done
cmake --build "${build_dir}/open_vins" --parallel "${jobs}"
cmake --install "${build_dir}/open_vins"

prefix_path="${install_dir}/ceres"
if [[ -n "${realsense_prefix}" ]]; then
  prefix_path="${prefix_path};${realsense_prefix}"
fi
cd "${repo_dir}"
cmake --preset linux-release \
  -DOVRS_OPENVINS_PREFIX:PATH="${install_dir}/open_vins" \
  -DOVRS_CERES_PREFIX:PATH="${install_dir}/ceres" \
  -DCMAKE_PREFIX_PATH:STRING="${prefix_path}" \
  -DCeres_DIR:PATH="${install_dir}/ceres/lib/cmake/Ceres"
cache_file="${repo_dir}/build/linux-release/CMakeCache.txt"
ovrs_cmake_cache_value_equals \
  "${cache_file}" Ceres_DIR \
  "${install_dir}/ceres/lib/cmake/Ceres" || {
    echo "CMake did not resolve repository-local Ceres: ${cache_file}" >&2
    exit 3
  }
ovrs_cmake_cache_value_equals \
  "${cache_file}" OVRS_CERES_PREFIX "${install_dir}/ceres" || {
    echo "CMake cache does not contain the required local Ceres prefix." >&2
    exit 3
  }
ovrs_cmake_cache_value_equals \
  "${cache_file}" OVRS_PROJECT_VERSION_RESOLVED "${project_version}" || {
    echo "CMake cache project version does not match VERSION." >&2
    exit 3
  }
source_fingerprint="$(
  ovrs_cmake_cache_value \
    "${cache_file}" OVRS_SOURCE_FINGERPRINT_RESOLVED
)" || {
  echo "CMake cache is missing the project source fingerprint." >&2
  exit 3
}
if [[ ! "${source_fingerprint}" =~ ^[0-9a-f]{64}$ ]]; then
  echo "CMake cache contains an invalid project source fingerprint." >&2
  exit 3
fi
current_source_fingerprint="$(
  ovrs_current_source_fingerprint "${repo_dir}"
)" || {
  echo "Could not calculate the current project source fingerprint." >&2
  exit 3
}
if [[ "${current_source_fingerprint}" != "${source_fingerprint}" ]]; then
  echo "CMake cache source fingerprint does not match the current source tree." >&2
  exit 3
fi
cmake --build --preset linux-release --parallel "${jobs}"
for executable in ovrs_inspect ovrs_record ovrs_live ovrs_replay; do
  executable_path="${repo_dir}/build/linux-release/${executable}"
  if ! ovrs_executable_build_identity_matches \
       "${executable_path}" "${executable}" "${project_version}" \
       "${source_fingerprint}"; then
    echo "${executable} is stale or was not built from the current source content." >&2
    echo "Expected source fingerprint: ${source_fingerprint}" >&2
    exit 3
  fi
done
ctest --test-dir "${repo_dir}/build/linux-release" \
  --output-on-failure --no-tests=error
{
  echo "resolved_ceres=${ceres_version}"
  echo "resolved_librealsense=${realsense_version}"
  echo "resolved_opencv=$(pkg-config --modversion opencv4)"
  echo "resolved_openvins=${openvins_tag}@${openvins_commit}"
  echo "resolved_openvins_patch=$(sha256sum "${openvins_patch}" | awk '{print $1}')"
  echo "resolved_project_version=${project_version}"
  echo "resolved_source_fingerprint=${source_fingerprint}"
} | tee -a "${deps_dir}/environment.txt"
