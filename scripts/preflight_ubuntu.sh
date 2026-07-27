#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "${BASH_SOURCE[0]%/*}/.." && pwd)"
# shellcheck source=scripts/lib/cmake_cache_checks.sh
source "${repo_dir}/scripts/lib/cmake_cache_checks.sh"
# shellcheck source=scripts/lib/repository_checks.sh
source "${repo_dir}/scripts/lib/repository_checks.sh"
require_camera=0
require_build=0
camera_serial=""
camera_stream_config="${repo_dir}/config/sensors/realsense_streams.yaml"
camera_stream_config_explicit=0
errors=0
warnings=0

usage() {
  echo "Usage: $0 [--require-build]"
  echo "       $0 --require-camera --serial SERIAL [--stream-config STREAMS.yaml]"
  echo "Read-only Ubuntu prerequisite, pin, build-cache, and D435i checks."
}

pass() {
  echo "PASS: $*"
}

info() {
  echo "INFO: $*"
}

warn() {
  echo "WARN: $*" >&2
  warnings=$((warnings + 1))
}

fail() {
  echo "FAIL: $*" >&2
  errors=$((errors + 1))
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --require-camera)
      require_camera=1
      require_build=1
      ;;
    --require-build)
      require_build=1
      ;;
    --serial)
      if [[ "$#" -lt 2 || -z "$2" ]]; then
        echo "--serial requires a value." >&2
        exit 2
      fi
      camera_serial="$2"
      shift
      ;;
    --stream-config)
      if [[ "$#" -lt 2 || -z "$2" ]]; then
        echo "--stream-config requires a path." >&2
        exit 2
      fi
      camera_stream_config="$2"
      camera_stream_config_explicit=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done
if [[ -n "${camera_serial}" && "${require_camera}" -ne 1 ]]; then
  echo "--serial is meaningful only with --require-camera." >&2
  exit 2
fi
if [[ "${camera_stream_config_explicit}" -eq 1 &&
      "${require_camera}" -ne 1 ]]; then
  echo "--stream-config is meaningful only with --require-camera." >&2
  exit 2
fi
if [[ "${require_camera}" -eq 1 &&
      ! "${camera_serial}" =~ ^[0-9]+$ ]]; then
  echo "--require-camera requires the numeric D435i serial via --serial." >&2
  exit 2
fi
if [[ "${require_camera}" -eq 1 &&
      ( ! -f "${camera_stream_config}" ||
        ! -r "${camera_stream_config}" ) ]]; then
  echo "--stream-config must name a readable stream configuration." >&2
  exit 2
fi

host_os="$(uname -s 2>/dev/null || true)"
if [[ "${host_os}" != "Linux" ]]; then
  fail "Linux is required; this host reports ${host_os:-unknown}."
  echo
  echo "Preflight summary: errors=${errors} warnings=${warnings}"
  echo "PREFLIGHT_RESULT=FAIL"
  exit 1
fi

if [[ ! -r /etc/os-release ]]; then
  fail "Cannot read /etc/os-release."
else
  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "24.04" ]]; then
    pass "Operating system is ${PRETTY_NAME}."
  elif [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "22.04" ]]; then
    warn "${PRETTY_NAME} is secondary-supported; this is not Ubuntu 24.04 verification."
  else
    fail "Expected Ubuntu 24.04; detected ${PRETTY_NAME:-unknown Linux}."
  fi
fi

case "$(uname -m)" in
  x86_64)
    pass "Architecture is x86_64."
    ;;
  aarch64)
    warn "ARM64 project code is portable, but this dependency stack is not validated here."
    ;;
  *)
    fail "Unsupported architecture: $(uname -m)."
    ;;
esac
info "Kernel: $(uname -r)"

required_commands=(
  git cmake ctest ninja pkg-config gcc g++ grep sed awk diff wc date tee timeout
  nproc install chmod cp dirname mkdir mktemp mv rm rmdir cmp sha256sum
)
for command_name in "${required_commands[@]}"; do
  if command -v "${command_name}" >/dev/null 2>&1; then
    pass "${command_name}: $(command -v "${command_name}")"
  else
    fail "Missing command: ${command_name}"
  fi
done

if command -v cmake >/dev/null 2>&1; then
  cmake_version="$(cmake --version | sed -n '1s/^cmake version //p')"
  if command -v dpkg >/dev/null 2>&1 &&
     dpkg --compare-versions "${cmake_version}" ge 3.22; then
    pass "CMake ${cmake_version} satisfies >= 3.22."
  else
    fail "CMake ${cmake_version:-unknown} does not satisfy >= 3.22."
  fi
fi

project_version=""
if [[ -r "${repo_dir}/VERSION" ]]; then
  project_version="$(sed -n '1p' "${repo_dir}/VERSION")"
  if [[ "${project_version}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    pass "Project version is ${project_version}."
  else
    fail "VERSION does not contain a valid semantic version."
  fi
else
  fail "VERSION is missing or unreadable."
fi

pin_value() {
  local pin_file="${repo_dir}/cmake/DependencyVersions.cmake"
  if [[ -r "${pin_file}" ]]; then
    sed -n "s/^set($1 \"\\([^\"]*\\)\").*/\\1/p" "${pin_file}" || true
  fi
}

openvins_tag="$(pin_value OVRS_OPENVINS_TAG)"
openvins_commit="$(pin_value OVRS_OPENVINS_COMMIT)"
ceres_version="$(pin_value OVRS_CERES_VERSION)"
ceres_commit="$(pin_value OVRS_CERES_COMMIT)"
realsense_version="$(pin_value OVRS_LIBREALSENSE_VERSION)"
realsense_commit="$(pin_value OVRS_LIBREALSENSE_COMMIT)"
kalibr_commit="$(pin_value OVRS_KALIBR_COMMIT)"
allan_commit="$(pin_value OVRS_ALLAN_VARIANCE_ROS_COMMIT)"
if [[ -n "${openvins_tag}" && -n "${openvins_commit}" &&
      -n "${ceres_version}" && -n "${ceres_commit}" &&
      -n "${realsense_version}" && -n "${realsense_commit}" &&
      -n "${kalibr_commit}" && -n "${allan_commit}" ]]; then
  pass "Dependency pins loaded: OpenVINS ${openvins_tag}, Ceres ${ceres_version}, librealsense ${realsense_version}."
  info "External calibration pins: Kalibr ${kalibr_commit}, allan_variance_ros ${allan_commit}."
else
  fail "One or more dependency pins are missing from cmake/DependencyVersions.cmake."
fi

calibration_dockerfile="${repo_dir}/docker/calibration.Dockerfile"
if [[ -r "${calibration_dockerfile}" ]] &&
   grep -Fq 'ARG KALIBR_IMAGE' "${calibration_dockerfile}" &&
   grep -Fq 'FROM ${KALIBR_IMAGE}' "${calibration_dockerfile}" &&
   grep -Fq 'COPY . /catkin_ws/src/allan_variance_ros' \
     "${calibration_dockerfile}" &&
   grep -Fq 'catkin build allan_variance_ros' \
     "${calibration_dockerfile}"; then
  pass "Pinned external-calibration Dockerfile contract is present."
else
  fail "External-calibration Dockerfile is missing or inconsistent."
fi

openvins_dir="${repo_dir}/third_party/open_vins"
openvins_patch="${repo_dir}/patches/openvins-zupt-velocity-constraint.patch"
if [[ -d "${openvins_dir}/.git" || -f "${openvins_dir}/.git" ]]; then
  actual_openvins_commit="$(git -C "${openvins_dir}" rev-parse HEAD 2>/dev/null || true)"
  if [[ "${actual_openvins_commit}" == "${openvins_commit}" ]]; then
    pass "OpenVINS submodule is at ${actual_openvins_commit}."
  else
    fail "OpenVINS is ${actual_openvins_commit:-unreadable}, expected ${openvins_commit}."
  fi
  if [[ -r "${openvins_patch}" ]] &&
     ovrs_git_tracked_content_matches_patch \
       "${openvins_dir}" "${openvins_patch}"; then
    pass "OpenVINS exactly matches the reviewed ZUPT velocity patch."
  else
    fail "OpenVINS does not exactly match the reviewed project patch."
    ovrs_git_print_tracked_content_changes "${openvins_dir}" || true
  fi
else
  fail "OpenVINS submodule is not initialized."
fi

check_pkg_config() {
  local package="$1"
  local label="$2"
  if pkg-config --exists "${package}" 2>/dev/null; then
    pass "${label}: $(pkg-config --modversion "${package}")"
  else
    fail "${label} development package is not visible to pkg-config."
  fi
}

if command -v pkg-config >/dev/null 2>&1; then
  check_pkg_config opencv4 OpenCV
  check_pkg_config eigen3 Eigen
fi

if command -v dpkg-query >/dev/null 2>&1; then
  for package in libboost-dev libgoogle-glog-dev libgflags-dev \
                 libblas-dev liblapack-dev libusb-1.0-0-dev libudev-dev \
                 libssl-dev; do
    if dpkg-query -W -f='${Status}' "${package}" 2>/dev/null |
       grep -Fq "install ok installed"; then
      pass "Package installed: ${package}"
    else
      fail "Missing package: ${package}"
    fi
  done
  if dpkg-query -W -f='${Status}' python3-tk 2>/dev/null |
     grep -Fq "install ok installed"; then
    pass "Optional interactive plotting package installed: python3-tk"
  else
    warn "Optional python3-tk is missing; only interactive Python plots need it."
  fi
fi

ceres_prefix="${repo_dir}/.deps/install/ceres"
ceres_config="${ceres_prefix}/lib/cmake/Ceres/CeresConfig.cmake"
ceres_build_cache="${repo_dir}/.deps/build/ceres/CMakeCache.txt"
if [[ -f "${ceres_config}" ]]; then
  if grep -Rqs "${ceres_version}" "${ceres_prefix}" 2>/dev/null; then
    pass "Repository-local Ceres ${ceres_version} is installed."
  else
    fail "Repository-local Ceres exists but does not report ${ceres_version}."
  fi
else
  if [[ "${require_build}" -eq 1 ]]; then
    fail "Repository-local Ceres is not built; run scripts/build_ubuntu.sh."
  else
    warn "Repository-local Ceres is not built yet; run scripts/build_ubuntu.sh."
  fi
fi
if [[ -f "${ceres_build_cache}" ]]; then
  if ovrs_cmake_cache_value_equals \
       "${ceres_build_cache}" SUITESPARSE OFF &&
     ovrs_cmake_cache_value_equals \
       "${ceres_build_cache}" CXSPARSE OFF; then
    pass "Ceres cache disables SuiteSparse and CXSparse compatibility backends."
  elif [[ -f "${ceres_config}" ]]; then
    fail "Installed local Ceres was not configured with the required sparse-backend policy."
  else
    warn "Partial Ceres cache predates the Ubuntu 24.04 sparse-backend fix; build_ubuntu.sh will reconfigure it in place."
  fi
fi
if command -v pkg-config >/dev/null 2>&1 &&
   pkg-config --exists ceres-solver 2>/dev/null; then
  info "System-visible Ceres is $(pkg-config --modversion ceres-solver); project CMake ignores it."
else
  info "No system Ceres is visible through pkg-config."
fi

realsense_ok=0
system_realsense_ok=0
local_realsense_ok=0
if command -v pkg-config >/dev/null 2>&1 &&
   pkg-config --exists realsense2 2>/dev/null; then
  system_realsense_version="$(pkg-config --modversion realsense2)"
  if [[ "${system_realsense_version}" == "${realsense_version}" ]]; then
    pass "System librealsense is exactly ${system_realsense_version}."
    realsense_ok=1
    system_realsense_ok=1
  else
    warn "System librealsense is ${system_realsense_version}; the build will ignore it and build ${realsense_version} locally."
  fi
fi
local_realsense_pc="${repo_dir}/.deps/install/librealsense/lib/pkgconfig"
if command -v pkg-config >/dev/null 2>&1 &&
   PKG_CONFIG_PATH="${local_realsense_pc}:${repo_dir}/.deps/install/librealsense/lib64/pkgconfig" \
   pkg-config --exists realsense2 2>/dev/null; then
  local_realsense_version="$(
    PKG_CONFIG_PATH="${local_realsense_pc}:${repo_dir}/.deps/install/librealsense/lib64/pkgconfig" \
      pkg-config --modversion realsense2
  )"
  if [[ "${local_realsense_version}" == "${realsense_version}" ]]; then
    pass "Repository-local librealsense is exactly ${local_realsense_version}."
    realsense_ok=1
    local_realsense_ok=1
  else
    fail "Repository-local librealsense is ${local_realsense_version}, expected ${realsense_version}."
  fi
fi
if [[ "${realsense_ok}" -eq 0 ]]; then
  if [[ "${require_build}" -eq 1 ]]; then
    fail "Pinned librealsense ${realsense_version} is unavailable; run build_ubuntu.sh."
  else
    warn "Pinned librealsense ${realsense_version} is not installed yet; build_ubuntu.sh will build it locally."
  fi
fi
realsense_build_cache="${repo_dir}/.deps/build/librealsense/CMakeCache.txt"
if [[ "${system_realsense_ok}" -eq 0 &&
      "${local_realsense_ok}" -eq 1 &&
      -f "${realsense_build_cache}" ]]; then
  realsense_cache_safe=1
  for disabled_option in BUILD_EXAMPLES BUILD_GRAPHICAL_EXAMPLES BUILD_TOOLS \
                         BUILD_UNIT_TESTS BUILD_PYTHON_BINDINGS \
                         BUILD_GLSL_EXTENSIONS BUILD_RS2_ALL \
                         CHECK_FOR_UPDATES IMPORT_DEPTH_CAM_FW; do
    if ! ovrs_cmake_cache_value_equals \
         "${realsense_build_cache}" "${disabled_option}" OFF; then
      realsense_cache_safe=0
    fi
  done
  if ! ovrs_cmake_cache_value_equals \
       "${realsense_build_cache}" FORCE_RSUSB_BACKEND ON ||
     ! ovrs_cmake_cache_value_equals \
       "${realsense_build_cache}" BUILD_SHARED_LIBS ON ||
     ! ovrs_cmake_cache_value_equals \
       "${realsense_build_cache}" CMAKE_INSTALL_PREFIX \
       "${repo_dir}/.deps/install/librealsense"; then
    realsense_cache_safe=0
  fi
  if [[ "${realsense_cache_safe}" -eq 1 ]]; then
    pass "Local librealsense cache uses the minimal RSUSB build policy."
  else
    warn "Local librealsense cache predates the minimal no-firmware-download policy; build_ubuntu.sh will reconfigure it in place."
  fi
fi

if command -v python3 >/dev/null 2>&1; then
  python_version="$(python3 -c 'import platform; print(platform.python_version())' 2>/dev/null || true)"
  if python3 -c 'import sys; raise SystemExit(sys.version_info < (3, 11))'; then
    pass "Optional plotting/calibration Python is ${python_version}."
  else
    warn "Python ${python_version:-unknown} cannot use the optional pinned plotting/calibration environment; the C++ runtime is unaffected."
  fi
  if python3 -m venv --help >/dev/null 2>&1; then
    pass "python3 venv support is available."
  else
    warn "python3-venv support is unavailable; only optional plotting/calibration tools are affected."
  fi
  if [[ -x "${repo_dir}/.venv/bin/python" ]]; then
    matplotlib_version="$(
      sed -n 's/^matplotlib==\([^[:space:]]*\)$/\1/p' \
        "${repo_dir}/requirements.txt" 2>/dev/null || true
    )"
    pyyaml_version="$(
      sed -n 's/^PyYAML==\([^[:space:]]*\)$/\1/p' \
        "${repo_dir}/requirements.txt" 2>/dev/null || true
    )"
    if [[ -n "${matplotlib_version}" ]] &&
       "${repo_dir}/.venv/bin/python" -c \
         'import matplotlib, sys; assert matplotlib.__version__ == sys.argv[1]' \
         "${matplotlib_version}"; then
      pass "Optional .venv has Matplotlib ${matplotlib_version}."
    else
      warn "Optional .venv does not match the pinned Matplotlib requirement."
    fi
    if [[ -n "${pyyaml_version}" ]] &&
       "${repo_dir}/.venv/bin/python" -c \
         'import yaml, sys; assert yaml.__version__ == sys.argv[1]' \
         "${pyyaml_version}"; then
      pass "Optional .venv has PyYAML ${pyyaml_version}."
    else
      warn "Optional .venv does not match the pinned PyYAML calibration requirement."
    fi
  else
    info "Optional .venv is absent; the C++ runtime does not require it."
  fi
else
  warn "python3 is missing; C++ runtime is unaffected, but calibration validation/promotion and plotting are unavailable."
fi

if command -v python3 >/dev/null 2>&1; then
  calibration_python_ok=1
  for script in calibration_common.py create_aprilgrid_target.py \
                plan_aprilgrid_target.py \
                validate_calibration_capture.py \
                export_calibration_capture.py \
                validate_calibration_export_set.py \
                validate_kalibr_outputs.py \
                prepare_imu_calibration_yaml.py \
                prepare_verified_calibration.py \
                migrate_openvins_transform_v050.py \
                analyze_stationary_imu.py plot_trajectory.py; do
    if ! PYTHONDONTWRITEBYTECODE=1 python3 -c \
         'from pathlib import Path; compile(Path(__import__("sys").argv[1]).read_text(encoding="utf-8"), __import__("sys").argv[1], "exec")' \
         "${repo_dir}/scripts/${script}"; then
      calibration_python_ok=0
    fi
  done
  if [[ "${calibration_python_ok}" -eq 1 ]] &&
     python3 "${repo_dir}/scripts/create_aprilgrid_target.py" \
       --help >/dev/null &&
     python3 "${repo_dir}/scripts/plan_aprilgrid_target.py" \
       --help >/dev/null &&
     python3 "${repo_dir}/scripts/validate_calibration_capture.py" \
       --help >/dev/null &&
     python3 "${repo_dir}/scripts/export_calibration_capture.py" \
       --help >/dev/null &&
     python3 "${repo_dir}/scripts/validate_calibration_export_set.py" \
       --help >/dev/null &&
     python3 "${repo_dir}/scripts/validate_kalibr_outputs.py" \
       --help >/dev/null &&
     python3 "${repo_dir}/scripts/prepare_imu_calibration_yaml.py" \
       --help >/dev/null &&
     python3 "${repo_dir}/scripts/prepare_verified_calibration.py" \
       --help >/dev/null &&
     python3 "${repo_dir}/scripts/migrate_openvins_transform_v050.py" \
       --help >/dev/null &&
     python3 "${repo_dir}/scripts/analyze_stationary_imu.py" \
       --help >/dev/null &&
     python3 "${repo_dir}/scripts/plot_trajectory.py" \
       --help >/dev/null; then
    pass "All project Python scripts parse and their CLI help is available."
  else
    fail "One or more project Python scripts failed syntax/help checks."
  fi
fi

shopt -s nullglob
local_camera_configs=(
  "${repo_dir}"/config/local/d435i-*/d435i_factory_imucam.yaml
  "${repo_dir}"/config/local/d435i-*/kalibr/kalibr_imucam_chain.yaml
)
for local_camera_config in "${local_camera_configs[@]}"; do
  legacy_count="$(
    grep -Ec '^[[:space:]]*T_cam_imu:[[:space:]]*' \
      "${local_camera_config}" || true
  )"
  openvins_count="$(
    grep -Ec '^[[:space:]]*T_imu_cam:[[:space:]]*' \
      "${local_camera_config}" || true
  )"
  if [[ "${legacy_count}" -ne 0 || "${openvins_count}" -ne 2 ]]; then
    fail "Local runtime camera config has the wrong OpenVINS transform contract: ${local_camera_config}"
  else
    pass "Local runtime camera config uses two T_imu_cam transforms: ${local_camera_config}"
  fi
done
shopt -u nullglob

main_cache="${repo_dir}/build/linux-release/CMakeCache.txt"
openvins_cache="${repo_dir}/.deps/build/open_vins/CMakeCache.txt"
main_source_fingerprint=""
current_source_fingerprint=""
if current_source_fingerprint="$(
     ovrs_current_source_fingerprint "${repo_dir}"
   )" &&
   [[ "${current_source_fingerprint}" =~ ^[0-9a-f]{64}$ ]]; then
  pass "Current project source-content fingerprint is valid."
else
  current_source_fingerprint=""
  fail "Could not calculate a valid project source-content fingerprint."
fi
if [[ -f "${main_cache}" ]]; then
  if ovrs_cmake_cache_value_equals \
       "${main_cache}" OVRS_CERES_PREFIX "${ceres_prefix}" &&
     ovrs_cmake_cache_value_equals \
       "${main_cache}" Ceres_DIR "${ceres_prefix}/lib/cmake/Ceres"; then
    pass "Main CMake cache resolves repository-local Ceres."
  else
    fail "Main CMake cache does not resolve the required local Ceres prefix."
  fi
  if ovrs_cmake_cache_value_equals \
       "${main_cache}" OVRS_PROJECT_VERSION_RESOLVED "${project_version}"; then
    pass "Main CMake cache matches project version ${project_version}."
  else
    fail "Main CMake cache project version does not match VERSION."
  fi
  if [[ -n "${current_source_fingerprint}" ]] &&
     main_source_fingerprint="$(
       ovrs_cmake_cache_value \
         "${main_cache}" OVRS_SOURCE_FINGERPRINT_RESOLVED
     )" &&
     [[ "${main_source_fingerprint}" == "${current_source_fingerprint}" ]]; then
    pass "Main CMake cache matches the current source-content fingerprint."
  else
    main_source_fingerprint=""
    fail "Main CMake cache is missing a valid source-content fingerprint; rebuild the project."
  fi
else
  if [[ "${require_build}" -eq 1 ]]; then
    fail "Release CMake cache is absent; run scripts/build_ubuntu.sh."
  else
    info "Release CMake cache is absent; run scripts/build_ubuntu.sh."
  fi
fi
if [[ -f "${openvins_cache}" ]]; then
  if ovrs_cmake_cache_value_equals \
       "${openvins_cache}" Ceres_DIR \
       "${ceres_prefix}/lib/cmake/Ceres" &&
     ovrs_cmake_cache_value_equals \
       "${openvins_cache}" ENABLE_ROS OFF &&
     ovrs_cmake_cache_value_equals \
       "${openvins_cache}" ENABLE_ARUCO_TAGS OFF &&
     ovrs_cmake_cache_value_equals \
       "${openvins_cache}" CMAKE_DISABLE_FIND_PACKAGE_catkin TRUE &&
     ovrs_cmake_cache_value_equals \
       "${openvins_cache}" CMAKE_DISABLE_FIND_PACKAGE_ament_cmake TRUE; then
    pass "OpenVINS cache uses local Ceres and disables ROS/ArUco integration."
  else
    fail "OpenVINS cache does not enforce local Ceres and the non-ROS policy."
  fi
elif [[ "${require_build}" -eq 1 ]]; then
  fail "OpenVINS CMake cache is absent; run scripts/build_ubuntu.sh."
fi

for executable in ovrs_inspect ovrs_record ovrs_live ovrs_replay; do
  executable_path="${repo_dir}/build/linux-release/${executable}"
  if [[ -x "${executable_path}" ]]; then
    if "${executable_path}" --help >/dev/null; then
      pass "${executable} --help"
    else
      fail "${executable} --help returned an error."
    fi
    if [[ -n "${main_source_fingerprint}" ]] &&
       ovrs_executable_build_identity_matches \
         "${executable_path}" "${executable}" "${project_version}" \
         "${main_source_fingerprint}"; then
      pass "${executable} --version/source fingerprint"
    elif [[ -z "${main_source_fingerprint}" ]] &&
         "${executable_path}" --version |
           grep -Fqx "${executable} ${project_version}"; then
      fail "${executable} reports the version but cannot be matched to current source content."
    else
      fail "${executable} is stale or does not match VERSION/source content."
    fi
  elif [[ "${require_build}" -eq 1 ]]; then
    fail "${executable} is missing; run scripts/build_ubuntu.sh."
  fi
done
if [[ "${require_build}" -eq 1 ]]; then
  ctest_listing="$(
    ctest --test-dir "${repo_dir}/build/linux-release" -N 2>&1 || true
  )"
  if ! grep -Fq "ovrs_core_tests" <<<"${ctest_listing}" ||
     ! grep -Fq "ovrs_mock_replay_test" <<<"${ctest_listing}"; then
    fail "The release build does not register both mandatory C++ tests."
    printf '%s\n' "${ctest_listing}" >&2
  elif command -v python3 >/dev/null 2>&1; then
    if grep -Fq "ovrs_calibration_scripts_test" <<<"${ctest_listing}"; then
      pass "Both C++ tests and the optional Python calibration test are present."
    else
      fail "Python is available but the calibration script test is not registered."
      printf '%s\n' "${ctest_listing}" >&2
    fi
  else
    pass "Both mandatory C++ tests are present."
    warn "Python is unavailable, so the optional calibration script test is not registered."
  fi
fi

camera_found=0
if command -v rs-enumerate-devices >/dev/null 2>&1; then
  camera_output="$(timeout 15 rs-enumerate-devices -s 2>&1 || true)"
  if grep -Eqi 'D435I|Intel RealSense D435I' <<<"${camera_output}" &&
     ! grep -Fqi "No device detected" <<<"${camera_output}"; then
    pass "A D435i is visible to librealsense."
    camera_found=1
  else
    info "rs-enumerate-devices did not report a D435i."
  fi
else
  info "rs-enumerate-devices is not installed."
fi
inspect_path="${repo_dir}/build/linux-release/ovrs_inspect"
if [[ -x "${inspect_path}" ]] &&
   [[ "${camera_found}" -eq 0 || "${require_camera}" -eq 1 ]]; then
  inspect_arguments=(
    --duration 1
    --stream-config "${camera_stream_config}"
  )
  if [[ -n "${camera_serial}" ]]; then
    inspect_arguments+=(--serial "${camera_serial}")
  fi
  if camera_output="$(
       timeout 15 "${inspect_path}" "${inspect_arguments[@]}" 2>&1
     )" &&
     grep -Eqi 'D435I|Intel RealSense D435I' <<<"${camera_output}" &&
     grep -Fq 'Timestamp monotonic/domain check: PASS' \
       <<<"${camera_output}"; then
    if [[ -n "${camera_serial}" ]]; then
      pass "D435i ${camera_serial} completed a one-second ovrs_inspect sample."
    else
      pass "A D435i completed a one-second ovrs_inspect sample."
    fi
    camera_found=1
  elif [[ "${require_camera}" -eq 1 ]]; then
    fail "D435i ${camera_serial} did not complete a one-second ovrs_inspect sample."
    printf '%s\n' "${camera_output}" >&2
  else
    info "ovrs_inspect did not complete a D435i sample."
  fi
elif [[ "${require_camera}" -eq 1 ]]; then
  fail "ovrs_inspect is unavailable; build the project before requiring a camera sample."
fi
if [[ "${camera_found}" -eq 0 ]]; then
  if [[ "${require_camera}" -eq 1 ]]; then
    fail "A physical D435i was required but not detected."
  else
    warn "No physical D435i was verified; rerun with --require-camera after connecting it."
  fi
fi

echo
echo "Preflight summary: errors=${errors} warnings=${warnings}"
if [[ "${errors}" -gt 0 ]]; then
  echo "PREFLIGHT_RESULT=FAIL"
  exit 1
fi
if [[ "${warnings}" -gt 0 ]]; then
  echo "PREFLIGHT_RESULT=PASS_WITH_WARNINGS"
else
  echo "PREFLIGHT_RESULT=PASS"
fi
