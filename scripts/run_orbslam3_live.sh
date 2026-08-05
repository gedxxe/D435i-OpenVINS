#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "${BASH_SOURCE[0]%/*}/.." && pwd)"
serial=""
stream_config="${repo_dir}/config/sensors/realsense_streams_vio_90hz.yaml"
camera_stride=3
viewer_flag="--viewer"
allow_unverified=0
run_id="$(date -u +%Y%m%dT%H%M%SZ)"

usage() {
  cat <<'EOF'
Usage: run_orbslam3_live.sh --serial SERIAL [options]

Prepare, run, and independently evaluate one standalone D435i ORB-SLAM3
stereo-inertial attempt. This script keeps strict shell options inside the
child process, so a failed SLAM gate cannot enable errexit in the caller's
interactive terminal.

Options:
  --serial SERIAL                   Required numeric D435i serial.
  --allow-unverified-calibration    Explicitly acknowledge bootstrap calibration.
  --headless                        Disable the Pangolin viewer.
  --camera-stride COUNT             Source-frame stride (default: 3).
  --stream-config PATH              Stream YAML override.
  --run-id ID                       Safe output suffix override.
  -h, --help                        Show this help.
EOF
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --serial)
      [[ "$#" -ge 2 ]] || { echo "--serial requires a value" >&2; exit 2; }
      serial="$2"
      shift
      ;;
    --allow-unverified-calibration)
      allow_unverified=1
      ;;
    --headless)
      viewer_flag="--headless"
      ;;
    --camera-stride)
      [[ "$#" -ge 2 ]] || { echo "--camera-stride requires a value" >&2; exit 2; }
      camera_stride="$2"
      shift
      ;;
    --stream-config)
      [[ "$#" -ge 2 ]] || { echo "--stream-config requires a value" >&2; exit 2; }
      stream_config="$2"
      shift
      ;;
    --run-id)
      [[ "$#" -ge 2 ]] || { echo "--run-id requires a value" >&2; exit 2; }
      run_id="$2"
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

if [[ ! "${serial}" =~ ^[0-9]+$ ]]; then
  echo "--serial must be a numeric D435i serial" >&2
  exit 2
fi
if [[ ! "${camera_stride}" =~ ^[1-9][0-9]*$ ]]; then
  echo "--camera-stride must be a positive integer" >&2
  exit 2
fi
if [[ ! "${run_id}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$ ]]; then
  echo "--run-id must contain 1-64 safe filename characters" >&2
  exit 2
fi
if [[ ! -r "${stream_config}" ]]; then
  echo "--stream-config must name a readable file: ${stream_config}" >&2
  exit 2
fi

selected_dir="${repo_dir}/config/local/d435i-${serial}/selected_runtime"
estimator_config="${selected_dir}/estimator.yaml"
live_bundle="${repo_dir}/runs/orbslam3_live_config_${run_id}"
orb_run="${repo_dir}/runs/orbslam3_live_motion_${run_id}"
live_executable="${repo_dir}/build/linux-release/ovrs_orbslam3_live"
vocabulary="${repo_dir}/.deps/src/orb_slam3/Vocabulary/ORBvoc.txt"
backend_library="${repo_dir}/.deps/src/orb_slam3/lib/libORB_SLAM3.so"
backend_pin="${repo_dir}/config/research/orbslam3_backend.yaml"
backend_patch="${repo_dir}/patches/orbslam3-atlas-serialization-integrity.patch"

if [[ ! -r "${estimator_config}" ]]; then
  echo "Selected runtime is missing for serial ${serial}: ${estimator_config}" >&2
  exit 2
fi
if [[ "${allow_unverified}" -ne 1 ]]; then
  echo "This research path requires explicit --allow-unverified-calibration." >&2
  exit 2
fi
for required in "${live_executable}" "${vocabulary}" "${backend_library}" \
                "${backend_pin}" "${backend_patch}"; do
  if [[ ! -r "${required}" ]]; then
    echo "Required ORB-SLAM3 artifact is missing: ${required}" >&2
    exit 2
  fi
done
if [[ -e "${live_bundle}" || -e "${orb_run}" ]]; then
  echo "Refusing to overwrite an existing run or bundle for ${run_id}" >&2
  exit 2
fi

echo "Live bundle: ${live_bundle}"
echo "Run output:  ${orb_run}"
"${repo_dir}/scripts/preflight_ubuntu.sh" \
  --require-camera \
  --serial "${serial}" \
  --stream-config "${stream_config}"

python3 "${repo_dir}/scripts/prepare_orbslam3_live.py" \
  --estimator-config "${estimator_config}" \
  --stream-config "${stream_config}" \
  --output "${live_bundle}" \
  --camera-stride "${camera_stride}"

live_status=0
if "${live_executable}" \
  --settings "${live_bundle}/orbslam3_live_settings.yaml" \
  --live-bundle-manifest "${live_bundle}/live_manifest.yaml" \
  --vocabulary "${vocabulary}" \
  --config "${estimator_config}" \
  --stream-config "${stream_config}" \
  --serial "${serial}" \
  "${viewer_flag}" \
  --allow-unverified-calibration \
  --output "${orb_run}"; then
  live_status=0
else
  live_status=$?
fi

evaluation_status=0
if [[ -r "${orb_run}/run_summary.yaml" ]]; then
  if python3 "${repo_dir}/scripts/evaluate_orbslam3_live_run.py" \
    --run-dir "${orb_run}" \
    --live-bundle-manifest "${live_bundle}/live_manifest.yaml" \
    --backend-pin "${backend_pin}" \
    --backend-patch "${backend_patch}" \
    --backend-library "${backend_library}" \
    --live-executable "${live_executable}" \
    --vocabulary "${vocabulary}" \
    --output "${orb_run}/evaluation.yaml"; then
    evaluation_status=0
  else
    evaluation_status=$?
  fi
else
  echo "No terminal run_summary.yaml exists; independent evaluation skipped." >&2
  evaluation_status=1
fi

echo "Live exit status: ${live_status}"
echo "Evaluation exit status: ${evaluation_status}"
echo "Run output retained at: ${orb_run}"
if [[ "${live_status}" -ne 0 ]]; then
  exit "${live_status}"
fi
exit "${evaluation_status}"
