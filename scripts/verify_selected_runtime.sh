#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
selected_serial=""

usage() {
  echo "Usage: $0 [--serial SERIAL]"
  echo "Verify the selected runtime bundle for one D435i serial."
}

while (($# > 0)); do
  case "$1" in
  --serial)
    if (($# < 2)); then
      echo "--serial requires a value." >&2
      exit 2
    fi
    selected_serial="$2"
    shift 2
    ;;
  -h | --help)
    usage
    exit 0
    ;;
  *)
    echo "Unknown argument: $1" >&2
    usage >&2
    exit 2
    ;;
  esac
done

if [[ -z "${selected_serial}" ]]; then
  shopt -s nullglob
  candidate_configs=(
    "${repo_dir}"/config/local/d435i-*/selected_runtime/estimator.yaml
  )
  shopt -u nullglob

  if ((${#candidate_configs[@]} != 1)); then
    echo "Expected exactly one selected runtime; pass --serial SERIAL." >&2
    exit 2
  fi

  device_dir="$(basename "$(dirname "$(dirname "${candidate_configs[0]}")")")"
  selected_serial="${device_dir#d435i-}"
fi

if [[ ! "${selected_serial}" =~ ^[0-9]+$ ]]; then
  echo "D435i serial must contain digits only." >&2
  exit 2
fi

selected_dir="${repo_dir}/config/local/d435i-${selected_serial}/selected_runtime"
estimator_config="${selected_dir}/estimator.yaml"
stream_config="${repo_dir}/config/sensors/realsense_streams_vio_90hz.yaml"
openvins_patch="${repo_dir}/patches/openvins-zupt-velocity-constraint.patch"

if [[ ! -f "${estimator_config}" ]]; then
  echo "Selected runtime not found for D435i ${selected_serial}." >&2
  exit 1
fi

printf '%s  %s\n' \
  "be37da3454190ba262a204afa709ee58d034784814e3a7c09fb629be02479867" \
  "${estimator_config}" \
  "c23713d7830e2d76e7d281edb0f8decb192a7f740ef15af0927d38d2816fa830" \
  "${selected_dir}/post_rs_imu_candidate_a_imu.yaml" \
  "0e911d87f1d2f508de1e9504354272220a999e76d821c9e4dc6b3a6fd3006f4f" \
  "${selected_dir}/post_rs_imu_candidate_a_imucam.yaml" \
  "cc0cf24730f056dcd6af1d2eebcac21bc3c6e3266e752a17b0d86e61fa8cff03" \
  "${stream_config}" \
  "000c826231727ee10cf240d89469e956e44028ddaff5cf3ccb2c744b92368d37" \
  "${openvins_patch}" |
  sha256sum --check

calibrated_serial="$(
  sed -n \
    's/^[[:space:]]*calibrated_serial:[[:space:]]*"\{0,1\}\([0-9][0-9]*\)"\{0,1\}[[:space:]]*$/\1/p' \
    "${estimator_config}"
)"
if [[ "${calibrated_serial}" != "${selected_serial}" ]]; then
  echo "Selected estimator serial does not match ${selected_serial}." >&2
  exit 1
fi

printf 'Selected runtime integrity: PASS (D435i %s)\n' "${selected_serial}"
