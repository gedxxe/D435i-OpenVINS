#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "${BASH_SOURCE[0]%/*}/.." && pwd)"
bundle=""
stream_config=""

usage() {
  printf '%s\n' \
    "Usage: upgrade_local_bundle_v040.sh --bundle DIR --stream-config FILE" \
    "" \
    "Adds the v0.4.0 identity and RealSense motion-correction provenance" \
    "keys to an existing ignored local IMU YAML. Numeric calibration is not" \
    "changed. Identity is copied from the bundle's main bootstrap YAML." \
    "The original is retained as d435i_factory_imu.pre-v0.4.0.yaml."
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --bundle)
      [[ "$#" -ge 2 && -n "$2" ]] || {
        echo "--bundle requires a directory." >&2
        exit 2
      }
      bundle="$2"
      shift 2
      ;;
    --stream-config)
      [[ "$#" -ge 2 && -n "$2" ]] || {
        echo "--stream-config requires a file." >&2
        exit 2
      }
      stream_config="$2"
      shift 2
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
done

for command_name in grep sed awk cp mktemp mv rm; do
  command -v "${command_name}" >/dev/null 2>&1 || {
    echo "Missing required command: ${command_name}" >&2
    exit 2
  }
done

[[ -d "${bundle}" ]] || {
  echo "--bundle must name an existing local bundle." >&2
  exit 2
}
[[ -f "${stream_config}" && -r "${stream_config}" ]] || {
  echo "--stream-config must name a readable file." >&2
  exit 2
}

bundle_absolute="$(cd "${bundle}" && pwd -P)"
local_absolute="$(cd "${repo_dir}/config/local" && pwd -P)"
case "${bundle_absolute}" in
  "${local_absolute}"/*) ;;
  *)
    echo "Refusing to modify a bundle outside config/local." >&2
    exit 2
    ;;
esac

imu_file="${bundle_absolute}/d435i_factory_imu.yaml"
[[ -f "${imu_file}" && -r "${imu_file}" ]] || {
  echo "Bundle has no readable d435i_factory_imu.yaml." >&2
  exit 2
}
main_file="${bundle_absolute}/bootstrap.yaml"
[[ -f "${main_file}" && -r "${main_file}" ]] || {
  echo "Bundle has no readable bootstrap.yaml." >&2
  exit 2
}

main_state_count="$(
  grep -Ec \
    '^[[:space:]]*calibration_state:[[:space:]]*"?((BOOTSTRAP_UNVERIFIED)|(KALIBR_VERIFIED))"?[[:space:]]*$' \
    "${main_file}" || true
)"
main_serial_count="$(
  grep -Ec \
    '^[[:space:]]*calibrated_serial:[[:space:]]*"?[0-9]+"?[[:space:]]*$' \
    "${main_file}" || true
)"
[[ "${main_state_count}" -eq 1 && "${main_serial_count}" -eq 1 ]] || {
  echo "Main YAML needs exactly one valid calibration state and numeric serial." >&2
  exit 2
}
calibration_state="$(
  sed -n \
    's/^[[:space:]]*calibration_state:[[:space:]]*"\{0,1\}\(BOOTSTRAP_UNVERIFIED\|KALIBR_VERIFIED\)"\{0,1\}[[:space:]]*$/\1/p' \
    "${main_file}"
)"
calibrated_serial="$(
  sed -n \
    's/^[[:space:]]*calibrated_serial:[[:space:]]*"\{0,1\}\([0-9][0-9]*\)"\{0,1\}[[:space:]]*$/\1/p' \
    "${main_file}"
)"

stream_count="$(
  grep -Ec \
    '^[[:space:]]*motion_correction_enabled:[[:space:]]*(true|false)[[:space:]]*$' \
    "${stream_config}" || true
)"
[[ "${stream_count}" -eq 1 ]] || {
  echo "Stream file needs exactly one boolean motion_correction_enabled." >&2
  exit 2
}
policy="$(
  sed -n \
    's/^[[:space:]]*motion_correction_enabled:[[:space:]]*\(true\|false\)[[:space:]]*$/\1/p' \
    "${stream_config}"
)"

existing_count="$(
  grep -Ec \
    '^[[:space:]]*realsense_motion_correction_enabled:[[:space:]]*(true|false)[[:space:]]*$' \
    "${imu_file}" || true
)"
existing=""
if [[ "${existing_count}" -eq 1 ]]; then
  existing="$(
    sed -n \
      's/^[[:space:]]*realsense_motion_correction_enabled:[[:space:]]*\(true\|false\)[[:space:]]*$/\1/p' \
      "${imu_file}"
  )"
  [[ "${existing}" == "${policy}" ]] || {
    echo "Existing IMU policy ${existing} conflicts with stream policy ${policy}." >&2
    exit 2
  }
elif [[ "${existing_count}" -ne 0 ]]; then
  echo "IMU YAML has duplicate motion-correction policy keys." >&2
  exit 2
fi

imu_state_count="$(
  grep -Ec '^[[:space:]]*calibration_state:' "${imu_file}" || true
)"
imu_serial_count="$(
  grep -Ec '^[[:space:]]*calibrated_serial:' "${imu_file}" || true
)"
[[ "${imu_state_count}" -le 1 && "${imu_serial_count}" -le 1 ]] || {
  echo "IMU YAML has duplicate calibration identity keys." >&2
  exit 2
}
if [[ "${imu_state_count}" -eq 1 ]]; then
  imu_state="$(
    sed -n \
      's/^[[:space:]]*calibration_state:[[:space:]]*"\{0,1\}\([^"]*\)"\{0,1\}[[:space:]]*$/\1/p' \
      "${imu_file}"
  )"
  [[ "${imu_state}" == "${calibration_state}" ]] || {
    echo "IMU calibration state conflicts with the main YAML." >&2
    exit 2
  }
fi
if [[ "${imu_serial_count}" -eq 1 ]]; then
  imu_serial="$(
    sed -n \
      's/^[[:space:]]*calibrated_serial:[[:space:]]*"\{0,1\}\([0-9][0-9]*\)"\{0,1\}[[:space:]]*$/\1/p' \
      "${imu_file}"
  )"
  [[ "${imu_serial}" == "${calibrated_serial}" ]] || {
    echo "IMU calibrated serial conflicts with the main YAML." >&2
    exit 2
  }
fi

if [[ "${existing_count}" -eq 1 &&
      "${imu_state_count}" -eq 1 &&
      "${imu_serial_count}" -eq 1 ]]; then
  echo "Local bundle already declares matching v0.4.0 identity and motion policy."
  exit 0
fi

[[ "$(grep -Ec '^[[:space:]]*imu0:[[:space:]]*$' "${imu_file}")" -eq 1 ]] || {
  echo "IMU YAML must contain exactly one imu0 block." >&2
  exit 2
}

backup="${bundle_absolute}/d435i_factory_imu.pre-v0.4.0.yaml"
[[ ! -e "${backup}" ]] || {
  echo "Backup already exists; refusing another migration: ${backup}" >&2
  exit 2
}
temporary="$(mktemp "${bundle_absolute}/.imu-v040.XXXXXX")"
cleanup() {
  [[ ! -e "${temporary:-}" ]] || rm -f -- "${temporary}"
}
trap cleanup EXIT

awk -v add_state="$((1 - imu_state_count))" \
    -v add_serial="$((1 - imu_serial_count))" \
    -v add_motion="$((1 - existing_count))" \
    -v state="${calibration_state}" \
    -v serial="${calibrated_serial}" \
    -v policy="${policy}" '
  /^[[:space:]]*imu0:[[:space:]]*$/ {
    if (add_state == 1) {
      print "calibration_state: \"" state "\""
    }
    if (add_serial == 1) {
      print "calibrated_serial: \"" serial "\""
    }
    if (add_state == 1 || add_serial == 1) {
      print ""
    }
    print
    if (add_motion == 1) {
      print "  realsense_motion_correction_enabled: " policy
    }
    next
  }
  { print }
' "${imu_file}" >"${temporary}"
grep -Fq "calibration_state: \"${calibration_state}\"" "${temporary}" &&
grep -Fq "calibrated_serial: \"${calibrated_serial}\"" "${temporary}" &&
grep -Fq "  realsense_motion_correction_enabled: ${policy}" "${temporary}" || {
  echo "Could not create migrated IMU YAML." >&2
  exit 3
}
cp -p -- "${imu_file}" "${backup}"
mv -- "${temporary}" "${imu_file}"
temporary=""
trap - EXIT

echo "Added missing identity/motion provenance; numeric IMU values are unchanged."
echo "Backup: ${backup}"
echo "Continue with the current root README and record a new dataset; legacy datasets remain intentionally ambiguous."
