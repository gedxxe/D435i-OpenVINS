#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "${BASH_SOURCE[0]%/*}/.." && pwd)"
calibration_path=""
expected_serial=""
stream_config_path=""
acknowledged=0
validate_only=0

for required_command in grep sed awk install chmod mkdir mktemp mv rm rmdir; do
  if ! command -v "${required_command}" >/dev/null 2>&1; then
    echo "Missing required command: ${required_command}" >&2
    exit 2
  fi
done

usage() {
  printf '%s\n' \
    "Usage:" \
    "  prepare_bootstrap_config.sh \\" \
    "    --calibration FACTORY_EXPORT.yaml \\" \
    "    --expected-serial SERIAL --stream-config STREAMS.yaml \\" \
    "    --acknowledge-reviewed-factory-export" \
    "  prepare_bootstrap_config.sh --calibration FACTORY_EXPORT.yaml \\" \
    "    --expected-serial SERIAL --stream-config STREAMS.yaml --validate-only" \
    "" \
    "Validates a reviewed ovrs_inspect factory export and creates an ignored," \
    "serial-specific BOOTSTRAP_UNVERIFIED configuration under config/local/." \
    "It does not perform Kalibr calibration or certify estimator accuracy."
}

while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --calibration)
      if [[ "$#" -lt 2 || -z "$2" ]]; then
        echo "--calibration requires a path." >&2
        exit 2
      fi
      calibration_path="$2"
      shift 2
      ;;
    --expected-serial)
      if [[ "$#" -lt 2 || -z "$2" ]]; then
        echo "--expected-serial requires a value." >&2
        exit 2
      fi
      expected_serial="$2"
      shift 2
      ;;
    --stream-config)
      if [[ "$#" -lt 2 || -z "$2" ]]; then
        echo "--stream-config requires a path." >&2
        exit 2
      fi
      stream_config_path="$2"
      shift 2
      ;;
    --acknowledge-reviewed-factory-export)
      acknowledged=1
      shift
      ;;
    --validate-only)
      validate_only=1
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
done

if [[ -z "${calibration_path}" || ! -f "${calibration_path}" ||
      ! -r "${calibration_path}" ]]; then
  echo "--calibration must name a readable factory-export YAML file." >&2
  exit 2
fi
if [[ ! "${expected_serial}" =~ ^[0-9]+$ ]]; then
  echo "--expected-serial must be the numeric D435i serial." >&2
  exit 2
fi
if [[ -z "${stream_config_path}" || ! -f "${stream_config_path}" ||
      ! -r "${stream_config_path}" ]]; then
  echo "--stream-config must name the readable stream configuration used for capture." >&2
  exit 2
fi
if [[ "${acknowledged}" -eq 1 && "${validate_only}" -eq 1 ]]; then
  echo "Choose either --validate-only or --acknowledge-reviewed-factory-export." >&2
  exit 2
fi
if grep -Fq "INVALID PLACEHOLDER" "${calibration_path}"; then
  echo "Calibration still contains an invalid placeholder." >&2
  exit 2
fi
if grep -Eqi '(^|[^[:alnum:]_])(nan|[+-]?inf(inity)?)([^[:alnum:]_]|$)' \
     "${calibration_path}"; then
  echo "Calibration contains a nonfinite numeric token." >&2
  exit 2
fi

calibration_state="$(
  sed -n 's/^[[:space:]]*calibration_state:[[:space:]]*"\([^"]*\)".*/\1/p' \
    "${calibration_path}"
)"
serial="$(
  sed -n 's/^[[:space:]]*calibrated_serial:[[:space:]]*"\([^"]*\)".*/\1/p' \
    "${calibration_path}"
)"
if [[ "${calibration_state}" != "BOOTSTRAP_UNVERIFIED" ]]; then
  echo "Factory export must declare calibration_state BOOTSTRAP_UNVERIFIED." >&2
  exit 2
fi
if [[ ! "${serial}" =~ ^[0-9]+$ ]]; then
  echo "Factory export must contain one numeric calibrated_serial." >&2
  exit 2
fi
if [[ "${serial}" != "${expected_serial}" ]]; then
  echo "Factory export serial ${serial} does not match expected serial ${expected_serial}." >&2
  exit 2
fi

require_key_count() {
  local key="$1"
  local expected="$2"
  local actual
  actual="$(
    grep -Ec "^[[:space:]]*${key}:[[:space:]]*" \
      "${calibration_path}" || true
  )"
  if [[ "${actual}" -ne "${expected}" ]]; then
    echo "Factory export requires ${expected} '${key}' entries; found ${actual}." >&2
    exit 2
  fi
}

require_key_count calibration_state 1
require_key_count calibrated_serial 1
require_key_count T_gyro_accel 1
require_key_count cam0 1
require_key_count cam1 1
require_key_count T_imu_cam 2
require_key_count T_cam_imu 0
require_key_count intrinsics 2
require_key_count realsense_distortion_model 2
require_key_count realsense_distortion_coeffs 2
require_key_count distortion_model 2
require_key_count distortion_coeffs 2
require_key_count resolution 2
require_key_count timeshift_cam_imu 2

mapfile -t resolutions < <(
  sed -n \
    's/^[[:space:]]*resolution:[[:space:]]*\[\([0-9][0-9]*\),[[:space:]]*\([0-9][0-9]*\)\][[:space:]]*$/\1x\2/p' \
    "${calibration_path}"
)
if [[ "${#resolutions[@]}" -ne 2 ||
      ! "${resolutions[0]}" =~ ^[1-9][0-9]*x[1-9][0-9]*$ ||
      "${resolutions[0]}" != "${resolutions[1]}" ]]; then
  echo "Both camera resolutions must be identical positive integer pairs." >&2
  exit 2
fi

stream_key() {
  local key="$1"
  local count
  count="$(
    grep -Ec "^[[:space:]]*${key}:[[:space:]]*" \
      "${stream_config_path}" || true
  )"
  if [[ "${count}" -ne 1 ]]; then
    echo "Stream configuration requires exactly one '${key}' entry; found ${count}." >&2
    exit 2
  fi
  sed -n "s/^[[:space:]]*${key}:[[:space:]]*\\([0-9][0-9]*\\)[[:space:]]*$/\\1/p" \
    "${stream_config_path}"
}

stream_width="$(stream_key width)"
stream_height="$(stream_key height)"
if [[ ! "${stream_width}" =~ ^[1-9][0-9]*$ ||
      ! "${stream_height}" =~ ^[1-9][0-9]*$ ]]; then
  echo "Stream width and height must be positive integers." >&2
  exit 2
fi
expected_resolution="${stream_width}x${stream_height}"
if [[ "${resolutions[0]}" != "${expected_resolution}" ]]; then
  echo "Factory export resolution ${resolutions[0]} does not match stream configuration ${expected_resolution}." >&2
  exit 2
fi

motion_correction_count="$(
  grep -Ec \
    '^[[:space:]]*motion_correction_enabled:[[:space:]]*(true|false)[[:space:]]*$' \
    "${stream_config_path}" || true
)"
if [[ "${motion_correction_count}" -ne 1 ]]; then
  echo "Stream configuration requires exactly one boolean motion_correction_enabled entry." >&2
  exit 2
fi
stream_motion_correction="$(
  sed -n \
    's/^[[:space:]]*motion_correction_enabled:[[:space:]]*\(true\|false\)[[:space:]]*$/\1/p' \
    "${stream_config_path}"
)"

mapfile -t realsense_models < <(
  sed -n \
    's/^[[:space:]]*realsense_distortion_model:[[:space:]]*//p' \
    "${calibration_path}" |
    sed 's/[[:space:]]*$//; s/^"//; s/"$//'
)
mapfile -t camera_models < <(
  sed -n 's/^[[:space:]]*camera_model:[[:space:]]*//p' \
    "${calibration_path}" |
    sed 's/[[:space:]]*$//; s/^"//; s/"$//'
)
mapfile -t openvins_models < <(
  sed -n 's/^[[:space:]]*distortion_model:[[:space:]]*//p' \
    "${calibration_path}" |
    sed 's/[[:space:]]*$//; s/^"//; s/"$//'
)
mapfile -t realsense_coefficients < <(
  sed -n \
    's/^[[:space:]]*realsense_distortion_coeffs:[[:space:]]*\[\(.*\)\][[:space:]]*$/\1/p' \
    "${calibration_path}"
)
mapfile -t openvins_coefficients < <(
  sed -n \
    's/^[[:space:]]*distortion_coeffs:[[:space:]]*\[\(.*\)\][[:space:]]*$/\1/p' \
    "${calibration_path}"
)
mapfile -t intrinsic_values < <(
  sed -n \
    's/^[[:space:]]*intrinsics:[[:space:]]*\[\(.*\)\][[:space:]]*$/\1/p' \
    "${calibration_path}"
)
if [[ "${#realsense_models[@]}" -ne 2 ||
      "${#camera_models[@]}" -ne 2 ||
      "${#openvins_models[@]}" -ne 2 ||
      "${#realsense_coefficients[@]}" -ne 2 ||
      "${#openvins_coefficients[@]}" -ne 2 ||
      "${#intrinsic_values[@]}" -ne 2 ]]; then
  echo "Factory export camera-model fields are malformed." >&2
  exit 2
fi
for camera_id in 0 1; do
  if [[ "${camera_models[${camera_id}]}" != "pinhole" ||
        "${openvins_models[${camera_id}]}" != "radtan" ]]; then
    echo "Factory export cam${camera_id} is not a pinhole/radtan mapping." >&2
    exit 2
  fi
  if [[ "${realsense_models[${camera_id}]}" != "Brown Conrady" &&
        "${realsense_models[${camera_id}]}" != "None" ]]; then
    echo "Factory export cam${camera_id} has an unsupported RealSense distortion model." >&2
    exit 2
  fi
  if ! awk \
      -v rs="${realsense_coefficients[${camera_id}]}" \
      -v ov="${openvins_coefficients[${camera_id}]}" \
      -v intrinsics="${intrinsic_values[${camera_id}]}" \
      -v rs_model="${realsense_models[${camera_id}]}" \
      -v width="${stream_width}" -v height="${stream_height}" '
        function trim(value) {
          gsub(/^[[:space:]]+|[[:space:]]+$/, "", value)
          return value
        }
        function numeric(value) {
          return value ~ /^[+-]?(([0-9]+([.][0-9]*)?)|([.][0-9]+))([eE][+-]?[0-9]+)?$/
        }
        function absolute(value) {
          return value < 0 ? -value : value
        }
        BEGIN {
          rs_count = split(rs, rs_values, ",")
          ov_count = split(ov, ov_values, ",")
          in_count = split(intrinsics, in_values, ",")
          if (rs_count != 5 || ov_count != 4 || in_count != 4) {
            exit 1
          }
          for (item = 1; item <= rs_count; ++item) {
            rs_values[item] = trim(rs_values[item])
            if (!numeric(rs_values[item])) {
              exit 1
            }
          }
          for (item = 1; item <= ov_count; ++item) {
            ov_values[item] = trim(ov_values[item])
            if (!numeric(ov_values[item]) || absolute((rs_values[item] + 0) - (ov_values[item] + 0)) > 1e-12) {
              exit 1
            }
          }
          for (item = 1; item <= in_count; ++item) {
            in_values[item] = trim(in_values[item])
            if (!numeric(in_values[item])) {
              exit 1
            }
          }
          if (absolute(rs_values[5] + 0) > 1e-12 ||
              in_values[1] + 0 <= 0 || in_values[2] + 0 <= 0 ||
              in_values[3] + 0 < 0 || in_values[3] + 0 >= width ||
              in_values[4] + 0 < 0 || in_values[4] + 0 >= height) {
            exit 1
          }
          if (rs_model == "None") {
            for (item = 1; item <= rs_count; ++item) {
              if (absolute(rs_values[item] + 0) > 1e-12) {
                exit 1
              }
            }
          }
        }
      '; then
    echo "Factory export cam${camera_id} has incompatible coefficients or intrinsics." >&2
    exit 2
  fi
done
if [[ "${validate_only}" -eq 1 ]]; then
  echo "Factory export structure is valid for serial ${serial}, resolution ${resolutions[0]}."
  echo "No local configuration was created."
  exit 0
fi
if [[ "${acknowledged}" -ne 1 ]]; then
  echo "Structure is valid. Review serial, resolution, distortion mapping," >&2
  echo "transforms, baseline, and the zero factory time offset before" >&2
  echo "using --acknowledge-reviewed-factory-export." >&2
  exit 2
fi

template="${repo_dir}/config/sensors/d435i_bootstrap.yaml"
imu_template="${repo_dir}/config/sensors/d435i_factory_imu.yaml"
imu_motion_correction="$(
  sed -n \
    's/^[[:space:]]*realsense_motion_correction_enabled:[[:space:]]*\(true\|false\)[[:space:]]*$/\1/p' \
    "${imu_template}"
)"
if [[ -z "${imu_motion_correction}" ||
      "${imu_motion_correction}" != "${stream_motion_correction}" ]]; then
  echo "Stream and bootstrap IMU motion-correction policies do not match." >&2
  exit 2
fi
placeholder='calibrated_serial: "REPLACE_WITH_DEVICE_SERIAL"'
if [[ "$(grep -Fc "${placeholder}" "${template}")" -ne 1 ]]; then
  echo "Bootstrap template no longer has exactly one serial placeholder." >&2
  exit 3
fi
if [[ "$(grep -Fc "${placeholder}" "${imu_template}")" -ne 1 ]]; then
  echo "Bootstrap IMU template no longer has exactly one serial placeholder." >&2
  exit 3
fi

local_root="${repo_dir}/config/local"
destination="${local_root}/d435i-${serial}"
if [[ -e "${destination}" ]]; then
  echo "Local configuration already exists; refusing to overwrite: ${destination}" >&2
  exit 2
fi
mkdir -p "${local_root}"
temporary="$(mktemp -d "${local_root}/.prepare.XXXXXX")"

cleanup() {
  if [[ -n "${temporary:-}" && -d "${temporary}" ]]; then
    case "${temporary}" in
      "${local_root}"/.prepare.*)
        rm -f -- "${temporary}/bootstrap.yaml" \
          "${temporary}/d435i_factory_imu.yaml" \
          "${temporary}/d435i_factory_imucam.yaml"
        rmdir -- "${temporary}" 2>/dev/null || true
        ;;
    esac
  fi
}
trap cleanup EXIT

sed "s/${placeholder}/calibrated_serial: \"${serial}\"/" \
  "${template}" >"${temporary}/bootstrap.yaml"
sed "s/${placeholder}/calibrated_serial: \"${serial}\"/" \
  "${imu_template}" >"${temporary}/d435i_factory_imu.yaml"
chmod 0644 "${temporary}/d435i_factory_imu.yaml"
install -m 0644 "${calibration_path}" \
  "${temporary}/d435i_factory_imucam.yaml"
mv -T -- "${temporary}" "${destination}"
temporary=""
trap - EXIT

relative_destination="config/local/d435i-${serial}"
echo "Prepared BOOTSTRAP_UNVERIFIED configuration:"
echo "  ${relative_destination}/bootstrap.yaml"
echo "Use it only for careful handheld replay/live validation."
echo "IMU noise/intrinsic values remain conservative unmeasured placeholders."
