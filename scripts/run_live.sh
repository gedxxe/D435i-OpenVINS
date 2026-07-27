#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
estimator_config="${OVRS_ESTIMATOR_CONFIG:-}"
if [[ "$#" -gt 0 && "$1" != -* ]]; then
  estimator_config="$1"
  shift
fi
if [[ -z "${estimator_config}" ]]; then
  echo "Usage: $0 ESTIMATOR_CONFIG [additional ovrs_live options]" >&2
  echo "Set OVRS_ESTIMATOR_CONFIG instead of the positional path if preferred." >&2
  exit 2
fi
exec "${repo_dir}/build/linux-release/ovrs_live" \
  --config "${estimator_config}" \
  --stream-config \
  "${OVRS_STREAM_CONFIG:-${repo_dir}/config/sensors/realsense_streams.yaml}" \
  "$@"
