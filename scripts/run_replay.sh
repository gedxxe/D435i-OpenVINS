#!/usr/bin/env bash
set -euo pipefail
if [[ "$#" -lt 1 ]]; then
  echo "Usage: $0 DATASET ESTIMATOR_CONFIG [additional ovrs_replay options]" >&2
  exit 2
fi
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dataset="$1"
shift
estimator_config="${OVRS_ESTIMATOR_CONFIG:-}"
if [[ -z "${estimator_config}" && "$#" -gt 0 && "$1" != -* ]]; then
  estimator_config="$1"
  shift
fi
if [[ -z "${estimator_config}" ]]; then
  echo "A reviewed serial-specific ESTIMATOR_CONFIG is required." >&2
  echo "Usage: $0 DATASET ESTIMATOR_CONFIG [additional ovrs_replay options]" >&2
  exit 2
fi
exec "${repo_dir}/build/linux-release/ovrs_replay" \
  --dataset "${dataset}" \
  --config "${estimator_config}" \
  "$@"
