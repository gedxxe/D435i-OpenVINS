#!/usr/bin/env bash
set -euo pipefail

# CMake preserves command-line -D values as UNINITIALIZED until a project
# promotes their type. Compare the key's value without assuming PATH, BOOL,
# STRING, or UNINITIALIZED serialization in CMakeCache.txt.
ovrs_cmake_cache_value() {
  local cache_file="$1"
  local key="$2"
  local line
  [[ -r "${cache_file}" ]] || return 1
  while IFS= read -r line; do
    if [[ "${line}" == "${key}:"*"="* ]]; then
      printf '%s\n' "${line#*=}"
      return 0
    fi
  done <"${cache_file}"
  return 1
}

ovrs_cmake_cache_value_equals() {
  local cache_file="$1"
  local key="$2"
  local expected_value="$3"
  local actual_value
  actual_value="$(ovrs_cmake_cache_value "${cache_file}" "${key}")" || return 1
  [[ "${actual_value}" == "${expected_value}" ]]
}

ovrs_executable_build_identity_matches() {
  local executable_path="$1"
  local executable_name="$2"
  local project_version="$3"
  local source_fingerprint="$4"
  local version_output
  [[ -x "${executable_path}" ]] || return 1
  version_output="$("${executable_path}" --version 2>&1)" || return 1
  grep -Fqx "${executable_name} ${project_version}" <<<"${version_output}" &&
    grep -Fqx "Source fingerprint ${source_fingerprint}" \
      <<<"${version_output}"
}

ovrs_current_source_fingerprint() {
  local repo_dir="$1"
  cmake "-DOVRS_SOURCE_DIR=${repo_dir}" \
    -P "${repo_dir}/cmake/SourceFingerprint.cmake"
}
