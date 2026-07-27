#!/usr/bin/env bash
set -euo pipefail

# Compare tracked content independently of Windows/Linux checkout conventions.
# Only executable-mode metadata and CR characters immediately before LF are
# ignored. Binary changes, other whitespace changes, and all substantive text
# changes remain visible and fail the check.
ovrs_git_tracked_content_is_clean() {
  local checkout_dir="$1"
  git -c core.fileMode=false -c core.autocrlf=false \
    -C "${checkout_dir}" diff --ignore-cr-at-eol --quiet -- &&
    git -c core.fileMode=false -c core.autocrlf=false \
      -C "${checkout_dir}" diff --cached --ignore-cr-at-eol --quiet --
}

ovrs_git_checkout_has_raw_tracked_differences() {
  local checkout_dir="$1"
  ! git -c core.autocrlf=false -C "${checkout_dir}" diff --quiet -- ||
    ! git -c core.autocrlf=false -C "${checkout_dir}" \
      diff --cached --quiet --
}

ovrs_git_print_tracked_content_changes() {
  local checkout_dir="$1"
  git -c core.fileMode=false -c core.autocrlf=false \
    -C "${checkout_dir}" diff --ignore-cr-at-eol --name-status -- >&2
  git -c core.fileMode=false -c core.autocrlf=false \
    -C "${checkout_dir}" diff --cached --ignore-cr-at-eol \
      --name-status -- >&2
}

ovrs_git_tracked_content_matches_patch() {
  local checkout_dir="$1"
  local patch_file="$2"
  local actual_diff
  local normalized_diff

  actual_diff="$(mktemp)"
  normalized_diff="${actual_diff}.normalized"
  if ! git -c core.fileMode=false -c core.autocrlf=false \
       -C "${checkout_dir}" diff --ignore-cr-at-eol --binary -- \
       >"${actual_diff}" ||
     ! sed 's/\r$//' "${actual_diff}" >"${normalized_diff}"; then
    rm -f "${actual_diff}" "${normalized_diff}"
    return 1
  fi

  if ! cmp -s "${patch_file}" "${normalized_diff}" ||
     ! git -c core.fileMode=false -c core.autocrlf=false \
       -C "${checkout_dir}" diff --cached --ignore-cr-at-eol --quiet --; then
    rm -f "${actual_diff}" "${normalized_diff}"
    return 1
  fi

  rm -f "${actual_diff}" "${normalized_diff}"
}
