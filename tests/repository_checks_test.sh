#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib/repository_checks.sh
source "${repo_dir}/scripts/lib/repository_checks.sh"

fixture_dir="$(mktemp -d)"
trap 'rm -rf "${fixture_dir}"' EXIT

git init -q "${fixture_dir}"
git -C "${fixture_dir}" config user.name "OVRS Test"
git -C "${fixture_dir}" config user.email "ovrs-test@example.invalid"

printf 'alpha\nbeta\n' >"${fixture_dir}/tracked.txt"
git -C "${fixture_dir}" add tracked.txt
git -C "${fixture_dir}" commit -q -m "fixture base"

printf 'alpha\ngamma\n' >"${fixture_dir}/tracked.txt"
git -C "${fixture_dir}" diff --binary -- >"${fixture_dir}/reviewed.patch"
sed -E \
  's/^index ([0-9a-f]+)\.\.[0-9a-f]+/index \1..0000000/' \
  "${fixture_dir}/reviewed.patch" >"${fixture_dir}/reviewed-stale-index.patch"

if ! ovrs_git_tracked_content_matches_patch \
     "${fixture_dir}" "${fixture_dir}/reviewed-stale-index.patch"; then
  echo "Equivalent patch content was rejected because of post-image metadata." >&2
  exit 1
fi

printf 'unexpected\n' >>"${fixture_dir}/tracked.txt"
if ovrs_git_tracked_content_matches_patch \
     "${fixture_dir}" "${fixture_dir}/reviewed-stale-index.patch"; then
  echo "Unexpected tracked content was accepted as the reviewed patch." >&2
  exit 1
fi

printf 'repository_checks_test passed\n'
