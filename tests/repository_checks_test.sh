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

if grep -R -n -E '/(home|media)/[^/[:space:]]+/' \
     "${repo_dir}/README.md" "${repo_dir}/docs"; then
  echo "Repository documentation contains a personal absolute path." >&2
  exit 1
fi

if git -C "${repo_dir}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  mapfile -t selected_runtime_files < <(
    find "${repo_dir}/config/local" \
      -path '*/selected_runtime/*.yaml' -type f -print | sort
  )
  if ((${#selected_runtime_files[@]} == 0)); then
    echo "No publishable selected-runtime YAML files were found." >&2
    exit 1
  fi
  for selected_runtime_file in "${selected_runtime_files[@]}"; do
    selected_runtime_relative="${selected_runtime_file#"${repo_dir}/"}"
    if git -C "${repo_dir}" check-ignore -q "${selected_runtime_relative}"; then
      echo "Selected runtime is ignored: ${selected_runtime_relative}" >&2
      exit 1
    fi
  done
fi

bash -n "${repo_dir}/scripts/verify_selected_runtime.sh"

for markdown_file in \
  "${repo_dir}/README.md" \
  "${repo_dir}/AUDIT_REPORT.md" \
  "${repo_dir}/CHANGELOG.md" \
  "${repo_dir}"/docs/*.md \
  "${repo_dir}/config/estimator/README.md"; do
  if ! awk '
      /^```bash[[:space:]]*$/ { inside = 1; next }
      /^```[[:space:]]*$/ {
        if (inside) {
          print ""
        }
        inside = 0
        next
      }
      inside { print }
    ' "${markdown_file}" | bash -n; then
    echo "Invalid Bash block in ${markdown_file}." >&2
    exit 1
  fi
done

if awk '
    /^```/ {
      in_fence = !in_fence
      next
    }
    !in_fence && /\\\(|\\\)/ {
      printf "%s:%d:%s\n", FILENAME, FNR, $0
      invalid = 1
    }
    END { exit invalid }
  ' "${repo_dir}/README.md" "${repo_dir}/AUDIT_REPORT.md" \
    "${repo_dir}/CHANGELOG.md" "${repo_dir}"/docs/*.md \
    "${repo_dir}/config/estimator/README.md"; then
  :
else
  echo "Markdown contains unsupported inline \\\\( ... \\\\) math delimiters." >&2
  exit 1
fi

printf 'repository_checks_test passed\n'
