# Pre-push audit summary

This file is the current review entry point. Detailed historical investigations
and superseded conclusions are preserved in
[docs/audit_history.md](docs/audit_history.md); they are not deleted or
silently rewritten.

## Current verdict

The repository remains a standalone D435i/OpenVINS diagnostic VIO system. The
selected serial-specific bundle is reproducible but still
`BOOTSTRAP_UNVERIFIED`; it must not be described as drift-safe or
accuracy-certified.

The current implementation:

- keeps RealSense callbacks limited to bounded frame ownership and enqueueing;
- gives the ordered dispatcher sole ownership of OpenVINS ingestion;
- preserves raw device timestamps beside normalized timestamps;
- rejects malformed input, timestamp regression, non-finite estimator states,
  excessive estimated speed, and excessive accelerometer bias;
- keeps OpenVINS pinned at v2.7 with ROS disabled;
- records stream, serial, gyro sensitivity, gyro scale, timing, calibration,
  and health provenance;
- treats visual-support status as a continuity diagnostic, not a pose-quality
  certificate;
- uses ZUPT only to constrain velocity after conservative stationarity gates;
  it does not claim to recover accumulated position drift.

## Connected-camera evidence boundary

The 2026-07-29 marked-pose trials showed:

| Stream and lighting | Return error | Estimated error/path | Capture integrity |
| --- | ---: | ---: | --- |
| 90 Hz, poor lighting | 0.522 m | 1.78% | zero stereo/IMU drops |
| 90 Hz, room light, first run | 0.141 m | 0.53% | zero stereo/IMU drops |
| 90 Hz, room light, repeat | 0.405 m | 1.29% | zero stereo/IMU drops |
| 30 Hz, room light | 0.834 m | 2.48% | zero stereo/IMU drops |

These handheld runs were not ground-truth trajectories. They establish that
30 Hz was not a better live policy and that adequate lighting reduced sensor
gain, but they do not prove a repeatable absolute-accuracy bound. The selected
live policy therefore remains 90 Hz with adequate static scene illumination.
The 30 Hz stream remains the calibration-capture profile.

## Known limitation exposed by the trials

Frontend track count stayed high while endpoint error accumulated. The current
visual-support gate can detect sustained track loss; it cannot detect every
geometrically inconsistent correspondence or certify pose correctness.
Position correction after accumulated drift requires an external absolute
constraint, mapping, or loop closure, all intentionally outside repository
scope.

## Review and validation contract

Before a commit is considered ready:

```bash
git diff --check
./scripts/verify_selected_runtime.sh
cmake --preset portable-core
cmake --build --preset portable-core
ctest --preset portable-core --output-on-failure
./scripts/build_ubuntu.sh
ctest --test-dir build/linux-release \
  --output-on-failure \
  --no-tests=error
```

Also review:

- every documented Bash block with `bash -n`;
- all local Markdown links and math delimiters;
- CLI `--help` output against documented commands;
- the pinned OpenVINS patch against the dirty submodule;
- the final diff for personal paths, generated artifacts, secrets, and
  fabricated measurements.

A build or replay does not substitute for a connected-camera test. No EEPROM
or firmware write is part of this repository's validation workflow.

## Validation completed on 2026-07-29

The current source and pinned dependency patch passed:

- the Ubuntu release build and all 4 registered CTest cases;
- the portable-core build and all 4 registered CTest cases;
- the repository policy, Markdown-math, and documentation checks;
- all selected-runtime hash checks;
- `git diff --check` in both the project and OpenVINS submodule;
- CLI-help and source-fingerprint checks for all four applications;
- Python parsing and Bash syntax checks; and
- replay instrumentation checks whose trajectories remained byte-identical
  before and after adding read-only MSCKF update statistics.

The final connected-camera preflight was attempted but did not pass:
`rs-enumerate-devices` reported no D435i, and the one-second `ovrs_inspect`
sample could not start. Consequently, the final worktree has no post-change
live-camera validation claim. This environmental failure does not invalidate
the earlier connected trials above, but those trials predate the final
diagnostic-only source changes.

## Historical evidence

See [docs/audit_history.md](docs/audit_history.md) for the full chronology,
including calibration candidates, replay matrices, viewer work, ZUPT patch
review, kernel diagnostics, build commands, and superseded hypotheses.
