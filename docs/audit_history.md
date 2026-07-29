# Preflight audit report

Audit date: 2026-07-27, updated 2026-07-29

Release audited: **v0.5.2**

## Final verdict

**GYRO 0.5 RUNAWAY FIXED; RESIDUAL PITCH-MOTION DRIFT REMAINS; DIAGNOSTIC
USE ONLY**

### 2026-07-29 code and documentation cleanup

A conservative whole-worktree review preserved the existing calibration
files, datasets, and reviewed OpenVINS ZUPT behavior. It found one semantic
defect in the new diagnostics: `visual_support_features` added the total
number of persistent SLAM landmarks, even when those landmarks were not
observed in the current frame. Once the state held 25 SLAM landmarks, that
could keep a 12-feature gate green through weak frontend tracking.

The corrected gate uses a minimal read-only OpenVINS accessor for the current
camera-0 frontend track count. It does not modify tracking, MSCKF/SLAM
updates, pose, covariance, or ZUPT. On
`vio_gyro1_marked_down_return_20260729T0535`, the trajectory remained
byte-identical while visual support became `HEALTHY` at 3.666286 seconds.
The 5199 states contained 78-102 current tracks and ended `HEALTHY`; final
displacement remained 0.021163 m. A pitch-down replay and short low-light
replay also retained many frontend tracks, confirming that this gate reports
frontend continuity rather than geometric correctness or illumination
quality.

Selected-runtime hashes now have one executable source,
`scripts/verify_selected_runtime.sh`, plus one human-readable table in
`docs/selected_runtime.md`. Three personal checkout paths were removed from
the kernel runbook. Repository tests now reject personal absolute paths and
syntax-check documented Bash blocks. Local `calibration.bin`,
`calibration.json`, and `rs-imu-calibration-fixed.py` remain present but are
ignored because the supported repository workflow never writes camera
EEPROM.

### 2026-07-29 official EEPROM recalibration and gyro 1.0 supersession

The official RealSense IMU calibration utility collected 6000 measurements,
reported 9.803658 m/s^2 corrected norm against 9.806650 m/s^2, and
successfully wrote the calibration to serial `843212070146`. OVRS read back
the new accelerometer matrix with device motion correction active. A
stationary hardware capture measured 9.872342 m/s^2 mean sample norm and
0.017115 m/s^2 norm standard deviation with zero integrity errors.

The new 50.44-second raw VIO capture retained 4488 stereo pairs and 9980
synchronized IMU rows with zero drop/error counters. Replay with the former
gyro factor `0.5` reached the 3 m/s gate at 42.36 seconds. An identical-data
copy changing only synchronized gyro values to factor `1.0` completed all
47.82 initialized seconds with 0.0692 m/s maximum speed, 0.0065 m/s final
speed, 0.0588 m final displacement, and 0.557 m estimated path.

A connected 131.13-second live run using factor `1.0` also completed cleanly
without a safety failure. The operator reported about 0.2 m of physical
translation, while the estimator ended at 0.819 m and 0.0137 m/s. Factor
`1.0` therefore supersedes `0.5` for the current device-calibration state, but
the remaining approximately 0.62 m endpoint error prevents a drift-safe
claim. The world-locked viewer grid was physically confirmed to remain fixed.

Two later marked-pose hardware datasets tested complete upward and downward
pitch cycles with native factor `1.0`. Identical-data tracker A/B rejected
CLAHE, FAST threshold 10, their combination, a 300-point/minimum-spacing-12
variant, and disabled equalization. Some candidates increased the reported
feature count or improved one orientation, but all regressed trajectory
metrics in the other orientation or in the same ceiling run. The selected
global histogram equalization, FAST threshold 20, 200 points, and 15 px
minimum spacing remain unchanged. Exact dataset counts and replay metrics are
recorded in `docs/selected_runtime.md`.

### 2026-07-29 pitch-motion invalidation

Later connected live runs invalidate the earlier operational verdict. With
clean camera, timestamp, queue, and non-finite counters, a 29.11-second
initialized run accumulated 6.698 m estimated path and 4.147 m final
displacement while speed approached the 3 m/s safety gate. The failure was
reported when pitching toward the ceiling or floor even with sharp visible
edges.

The historical selected replay cited below also accumulated 60.893 m estimated
path and 10.677 m final displacement before stop recovery reduced final speed
to 0.0033 m/s. Final velocity was therefore mistaken for position stability.

Debug replay showed that MSCKF update features collapsed from periodic bursts
up to 35 before motion to fewer than 0.3 per camera frame and eventually zero
after motion began. Sharp edges were visible, but they did not become usable
triangulated update constraints. An identical-data `max_clones: 22` trial
reduced peak speed but worsened final displacement to 19.251 m and exceeded
the 90 Hz compute budget, so clone horizon alone is not a fix.

The world/grid motion in the viewer was a separate confirmed rendering bug:
the plot recomputed its centre and scale from every new trajectory state. The
viewer now keeps a world-locked frame until explicit `F` fit, uses a compact
axis triad, and follows the resized window aspect ratio.

No replacement estimator is promoted. The later official-calibration capture
and gyro-scale A/B above supersede the old `0.5` acquisition policy, but
residual position drift still requires controlled physical bounds before
accuracy acceptance.

### 2026-07-28 selected-runtime addendum

The active configuration is now the serial-specific local bundle:

```text
config/local/d435i-843212070146/selected_runtime/estimator.yaml
```

At this historical selection point it was post-RealSense-IMU-update Kalibr
candidate A, with a fixed
-4.900203074 ms shared camera-IMU offset, online time-offset estimation off,
explicit gyro scale 0.5 before synchronization, and visually gated continuous
stop recovery. Candidate B, the old factory/promoted bundles, and online-offset
results are preserved but are not operator alternatives.

The bundle remains `BOOTSTRAP_UNVERIFIED`. This is intentional: two
independent camera-IMU runs were close (0.060919 degrees, 3.193655 mm, and
0.688995 ms cam0-offset difference), but the available AprilGrid was small and
the Allan noise capture predates the official RealSense IMU table update.
Operational selection does not satisfy the repository's strict
`KALIBR_VERIFIED` promotion contract.

Evidence for the current selection:

- candidate A camera residuals were 0.291736 px and 0.281472 px, tighter than
  candidate B;
- a connected 17.3 s stationary live smoke test ended at 5.244 mm
  displacement with zero stereo/IMU drops and no rejected non-finite state;
- a 72.7 s stationary replay had 5.453 mm median-window return and
  0.068 degrees orientation return;
- a 120.6 s capture retained 3597 stereo pairs and 23970 synchronized IMU
  samples with no camera/queue/callback errors;
- four identical-data final comparison replays each produced 3531/3531 healthy
  states;
- the canonical moved path `selected_runtime/estimator.yaml` replayed the
  stationary dataset cleanly with the fixed offset and no non-finite state.

Online time-offset estimates varied from approximately -9.23 ms to +2.79 ms
across motion/observability conditions, so normal replay/live explicitly use
`--online-time-offset off`. Earlier continuous-ZUPT trials were correctly
rejected, but later debug showed their duration gate was unreachable: MSCKF
cleanup removed the interval's first observation before an interval-wide track
lookup. The reviewed patch now uses consecutive per-frame tracked-disparity
checks and the selected policy requires one second of visual stationarity.

The final moving capture for this review,
`datasets/vio_gyro_scale05_final_20260728T165508Z`, recorded 5388 stereo pairs
and 11982 synchronized IMU samples with all capture-integrity counters zero.
Corrected visual/gyro rotation agreed on representative windows. Exact
selected replay completed 57.83 seconds under the unchanged 3 m/s safety gate
and ended at 0.0033 m/s after stop recovery. Its path has no external ground
truth and its initial/final images do not prove the same physical pose, so this
is not an absolute-position or zero-drift claim.

After the acquisition/ZUPT changes, the full Ubuntu dependency build/install
path rebuilt OpenVINS and all 29 project targets; all four registered CTest
cases passed. A connected D435i produced the final capture above. Hardware
capture, replay completion, and absolute accuracy remain distinct claims.

The current operator source of truth is
[selected_runtime.md](selected_runtime.md). The calibration workflow
is retained only to reproduce or replace this selection.

### Historical pre-recalibration replay addendum

This section is retained as failed historical evidence. Its promoted
pre-recalibration bundle is not the selected runtime.

This addendum supersedes the older pre-calibration `BLOCKED` conclusions
below. The operator completed the physical D435i captures, Allan analysis,
stereo calibration, IMU-camera calibration, explicit review, and
serial-specific promotion. The promoted local bundle follows
`config/local/d435i-SERIAL/kalibr/estimator.yaml`.

The first reviewed-calibration replays still diverged: the controlled linear
round trip accumulated a 120.727 m path and ended 106.816 m from its start;
the room loop accumulated 147.190 m and ended 135.321 m from its start.
Stereo correspondence checks showed positive depth and subpixel epipolar
agreement, while short-window visual rotation agreed with IMU rotation.
Diagnostic instrumentation then showed that OpenVINS' generic feature
initializer rejected most otherwise consistent 3-5 m room features: its
condition-number limit was 10000 and its depth-to-baseline limit was 40.

The permanent D435i template and promoted local estimator now use:

```yaml
init_window_time: 2.0
num_pts: 200
histogram_method: "HISTOGRAM"
fi_max_cond_number: 60000
fi_max_baseline: 100
```

No online calibration flag or chi-square multiplier was loosened. Temporary
upstream logging was removed, and the pinned OpenVINS checkout again has no
tracked content changes. The project adapter now also honors the estimator
YAML `verbosity` setting.

Final replay with the active promoted configuration produced:

| Dataset | Path length | Final displacement | Peak displacement |
| --- | ---: | ---: | ---: |
| controlled linear round trip | 0.865 m | 0.031 m | 0.343 m |
| room loop | 12.083 m | 1.095 m | 2.554 m |

The stationary control remained entirely in accepted ZUPT updates. It
therefore produced no trajectory rows and returned the replay contract's exit
5 with an `INCOMPLETE` run; that is expected for a sequence with no transition
to motion, not evidence of VIO trajectory accuracy.

The post-replay live checks exposed a separate dynamic-scene and stop-recovery
failure. Three cleanly recorded runs diverged by 87-592 m despite zero input
drops. With the D435i stationary, foreground motion could release the
disparity-only ZUPT. After a real two-second camera motion stopped, orientation
also stopped but the estimated velocity remained about 8.7 m/s and the
accelerometer bias had been driven far from its initialized value.

A hybrid, continuous ZUPT experiment reduced the divergence but did not solve
it. Pinned OpenVINS' active zero-velocity path can advance the state and update
orientation and bias while leaving corrupted velocity unchanged. A reviewed,
opt-in project patch now adds the missing zero-velocity observation only after
accumulated visual disparity confirms a stationary interval for 0.25 seconds.
It does not enable OpenVINS' untested integrated-acceleration branch.

Replay with the patched stop detector produced:

| Dataset | Path length | Final displacement | Peak displacement | Final speed |
| --- | ---: | ---: | ---: | ---: |
| stationary control | 0.159 m | 0.013 m | 0.013 m | 0.0010 m/s |
| controlled linear round trip | 0.906 m | 0.029 m | 0.350 m | 0.0043 m/s |
| room loop | 10.744 m | 0.375 m | 2.374 m | 0.0056 m/s |

These no-ground-truth replays verify that slow real motion is no longer
suppressed while velocity converges near zero after stops. Live acceptance
remains blocked until this exact patched build passes the physical stationary
foreground and move-then-stop gates with the connected D435i.

The live failure also exposed a missing integrity boundary: finite covariance
and finite state values were treated as healthy even after estimated speed
reached 166.981 m/s and accelerometer bias exceeded 20 m/s². The runtime now
fails closed at configured limits of 3 m/s and 2 m/s², preserving the failed
run instead of serializing a misleading multi-kilometre trajectory.

Ubuntu 24.04 Release build, all 17 Python calibration tests, all three
registered CTest cases, source fingerprints, non-ROS OpenVINS cache, and
`preflight_ubuntu.sh --require-build` passed. Preflight did not see a connected
D435i during this final software check, so no new live/hotplug hardware success
is claimed. The recorded replays have no external ground truth: the large
divergence is fixed and round-trip consistency is substantially improved, but
absolute metric accuracy and final live behavior remain limitations.

### Superseding v0.5.2 conclusion

v0.5.2 closes state and external-tool gaps found while re-auditing README
Steps 8-10. It does not change estimator mathematics or invent calibration
values.

The three portable exports are now checked as one calibration set before bag
creation. `scripts/validate_calibration_export_set.py` re-hashes every fixed
provenance file and rejects mixed serials, capture modes, target hashes,
stereo/IMU-camera IR profiles, Allan/IMU-camera sample rates, row-policy
violations, inactive motion correction, manifest/source disagreement, staged
camera index/image count or dimension errors, and staged IMU row-count errors.
It writes a non-overwriting `UNVERIFIED_EXPORT_SET` report containing hashes
of the exact three manifests. Automated regressions cover manifest tampering,
missing staged data, serial mismatch, and an independently re-hashed
physically measured target mismatch.

README Step 8 now re-establishes the serial, measured target path, and one
calibration capture ID after a reboot or terminal restart. Failed gates unset
state. Every dependent block has an explicit `${variable:?message}` stop, so a
stale or empty variable cannot become `/bootstrap.yaml` or an unintended
output path. Step 9 runs in a strict subshell and stops at the first failed
capture, export, or cross-export check without closing the user's interactive
terminal.

The Docker branch now documents Docker CE installation from Docker's official
signed Ubuntu repository, inspects conflicting packages before any removal,
requires an explicit rootless/docker-group security choice, and verifies
non-root `docker info` plus `hello-world`. Repository scripts still do not
install Docker, ROS, firmware, kernel modules, or global Python packages.

The combined Kalibr/Allan image is now built by
`docker/calibration.Dockerfile` from a clean `git archive` of the pinned Allan
commit. Mutable `docker create`/`cp`/`commit` assembly was removed. Kalibr's
pinned `--cams` interface and output naming were checked against source. A
second source-level review found that Kalibr derives output paths from the
`--bag` argument: the procedure therefore creates a local bag symlink inside
each new empty work directory and passes its relative name. This guarantees
that the camchain, report, result text, and IMU output land in the directory
whose artifact counts are checked. The unexplained stereo `--bag-freq 10.0`
downsampling was removed.

The generated AprilGrid PDF is no longer described as an A4-layout guarantee.
The procedure explicitly centres its content-sized page on A4 at 100%/actual
size, checks all four physical borders in print preview, mounts it flat, and
uses measured tag/gap dimensions. Preview remains visibility/blur guidance;
only Kalibr `--show-extraction` is a tag-detection gate.

An Ubuntu operator run then exposed a Docker-only command-discovery error:
the pinned catkin build does not put `kalibr_create_target_pdf` on the generic
shell `PATH`. Source inspection confirmed that Kalibr installs the generator,
bag creator, and calibrators as ROS package executables. README now invokes
all four through `rosrun kalibr`, and the image build smoke-tests every
`--help` interface. The reported image build itself succeeded and can be
reused; no source rebuild or Allan re-recording is required for this fix.

Current-host v0.5.2 results:

| Check | Actual result |
| --- | --- |
| Host | Windows; this is not Ubuntu 24.04 validation |
| Direct GCC portable core | Fresh build in `build/audit-v0.5.2-windows-20260727`; 18/18 passed with `-Wall -Wextra -Wpedantic` |
| Direct GCC synthetic replay | Fresh build; 1/1 passed |
| Dependency-disabled CLIs | All four freshly compiled; every `--help` and `--version` returned success. These direct builds intentionally identify as non-CMake audit builds |
| Python calibration regressions | `Ran 15 tests ... OK` on CPython 3.13.12 |
| Python syntax/help | All repository Python compiled; 11 supported script `--help` calls passed |
| Calibration-set mismatch gates | Matching fixture passed; manifest/source tampering, missing staged image, serial mismatch, and independently re-hashed AprilGrid mismatch were rejected |
| Shell syntax | 10 project scripts, 10 pinned OpenVINS scripts, and 45 README Bash blocks passed Git Bash `bash -n` |
| Configuration/build metadata | 10 YAML files parsed after read-only OpenCV-header normalization; `CMakePresets.json`, 13 CMake source paths, five 40-hex commits, and Dockerfile contract passed |
| Submodule | Gitlink and checked-out OpenVINS are both `93adc241390d13e99232652cf05cbe18a93c7bea`; tracked tree is clean |
| Plot smoke | Summary and PNG generation passed; PNG was visually inspected; deliberate path-bound violation returned exit 5 |
| Diff hygiene | `git diff --check` passed; no personal camera serial/path was found in tracked project content |
| Sanitizers | Not run: this MinGW installation lacks `libasan` and `libubsan`; the attempted link failed with `cannot find -lasan/-lubsan` |
| CMake/CTest | Not run for v0.5.2 on this host: `cmake`, `ctest`, and `ninja` are absent |
| Docker image/tool execution | Docker is absent on the audit host. The operator's Ubuntu log proves the pinned Kalibr image build completed; the corrected four-tool `rosrun` smoke test and PDF generation still require rerun |
| Formatter/linter | `clang-format` and `shellcheck` are absent; strict GCC warnings and Bash syntax checks passed |
| Python environment | Repository `.venv` is an Ubuntu Python 3.12 environment and cannot run on Windows. No global package was installed or changed. Current Windows global `pip check` has an unrelated basemap/NumPy conflict, so it is not used as venv evidence |
| Ubuntu/D435i/Kalibr | v0.5.2 build, preview GUI, physical capture, Docker build, Allan fit, both Kalibr optimizations, manual PDF review, promotion, and post-calibration VIO remain unexecuted |

The user-provided earlier Ubuntu logs establish that v0.5.1's predecessor
stack could build and see the D435i, but they do not verify the current v0.5.2
tree or calibration results. The existing 10-hour Allan capture remains valid
input evidence; it must not be re-recorded merely because the terminal was
closed. It still requires external Allan processing and human review.

Run this first after booting Ubuntu, from the repository root:

```bash
./scripts/build_ubuntu.sh
```

Expected success includes `project_version=0.5.2`, pinned OpenVINS/Ceres/
librealsense identities, four matching executable source fingerprints, named
CTest cases, and no `No tests were found`. Then run:

```bash
./scripts/preflight_ubuntu.sh \
  --require-camera \
  --serial "${D435I_SERIAL}" \
  --stream-config config/sensors/realsense_streams.yaml
```

Set and verify `D435I_SERIAL` through README Step 2; the repository does not
embed a personal unit identifier. After preflight, resume README Step 7 for
the measured A4 target and Step 8's state gate. Recover the completed Allan
capture through Step 8A instead of recording it again.

Common immediate failure signatures:

- `VERSION`/cache reports `0.5.1`: the build is stale; rerun
  `./scripts/build_ubuntu.sh`;
- `No tests were found`: the build/test registration is invalid; stop;
- Ceres resolves outside `.deps/install/ceres`: stop; system Ceres must not be
  used;
- `calibration export set: FAIL`: do not make bags; correct the named
  serial/target/profile/rate/mode mismatch;
- Docker socket permission failure: finish the documented non-root setup and
  log in again; do not use `sudo docker`;
- Kalibr reports missing/duplicate artifacts: preserve the isolated work
  directory and retry with a new tool-run ID after correcting the cause.

### Superseding v0.5.1 conclusion

The previous calibration work area, including the 10-hour Allan capture, was
deleted on 2026-07-27 at the operator's explicit request before a clean
recapture. Historical row-count and diagnostic observations are no longer
usable calibration evidence. The current workflow records duration as
provenance rather than using a hardcoded hour threshold; timestamp integrity,
stationarity, SI units/axes, exact stream rates, Allan fit quality, and
multi-orientation intrinsic review remain independent gates.

The observed `/bootstrap.yaml` traceback was a repository bug: README relied
on an empty session variable and `analyze_stationary_imu.py` read the resulting
path without validation. v0.5.1 now recovers the serial and bundle from capture
metadata, checks every path, and returns a concise argument error without a
traceback. The short stationary check is diagnostic by default; a gravity
threshold is no longer something an operator must guess.

README Steps 7-10 now define the board as one official Kalibr-generated,
physically measured AprilGrid mounted on rigid backing. An A4 planner checks
paper margins, active grid size, and the required white border. The procedure
pins Kalibr and Allan commits, gives an isolated Docker/VM decision branch, and
requires Kalibr `--show-extraction`.

`ovrs_record --preview` now provides main-thread IR1/IR2 preview for stereo and
camera-IMU capture. Space stops the preview pipeline and starts a new clean
capture pipeline; preview frames/counters cannot enter the dataset. The
recording remains fail-closed on window close, `q`, Escape, Ctrl+C, camera or
queue failure. The preview checks visibility, exposure, and blur only; it is
not presented as AprilTag detection.

v0.5.1 portable/Python/documentation validation is recorded below. The new
OpenCV HighGUI recorder path has not run on Ubuntu 24.04 or a physical D435i in
this audit environment. The verdict therefore remains `BLOCKED` until the
operator rebuilds v0.5.1 on Ubuntu, exercises both preview capture modes, runs
the external Allan/Kalibr workflow, and promotes reviewed serial-specific
calibration.

Current-host v0.5.1 results:

| Check | Actual result |
| --- | --- |
| Direct GCC portable core | 18/18 passed with `-Wall -Wextra -Wpedantic` in new `build/audit-v0.5.1-windows-20260727` |
| Direct GCC synthetic replay | 1/1 passed |
| Dependency-disabled CLIs | All four compiled; `--help` and `--version` returned success. These direct builds correctly identify themselves as non-CMake audit builds, not v0.5.1 release binaries |
| Python workflow tests | `Ran 13 tests ... OK` under Windows Python 3.13 and through Git Bash |
| Python syntax | All repository Python files under `scripts/` and `tests/` byte-compiled |
| Shell syntax | All 10 project shell scripts, 10 pinned-submodule shell scripts, and 41 README Bash blocks passed Git Bash `bash -n` |
| A4 planner | 6x6, 18 mm, 0.3 spacing, 5 mm printer margin computed 135.0 mm active grid and 181.8 mm including border; portrait/landscape fit passed |
| Missing-config regression | Automated test confirms exit 2, explicit unreadable-config error, and no traceback |
| Real 10-hour Allan capture | Full read-only bounded-memory scan passed: mode `imu-allan`, 7,193,396 synchronized rows, 36,000.713241 s, no stderr; row counts match the recorder summary |
| Hardware recorder syntax | New preview source, RealSense source, and recorder passed `-fsyntax-only` with strict warnings against pinned librealsense headers and a local OpenCV API stub; this is not a link, GUI, camera, or runtime test |
| Config/path checks | 10 YAML files parsed, CMake preset JSON parsed, 14 CMake source paths resolved, and five 40-hex dependency commits validated |
| CMake/CTest | Not run: `cmake`, `ctest`, and `ninja` are absent on this Windows host |
| Formatting | `clang-format` is absent; edited C++ was manually inspected and strict GCC syntax emitted no project warnings |
| Docker | Not run: Docker is absent on this host; the documented pinned image build remains unexecuted |
| Ubuntu 24.04 + D435i preview | Not run |

### Superseding v0.5.0 conclusion

Pinned OpenVINS v2.7 requests `T_imu_cam`. Its matrix parser also has an
explicit compatibility fallback that reads `T_cam_imu` and applies an SE(3)
inverse. The earlier key therefore was not ignored and is not a proven root
cause of the observed divergence. The mixed naming was still an unsafe
repository contract because manual review or renaming could confuse opposite
transform directions.

v0.5.0 now exports and validates canonical OpenVINS `T_imu_cam`, rejects the
opposite key in runtime files, and validates/inverts Kalibr `T_cam_imu` only
during promotion. The ignored local bootstrap bundle was migrated with a
retained `.pre-v0.5.0.yaml` backup; an independent comparison reported
`max_inverse_difference=0`, so the migration preserved the transform represented
by the upstream fallback rather than changing estimator geometry.

This release also adds explicit VIO/Allan/stereo/IMU-camera capture modes,
fail-closed calibration export, measured AprilGrid generation, Allan/noise
binding, structural Kalibr/PDF review, and serial-specific promotion with
explicit shared time-offset selection. Docker is absent and not assumed;
Kalibr/Allan execution remains an external ROS1 environment step.

The Allan procedure now stops on an operator-bounded stationary gravity
mismatch before export. It does not infer a three-axis scale matrix from one
orientation. Live finalization now also fails closed on capture loss,
dispatcher queue loss/nonmonotonic input, and IMU synchronization integrity
errors; the dependency-enabled path still requires the Ubuntu rebuild below.

Calibration export format v2 carries copied source metadata and flat SHA-256
bindings. Downstream Allan/Kalibr preparation rejects a missing or modified
source report, summary, stream configuration, device report, or target.

Current-host v0.5.0 verification:

| Check | Actual result |
| --- | --- |
| Fresh direct GCC portable core build/test | 18/18 passed with `-Wall -Wextra -Wpedantic` under `build/audit-v0.5.0-final2-20260726` |
| Fresh direct GCC synthetic replay build/test | 1/1 passed |
| Four freshly compiled dependency-disabled CLI help/version checks | Passed |
| RealSense source and hardware-enabled inspector syntax | Passed against pinned 2.56.5 headers; warnings were upstream headers |
| Hardware-enabled recorder syntax | Passed against pinned RealSense headers and a local OpenCV API stub; not a link/runtime test |
| `python tests/calibration_scripts_test.py` | `Ran 9 tests ... OK` on Python 3.13.12 |
| Python syntax/CLI checks | `py_compile` passed for 10 project Python files; eight CLI `--help` checks passed |
| Plot smoke | Summary/bounds passed; PNG generated and visually inspected; deliberate bound violation returned exit 5 |
| Shell/README syntax | All 10 shell scripts and all 33 README Bash blocks passed `bash -n` |
| Configuration parse/path checks | All 10 YAML files and `CMakePresets.json` parsed; 32 C++ source/include paths resolved |
| C++ formatting tool | Not run: `clang-format` is unavailable on the current host; edited C++ was manually reviewed against `.clang-format` |
| Legacy transform migration | Passed; backup retained |
| Exact inverse comparison | `max_inverse_difference=0` |
| CMake/C++ build | Not run: CMake unavailable on current Windows `PATH` |
| Dependency-enabled live application syntax | Passed with both feature macros and pinned librealsense headers; estimator/viewer implementations still could not compile/link without the OpenCV/OpenVINS development graph |
| WSL build | Not run: distro enumeration was denied |
| Ubuntu 24.04/D435i v0.5.0 test | Not run |
| Allan/Kalibr calibration | Not run |

The verdict remains `BLOCKED` until the corrected C++ builds/tests on Ubuntu,
the external Allan/Kalibr workflow produces reviewed unit-specific files, and
a fresh physical trajectory passes operator-defined bounds.

Run this first from the repository root on Ubuntu:

```bash
./scripts/build_ubuntu.sh
```

### Historical v0.4.0 evidence

Ubuntu 24.04.4, the required compiler/build packages, repository-local Ceres
2.1.0, repository-local librealsense 2.56.5, OpenVINS v2.7, and physical D435i
capture were verified from operator logs. The pinned build correctly ignored
system librealsense 2.58.3.

The physical `capture_20260725T213451Z` dataset is complete and internally
ordered: 60.55 seconds, 1,796 stereo pairs, 11,975 synchronized gyro samples,
zero reported camera/queue drops, and no malformed/timestamp/callback errors.
This proves capture integrity, not estimator accuracy.

Replay and live did initialize, but both estimator results are invalid for the
motion the operator actually performed (slow rotation followed by a return
near the starting pose):

| Run | Duration | Estimated path | Final displacement |
| --- | ---: | ---: | ---: |
| Replay | 53.700641 s | 666.456174 m | 660.730498 m |
| Live | 41.607496 s | 55.030973 m | 33.184267 m |

The repository and recorded data were cross-checked without applying a scale
factor or clamping the result. OpenVINS requests `T_imu_cam`; direct inspection
also confirmed its compatibility parser reads `T_cam_imu` and returns its
inverse. The legacy key alone therefore does not explain this failed run.

A concrete calibration inconsistency is present. During the first
operator-confirmed stationary three seconds, 584 synchronized samples have a
mean acceleration norm of 9.415342258 m/s² while the estimator uses
9.806650000 m/s², an error of 0.391307742 m/s². The initial state absorbs
approximately the same amount as accelerometer bias. Once the unit rotates,
an unmodelled accelerometer scale/cross-axis error would become
orientation-dependent and could not be represented by one fixed bias. The
available single-orientation interval does not prove that this is the sole
cause; camera-IMU rotation/time offset, temperature, and visual tracking also
remain candidates. It is evidence that the current calibration/model is
inadequate, but not enough to fabricate a 3-axis intrinsic matrix, bias random
walk, or camera-IMU time offset.

v0.4.0 therefore makes the safe repository-local fixes:

- live/replay require `KALIBR_VERIFIED` by default; factory bootstrap use
  requires an explicit diagnostic override;
- librealsense motion correction is explicitly set and its requested/actual
  state plus factory motion intrinsics are recorded;
- replay binds captured motion-correction semantics to the IMU calibration
  and rejects ambiguous legacy datasets;
- the main, IMU, and camera YAML files must identify the same unit and
  calibration state; positive finite IMU noise values are checked before
  OpenVINS starts;
- a stationary-IMU analyzer reports gravity mismatch and short-term
  white-noise evidence without inventing random walk;
- trajectory validation accepts only operator-supplied physical bounds;
- native live/replay IR and pose viewing is available through `--viewer`,
  with all HighGUI work on the main thread and bounded trajectory history;
- a live output race between worker-thread state writes and main-thread
  diagnostics writes is fixed by serializing `RunWriter`.

The verdict remains `BLOCKED` because v0.4.0 has not yet been fully rebuilt on
Ubuntu, its viewer has not been exercised on the Ubuntu graphical backend,
and no unit-specific multi-orientation IMU/Allan/camera-IMU temporal
calibration has been supplied. The existing hundreds-of-metres trajectory is
explicitly failed evidence, not a successful OpenVINS result.

The historical v0.4.0 handoff command was:

```bash
./scripts/install_ubuntu_dependencies.sh
```

It is superseded by the v0.5.1 build command above because the operator has
already run the dependency installer.

## Version progression

These are documented development milestones, not claims that historical Git
tags exist:

| Version | Meaning |
| --- | --- |
| v0 | Standalone D435i/OpenVINS design boundary and handheld-validation scope |
| v0.1.0 | First ROS-free C++ target graph, capture/synchronization, estimator adapter, recording, replay, and initial docs |
| v0.2.0 | First full preflight hardening: exact dependency pins, local Ceres isolation, calibration/dataset/runtime safety gates, venv workflow, tests, and Ubuntu scripts |
| v0.3.0 | Second audit: authoritative version source, CLI/version validation, strict round-trippable stream config, reduced duplication, startup/output/replay hardening, and refreshed verification |
| v0.3.1 | RealSense lifecycle and calibration patch: one shared SDK context, event-driven hot-unplug detection, Y16-safe capability enumeration, calibration-resolution gates, and stronger hardware preflight |
| v0.3.2 | Content-identity hardening: deterministic source fingerprint embedded in every executable and verified by both Ubuntu build and preflight |
| v0.3.3 | Operator/run-integrity hardening: serial/build/stream-bound preflight and preparation, safe distortion mapping, partial-run markers, stricter replay validation, and corrected shared-IMU-bracket dispatch |
| v0.4.0 | Estimator-evidence and visualization hardening: verified-calibration default gate, explicit motion-correction provenance, stationary IMU analysis, operator-defined trajectory bounds, serialized run output, and main-thread live/replay viewer |
| v0.5.0 | Canonical OpenVINS transform direction/key, calibration-specific capture/export, Allan/Kalibr review and promotion gates, and procedural stop/go documentation |
| v0.5.1 | A4 AprilGrid planning, fail-safe stereo calibration preview, pinned isolated external tools, resumable/bounded-memory Allan validation, and corrected procedural gates |
| v0.5.2 | Cross-export coherence gate, restart-safe Step 8 state, reproducible calibration Dockerfile, source-checked Kalibr output isolation, and official Docker install branch |

Detailed entries are in `CHANGELOG.md`.

## Scope and current-host boundary

The entire project-owned tree, CMake files, presets, scripts, documentation,
configuration, tests, version interface, and pinned OpenVINS
submodule were inspected. The ignored local librealsense source checkout was
also checked against its pin. Project include and CMake source references were
checked for missing files.

There are no committed generated C++ source files. Build/version metadata is
supplied from CMake definitions with safe non-CMake fallbacks. `VERSION`
contains the single machine-readable project version; native dependency pins
remain centralized in `cmake/DependencyVersions.cmake`.

Initial Windows audit host:

- Windows 11, not Ubuntu;
- Git 2.54.0.windows.1;
- MSYS2 GCC/G++ 12.2.0;
- CPython 3.13.12;
- global Matplotlib 3.10.8;
- no CMake, Ninja, pkg-config, Docker, OpenCV SDK, Eigen, Ceres, or usable
  librealsense runtime on `PATH`;
- no connected/accessible D435i.

Docker was not installed, so no Ubuntu 24.04 container was used or installed.
The existing build directories were preserved. Each audit rebuild used a new
named directory under `build/`; the current one is
`build/audit-v0.4.0-core-20260726`.

Actual Ubuntu evidence supplied by the operator:

- Ubuntu 24.04.4 LTS x86-64, kernel 7.0.0-28-generic;
- GCC 13.3.0, CMake 3.28.3, OpenCV 4.6.0, Eigen 3.4.0, Python 3.12.3;
- the dependency installer completed successfully and found every requested
  package already installed;
- preflight verified all pins and the exact OpenVINS commit; the latest Ubuntu
  run also confirmed that the CRLF-aware check accepts the semantically clean
  NTFS checkout;
- the first Ceres configuration failed because `CXSparse::CXSparse` was not
  defined; the second run verified the fix and installed local Ceres 2.1.0;
- a later build configured, built, and installed local librealsense 2.56.5,
  then configured OpenVINS against local Ceres 2.1.0 with ROS disabled;
- that earlier run stopped at a repository validator false-negative before OpenVINS
  compilation and before `build/linux-release` was configured;
- the librealsense run also exposed unnecessary default firmware download/tool
  targets and a failing global `ldconfig` attempt during local installation;
  all three are now disabled or narrowly intercepted by repository code;
- a physical D435i was enumerated by both the project inspector and
  `rs-enumerate-devices`; its firmware and BMI055 IMU type were readable;
- the newest enumeration reports USB 3.2 and matching IR1/IR2
  `848x480@30` and `640x480@30` support, plus 200/400 Hz gyro and 63/250 Hz
  accelerometer profiles;
- an earlier preflight correctly failed when a stale v0.3.1 inspector aborted
  its required sample; the subsequent rebuilt inspector completed the physical
  sample and exported a serial-specific factory report;
- system librealsense 2.58.3 is present but is not selected by the build;
- `ovrs_live` and `ovrs_replay` are hardware/OpenVINS-enabled and reached the
  intended serial-specific calibration gate;
- a historical 60.55-second recording completed at 848x480 Y8, 30 Hz, 200 Hz
  gyro, and 250 Hz accelerometer with 1,796 images per camera and 11,980
  synchronized IMU rows; the currently shared dataset path instead contains a
  later complete 10.43-second run with 297 images per camera and 1,982
  synchronized IMU rows;
- the Y16 intrinsics-enumeration defect is fixed and was physically rerun: the
  inspector sampled for 10.4 seconds, reported no malformed frames, rejected
  timestamps, or callback errors, and wrote the 848x480 factory export.

Version-control warning: this is still an initial repository with no `HEAD`
commit. The original 68 repository entries remain staged as additions,
including the exact OpenVINS gitlink; this latest fix modifies tracked entries
and adds 18 unstaged source/helper files. All 86 status entries were reviewed.
No
build, dependency, dataset, run, output, venv, or Python-cache artifact is
staged. Both staged and unstaged diff whitespace checks passed. No historical
release tag was fabricated.

## Dependency and version matrix

| Dependency | Repository policy | Audit status |
| --- | --- | --- |
| Project | `0.5.2` from root `VERSION` | Full Ubuntu/OpenCV/OpenVINS build, Linux and portable CTest, Python calibration tests, and documentation checks pass in the final 2026-07-28 addendum |
| Ubuntu | 24.04 x86-64 primary; 22.04 secondary | Ubuntu 24.04.4 x86-64 preflight passes; connected D435i capture was performed on kernel 6.17.0-35 |
| OpenVINS | tag `v2.7`, commit `93adc241390d13e99232652cf05cbe18a93c7bea` | Exact revision, reviewed ZUPT patch, non-ROS/local-Ceres build, link, replay, and stop recovery verified |
| Ceres Solver | repository-local `2.1.0`, commit `f68321e7de8929fbcdb95dd42877531e64f72f66` | Ubuntu configure, build, install, exact cache policy, and local prefix verified |
| librealsense2 | exact `2.57.3`, tag `v2.57.3`, commit `5e046e509995cda79b42d89fa95ab65f90678641` | Repository-local install/runtime and final 60.56-second physical moving capture verified |
| Kalibr | external-only commit `1f60227442d25e36365ef5f72cd80b9666d73467` | Source/Docker contract inspected and pinned; image build and calibration not run on this host |
| allan_variance_ros | external-only commit `1d54b602ee7f2ba0427865d63afe4945d913ed24` | Ubuntu 20.04/devcontainer contract and three-hour minimum inspected and pinned; 10-hour capture not yet fitted |
| D435i firmware | read-only; never changed by this repository | Observed 5.17.3.10; compatibility was validated by read-only streaming, not inferred from a release matrix |
| OpenCV | major version 4; hardware recorder now requires `core`, `imgproc`, `imgcodecs`, `highgui`; OpenVINS also uses `features2d`, `calib3d` | Ubuntu package reports 4.6.0; preview and interactive trajectory-viewer link/runtime verified |
| Eigen | CMake requires `>=3.3` | Ubuntu package reports 3.4.0 |
| Boost | `system`, `filesystem`, `thread`, `date_time` | Ubuntu 1.83.0 (`libboost-all-dev` 1.83.0.1ubuntu2) verified |
| glog / gflags | Ceres/OpenVINS logging and flags dependencies | Ubuntu glog 0.6.0 and gflags 2.2.2 verified |
| GCC/G++ | C++17, no extensions; Ubuntu distro compiler | GCC 13.3.0 built prior Ubuntu hardware executables; GCC 12.2 built v0.5.0 portable core, 18 core cases, synthetic replay, and four dependency-disabled CLIs with strict warnings |
| CMake | `>=3.22` | Ubuntu CMake 3.28.3 verified |
| Ninja | required by supported presets/build | Ubuntu Ninja 1.11.1 verified |
| BLAS/LAPACK | Ubuntu development packages for Ceres dense solvers | Ubuntu 3.12.0 development packages verified |
| SuiteSparse/CXSparse | not required by the selected OpenVINS solver; disabled in local Ceres | SuiteSparse 7.6.1/CXSparse 4.3.1 are installed but intentionally not linked |
| libusb/udev/OpenSSL | Ubuntu development packages for pinned RSUSB fallback | libusb 1.0.27, udev 255.4, and OpenSSL 3.0.13 development packages verified |
| Python | optional for C++ runtime; required for supported calibration validation/promotion; CPython 3.11-3.13 | Ubuntu Python 3.12.3, repository `.venv`, script CLI checks, and calibration workflow tests pass |
| Matplotlib | optional venv pin `3.11.1` | Repository Ubuntu `.venv` import and plotting support verified |
| PyYAML | optional venv pin `6.0.3` | Repository Ubuntu `.venv` import and calibration validation/promotion tests pass |
| Tk | Ubuntu `python3-tk`; optional interactive plotting backend | Installed and accepted by Ubuntu preflight |

Pin sources:

- OpenVINS v2.7:
  <https://github.com/rpng/open_vins/releases/tag/v2.7>
- Ceres 2.1.0:
  <https://github.com/ceres-solver/ceres-solver/releases/tag/2.1.0>
- librealsense 2.57.3:
  <https://github.com/realsenseai/librealsense/releases/tag/v2.57.3>
- D400 firmware release matrix:
  <https://dev.realsenseai.com/docs/firmware-releases-d400/>
- Matplotlib 3.11.1 and its CPython 3.13 wheels:
  <https://pypi.org/project/matplotlib/3.11.1/>

`cmake/DependencyVersions.cmake` is the single repository source for native
pins. Project CMake searches Ceres only below `OVRS_CERES_PREFIX` with
`NO_DEFAULT_PATH`; Ubuntu system Ceres 2.2 cannot be selected accidentally.
The local Ceres build enables position-independent code because OpenVINS builds
a shared library. It now disables the optional SuiteSparse and CXSparse
backends. This avoids Ceres 2.1.0's missing imported-target failure with
Ubuntu's CXSparse package. The pinned OpenVINS dynamic initializer explicitly
uses `ceres::DENSE_SCHUR`, so this does not alter its selected solver path.

OpenVINS is configured with `ENABLE_ROS=OFF`, `ENABLE_ARUCO_TAGS=OFF`, and
CMake package discovery disabled for both `catkin` and `ament_cmake`. This
prevents an existing ROS installation from leaking include or link
dependencies into the standalone build.

## Python and venv status

Python is not required by `ovrs_inspect`, `ovrs_record`, `ovrs_live`, or
`ovrs_replay`; missing Python does not change the independent C++ runtime.
Python is, however, required for the supported calibration artifact workflow:
target generation, capture export, stationary analysis, Allan binding, Kalibr
validation, legacy transform migration, and verified-bundle promotion.
Third-party packages for those tools and plotting belong only in `.venv`.
Several validation/export operations use only the standard library, while
Kalibr YAML validation, migration, and promotion require PyYAML.

The plotting script:

- passed byte-compilation and execution on CPython 3.13.12;
- imports Matplotlib only when a plot is requested;
- supports `--summary-only` without third-party packages;
- selects a real GUI backend for interactive use and gives an error instead of
  emitting a meaningless `FigureCanvasAgg` warning;
- supports deterministic `--save` output and operator-supplied physical
  acceptance bounds;
- rejects nonfinite and non-increasing trajectory timestamps;
- gives an actionable error when Matplotlib is absent.

The stationary analyzer uses only the Python standard library, passed CPython
3.13 byte-compilation and execution on the physical dataset, and explicitly
does not estimate random walk from the short record.

The pinned OpenVINS submodule contains three upstream Python/launch files.
All three passed CPython 3.13 syntax compilation, but their imports require
ROS1/ROS2 (`rospy`, `rosnode`, `launch`, `launch_ros`, or
`ament_index_python`) and, for the process monitors, `psutil`. They are not
configured, installed, imported, or executed by this standalone target graph;
they are not project Python dependencies and were deliberately not added to
`requirements.txt`.

`requirements.txt` exactly pins Matplotlib and its runtime dependency set for
CPython 3.11-3.13. `.venv/` is ignored. No global package was installed or
modified, and no `sudo pip` command exists. A Windows venv was deliberately not
created because it would be unusable after booting Ubuntu.

Ubuntu workflow:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --requirement requirements.txt
.venv/bin/python scripts/plot_trajectory.py --help
```

## Issues found and safe fixes applied

### Build and dependency resolution

- Promoted the repository to v0.3.0 using one root `VERSION` source, added
  `--version` to every executable, and made the build/preflight cache check the
  resolved version.
- Added a retrospective `CHANGELOG.md` that distinguishes v0, v0.1.0, v0.2.0,
  and v0.3.0 without fabricating historical Git tags.
- Fixed clean-build OpenCV header visibility for `ovrs_record` and
  `ovrs_replay`.
- Removed unnecessary OpenCV linkage from `ovrs_realsense`.
- Added the missing `imgcodecs` component to the OpenVINS build combination.
- Applied project warnings and optional sanitizers consistently to libraries,
  applications, and tests while treating third-party headers as system
  headers.
- Centralized Ceres and librealsense versions with the existing OpenVINS pin.
- Enforced repository-local exact Ceres resolution and added cache checks.
- Added Ceres PIC for the shared OpenVINS library.
- Disabled Ceres 2.1.0's unused SuiteSparse/CXSparse backends after the actual
  Ubuntu 24.04 configure exposed a missing `CXSparse::CXSparse` imported
  target. The script validates both cache options before compiling.
- Centralized dependency-checkout integrity checks. They ignore only
  cross-filesystem executable-mode metadata and CR at end-of-line while still
  rejecting binary changes, other whitespace changes, and substantive staged
  or unstaged text changes.
- Replaced cache-type-specific `grep` checks with one exact-value helper.
  CMake may legitimately retain command-line values as `UNINITIALIZED`; the
  helper accepts that representation without weakening the required value.
- Reduced the local librealsense build to its required library surface:
  examples, graphical extensions, tools, unit tests, Python bindings, update
  checks, the all-in-one static bundle, and firmware downloads are disabled.
- Prevented librealsense's unconditional bare `ldconfig` install hook from
  touching the global loader cache. The build installs a private no-op shim
  under `.deps/build`, prepends it to `PATH` only for the local install call,
  and never changes the caller's persistent environment.
- Preserved the existing dependency build/install directories as requested.
  Optional librealsense binaries and downloaded firmware data produced by the
  earlier default-heavy run may remain as ignored `.deps` artifacts, but the
  current target graph does not rebuild, install, invoke, link, or add them to
  the runtime `PATH`.
- Replaced the ambiguous Linux CTest preset invocation with an explicit build
  directory and `--no-tests=error`.
- Disabled ROS1/ROS2 package discovery, not merely ROS source compilation.
- Rechecked OpenVINS's misleadingly named `cmake/ROS1.cmake`: with discovery
  disabled and `ENABLE_ROS=OFF`, its non-ROS branch sets `ROS_AVAILABLE=0`,
  builds the monolithic library, and provides the local header/library install
  rules without linking catkin.
- Made Ubuntu and Windows build entry points independent of the launch
  directory.
- Made the Ubuntu job count and required-command checks fail clearly.
- Added required RSUSB development packages and removed global
  `python3-matplotlib` installation.
- Enabled install RPATH link-path propagation for installed executables.
- Made the install surface application-only (`<prefix>/bin`) instead of
  installing headers without corresponding public libraries.

### Configuration and modularity

- Added a small reusable stream-YAML loader instead of leaving the committed
  stream configuration unused.
- `ovrs_inspect`, `ovrs_record`, and `ovrs_live` now accept
  `--stream-config`; explicit CLI options override file values.
- All three hardware applications now use one shared stream-option allowlist.
  This closed the inconsistency that parsed overrides in `ovrs_inspect` but
  rejected them during top-level argument validation.
- Unsupported camera profiles remain fatal. No automatic resolution fallback
  was introduced because it could silently mismatch camera calibration.
- Device selection now filters for a SDK-reported D435i before applying the
  optional serial. With no serial, a different RealSense model can no longer
  be selected merely because it enumerated first.
- Inspector capability listing no longer assumes that every advertised video
  profile exposes intrinsics. Y16/non-calibration profiles may report no
  intrinsics without aborting, while selected Y8 profiles must return finite,
  dimension-consistent intrinsics before export.
- Boolean CLI values now reject anything other than `on` or `off`.
- Live mode defaults to the calibrated serial when `--serial` is omitted.
- Live/replay require the actual recorded gyro rate to match the referenced
  IMU calibration `update_rate`.
- Stream settings, dependency pins, capture, synchronization, estimator
  adaptation, and output remain in separate modules. Estimator equations,
  feature logic, and OpenVINS state propagation were not altered.
- Centralized stream CLI overrides and resolved-stream serialization, removing
  duplicate parsing/YAML construction from live and record.
- Resolved stream YAML now uses the loader's canonical keys and passes a
  round-trip test.
- Removed the unused calibration-state member from stream configuration;
  calibration approval remains in the main/IMU/camera YAML validation gate.
- Removed unused per-image fields and exposed the existing maximum IMU
  interpolation-delay statistic in runtime/recording logs.
- CLI parsing rejects unknown or duplicate options and missing values. Numeric
  parsing rejects trailing junk, nonfinite values, negative unsigned values,
  and duplicate stream YAML keys; serial text is constrained to a safe
  portable character set.
- The live wrapper accepts a positional estimator YAML only when the first
  argument is not an option and otherwise uses an explicitly set
  `OVRS_ESTIMATOR_CONFIG`. Live/replay wrappers no longer fall back to the
  intentionally invalid committed template.
- Replay now gives a dataset-identity explanation for `--serial` instead of a
  context-free unknown-option error.

### Calibration and path safety

- Closed a fatal placeholder path: changing only the main serial can no longer
  start VIO with the committed invalid camera intrinsics/extrinsics.
- The main estimator YAML and referenced IMU/camera YAML files must have
  matching `calibration_state` and `calibrated_serial`.
- Referenced estimator files must be safe relative paths; absolute, parent
  traversal, and Windows-style paths are rejected before OpenVINS reads them.
- Live and replay reject missing, unknown, or placeholder calibration
  identities.
- Live and replay require exactly two camera `resolution` entries and compare
  both against the selected live stream or recorded stream configuration
  before constructing OpenVINS or creating run output.
- Added a fail-closed factory-export preparation script. It validates serial,
  required key counts, identical positive camera resolutions, and nonfinite
  tokens; requires explicit review acknowledgement; writes only to ignored
  `config/local`; and refuses overwrite.
- Allan YAML preparation now cross-checks its own topic and update rate against
  both matching capture manifests before accepting any noise values.
- Replaced list-based CSV validation/export with streaming validation so a
  multi-hour Allan capture cannot expand into tens of millions of retained
  Python dictionaries. Added a read-only capture validator CLI.
- Corrected the flat metadata parser to ignore indented RealSense diagnostic
  mappings; repeated child keys under gyro and accelerometer sections are
  valid and no longer look like duplicate top-level metadata.
- Capture validation now requires actual camera, raw IMU, and synchronized IMU
  CSV row counts to equal the recorder summary. Truncated data fails closed.
- Replaced the README's manual placeholder-editing gap with one ordered,
  copyable build, inspect, validate, review, prepare, record, replay, live, and
  plotting procedure.
- Simplified that procedure to two meaningful session variables only
  (`D435I_SERIAL` and `D435I_RUN_ID`). File paths are visible where used,
  numeric profiles come from one literal stream file, and strict subshell
  blocks stop multi-command steps without closing the user's terminal.
- Converted `docs/manual_test.md` into an evidence checklist so it cannot drift
  into a second conflicting command procedure.

### Runtime and data integrity

- Confirmed native OpenVINS state convention from the pinned source:
  `q_GtoI`, JPL, `x y z w`; no undocumented inversion is performed.
- Kept librealsense frame bytes in owned vectors before callback return.
- Moved recorder directory, CSV, and `INCOMPLETE` creation until after
  `RealSenseSource::start` succeeds. A missing device or unavailable stream
  profile now leaves no new dataset; a failure after capture starts still
  retains `INCOMPLETE`.
- Added image-buffer size, stride, and finite timestamp validation.
- Added finite motion-vector validation before capture callbacks and in the IMU
  synchronizer.
- Preserved raw acceleration and rotated synchronized acceleration from accel
  axes to gyro axes using librealsense extrinsics; translation is correctly not
  applied to a free vector.
- Kept acceleration interpolation strictly bracketed with no extrapolation.
- Made nonfinite initialized OpenVINS state a fatal error instead of silently
  looking like "no state yet."
- Added finite diagnostics validation and checked trajectory, diagnostics,
  application-log, metadata, device-report, and final flush results.
- Replaced the signal flag with standard `sig_atomic_t` handling and installed
  handlers before capture/replay work.
- Recorded pipeline-stop failures.
- Made the pipeline reuse the context that selected and configured the device,
  then replaced active-stream `query_devices()` polling with librealsense's
  device-change callback. A confirmed removal sets the disconnect state;
  unrelated SDK callback failures remain fatal but are not mislabeled as a
  hot-unplug.
- Recorder output now checks file creation, drains capture queues, shuts down
  the IMU synchronizer, verifies flushes, writes final metadata, and only then
  removes `INCOMPLETE`.
- Camera CSV/image identity now uses the shared RealSense frameset number rather
  than assuming the two per-stream frame counters are identical.
- Replay now requires complete format metadata, device report, resolved stream
  config, recorded stereo tolerance, matching serial, matching camera row
  counts, safe image names, and finite ordered measurements.
- Test temporary output names are unique rather than deleting a fixed shared
  path.
- RealSense startup exceptions now stop an already-started pipeline before
  unwinding, and Y8 allocation validates data pointers, dimensions, stride,
  and multiplication overflow first.
- Run output rejects duplicate or regressing estimator-state timestamps.
- Replay validates the complete resolved stream configuration and requires its
  serial to match both device metadata and calibration before creating output.
- Replay copies that dataset stream configuration into its run directory,
  matching the self-contained evidence contract already used by live mode.
- Removed duplicate synchronized-IMU CSV serialization in the recorder.
- Replay pacing now checks Ctrl+C at 20 ms intervals and returns 130 with an
  explicit partial-run message instead of claiming an interrupted replay was
  complete.
- Added `--require-build` and made the physical preflight bind its sample to
  one numeric serial and one stream-config file.
- Bound factory preparation to the same expected serial and stream
  resolution. Its structural path rejects unsafe distortion conversion,
  nonfinite or malformed intrinsics, and mismatched identities before writing.
- Rejected duplicate identity, dependency-path, and IMU-rate YAML keys so the
  validator and downstream configuration parser cannot resolve an ambiguous
  file differently.
- Made the Windows portable build use `--no-tests=error`; an empty test
  registration can no longer be reported as a successful build there.
- Set all three CTest presets to `noTestsAction: error`, closing the same
  false-success path for direct `ctest --preset ...` use.
- Added `CMakePresets.json` to the embedded content fingerprint so changes to
  build feature flags also invalidate stale executables.
- Made inspector export fail if sampling is interrupted/disconnected, a
  required stream is empty, or counters show malformed frames, camera-frame
  loss, timestamp rejection, or callback errors.
- Made recorder, replay, and live success transactional: dataset/run
  `INCOMPLETE` is removed only after explicit clean finalization. Replay/live
  also require initialization and at least one valid estimator state.
- Added strict replay checks for raw camera timestamp/frame ordering,
  canonical image filenames, decoded Y8 type, and recorded dimensions.
- Fixed ordered-dispatcher IMU coverage accounting for multiple nearby stereo
  frames that share the same already-fed interpolation bracket.
- Validated initialized quaternions and processing latency before output, and
  validated calibration dependency paths after canonicalization so symlinks
  cannot escape the serial-specific bundle.

## Commands executed and results

Executed successfully:

```text
git status/diff/config/submodule and exact-tag inspections
final staged diff, mode, artifact-exclusion, and diff-whitespace checks
PowerShell JSON parse of CMakePresets.json
project-local include and CMake source-path existence checker
Git Bash: bash -n on every .sh file
Historical v0.3.x: Python 3.13.12 py_compile on the then-present four
  repository/submodule .py files
Python 3.13.12: plot_trajectory.py tests/data/trajectory_sample.txt --summary-only
Python 3.13.12 + global Matplotlib 3.10.8: PNG generation and visual inspection
GCC 12.2 C++17 fresh v0.3.0 portable build with -Wall -Wextra -Wpedantic -Werror
post-Ubuntu-fix fresh portable rebuild under build/audit-ubuntu-fix-20260725
GCC 12.2 fresh v0.3.1 portable rebuild under
  build/audit-v0.3.1-runtime-fix-20260725
GCC 12.2 fresh calibration-fix rebuild under
  build/audit-intrinsics-fix-20260725
GCC fresh v0.3.2 content-identity rebuild under
  build/audit-v0.3.2-fingerprint-20260725
final GCC 12.2 v0.3.2 portable rebuild under
  build/audit-final-20260726
final GCC 12.2 v0.3.3 portable rebuild under
  build/audit-v0.3.3-readme-20260726
GCC 12.2 v0.4.0 portable core and dependency-disabled CLI rebuild under
  build/audit-v0.4.0-core-20260726
GCC syntax checks with OVRS_HAS_REALSENSE/OVRS_HAS_OPENVINS enabled
all four portable CLI --help invocations
all four portable CLI --version and unknown-option checks
missing-value CLI rejection check
Git Bash preflight option-parser positive/negative checks
Git Bash factory-export serial/resolution positive/negative checks
Historical v0.3.3: Git Bash syntax checks for the then-present 10 shell
  scripts and 20 README Bash blocks
```

Latest v0.5.2 current-tree results:

```text
Python 3.13.12: python tests/calibration_scripts_test.py
  15/15 passed, including manifest/source tampering, missing staging data,
  cross-serial, target-hash, Tg, missing-config, gravity-bound rejection, and
  the README catkin/rosrun command contract
Python 3.13.12: python -m compileall -q scripts tests
  passed; generated __pycache__ directories were then removed
Python 3.13.12: all 11 supported Python --help calls passed
GCC direct portable core:
  build/audit-v0.5.2-windows-20260727/ovrs_core_tests.exe
  18/18 passed with -Wall -Wextra -Wpedantic
GCC direct synthetic replay:
  build/audit-v0.5.2-windows-20260727/ovrs_mock_replay_test.exe
  1/1 passed
Four dependency-disabled CLIs: all eight --help/--version calls passed
Git Bash bash -n: 10 project scripts, 10 pinned OpenVINS scripts, and all
  45 README Bash blocks passed
Preflight parser: --help returned 0; both invalid camera/serial combinations
  returned the required exit 2
PyYAML/JSON/CMake path audit: 10 config YAML files, CMakePresets.json,
  13 referenced source paths, and five 40-hex commit pins passed
OpenVINS index gitlink and checkout:
  93adc241390d13e99232652cf05cbe18a93c7bea; submodule status clean
plot_trajectory.py: summary and 91,444-byte PNG passed; deliberate
  max-path violation returned the required exit 5
git diff --cached --check and git diff --check: passed
Project-owned trailing whitespace and personal serial/path scans: no matches
```

Latest v0.5.1 current-tree results:

```text
Python 3.13.12: tests/calibration_scripts_test.py ran 13 tests, all passed
Real 10-hour Allan capture, complete read-only validation:
  mode=imu-allan
  synchronized_rows=7193396
  duration_s=36000.713241
  stderr empty
  observed validator working set approximately 21 MB
Stationary diagnostics, each 60 seconds:
  start 300 s: 11989 samples, accel norm 9.443404146 m/s^2,
    gravity error 0.363245854 m/s^2, gyro mean norm 0.001393007 rad/s
  start 18000 s: 11990 samples, accel norm 9.411772931 m/s^2,
    gravity error 0.394877069 m/s^2, gyro mean norm 0.001407468 rad/s
  start 35400 s: 11989 samples, accel norm 9.408384432 m/s^2,
    gravity error 0.398265568 m/s^2, gyro mean norm 0.001417936 rad/s
  all reported validation: NOT_REQUESTED; no guessed tolerance was applied
GCC 12.2 direct portable core: 18/18 passed
GCC 12.2 direct synthetic replay: 1/1 passed
New recorder/preview/RealSense files passed strict -fsyntax-only using pinned
  librealsense headers and a local OpenCV API stub; not a link/runtime test
  (the first Windows invocation used a relative MinGW include path and failed
  to find the audit stub; the absolute-path rerun passed)
```

Latest v0.5.0 current-tree results:

```text
GCC 12.2 fresh portable core build:
  build/audit-v0.5.0-final2-20260726/ovrs_core_tests.exe
  18/18 passed with -Wall -Wextra -Wpedantic
GCC 12.2 fresh synthetic replay build:
  build/audit-v0.5.0-final2-20260726/ovrs_mock_replay_test.exe
  1/1 passed
GCC 12.2 fresh dependency-disabled ovrs_inspect, ovrs_record, ovrs_live,
  and ovrs_replay: compile, --help, and --version passed
GCC 12.2 -fsyntax-only with OVRS_HAS_REALSENSE=1:
  realsense_source.cpp and ovrs_inspect.cpp passed against pinned 2.56.5
  headers; only upstream-header warnings were emitted
GCC 12.2 -fsyntax-only with OVRS_HAS_REALSENSE=1:
  ovrs_record.cpp passed against pinned headers and the audit OpenCV API stub
GCC 12.2 -fsyntax-only with OVRS_HAS_REALSENSE=1 and OVRS_HAS_OPENVINS=1:
  active ovrs_live.cpp passed, including the final integrity-counter gate
Python 3.13.12: tests/calibration_scripts_test.py ran 9 tests, all passed
Python 3.13.12: py_compile passed for all 10 project Python files
Python 3.13.12: all eight Python CLI --help checks passed
plot_trajectory.py: finite summary/operator bounds passed; PNG generated and
  visually inspected; deliberate bound violation returned expected exit 5
Git Bash: bash -n passed for all 10 shell scripts and 33 README Bash blocks
PyYAML: all 10 tracked configuration YAML files parsed
PowerShell: CMakePresets.json/version/source/include checks passed
Git: OpenVINS tracked content clean at pinned commit; local Ceres and
  librealsense source revisions matched their central pins
```

The active dependency-enabled portion of `ovrs_live.cpp`, including the final
integrity-counter gate added in this audit, passed syntax checking. The
OpenCV-backed viewer, OpenVINS adapter, complete link, and runtime still could
not be compiled on Windows because their development dependencies are
unavailable. Only `scripts/build_ubuntu.sh` can close that verification gap.

Latest v0.4.0 test results:

```text
ovrs_core_tests: 17/17 passed
ovrs_mock_replay_test: 1/1 passed
all four --help commands: exit 0
librealsense-enabled realsense_source.cpp syntax: passed using pinned 2.56.5
headers as a system include (upstream warnings excluded from project -Werror)
Git Bash syntax: install, preflight, build, and preparation scripts passed
Python 3.13.12: both project scripts byte-compiled
stationary IMU analysis: 584 samples, 199.548253 Hz, acceleration norm
  9.415342258 m/s^2, configured gravity 9.806650000 m/s^2
known rotation-only replay with operator bound: expected validation FAIL,
  displacement 660.730498 m
PNG trajectory generation: passed, 91,868-byte equal-axis artifact visually
  inspected
v0.4.0 local-bundle migration: added matching state/serial/motion provenance
  without numeric changes; second invocation passed the idempotency check
```

The v0.4.0 portable rebuild compiled the core, both test executables, and all
four dependency-disabled application paths from current source with GCC 12.2,
C++17, `-Wall -Wextra -Wpedantic -Werror`. The new seventeenth core regression
executes concurrent state and diagnostics writes and verifies clean
finalization. CMake, OpenCV HighGUI, Eigen, Ceres, and a usable D435i runtime
are unavailable on the current Windows PATH, so the v0.4.0 full link graph,
native viewer, and physical capture require the Ubuntu rebuild.

The v0.4.0 calibration checks accepted the supplied physical export for the
operator's D435i serial (redacted from tracked documentation) at 848x480 and
rejected an incorrect expected serial and a
640x480 export paired with the 848x480 stream configuration. No acknowledged
replacement physical bundle was created. The existing ignored physical bundle
was migrated with a retained pre-v0.4.0 IMU backup and no numeric changes; a
synthetic creation fixture was verified and then removed.

The current RealSense source passed a C++17 syntax check using the exact pinned
librealsense 2.56.5 headers. The new HighGUI viewer and dependency-enabled
live/replay paths could not be compiled on this Windows host because OpenCV
headers/libraries are absent; they are not claimed successful.

The Ubuntu preflight script was also invoked on Windows. It returned
`PREFLIGHT_RESULT=FAIL` with the reason that Linux is required; this is the
intended result, not an Ubuntu test.

Final Windows-harness retries were kept separate from source results. The
system `bash.exe` is a WSL launcher and could not run in this session, so all
shell checks used Git Bash. Direct non-login Git Bash invocations lacked
`/usr/bin`; rerunning the validator through `bash -lc` found its required
tools and passed. A first CLI fingerprint assertion treated multiline native
output as a PowerShell array; joining the output corrected the harness and all
four binaries passed. A first plot smoke command used nonexistent
`--input/--output` options; rerunning the documented positional/`--save`
interface passed. None of these harness failures were counted as application
successes. Git ownership checks used command-local `safe.directory`; global
Git configuration was not modified.

Commands subsequently executed by the operator on Ubuntu 24.04.4:

```text
./scripts/preflight_ubuntu.sh
  prerequisites and pins passed
  OpenVINS content matched, but file-mode-only differences caused 1 error
  result: PREFLIGHT_RESULT=FAIL (errors=1, warnings=3)

./scripts/install_ubuntu_dependencies.sh
  apt completed successfully
  all requested packages were already installed

./scripts/build_ubuntu.sh
  Ceres 2.1.0 source and pin passed
  configure/generate failed: missing target CXSparse::CXSparse
  OpenVINS, librealsense, project build, and tests were not reached

ctest --preset linux-release --output-on-failure
  "No tests were found"
  not a successful test: build/linux-release had never been configured
```

The operator then reran the Ceres/backend patch:

```text
./scripts/preflight_ubuntu.sh
  local Ceres 2.1.0 and both disabled sparse backends passed
  388 CRLF-converted OpenVINS files were still reported as content changes
  result: PREFLIGHT_RESULT=FAIL (errors=1, warnings=2)

./scripts/build_ubuntu.sh
  Ceres reported "Building without SuiteSparse" and
  "Building without CXSparse"
  configure, generation, build, and repository-local install succeeded
  build then refused 2,909 apparent librealsense modifications
  librealsense configuration, OpenVINS, project build, and CTest were not reached
```

Local comparison with `core.autocrlf=false` reproduced 388 raw OpenVINS
differences, while `--ignore-cr-at-eol` reduced them to zero. The same narrow
comparison reports all three pinned checkouts semantically clean and still
rejects the root repository's real staged changes. The shared integrity helper
was then exercised by the operator's latest Ubuntu preflight:

```text
./scripts/preflight_ubuntu.sh
  OpenVINS exact revision and semantic content check passed
  local Ceres 2.1.0 and both disabled sparse backends passed
  local librealsense 2.56.5 passed
  OpenVINS cache policy check failed
  result: PREFLIGHT_RESULT=FAIL (errors=1, warnings=1)

./scripts/build_ubuntu.sh
  local Ceres 2.1.0 configured, built, and installed successfully
  local librealsense 2.56.5 configured, built, and installed successfully
  OpenVINS configured with Ceres 2.1.0 from .deps/install/ceres
  CMake printed "BUILDING WITHOUT ROS!"
  repository cache validator then incorrectly rejected Ceres_DIR because its
  type was UNINITIALIZED rather than PATH
  OpenVINS compilation/install, main project configure/build, and CTest were
  not reached

ctest --test-dir build/linux-release --output-on-failure --no-tests=error
  "No tests were found"
  not a successful test: build/linux-release had still not been configured
```

The corrected type-independent cache helper passed `bash -n` and was tested
against that exact generated OpenVINS cache: it accepted the correct
`Ceres_DIR`, `ENABLE_ROS=OFF`, and ROS discovery values, and rejected an
intentionally wrong `ENABLE_ROS=ON` expectation. This was a Windows-hosted Git
Bash check against Ubuntu-generated data, not a completed Ubuntu build.

The operator subsequently supplied physical-device output:

```text
rs-enumerate-devices
  confirmed the physical D435i, firmware 5.17.3.10, and BMI055 IMU
  USB descriptor: 3.2
  confirmed IR1/IR2 848x480@30 and 640x480@30 support
  confirmed gyro 200/400 Hz and accelerometer 63/250 Hz

ovrs_inspect ... --width 640 --height 480 --camera-fps 30
  "unknown option: --width"
  the executable predates the already-present v0.3.0 inspector CLI fix

ovrs_record ... --width 640 --height 480 --camera-fps 30
  capture started and then reported "failed to set power state"
  dataset remained marked INCOMPLETE
```

At that time the dataset path contained only empty `cam0`, `cam1`, and `imu`
directories plus `INCOMPLETE`; the audit did not delete it. The operator later
reused or recreated that path for the completed recording described below.
A fresh portable build under `build/audit-calibration-fix-20260725` compiled
all six targets with `-Werror`; core tests passed 15/15 and synthetic replay
passed 1/1. Integration-enabled syntax checks for the modified inspector,
recorder, and RealSense source passed against pinned librealsense headers
(with the existing minimal OpenCV audit stub for recorder syntax). Inspector
profile overrides passed top-level CLI validation. This is not a replacement
for rebuilding and running the modified hardware path on Ubuntu.

The v0.3.1 follow-up then removed all device-list polling from the active
capture path, constructed the pipeline with the same context used for device
selection, and registered `set_devices_changed_callback` for hot-unplug
events. The modified RealSense source passed a C++17 `-Wall -Wextra
-Wpedantic -Werror` syntax check against the pinned v2.56.5 headers.

The next Ubuntu run supplied this newer evidence:

```text
./scripts/preflight_ubuntu.sh --require-camera
  errors=0 warnings=1
  project 0.3.1, dependency pins, local Ceres, local librealsense,
  non-ROS OpenVINS cache, all CLI help/version checks, and D435i visibility pass
  system librealsense 2.58.3 is ignored for local 2.56.5

ovrs_record ... --output datasets/walk_loop_001
  Recording complete

ovrs_live / ovrs_replay
  stopped at the intended reviewed serial-specific calibration gate

ovrs_inspect ... --export-calibration ...
  aborted while listing Y16 1280x800:
  "No intrinsics are available for this stream profile"
```

After the preflight was strengthened to sample the project inspector, the
operator reran it and received the same Y16 abort. Preflight therefore ended
`PREFLIGHT_RESULT=FAIL` as intended. Inspection of
`build/linux-release/ovrs_inspect` found the old
`"profile was not found"` message and none of the new
`"intrinsics=unavailable"`/valid-calibration messages, proving that the
executable was stale despite its v0.3.1 label. Its object timestamp was later
than the edited source on the shared NTFS mount. v0.3.2 replaces semantic
version-only acceptance with current-source/cache/executable fingerprint
equality.

The next physical run confirmed the repaired inspector path:

```text
ovrs_inspect ... --serial SERIAL --export ... --export-calibration ...
  listed Y16 profiles without requesting intrinsics
  selected IR1/IR2 848x480 Y8 at 30 Hz
  stereo transform baseline approximately 49.9 mm
  sampled 10.4 s: stereo 28.5 Hz, gyro 190.5 Hz, accel 243.5 Hz
  malformed=0, rejected timestamps=0, callback errors=0
  timestamp monotonic/domain check PASS; all streams Global Time
  serial-specific BOOTSTRAP_UNVERIFIED export written

ovrs_live --config config/sensors/d435i_bootstrap.yaml ...
  rejected the committed unresolved serial placeholder as designed

ovrs_replay ... --serial SERIAL ...
  rejected an unsupported option; replay identity belongs to dataset metadata
```

The earlier completed dataset declared `complete: true`, local librealsense
2.56.5, USB 3.2, 848x480 Y8 at 30 Hz, 200 Hz gyro, 250 Hz accelerometer, and
Global Time for all streams. It contained 1,796 images per camera and 11,980
synchronized IMU rows. The same ignored path currently contains the
operator's newer 10.43-second attempt: 297 images per camera, 1,982
synchronized IMU rows, matching CSV/image counts, zero reported drops, and no
`INCOMPLETE` marker. The audit cannot determine from the supplied commands
when the earlier ignored directory was removed, so it does not claim that the
recorder overwrote it; current recorder code rejects a non-empty output path.

The inspector failure was traced to diagnostic enumeration calling
`get_intrinsics()` on every video profile. The fix queries only Y8
calibration profiles, tolerates unavailable non-target intrinsics, validates
the selected values once, and reuses them for export.

Still not executed successfully:

- ASan/UBSan binaries;
- the latest registered CTest output from `build/linux-release`;
- replay or live OpenVINS initialization with reviewed calibration;
- any firmware, EEPROM, kernel, Secure Boot, distribution, or global Python
  modification.

## What remains unverified

Still requiring Ubuntu execution:

- creation of the acknowledged local bootstrap bundle (its structural
  `--validate-only` path already passed against the physical factory export);
- the live/replay calibration-resolution gate with that prepared bundle;
- the latest registered CTest output and the sanitizer-enabled debug preset.

Known from the connected physical D435i:

- device enumeration, identity, firmware query, motion-module identity, and
  advertised profile listing work;
- the current connection reports USB 3.2;
- both IR sensors advertise `848x480@30` and `640x480@30`;
- the motion module advertises the requested 200 Hz gyro and 250 Hz
  accelerometer rates;
- a historical sustained 848x480 stereo/IMU recording completed for 60.55
  seconds; all streams reported Global Time, both camera sides stored 1,796
  images, and the summary reported zero frame and queue drops;
- the rebuilt inspector continued past non-intrinsic Y16 profiles, sampled
  stereo and IMU for 10.4 seconds with no malformed frames, rejected
  timestamps, or callback errors, and exported the 848x480 factory report;
- a subsequent 10.43-second recording stored 297 images per camera and 1,982
  synchronized IMU samples with zero reported drops.

Still unverified with the physical D435i:

- the new shared-context and hotplug-callback lifecycle in the real SDK
  during an actual cable disconnect;
- factory distortion-model compatibility and unit-specific transform review;
- live initialization, finite state/covariance behavior, latency, and replay
  agreement.

## Remaining fatal-risk candidates

1. The committed calibration remains intentionally non-runnable. A
   serial-specific camera file from `ovrs_inspect` still requires human review
   of distortion mapping, transform direction, and time offset.
2. Factory API output does not calibrate camera/IMU time offset or IMU noise.
   `BOOTSTRAP_UNVERIFIED` is limited to careful handheld validation.
3. Pinned upstream OpenVINS contains internal `std::exit` paths for conditions
   such as a negative covariance diagonal. These cannot be converted safely
   without changing estimator behavior and remain a physical-test risk.
4. The current complete dataset is 848x480. A 640x480 export is incompatible
   with it; the new resolution gate rejects this mismatch. The matching
   848x480 factory export exists and passes structural validation but still
   requires human review.
5. RSUSB avoids kernel patching but may require a reviewed udev rule. Running
   VIO as root is not an acceptable workaround.
6. The host kernel 7.0.0 is newer than the kernels listed for librealsense
   v2.56.5, and device firmware 5.17.3.10 was released with SDK 2.58.1. The
   stable v2.56.5 notes allow D400 firmware 5.17.0.10 or later, so neither fact
   proves incompatibility; both remain suspects if a current v0.5.1 capture
   still fails. Do not patch the kernel or flash firmware automatically.
7. No accuracy or initialization claim exists until a reviewed calibration,
   real capture, and replay have both run.
8. OpenVINS online calibration flags remain disabled. Enabling them to tune
   away this failed run would change estimator behavior without proving
   observability or replacing a unit-specific offline calibration.
9. Three sampled stationary windows in the real Allan capture show a
   repeatable acceleration norm about 0.36-0.40 m/s^2 below the configured
   gravity. One orientation cannot separate bias, scale, and cross-axis
   effects. The repository does not silently rescale it or enable OpenVINS
   online IMU-intrinsic estimation. Allan/Kalibr residual and bias review must
   reject promotion if the identity-after-factory-correction model is
   inadequate; independent multi-orientation intrinsic calibration would then
   be required.

## Exact Ubuntu 24.04 sequence

From the repository root:

```bash
./scripts/build_ubuntu.sh
./scripts/preflight_ubuntu.sh --require-build
ctest --test-dir build/linux-release --output-on-failure --no-tests=error
```

The dependency installer has already completed in the supplied Ubuntu log; run
it again only if preflight reports a missing reviewed package. The C++ runtime
remains independent of `.venv`. The build reconfigures the existing
Ceres, librealsense, and OpenVINS caches in place and runs CTest with an
explicit build directory; do not delete `.deps` for this fix. To repeat only
the registered tests afterward:

```bash
ctest --test-dir build/linux-release --output-on-failure --no-tests=error
```

After these three commands pass, continue with README section 2 in the same
terminal. `README.md` is now the only copyable inspect/calibration/record/
replay/live procedure. This report intentionally does not duplicate those
commands because two independently maintained procedures caused the earlier
serial, stream, and output-path drift. `docs/manual_test.md` is the matching
acceptance checklist.

## Expected success output and common failure signatures

Expected:

- the first preflight on a fresh Ubuntu installation may end in
  `PREFLIGHT_RESULT=FAIL` only for missing prerequisites; install them with the
  repository script, build, and rerun preflight;
- preflight ends in `PREFLIGHT_RESULT=PASS` or, before the camera is required,
  `PREFLIGHT_RESULT=PASS_WITH_WARNINGS`;
- `--require-build` verifies all applications, fingerprints, caches, and the
  registered tests;
- `--require-camera --serial SERIAL` runs the project inspector for that exact
  unit and stream file and reports a completed one-second sample; enumeration
  alone is not accepted;
- Ceres reports `Building without SuiteSparse` and
  `Building without CXSparse`;
- the local librealsense configuration does not print `Fetching recommended
  firmwares` and does not build `rs-fw-update` or other optional tools;
- local librealsense installation prints `Skipping ldconfig for
  repository-local librealsense installation` instead of attempting to write
  `/etc/ld.so.cache`;
- CMake reports Ceres `2.1.0`, OpenCV 4, and a non-ROS OpenVINS build;
- CTest reports all registered project tests passed;
- each `ovrs_* --version` first line reports that executable followed by
  `0.5.2`, and each reports the same 64-character source fingerprint;
- `ovrs_inspect` prints the selected serial, firmware, USB descriptor,
  profiles, extrinsics, sampled rates, and
  `Timestamp monotonic/domain check: PASS`;
- a completed recording prints `Recording complete:` and has no
  `INCOMPLETE` marker.

Common failures and meaning:

- `PREFLIGHT_RESULT=FAIL`: resolve each preceding `FAIL:` line before testing.
- `OpenVINS submodule is not initialized`: run the exact submodule command
  `git submodule update --init --recursive`.
- `OpenVINS submodule has tracked content modifications`: inspect
  `git -c core.fileMode=false -c core.autocrlf=false -C
  third_party/open_vins diff --ignore-cr-at-eol --name-status`; do not reset
  real content changes blindly.
- `librealsense has tracked content changes`: run the same command against
  `.deps/src/librealsense`. Empty output means only the narrowly ignored
  cross-filesystem metadata differs; any listed path is a real blocker.
- `Target "ceres" links to CXSparse::CXSparse`: the current script should
  reconfigure the cache with `CXSPARSE:BOOL=OFF`. Stop and preserve
  `.deps/build/ceres/CMakeCache.txt` if this signature recurs.
- `CMake did not resolve repository-local Ceres`: inspect the generated cache
  and rerun the supported build; do not uninstall system Ceres.
- `OpenVINS did not resolve repository-local Ceres`: with the current script,
  this means the cache value itself differs from `.deps/install/ceres`; a
  harmless `UNINITIALIZED` cache type alone no longer triggers this error.
- `Fetching recommended firmwares` or an `ldconfig` permission error during
  the local fallback indicates that an older build script is running. Stop,
  confirm the current checkout, and rerun without deleting `.deps`.
- A pre-existing `.deps/install/librealsense/bin/rs-fw-update` may be a stale
  artifact from the earlier run. The supported build never invokes it; do not
  use it for this application.
- `No tests were found`: this is an error, not a pass. Confirm
  `build/linux-release/CTestTestfile.cmake` exists after a successful build.
- `unknown option: --width`, Y16 `No intrinsics are available`, an executable
  reporting an older version, or a source-fingerprint mismatch means
  `build/linux-release` is stale. Run `./scripts/build_ubuntu.sh`; v0.5.2
  requires cache, current source, and every executable to share one content
  fingerprint.
- `failed to set power state` from a v0.3.0 capture: rebuild v0.5.2, which no
  longer polls `query_devices()` with a separate context during streaming. If
  it recurs on v0.5.2, preserve the complete log and kernel messages; the
  remaining cause is in the SDK/USB runtime and is not bypassed automatically.
- `No intrinsics are available for this stream profile` immediately after a
  Y16 profile: the inspector binary predates the current calibration fix.
  Rebuild; non-Y8 capability profiles no longer abort enumeration.
- `unsupported RealSense distortion model`, `nonzero fifth Brown-Conrady
  coefficient`, or incompatible coefficients/intrinsics: factory data cannot
  be represented safely by the OpenVINS bootstrap model. Do not rename the
  model or truncate coefficients; use a reviewed Kalibr calibration.
- `camera calibration cam0/cam1 resolution ... does not match stream
  resolution ...`: use a reviewed calibration exported for the dataset/live
  profile. Do not edit only the resolution field or bypass the check.
- `no RealSense device found` / `No device detected`: check USB 3 cable, port,
  power, `rs-enumerate-devices`, and udev permissions.
- `requested stereo Y8 profile is unavailable`: use inspector output; do not
  invent a profile. Reconnect over USB 3 or explicitly pass one common IR1/IR2
  profile.
- `Output directory already exists and is not empty`: preserve that attempt and
  choose a new output path. The recorder never overwrites a dataset.
- `No dataset was initialized`: capture did not start, so the current recorder
  created no `INCOMPLETE` dataset for that failure.
- `timestamp domain changed` or `duplicate or regressing timestamp`: stop and
  inspect device clock/USB behavior; do not widen tolerance.
- `IMU/camera calibration state/serial does not match`: keep live/replay
  blocked until all three reviewed YAML files identify the same unit and
  calibration state.
- `Dataset is marked INCOMPLETE`: preserve and manually recover/review it;
  never delete the marker blindly.
- run-directory `INCOMPLETE`: replay/live did not finalize successfully.
  Preserve it and inspect the log; it is not a successful estimator result.
- `dataset metadata does not declare a complete ovrs-euroc-like-v1 recording`:
  the dataset is missing or not finalized.
- `OpenVINS produced a nonfinite initialized state`: stop immediately and
  review calibration, timestamps, units, and data integrity.

No repository script patches kernel modules, changes firmware, writes EEPROM,
disables Secure Boot, upgrades the distribution, removes system packages,
installs to `/usr/local`, or modifies system Python.

## 2026-07-28 interactive-viewer and milestone-documentation audit

The fixed isometric trajectory panel was replaced by an interactive global XYZ
view without changing estimator state or its ingestion path. OpenCV HighGUI
delivers mouse/key events and renders the result on the application main
thread. Dependency-light view-controller math in `ovrs_core` performs orbit,
pan, cursor-centred zoom, fit, and reset. The viewer adds a world-locked XY
ground grid, labelled XYZ axes, start/current markers, axis spans, accumulated
3D path, displacement, and view-angle/zoom status.

README, the selected-runtime contract, architecture notes, manual acceptance
checklist, changelog, and both live/replay CLI help now agree on the controls:
left-drag orbit, middle/right-drag pan, mouse-wheel zoom, `F` fit, `R`/`0` or
double-left-click reset, and `q`/Escape clean shutdown.

The reproducibility contract now binds controlled diagnostic runs to:

- D435i serial `843212070146` and the five recorded SHA-256 values;
- the selected 848x480 stereo/IMU stream policy;
- fixed -4.900203074 ms camera-IMU offset with online estimation off;
- initialization ZUPT plus the reviewed visually gated one-second stop
  recovery;
- sharp static IR texture, even illumination, stationary initialization, and
  smooth motion without blur or occlusion;
- a marked-pose 120-second capture: 0-20 seconds still, 20-50 seconds smooth
  outbound, 50-65 seconds smooth return, and 65-120 seconds still;
- replay-first acceptance against operator-declared physical path and final
  displacement bounds.

Fast handheld motion remains outside the accepted envelope. Numeric health,
zero drops, or a visually flat plot are not substituted for physical
agreement.

Validation performed after the change:

- `./scripts/build_ubuntu.sh`: passed and rebuilt the OpenCV/OpenVINS
  executables from the current source fingerprint;
- Linux release CTest: 4/4 passed;
- portable-core configure/build/CTest: 4/4 passed;
- selected-runtime, acquisition, and reviewed-patch SHA-256 checks: all five
  passed;
- `./scripts/preflight_ubuntu.sh --require-build`: passed with one expected
  warning because no D435i was connected;
- a stored-dataset GUI replay opened and rendered the new viewer, then
  operator `q` interruption retained an incomplete partial run as designed;
- orbit/pan/zoom anchoring, clamping, fit, and reset passed the core regression
  test.

The viewer-only change originally had no new connected-camera run. Later
connected runs supplied hardware evidence, but the 2026-07-29 pitch-motion
invalidation above proves that their successful completion was not an
estimator-accuracy closure. This audit does not upgrade the calibration label
or expand the accepted motion envelope.

## 2026-07-28 fast-motion capture and 90 Hz replay audit

A complete 30 Hz raw VIO capture reproduced the reported drift independently
of the live viewer. Its transport counters were clean, but the camera saw
severe motion blur followed by low-texture, saturated ceiling views. Replay
crossed 1 m displacement at about 41.1 seconds and OpenVINS reached the 3 m/s
safety gate near 69.2 seconds, after accumulating 51.6 m displacement. This
separates loss of visual constraint from transport, serialization, viewer, and
non-finite-state failures.

A later physical capture from D435i `843212070146` used 848x480 Y8 stereo at
90 Hz, gyro at 200 Hz, and accelerometer at 250 Hz. It finalized with 8083
stereo pairs, 17974 synchronized IMU rows, and zero dropped camera frames,
malformed frames, rejected timestamps, callback errors, or queue drops. One
leading stereo pair preceded the first synchronized IMU sample by 3.317 ms.

Replay was repaired to decode and validate such a leading prefix, skip it
without changing any timestamp, and report the count. Missing IMU coverage
after replay begins remains fatal. The selected fixed-offset estimator then
processed 8082 stereo pairs for 87.83 seconds and finalized normally:

- estimated path length: 5.784 m;
- final displacement from initialized origin: 0.486 m;
- maximum estimated speed: 0.544 m/s;
- final 17.8-second displacement: 4.2 mm;
- average processing latency: about 5.4 ms, below the 11.11 ms camera period;
- final fixed camera-IMU offset: -4.900203074 ms;
- zero rejected non-finite states.

The 30 Hz and 90 Hz records are separate physical captures, not an
identical-motion frame-rate A/B, and neither has external pose ground truth.
The later recorded run proves that the 90 Hz profile avoided catastrophic
runaway under that capture and had replay compute margin; it does not prove
absolute trajectory accuracy or zero drift.

A subsequent connected-camera 90 Hz live run failed the next gate. It remained
near the origin for roughly 19.2 seconds of initialized stationary state, then
estimated speed crossed 0.05, 0.5, and 3 m/s within about 1.7 seconds after
motion began. The run stopped at the configured safety gate with 2.153 m final
displacement and 2.374 m estimated path. Stereo ran at 88.6 Hz and synchronized
IMU at 197.1 Hz with zero queue drops, rejected timestamps, or non-finite
states. Because live mode did not retain raw images or IMU rows, this result
does not distinguish loss of visual information from a motion-dependent model
or calibration error. A raw VIO capture of the same failing physical motion is
the required next diagnostic; tuning before that capture would be assumption.

## 2026-07-28 raw failure capture and gyro-scale diagnosis

The required raw capture was recorded as
`datasets/vio_live_failure_repro_20260728T145412Z`. It contains 4039 stereo
pairs and 8980 synchronized IMU rows over 45.50 seconds at effective rates of
88.77 Hz and 197.36 Hz, with zero dropped camera frames, malformed frames,
rejected timestamps, callback errors, stereo queue drops, or IMU capacity
drops. Selected-runtime replay reproduced the live failure and reached the
configured 3 m/s safety gate near 21.3 seconds.

Stereo geometry was independently checked with the documented camera models
and camera-to-IMU transforms. The correct transform direction produced
positive triangulated depth for all accepted points, approximately 3.0-3.4 m
median depth, and roughly 0.4 px reprojection error; using the inverse was
worse. Feature tracking plus calibrated stereo PnP over a 0.501 s interval
measured 12.019 degrees of camera rotation. An essential-matrix estimate
measured 11.720 degrees, while integration of the recorded SDK gyro measured
24.203 degrees. This approximately 2x relationship was stable over shorter
subintervals and used many PnP inliers.

The mismatch is not a universal calibration scale: both post-device-update
Kalibr repeat captures and the earlier completed 90 Hz VIO capture showed
approximately one-to-one visual and gyro rotation. A diagnostic dataset copy
changed only the gyro CSV values by 0.5; selected replay then completed all
42.83 seconds without the speed gate, estimating 5.188 m path and 1.068 m
final displacement. That intervention establishes causality for the runaway
but is not an accuracy result and is not promoted.

The connected D435i and pinned SDK expose
`RS2_OPTION_GYRO_SENSITIVITY` indices 0 through 4. The previous runtime did not
explicitly set or record that dynamic option, so the failing legacy capture
cannot prove which acquisition state produced its rad/s values. OVRS now:

- sets configured sensitivity level 1 before pipeline start;
- validates it against the device-advertised option range;
- reads back the exact active value;
- records requested, available, active, and descriptive values;
- rejects startup and replay-policy mismatches instead of guessing a scale.

A physical inspect run confirmed requested/active level 1
(`30.5 mDeg/s/LSB`). A subsequent 45.5-second physical capture recorded the
same exact values, 4040 stereo pairs, 8984 synchronized IMU rows, and zero
transport-integrity errors. Its full 42.83-second stationary replay estimated
0.219 m path and 14.15 mm final displacement. Because the camera was not
meaningfully moved, this validates option control, capture, and stationary
estimation only.

### Moving sensitivity, scale, and stop-recovery closure

The later moving gate changed the earlier decision with stronger evidence.
An explicit level-1 moving capture and
`datasets/vio_sensitivity0_final_20260728T164343Z` at level 0 both retained
clean transport and independently showed approximately 2x SDK gyro rotation.
Twelve accepted calibrated visual/gyro windows in the level-0 dataset had a
median ratio of 2.017 and mean 1.993. The original replay reached the 3 m/s
gate; a copy changing only recorded gyro values by 0.5 completed 57.83 s.

At that stage OVRS promoted an explicit `gyro_scale_factor` acquisition field,
default 1.0 generically and selected as 0.5 for serial `843212070146`.
RealSenseSource applies it to the SDK gyro vector before recording and
synchronization. Device report and dataset metadata record the configured and
applied values. Replay consumes CSV values unchanged, so historical evidence
is never double-scaled. Calibration capture/export validation binds the same
factor across Allan and camera-IMU inputs.

The later official EEPROM recalibration and post-calibration A/B documented at
the top of this report reject that serial-specific `0.5` selection and restore
the selected factor to `1.0`.

The native corrected capture
`datasets/vio_gyro_scale05_final_20260728T165508Z` recorded:

- 5388 stereo pairs and 11982 synchronized IMU samples over 60.56 s;
- effective rates 88.97 Hz stereo and 197.86 Hz synchronized IMU;
- sensitivity requested/active 1 and scale configured/applied 0.5;
- zero camera, queue, malformed, timestamp, callback, duplicate, regression,
  invalid-value, missing-bracket, or synchronizer-capacity errors;
- representative visual/gyro rotations of 13.667/13.931 and
  11.869/11.683 degrees.

With initialization-only ZUPT, replay still reached the 3 m/s gate at 45.62 s.
Debug then proved the reviewed continuous-ZUPT patch's accumulated feature
test always reached zero features after normal MSCKF cleanup. The patch now
measures duration across consecutive per-frame checks, resetting on any
moving or unknown frame. The selected conservative policy uses 2 px maximum
disparity, more than 20 common tracks, one second minimum duration, chi-square
multiplier 10, and a 3 m/s candidate/safety bound.

Exact selected replay completed all 57.83 s and ended at 0.0033 m/s, but its
60.893 m path and 10.677 m final displacement invalidate the former closure
claim. The gyro-scale evidence remains useful; stop recovery is only a final
velocity constraint and is not a position-drift fix.
