# Pre-push audit summary

This file is the current review entry point. Detailed historical investigations
and superseded conclusions are preserved in
[docs/audit_history.md](docs/audit_history.md); they are not deleted or
silently rewritten.

## v0.6.0 research-branch validation on 2026-07-30

The research branch was created from clean commit `f5a6e2c`; local `main` and
`origin/main` both remained at that commit after the switch. No commit or push
was performed during this validation.

The branch adds a backend-neutral offline export boundary and the first
ORB-SLAM3-specific adapter. The pinned upstream ORB-SLAM3 stereo-inertial
runner was built in the ignored dependency tree and exercised on an existing
capture. OKVIS2 has not been built or run, and no accepted map,
relocalization, or loop-closure result is claimed.

Completed checks:

- portable-core configure/build and all 5 registered CTest cases passed;
- full Ubuntu/OpenVINS release configure/build and all 5 registered CTest
  cases passed;
- all four release executables reported project version 0.6.0;
- twenty-six dependency-light VSLAM workflow regression cases passed;
- repository Markdown, Bash, selected-runtime, submodule-cleanliness, and
  personal-path checks passed;
- `git diff --check` passed;
- the existing complete
  `datasets/vio_rsusb_sensitivity_fix_20260729T1315Z` recording exported to the
  backend-neutral EuRoC layout with 2,241 stereo pairs and 4,984 synchronized
  IMU rows;
- the exporter recorded one source stereo pair before the synchronized IMU
  range as skipped, matching the existing replay coverage rule;
- the exact ORB-SLAM3 commit
  `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` built an x86-64
  stereo-inertial runner; the later atlas experiment uses the separately
  pinned serialization-integrity patch; and
- the adapter preserved all 2,241 exported stereo pairs after applying the
  selected `-4.900203074 ms` camera-to-IMU offset.

The export is local ignored output and declares
`state: EXPORTED_NOT_EVALUATED`.

The ORB-SLAM3 smoke process returned zero but repeatedly rejected inertial
initialization for insufficient acceleration or motion and reset its IMU map
five times. Its final 91 frame poses and four keyframe poses are partial
diagnostic output, not an accepted trajectory. This result proves that the
pinned backend, generated settings, images, and IMU input execute together; it
does not prove tracking accuracy, loop closure, or relocalization.

A later connected-camera preflight passed with zero errors and warnings. A new
150.62-second VIO recording then completed with 13,476 stereo pairs, 29,960
synchronized IMU rows, and zero capture-integrity counters. The upstream
stereo-only backend tracked all 13,475 exported pairs in one map, although its
process segfaulted during shutdown after writing both trajectory files. It did
not report loop detection.

The first controlled sequence remained below the acceptance gate. The 90 Hz
baseline reset its IMU map 40 times. A same-recording stride-three view retained
all IMU rows and processed 4,492 stereo pairs at 30 Hz, but still reset 20
times. A zero camera-time-offset A/B reset four times and produced one longer
final map; an identical repeat reset 22 times and ended with only 47 frame
poses. The calibrated offset therefore remains the default, and no
stereo-inertial or loop-closure pass is claimed.

A second connected capture changed the motion envelope rather than the
extractor or calibration. It completed 120.57 seconds with 10,782 stereo pairs,
23,971 synchronized IMU rows, and zero integrity counters. Two identical
calibrated-offset, 30 Hz adapted runs each initialized one map, completed both
inertial BA stages, reported zero reset or tracking failure, and wrote 3,110
frame poses through the end of the recording.

One run did not detect a loop. The repeat rejected two loop candidates and
sent a third through ORB-SLAM3's loop-correction path. Its estimated return
displacement was 0.073 m versus 0.270 m in the run without a correction.
Because the operator-returned path had no independent reference and the loop
event was not repeatable across identical runs, these values are diagnostic
only. The branch now has a repeatable stereo-inertial initialization baseline,
not an accepted loop-closure or accuracy result.

The project-owned runner and evaluator now capture the backend exit status,
hash the adapter, log, status, and trajectory files, and emit an explicit
`ovrs-orbslam3-result-v1` state. The two runs are classified as
`TRACKING_PASS_NO_LOOP_CORRECTION` and
`TRACKING_PASS_LOOP_CORRECTION_NOT_REFERENCE_VALIDATED`; both retain
`independent_reference_present: false`.

A third connected capture used the same two rigid contact surfaces at the
start and end. The reference values were not consumed by the estimator. It
completed 150.65 seconds with 13,475 stereo pairs,
29,961 synchronized IMU rows, and zero integrity counters. Two identical
calibrated-offset 30 Hz runs passed the tracking gate with one map, both
inertial BA stages complete, zero reset, and zero local-map failure. Neither
run detected a loop.

The result manifests bind the automatically captured zero exit status, backend
executable, dynamically linked library, vocabulary, adapter, log,
trajectories, and independent-reference file. Estimated return residuals were
0.0129 m and 0.0165 m, with 2.88 and
2.75 degree rotation. Both are consistent with the recorded 0.02 m and
5 degree fixture tolerances. This is a repeatable referenced endpoint result,
not ATE/RPE or a loop-closure pass.

The offline adapter now has a guarded atlas path. It can save a first atlas,
stage a hashed atlas into a second recording's adapter, and request a new
revision through ORB-SLAM3's upstream settings. The runner refuses overwrite,
verifies settings, backend patch, and input hashes, and writes a provisional
atlas manifest only after the tracking gate passes. For a loaded atlas, the
evaluator also requires a completed cross-session map merge and one final map.

The first upstream atlas save passed those gates but its reload segfaulted
during `Map::PostLoad()` because serialized graph references could name
objects omitted from the saved vectors. Shutdown could also serialize before
all graph-mutating threads finished. The invalid atlas was retired. A reviewed
patch now waits for shutdown completion, serializes a self-contained nonbad
graph, and rejects unresolved references instead of inserting null pointers.
The adapter and preflight bind that patch by SHA-256.

A stricter follow-up exposed one remaining lifecycle fault after a completed
merge: the new but empty active map could remain in the runtime registry even
though it was excluded from serialization. The evaluator correctly rejected
that run as `ATLAS_MERGE_NOT_ESTABLISHED` because the final atlas reported two
maps. The patch now collects empty maps first and marks them bad only after
iteration, avoiding container invalidation while removing them before the
final registry count and serialization.

A fresh atlas built with the final patch contained 71 keyframes and wrote
3,807 active-session frame poses after both inertial BA stages, with zero
IMU-map reset or local-map failure. Reloading it completed deserialization,
one cross-session merge, and a one-map save with 123 keyframes. Loading that
merged revision and saving a third generation repeated those gates with one
merge, one final map, and 192 keyframes. Both reloads wrote 3,807
active-session frame poses with zero reset or tracking failure. The supported
wrapper manifests bind the patch, executable, shared library, input atlas, and
each output atlas by SHA-256. This validates offline save/reload, chained
serialization, and multi-session merge on one repeatedly played recording; it
does not measure false merges, relocalization accuracy, or behavior on a
distinct revisit sequence.

After the final source changes, the full Ubuntu build and portable build each
passed all five registered tests. Build and connected-camera preflight then
passed without warnings; physical D435i serial `843212070146` completed the
one-second inspector sample. This verifies current build provenance and a
short hardware stream, not SLAM accuracy or a live atlas workflow.

This deliberately does not call the upstream localization-only switch. The
pinned inertial tracking source marks that mode unavailable, and loading an
atlas creates a new active map. The implemented contract is therefore
multi-session map merge, not a claim that relocalization-only startup works.
The atlas result uses offline playback of a previously recorded connected-camera
sequence. No new live-camera atlas run was performed during this test.

The later live-path audit found a separate continuity defect in the project
adapter: it wrote every upstream `Tracking::OK` pose to one file even before
inertial initialization and across active-map clears. Upstream retains the map
ID when it clears the active map, so map ID could not reveal the discontinuity.
This did not feed OpenVINS or an EKF, but the output file could be mistaken for
a continuous trajectory.

The reviewed backend patch now exposes a monotonic applied-reset count, pending
reset state, and the exact stereo-inertial acceleration-init delta/threshold.
The project adapter publishes its canonical TUM trajectory only after three
continuous seconds of initialized inertial state with zero resets. Any applied
or pending reset or initialized-to-uninitialized regression rejects the run
permanently. Pre-initialization visual poses are retained in a separately named
diagnostic file. The upstream `0.5 m/s^2` threshold remains unchanged and fast
initialization remains disabled. A dependency-light state machine covers
stability, pre-init reset, post-acceptance reset, pending reset, and inertial
regression cases.

After this patch, full Ubuntu/OpenVINS and portable-core builds each passed all
five registered CTest targets. Connected D435i `843212070146` preflight passed
with zero errors and warnings. A 31.42-second stationary live ORB run processed
927 submitted stereo frames at 29.51 Hz and 6,188 synchronized IMU samples at
196.96 Hz with zero camera, queue, timestamp, and in-stream IMU-coverage
errors. Its exact acceleration delta never passed the gate (maximum
`0.040359 m/s^2` versus `0.5 m/s^2`), so it correctly ended
`EXPERIMENTAL_RUN_FAILED`, retained `INCOMPLETE`, and published no canonical
trajectory.

Because the lifecycle patch and shared-library hash changed again during the
re-audit, atlas evidence was rebuilt from scratch rather than inherited from
the older binary. The fresh build, reload, and second-reload runs each
processed 3,807 active-session poses from 4,491 input frames with terminal
coverage, both inertial BA stages complete, zero reset, and zero local
tracking failure. The build saved one map with 70 keyframes. Both reloads
completed exactly one cross-session merge and saved exactly one final map,
with 125 and 181 keyframes respectively. Manifests bind the complete reviewed
patch SHA-256
`e94234bae11c24527e09fc3a7ece18be65f2e3715659b486d5fdebdf4dc892ca`,
shared-library
SHA-256
`a577b3540793e648cfb14cee1a730535d2246d720c78278f02768f942c8ae72f`
and the three successive atlas hashes. The first two revisions therefore have
a successful subsequent reload; the third saved revision remains provisional
until a future reload. This validates identical-data offline continuity and
two chained serialization boundaries. A new live moving-camera trajectory
test was not run while the connected camera remained stationary, so live
motion robustness and any drone/EKF use remain unvalidated.

## Current verdict

The live runtime remains a standalone D435i/OpenVINS diagnostic VIO system.
The v0.6.0 branch now has an offline markerless-SLAM benchmark foundation and
a repeatable ORB-SLAM3 stereo-inertial initialization result. It does not yet
have an accepted loop-closure or accuracy result. The selected serial-specific
bundle is reproducible but still `BOOTSTRAP_UNVERIFIED`; it must not be
described as drift-safe or accuracy-certified.

The current implementation:

- keeps RealSense callbacks limited to bounded frame ownership and enqueueing;
- gives the ordered dispatcher sole ownership of OpenVINS ingestion;
- preserves raw device timestamps beside normalized timestamps;
- rejects malformed input, timestamp regression, non-finite estimator states,
  excessive estimated speed, and excessive accelerometer bias;
- keeps OpenVINS pinned at v2.7 with ROS disabled;
- builds only the pinned repository-local librealsense with the reviewed RSUSB
  gyro-sensitivity patch and rejects system-library fallback;
- separates process/CLI helpers from calibration and estimator-bundle
  validation without changing the validation contract;
- requires the selected stream to retain gyro sensitivity 1 and project gyro
  scale 1.0;
- records stream, serial, gyro sensitivity, gyro scale, timing, calibration,
  and health provenance;
- treats visual-support status as a continuity diagnostic, not a pose-quality
  certificate;
- uses ZUPT only to constrain velocity after conservative stationarity gates;
  it does not claim to recover accumulated position drift.

## Connected-camera evidence boundary

An unpatched zero-drop capture reproduced an approximately 2× gyro/visual
rotation ratio. After correcting the pinned RSUSB feature-report encoding, a
connected capture produced three strong gyro/visual comparisons within about
1.6%, and replay completed without the prior speed runaway. This establishes
the tested angular-scale correction, not absolute trajectory accuracy.

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
constraint, mapping, or loop closure. The new branch investigates markerless
mapping and loop closure offline; neither is part of the accepted live runtime.

## Review and validation contract

Before a commit is considered ready:

```bash
git diff --check
./scripts/verify_selected_runtime.sh --serial 843212070146
cmake --preset portable-core
cmake --build --preset portable-core
ctest --preset portable-core --output-on-failure
./scripts/build_ubuntu.sh
ctest --test-dir build/linux-release \
  --output-on-failure \
  --no-tests=error
./scripts/preflight_ubuntu.sh \
  --require-build \
  --require-camera \
  --serial 843212070146 \
  --stream-config config/sensors/realsense_streams_vio_90hz.yaml
```

Also review:

- every documented Bash block with `bash -n`;
- all local Markdown links and math delimiters;
- CLI `--help` output against documented commands;
- public headers and CMake source registration after structural refactors;
- the clean pinned OpenVINS submodule and the patched `.deps` build checkout;
- the final diff for personal paths, generated artifacts, secrets, and
  fabricated measurements.

A build or replay does not substitute for a connected-camera test. No EEPROM
or firmware write is part of this repository's validation workflow.

## Validation completed on 2026-07-29

The current source and pinned dependency patches passed:

- the Ubuntu release build and all 4 registered CTest cases;
- the portable-core build and all 4 registered CTest cases;
- the repository policy, Markdown-math, and documentation checks;
- all selected-runtime hash and semantic sensitivity/scale checks;
- `git diff --check` in both the project and OpenVINS submodule;
- CLI-help and source-fingerprint checks for all four applications;
- all 26 Python calibration tests plus Python parsing and Bash syntax checks;
  and
- replay instrumentation checks whose trajectories remained byte-identical
  before and after adding read-only MSCKF update statistics.

The structural cleanup replayed
`datasets/vio_rsusb_sensitivity_fix_20260729T1315Z` again. Its trajectory and
the first 48 state columns were byte-identical to the pre-refactor replay; only
the measured processing-latency column differed.

The final connected-camera preflight passed with zero errors and zero warnings.
It confirmed D435i serial `843212070146`, completed the one-second inspector
sample, and verified that all three hardware executables load the exact
repository-local patched librealsense. A 323.97-second live-viewer run then
completed at 89.59 Hz stereo and 199.20 Hz synchronized IMU with zero drops.
The final state was healthy and nearly stationary, but its 1.709 m estimated
endpoint and unmeasured physical path prevent any position-accuracy claim.

## ORB-SLAM3 live BA2 continuity finding on 2026-07-30

The connected D435i motion run
`runs/orbslam3_live_final_motion_20260730_b` completed 81.55 seconds with
2,425 ORB stereo submissions, 16,182 synchronized IMU samples, zero capture
drops, zero lost frames, zero active-map resets, and no timestamp regression.
The first revision of the live gate accepted 1,784 poses after
`isImuInitialized()` had remained true for three seconds.

Post-run frame-to-frame inspection invalidated that provisional acceptance.
At timestamp 32.552259465 the streamed map pose moved 0.566411 m in 0.033359 s,
while every other large step was below 0.025 m. The discontinuity coincided
with ORB-SLAM3 `VIBA 2`: upstream marks the map IMU-initialized before its
second inertial bundle adjustment, and that later stage can change the map
scale and gravity alignment without incrementing the active-map reset counter.
The run is retained as defect evidence and must not be treated as accepted
continuous odometry despite its original summary state.

The corrected contract waits for the active map's inertial BA2-complete flag,
then requires the configured continuous stability window. It also records the
active-map big-change index and rejects a candidate if loop closure or global
BA changes the map after streaming acceptance begins. This remains an
experimental pure ORB-SLAM3 map-frame output; it is not an OpenVINS correction
or a flight-EKF odometry interface.

The fresh connected run
`runs/orbslam3_live_final_motion_ba2_gate_20260730_g` then exercised the
corrected contract with continuous translational motion followed by a
stationary hold. It completed 67.61 seconds with 2,011 submitted stereo
frames, 13,422 synchronized IMU samples, zero transport drops, zero lost
frames, zero reset or pending-reset observations, both inertial BA stages
complete, and no active-map correction. The post-BA2 stability gate published
1,345 canonical poses over 44.87 seconds.

The diagnostic pre-gate stream contains a 166.736 m one-frame BA2 map-frame
correction. That correction is absent from the canonical file: its timestamps
are strictly increasing, maximum adjacent translation is 0.026475 m,
quaternion norm error remains below 1e-6, and the motion-to-stationary
transition remains `Tracking::OK`. The canonical relative path is 6.94 m
inside a 0.393 by 0.363 by 0.253 m axis-aligned box. Those values establish
stream continuity for this run, not metric accuracy; there was no external
ground truth, and the large pre-gate correction reinforces that the diagnostic
map frame must not be treated as flight odometry.

The follow-up independent live evaluator now recomputes that pass from the raw
tracking and IMU rows rather than trusting `run_summary.yaml`. On the retained
successful run it independently found 2,011 frame records, zero loss/reset or
state regression, 1,345 timestamp-bound canonical poses, the same 44.866 s
duration and 0.026475 m maximum adjacent translation, and matching bundle,
settings, backend patch, shared-library, serial, and artifact hashes. On the
retained failed run
`runs/orbslam3_live_final_motion_ba2_gate_20260730_a`, it preserved a coherent
`LIVE_GATE_FAILED` result with the incomplete marker, two active-map resets,
pending-reset observation, inertial regression, unfinished BA2, and no
canonical trajectory. This validates evaluator discrimination; it does not add
trajectory-accuracy evidence.

The live application now emits an operator cue only when canonical acceptance
opens. A physically referenced return-to-start run must finish initialization,
hold the camera at the selected home fixture, wait for that cue, traverse the
loop, and return to the same fixture before shutdown. No such referenced live
run has been executed yet. ORB output remains isolated from OpenVINS and any
flight EKF.

The re-audit then closed the remaining executable-provenance limitation for
future runs. Before constructing ORB-SLAM3 workers, the application now writes
a launch manifest that binds the running executable, actual backend shared
library, vocabulary, generated settings, source bundle manifest, and compiled
source fingerprint. The evaluator requires those capture-time hashes for the
new provenance format and rejects a subsequently changed executable, library,
vocabulary, settings, bundle, or launch manifest. The retained July 30 runs
predate this contract and therefore remain explicitly labelled legacy
unattested; their continuity/reset evidence is not retroactively upgraded.

The same re-audit reproduced a startup SIGSEGV in a restricted hardware
environment and captured its stack with AddressSanitizer. The fault was below
ORB-SLAM3: librealsense 2.57.3 logged a failed `libusb_init()` and then called
`libusb_get_device_list()` with a null context. The reviewed librealsense patch
now throws a backend error before enumeration and also checks negative device
list results. `ovrs_orbslam3_live` constructs the hardware context before ORB
workers, and an exception-safe guard coordinates cleanup after worker
construction. The ORB patch serializes concurrent viewer/application shutdown
and makes repeated shutdown calls idempotent. It also breaks the circular-wait
case in which the viewer Stop action overlaps an externally owned shutdown.

A fresh connected, stationary headless smoke run exercised those changes for
26.45 seconds. It submitted 777 ORB stereo frames from 2,333 source pairs and
5,186 synchronized IMU samples at 29.38, 88.21, and 196.08 Hz respectively,
with zero queue drops, timestamp rejection, in-stream IMU coverage loss,
tracking loss, reset, or pending-reset observation. Shutdown completed without
a crash. Because the stationary camera never crossed the unchanged 0.5 m/s²
upstream acceleration gate, the application and independent evaluator both
correctly classified it as failed initialization with no canonical pose. This
is connected transport and lifecycle evidence, not motion, continuity, homing,
or accuracy evidence.

A connected viewer smoke used the complete patch and instrumented source
implementation. It reached `Starting the Viewer` and streamed for 21.47
seconds, receiving 1,865 source stereo pairs, submitting 621 pairs to ORB, and
synchronizing 4,147 IMU samples with zero queue drop, timestamp rejection,
reset, pending reset, or tracking-loss count. The external interrupt completed
without a
segmentation fault or deadlock. The intentionally stationary scene did not
provide a valid inertial sequence, so the terminal gate correctly rejected it
without publishing a canonical trajectory. After both windows closed, Qt
HighGUI still emitted two timer-affinity warnings; viewer shutdown is
therefore crash-free in this smoke but not warning-free.

### First-frame invariant and continuous-tracking re-audit

A five-process connected stationary stress reproduced the remaining
intermittent startup failure in three trials. The RealSense stream remained
connected, while the dispatcher reported `ORB-SLAM3 TrackStereo failed:
Operation not permitted`. A debugger breakpoint on
`std::__throw_system_error` localized the exception to
`ORB_SLAM3::Frame::setIntegrated()` through `Tracking::Track()` and
`System::TrackStereo()`.

The stereo `Frame` constructor could return when ORB extraction found no
keypoints, before allocating `mpMutexImu`. Tracking still calls
`setIntegrated()` for that first/no-previous-frame path, so it attempted to
lock an indeterminate mutex pointer. The same late-allocation invariant
existed in the other image-frame constructors. The reviewed ORB patch now
allocates the mutex at the start of all five applicable constructors while
retaining upstream copy-frame sharing semantics. Its tracked patch SHA-256 is
`569daedf09fc13fe93d8e68eb2be3c389b21b031c949529f7b03469318eecc87`;
the rebuilt `libORB_SLAM3.so` SHA-256 is
`08406c801cf3e8c5bddb25c72611a47c9d4adbe36d30cfb14ac9c0ab0833c453`.

Ten consecutive connected stationary process starts with that library all
streamed until their requested timeout. Each submitted 231 to 238 ORB frames
with zero queue drop, nonmonotonic rejection, in-stream IMU coverage failure,
active-map reset, `EPERM`, or segmentation fault. All ten correctly failed
only because a stationary device cannot establish the unchanged ORB inertial
initialization gate. This is direct hardware evidence for the first-frame
lifecycle fix, not motion, trajectory, homing, or accuracy evidence.

The final current-schema stationary smoke
`runs/orbslam3_live_stationary_continuity_v9_20260730` submitted 621 ORB
frames from 1,865 source pairs and synchronized 4,148 IMU samples. Its maximum
submitted-frame interval was 0.033615 seconds against the pinned
0.1-second limit, with zero loss, gap, reset, queue drop, timestamp rejection,
or in-stream IMU coverage failure. The capture-time-attested v2 evaluator
recomputed the same interval history and classified only the expected
stationary `INERTIAL_NOT_INITIALIZED`/no-canonical-pose conditions. A first
attempt in the restricted execution context failed cleanly at
`libusb_init()` during the preceding v8 check; direct USB execution produced
the evidence above. The later successful direct runs do not retroactively turn
the restricted-context attempt into a camera fault.

The application/evaluator contract audit also found that the old stability
timer could accrue while visual tracking was not valid, and a tracking-loss
episode after acceptance could leave a gap in the candidate trajectory without
making the application reject the run. The gate now requires continuous
`Tracking::OK`/`OK_KLT` throughout the BA2 stability window and permanently
rejects any later non-pose state. It reports post-acceptance loss separately
from normal pre-initialization loss. A second fail-closed interval gate is
generated from the pinned nominal rate: the current factor of three at 30 Hz
permits at most 0.1 seconds between submitted frames. A pre-acceptance excess
restarts the timer; a post-acceptance excess rejects the run. The independent
evaluator recomputes the stability timer, loss boundary, interval history, and
maximum interval rather than accepting terminal summary claims. Synthetic
tracking-loss, frame-gap, fabricated-timer, reset, and clean-pass cases are
covered by the registered tests.

Because the `Frame.cc` correction changes `libORB_SLAM3.so`, the prior atlas
chain was not reused as validation. A fresh revision 1 was built with patch
SHA-256 `569daedf...` and library SHA-256 `08406c...`; revision 1 to revision
2 then completed load, one detected/finished merge, save, one final map with
126 keyframes, and zero reset. A second process loaded revision 2, again
completed exactly one merge and one-map save with 198 keyframes and zero
reset. At that stage revision 3 remained explicitly reload-unverified, as
required for the newest output; the later provenance re-audit below reloads
it and continues the chain. This repeated-recording chain validates
serialization and cross-process merge integrity for the patched binary, not
false-merge rate or
distinct-session relocalization accuracy.

No operator motion was available for this re-audit. A fresh current-schema
motion run, rigid-stop return-to-home measurement, independent ground truth,
false-loop accounting, and Raspberry Pi 5 resource measurements remain
required before stronger continuity, accuracy, relocalization, or real-time
claims. ORB output remains isolated from OpenVINS and every flight interface.

The return-to-home evaluator was additionally hardened before that future
physical run. The earlier live contract compared one first and one last pose;
a single noisy frame could therefore create a false pass or failure. The new
live-only `ovrs-closed-loop-reference-v2` requires predeclared start/end hold
windows, minimum sample counts, and within-window position/orientation
dispersion limits. Evaluation requires nonoverlapping windows with time
coverage consistent with the configured tracking interval. It uses
component-wise median position and a quaternion-sign-aligned normalized mean,
then compares those robust endpoint poses. Synthetic regressions prove a clean
multi-sample return, reject a raw-zero first/last residual whose end-window
median is 0.1 m away, reject insufficient holds and orientation dispersion,
reject paths below predeclared duration/excursion bounds, and accept equivalent
`q`/`-q` quaternion representations. No physical return-to-home pass is
inferred from those logic tests. Evaluation now reports
continuity failures separately from reference failures, so a coherent
trajectory with an invalid hold is explicitly
`LIVE_GATE_PASS_CLOSED_LOOP_REFERENCE_FAILED` rather than being
misrepresented as a tracking-continuity failure.

### 2026-08-04 offline atlas provenance re-audit

The offline load path previously hashed a caller-supplied `.osa` but did not
require the companion atlas manifest. It could therefore stage an atlas whose
camera serial, calibration, camera cadence, time offset, backend patch, shared
library, vocabulary, or frame policy differed from the revisit. The evaluator
also applied the single-session `keyframes <= frames` invariant to a merged
atlas, which would incorrectly reject a mature atlas loaded for a short revisit.

The adapter now requires and stages `.osa.manifest.yaml`, validates every
compatibility field above, and hashes that parent manifest into the new adapter
and child-atlas chain. The runner checks the parent against the exact runner,
`libORB_SLAM3.so`, and vocabulary before starting; immutable launch inputs are
rehashed after exit. The evaluator independently checks the staged parent,
records exact ELF library resolution, allows historical keyframes to exceed
current-session frames only in multi-session mode, and requires the number of
completed merges to equal the number detected before accepting one final map.
Dependency-light tests cover missing and wrong-serial manifests plus the
large-atlas/short-revisit case.

The final-code fresh-process run
`slam_closed_loop_ref_20260729T212419Z_orbslam3_30hz_atlas_provenance_v9_reload4`
loaded revision 4 (`562e8283...`) under the unchanged patch (`569daedf...`) and
shared library (`08406c80...`). It wrote 3,807 poses from 4,491 inputs, reached
both inertial BA stages and terminal coverage, reported zero reset and local-map
tracking failure, detected and completed exactly one merge, and saved one map
with 303 keyframes. The ELF runner resolved the requested shared library and
all immutable launch inputs retained their hashes through backend exit. This
establishes revision 4 as reload-tested; revision 5 (`5cf572f4...`) remains
explicitly reload-unverified.

The run replayed the same recording again. Its estimated endpoint displacement
was 0.607 m in the merged atlas frame, and no independent trajectory reference
or distinct revisit was present. It is serialization/provenance/merge-lifecycle
evidence only, not correct-place identity, false-merge, relocalization accuracy,
GPS-denied navigation, live-camera atlas, or Raspberry Pi 5 evidence.

### 2026-08-04 connected live viewer and input-integrity re-audit

A connected D435i session initially produced a stable acceleration magnitude
near 20 m/s2. Project capture with motion correction on and off, a zero gyro
sensitivity setting, and the independent low-level librealsense collector all
reproduced the magnitude, so it was not attributed to the project rotation,
synchronizer, or one configuration switch. A non-persistent librealsense
hardware reset restored approximately 9.96 m/s2 without changing calibration,
firmware, or adding an accelerometer scale. The live path now requires a pinned
continuous low-motion startup window inside the selected estimator's existing
gravity/bias envelope. A stable gravity mismatch or failure to establish that
window is terminal; no observed one-pose ratio is used to rescale IMU data.

The first post-reset stationary run
`runs/orbslam3_live_stationary_startup_gate_20260804_v13` passed the startup
gate, submitted 615 stereo pairs with zero reset or continuity failure, and
measured 8.97 ms mean and 11.51 ms maximum ORB tracking latency with zero
33.33 ms frame-budget misses. It correctly failed because a stationary camera
cannot initialize ORB inertial SLAM, and it wrote no canonical trajectory.

The Pangolin viewer was then exercised with connected-camera operator motion in
`runs/orbslam3_live_viewer_motion_20260804_v14`. The startup gate again passed
and ORB was inertial-initialized transiently, but the run never completed BA2,
reset the active map 30 times, regressed inertial state 12 times, and produced
no accepted pose. Tracking latency was 11.87 ms mean and 54.71 ms maximum, with
15 of 2,057 submissions over the nominal frame budget. The viewer therefore
opened and consumed live data, but the current system is not yet demonstrated
stable enough to generate a usable physical-motion trajectory.

The run's very large reported ORB initialization-delta maximum is not a second
raw-IMU measurement. A source trace confirms that upstream ORB computes it as
the norm of the difference between consecutive preintegrated-frame average
accelerations. With repeated map resets it is retained only as a diagnostic;
the raw startup/capture IMU statistics and fail-closed reset gate remain the
authoritative health evidence.

That run also exposed an evidence defect: its wall-clock duration was 116.86 s
while sensor counts represented only about 68.6 s at their configured rates.
The live bundle is now v4 and pins a 1.0 s raw-input wall-clock watchdog. Either
silent stereo or silent IMU input is terminal. Capture and shutdown durations
are recorded separately, and the evaluator-v5 cross-checks the watchdog flag,
observed gaps, threshold, settings provenance, and latency aggregates.

After the change, the supported full build and all five CTest targets passed,
including 41 dependency-light research-script tests. Preflight then caught two
successive zero-frame device samples even though enumeration succeeded. A
second non-persistent hardware reset restored a 4.35 s diagnostic sample at
81.61 stereo, 181.84 gyro, and 233.33 accelerometer Hz with zero integrity
counters. The final current-code stationary run
`runs/orbslam3_live_stationary_input_watchdog_20260804_v17` captured 10.60 s,
submitted 316 stereo pairs at 29.80 Hz, synchronized IMU at 198.99 Hz, observed
no input stall (maximum reported stereo/IMU wall gap 0.609 s), and recorded a
separate 0.792 s shutdown. Its startup acceleration was 9.94 m/s2 with 0.0146
m/s2 standard deviation. Tracking latency was 8.93 ms mean and 12.15 ms maximum
with no deadline miss. Independent evaluation is coherently
`LIVE_GATE_FAILED` only for the expected no-motion inertial/trajectory gates.

These runs establish current x86-64 capture, viewer launch, startup-IMU,
callback-continuity, shutdown-accounting, and fail-closed evidence. They do not
establish stable live motion tracking, trajectory accuracy, return-to-home,
false-loop performance, relocalization, Raspberry Pi 5 real-time performance,
or GPS-denied navigation readiness. ORB output remains isolated from OpenVINS
and all flight interfaces.

### 2026-08-04 controlled recapture and cadence isolation

The connected D435i passed a fresh 4.36 s pre-capture stream check at 81.48
stereo, 181.33 gyro, and 232.97 accelerometer Hz with zero integrity counters.
The operator then recorded
`datasets/slam_init_motion_20260804_v19` using the selected 90 Hz VIO profile,
separate preview pipeline, five-second stationary start, staged translation
and rotation, return, and stationary finish. The recorder exited zero and
removed `INCOMPLETE`. Fail-closed export retained 10,780 stereo pairs and
23,970 synchronized IMU rows as `EXPORTED_NOT_EVALUATED`.

The unchanged calibrated-offset, stride-three adapter retained 3,593 stereo
pairs at 30 Hz and every IMU row. The pinned backend completed but failed the
tracking gate: 19 IMU-map resets, nine local-map tracking failures, 29 created
maps, three transient BA1 completions, no BA2 completion, and only 55 terminal
frame poses. An identical-data 90 Hz adapter also failed, with 24 resets, 18
local-map failures, 43 created maps, one BA1 completion, no BA2, and 138 of
10,779 terminal frame poses. Higher camera cadence therefore did not rescue
the initialization, so the 30 Hz stride is not identified as the primary
cause.

An isolation run using the same 30 Hz images in upstream stereo-only mode
tracked all 3,593 frames in one map and wrote 138 keyframes. The upstream
stereo-only process then exited 139 in its known post-trajectory shutdown path;
the files are visual-continuity diagnostics, not a formal backend pass. This
shows that end-to-end stereo tracking was possible while the stereo-inertial
map repeatedly failed.

An identical-method IMU comparison against the earlier repeatable
`slam_init_motion_20260729T204642Z` pass found similar acceleration-magnitude
standard deviation (0.287 m/s2 in each), but the new run was more
rotation-heavy: mean gyro magnitude was 0.516 versus 0.351 rad/s. At 30 Hz,
609 of 3,598 consecutive averaged-acceleration deltas reached 0.5 m/s2 versus
994 of 3,598 in the passing capture. A separate raw-image ORB feature proxy
found lower first-30-second stereo correspondence and temporal retention in
the new capture; this proxy is not the backend's internal match counter.
Visual review showed close foreground bottles and a keyboard, a moving person,
large tilt, and some motion blur, whereas the passing capture was dominated by
rigid mid/far room structure.

The evidence supports a new translation-dominant recapture in a cleared,
static, textured scene before another live viewer attempt. It does not justify
changing calibration, time offset, FAST thresholds, camera stride, or the
upstream 0.5 m/s2 initialization threshold. The new capture and both failed
inertial result manifests are retained with their provenance; no trajectory or
GPS-denied navigation claim is made.

### 2026-08-04 predeclared v20 recapture result

Before recording, the v20 protocol fixed a rigid mid/far static scene, a
translation-dominant first 55 seconds, an unchanged calibrated time offset and
ORB extractor, and fail-closed acceptance of zero reset/failure events, both
BA1 and BA2, one created/final map, at least 80 percent trajectory coverage,
and two identical offline passes before live-viewer promotion. The protocol
hash is
`fb6b5a8cdb9f8a198615c040835441a9781a5a16686255fc72b97afaa0959b53`.

A connected D435i preflight then ran for 4.39 seconds at 80.91 stereo, 180.29
gyro, and 231.34 accelerometer Hz with zero integrity counters. The completed
120.73-second `datasets/slam_init_motion_20260804_v20` recording retained
10,780 stereo pairs and 23,969 synchronized IMU rows with zero malformed
frames, rejected timestamps, callback errors, queue drops, duplicate or
regressing IMU timestamps, invalid values, or capacity drops. The single
missing interpolation bracket is the expected stream-boundary sample. Its
dataset metadata and recording-summary hashes are respectively
`63fbeca3f60b4adf90afffe369d7dfd7c0bb5e582f327e22c45e35650d379f2d`
and
`fa41f28dd8b3475f0b70b8572c47cc3c6012250826174e455124bc14a7bd4ad9`.
Fail-closed EuRoC export retained 10,779 in-range stereo pairs and all 23,969
IMU rows as `EXPORTED_NOT_EVALUATED`.

The recorded IMU, rather than the intended operator timeline, is authoritative
for delivered motion. Against the earlier repeatable capture, v20 remained
more rotation-heavy (0.482 versus 0.351 rad/s mean gyro magnitude) and had
fewer 30 Hz averaged-acceleration changes at or above 0.5 m/s2 (661 versus 994
of 3,598). A diagnostic raw-image proxy improved over v19 but still had lower
stereo correspondence than the repeatable capture. These metrics describe
capture excitation and imagery; they are not accuracy or ground-truth results.

The first unchanged 30 Hz pinned ORB-SLAM3 evaluation completed the backend but
failed the predeclared tracking gate. It reset the IMU map 18 times, reported
nine local-map tracking failures, created 28 maps, completed BA1 twice but
never BA2, and retained only 117 of 3,593 frame poses. Its 3.26 percent
trajectory coverage covers only 3.87 seconds near terminal input, despite
input consumption reaching the dataset end. The final atlas contained one map
only after repeated resets; that does not satisfy the one-created-map gate.
The adapter, settings, and result-manifest hashes are respectively
`5d79a24658955d142b41f303dd707a8a651f45054880c814f2fd5a9b8e119a25`,
`9234867bbfd3cbdc59fe51971ceecc5ff250caa3cd7fafa95ef7f23ea126f81a`,
and
`3d2e16e91d3bfe5f36a18578d14c2bea19e07abf528a89cb5ed4c85aa3bdd2b0`.

Because the first pass irreversibly failed its predeclared gates, a second
offline repeat, a higher-cadence retry, parameter tuning, and another live
viewer attempt were intentionally not run. V19 already showed that 90 Hz did
not rescue the same failure class. V20 is a clean hardware capture and a valid
negative initialization result, but it is not a stable trajectory, physical
accuracy, return-to-home, relocalization, or GPS-denied navigation result. The
next capture should not be another unconstrained freehand repetition: it needs
a physically constrained translation path and an operator-visible excitation
quality check before another expensive backend evaluation.

The follow-up added a dependency-free, provenance-bound pre-backend evaluator.
It verifies both EuRoC IMU hashes and requires the comparison result to be a
zero-reset, one-map ORB tracking pass with BA1 and BA2, bound through its adapter
to the reference export. At 30 Hz it measures changes between averaged
acceleration vectors and mean angular-rate magnitude without changing or
feeding any value to ORB-SLAM3. Predeclared comparison bounds require at least
0.95 reference duration, 0.85 of the reference's acceleration-event count at
the existing 0.5 m/s2 level, and no more than 1.25 of its mean gyro magnitude.

The repeatable pass is the unit control at ratios 1.0 and 1.0. V19 reproduced
609 versus 994 acceleration events (ratio 0.613) and a gyro ratio of 1.473.
V20 reproduced 661 events (ratio 0.665) and a gyro ratio of 1.376. Both are
`CAPTURE_EXCITATION_GATE_FAILED`, consistently rejecting the two captures that
also failed ORB initialization. This correlation is a capture-screening result,
not proof that a future excitation-gate pass will initialize or be accurate;
visual quality and all backend gates remain separately required.

### 2026-08-04 excitation screening and live trajectory promotion

A dependency-free capture-excitation evaluator was added before further
backend work. It verifies neutral-export IMU hashes, binds its reference to a
zero-reset one-map BA1/BA2 ORB pass, and reports duration, 30 Hz averaged
acceleration-change, and angular-rate ratios without feeding any value to the
estimator. The known pass reproduced ratios 1.0/1.0. V19 and v20 failed at
acceleration-event ratios 0.613 and 0.665 with gyro ratios 1.473 and 1.376.

Two additional connected D435i captures were clean at the transport layer.
V22 retained 10,782 stereo pairs and 23,975 synchronized IMU samples, but its
761/994 acceleration events produced a 0.766 ratio. V23 retained 10,783 pairs
and 23,975 IMU samples and improved to 801/994, ratio 0.806. Both remained
below the predeclared 0.85 gate and were not promoted. An explicitly
non-promotable unchanged v23 diagnostic later completed BA1 and BA2 and
tracked 2,635 of 3,594 frames, but only after four active-map resets; it
therefore remained `TRACKING_GATE_FAILED`.

Live v24 and v25 established clean viewer/capture startup but failed before
BA2. V24 was not a valid operator-motion attempt because the cue arrived late;
v25 recorded real motion but reset five times and produced no accepted pose.
This exposed that pre-canonical initialization retries and post-canonical
trajectory discontinuities were represented by one overly broad terminal
condition. The live gate now permits at most five backend-pinned active-map
resets only before acceptance, restarting the full BA2/stability window each
time. Every reset request or reset, loss, tracking gap, inertial regression,
or map correction after acceptance remains terminal. Core tests cover bounded
recovery, limit exhaustion, and permanent post-acceptance rejection;
evaluator-v6 independently recomputes the phase counts.

The fresh connected-D435i viewer run
`runs/orbslam3_live_motion_20260804_v26` passed without using that allowance:
its monotonic active-map reset count was zero. The startup IMU gate passed,
BA1 and BA2 completed, and the canonical gate remained open through at least
15 seconds of additional operator motion plus a rigid hold. Capture completed
cleanly after 79.43 seconds with 2,375 ORB submissions, 15,848 synchronized
IMU samples, and zero post-acceptance loss, gap, reset, pending reset, map
change, or trajectory discontinuity. All 1,404 candidate poses were atomically
published as the canonical trajectory, spanning 46.83 seconds and an
estimated 5.63 m path. Mean/maximum tracking latency was 13.18/18.03 ms with
zero 33.33 ms frame-budget misses on this x86-64 host.

Independent evaluation is
`LIVE_GATE_PASS_CONTINUITY_NOT_ACCURACY_VALIDATED`. No independent ground
truth was present, so the 0.084 m endpoint displacement, 8.95 degree endpoint
rotation, path length, and bounding box remain estimator outputs rather than
physical accuracy or return-to-home evidence. This run establishes a live,
viewer-enabled, canonical ORB-SLAM3 trajectory on the connected D435i. It does
not establish ATE/RPE, false-loop performance, persistent-map relocalization,
Raspberry Pi 5 real-time performance, or navigation/flight readiness.

The next operator-started viewer attempt,
`runs/orbslam3_live_motion_20260803T222721Z` (UTC suffix; 2026-08-04 local),
did not reproduce v26. Its startup stationary gate passed and D435i transport
remained clean, with 1,293 submitted stereo frames, 8,628 synchronized IMU
samples, no queue drops, no rejected timestamps, and no input stall over
43.30 seconds. ORB nevertheless initialized then regressed six times at
approximately 14.7, 23.2, 27.0, 29.8, 34.6, and 39.8 seconds. BA2 never
finished, the canonical gate never opened, and the independent result is
`LIVE_GATE_FAILED`. Upstream `LocalMapping.cc` requests this active-map reset
while BA2 is incomplete when recent keyframe-centre translation remains below
0.02 m while its motion-qualified initialization clock remains below ten
seconds. That clock advances only when the recent two-keyframe translation
exceeds 0.05 m, and BA2 is scheduled after it exceeds fifteen seconds. Some
failed map attempts accumulated substantial diagnostic visual path before a
pause, so the failure is not simply low total hand motion; it is
insufficiently continuous visual translation at the decisive keyframes. This
evidence points away from USB loss and also disproves any claim that one
successful v26 run made initialization operationally stable.

The apparent terminal closure had a separate launch cause. The earlier manual
recipe enabled `set -e` directly in the interactive shell; the correctly
nonzero live exit status 5 after the rejected gate could therefore terminate
that shell. `scripts/run_orbslam3_live.sh` now scopes strict shell options to a
child launcher, performs preflight, bundle preparation, viewer execution, and
independent evaluation, and never changes the parent shell options. The live
adapter also emits an explicit retry count and motion cue, automatically
requests clean shutdown when the bounded pre-acceptance retry limit is
exceeded, and automatically stops after a post-acceptance discontinuity.
These are lifecycle and operator-observability hardening changes; they do not
repair the still-variable initialization itself. A fresh connected-D435i run
is required after rebuild before claiming improved live behavior.

After the hardening changes, both the full Ubuntu build and portable-core
build passed all five registered CTest targets; the dependency-light research
suite contains 46 passing cases. The final connected-camera preflight passed
with zero errors and warnings and serial `843212070146` completed the
one-second inspector sample. The rebuilt live executable is version 0.6.0,
source fingerprint
`5546f31b97c8db0e1d31a558005168043cbbab6ba2e0bef32e83b0c55bb7889d`,
and SHA-256
`4296bcf004b29709a240115123391d18e291131cf7f13ef39b0145bb221afb50`.
This verifies the final binary and connected input path, not the new cue or
automatic-stop behavior under a moving live SLAM run. Capture-time binary
copies preserve re-evaluation of both the failed attempt and v26 across the
rebuild; their independent states remain respectively `LIVE_GATE_FAILED` and
`LIVE_GATE_PASS_CONTINUITY_NOT_ACCURACY_VALIDATED`.

### 2026-08-05 visual-support continuity hardening

The live canonical gate previously treated upstream `Tracking::OK`/`OK_KLT`
as sufficient visual readiness once inertial BA2 and the stability interval
passed. The tracking CSV already retained non-null tracked map-point counts,
but neither the application gate nor the independent evaluator required a
minimum. A nominal pose state with severely weakened map support could
therefore remain canonical until upstream declared tracking lost.

New bundle-v5/runtime-provenance-v6 runs pin a minimum of 50 tracked map
points. Weak support before acceptance restarts the complete post-BA2
stability interval. Weak support after acceptance is a terminal continuity
failure, stops publication, retains `INCOMPLETE`, and is independently
recomputed by evaluator-v7. The retained v26 trace had a minimum of 139 map
points over its 1,404 accepted rows, so 50 is a conservative floor below that
observed continuity run rather than a threshold selected to make a failed run
pass. Bundle-v4 evidence remains re-evaluable under its original zero-floor
contract and is not retroactively promoted.

Dependency-light regression tests cover pre-acceptance stability restart,
permanent post-acceptance rejection, evaluator failure attribution, and
legacy-v4 re-evaluation. The actual ORB-linked Linux build and portable build
each passed all five registered CTest targets; the dependency-light research
suite contains 48 passing cases. Generated bundle-v5 settings and manifest
fields were also checked against the selected serial-bound configuration.
These build and synthetic results are not a connected-camera result. No new
physical run has exercised bundle-v5, so this change is code-level fail-closed
hardening only. It does not establish correspondence correctness, trajectory
accuracy, initialization repeatability, relocalization, or navigation/flight
readiness.

### 2026-08-05 finite pose-jump continuity hardening

The canonical live path still had one downstream-output defect after the
visual-support fix: any finite pose returned with `Tracking::OK` could be
published even if its translation or orientation jumped implausibly between
adjacent frames. The evaluator's optional adjacent-translation argument was
post-run only, had no angular counterpart, and could not prevent a bad pose
from entering the candidate stream.

Bundle-v6/runtime-provenance-v7 now pins a 2.0 m/s linear and 6.0 rad/s angular
pose-rate envelope. Pose-valid tracking rows retain translation and
quaternion components; quaternion deltas are normalized and sign-invariant.
Reset, map-boundary, pending-reset, tracking-interruption, and over-limit frame
events clear the comparison baseline. An over-limit pre-acceptance sample
restarts stability, while an over-limit post-acceptance sample is terminal and
is not published as canonical. Evaluator-v8 independently recomputes both
rates and cross-checks runtime counts and maxima.

The audit also found that `reset_pending` prevented final acceptance but was
not part of the stability predicate, allowing the timer to begin on the last
pending frame. A pre-acceptance map-change index update likewise did not
explicitly restart stability. Both now require a complete fresh stability
window after the reset clears or the map frame changes, and the evaluator
recomputes the same rule.

The retained v26 accepted trace peaked at 0.383841 m/s and 1.821113 rad/s,
well inside the new envelope. Its full pre-acceptance diagnostic trajectory
contained map/inertial-frame transitions as high as 3.037643 m/s and
47.652499 rad/s, confirming why comparison baselines must be reset at declared
map/continuity boundaries rather than applying a naive whole-file threshold.
These observations justify a broad discontinuity rejection envelope only.
They do not measure platform dynamics or trajectory accuracy.

Core tests cover bounded motion, quaternion sign equivalence, translation and
rotation jumps, invalid-pose reset, and terminal post-acceptance rejection.
The dependency-light evaluator suite contains 51 passing cases including
legacy bundle-v4/v5 re-evaluation and independent attribution of a synthetic
3 m/s jump. The actual ORB-linked executable compiled successfully. No
connected D435i run has exercised bundle-v6, so live behavior, accuracy,
repeatability, relocalization, and navigation readiness remain unvalidated.

## Historical evidence

See [docs/audit_history.md](docs/audit_history.md) for the full chronology,
including calibration candidates, replay matrices, viewer work, ZUPT patch
review, kernel diagnostics, build commands, and superseded hypotheses.
