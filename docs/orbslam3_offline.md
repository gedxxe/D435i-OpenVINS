# ORB-SLAM3 offline baseline

This document covers the first external backend experiment on
`research/v0.6.0-markerless-vislam`. It does not change the live OpenVINS
runtime and does not claim that ORB-SLAM3 has passed the SLAM acceptance gates.

## Pinned source and license

The exact source and dependency revisions are recorded in
`config/research/orbslam3_backend.yaml`. ORB-SLAM3 is GPL-3.0-or-later. Its
source, vocabulary, build tree, and binaries remain under the ignored
`.deps/` directory; they are not copied into this repository.

The first desktop build used the upstream stereo-inertial EuRoC runner at
commit `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`. The build completed on the
current x86-64 Ubuntu host. That result does not establish an aarch64 or
Raspberry Pi 5 build.

Pangolin v0.6 required a build-only `-include cstdint` workaround with the
current compiler. The upstream source was not edited. Pangolin's old bundled
GLEW path was not used; the separately pinned `glew-cmake` source was installed
under `.deps`. The old Sophus test target is incompatible with the current
Eigen/compiler warning set, but ORB-SLAM3 uses its headers and the required
runner built successfully. These compatibility details must be revisited for a
reproducible Pi build rather than hidden by disabling project warnings.

## Prepare identical input

First create the backend-neutral export:

```bash
python3 scripts/export_vislam_benchmark.py \
  --dataset datasets/RECORDING \
  --output runs/benchmarks/RECORDING_euroc \
  --image-mode hardlink
```

Then create the ORB-SLAM3 view:

```bash
D435I_SERIAL="843212070146"
SELECTED_DIR="config/local/d435i-${D435I_SERIAL}/selected_runtime"

python3 scripts/prepare_orbslam3_benchmark.py \
  --benchmark runs/benchmarks/RECORDING_euroc \
  --imucam-config \
    "${SELECTED_DIR}/post_rs_imu_candidate_a_imucam.yaml" \
  --imu-config \
    "${SELECTED_DIR}/post_rs_imu_candidate_a_imu.yaml" \
  --output runs/benchmarks/RECORDING_orbslam3 \
  --file-mode hardlink
```

Use `--file-mode copy` across filesystems. Preparation fails when the recording
did not use gyro scale `1`, SDK motion correction, global time, the selected
resolution, matching camera/IMU calibration states, or a complete neutral
export.

The upstream runner assumes camera and IMU timestamps already share one clock.
The adapter therefore labels each image with:

```math
t_{\mathrm{camera\ in\ IMU\ clock}}
=
t_{\mathrm{camera}}
+
\Delta t_{\mathrm{camera\rightarrow IMU}}.
```

For the selected profile the fixed offset is
`-0.004900203074244532 s`. IMU timestamps remain unchanged. The adapter skips
only boundary frames that would lack an IMU bracket after this shift.

Transform directions are also explicit:

```math
T_{b\leftarrow c_1}=T_{\mathrm{imu}\leftarrow\mathrm{cam0}},
```

```math
T_{c_1\leftarrow c_2}
=
T_{\mathrm{imu}\leftarrow\mathrm{cam0}}^{-1}
T_{\mathrm{imu}\leftarrow\mathrm{cam1}}.
```

The generated settings use the recorded resolution and camera rate, selected
intrinsics, distortion, extrinsics, and Allan noise values. The initial
extractor settings remain the upstream EuRoC baseline: 1,200 features, FAST
threshold 20, and fallback threshold 7. Low-light tuning starts only after a
valid untuned baseline.

Two adapter options support controlled A/B experiments:

```bash
python3 scripts/prepare_orbslam3_benchmark.py \
  --benchmark runs/benchmarks/RECORDING_euroc \
  --imucam-config \
    "${SELECTED_DIR}/post_rs_imu_candidate_a_imucam.yaml" \
  --imu-config \
    "${SELECTED_DIR}/post_rs_imu_candidate_a_imu.yaml" \
  --output runs/benchmarks/RECORDING_orbslam3_30hz \
  --file-mode hardlink \
  --camera-stride 3
```

`--camera-stride 3` adapts a 90 Hz recording to 30 Hz while retaining every
IMU row. The manifest records both rates and the skipped frame count. The
default camera time-offset policy remains `calibrated`.
`--camera-time-offset-policy zero` is a diagnostic-only alternative whose
applied and calibrated offsets are recorded separately. Neither option changes
the live OpenVINS configuration.

## First execution result

The existing recording
`vio_rsusb_sensitivity_fix_20260729T1315Z` was exported with 2,241 stereo pairs
and 4,984 synchronized IMU rows. All 2,241 pairs remained bracketed after the
fixed time shift. The upstream runner loaded the generated calibration and ran
to completion without a viewer.

This was a backend-execution smoke test, not a SLAM pass. The recording did not
provide enough translational/accelerometer excitation for ORB-SLAM3 inertial
initialization. The log repeatedly reported insufficient acceleration or
motion and five IMU-map resets. Although the process returned zero and wrote
91 frame poses plus four keyframe poses from its final partial map, those files
must not be treated as an accepted trajectory.

This exposes an important result gate: process exit status and a non-empty pose
file are insufficient. A backend run fails when its log reports inertial map
reset or incomplete initialization.

## Controlled loop capture on 2026-07-30

A connected D435i capture was recorded as the ignored local dataset
`slam_loop_20260729T200612Z`. It completed 150.62 seconds with 13,476 stereo
pairs and 29,960 synchronized IMU rows. All camera, queue, timestamp, malformed
frame, callback, and IMU-integrity counters were zero. The effective rates were
89.47 Hz stereo and 198.91 Hz synchronized IMU.

The upstream stereo-only runner tracked all 13,475 exported stereo pairs in
one map. It wrote a 13,475-frame trajectory and 77 keyframes, with no local-map
tracking failure. Its shutdown later segfaulted after both trajectory files
were written, so the process is still marked failed and the output is
diagnostic only. No loop-detection message was present.

The corresponding stereo-inertial baseline did not initialize reliably. At
90 Hz it reset the IMU map 40 times and retained only 112 frame poses from its
final partial map. A deterministic stride-three view kept 4,492 stereo pairs
at 30 Hz and all 29,960 IMU rows; it still reset 20 times and retained only 119
frame poses. This rejects camera cadence as a sufficient fix.

A zero-offset diagnostic produced one substantially longer final map, but an
identical repeat did not reproduce it:

| 30 Hz diagnostic | IMU-map resets | Final-map frame poses | Keyframes |
| --- | ---: | ---: | ---: |
| calibrated offset | 20 | 119 | 13 |
| zero offset, first run | 4 | 3,036 | 78 |
| zero offset, repeat | 22 | 47 | 6 |

All three runs failed the no-reset gate. The calibrated offset remains the
default. The zero-offset result is evidence for further timestamp analysis,
not a replacement calibration.

An offline comparison between the full stereo-only orientation and recorded
gyro showed strong angular agreement: 0.983 overall correlation and a 0.985
least-squares gyro scale over energetic intervals. The best lag was near the
unshifted camera labels. This makes a gross gyro scale or axis sign error
unlikely, while motivating a reproducible timing evaluator before changing
the adapter default.

The physical path stayed within about 0.52 m of its starting point. It was
stationary for roughly the first 30 seconds and again after about 110 seconds.
ORB-SLAM3's upstream inertial initializer repeatedly rejected the short or
low-translation keyframe windows. The next sequence must provide earlier and
larger translation instead of changing FAST thresholds.

## Initialization-focused capture on 2026-07-30

A second connected capture used about 5 seconds of stationary startup followed
by immediate forward/back, lateral, and vertical translation. Rotation was
mixed with translation rather than performed in place, and the path returned
toward its starting view.

For a repeat, keep most of the view on rigid mid-distance and far room
structure. Clear people and moving objects from both infrared views and avoid
letting close bottles, keyboards, hands, or other foreground objects dominate
the frame. During the initialization interval, prioritize distinct smooth
forward/back, lateral, and vertical acceleration changes. Add only modest
rotation while translating; a rotation-heavy path can preserve stereo-only
tracking yet repeatedly fail the inertial map gate. Avoid motion blur and keep
the camera approximately level enough that both stereo views retain the same
static structure.

Use `ovrs_record --capture-mode vio --preview` with the selected 90 Hz stream.
Preview first uses a separate pipeline and starts the clean dataset only after
Space is pressed. It then displays the latest owned stereo pair while the
recording continues; HighGUI is never called from the RealSense callback.

Before spending another ORB-SLAM3 evaluation, compare the exported candidate's
synchronized IMU excitation with the provenance-bound repeatable pass:

```bash
python3 scripts/evaluate_orbslam3_capture_excitation.py \
  --candidate runs/benchmarks/RECORDING_euroc \
  --reference \
    runs/benchmarks/slam_init_motion_20260729T204642Z_euroc \
  --reference-result \
    runs/benchmarks/slam_init_motion_20260729T204642Z_orbslam3_30hz/experiment_init_motion.yaml \
  --minimum-duration-ratio 0.95 \
  --minimum-acceleration-delta-count-ratio 0.85 \
  --maximum-gyro-mean-ratio 1.25 \
  --output runs/benchmarks/RECORDING_orbslam3_30hz/capture_excitation.yaml
```

The thresholds are pre-backend capture controls, not ORB-SLAM3 parameters.
They require a comparable recording duration, at least 85 percent as many
30 Hz averaged-acceleration changes above the pinned 0.5 m/s2 level, and no
more than 1.25 times the reference's mean angular rate. The evaluator verifies
the neutral-export IMU hash and requires a successful, zero-reset, BA1/BA2
reference result bound to the supplied reference export. A pass is explicitly
`CAPTURE_EXCITATION_GATE_PASS_VISUAL_NOT_EVALUATED`: it cannot establish image
quality, inertial initialization, trajectory correctness, or accuracy. A
candidate must still pass the unchanged backend gates twice before live use.

The ignored local dataset `slam_init_motion_20260729T204642Z` completed 120.57
seconds with 10,782 stereo pairs, 23,971 synchronized IMU rows, and zero
capture-integrity counters. The emitter was off. The effective rates were
89.42 Hz stereo and 198.81 Hz synchronized IMU.

The calibrated-offset, stride-three adapter retained 3,594 stereo pairs at
30 Hz and every IMU row. Two identical headless runs both:

- created one map;
- completed inertial BA stages 1 and 2;
- reported zero IMU-map resets and zero local-map tracking failures; and
- wrote 3,110 frame poses from 16.21 through 119.99 seconds.

The first run wrote 156 keyframes and did not detect a loop. The repeat wrote
150 keyframes and found three loop candidates. ORB-SLAM3 rejected two as bad;
the third entered its loop-correction path. The estimated return displacement
was 0.270 m without that correction and 0.073 m in the corrected repeat.
These are estimator outputs from an operator-returned path, not independent
ground truth.

This is the first repeatable stereo-inertial initialization result in the
branch. It passes backend execution and the no-reset gate for the 30 Hz adapted
view. It does not pass the loop-closure acceptance gate because the correction
occurred in only one of two identical runs and no independent reference was
recorded. FAST and image preprocessing remain unchanged.

## Result manifest

Use the supported runner for new experiments so the process exit status is
captured rather than entered manually:

```bash
python3 scripts/run_orbslam3_benchmark.py \
  --adapter-dir runs/benchmarks/RECORDING_orbslam3_30hz \
  --runner .deps/src/orb_slam3/Examples/Stereo-Inertial/stereo_inertial_euroc \
  --backend-library .deps/src/orb_slam3/lib/libORB_SLAM3.so \
  --vocabulary .deps/src/orb_slam3/Vocabulary/ORBvoc.txt \
  --run-id baseline_a
```

It writes `backend_baseline_a.log`, `backend_baseline_a.status`, the upstream
frame and keyframe trajectories, and `experiment_baseline_a.yaml`. The result
manifest distinguishes:

- inertial initialization and continuous tracking;
- loop candidates rejected by geometric checks;
- loop corrections actually applied;
- repeatability across identical runs; and
- return error against a reference that the estimator never consumes.

The two completed motion-focused runs were evaluated with this contract. Their
states are `TRACKING_PASS_NO_LOOP_CORRECTION` and
`TRACKING_PASS_LOOP_CORRECTION_NOT_REFERENCE_VALIDATED`; both explicitly
record `independent_reference_present: false`.

## Closed-loop reference

The next physical sequence should use a rigid, nonvisual start/end stop. For
example, place the camera against the same table corner with two fixed contact
surfaces before and after the path. This is an evaluation fixture, not a
fiducial, and it is never visible to or consumed by the estimator.

Record the physical setup before running the backend:

```yaml
%YAML:1.0
format: "ovrs-closed-loop-reference-v1"
reference_type: "COLOCATED_START_END"
estimator_consumed_reference: false
physical_path_completed: true
method: "camera returned against the same two rigid contact surfaces"
position_tolerance_m: 0.01
orientation_tolerance_deg: 2
```

Pass that file through `--closed-loop-reference`. The tolerance values must
describe the real fixture; do not copy the example numbers when the physical
setup cannot support them. A return residual within those values means the
estimate is consistent with the recorded placement tolerance; it is not an
ATE or RPE result.

## Referenced closed-loop result on 2026-07-30

The ignored local dataset `slam_closed_loop_ref_20260729T212419Z` used the same
two rigid contact surfaces at the start and end. The operator confirmed both
contacts after the 150.65-second path. The recorded fixture tolerances were
2 cm in position and 5 degrees in orientation. The reference file and its
tolerances were supplied only to the evaluator after the trajectory was
written, never to the estimator.

Capture integrity was clean: 13,475 stereo pairs, 29,961 synchronized IMU
rows, and zero camera, queue, callback, timestamp, malformed-frame, duplicate,
regression, invalid-value, or capacity-drop counters. The emitter was off.

The calibrated-offset stride-three adapter retained 4,491 stereo pairs at
30 Hz and every IMU row. Two runs executed through the supported runner and
result evaluator:

| Result | Run A | Run B |
| --- | ---: | ---: |
| Tracking gate | pass | pass |
| IMU-map resets | 0 | 0 |
| Local-map failures | 0 | 0 |
| Final atlas maps | 1 | 1 |
| Frame poses | 3,807 | 3,807 |
| Loop candidates/corrections | 0 / 0 | 0 / 0 |
| Estimated return displacement | 0.0129 m | 0.0165 m |
| Estimated return rotation | 2.88 deg | 2.75 deg |

Both returns are consistent with the recorded 0.02 m and 5 degree placement
tolerances. This is a repeatable referenced return result for stereo-inertial
odometry. It is not a loop-closure pass because neither run detected a loop,
and it is not a full accuracy result because a colocated endpoint does not
provide ATE or RPE over the path.

Only after a referenced sequence and identical-run repeatability should
low-light FAST or image-preprocessing A/B experiments begin.

## Persistent atlas experiment

The pinned stereo-inertial tracker does not provide a safe localization-only
startup path. `System::ActivateLocalizationMode()` stops local mapping after a
tracking call, while the inertial tracking source labels that mode as not
available. Loading an atlas also creates a new active map. This workflow
therefore uses ORB-SLAM3's upstream multi-session map merge instead of
presenting localization-only as supported.

Build the first map from a deliberate commissioning sequence:

```bash
python3 scripts/prepare_orbslam3_benchmark.py \
  --benchmark runs/benchmarks/MAP_BUILD_euroc \
  --imucam-config \
    config/local/d435i-843212070146/selected_runtime/post_rs_imu_candidate_a_imucam.yaml \
  --imu-config \
    config/local/d435i-843212070146/selected_runtime/post_rs_imu_candidate_a_imu.yaml \
  --output runs/benchmarks/MAP_BUILD_orbslam3_30hz \
  --camera-stride 3 \
  --save-atlas-name room_revision_1

python3 scripts/run_orbslam3_benchmark.py \
  --adapter-dir runs/benchmarks/MAP_BUILD_orbslam3_30hz \
  --runner .deps/src/orb_slam3/Examples/Stereo-Inertial/stereo_inertial_euroc \
  --backend-library .deps/src/orb_slam3/lib/libORB_SLAM3.so \
  --vocabulary .deps/src/orb_slam3/Vocabulary/ORBvoc.txt \
  --run-id map_build
```

Prepare a different recording that revisits a distinctive part of that map:

```bash
python3 scripts/prepare_orbslam3_benchmark.py \
  --benchmark runs/benchmarks/REVISIT_euroc \
  --imucam-config \
    config/local/d435i-843212070146/selected_runtime/post_rs_imu_candidate_a_imucam.yaml \
  --imu-config \
    config/local/d435i-843212070146/selected_runtime/post_rs_imu_candidate_a_imu.yaml \
  --output runs/benchmarks/REVISIT_orbslam3_30hz \
  --camera-stride 3 \
  --load-atlas \
    runs/benchmarks/MAP_BUILD_orbslam3_30hz/room_revision_1.osa \
  --save-atlas-name room_revision_2

python3 scripts/run_orbslam3_benchmark.py \
  --adapter-dir runs/benchmarks/REVISIT_orbslam3_30hz \
  --runner .deps/src/orb_slam3/Examples/Stereo-Inertial/stereo_inertial_euroc \
  --backend-library .deps/src/orb_slam3/lib/libORB_SLAM3.so \
  --vocabulary .deps/src/orb_slam3/Vocabulary/ORBvoc.txt \
  --run-id revisit
```

The adapter stages the input atlas under a fixed local name because upstream
resolves atlas names relative to its working directory and appends `.osa`.
It requires the source `.osa.manifest.yaml`, validates the atlas against its
camera serial, calibration, cadence, time-offset, backend, patch, and frame
contract, then stages and hashes both files. The runner separately hashes both
the executable and its `libORB_SLAM3.so` and verifies that the ELF loader
resolves that exact library; hashing an arbitrary library beside the executable
would not bind the dynamically linked implementation. It refuses to overwrite
an atlas and writes a provisional atlas manifest only when process, inertial
tracking, terminal coverage, save, and one-map gates all pass. A newly
serialized atlas remains
`TRACKING_GATE_PASS_ATLAS_RELOAD_UNVERIFIED` until a later result manifest
records a complete load and full tracking run; a successful save alone does
not establish reload integrity.

For a revisit, a successful process is not enough. The evaluator requires an
upstream `*Merge detected` followed by `Merge finished!`, and the final atlas
must contain one map. Otherwise the result is
`ATLAS_MERGE_NOT_ESTABLISHED`, the runner returns nonzero, and no accepted
atlas manifest is written. This proves that a cross-session map merge was
applied; without an independent pose reference it does not prove a correct
merge, relocalization accuracy, or false-merge rate.

### Serialization failure and local patch

The first unpatched save completed and produced a nonempty atlas, but loading
that atlas failed after about four seconds with six missing MapPoint reference
messages and a segmentation fault. SHA-256 confirmed that staging had not
changed the file. A debugger located the null dereference in
`KeyFrame::UpdateBestCovisibles()` during `Map::PostLoad()`.

Source inspection found that upstream serialization could omit bad keyframes
and map points while retaining graph references to them. Shutdown could also
begin serialization before mapping and loop-closing threads had fully
finished. The old local atlas is therefore invalid and must not be reused.

The reviewed
`patches/orbslam3-atlas-serialization-integrity.patch` waits for graph-mutating
threads before save, serializes a self-contained nonbad graph, and fails closed
on unresolved references while loading. It also removes transient empty maps
from the runtime registry before serialization, using a two-phase collection
so the registry is never erased while it is being iterated. Its hash is pinned in
`config/research/orbslam3_backend.yaml` and verified during adapter
preparation. A fresh save and reload test is required after every patch or
backend revision.

The current patch validation built a 70-keyframe atlas, then completed four
fresh-process reload/merge generations with 126, 198, 256, and 303 keyframes.
Every generation wrote 3,807 active-session frame poses with zero IMU-map reset
or local-map tracking failure; each reload detected and completed exactly one
merge and finished with one atlas map. The final two reloads additionally
verified the full parent-manifest chain and exact dynamic-library resolution;
the last run also rehashed every immutable launch input after backend exit.
Revisions 1 through 4 now have subsequent reload evidence. Revision 5 remains
explicitly reload-unverified. This establishes chained offline serialization
on one repeated input; it does not establish correct merge identity,
false-merge rate, relocalization accuracy, or navigation readiness on a
distinct revisit.
