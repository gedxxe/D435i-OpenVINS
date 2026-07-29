# Selected diagnostic runtime configuration

This is the reproducible diagnostic baseline for the D435i currently
calibrated in this checkout. It is not an accepted drift-safe runtime.

## 2026-07-29 official EEPROM recalibration and gyro-scale replacement

The official `rs-imu-calibration-fixed.py` workflow collected 6000
measurements, reported a corrected norm of 9.803658 m/s^2 against the
9.806650 m/s^2 target, and successfully wrote the result to D435i serial
`843212070146`. A subsequent OVRS stationary hardware capture read back the
new accelerometer matrix with RealSense motion correction active. Its middle
stationary interval measured 9.872342 m/s^2 mean sample norm, 0.017115 m/s^2
norm standard deviation, and zero capture-integrity errors.

The new operator-cued raw VIO dataset
`datasets/vio_post_official_calibration_cued_20260729T0500` contains 4488
stereo pairs and 9980 synchronized IMU rows over 50.44 seconds with every
drop/error counter at zero. Replay with the former `gyro_scale_factor: 0.5`
reached the 3 m/s safety gate at 42.36 seconds. An identical-data diagnostic
copy changing only synchronized gyro values to the SDK's unscaled `1.0`
completed all 47.82 initialized seconds: maximum estimated speed was
0.0692 m/s, final speed 0.0065 m/s, final displacement 0.0588 m, and estimated
path 0.557 m.

A connected 131.13-second live run with the `1.0` override then completed
cleanly with zero stereo/IMU drops and no safety failure. The world grid stayed
fixed as designed. The operator moved almost entirely by rotation with about
0.2 m of physical translation; the estimator ended at 0.819 m displacement
and 0.0137 m/s, so at least approximately 0.62 m of residual endpoint error
remains. This promotes gyro scale `1.0` over `0.5`, but it does not promote the
overall runtime to drift-safe or accuracy-certified status.

### 2026-07-29 lighting and frame-rate live repeats

Four connected marked-pose handheld trials compared the selected stream under
poor lighting, two room-lit 90 Hz repeats, and one room-lit 30 Hz diagnostic.
All completed with zero stereo/IMU drops:

| Stream and lighting | Return error | Estimated error/path | Mean IR gain |
| --- | ---: | ---: | ---: |
| 90 Hz, poor lighting | 0.522 m | 1.78% | 245.5 |
| 90 Hz, room light, first run | 0.141 m | 0.53% | 38.3 |
| 90 Hz, room light, repeat | 0.405 m | 1.29% | 39.1 |
| 30 Hz, room light | 0.834 m | 2.48% | 16.1 |

The manual trajectories were not identical and had no ground truth, so the
0.141 m result is not a repeatable accuracy claim. The 30 Hz run used
22.410 ms mean exposure versus the 90 Hz ceiling of 9.947 ms and performed
worse despite lower gain. The selected live policy therefore remains 90 Hz
with adequate room illumination; 30 Hz remains the calibration profile.

Frontend support stayed `HEALTHY` throughout material endpoint error. A
read-only follow-up now records candidate and accepted MSCKF batch counts,
acceptance ratio, and batch age. Identical-data replays of the marked ceiling
and downward datasets remained byte-identical after instrumentation.
Aggregate accepted/candidate ratios differed, but short-window distributions
overlapped, so no unevidenced ratio threshold was added to the health gate.

### Controlled post-calibration pitch tracker A/B

Two subsequent native-factor-`1.0` hardware datasets provided complete marked
pose cycles. `datasets/vio_gyro1_marked_pitch_cued_20260729T0520` retained
6736 stereo pairs and 14978 synchronized IMU rows over 75.53 seconds.
`datasets/vio_gyro1_marked_down_return_20260729T0535` retained 5388 stereo
pairs and 11979 synchronized IMU rows over 60.51 seconds. Both captures
reported zero transport-integrity errors.

Identical-data replay compared the selected global histogram equalization,
FAST threshold 20, 200-point baseline against one-variable CLAHE, FAST
threshold 10, and 300-point/minimum-spacing-12 trials, plus the combined
CLAHE/FAST-10 trial. A final no-equalization ceiling replay completed the three
supported preprocessing modes. None passed both orientation datasets:

- CLAHE improved the ceiling marked-pose return from 5.15 cm to 4.25 cm and
  increased ceiling MSCKF updates, but worsened the downward run's estimated
  path from 0.342 m to 0.470 m and final displacement from 2.12 cm to 5.10 cm.
- FAST threshold 10 improved the downward final displacement to 1.06 cm, but
  worsened the ceiling return to 6.93 cm, peak speed to 0.086 m/s, and SLAM
  feature mean during the ceiling hold from 18.45 to 11.57.
- Combined CLAHE/FAST-10 improved the ceiling return to 4.01 cm, but worsened
  the downward return to 1.92 cm, path to 0.442 m, and final displacement to
  5.93 cm.
- Increasing feature density worsened the ceiling return to 6.07 cm, peak
  speed to 0.082 m/s, path to 0.665 m, and processing latency from 5.05 ms to
  6.25 ms. It was rejected without a redundant downward replay.
- Disabling equalization worsened the ceiling return to 5.83 cm and peak speed
  to 0.091 m/s; ceiling-hold MSCKF updates fell to zero and mean SLAM features
  to 6.95. It was rejected without a redundant downward replay.

The selected `HISTOGRAM`, `fast_threshold: 20`, `num_pts: 200`, and
`min_px_dist: 15` values therefore remain unchanged. The ceiling image
contains mostly repeated parallel grooves and saturated lamps, whereas the
downward image contains non-parallel local texture and structure at multiple
depths. More detected corners did not consistently produce better
triangulated constraints or lower trajectory error.

## Known failure that invalidates the earlier acceptance

Pre-recalibration pitch-motion live runs on 2026-07-29 reproduced severe false
translation while camera, timestamp, queue, and non-finite counters remained
clean. One run
produced 6.698 m of estimated path and 4.147 m final displacement in 29.11
seconds; estimated speed rose from 0.5 to 1.0 m/s in about 0.45 seconds and
then approached the configured 3 m/s safety gate.

The historical replay previously cited as the final milestone also cannot
support that conclusion. Although it completed 57.83 seconds and ended at
0.0033 m/s, its own log reports 60.893 m estimated path and 10.677 m final
displacement. The stop-recovery patch constrained final velocity after the
position estimate had already diverged.

Debug replay of the same raw dataset showed MSCKF update features collapsing
from periodic bursts up to 35 before motion to fewer than 0.3 per camera frame
and eventually zero after motion began. Visible edges are therefore not proof
of usable translational/stereo constraints. Raising `max_clones` from 11 to 22
on identical data reduced peak speed but worsened final displacement to 19.251
m and exceeded the 90 Hz frame budget, so clone count alone is not a fix.

Do not tune around the remaining drift or call it solved without raw,
operator-cued multi-orientation captures, identical-data replay A/B, and
physical path bounds.

### 2026-07-29 operator-cued pitch capture

`datasets/vio_pitch_cued_20260729T035200Z` completed a 30.374-second hardware
capture with 2691 stereo pairs, 5984 synchronized IMU rows, and zero camera,
timestamp, callback, or queue errors. The requested 90 Hz profile, gyro
sensitivity 1, gyro scale 0.5, motion correction, and Global Time were all
active.

The recorded IMU, rather than the intended cue schedule, defines the physical
test: the camera remained level through about 12 seconds, rotated roughly 90
degrees toward the ceiling from about 12 to 22 seconds, then remained in that
orientation. It did not perform the intended pitch-down and return-to-level
phases, so this dataset must not be presented as a return-to-origin test.

Selected-runtime replay nevertheless reproduced the reported failure while
the camera was stationary facing the ceiling. Displacement grew from 0.075 m
at 22 seconds to 2.147 m at the end, and speed grew from 0.185 to 0.559 m/s.
MSCKF update features were zero on 99.6% of the final stationary frames, while
SLAM features fell to 6. The IR image retained sharp edges, but most ceiling
structure was parallel and repetitive and the lamps were saturated; these are
not equivalent to well-distributed, non-repeating stereo corners.

The mean stationary accelerometer norm also changed with orientation:
9.8103 m/s^2 while level versus 9.9469 m/s^2 facing the ceiling. A temporary
`Ta_zz: 1.0143` A/B reduced this dataset's final displacement to 0.0285 m but
worsened the independent historical moving dataset from 10.677 m to 14.470 m.
A separate `fast_threshold: 10` A/B reduced this dataset to 0.154 m but also
worsened the historical dataset to 11.092 m. Both changes were therefore
rejected as non-generalizing. That historical capture remains unchanged; the
later official EEPROM recalibration and gyro scale `1.0` A/B are documented
above.

### 2026-07-29 pitch-down capture

`datasets/vio_pitch_down_cued_20260728T210500Z` completed a second
35.449-second connected capture with 3140 stereo pairs, 6983 synchronized IMU
rows, and zero capture-integrity errors. The raw gyro records a stationary
level interval through about 13 seconds, a roughly 80-degree pitch in the
opposite direction through about 21 seconds, and a stationary downward-facing
interval through the end. The intended return-to-level motion again did not
occur, so no marked-pose return claim is made.

Unlike the ceiling run, selected-runtime replay remained bounded: estimated
path was 0.177 m, final displacement was 0.0647 m, and final speed was
0.0028 m/s. During the final stationary interval, SLAM features stayed at
24-25 and MSCKF updates continued. The downward image saw a nearby desk,
keyboard, mouse, cables, and other non-repeating structure at multiple depths;
this is materially better feature geometry than the repeated parallel ceiling
lines.

The accelerometer still showed orientation dependence. Mean stationary norm
was 9.8133 m/s^2 while level and 9.6454 m/s^2 while facing downward, versus
9.9469 m/s^2 in the upward-facing capture. Thus the downward result does not
clear the inertial calibration: visual/ZUPT constraints bounded velocity while
the estimated accelerometer-bias norm grew to 1.791 m/s^2. Together the two
captures show a coupled failure: residual accelerometer calibration can drive
false motion, while scene-dependent loss of usable visual constraints
determines whether the estimator can arrest it.

## Exact selection

| Item | Selected value |
| --- | --- |
| D435i serial | `843212070146` |
| Estimator | `config/local/d435i-843212070146/selected_runtime/estimator.yaml` |
| IMU | `post_rs_imu_candidate_a_imu.yaml` beside the estimator |
| Camera/IMU | `post_rs_imu_candidate_a_imucam.yaml` beside the estimator |
| VIO streams | `config/sensors/realsense_streams_vio_90hz.yaml` |
| Camera | 848x480 Y8 stereo at 90 Hz |
| IMU | gyro 200 Hz, accelerometer 250 Hz |
| Gyro sensitivity | SDK/FW level 1, set before streaming and read back |
| Gyro scale | SDK rad/s multiplied by `1.0` before synchronization |
| Motion correction | required and read back active |
| Global Time | required and read back active |
| Camera-IMU offset | fixed at -4.900203074 ms |
| Online offset | off |
| Stop recovery | visually gated continuous ZUPT |

The selected files are bound to this review:

| File | SHA-256 |
| --- | --- |
| `estimator.yaml` | `be37da3454190ba262a204afa709ee58d034784814e3a7c09fb629be02479867` |
| `post_rs_imu_candidate_a_imu.yaml` | `c23713d7830e2d76e7d281edb0f8decb192a7f740ef15af0927d38d2816fa830` |
| `post_rs_imu_candidate_a_imucam.yaml` | `0e911d87f1d2f508de1e9504354272220a999e76d821c9e4dc6b3a6fd3006f4f` |
| `config/sensors/realsense_streams_vio_90hz.yaml` | `cc0cf24730f056dcd6af1d2eebcac21bc3c6e3266e752a17b0d86e61fa8cff03` |
| `patches/openvins-zupt-velocity-constraint.patch` | `000c826231727ee10cf240d89469e956e44028ddaff5cf3ccb2c744b92368d37` |

`scripts/verify_selected_runtime.sh` is the single executable check for this
table. README commands call that script instead of duplicating the hashes.

The files under `config/local/` are serial-specific. Never copy them to
another camera. The bundle remains `BOOTSTRAP_UNVERIFIED`, so
`--allow-unverified-calibration` is mandatory. Its repeated Kalibr calibration
is useful evidence, but the small AprilGrid, approximately 3.19 mm translation
repeatability, pre-device-update Allan capture, and lack of external trajectory
ground truth prevent an honest `KALIBR_VERIFIED` label.

## Why candidate A and the fixed offset remain selected

Two post-device-update Kalibr runs agreed to 0.060919 degrees in rotation,
3.193655 mm in translation, and 0.688995 ms for the cam0 time offset.
Candidate A had the tighter camera residuals: 0.291736 px for cam0 and
0.281472 px for cam1. Its two camera time estimates differed by only
0.007121 ms.

Online time-offset estimates varied from approximately -9.23 ms to +2.79 ms
across motion and observability conditions. Those run-specific results are
not a replacement for the repeatable fixed Kalibr value.

## Gyro acquisition and scale contract

Sensitivity and scale solve different problems. OVRS sets D435i dynamic
gyro-sensitivity level `1` before pipeline start and verifies the same value
after streaming begins. It then applies the project-owned scale factor to the
SDK rad/s vector before the synchronizer, recorder, or estimator sees it.

Every new motion capture must contain:

```yaml
# resolved_stream_config.yaml
gyro_sensitivity: 1
gyro_scale_factor: 1.0

# device_report.yaml
gyro_sensitivity_requested: 1
gyro_sensitivity_available: true
gyro_sensitivity_active: 1
gyro_scale_factor_configured: 1
gyro_scale_factor_applied: 1

# dataset_metadata.yaml
gyro_sensitivity_active: 1
gyro_scale_factor_applied: 1
```

The current direct-SDK policy is explicit rather than inferred during replay.
The former `0.5` selection had this historical evidence:

- a clean moving level-1 capture and an independent strong moving level-0
  capture both measured approximately twice as much integrated SDK gyro
  rotation as calibrated stereo PnP/essential-matrix rotation;
- 12 accepted windows in
  `datasets/vio_sensitivity0_final_20260728T164343Z` had median
  gyro/visual ratio 2.017 and mean 1.993;
- the original replay hit the 3 m/s safety gate, while a diagnostic copy that
  changed only gyro values by 0.5 completed all 57.83 s;
- the first native capture with the correction,
  `datasets/vio_gyro_scale05_final_20260728T165508Z`, recorded 5388 stereo
  pairs and 11982 synchronized IMU rows with every transport integrity
  counter at zero;
- representative corrected comparisons were 13.667 vs 13.931 degrees and
  11.869 vs 11.683 degrees for visual vs gyro rotation.

That evidence did not generalize after the official EEPROM recalibration. On
the new identical post-calibration dataset, `0.5` caused a safety-gate failure
while `1.0` completed; a connected 131-second `1.0` live run independently
avoided runaway. The selected factor is therefore now `1.0`.

Historical datasets are never modified and replay is never silently
rescaled. A legacy dataset without the scale fields is replayed exactly as
recorded. New calibration exports bind the scale from the Allan and
camera-IMU captures and reject disagreement.

The `1.0` selection is serial- and device-calibration-state-bound.
Recalibration, firmware changes, or a different camera require a fresh moving
visual/gyro cross-check.

## Stop-recovery policy

The selected gyro scale removes the immediate rotation inconsistency, but the
selected estimator still needed a safe way to stop velocity growth after real
motion. The active settings are:

```yaml
try_zupt: true
zupt_chi2_multipler: 10
zupt_max_velocity: 3.0
zupt_noise_multiplier: 10
zupt_max_disparity: 2.0
zupt_only_at_beginning: false
zupt_constrain_velocity: true
zupt_velocity_noise: 0.05
zupt_min_stationary_time: 1.0
```

The reviewed OpenVINS patch requires more than 20 tracked features and one
second of consecutive low-disparity frames. Any moving or unknown frame resets
the candidate. The patch formerly tried to find one track spanning the entire
interval after normal MSCKF cleanup; that always produced zero features and
made recovery unreachable. It now measures duration across consecutive
per-frame checks.

On the corrected moving capture, the old initialization-only configuration hit
the 3 m/s gate at 45.62 s. The selected candidate completed all 57.83 s and
ended at 0.0033 m/s, but it accumulated 60.893 m of estimated path and 10.677
m final displacement. This proves only that the recovery can constrain final
velocity; it does not establish bounded position or acceptable VIO.

## Reproducing this milestone

Use a direct USB 3 connection, rigid grip or mount, even illumination, and
sharp non-repeating static structure visible in both IR cameras. Avoid blank
walls, repeating patterns, moving foreground objects, saturation, occlusion,
and whip motion.

1. Verify all five hashes and pass both preflight commands.
2. Hold the camera still until initialization and for several seconds after.
3. Move smoothly. Before stopping, decelerate rather than snapping the camera.
4. Hold the final pose still for at least two seconds so the one-second
   visual-stationarity gate can complete.
5. Reject the capture if any drop/error counter is nonzero or if the
   sensitivity/scale fields above disagree.
6. Replay the identical dataset before interpreting live behavior.
7. Declare physical path and endpoint bounds before viewing the estimate.
   `healthy=1`, a completed replay, and a flat final trace are integrity
   evidence, not external accuracy proof.

Standalone VIO has no mapping, loop closure, GPS, or other absolute reference.
It cannot guarantee zero long-term drift.

## Diagnostic live viewer

Run this whole block from the repository root in a graphical Ubuntu session:

```bash
(
  set -euo pipefail

  D435I_SERIAL="843212070146"
  SELECTED_DIR="config/local/d435i-${D435I_SERIAL}/selected_runtime"
  ESTIMATOR_CONFIG="${SELECTED_DIR}/estimator.yaml"
  STREAM_CONFIG="config/sensors/realsense_streams_vio_90hz.yaml"
  LIVE_RUN="runs/live_diagnostic_$(date -u +%Y%m%dT%H%M%SZ)"

  ./scripts/verify_selected_runtime.sh

  ./scripts/preflight_ubuntu.sh --require-build
  ./scripts/preflight_ubuntu.sh \
    --require-camera \
    --serial "${D435I_SERIAL}" \
    --stream-config "${STREAM_CONFIG}"

  ./build/linux-release/ovrs_live \
    --config "${ESTIMATOR_CONFIG}" \
    --stream-config "${STREAM_CONFIG}" \
    --serial "${D435I_SERIAL}" \
    --viewer \
    --viewer-history 5000 \
    --allow-unverified-calibration \
    --online-time-offset off \
    --output "${LIVE_RUN}"

  test ! -e "${LIVE_RUN}/INCOMPLETE"
  printf 'Completed live run: %s\n' "${LIVE_RUN}"
)
```

Viewer controls:

- left-drag: orbit;
- middle/right-drag: pan;
- mouse wheel: cursor-centred zoom;
- `F`: fit the trajectory without changing view direction;
- `R`, `0`, or double-left-click: reset the isometric view;
- `q` or Escape: clean shutdown.

The viewer includes IR1/IR2, a world-locked grid and compact labelled XYZ
axes, start/current markers, axis spans, path length, and displacement. `F`
explicitly fits the accumulated path; incoming states do not silently move or
rescale the world. The canvas follows the resized window aspect ratio. It is
not RViz and it does not provide ground truth.

## Replacement rule

Replace this selection only with stronger serial-bound evidence: a larger
accurately measured rigid AprilGrid, repeated independent Kalibr results,
post-device-update Allan characterization, controlled identical-data replay,
moving visual/gyro scale validation, and connected-camera acceptance. Preserve
failed datasets and this evidence instead of overwriting them.
