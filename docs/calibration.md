# Calibration contract

This repository has three distinct calibration states:

| State | Meaning | Estimation allowed by default |
| --- | --- | --- |
| `BOOTSTRAP_UNVERIFIED` | RealSense factory intrinsics/extrinsics plus conservative IMU placeholders | No |
| `UNVERIFIED_CAPTURE` / `UNVERIFIED_KALIBR_INPUT` | Validated evidence or intermediate files; external analysis/manual review incomplete | No |
| `KALIBR_VERIFIED` | Serial-specific Kalibr/Allan results passed structural checks and explicit human review | Yes |

Changing a state string is not calibration. Promotion is performed only by
`scripts/prepare_verified_calibration.py`, which verifies source hashes and
requires all review acknowledgements.

## v0.5.0 camera-transform contract

Pinned OpenVINS v2.7 reads this camera transform key:

```yaml
T_imu_cam:
```

Its upstream comment defines it as the rotation from camera to IMU and the
position of the camera origin expressed in IMU coordinates. The pinned parser
also has a compatibility path: if only `T_cam_imu` exists, it reads that
matrix and applies an SE(3) inverse before returning `T_imu_cam`.

Kalibr instead emits:

```yaml
T_cam_imu:
```

Kalibr defines this as the transform from IMU coordinates to camera
coordinates. These matrices are inverses:

```text
T_imu_cam = inverse(T_cam_imu)
```

Before v0.5.0, the repository passed `T_cam_imu` into OpenVINS configuration.
That path was accepted by the upstream compatibility parser, so it is not a
proven root cause of the observed trajectory divergence. It was nevertheless
an unsafe project contract because a human could rename the key without
inverting the matrix or compare opposite directions as if they were equal.

v0.5.0 enforces the boundary:

- `ovrs_inspect --export-calibration` directly asks librealsense for the
  camera-to-gyro extrinsic and writes `T_imu_cam`;
- runtime validation requires exactly two `T_imu_cam` keys and rejects any
  `T_cam_imu` key;
- Kalibr output validation checks the original `T_cam_imu` values;
- promotion rigid-transform-validates and inverts both matrices before writing
  OpenVINS files;
- the one-time migration script preserves the complete legacy YAML before
  replacing its two matrices.

No transform is renamed without inversion.

The pre-v0.5.0 divergent trajectory remains failed evidence because it used
bootstrap, not unit-verified, calibration and disagreed grossly with the
operator's motion. It must not be presented as valid, but v0.5.0 does not
attribute that divergence to the legacy key alone.

## Coordinate frames

The application uses the D435i gyroscope coordinates as its IMU frame.
librealsense accelerometer samples are rotated into the gyro frame using the
factory accel-to-gyro extrinsic before synchronization and estimator
ingestion.

`T_imu_cam` maps a homogeneous camera-coordinate point into IMU coordinates:

```text
p_imu = T_imu_cam * p_cam
```

For this direction, the translation column is the camera origin expressed in
IMU coordinates. The stereo baseline is the Euclidean distance between the two
translation columns. Both matrices must have an orthonormal rotation,
determinant +1, homogeneous bottom row `[0, 0, 0, 1]`, and a nonzero baseline.

Kalibr's `T_cam_imu` maps the opposite direction:

```text
p_cam = T_cam_imu * p_imu
```

For a rigid transform with rotation `R` and translation `t`, promotion uses:

```text
R_inverse = transpose(R)
t_inverse = -transpose(R) * t
```

## Factory bootstrap limits

`ovrs_inspect --export-calibration` exports the selected Y8 intrinsics,
factory IR/IMU extrinsics, serial, resolution, RealSense distortion metadata,
and a zero camera/IMU time offset. It does not:

- measure the printed target;
- optimize camera intrinsics or stereo extrinsics;
- estimate camera/IMU clock offset;
- characterize noise density or random walk;
- estimate IMU scale, cross-axis, or g-sensitivity;
- run Kalibr;
- verify VIO accuracy.

Therefore factory output always remains `BOOTSTRAP_UNVERIFIED`.

Y16 stream profiles may not expose video intrinsics. The inspector lists them
without aborting. Export uses only the selected Y8 IR1/IR2 profile.

## Motion-correction policy

The stream configuration explicitly requests RealSense factory motion
correction. The source sets and reads back
`RS2_OPTION_ENABLE_MOTION_CORRECTION`; device report, dataset metadata, and IMU
configuration must agree.

After SDK motion correction, the project rotates accelerometer values into the
gyro frame. The portable Kalibr `imu0.csv` contains this synchronized
post-correction signal. Under that exact policy, the generated OpenVINS IMU
intermediate represents `Tw`, `Ta`, `R_IMUtoGYRO`, and `R_IMUtoACC` as
identity. It uses zero `Tg` because g-sensitivity was not estimated and online
g-sensitivity calibration is disabled. The generator refuses to make this
representation if motion correction was inactive.

The unmodified separately sampled gyro/accelerometer CSV files are retained
only as provenance under `ovrs_metadata/imu_raw`; Kalibr and Allan use the
synchronized `imu0.csv`.

## Capture modes

`ovrs_record` has four explicit modes:

| Mode | Stereo | Motion | Target required | Stationary confirmation | Replay compatible |
| --- | --- | --- | --- | --- | --- |
| `vio` | Yes | Yes | No | No | Yes |
| `imu-allan` | No | Yes | No | Yes | No |
| `stereo-calibration` | Yes | No | Yes | No | No |
| `imu-camera-calibration` | Yes | Yes | Yes | No | No |

Calibration capture fails closed on interruption, disconnect, missing streams,
nonfinite values, timestamp rejection/regression/duplication, queue or
synchronizer capacity drop, camera-frame drop, malformed frame, callback
error, or file-write failure. The `INCOMPLETE` marker is removed only after all
files and final metadata flush successfully.

## AprilGrid target

The board is an official Kalibr-generated AprilGrid print mounted on a flat,
rigid white backing. For A4, run `scripts/plan_aprilgrid_target.py` before
generation; it checks active grid size, printer margins, and Kalibr's minimum
one-grid-element white border. The generated PDF page follows its content, so
the print dialog must centre it on A4 at actual size and visibly retain that
physical border. Do not scale the official A0 download to A4.

Print at actual size, measure multiple black tag edges and white gaps after
printing, then create target YAML from those physical dimensions:

```bash
.venv/bin/python scripts/create_aprilgrid_target.py \
  --tag-rows ROWS \
  --tag-cols COLUMNS \
  --tag-size-mm MEASURED_TAG_EDGE_MILLIMETRES \
  --tag-gap-mm MEASURED_ADJACENT_GAP_MILLIMETRES \
  --output calibration/target.yaml
```

The generator computes Kalibr `tagSpacing` as gap divided by tag size. It has
no default target dimensions and refuses overwrite.

## Allan noise workflow

Longer stationary captures expose lower-frequency IMU behavior, but duration
is provenance rather than a quality certificate. The repository therefore
records a positive capture duration without imposing a hardcoded minimum or
requiring a short-sample override. A one-hour run can be processed normally;
its Allan CSV, fitted region, plots, timestamp integrity, stationary
diagnostics, and repeatability still require review. A longer run must be
rejected too when those checks fail.
After capture, the root README runs `analyze_stationary_imu.py` on an
operator-selected interval of at least 60 seconds. Its default result is a
diagnostic with `validation: NOT_REQUESTED`. A gravity-error threshold is
allowed only when an independent physical acceptance requirement was declared
before looking at the result. This optional gate can reject a gross
motion-correction/model mismatch, but one stationary orientation cannot
identify a three-axis scale, cross-axis matrix, or bias random walk.

The RealSense runtime requires the exact configured motion profiles instead
of silently selecting the nearest rates. Librealsense motion samples are
treated as rad/s and m/s². Accelerometer samples are rotated by the reported
accel-to-gyro extrinsics and interpolated to gyro timestamps; both the
requested/active rates and this axis policy are retained in the device report.

Before analysis or export, run `validate_calibration_capture.py` on the
completed capture. It streams all CSV rows with bounded memory, validates
strict timestamps and finite values, and requires the actual raw/synchronized
row counts to match `recording_summary.yaml`. This integrity pass does not
certify that the sensor was physically stationary or replace the Allan fit.
Sample a multi-hour recording near its beginning, midpoint, and end. A
repeatable gravity-magnitude mismatch must remain review evidence: do not
change estimator gravity or infer a three-axis `Ta` from one static
orientation. If the later Kalibr IMU residual/bias review cannot support the
factory-corrected identity intrinsic model, promotion stops until an
independent multi-orientation IMU intrinsic calibration is available.

The in-repository steps are:

1. Rigidly secure the camera and record `--capture-mode imu-allan
   --confirm-stationary`.
2. Run the stationary diagnostic. If an independently declared bound is also
   applied, stop if that bound fails.
3. Export with `scripts/export_calibration_capture.py`.
4. Preserve `imu0.csv`, raw provenance, export manifest, and the generated
   `allan_variance_config.yaml`. Export v2 also carries SHA-256-bound copies
   of the capture metadata, summary, device report, and stream configuration.
5. In a separate supported ROS1 environment, create `allan.bag` with
   `kalibr_bagcreater`.
6. Run the official `allan_variance_ros` computation and `analysis.py`.
7. Inspect the Allan plots and record whether/how values were inflated. The
   upstream OpenVINS guide suggests testing inflation for unmodelled errors; it
   is not applied automatically here.
8. Feed the reviewed Allan `imu.yaml` plus the matching Allan and
   IMU-camera export manifests to
   `scripts/prepare_imu_calibration_yaml.py`.

That script verifies the same serial, gyro rate, and motion-correction policy,
re-hashes the copied export provenance, and requires the Allan YAML itself to
report `/imu0` at the captured gyro rate.
It produces:

- `kalibr_imu.yaml`: the flat Kalibr input using `/imu0`;
- `openvins_imu.yaml`: extended OpenVINS IMU structure, still unverified;
- a provenance manifest with all input hashes and matrix policies.

Allan variance characterizes noise and random walk; it does not estimate the
accelerometer/gyroscope scale or cross-axis matrices. Active RealSense motion
correction is recorded as provenance but is not sufficient evidence that
identity `Ta`/`Tw` matrices are accurate. Identity outputs remain marked
`IDENTITY_ASSUMPTION_REQUIRES_MULTI_ORIENTATION_REVIEW` and cannot be promoted
until an independent, reviewed multi-orientation intrinsic calibration is
supplied.

After a separate multi-orientation calibration has produced reviewed `Tw`,
`Ta`, `R_IMUtoGYRO`, `R_IMUtoACC`, and `Tg` matrices using the same
motion-correction policy, pass that YAML with both
`--imu-intrinsics-yaml PATH` and
`--acknowledge-multi-orientation-intrinsics-reviewed`. The script checks matrix
shape, finiteness, rotation validity, nonsingular scale matrices, policy
agreement, and records the source SHA-256. This is a software-only workflow and
never writes device EEPROM.

The D435i estimator template uses ZUPT only during the initial stationary
phase. Continuous ZUPT is an opt-in experiment requiring calibrated,
ground-truthed replay; slow handheld motion can otherwise resemble a stop.

## Stereo and camera-IMU workflow

The runtime remains ROS-free. Kalibr requires ROS1, so conversion and
optimization occur in a separate Ubuntu 20.04 environment: a supported
machine/VM or the pinned Docker workflow in the root README. Repository scripts
do not install Docker or ROS on the Ubuntu 24.04 host.

The Docker workflow uses `docker/calibration.Dockerfile` to extend the pinned
Kalibr image with the pinned Allan source. It does not rely on mutable
`docker commit` state. Before bag creation,
`scripts/validate_calibration_export_set.py` re-hashes export provenance and
requires one serial, matching stereo/IMU-camera IR profiles and targets, and
matching Allan/IMU-camera rates plus active motion correction. It also binds
manifest fields back to the hashed source metadata and checks staged camera
index/image counts, image dimensions, and IMU row counts. Its
`UNVERIFIED_EXPORT_SET` report is a coherence gate, not promotion.

Kalibr's catkin build installs its Python commands as ROS package executables,
not as bare commands guaranteed on `PATH`. The supported container workflow
therefore runs the target generator, bag creator, and both calibrators as
`rosrun kalibr <tool>`. The image-build gate checks all four `--help`
interfaces before the image is accepted.

1. Record a target-rich `stereo-calibration` dataset with `--preview`.
2. Record a smooth 30–60 second `imu-camera-calibration` dataset. Excite all
   rotations, multiple translations, avoid shocks/blur, and keep the fixed
   target visible; use the same recorder preview.
3. Export both. The exported tree follows the official bag creator layout:
   `cam0/<timestamp_ns>.png`, `cam1/<timestamp_ns>.png`, and root
   `imu0.csv`.
4. Run `kalibr_bagcreater` in the external ROS1 environment.
5. Run `kalibr_calibrate_cameras` with two `pinhole-radtan` models and
   `--show-extraction`; inspect detections and its PDF. OpenVINS guidance
   describes good final reprojection errors as
   roughly below 0.2–0.5 pixels; treat this as a review criterion, not a
   repository-enforced magic number.
6. Run `kalibr_calibrate_imu_camera --show-extraction` with the reviewed
   stereo camchain and generated Kalibr IMU YAML.
7. Inspect camera/IMU residuals, 3-sigma bounds, bias behaviour, timestamp
   deltas, target detections, transform direction, baseline, time-offset sign,
   and physical plausibility.

Official references:

- <https://docs.openvins.com/gs-calibration.html>
- <https://github.com/ethz-asl/kalibr/wiki/camera-imu-calibration>
- <https://github.com/ethz-asl/kalibr/wiki/bag-format>
- <https://github.com/ethz-asl/kalibr/wiki/yaml-formats>
- <https://github.com/ori-drs/allan_variance_ros>

## Structural validation and promotion

`scripts/validate_kalibr_outputs.py` checks:

- serial, capture mode, resolution, rate, and motion policy;
- finite positive intrinsics/noise values;
- rigid `T_cam_imu`, nonzero stereo baseline, and finite time offsets;
- an operator-supplied cam0/cam1 time-offset disagreement limit;
- PDF signatures and source hashes;
- extended IMU matrices and exactly zero `Tg`.

Its success verdict is `STRUCTURAL_PASS_MANUAL_REVIEW_REQUIRED`. It deliberately
leaves a checklist unchecked.

`scripts/prepare_verified_calibration.py` then:

1. rechecks the structural report and current source hashes;
2. requires four explicit human-review acknowledgements;
3. requires the operator to choose `cam0` or `cam1` as the single shared
   time-offset source used by OpenVINS v2.7;
4. validates and inverts both Kalibr `T_cam_imu` matrices to `T_imu_cam`;
5. writes only
   `config/local/d435i-SERIAL/kalibr`, refusing an existing destination;
6. records all hashes, acknowledgements, and the offset-source decision.

Only that output is labelled `KALIBR_VERIFIED`.

## Rejection rules

Reject and recapture/recalibrate if any of these occur:

- serial, resolution, rate, or motion policy mismatch;
- `INCOMPLETE` marker or any nonzero integrity counter;
- insufficient target coverage or blurred frames;
- missing camera/IMU topics in the generated bag;
- poor camera residuals;
- residuals/biases outside reviewed uncertainty bounds;
- timestamp batching, discontinuity, or unexplained time-offset sign;
- invalid/degenerate transform or physically implausible baseline;
- nonzero `Tg` while the estimator's g-sensitivity calibration remains off;
- severe trajectory divergence after the corrected verified configuration.

Do not repair a rejected result by changing state labels, copying example
numbers, averaging offsets silently, clipping trajectories, or relaxing
physical bounds after seeing the output.
