# Dataset formats

## Replay dataset: `ovrs-euroc-like-v1`

```text
dataset/
  dataset_metadata.yaml
  device_report.yaml
  resolved_stream_config.yaml
  recording_summary.yaml
  INCOMPLETE                 # present until clean finalization
  cam0/
    data.csv
    data/<frameset_number>.png
  cam1/
    data.csv
    data/<frameset_number>.png
  imu/
    gyro.csv
    accelerometer.csv
    synchronized.csv
```

Images are lossless 8-bit grayscale PNG. Camera CSV columns are normalized
timestamp seconds, original device timestamp milliseconds, frameset number,
and relative filename. The two camera rows must have equal frameset numbers and
timestamps within the configured stereo tolerance.

`gyro.csv` stores the gyroscope signal in gyro coordinates in rad/s after the
configured `gyro_scale_factor` is applied. `accelerometer.csv` stores the
SDK-delivered accelerometer signal in
accelerometer coordinates in m/s^2 before the project's accel-to-gyro rotation.
Before motion streaming, OVRS explicitly selects the configured dynamic D435i
gyro-sensitivity index and reads it back. The selected index, availability,
active index, and firmware/SDK description are recorded in
`device_report.yaml`; the resolved index is also preserved in
`resolved_stream_config.yaml`. The project scale is also recorded as configured
and applied values in `device_report.yaml` and as an applied value in dataset
metadata. Replay consumes recorded values unchanged; it never silently
reapplies the factor.
When RealSense motion correction is enabled, librealsense applies the available
factory correction to both streams. That active option is provenance, not proof
that the returned scale/cross-axis correction is accurate enough for VIO;
multi-orientation validation remains required. “Raw” here means SDK-delivered,
not uncalibrated device register values.
`synchronized.csv` stores gyro timestamps, angular velocity in gyro axes,
linearly interpolated acceleration rotated into gyro axes, and interpolation
delay. All numeric CSV values use enough decimal precision to round-trip a
double.

`device_report.yaml` records the serial, firmware, USB descriptor, selected
profiles, actual selected rates, gyro-sensitivity request/readback, gyro scale,
timestamp domains, and axis policy. When the infrared frame metadata is
available, the final report also records the sample count and the last,
minimum, maximum, and mean actual exposure in microseconds and sensor gain.
These are runtime readbacks; the requested auto-exposure setting alone does
not prove how the imager exposed a dark scene.
`recording_summary.yaml` records malformed frames, timestamp/callback errors,
queue drops, and effective rates. A clean recording cannot finalize with a
nonzero malformed-frame, rejected-timestamp, or callback-error counter.

The recorder validates its configuration and starts the selected camera
profile before creating `INCOMPLETE` or any dataset files. It removes the
marker only after capture stops, queues drain, and CSV streams close
successfully. Replay refuses that marker. Replay also requires `complete:
true`, the exact format name, a device report, a resolved stream
configuration, matching camera row counts, finite ordered IMU values, and a
dataset serial matching the estimator calibration. Camera timestamps, raw
timestamps, and frameset numbers must increase strictly; each filename must be
`<frameset_number>.png`; and every decoded image must be Y8 at the exact
recorded resolution.

A finalized capture can contain a small leading stereo prefix before the first
synchronized IMU row because the camera and motion streams start
asynchronously. Replay decodes and validates those images, skips only that
unbracketed leading prefix, and records
`skipped_leading_stereo_without_imu_bracket` in `application.log`. It never
shifts timestamps, synthesizes IMU, extrapolates, or tolerates a missing IMU
bracket after replay has begun. The selected 90 Hz hardware capture exercised
this policy with exactly one skipped stereo pair.

Recovery is manual:
preserve the original, inspect final complete CSV rows and matching PNG pairs,
remove incomplete tail rows/files if needed, then remove the marker only after
review.

Replay and live run directories use the same fail-closed marker name.
`RunWriter` creates `INCOMPLETE` before opening output streams and removes it
only after every run file flushes during explicit successful finalization.
Interrupted replay, missing initialization/state, camera disconnect, runtime
failure, or write failure retains the marker. A run directory with
`INCOMPLETE` is diagnostic partial output, not a successful estimator result.
Both modes preserve `resolved_stream_config.yaml`: live writes its resolved
capture settings and replay copies the exact resolved stream file from the
dataset.

## Markerless VSLAM benchmark export: `ovrs-vislam-benchmark-v1`

The v0.6.0 research exporter converts a complete replay dataset without
changing the source:

```text
benchmark/
  benchmark_manifest.yaml
  INCOMPLETE                         # present until clean finalization
  mav0/
    cam0/
      data.csv
      data/<timestamp_ns>.png
    cam1/
      data.csv
      data/<timestamp_ns>.png
    imu0/
      data.csv
  ovrs_metadata/
    dataset_metadata.yaml
    device_report.yaml
    resolved_stream_config.yaml
    recording_summary.yaml
```

The two camera indexes use the same pair-midpoint timestamp in integer
nanoseconds. IMU values come from `imu/synchronized.csv`: angular velocity and
linearly interpolated acceleration are both in the D435i gyroscope frame.
Source metadata SHA-256 values, camera serial, row counts, timestamp ranges,
image-transfer mode, and hashes of all three generated CSV indexes are recorded
in the benchmark manifest.
Stereo pairs before the first or after the last synchronized IMU timestamp are
not exported, matching replay's requirement for IMU coverage. Source, exported,
leading-skip, and trailing-skip counts remain explicit.

The exporter accepts only complete `vio` recordings whose recorder-fatal
capture counters are zero. Unmatched IMU interpolation brackets at capture
boundaries remain visible in the copied summary but are not fatal because no
unbracketed sample appears in `synchronized.csv`. Unsafe filenames, missing
images, unequal stereo counts, frameset mismatches, timestamps beyond the
recorded tolerance, non-finite values, and non-increasing timestamps fail
closed. A failed export keeps `INCOMPLETE`.

The final manifest state is `EXPORTED_NOT_EVALUATED`. It proves neither backend
execution nor trajectory accuracy. Backend configuration, calibration
conversion, results, and independent ground truth belong to a separate
experiment manifest described in
[the markerless VSLAM research plan](vislam_research_plan.md).

## ORB-SLAM3 adapter: `ovrs-orbslam3-adapter-v1`

The project-owned adapter creates a separate view of a complete neutral
benchmark. It preserves IMU samples, relabels camera files in the IMU clock
using the selected fixed camera-to-IMU offset, derives the upstream camera/body
and stereo transforms, and generates an ORB-SLAM3 settings file. It records
input and output hashes and finishes in `PREPARED_NOT_RUN`.

`--camera-stride N` keeps every Nth stereo pair while retaining every IMU row.
The source rate, adapted rate, stride, and skipped pair count are explicit in
the adapter manifest. The source camera rate must be divisible by the stride.
This supports a same-recording cadence A/B; it does not change the selected
live stream.

The default `--camera-time-offset-policy calibrated` applies the selected
fixed offset. `--camera-time-offset-policy zero` exists only for a controlled
timing A/B. The manifest records the calibrated and applied values separately,
so a zero-offset diagnostic cannot be mistaken for the selected calibration.

The adapter rejects a neutral export created before the generated CSV hashes
were added; rerun the exporter instead of manually editing its manifest. It
also rejects an input recorded with a gyro scale other than `1`, inactive SDK
motion correction or global time, mismatched serial/resolution/calibration
state, malformed indexes, or unbracketed adjusted timestamps. Full commands
and the first smoke-test boundary are in
[the ORB-SLAM3 offline notes](orbslam3_offline.md).

## ORB-SLAM3 result: `ovrs-orbslam3-result-v1`

The supported runner captures the backend log and process exit status before
calling the result evaluator. The result manifest hashes the adapter manifest,
backend executable, dynamically linked ORB-SLAM3 library, vocabulary, log,
exit-status file, frame trajectory, and keyframe trajectory.
For an ELF runner it also records that `ldd` resolved the requested
`libORB_SLAM3.so`. A multi-session run binds the staged parent atlas manifest
and records whether that parent passed a complete reload/merge/tracking gate.

The evaluator validates ordered finite active-session frame poses, normalized
quaternions, input timestamp coverage, final atlas and keyframe counts,
inertial BA completion, IMU-map resets, local-map tracking failures, loop
candidates, rejected candidates, and applied corrections. A merged atlas may
write keyframes in map/KeyFrame-ID order, so keyframe timestamp ranges may
restart at a session boundary; their numeric and quaternion validation remains
mandatory. A non-empty trajectory alone is never a pass.
Accumulated atlas keyframes may legitimately outnumber frames in the current
revisit session; that relationship is therefore not used as a multi-session
rejection rule.

Without an independent reference the result remains either
`TRACKING_PASS_NO_LOOP_CORRECTION` or
`TRACKING_PASS_LOOP_CORRECTION_NOT_REFERENCE_VALIDATED`. The latter records
that the pinned backend entered its correction path; it does not certify that
the loop was true.

An optional closed-loop reference uses
`ovrs-closed-loop-reference-v1`. It supports only a physically colocated
start/end pose established independently of the images and never consumed by
the estimator. The manifest records the placement tolerances and estimated
return residual. It reports whether that residual is consistent with the
recorded placement tolerance, but does not reinterpret fixture repeatability
as full trajectory accuracy.

The isolated live ORB evaluator uses the stricter
`ovrs-closed-loop-reference-v2` extension documented in
[the ORB-SLAM3 live guide](orbslam3_live.md). It adds predeclared start/end
hold windows, minimum sample count, and within-window position/orientation
dispersion limits, plus minimum planned path duration and excursion. The
offline evaluator retains v1 because its historical recorded sequences do not
contain a gate-open endpoint-hold contract. Neither reference format is
consumed by an estimator.

Each initialized row in `state.csv` records the latest non-empty MSCKF batch:
candidate features before the updater, accepted features after triangulation,
refinement, and chi-square rejection, accepted/candidate ratio, and batch age
in seconds. `diagnostics.csv` samples the same fields at the configured
diagnostic rate. Values are `NA` until the first non-empty batch. These fields
are observational and do not alter the filter or define an accuracy threshold.

## Calibration capture: `ovrs-calibration-capture-v1`

The directory contains the same metadata and only the streams selected by its
mode:

- `imu-allan`: `imu/` only;
- `stereo-calibration`: `cam0/`, `cam1/`, and
  `calibration_target.yaml`;
- `imu-camera-calibration`: both camera directories, `imu/`, and the target.

Calibration captures set `replay_compatible: false`. They cannot be passed to
`ovrs_replay`.

Final metadata records capture mode, serial, actual stream profile/rates,
motion-correction state, requested and observed RealSense timestamp policy,
stationary confirmation where required, and target presence.
`recording_summary.yaml` includes camera, queue, timestamp, callback, and IMU
synchronizer integrity counters. Any counter treated as fatal by the recorder
leaves `INCOMPLETE`.

## Portable calibration export: `ovrs-calibration-export-v2`

`scripts/export_calibration_capture.py` validates a complete calibration
capture and writes:

```text
export/
  calibration_export_manifest.yaml
  README.txt
  target.yaml                         # target-based modes
  cam0/<timestamp_ns>.png             # stereo modes; 19 digits
  cam1/<timestamp_ns>.png
  imu0.csv                            # motion modes
  allan_variance_config.yaml          # imu-allan only
  ovrs_metadata/
    source_dataset_metadata.yaml
    source_recording_summary.yaml
    source_device_report.yaml
    source_resolved_stream_config.yaml
    source_calibration_target.yaml       # target-based modes
    cam0_index.csv
    cam1_index.csv
    imu_raw/
      gyro.csv
      accelerometer.csv
```

The root image/CSV layout is the official `kalibr_bagcreater` staging
contract. `imu0.csv` has
`timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z`, with nanosecond
timestamps and synchronized post-correction values in the gyro frame.
Image and IMU timestamps are zero-padded to 19 decimal digits because the
pinned `kalibr_bagcreater` splits the final nine digits from the seconds field.
`ovrs_metadata` preserves source seconds, device milliseconds, frameset
numbers, original filenames, separately sampled motion streams, and the
original measured target. Root `target.yaml` is a value-equivalent standard
YAML staging file accepted by the pinned Kalibr/PyYAML toolchain.

The export manifest says `UNVERIFIED_CAPTURE`, `ros_bag_created: false`, and
`kalibr_executed: false`. It records the verified `global_time_enabled` policy
and contains flat SHA-256 fields for every copied source metadata file and for
both target files when present. Downstream preparation and validation re-hash
those fixed paths and reject missing, modified, or mixed-clock provenance.
Export never promotes calibration.
