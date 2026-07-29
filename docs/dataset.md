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
