# Markerless VSLAM research plan

## Objective

The v0.6.0 branch investigates:

> Robust markerless stereo-inertial relocalization and loop-closure integration
> for a D435i on Raspberry Pi 5 under low-light and low-texture indoor
> conditions.

The operating environment must not contain AprilTags, ArUco boards, QR codes,
or other fiducials used by the estimator. The system must rely on the D435i
infrared stereo pair and IMU.

This is a research objective, not a statement that the current runtime already
provides SLAM. The v0.5.2 OpenVINS path remains the reference local odometry
implementation.

## Why the work starts from this repository

The repository already provides the parts that must remain identical during a
fair estimator comparison:

- bounded D435i capture and owned Y8 images;
- synchronized stereo and IMU recording;
- normalized and raw timestamps;
- serial-bound calibration and stream provenance;
- deterministic replay;
- capture-integrity and state safety gates;
- trajectory and diagnostic output.

Replacing this capture path while changing the estimator would confound sensor
transport, exposure, timing, calibration, and backend behaviour. The first
research phase therefore exports one validated recording to every candidate
backend.

## Scope boundaries

Included:

- backend-neutral offline dataset export;
- OpenVINS odometry baseline measurements;
- ORB-SLAM3 stereo-inertial evaluation;
- OKVIS2 evaluation;
- persistent map, place recognition, loop closure, and relocalization
  experiments;
- desktop and Raspberry Pi 5 CPU, memory, latency, temperature, and power
  measurements;
- a future project-owned adapter after a backend passes the offline gates.

Excluded:

- fiducial markers;
- ROS or ROS2 integration;
- GPS, MAVLink, ArduPilot, Pixhawk, navigation, or flight control;
- depth or RGB processing;
- modifying OpenVINS state directly with a loop-closure correction;
- claiming accuracy without independent reference data.

## Candidate order

| Order | Candidate | Purpose |
| --- | --- | --- |
| 1 | OpenVINS v2.7 | Existing local-odometry control |
| 2 | ORB-SLAM3 stereo-inertial | First full-SLAM baseline |
| 3 | OKVIS2 | Independent keyframe visual-inertial SLAM comparison |
| 4 | Project-owned map overlay | Only if evidence justifies custom work |

External backends must be pinned to exact commits after license and build
review. They are not added as unpinned package-manager dependencies or copied
casually into the repository.

The first ORB-SLAM3 pin, desktop build, adapter, failed-initialization smoke
test, and controlled loop-sequence diagnostics are recorded in
[the offline ORB-SLAM3 baseline](orbslam3_offline.md). Stereo-only tracking
completed the first controlled sequence, while its inertial variants reset. A
second motion-focused sequence initialized stereo-inertial tracking without
reset in two identical runs; one repeat applied a loop correction. This passes
backend initialization. A later rigid-stop referenced sequence repeated a
small return residual twice without a loop correction. It does not yet pass
the full-trajectory accuracy, repeatable-loop, or relocalization gates below.
The adapter now supports hashed atlas save/load experiments through the
upstream multi-session merge path. Chained offline reload has passed on one
repeated connected-camera recording, but no distinct-revisit, false-merge,
live-atlas, or ground-truth result has been collected yet.

## Staged implementation

### Gate 0: freeze the baseline

- Keep `main` at the reviewed v0.5.2 commit.
- Perform all work on `research/v0.6.0-markerless-vislam`.
- Do not change selected calibration, gyro scale, capture queues, timestamp
  normalization, or OpenVINS ingestion for an offline backend experiment.
- Record the source commit and dirty-tree state with every result.

### Gate 1: prepare identical backend input

Use the project-owned exporter:

```bash
python3 scripts/export_vislam_benchmark.py \
  --dataset datasets/RECORDING \
  --output runs/benchmarks/RECORDING_euroc \
  --image-mode hardlink
```

Use `--image-mode copy` when the source and destination are on different
filesystems.

The exporter:

- accepts only complete, replay-compatible VIO recordings;
- rejects counters treated as fatal by the recorder; unmatched IMU
  interpolation brackets at capture boundaries remain recorded provenance but
  are not reclassified as a fatal error;
- checks ordered stereo rows, matched framesets, timestamp tolerance, safe
  filenames, present images, and ordered finite IMU rows;
- writes the standard `mav0/cam0`, `mav0/cam1`, and `mav0/imu0` layout;
- exports only stereo pairs inside the synchronized IMU time range and records
  skipped leading or trailing pairs;
- preserves normalized time by converting decimal seconds to integer
  nanoseconds;
- records stereo pair-midpoint timing and the gyro-frame acceleration policy;
- copies source metadata and records SHA-256 provenance;
- leaves `INCOMPLETE` after a partial failure;
- labels a complete export `EXPORTED_NOT_EVALUATED`.

An export pass proves format and provenance only. It does not prove that a
backend ran or that its trajectory was accurate.

### Gate 2: offline backend adapters

Each adapter must consume the same export and produce:

```text
benchmark_run/
  experiment_manifest.yaml
  trajectory_tum.txt
  backend.log
  timing.csv
  resources.csv
  evaluation.yaml
  INCOMPLETE
```

The manifest must bind:

- repository commit;
- source benchmark-manifest hash;
- backend name and exact commit;
- backend configuration hash;
- camera/IMU calibration hashes;
- map mode: odometry, map build, relocalization, or loop evaluation;
- host and architecture;
- whether independent ground truth is present.

Backend-specific camera and IMU frame conversions must be written explicitly.
No adapter may infer a transform from a filename or silently invert a
calibration matrix.

### Gate 3: controlled dataset matrix

Every candidate receives the same sequence categories:

| Category | Required motion or condition |
| --- | --- |
| Stationary start | At least 10 seconds before deliberate motion |
| Rotation | Slow and fast yaw, roll, and pitch in place |
| Up/down view | Ceiling and floor transitions, then return |
| Translation | Measured straight movement and return |
| Loop | Leave and revisit the starting region |
| Corridor | Forward and reverse travel with distant structure |
| Poor texture | Plain wall, floor, or ceiling without fiducials |
| Low light | Fixed lighting levels recorded separately |
| Blur | Increasing angular speed until controlled tracking loss |
| Relocalization | Restart and localize in a previously saved map |
| Recovery | Occlusion or tracking loss followed by a mapped revisit |
| Repetition | Similar doors, tiles, or structural patterns |

Lighting, emitter state, stream configuration, calibration, and physical
sequence must be recorded rather than reconstructed from memory.

Public datasets with external reference trajectories may be used for algorithm
validation. D435i-specific accuracy claims require an independent reference
that is not consumed by the estimator. A visual marker is not introduced into
the operating scene for this project.

### Gate 4: metrics and rejection rules

Required accuracy metrics:

- Absolute Trajectory Error after metric rigid alignment;
- translational and rotational Relative Pose Error;
- final loop error;
- metric scale error;
- tracking-loss count and duration;
- relocalization success rate and latency;
- loop-closure precision, recall, and correction magnitude.

Required runtime metrics:

- input-to-pose latency and pose output rate;
- CPU utilization per core;
- peak resident memory;
- temperature and throttling state;
- average and peak power where measured;
- dropped input or output measurements.

Immediate rejection conditions:

- a false loop closure accepted as valid;
- a backend consumes ground truth or a fiducial during estimation;
- timestamp or calibration provenance is incomplete;
- metric scale is recovered from the evaluation trajectory;
- a partial run is reported as complete;
- desktop success is described as Raspberry Pi 5 success;
- local pose continuity is broken by an unsmoothed global correction.

Threshold values are set in the experiment protocol before running a sequence.
They are not selected after inspecting the result.

### Gate 5: live desktop integration

Only the selected offline backend is integrated live. Initial live work is
headless and diagnostic:

- no output goes to flight hardware;
- bounded queues remain mandatory;
- OpenVINS behaviour remains available as an A/B baseline;
- map construction and optimization cannot block the capture callback or
  ordered IMU/camera ingestion;
- canonical output requires sustained, provenance-pinned visual map support
  and bounded pose rates in addition to backend tracking state, inertial BA2,
  reset safety, and frame continuity; these are rejection envelopes, not
  accuracy metrics or platform-dynamics measurements;
- map state and local odometry state remain separate.

The intended transform contract is:

```math
T_{\mathrm{map}\leftarrow\mathrm{body}}
=
T_{\mathrm{map}\leftarrow\mathrm{odom}}
T_{\mathrm{odom}\leftarrow\mathrm{body}}.
```

`T_odom_body` is continuous local motion. Loop closure updates
`T_map_odom`. It must not rewrite the OpenVINS covariance, clones, bias, or
velocity state.

### Gate 6: persistent map and relocalization

A persistent map is produced during a deliberate commissioning traversal, not
by leaving the camera stationary at startup. Saved maps must contain a
manifest binding the map to:

- D435i serial;
- camera and IMU calibration hashes;
- stream profile;
- backend and configuration commits;
- descriptor vocabulary hash where applicable;
- coordinate-frame convention;
- creation dataset and map revision.

At a later startup the system reports one of:

- `LOCALIZED`: geometric verification found a valid map pose;
- `LOCAL_ODOMETRY_ONLY`: local tracking works but global localization is not
  established;
- `TRACKING_LOST`: neither local tracking nor map localization is valid.

No pose is silently presented in the map frame while localization is unknown.

For the pinned ORB-SLAM3 stereo-inertial baseline, the first implementation
uses upstream atlas save/load and requires a geometrically accepted map merge.
It does not enable the upstream localization-only switch: the inertial tracker
marks that path unsupported, and a loaded atlas starts with a new active map.
Until a separate project-owned adapter exposes a verified global pose state,
an accepted offline merge remains evidence for multi-session place
recognition, not a `LOCALIZED` live-runtime state.

### Gate 7: Raspberry Pi 5

Raspberry Pi 5 work begins only after a desktop backend passes accuracy and
false-loop gates. Optimization may reduce keyframe or map-optimization rates,
bound the active map, disable the viewer, and select build flags. It must not
silently lower the recorded sensor rate or change calibration.

A Pi result is real-time only when measured processing keeps up for the entire
sequence without an increasing queue backlog, thermal throttling, or dropped
measurements.

## Decision record

The backend decision must be evidence-driven:

1. prefer a mature backend when it meets accuracy, false-loop, and compute
   requirements;
2. retain OpenVINS for local odometry only if it materially improves pose
   continuity or resource use;
3. create a project-owned loop-closure overlay only when existing candidates
   fail a documented requirement or when the custom contribution is the
   explicit research objective;
4. do not merge the research branch into `main` until its public scope,
   dependencies, licenses, tests, and validated claims are reviewed.

Primary upstream references:

- [OpenVINS](https://docs.openvins.com/)
- [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [OKVIS2](https://github.com/ethz-mrl/okvis2)
