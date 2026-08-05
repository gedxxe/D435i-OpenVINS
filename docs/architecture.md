# Architecture

```text
D435i
 |-- IR1/IR2 ------> StereoSynchronizer --\
 `-- gyro/accel ---> ImuSynchronizer -------+--> Ordered dispatcher
                                                   |
                                                   v
                                           OpenVINS ov_msckf
                                                   |
                                                   v
                                native state + covariance + health + logs
                                                   |
                                      bounded latest snapshot
                                                   |
                                      main-thread OpenCV viewer
```

IR1 is OpenVINS camera 0 and IR2 is camera 1. Only Y8 data enters the
estimator. RGB, depth, and point-cloud streams are never enabled.

## Ownership and threading

`RealSenseSource` owns the librealsense pipeline and accepts only a
SDK-reported D435i, optionally narrowed by serial. Its callback validates
timestamps and frames, copies each Y8 buffer once into reference-counted owned
memory, and invokes enqueue callbacks. No estimator method runs there.

Recorder preview follows the same boundary. HighGUI runs only on the main
thread. Before recording it uses a disposable preview-only RealSense pipeline;
Space stops that pipeline before dataset creation. During recording it displays
the latest owned stereo pair after the main thread drains the bounded queue.
Closing the preview aborts the capture and keeps the dataset marked
`INCOMPLETE`.

`ImuSynchronizer` retains ordered accelerometer and gyro samples. A gyro is
emitted only after two acceleration measurements bracket it. `StereoSynchronizer`
accepts only camera 0/1 from the same frameset within tolerance.

`MeasurementDispatcher` owns a joinable worker thread. For each image it feeds
all earlier IMU samples and the first bracketing IMU sample at or after the
image, then feeds the stereo image. OpenVINS calls and state extraction are
therefore serialized. Queues are bounded; overflows increment counters rather
than growing without limit.

Shutdown order is capture stop, synchronizer stop, dispatcher drain/join, file
flush/close. A callback exception is recorded as a fatal diagnostic rather than
crossing the C callback boundary.

The project-owned tracking-health gate runs after OpenVINS state extraction.
It records the current camera-0 frontend track count as
`visual_support_features`. Persistent SLAM landmarks that were not observed
in that frame do not inflate this count, while a current track does not need
triangulated depth merely to report frontend continuity. Time hysteresis
prevents a single weak or strong frame from flipping status: the selected
runtime requires at least 12 support features for 1.5 consecutive seconds to
become `HEALTHY`, and marks the state `DEGRADED` after one consecutive second
below that threshold. A three-second warm-up timeout also rejects flickering
support that never becomes stable. A camera gap over 0.5 seconds resets the
gate to `WARMING_UP`.

The gate is observational only. It never modifies pose, velocity, covariance,
feature tracks, ZUPT behavior, or OpenVINS internals. `healthy=false` therefore
means the pose must not be trusted for accuracy, while `healthy=true` means
only that numerical validity and the configured minimum visual-support
contract passed. It is not ground truth and cannot detect every geometrically
wrong but internally accepted feature update.

For deeper diagnosis, the local OpenVINS patch also exposes read-only
statistics for the latest non-empty MSCKF batch. OVRS records the number of
candidate features before the updater, the number remaining after
triangulation, refinement, and chi-square rejection, their ratio, and the age
of that batch. These fields make `80 frontend tracks but 0/16 accepted MSCKF
features` visible instead of presenting both situations as equivalent.

MSCKF acceptance is intentionally not folded into the binary health gate.
Accepted-update ratios are event-driven, depend on motion and marginalization,
and can be low during legitimate stationary or weak-parallax intervals. The
two marked-pose replays showed different aggregate ratios but overlapping
short-window distributions, so a universal pass threshold would be fabricated.
The ratio is evidence for diagnosis, not ground truth.

`LiveViewer` is opt-in. The capture/dispatcher threads publish only
reference-counted owned image buffers and copied estimator states behind a
mutex. The application main thread alone calls OpenCV HighGUI. Trajectory
history has a configurable bound, window closure requests the same clean
shutdown path as Ctrl+C, and headless operation does not create a window.
Its view-controller math remains in `ovrs_core`, independent of OpenCV, so
orbit, pan, cursor-centred zoom, fit, and reset behavior are deterministic and
unit-tested. HighGUI owns only mouse/key delivery and rendering. The displayed
ground grid and XYZ axes are in the estimator global frame; they do not add an
absolute reference or change estimator state. The default viewer camera draws
positive global Z upward on screen and the X/Y plane receding downward, but
this is only a projection convention. Serialized state remains native
OpenVINS: its configured gravity vector is `(0, 0, +gravity_mag)`, and OVRS
does not silently convert it to ENU, NED, FLU, or FRD.
Both viewer windows label the current visual-support state and feature count;
the trajectory window also shows accepted/candidate MSCKF counts and ratio.
Weak support is red. The complete status and hysteresis durations remain
recorded even when the viewer is disabled. The trajectory origin is the first
pose whose visual-support state is `HEALTHY`, not an untrusted warm-up
estimate. Later movement away from that axis is drawn unchanged, including
drift or degraded states, and its path/displacement labels explicitly remain
estimates.

`RunWriter` serializes state, diagnostics, metadata log, close, and finalize
operations. This is required because live state output originates on the
dispatcher worker while periodic diagnostics originate on the main thread.

## Dependency boundaries

`ovrs_core` has no OpenCV, RealSense, Eigen, Ceres, or OpenVINS dependency.
`ovrs_realsense` owns RealSense types. `ovrs_openvins` owns all upstream types.
`ovrs_viewer` owns OpenCV HighGUI rendering. Application and log interfaces
use project-owned structs.

Within `ovrs_core`, `app_support` contains only process lifecycle, file, CLI,
and version helpers. Calibration identity, dependency-path checks, camera
geometry, sensor-policy provenance, and estimator-bundle validation live in
`calibration_validation`. Keeping these concerns separate prevents ordinary
CLI code from becoming the owner of calibration semantics.

OpenVINS is built from its `ov_msckf` directory with `ENABLE_ROS=OFF` and
`ENABLE_ARUCO_TAGS=OFF`; `catkin` and `ament_cmake` package discovery are
disabled as well. Ubuntu 24.04 uses repository-local Ceres 2.1.0 because
OpenVINS v2.7 still uses `LocalParameterization`. The tracked OpenVINS
submodule stays clean; its reviewed patch is applied to an ignored clone under
`.deps/src/open_vins`, and the build cache is checked against that source.

librealsense v2.57.3 is pinned because its release line explicitly includes
Ubuntu 24.04. The supported build always uses its repository-local RSUSB
checkout with `patches/librealsense-rsusb-gyro-sensitivity.patch`; a system
library is not accepted by version alone. The patch corrects host-side
feature-report encoding and rejects invalid libusb contexts before device
enumeration. It does not patch kernel modules, firmware, or EEPROM.
Dependency commits and reviewed patch hashes are pinned together in
`cmake/DependencyVersions.cmake`.

## Frames and output

OpenVINS state stores:

- `p_IinG`: IMU origin expressed in the estimator global frame, metres;
- `v_IinG`: velocity expressed in the global frame, m/s;
- `q_GtoI`: JPL quaternion mapping global coordinates into IMU coordinates,
  serialized `qx qy qz qw`;
- gyro/accelerometer biases in rad/s and m/s^2.

The trajectory file uses TUM's eight-column syntax but preserves this native
JPL convention. It does not perform a Hamilton inversion or NED/FRD conversion.
Covariance is the real 15-state OpenVINS marginal in order orientation,
position, velocity, gyro bias, accelerometer bias.

## Offline markerless-SLAM boundary

The v0.6.0 research branch adds a backend-neutral path beside, not inside, the
live estimator:

```text
complete OVRS VIO dataset
          |
          v
fail-closed EuRoC export + immutable source hashes
          |
          +--> OpenVINS baseline
          +--> ORB-SLAM3 adapter       (offline baseline and isolated
                                        experimental live path implemented)
          `--> OKVIS2 adapter          (planned)
```

`scripts/export_vislam_benchmark.py` does not import a SLAM library and does
not alter OpenVINS state. It validates capture integrity, converts normalized
decimal seconds to integer nanoseconds, gives a stereo pair one midpoint
timestamp, and exports synchronized gyro-frame IMU measurements. Backend
configuration and camera/IMU transform conversion remain explicit adapter
responsibilities. `scripts/prepare_orbslam3_benchmark.py` implements the first
such boundary without linking ORB-SLAM3 into the project. It shifts camera
labels into the IMU clock with the selected fixed offset, leaves IMU timestamps
unchanged, derives both transform directions explicitly, and hashes the
generated settings and indexes.

`scripts/run_orbslam3_benchmark.py` captures the backend log and exit status.
`scripts/evaluate_orbslam3_run.py` then validates trajectory structure,
terminal input coverage, inertial BA completion, resets, tracking failures,
atlas-manifest compatibility, exact ELF backend-library resolution, and
pinned-backend loop/merge messages before writing a hashed result manifest.
Accuracy remains unevaluated unless a separate start/end reference is supplied
and explicitly marked as unavailable to the estimator. See
[the ORB-SLAM3 offline baseline](orbslam3_offline.md).
The isolated [live ORB-SLAM3 adapter](orbslam3_live.md) reuses project capture
and ordering but does not consume or correct the OpenVINS state. It therefore
remains pure ORB-SLAM3 rather than the future `T_map_odom` hybrid. Its
canonical trajectory is published only after inertial BA2 completes and the
combined inertial-ready state remains stable with zero active-map resets.
Pre-BA2 visual poses remain in a separate diagnostic file. Any reset,
inertial/BA2 regression, or loop/global-BA map change after acceptance rejects
the candidate instead of joining discontinuous map-local pose segments.

A future globally corrected pose must preserve two frames:

```math
T_{\mathrm{map}\leftarrow\mathrm{body}}
=
T_{\mathrm{map}\leftarrow\mathrm{odom}}
T_{\mathrm{odom}\leftarrow\mathrm{body}}.
```

The local `odom` pose must remain continuous. Relocalization and loop closure
may update the `map` to `odom` transform only after place recognition and
geometric verification. They must not silently overwrite the OpenVINS filter
state. The complete research gates are in
[the markerless VSLAM plan](vislam_research_plan.md).
