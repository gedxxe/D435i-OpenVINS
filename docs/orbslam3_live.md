# Experimental ORB-SLAM3 live path

`ovrs_orbslam3_live` is the first desktop live integration on the v0.6
research branch. It is **pure ORB-SLAM3 stereo-inertial SLAM** behind the
project-owned D435i capture, timestamp, bounded-queue, calibration, and
provenance boundaries.

It is not a fused OpenVINS/ORB estimator:

- ORB-SLAM3 owns visual tracking, IMU estimation, local mapping, loop closure,
  relocalization, atlas state, and the Pangolin viewer;
- the OpenVINS pose is not consumed;
- ORB global corrections are not fed into OpenVINS;
- `ovrs_live` remains the reviewed OpenVINS v0.5.2 odometry baseline.

This separation makes identical-camera A/B runs possible without silently
changing the baseline. A future hybrid must explicitly implement and validate
`T_map_odom`; this executable must not be described as that hybrid.

## Build

The external GPL ORB-SLAM3 source and its Pangolin/GLEW dependencies remain in
the ignored `.deps` tree at the revisions pinned by
`config/research/orbslam3_backend.yaml`. After preparing that reviewed backend:

```bash
cmake --preset linux-release -DOVRS_ENABLE_ORBSLAM3=ON
cmake --build build/linux-release --target ovrs_orbslam3_live --parallel
```

CMake fails closed if the pinned checkout headers or required shared
libraries are absent. The portable build still produces a stub CLI without
linking GPL or hardware dependencies.

## Generate a serial-bound settings bundle

Do not hand-copy camera matrices or timing offsets into an ORB YAML:

```bash
D435I_SERIAL="843212070146"
SELECTED_DIR="config/local/d435i-${D435I_SERIAL}/selected_runtime"
LIVE_BUNDLE="runs/orbslam3_live_config_$(date -u +%Y%m%dT%H%M%SZ)"

python3 scripts/prepare_orbslam3_live.py \
  --estimator-config "${SELECTED_DIR}/estimator.yaml" \
  --stream-config config/sensors/realsense_streams_vio_90hz.yaml \
  --output "${LIVE_BUNDLE}" \
  --camera-stride 3
```

The default keeps every third stereo pair (30 Hz from the selected 90 Hz
capture) while retaining every synchronized 200 Hz IMU sample. This matches
the validated offline adapter rate. The generated settings bind the serial,
calibration state and hashes, calibrated camera-to-IMU offset, backend commit,
reviewed patch hash, source rate, stride, and pose frame.

## Run the ORB viewer

The supported launcher contains strict shell handling inside a child script.
It does not alter the caller's interactive-shell options, and it retains and
independently evaluates a failed run instead of hiding its nonzero status.
Do not paste `set -e` into an interactive terminal before a hardware attempt:
an expected fail-closed exit would then close that shell on some terminal
configurations.

The current selected calibration is still labelled `BOOTSTRAP_UNVERIFIED`, so
its diagnostic acknowledgement remains explicit:

```bash
./scripts/run_orbslam3_live.sh \
  --serial 843212070146 \
  --allow-unverified-calibration
```

The lower-level equivalent remains available for debugging:

```bash
ORB_RUN="runs/orbslam3_live_$(date -u +%Y%m%dT%H%M%SZ)"

./build/linux-release/ovrs_orbslam3_live \
  --settings "${LIVE_BUNDLE}/orbslam3_live_settings.yaml" \
  --live-bundle-manifest "${LIVE_BUNDLE}/live_manifest.yaml" \
  --vocabulary .deps/src/orb_slam3/Vocabulary/ORBvoc.txt \
  --config "${SELECTED_DIR}/estimator.yaml" \
  --stream-config config/sensors/realsense_streams_vio_90hz.yaml \
  --serial "${D435I_SERIAL}" \
  --viewer \
  --allow-unverified-calibration \
  --output "${ORB_RUN}"
```

Keep the camera still when the `ORB startup IMU gate` cue appears. The live
adapter requires one continuous one-second low-motion window before any pose
can become canonical. Its acceleration standard-deviation and gyro bounds are
pinned in `config/research/orbslam3_backend.yaml`; its broad gravity-error
limit is the selected estimator's existing `max_accel_bias_m_s2` safety bound,
not a fitted accelerometer scale. A stable gravity mismatch fails immediately.
Power-cycle or SDK-reset the D435i and repeat the gate; never divide IMU data by
an observed one-pose ratio.

One initial upstream `not IMU meas` line can occur before the first complete
frame-to-frame IMU preintegration exists. Repeated `not enough acceleration`
messages now include the exact norm used by ORB-SLAM3 and its configured
threshold. They mean the camera has not undergone enough varied translational
acceleration for the stereo-inertial initializer; they are not a viewer crash.
The threshold remains pinned at its upstream value of `0.5 m/s^2` and
`IMU.fastInit` remains disabled. Keep the camera stationary only for
transport/viewer integrity checks. A successful SLAM run requires deliberate
translation and rotation with textured stereo content.

Start stationary. Move only after `ORB startup IMU gate PASS`, then use a
deliberate, textured, well-lit motion sequence; inertial initialization cannot
be established by a permanently stationary camera. Do not promote a live
attempt solely because a stereo-only trajectory is continuous. Before another
live viewer run, require the exact recording and calibrated 30 Hz adapter to
complete both inertial BA stages with zero IMU-map reset and zero local-map
tracking failure. Favor rigid mid/far room structure and
translation-dominant initialization motion; close dynamic foreground,
rotation-heavy motion, and motion blur can leave stereo tracking intact while
the stereo-inertial initializer repeatedly clears its map.

Do not move briefly and then pause to wait for initialization. In this pinned
upstream logic, the local-mapping initialization clock advances only on
keyframes whose recent two-step camera-centre translation exceeds 0.05 m. A
recent two-step translation below 0.02 m can reset the active map while that
qualified clock is still below 10 seconds, while BA2 is scheduled only after
the clock exceeds 15 seconds. After the startup gate passes, continue smooth
translation with parallax and modest rotation for the entire initialization;
stop or hold only after `canonical trajectory gate OPEN` is printed.

Press Ctrl+C to stop. The callback only copies and enqueues frames. ORB
tracking runs on the ordered dispatcher thread, and
its own mapping, loop-closing, and viewer workers shut down through
`ORB_SLAM3::System::Shutdown()`.
If the bounded initialization-reset limit is exceeded, or canonical
continuity fails after the gate opens, the adapter now requests the same clean
shutdown automatically. A nonzero exit still denotes a rejected run; when
using the supported launcher it does not close the parent terminal.
The hardware/libusb context is resolved before those workers are created. The
reviewed dependency patches turn an unavailable USB context into a clean
startup error and serialize repeated or concurrent viewer/application
shutdown. An external owner waits for and joins the viewer thread, avoiding
Pangolin/Qt objects being left live during process teardown. A simultaneous
viewer Stop action unwinds instead of waiting on the external owner and
forming a circular wait. On the current Ubuntu Qt HighGUI backend, a clean
externally requested shutdown can still print timer-affinity warnings after
both viewer windows close. The tested path exited without a crash, but the
warnings remain a viewer-backend limitation.

## Fail-closed trajectory contract

`Tracking::OK` alone is not an accepted visual-inertial trajectory. ORB-SLAM3
can report visually tracked poses before inertial initialization and can clear
its active map during a failed initialization. Active-map clear retains the
upstream map ID, so the reviewed backend exposes a monotonic reset counter
instead of inferring continuity from map ID.

The live adapter applies all of these gates:

- the initial stationary IMU window must pass its pinned motion bounds and
  remain within the selected estimator's configured gravity/bias envelope;
- ORB-SLAM3 must complete its second inertial bundle-adjustment stage
  (`VIBA 2`/inertial BA2); the initial `isImuInitialized()` transition is not
  sufficient because BA2 can still rotate or rescale the active map;
- the combined IMU-initialized, BA2-complete, and valid-pose tracking state
  must remain established for the configured stability window (currently
  3 seconds);
- a pre-acceptance tracking interruption or frame interval above the pinned
  limit restarts that window. The limit is generated as
  `live_maximum_tracking_interval_factor / Camera.fps` (currently
  `3 / 30 = 0.1` seconds), not embedded as an executable-only constant;
- at most the backend-pinned number of active-map resets may occur before any
  canonical pose exists (currently five); each retry clears the stability
  timer, and the gate cannot open until the fresh map completes BA2 and remains
  continuously tracked for the full stability window;
- a reset request or applied active-map reset after candidate acceptance is
  terminal, and no subsequent pose can re-open the canonical stream;
- a loop-closure/global-BA map change after candidate acceptance marks the
  streaming candidate discontinuous; the corrected map and continuous local
  odometry are different frame contracts;
- a reset request still pending at final shutdown fails. A pre-acceptance
  request may recover only if it is applied within the pinned retry bound and
  later clears before a fresh BA2-stable window;
- an initialized-to-uninitialized or BA2-complete-to-incomplete regression is
  a failure;
- a tracking loss, over-limit frame interval, reset, inertial regression, or
  map correction after acceptance permanently marks the candidate
  discontinuous;
- capture, timestamp, synchronization, queue, and image-integrity counters
  must remain clean.

`live_visual_tracking_trajectory_tum.txt` contains all visually tracked
`Tracking::OK` poses for diagnosis and may cross initialization attempts. It
must never be consumed as stable odometry. Stable poses are first written to
`live_camera_trajectory_candidate_tum.txt`. Only a run that passes every gate
atomically renames that file to `live_camera_trajectory_tum.txt`; a failed run
has no canonical accepted trajectory file and retains `INCOMPLETE`.

`live_tracking_states.csv` records inertial and BA2 state, reset count, active
map change index, combined tracking/inertial stability duration, and per-frame
startup-IMU state plus candidate acceptance. `run_summary.yaml` separately records total lost frames,
post-acceptance loss and frame-gap counts, the configured interval limit, and
the maximum observed interval. It also records tracking-latency mean/maximum,
the nominal camera-frame budget, and the number/ratio of deadline misses. These
are host diagnostics; Raspberry Pi 5 real-time claims still require the
research-plan resource benchmark.
`live_imu_excitation.csv` records the synchronized IMU batch plus the exact ORB
initialization delta/threshold diagnostic. `run_summary.yaml` is the terminal
accept/reject record. An atlas name supplied during bundle generation is saved
inside the run directory.

## Independent live evaluation and return-to-start protocol

The application prints `canonical trajectory gate OPEN` exactly once, when
BA2 and valid-pose tracking have remained continuous for the configured
window after any bounded initialization retries, with no post-acceptance
reset. This is the earliest permitted start of a closed-loop return-to-start
test:

1. begin at rest, then perform the textured translation and rotation needed
   for inertial initialization;
2. place and hold the camera at the physical pose that will be treated as
   home;
3. wait for the gate-open message, then continue holding for at least the
   predeclared `endpoint_window_seconds` before leaving that pose;
4. move through the intended loop without stopping the process, return the
   camera against the same rigid stop, hold it for at least the same endpoint
   window, then press Ctrl+C.

The gate-open pose, not application startup or the first visual-only pose, is
the start of the canonical trajectory. Do not use a hand-estimated return as
an independent placement reference. Before starting motion, record the
rigid-stop method, measured placement tolerances, endpoint-window duration,
minimum sample count, and permitted within-hold dispersion:

```yaml
%YAML:1.0
format: "ovrs-closed-loop-reference-v2"
reference_type: "COLOCATED_START_END"
estimator_consumed_reference: false
physical_path_completed: true
method: "camera returned against the same two rigid contact surfaces"
position_tolerance_m: 0.01
orientation_tolerance_deg: 2
endpoint_window_seconds: 2
minimum_endpoint_samples: 30
maximum_endpoint_position_spread_m: 0.005
maximum_endpoint_orientation_spread_deg: 1
minimum_path_duration_seconds: 10
minimum_path_excursion_m: 0.5
```

These numbers are examples, not defaults. Measure the fixture tolerance and
select hold/dispersion limits plus a conservative lower bound on the planned
physical path duration and farthest excursion before examining the
trajectory. At 30 Hz a two-second window normally contains many more than 30
poses, but tracking quality—not nominal rate—determines the actual count.

After shutdown, recompute the result from the raw CSV and trajectory files:

```bash
python3 scripts/evaluate_orbslam3_live_run.py \
  --run-dir "${ORB_RUN}" \
  --live-bundle-manifest "${LIVE_BUNDLE}/live_manifest.yaml" \
  --backend-library .deps/src/orb_slam3/lib/libORB_SLAM3.so \
  --live-executable build/linux-release/ovrs_orbslam3_live \
  --vocabulary .deps/src/orb_slam3/Vocabulary/ORBvoc.txt \
  --closed-loop-reference path/to/closed_loop_reference.yaml
```

Before ORB-SLAM3 constructs any worker, the live application writes
`launch_provenance.yaml` and a byte-for-byte copy of the bundle manifest. The
launch record binds the running executable, loaded-backend path, vocabulary,
settings, bundle, and compiled source fingerprint by SHA-256. It also rejects
a backend library that changed after the executable was built.

The evaluator validates those capture-time hashes, pinned patch and shared
library, bundle/settings hash, connected-device serial, exact CSV schemas and
row agreement, monotonic timestamps, the configured and observed frame
intervals, the continuous tracking/BA2 stability timer, quaternion norms,
accepted-pose timestamps, run summary, reset and BA2 history, transport
counters, and canonical-file publication. A malformed or inconsistent run
produces no evaluation manifest. A coherent failed run produces
`LIVE_GATE_FAILED` with explicit reasons. Its visual and candidate trajectory
files may be empty only when their independently recomputed and terminal pose
counts are also zero; this preserves valid pre-initialization failures without
accepting a missing trajectory from a claimed pass.

For a v2 closed-loop reference, the evaluator requires nonoverlapping
start/end windows with the declared sample count and time coverage. It uses
the component-wise median position and a sign-aligned normalized quaternion
mean for each window, verifies maximum position/orientation dispersion, and
then compares the two robust endpoint poses. It also requires the canonical
trajectory to meet the predeclared minimum path duration and estimated
excursion, preventing a never-left-home trajectory from being labelled a
closed loop. Raw first-to-last residuals remain diagnostic only. These are
sanity gates, not substitutes for trajectory ground truth.

The manifest reports continuity and physical-reference failures separately.
`LIVE_GATE_PASS_CLOSED_LOOP_REFERENCE_FAILED` means the ORB trajectory passed
its reset/tracking/timing contract but the endpoint hold or robust return
check failed; it must not be described as a return-to-home pass.

No motion-discontinuity threshold is silently hardcoded. An operational
envelope may be supplied explicitly with
`--maximum-adjacent-translation-m`; its origin and applicability must be
justified by the experiment. Without it the manifest says
`NOT_EVALUATED_NO_OPERATIONAL_ENVELOPE`.

Passing the continuity gate without a physical start/end reference yields
`LIVE_GATE_PASS_CONTINUITY_NOT_ACCURACY_VALIDATED`. Passing the recorded
rigid-stop tolerances yields a stronger return-consistency state, but still
does not establish full-trajectory accuracy without independent ground truth.
New bundle-v4/runtime-provenance-v5 runs report `CAPTURE_TIME_ATTESTED`.
Evaluator-v6 additionally recomputes the bounded pre-acceptance and strict
zero-post-acceptance reset counts from every tracking row.
Earlier run schemas remain historical evidence only with the evaluator source
revision and hashes recorded at that time; the current evaluator does not
retroactively upgrade them. A later binary must never be presented as their
capture-time executable.

The live process also fails closed when either the raw stereo callback or raw
IMU callback is silent for longer than the pinned
`live_maximum_input_stall_seconds` interval. The summary separates active
capture duration from ORB/viewer shutdown duration, so source and submission
rates do not silently include a blocked shutdown.

This contract prevents an active-map reset or a known global map correction
from being hidden as continuous local odometry. The live pose remains an
ORB-SLAM3 map-frame diagnostic, not an `odom`-frame interface. It still does
not authorize sending ORB poses or corrections into OpenVINS or a flight EKF.

## Claim boundary

A clean connected run proves that the pinned ORB backend can consume the live
D435i stream without transport errors. It does not prove trajectory accuracy,
false-loop performance, reliable relocalization, hybrid fusion, or Raspberry
Pi 5 real-time operation. Those claims still require the research-plan gates,
external ground truth, distinct-revisit evaluation, and measured target
hardware resources.
