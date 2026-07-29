# Version history

The repository did not previously contain release tags or a changelog. The
entries below document the actual development milestones represented by the
current tree; they must not be read as claims that historical Git tags exist.

## Unreleased

- Made reviewed `selected_runtime` YAML bundles publishable while keeping raw
  calibration captures, factory exports, and candidate bundles ignored.
  Repository checks now reject an ignored selected runtime.
- Rewrote the root README in plain English and added project/stack badges, the
  GPLv3 and author details, and a clickable live-viewer guide. The complete
  calibration procedure moved to `docs/operator_runbook.md`, historical audit
  detail moved to `docs/audit_history.md`, and mathematical notation moved to
  a focused GitHub-renderable document using `$...$` inline delimiters.
- Moved the 1,099-line inline implementation out of
  `include/ovrs/app_support.hpp` into `src/app_support.cpp`. CLI and test users
  now include YAML parsing declarations explicitly instead of relying on a
  transitive implementation header.
- Added output-only MSCKF batch-quality provenance: candidate count, accepted
  count after triangulation/refinement/chi-square rejection, acceptance ratio,
  and age of the last non-empty batch. Two identical-data replays remained
  byte-identical to their earlier trajectories. The metric is intentionally
  not a new pass/fail gate because accepted-ratio distributions overlapped and
  no defensible universal threshold was established.
- Centralized selected-runtime hash verification in
  `scripts/verify_selected_runtime.sh`, removed personal checkout paths from
  documentation, ignored local EEPROM-tool outputs without deleting them, and
  added repository checks for personal paths and Bash-block syntax.
- Added a project-owned visual tracking-health gate with time hysteresis and a
  bounded warm-up. Its support count uses current-frame camera-0 frontend
  tracks, not persistent unobserved SLAM landmarks. A minimal read-only
  `VioManager` accessor exposes that count without changing the tracker or
  filter. Live and replay record support, status, transition evidence, and
  resolved thresholds; both viewer windows show weak support in red. The gate
  is output-only and never rewrites OpenVINS pose, covariance, features, or
  ZUPT state.
- Replaced the selected serial's former `gyro_scale_factor: 0.5` with `1.0`
  after official EEPROM IMU recalibration. On one identical 50.44-second raw
  capture, `0.5` reached the 3 m/s safety gate while `1.0` completed with
  0.0588 m final displacement and 0.0065 m/s final speed. A connected
  131-second `1.0` live run then completed without runaway, but retained about
  0.62 m endpoint error beyond the operator-reported physical translation, so
  the runtime remains diagnostic rather than drift-safe.
- Invalidated the earlier selected-runtime acceptance after clean-transport
  pitch-motion runs reproduced metres of false translation. The historical
  57.83 s replay ended slowly only after accumulating 60.893 m path and 10.677
  m final displacement; stop recovery is now documented as a final-velocity
  constraint, not a position-drift fix. Debug replay also captured the
  collapse of usable MSCKF update features after motion.
- Added explicit, fail-closed D435i gyro-sensitivity and project gyro-scale
  acquisition contracts. Motion start sets and reads back SDK/FW sensitivity;
  the selected serial now applies `gyro_scale_factor: 1.0` before recording
  and synchronization. Stream, device, dataset, and calibration-export
  provenance must agree. Replay never silently rescales historical data; the
  earlier `0.5` evidence remains preserved as rejected history.
- Preserved and diagnosed clean-transport moving captures at sensitivity
  levels 1 and 0. Twelve strong level-0 windows had median gyro/visual ratio
  2.017 and mean 1.993. A native corrected hardware capture recorded 5388
  stereo pairs and 11982 synchronized IMU rows with zero integrity errors;
  representative corrected visual/gyro rotations agreed at 13.667/13.931 and
  11.869/11.683 degrees.
- Fixed the reviewed continuous-ZUPT duration gate. Normal MSCKF cleanup made
  its former interval-wide feature lookup return zero tracks, so post-motion
  recovery was unreachable. The selected policy now requires one second of
  consecutive low-disparity frames with more than 20 common features. It can
  constrain final velocity but does not make the trajectory drift-safe.
- Added a dedicated 848x480 Y8 stereo 90 Hz VIO stream profile while retaining
  the 30 Hz calibration profile. A connected D435i capture completed with zero
  transport integrity errors; replay completed 87.83 seconds without the
  earlier recorded fast-motion runaway and remained within 4.2 mm over its
  final 17.8-second stationary segment. A subsequent connected live 90 Hz
  motion still reached the 3 m/s gate with clean transport, so fast-motion
  acceptance remains failed. These runs have no external trajectory ground
  truth and are not an accuracy certification.
- Replay now validates and reports a leading stereo prefix that predates the
  first synchronized IMU sample, then begins at the first covered stereo pair.
  It does not shift timestamps, synthesize IMU, or tolerate a later missing
  bracket.
- Replaced the fixed isometric trajectory panel with an interactive global XYZ
  viewer: left-drag orbit, middle/right-drag pan, cursor-centred wheel zoom,
  explicit fit/reset controls, a world-locked ground grid, compact labelled
  axes, start/current markers, resizable aspect ratio, and live
  path/displacement metrics. New states no longer silently re-centre or
  rescale the world. View-controller math is dependency-light and covered by
  core regression tests.
- Added an explicit selected-runtime reproduction protocol and controlled
  diagnostic live-viewer guidance. It binds each run to the exact hashes,
  serial, fixed time-offset/ZUPT policy, controlled
  stationary/smooth-motion sequence, replay-first physical bounds, and the
  known fast-motion limit.
- Raised the exact librealsense pin from v2.56.5 to v2.57.3 after a physical
  D435i with a newly written official IMU calibration table opened four
  streams but delivered no frames through v2.56.5. The same device delivered
  frames through v2.57.3; the pin remains tag-and-commit verified.
- Selected the serial-specific post-update Kalibr candidate A as the current
  local runtime for D435i `843212070146`. Runtime keeps its fixed
  -4.900203074 ms offset, disables online time-offset estimation, and uses the
  reviewed visually gated stop recovery. The bundle remains
  `BOOTSTRAP_UNVERIFIED`; selection is not strict calibration promotion.
- Added a single ready-to-run README path and a selected-runtime evidence
  contract. Factory bootstrap, candidate B, online-offset results, and
  the earlier broken continuous-ZUPT trials are retained as diagnostic
  evidence, not interchangeable operator configurations. The fixed,
  visually-gated one-second stop recovery is the selected policy.
- Recorded the connected-camera limit honestly: stationary and smooth-motion
  behaviour improved materially with zero transport drops, while sufficiently
  fast handheld motion can still produce a physically inconsistent
  trajectory. Numeric health and clean transport are not presented as
  accuracy proof.

## v0.5.2 - 2026-07-27

- Added a fail-closed three-export coherence validator. It rejects mixed
  serials, AprilGrid hashes, IR profiles, IMU rates, capture modes, or motion
  policy before ROS bag creation. It also rebinds manifest fields to hashed
  source metadata, checks staged camera indexes/image dimensions and IMU row
  counts, and writes a SHA-256-bound unverified report.
- Replaced mutable calibration-image `docker commit` assembly with a
  repository-owned Dockerfile and pinned-source build context.
- Added an ordered Docker CE installation branch based on Docker's official
  signed Ubuntu repository, including conflict inspection, non-root access
  verification, and the root-equivalent `docker` group warning.
- Made Step 8 re-establish its camera, target, and capture-ID state after a
  terminal restart. Export and external-tool blocks now fail closed without
  terminating the user's interactive terminal.
- Isolated stereo and camera-IMU Kalibr outputs in new empty work directories,
  verified exactly one camchain/report/result set, removed unexplained stereo
  frame downsampling, and used the pinned commit's exact `--cams` interface.
- Clarified that Kalibr's generated target PDF is content-sized: the printer
  must centre it on A4 at actual size and preserve the measured physical white
  border.
- Corrected every pinned Docker Kalibr invocation to use the catkin package
  interface `rosrun kalibr <tool>`. The image build now smoke-tests the target
  generator, bag creator, stereo calibrator, and camera-IMU calibrator before
  accepting the image.

## v0.5.1 - 2026-07-27

- Added main-thread OpenCV preview to stereo and camera-IMU recording. The
  pre-capture source is stopped and restarted after Space so preview frames,
  drops, and timestamps cannot contaminate the saved capture.
- Preview now shows owned IR1/IR2 frames during recording and fails closed on
  `q`, Escape, window close, Ctrl+C, disconnect, malformed data, or existing
  recorder integrity gates. It is explicitly visibility/blur guidance, not an
  AprilTag detector.
- Added an A4 AprilGrid planner that checks printer margins, active grid
  dimensions, and Kalibr's minimum white border before printing.
- Documented official pinned Kalibr PDF generation, actual-size printing,
  rigid mounting, multi-position physical measurement, and measured-mm YAML
  creation.
- Made stationary Allan analysis resumable from capture metadata instead of
  stale terminal variables. Missing dataset/config paths now produce concise
  argument errors without Python tracebacks.
- Added read-only calibration-capture validation and changed validation/export
  to stream large CSV files with bounded memory. Nested RealSense diagnostic
  YAML keys no longer collide with top-level metadata, and recorder summary
  counts must match the actual CSV row counts.
- Removed the requirement to guess a gravity-error threshold. The stationary
  check is diagnostic by default; a numeric gate is accepted only when backed
  by an independently declared physical requirement.
- Pinned external Kalibr and `allan_variance_ros` commits and documented an
  isolated Ubuntu 20.04/ROS Noetic Docker route without adding ROS to the
  Ubuntu 24.04 runtime.
- Reworked README Steps 7-10 and the manual checklist to define the board,
  preview limits, detection gate, resume paths, Docker decision branches, and
  stop conditions explicitly.

## v0.5.0 - 2026-07-26

- Canonicalized the camera-transform contract. Pinned OpenVINS v2.7 requests
  `T_imu_cam`; its upstream parser can accept the opposite Kalibr
  `T_cam_imu` key and invert it, but runtime files now use one explicit
  direction to prevent ambiguous review or manual editing.
- `ovrs_inspect` now exports camera-to-IMU factory transforms directly;
  runtime validation requires two `T_imu_cam` matrices and rejects
  `T_cam_imu`.
- Inspector/preflight sampling now refuses export on Ctrl+C, disconnect, an
  empty required stream, or detected camera-frame loss.
- Added a non-overwriting, backup-preserving migration for legacy
  `BOOTSTRAP_UNVERIFIED` local bundles.
- Added explicit `vio`, `imu-allan`, `stereo-calibration`, and
  `imu-camera-calibration` recording modes with mode-specific RealSense stream
  selection and fail-closed finalization.
- RealSense selection now accepts only a SDK-reported D435i model; an omitted
  serial can no longer select a different RealSense product by enumeration
  order.
- Added portable calibration capture validation/export using the official
  Kalibr bag-creator image/IMU staging contract while retaining timestamp
  provenance. Export v2 copies source metadata and binds every fixed
  provenance path with SHA-256; downstream tools re-hash it.
- Added measured AprilGrid target generation with no board-size defaults.
- Added Allan/noise YAML preparation that binds matching serial, rate, and
  active RealSense motion-correction policy, and requires the Allan output
  itself to report `/imu0` at the captured gyro rate; generated OpenVINS `Tg`
  is zero.
- Added a mandatory operator-bounded stationary gravity check before Allan
  export. It detects gross scale/model disagreement without deriving an IMU
  scale matrix or random-walk value from insufficient data.
- Added Kalibr structural validation, PDF/manual-review gates, source hashing,
  explicit shared time-offset selection, and serial-specific verified-bundle
  promotion.
- Promotion validates and inverts Kalibr `T_cam_imu` to OpenVINS
  `T_imu_cam`; it never renames a transform without inversion.
- Rewrote README and manual acceptance documentation as ordered stop/go
  procedures, including the no-Docker external ROS1/Kalibr boundary.
- Extended preflight and CTest registration for optional Python calibration
  scripts; the C++ runtime remains independent of Python.
- Live runs now retain `INCOMPLETE` and fail when capture drops, dispatcher
  queue drops/nonmonotonic input, or IMU synchronization integrity counters
  are nonzero. A final unbracketed stereo tail caused solely by orderly
  shutdown remains diagnostic rather than a false failure.

## v0.4.0 - 2026-07-26

- Added an opt-in native OpenCV viewer for live and replay operation. The main
  thread owns all HighGUI calls; capture callbacks retain bounded,
  project-owned Y8 buffers and never touch the GUI.
- Added IR1/IR2 display, initialized pose text, bounded X-Y trajectory history,
  auto-scaling, total 3D path/displacement metrics, and clean `q`/Escape
  shutdown. Headless operation remains the default.
- Viewer startup now cleans up a partially created HighGUI window, and
  nonfinite derived extents/path metrics fail the run instead of reaching
  pixel conversion or being displayed as valid pose output.
- Fixed a data race in live output: estimator states and periodic diagnostics
  previously wrote different `RunWriter` streams concurrently without
  synchronization. All run output operations are now serialized and covered
  by a concurrent-write test.
- Made librealsense motion correction an explicit stream policy. The selected
  D435i must accept the requested option; requested/actual states and factory
  motion intrinsics are preserved in the device report.
- Bound estimator IMU calibration to that captured motion-correction policy.
  Replay refuses legacy or ambiguous datasets rather than silently mixing raw
  and corrected IMU semantics.
- Bound the main, IMU, and camera YAML files to one calibration state and
  serial. IMU noise density/random-walk fields must be positive and finite,
  preventing a `KALIBR_VERIFIED` camera file from silently retaining
  bootstrap or another unit's IMU parameters.
- Made `KALIBR_VERIFIED` the default estimation gate. Factory
  `BOOTSTRAP_UNVERIFIED` bundles require an explicit diagnostic override and
  are never presented as accuracy-qualified configurations.
- Added a dependency-free stationary-IMU analyzer. It reports gravity-norm
  mismatch and short-term per-axis white-noise estimates without fabricating
  bias random walk; random walk still requires a long Allan-deviation record.
- Added operator-supplied trajectory bounds to the plotting tool. This turns a
  known rotation-only/return-to-start test into an executable pass/fail check
  without embedding an assumed motion bound in the repository.
- Made interactive trajectory plotting select a real GUI backend and fail with
  a clear instruction when no desktop/backend exists. `--save` now creates a
  PNG deterministically and refuses accidental overwrite. Three-dimensional
  plots now keep equal metre scales on all axes.

## v0.3.3 - 2026-07-26

- Replaced the duplicated, non-copy-safe hardware checklist with a complete
  serial-bound build, inspect, review, record, replay, and live procedure.
- Added preflight `--require-build`; it fails unless all applications, exact
  source fingerprints, dependency caches, and both registered tests exist.
  `--require-camera` now implies that gate and requires the numeric serial of
  the unit being reviewed.
- Made factory preparation require both the expected serial and the exact
  stream-config file, preventing a valid export from the wrong unit or
  resolution from creating a local estimator bundle.
- Made inspector export, preparation, and runtime validation reject RealSense
  distortion models that cannot be represented safely as OpenVINS radtan,
  including a nonzero fifth Brown-Conrady coefficient.
- Inspector sampling now returns failure and suppresses export whenever it
  observes a malformed frame, rejected timestamp, or callback error; a printed
  timestamp failure can no longer exit successfully.
- Recorder completion now requires zero malformed frames, rejected timestamps,
  and callback errors and writes those counters into its summary; otherwise
  the dataset remains `INCOMPLETE`.
- Live finalization applies the same capture-integrity gate; nonzero malformed,
  timestamp-rejection, or callback counters retain a partial run marker.
- Added strict numeric bootstrap checks for focal lengths, principal points,
  coefficient counts/mapping, time offsets, and the numeric D435i identity.
- Rejected duplicate calibration identity, dependency-path, and IMU-rate YAML
  keys so validation and downstream parsing cannot resolve an ambiguous file
  differently.
- Made the Windows portable build treat an empty CTest registration as an
  error, matching the supported Ubuntu build contract.
- Set every CTest preset's `noTestsAction` to `error`; the previously observed
  `No tests were found` output cannot pass through a preset either.
- Included `CMakePresets.json` in the executable source fingerprint because
  preset feature flags affect the generated target graph.
- Estimator dependency paths must resolve to regular files inside the local
  bundle directory; symlink escape is rejected in addition to absolute and
  parent-traversal paths.
- Corrected dispatcher coverage tracking so closely spaced stereo pairs can
  reuse an already-fed IMU bracket without being rejected.
- Replay now requires strictly increasing raw camera clocks and frameset
  numbers, canonical frame filenames, and decoded Y8 images at the recorded
  resolution.
- Initialized state output now rejects a negative processing latency or a
  non-unit/nonfinite quaternion instead of serializing it as a healthy pose.
- Added `INCOMPLETE` markers to replay/live run directories. Only explicit
  successful finalization removes the marker; interruption, disconnect,
  runtime failure, and write failure retain it.
- Replay/live cannot finalize successfully without OpenVINS initialization
  and at least one finite state; an initialization failure remains partial.
- Removed hardcoded resolution text from the supported operator paths. One
  stream-config file is now the shared source for inspection, preparation,
  recording, and live capture.
- Reworked the README again as one beginner-oriented linear procedure using
  only camera serial and run ID variables. Output paths are visible at each
  step, strict subshell blocks stop on the first command failure, and the
  manual-test document is now a checklist rather than a competing procedure.
- Replay now copies the dataset's resolved stream configuration into its run
  directory so replay and live evidence have the same self-contained stream
  record.

## v0.3.2 - 2026-07-25

- Added a deterministic SHA-256 fingerprint of project C++ and CMake content
  to every executable and the generated CMake cache.
- Made the Ubuntu build and preflight compare each executable's embedded
  fingerprint with the configured source fingerprint. A binary that merely
  reports the right semantic version can no longer pass as current.
- This closes stale-object reuse observed on the shared NTFS checkout, where
  misleading cross-platform timestamps allowed an old v0.3.1 inspector to
  survive a nominal rebuild.
- Retained the v0.3.1 Y8-only intrinsics policy: Y16 capability profiles are
  listed without querying intrinsics, while selected Y8 calibration profiles
  remain strictly validated.
- Added a fail-closed `prepare_bootstrap_config.sh` workflow that converts an
  acknowledged factory export into an ignored serial-specific bundle without
  overwriting committed templates or existing local configurations.
- Reworked README into one ordered build, inspect, review, configure, record,
  replay, live, and plotting procedure.
- Made replay explicitly explain why `--serial` is invalid and made both
  convenience wrappers require a reviewed estimator configuration instead of
  defaulting to the intentionally invalid template.
- Added explicit inspector messages confirming report/calibration output paths.

## v0.3.1 - 2026-07-25

- Made the capture pipeline share the same librealsense context used for
  initial device selection and configuration.
- Replaced repeated `query_devices()` calls during active streaming with the
  SDK device-change callback. Disconnect detection remains fail-closed without
  repeatedly reopening USB device handles.
- Distinguished a confirmed hot-unplug from other librealsense callback
  failures and added callback-stage context to the reported error.
- Bumped the patch version so preflight rejects stale v0.3.0 Ubuntu
  executables before a hardware run.
- Made capability enumeration tolerate video profiles, such as D435i Y16,
  that do not expose intrinsics. Only Y8 profiles are queried for camera
  calibration, and the selected profile must still return finite, valid
  intrinsics before export.
- Reused the validated selected-profile intrinsics during export instead of
  querying the SDK a second time.
- Made live and replay fail closed when both camera-calibration resolutions do
  not exactly match the selected live stream or recorded dataset.
- Strengthened `preflight_ubuntu.sh --require-camera` to require a successful
  one-second project-inspector sample and timestamp check, not only device
  enumeration.
- Recorded operator evidence that the v0.3.1 recorder completed a 60-second
  physical D435i dataset at 848x480 Y8, 30 Hz with 200 Hz gyro and 250 Hz
  accelerometer. Calibration export and OpenVINS execution remain pending.

## v0.3.0 - 2026-07-25

- Added one authoritative `VERSION` file and exposed the version through every
  executable's `--version` output and run metadata.
- Corrected the Ubuntu 24.04 Ceres 2.1.0 build by disabling the unused
  SuiteSparse/CXSparse backends, while retaining the dense Schur path used by
  OpenVINS v2.7.
- Distinguished real tracked content changes from file-mode and CRLF-only
  differences in dependency checkouts shared through an NTFS mount.
- Made CMake cache validation depend on exact key values rather than the
  incidental `PATH`, `BOOL`, `STRING`, or `UNINITIALIZED` cache type.
- Added explicit stream-profile overrides to the inspector using the same
  centralized option contract as record/live; unsupported profiles still
  fail closed instead of changing calibration resolution automatically.
- Delayed recorder dataset initialization until after the requested camera
  profile starts successfully, preventing validation failures from leaving a
  new empty `INCOMPLETE` dataset.
- Reduced the local librealsense fallback to the required RSUSB library,
  disabled firmware downloads and updater/tool targets, and confined its
  unconditional upstream `ldconfig` hook to a repository-local no-op shim.
- Made the supported Linux test command select its build directory explicitly
  and fail when no tests are registered.
- Re-ran the repository-wide safety audit and synchronized this changelog,
  README, environment record, and `AUDIT_REPORT.md`.
- Centralized stream command-line overrides and resolved-stream YAML
  generation, removing duplicated parsing and serialization from live and
  record applications.
- Removed an unused calibration-state field from stream configuration;
  calibration approval remains solely in the estimator/camera YAML gate.
- Removed unused per-image fields and now records the existing maximum IMU
  interpolation-delay diagnostic instead of carrying dead state.
- Made resolved stream configuration round-trippable through the project
  loader and added strict numeric, duplicate-option, missing-value, and serial
  validation.
- Kept optional Python checks non-fatal to the independent C++ runtime.
- Made replay pacing interruptible and reports Ctrl+C as a retained partial run
  with exit code 130 instead of claiming replay completion.
- Clarified that installation exports applications only; this repository does
  not advertise an incomplete C++ SDK.

## v0.2.0 - preflight hardening milestone

- Pinned OpenVINS v2.7, repository-local Ceres 2.1.0, and librealsense2
  v2.56.5 by exact tag and commit.
- Prevented accidental system Ceres selection and disabled ROS package
  discovery in the standalone OpenVINS build.
- Added read-only Ubuntu preflight, strict/idempotent build scripts, optional
  Python 3 virtual-environment workflow, and reproducible Python requirements.
- Hardened calibration identity, timestamp/finite-value checks, owned
  RealSense frame lifetimes, bounded queues, shutdown/disconnect handling,
  dataset completeness, and replay validation.
- Added portable core tests, mock replay validation, CLI checks, and detailed
  operator documentation.

## v0.1.0 - implementation milestone

- Introduced the ROS-free C++17 target graph and the `ovrs_inspect`,
  `ovrs_record`, `ovrs_live`, and `ovrs_replay` applications.
- Added RealSense stereo/IMU capture, IMU synchronization, ordered estimator
  dispatch, the OpenVINS adapter, trajectory/log output, recording, and replay.
- Added initial sensor, estimator, logging, dataset, and build documentation.

## v0 - design baseline

- Defined the standalone D435i/OpenVINS scope and the separation between
  capture, synchronization, estimator adaptation, and output.
- Established handheld validation as the boundary: no ROS, flight controller,
  navigation, firmware writing, or flight-readiness claim.
- Contained initial bootstrap/placeholder calibration material that was not
  sufficient for safe live VIO without later validation gates.
