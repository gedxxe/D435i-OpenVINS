# Manual D435i acceptance checklist

This checklist is a stop/go sequence for one D435i serial. Do not skip a failed
gate. Record command output and artifact paths beside each checkbox. Commands
that use shell state define or guard it locally. Calibration capture
[Step 8 in the operator runbook](operator_runbook.md#8-capture-three-independent-calibration-datasets)
has its own serial/target state gate and must be rerun after a reboot or
terminal restart.

For normal operation of serial `843212070146`, use the selected-runtime branch
and `docs/selected_runtime.md`. The factory-smoke and replacement-calibration
branches are evidence/recovery paths, not alternative runtime configurations.

## Gate 1: source and build

- [ ] Root `VERSION` is `0.6.0` on the research branch.
- [ ] `git diff --check` reports no whitespace errors.
- [ ] `./scripts/build_ubuntu.sh` completed.
- [ ] `./scripts/preflight_ubuntu.sh --require-build` passed.
- [ ] CTest ran named tests; it did not print `No tests were found`.
- [ ] All four executables report v0.6.0 and one source fingerprint.
- [ ] Build cache points to repository-local Ceres 2.1.0.
- [ ] Build cache points to repository-local patched librealsense 2.57.3.
- [ ] Both reviewed patch files match the SHA-256 pins in
  `cmake/DependencyVersions.cmake`.
- [ ] `ldd` checks confirm `ovrs_inspect`, `ovrs_record`, and `ovrs_live`
  load that repository-local library rather than `/usr/local/lib`.
- [ ] OpenVINS v2.7 is pinned with ROS and ArUco integration disabled.

If any item fails: stop before connecting estimator output to a test claim.

## Gate 2: physical camera

- [ ] The serial was read from the physical unit, not guessed.
- [ ] The D435i is connected over a known-good USB 3 cable and port.
- [ ] Preflight selected the expected serial.
- [ ] One-second `ovrs_inspect` sampling completed.
- [ ] Timestamp monotonic/domain check passed.
- [ ] Malformed frames, camera-frame drops, rejected timestamps, and callback
  errors are zero.
- [ ] Motion correction was requested, available, and active.
- [ ] `gyro_sensitivity_requested` and `gyro_sensitivity_active` both equal the
  configured level `1`, and `gyro_sensitivity_available` is `true`.
- [ ] `gyro_scale_factor_configured` and `gyro_scale_factor_applied` both equal
  the selected value `1`.

If enumeration succeeds but sampling fails: treat the camera gate as failed.

## Gate 3: OpenVINS transform contract

Inspect the active camera YAML:

```bash
D435I_SERIAL="843212070146"
ACTIVE_CAMERA_YAML="config/local/d435i-${D435I_SERIAL}/selected_runtime/post_rs_imu_candidate_a_imucam.yaml"

grep -E 'T_(cam_imu|imu_cam)' \
  "${ACTIVE_CAMERA_YAML}"
```

- [ ] Exactly two `T_imu_cam` entries exist.
- [ ] No `T_cam_imu` entry exists in the OpenVINS runtime file.
- [ ] The meaning is camera coordinates to IMU coordinates.
- [ ] Both rotations are orthonormal with determinant +1.
- [ ] Both bottom rows are `[0, 0, 0, 1]`.
- [ ] Translation columns produce a nonzero, physically plausible baseline.

The selected file must report
`camera_calibration_method: "KALIBR_REPEATABILITY_CANDIDATE"` and the expected
source hashes. If a legacy factory bootstrap contains `T_cam_imu`, it is not
the active runtime; migrate it only when reconstructing the calibration
workflow. Never rename the key without inverting the matrix.

## Gate 4: choose the test branch

### Branch A: selected runtime for serial 843212070146

- [ ] The 2026-07-29 post-calibration gyro `1.0` A/B and remaining endpoint
  drift in `docs/selected_runtime.md` are understood; this branch remains
  diagnostic until controlled motion passes physical bounds.
- [ ] `BOOTSTRAP_UNVERIFIED` is understood.
- [ ] `--allow-unverified-calibration` is recorded in the test log.
- [ ] The config is
  `selected_runtime/estimator.yaml`, not the factory bootstrap,
  candidate B, or the archived pre-recalibration bundle.
- [ ] All three selected-runtime bundle SHA-256 values match
  `docs/selected_runtime.md`.
- [ ] The selected 90 Hz stream SHA-256 matches and contains
  `gyro_sensitivity: 1` and `gyro_scale_factor: 1.0`.
- [ ] `scripts/verify_selected_runtime.sh` passes its semantic sensitivity and
  scale checks; passing only a manually updated file hash is insufficient.
- [ ] The reviewed OpenVINS ZUPT patch SHA-256 matches
  `docs/selected_runtime.md`.
- [ ] Both replay and live explicitly use `--online-time-offset off`.
- [ ] `calib_cam_timeoffset: false`, `zupt_only_at_beginning: false`,
  `zupt_max_disparity: 2.0`, and `zupt_min_stationary_time: 1.0`.
- [ ] No drift-safe, accuracy, or successful-VIO claim is made from final
  velocity, `healthy=1`, clean transport counters, or replay completion alone.
- [ ] Moving acceptance uses a newly recorded dataset containing the explicit
  sensitivity and scale provenance; a legacy dataset or stationary-only
  capture is insufficient.
- [ ] When `ovrs_record --capture-mode vio --preview` is used, Space starts a
  clean recording after preview, both IR views remain live during capture, and
  preview frames are not counted as dataset frames.

Proceed to Gates 9 and 10. Gates 5-8 describe replacement calibration and do
not need to be repeated for every normal startup.

### Branch B: mechanical smoke test only

- [ ] The factory bootstrap is being used only to test mechanics.
- [ ] `--allow-unverified-calibration` is recorded in the test log.
- [ ] No accuracy, drift, or successful-VIO claim will use this output.

Proceed only to capture/replay/viewer mechanics.

### Branch C: replacement-calibration acceptance

- [ ] One official Kalibr-generated AprilGrid was printed at 100%/actual size.
- [ ] The print is mounted flat on a rigid white board with the required
  surrounding border.
- [ ] A4 page fit passed `plan_aprilgrid_target.py`; the A0 download was not
  scaled to A4.
- [ ] Tag and gap dimensions were measured at multiple places after printing.
- [ ] A measured AprilGrid target YAML exists and identifies this exact board.
- [ ] Allan, stereo, and IMU-camera calibration captures will be collected.
- [ ] A pinned isolated ROS1/Kalibr environment (Docker, VM, or separate
  machine) is available.
- [ ] If Docker was newly installed, its official signed `apt` repository was
  used, conflicting packages were reviewed rather than blindly removed, and
  both `docker info` and `docker run --rm hello-world` work without `sudo`.
- [ ] The Kalibr image gate passes `rosrun kalibr <tool> --help` for the target
  generator, bag creator, stereo calibrator, and camera-IMU calibrator.
- [ ] Docker group/rootless access was an explicit security decision; no
  calibration command is being run through `sudo docker`.

If the external environment is unavailable: stop. Do not install ROS into the
Ubuntu 24.04 runtime host merely to bypass this gate. Docker may host the
isolated Ubuntu 20.04/ROS Noetic tools; repository scripts never install it.

## Gate 5: calibration captures

### Allan

- [ ] Camera was rigidly secured and remained stationary.
- [ ] Capture used `--capture-mode imu-allan --confirm-stationary`.
- [ ] Duration met the chosen documented requirement.
- [ ] No bump, disconnect, Ctrl+C interruption, or temperature anomaly was
  observed.
- [ ] A terminal restart recovered capture serial/config from metadata; it did
  not resolve an empty bundle as `/bootstrap.yaml`.
- [ ] `validate_calibration_capture.py --capture ...` completed its read-only,
  bounded-memory full scan and reported matching summary/CSV counts.
- [ ] An operator-selected interval of at least 60 seconds was diagnosed with
  `analyze_stationary_imu.py` against the estimator gravity.
- [ ] A multi-hour capture was sampled near its beginning, midpoint, and end;
  repeatable gravity mismatch was retained as review evidence, not converted
  into a one-pose scale correction.
- [ ] A numeric gravity pass/fail tolerance was supplied only if independently
  declared by the physical test; otherwise `validation: NOT_REQUESTED` is the
  expected diagnostic result.
- [ ] A failed gravity check was not “fixed” by guessing scale from one
  orientation.
- [ ] The long Allan fit, not the short stationary diagnostic, supplied noise
  density and random-walk values.

### Stereo

- [ ] Capture used `--capture-mode stereo-calibration`.
- [ ] The measured target YAML was passed.
- [ ] `--preview` showed live IR1 and IR2 before Space started capture.
- [ ] Target covered centre, edges, corners, distances, and orientations.
- [ ] Both IR cameras saw the target without substantial blur.
- [ ] Preview was treated as visibility/blur guidance, not tag detection.

### IMU-camera

- [ ] Capture used `--capture-mode imu-camera-calibration`.
- [ ] The target remained fixed.
- [ ] `--preview` showed the board in both IR cameras before capture.
- [ ] Motion was smooth and excited all rotations plus multiple translations.
- [ ] The target remained visible and shocks were avoided.

For all three:

- [ ] `INCOMPLETE` is absent.
- [ ] `complete: true` is present.
- [ ] All three manifests identify the same numeric serial.
- [ ] Stereo and IMU-camera captures use the same IR profile.
- [ ] Allan and IMU-camera captures use the same gyro/accelerometer rates and
  active motion-correction policy.
- [ ] All three captures use one verified `global_time_enabled` policy and
  every selected stream reports the corresponding timestamp domain.
- [ ] Capture modes are exactly `imu-allan`, `stereo-calibration`, and
  `imu-camera-calibration`; Allan correctly has IR disabled.
- [ ] All drop/error/integrity counters are zero.

Any failed item requires a new output directory and a new capture.

## Gate 6: export and external tools

- [ ] Each export completed without `INCOMPLETE`.
- [ ] Each manifest reports `ovrs-calibration-export-v2`.
- [ ] Export manifests remain `UNVERIFIED_CAPTURE`.
- [ ] `validate_calibration_export_set.py` reported
  `calibration export set: PASS`.
- [ ] That gate rechecked source-bound manifest fields, staged camera
  index/image counts and dimensions, and staged IMU row counts.
- [ ] Its report remains `UNVERIFIED_EXPORT_SET` and hashes the exact three
  manifests used for external processing.
- [ ] Its report records one `global_time_enabled` value and rejects a
  Global Time/Hardware Clock mixture.
- [ ] `cam0` and `cam1` contain nanosecond PNG names where applicable.
- [ ] Root `imu0.csv` has the official seven-column bag-creator header where
  applicable.
- [ ] Raw timestamp/IMU provenance exists under `ovrs_metadata`.
- [ ] Copied source metadata exists under `ovrs_metadata` and every manifest
  SHA-256 still matches it.
- [ ] `rosbag info` shows the expected topics for each generated bag.
- [ ] Allan analysis produced plots and four positive noise parameters.
- [ ] Any noise inflation was justified and recorded; it was not hidden.
- [ ] Stereo Kalibr produced camchain, text result, and PDF.
- [ ] Camera-IMU Kalibr produced camchain, text result, and PDF.
- [ ] A second independently recorded camera-IMU run produced repeatable
  transforms, IMU intrinsics, and time offset; millisecond-scale session
  changes were treated as rejection evidence.
- [ ] Both Kalibr runs used `--show-extraction`; tag IDs and image coverage
  were visually checked in both cameras.
- [ ] Stereo processing ran in a new empty work directory with no unexplained
  `--bag-freq` downsampling.
- [ ] Camera-IMU processing used the pinned commit's exact `--cams` option and
  a separate new empty work directory.

Missing topics or output files fail the gate.

## Gate 7: structural and human review

- [ ] Structural validator matched the exact serial.
- [ ] Resolution/rate/motion policy matched capture provenance.
- [ ] Both Kalibr `T_cam_imu` transforms were rigid.
- [ ] Stereo baseline was nonzero and physically plausible.
- [ ] cam0/cam1 time-offset difference was within the operator-declared limit.
- [ ] Extended IMU YAML has positive noise values.
- [ ] Intrinsics came from Kalibr `scale-misalignment`, not its default
  `calibrated` model or a hand-authored status label.
- [ ] `Tw`/`Ta`, `C_gyro_i`, and g-sensitivity were repeatable against an
  independently captured, fully excited camera-IMU sequence.
- [ ] `Tg` equals the recorded Kalibr `A*C_gyro_i` conversion.
- [ ] Camera reprojection plots were inspected.
- [ ] IMU residuals and biases were inspected against 3-sigma bounds.
- [ ] Any repeatable stationary gravity mismatch was reconciled by those
  residual/bias plots or caused promotion to stop for independent
  multi-orientation IMU intrinsic calibration.
- [ ] IMU timestamp-delta plots showed no unexplained batching/gaps.
- [ ] Printed target dimensions were rechecked.
- [ ] Transform direction and time-offset sign were manually checked.
- [ ] Allan plots/fits and noise policy were reviewed.

The structural script's success still means manual review is required.

## Gate 8: promotion

This gate applies only to Branch C. The current selected candidate A did not
pass strict promotion and must not be relabelled.

- [ ] Review report hashes match the current camchain and IMU YAML.
- [ ] A shared time-offset source (`cam0` or `cam1`) was chosen explicitly.
- [ ] All four promotion acknowledgement flags reflect completed review.
- [ ] Promotion wrote a new, non-overwritten serial-specific `kalibr` bundle.
- [ ] Promoted camera YAML contains two `T_imu_cam`, no `T_cam_imu`.
- [ ] Promotion manifest records hashes, acknowledgements, and offset source.
- [ ] Estimator config, camera YAML, and IMU YAML all say
  `KALIBR_VERIFIED` and the same serial.

## Gate 9: new VIO dataset and replay

- [ ] Dataset was recorded by a compatible v0.5.2/v0.6.0 capture path using
      the reviewed librealsense 2.57.3 build, and its provenance was preserved.
- [ ] Dataset has no `INCOMPLETE` marker.
- [ ] Replay did not use `--serial`.
- [ ] Branch A used the exact selected candidate A config, the explicit
  `--allow-unverified-calibration` acknowledgement, and
  `--online-time-offset off`.
- [ ] Branch C used its matching promoted estimator without an unverified
  override.
- [ ] Viewer displayed IR1/IR2 and the interactive global XYZ trajectory with
      visible X, Y, and Z axes plus the ground grid.
- [ ] Incoming states did not move or rescale the grid/axes; only explicit
      `F` fit changed framing.
- [ ] Resizing the trajectory window changed its aspect ratio without forcing
      the old 640x480 canvas.
- [ ] Left-drag orbited, middle/right-drag panned, and wheel zoom stayed
      anchored near the cursor without changing estimator output.
- [ ] `F` fit the track and `R`, `0`, or double-left-click reset the view.
- [ ] Run output finalized cleanly with no `INCOMPLETE`.
- [ ] Trajectory summary contains finite, monotonic states.
- [ ] Path/displacement limits were selected from the physical test before
  judging the output.
- [ ] The trajectory stayed within those limits.

If the return-to-start trajectory diverges: preserve the dataset, run,
calibration bundle, device report, and terminal log. Stop before live
acceptance.

## Gate 10: live acceptance

- [ ] Replay passed first.
- [ ] Live command selected the exact serial.
- [ ] Live stream configuration matched calibration resolution/rates/policy.
- [ ] Branch A again used selected candidate A and
  `--online-time-offset off`; it did not inherit a diagnostic override from a
  previous shell.
- [ ] Viewer remained responsive and closed with `q`, `Esc`, or Ctrl+C.
- [ ] Viewer orbit/pan/zoom remained responsive during live state updates.
- [ ] Image resize preserved aspect ratio and the default trajectory view
  showed positive global Z upward.
- [ ] Viewer visual-support status progressed from `WARMING_UP` to `HEALTHY`
  in the well-lit stationary scene, without rapid status oscillation.
- [ ] `state.csv` and `diagnostics.csv` contained MSCKF candidate, accepted,
  ratio, and update-age fields after the first non-empty update batch. Review
  them as diagnostic evidence; do not invent an acceptance threshold from one
  run.
- [ ] The trajectory pane waited for the first `HEALTHY` pose, then placed its
  origin axis and camera marker together. Any later separation matched logged
  estimated motion rather than a viewer recenter.
- [ ] Deliberate visual occlusion or a dark/blank view held for at least one
  second produced `DEGRADED`; restoring a well-tracked view for at least 1.5
  seconds recovered `HEALTHY`.
- [ ] No camera disconnect, queue overflow, timestamp rejection, NaN, or
  unexpected weak visual-support state occurred outside the deliberate
  transition test.
- [ ] With the camera rigidly stationary, a person crossed the foreground
  without producing sustained camera translation or permanently disabling
  stationary behavior.
- [ ] After a deliberate two-second camera motion and a complete stop, the
  logged velocity returned near zero instead of continuing to propagate.
- [ ] Run finalized cleanly.
- [ ] Physically defined trajectory bounds passed.
- [ ] Fast motion did not produce a multi-metre jump inconsistent with the
  operator's actual movement. If it did, the clean transport/health flags were
  not reported as trajectory success.
- [ ] `state.csv`, `diagnostics.csv`, `run_metadata.yaml`, and
  `application.log` preserved the configured threshold, health state, visual
  support, MSCKF batch quality, and every health transition.

Only after all applicable gates pass may the run be described as successful
for that exact hardware, configuration, and test.

## Gate 11: isolated ORB-SLAM3 return-to-home experiment

- [ ] ORB remains a standalone experimental process; no pose or correction
      is connected to OpenVINS, an EKF, GPS, or flight control.
- [ ] The camera remained still for the startup cue; the startup IMU gate
      passed before deliberate initialization motion began. A gravity mismatch
      was not hidden with an ad-hoc accelerometer scale.
- [ ] Tracking-latency mean/maximum, frame-budget misses, source/submission
      rates, and queue-drop counters were retained for the viewer run.
- [ ] Neither stereo nor IMU exceeded the pinned wall-clock input-stall limit;
      capture duration and shutdown duration were recorded separately.
- [ ] The live bundle, executable, vocabulary, backend patch, and
      `libORB_SLAM3.so` hashes match the capture-time provenance.
- [ ] The `ovrs-closed-loop-reference-v2` file was written before motion and
      records measured rigid-stop tolerances, endpoint-window duration,
      minimum samples, within-hold dispersion limits, and conservative
      minimum path duration/excursion.
- [ ] After the gate-open cue, the camera stayed against the start stop for
      the complete endpoint window before leaving.
- [ ] The camera returned against the same stop and remained there for the
      complete endpoint window before shutdown.
- [ ] No reset, pending reset, map change, tracking loss, over-limit frame
      interval, queue drop, timestamp rejection, or inertial-state regression
      occurred after canonical acceptance.
- [ ] The independent evaluator found nonoverlapping start/end windows with
      sufficient time coverage and samples.
- [ ] Both endpoint windows stayed inside their predeclared position and
      orientation dispersion limits.
- [ ] Canonical duration and maximum estimated excursion met the predeclared
      minimum physical-path bounds; a never-left-home trace did not pass.
- [ ] The robust window-to-window residual, not only the first/last frame,
      stayed inside the independently recorded placement tolerance.
- [ ] Any continuity or return pass remains explicitly
      `NOT_ACCURACY_VALIDATED` without independent trajectory ground truth.

## Gate 12: offline ORB-SLAM3 persistent atlas

- [ ] The source `.osa` and adjacent `.osa.manifest.yaml` both exist and their
      recorded atlas hashes match.
- [ ] Parent and revisit use the same D435i serial, camera/IMU calibration
      hashes, source/adapted camera rates, stride, time-offset policy, backend
      commit/patch/pin, atlas frame policy, library, and vocabulary.
- [ ] The ELF runner resolves the exact `libORB_SLAM3.so` recorded in the
      result manifest.
- [ ] Atlas load and save completed, every detected merge finished, and the
      final atlas contains exactly one nonempty map.
- [ ] The active session reached terminal input coverage and inertial BA2 with
      zero IMU-map reset and zero local-map tracking failure.
- [ ] The result records `parent_atlas_reload_verified_by_this_run: true`.
- [ ] The newly saved atlas remains
      `TRACKING_GATE_PASS_ATLAS_RELOAD_UNVERIFIED` until a later process loads
      it and passes this gate.
- [ ] A repeated recording is reported only as serialization/merge-integrity
      evidence. Relocalization, correct-place identity, false-merge rate, and
      accuracy require a distinct revisit plus independent reference.
