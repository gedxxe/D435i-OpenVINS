# Manual D435i acceptance checklist

This checklist is a stop/go sequence for one D435i serial. Do not skip a failed
gate. Record command output and artifact paths beside each checkbox. Commands
assume the root README section 2 has set `D435I_SERIAL` for runtime gates.
README Step 8 has its own explicit serial/target state gate and must be rerun
after a reboot or terminal restart.

## Gate 1: source and build

- [ ] Root `VERSION` is `0.5.2`.
- [ ] `git diff --check` reports no whitespace errors.
- [ ] `./scripts/build_ubuntu.sh` completed.
- [ ] `./scripts/preflight_ubuntu.sh --require-build` passed.
- [ ] CTest ran named tests; it did not print `No tests were found`.
- [ ] All four executables report v0.5.2 and one source fingerprint.
- [ ] Build cache points to repository-local Ceres 2.1.0.
- [ ] Build cache points to repository-local librealsense 2.56.5.
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

If enumeration succeeds but sampling fails: treat the camera gate as failed.

## Gate 3: OpenVINS transform contract

Inspect the active camera YAML:

```bash
grep -E 'T_(cam_imu|imu_cam)' \
  "config/local/d435i-${D435I_SERIAL}/d435i_factory_imucam.yaml"
```

- [ ] Exactly two `T_imu_cam` entries exist.
- [ ] No `T_cam_imu` entry exists in the OpenVINS runtime file.
- [ ] The meaning is camera coordinates to IMU coordinates.
- [ ] Both rotations are orthonormal with determinant +1.
- [ ] Both bottom rows are `[0, 0, 0, 1]`.
- [ ] Translation columns produce a nonzero, physically plausible baseline.

If a legacy bootstrap file contains `T_cam_imu`, run the v0.5.0 migration once.
Confirm the `.pre-v0.5.0.yaml` backup exists. Never rename the key without
inverting the matrix.

## Gate 4: choose the test branch

### Branch A: mechanical smoke test only

- [ ] `BOOTSTRAP_UNVERIFIED` is understood.
- [ ] `--allow-unverified-calibration` is recorded in the test log.
- [ ] No accuracy, drift, or successful-VIO claim will use this output.

Proceed only to capture/replay/viewer mechanics.

### Branch B: estimator acceptance

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
- [ ] `Tg` is the zero matrix.
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

- [ ] Review report hashes match the current camchain and IMU YAML.
- [ ] A shared time-offset source (`cam0` or `cam1`) was chosen explicitly.
- [ ] All four promotion acknowledgement flags reflect completed review.
- [ ] Promotion wrote a new, non-overwritten serial-specific `kalibr` bundle.
- [ ] Promoted camera YAML contains two `T_imu_cam`, no `T_cam_imu`.
- [ ] Promotion manifest records hashes, acknowledgements, and offset source.
- [ ] Estimator config, camera YAML, and IMU YAML all say
  `KALIBR_VERIFIED` and the same serial.

## Gate 9: new VIO dataset and replay

- [ ] Dataset was recorded after v0.5.2 rebuild and calibration promotion.
- [ ] Dataset has no `INCOMPLETE` marker.
- [ ] Replay did not use `--serial`.
- [ ] Replay did not use `--allow-unverified-calibration`.
- [ ] Replay used the matching verified estimator config.
- [ ] Viewer displayed IR1/IR2 and the live isometric XYZ trajectory with
      visible X, Y, and Z axes.
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
- [ ] Viewer remained responsive and closed with `q`, `Esc`, or Ctrl+C.
- [ ] No camera disconnect, queue overflow, timestamp rejection, NaN, or
  unhealthy estimator state occurred.
- [ ] With the camera rigidly stationary, a person crossed the foreground
  without producing sustained camera translation or permanently disabling
  stationary behavior.
- [ ] After a deliberate two-second camera motion and a complete stop, the
  logged velocity returned near zero instead of continuing to propagate.
- [ ] Run finalized cleanly.
- [ ] Physically defined trajectory bounds passed.

Only after all applicable gates pass may the run be described as successful
for that exact hardware, configuration, and test.
