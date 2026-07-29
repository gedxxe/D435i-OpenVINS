# Estimator configuration

`config/sensors/d435i_bootstrap.yaml` is a non-runnable source template.
Do not pass it directly to live or replay.

After reviewing an `ovrs_inspect --export-calibration` result, use
`bash scripts/prepare_bootstrap_config.sh` with the expected serial and the
same stream-config path used by inspector/record/live, as documented in the
root README.
It creates a local bundle under `config/local/d435i-SERIAL/` because OpenVINS
resolves IMU and camera files relative to the main YAML. Calibration captures,
factory exports, and candidate bundles remain ignored. Only YAML files copied
into a reviewed `selected_runtime/` directory are publishable, which lets a
fresh clone reproduce the documented runtime without including raw evidence.

The main, IMU, and camera YAML files must carry the same `calibration_state`
and `calibrated_serial`. This prevents relabelling only the main/camera files
as verified while retaining another unit's or the bootstrap IMU noise values.
Both camera `resolution` entries must match the live stream or recorded
dataset. Never copy a bundle from another D435i.

The project canonical runtime key is `T_imu_cam`; it maps camera coordinates
into IMU coordinates. Pinned OpenVINS v2.7 also has a compatibility fallback
for Kalibr's inverse `T_cam_imu`, but project validation rejects that ambiguity.
Only `scripts/prepare_verified_calibration.py` should convert Kalibr output. A
legacy local factory bundle containing `T_cam_imu` must pass through
`scripts/migrate_openvins_transform_v050.py` before use.

The factory bundle remains `BOOTSTRAP_UNVERIFIED` and is blocked by default.
The full Allan/Kalibr workflow in the root README can create a strict
`KALIBR_VERIFIED` bundle under
`config/local/d435i-SERIAL/kalibr/estimator.yaml`.

This checkout currently selects the limited-evidence repeatability bundle for
serial `843212070146`:

```text
config/local/d435i-843212070146/selected_runtime/estimator.yaml
```

It is the only normal runtime choice for that unit. It requires
`--allow-unverified-calibration` because its small-target and Allan provenance
do not satisfy strict promotion. It also requires
`--online-time-offset off`; candidate B, the factory bootstrap, and online
offset results are diagnostic evidence rather than interchangeable configs.
See the [selected runtime contract](../../docs/selected_runtime.md).

## D435i short-baseline defaults

The source template deliberately overrides two generic OpenVINS feature
initializer gates:

- `fi_max_cond_number: 60000`
- `fi_max_baseline: 100`

The D435i stereo baseline is about 50 mm, so features several metres away can
exceed OpenVINS' generic condition-number and depth-to-baseline limits even
when their stereo depth and epipolar agreement are valid. The template also
uses a two-second initialization window, 200 tracked points, and global
histogram equalization. Controlled replay showed that all four choices were
needed to prevent the visual updates from collapsing on the recorded indoor
sequences; changing the MSCKF chi-square multiplier was not needed.

The reviewed project patch makes a velocity-constraining ZUPT available:

- `zupt_constrain_velocity: true`
- `zupt_velocity_noise: 0.05`
- `zupt_min_stationary_time: 1.0` in the selected runtime

The velocity observation is not triggered by one low-disparity frame. It
requires more than 20 common tracks and consecutive per-frame disparity below
`zupt_max_disparity` for the entire confirmation interval; moving or unknown
frames reset that interval. The generic source template retains
`zupt_only_at_beginning: true`. The serial-specific selected runtime uses
`zupt_only_at_beginning: false` after corrected moving replay proved the older
interval-wide track check was unreachable and the reviewed per-frame gate
recovered bounded near-zero velocity after a real stop. This is evidence for
the selected camera and dataset, not a universal continuous-ZUPT default.

Runtime also fails closed when estimated speed exceeds 3 m/s or accelerometer
bias exceeds 2 m/s^2. These are integrity limits, not accuracy claims; a run
that trips either limit must be treated as failed and preserved for diagnosis.

## Tracking-health gate

The main estimator YAML also configures a project-owned, output-only tracking
health gate:

```yaml
tracking_health_gate_enabled: true
tracking_health_min_visual_support_features: 12
tracking_health_degrade_after_s: 1.0
tracking_health_recover_after_s: 1.5
tracking_health_warmup_timeout_s: 3.0
```

Visual support is the frontend track count observed in camera 0 for the
current frame. It deliberately does not count persistent SLAM landmarks that
were not observed in that frame and does not require a track to have a
triangulated depth merely to report frontend continuity. The gate begins
fail-closed in `WARMING_UP`, becomes `HEALTHY` only after continuous recovery
evidence, and becomes `DEGRADED` only after continuous weak evidence. A
three-second warm-up timeout also marks flickering support `DEGRADED` instead
of leaving it in `WARMING_UP` indefinitely. It does not clamp, reset, or
correct pose.
Live and replay apply identical logic, serialize transitions and durations,
and show the current status in the viewer.

The viewer and run CSV files additionally report the latest non-empty MSCKF
batch as accepted/candidate features, acceptance ratio, and batch age. This is
read-only evidence from the updater and is deliberately not controlled by a
hidden threshold. Event-driven MSCKF acceptance depends on motion, parallax,
and marginalization; it cannot by itself certify pose accuracy.

Do not lower the threshold merely to turn a red warning green. Compare
identical-data replay and a physically bounded live test. Even `HEALTHY`
cannot correct accumulated position drift or replace ground truth, mapping,
loop closure, or an external position anchor.

These are estimator settings, not calibration results. Replay without external
ground truth can demonstrate consistency and reduced round-trip drift, but
cannot certify metric trajectory accuracy.
