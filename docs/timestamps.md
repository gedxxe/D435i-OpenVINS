# Timestamp and buffering contract

librealsense sensor timestamps are retained in milliseconds and normalized to
double-precision seconds from the first accepted timestamp in a session. Every
stream records its timestamp domain. The normalizer rejects a domain change,
duplicate timestamp, or per-stream regression. It does not silently mix
hardware clock, system time, or global time.

`global_time_enabled` is an explicit, fail-closed stream policy. Startup sets
`RS2_OPTION_GLOBAL_TIME_ENABLED` on every selected RealSense sensor, reads it
back, and refuses to stream when the option is unavailable or does not accept
the requested value. The device report records the requested/active state and
the timestamp domain actually observed on every stream.

The default `config/sensors/realsense_streams.yaml` retains Global Time for
compatibility. The separate
`config/sensors/realsense_streams_hardware_clock_diagnostic.yaml` requests
Hardware Clock only for controlled repeatability experiments. It is not a
claim that Hardware Clock has passed calibration or VIO acceptance on a D435i.
Captures and calibration exports made under the two policies must never be
mixed. A promoted IMU YAML binds the selected policy, and live/replay startup
rejects a stream configuration or device report that disagrees with it.

OpenVINS v2.7 can estimate its shared camera-to-IMU time offset as part of the
state. The estimator YAML remains authoritative by default through
`calib_cam_timeoffset`. `ovrs_live` and `ovrs_replay` provide the explicit
diagnostic override `--online-time-offset on|off`; the override and resolved
state are recorded in `run_metadata.yaml`. Each initialized row in `state.csv`
records the offset, its marginal standard deviation when online, and whether
online estimation is active. The final values are repeated in
`application.log`.

Online estimation is not a replacement for repeatable spatial calibration.
It must not be enabled merely to hide a rejected Kalibr result: compare
independent runs, require bounded trajectories, and preserve failed evidence.
The serial-specific D435i trial recorded on 2026-07-27 demonstrated that the
state is observable, but its unverified factory bootstrap trajectory diverged
by kilometres and was rejected. It does not justify changing the production
default.

The later selected runtime for serial `843212070146` keeps online estimation
off and uses candidate A's fixed -4.900203074 ms Kalibr offset. Online replay
estimates varied between approximately -6.24 ms, -9.23 ms, and -7.63 ms; a
stationary connected-camera trial ended at +2.787724 ms while the state was
weakly observable. Those values are rejected diagnostic evidence and must not
be copied into YAML. Both normal replay and live commands pass
`--online-time-offset off` explicitly; see
[selected_runtime.md](selected_runtime.md).

The initial session origin can make a later-arriving frame slightly negative if
that frame was captured before the first callback. Such a frame cannot have an
IMU bracket and is rejected by synchronization/dispatch rather than
extrapolated. A device clock reset is observed as regression and is fatal for a
live run.

## IMU

Acceleration samples are linearly interpolated at gyro timestamps:

```text
a(tg) = a0 + (a1 - a0) * (tg - t0) / (t1 - t0)
```

`t0 <= tg <= t1` is mandatory. There is no latest-sample substitution and no
extrapolation. Samples before the oldest acceleration bracket are counted as
missing; future samples wait for the next acceleration measurement. Buffer
capacity drops are explicit diagnostics.

librealsense reports motion samples in SI units. v0.4.0 explicitly configures
the SDK motion-correction option and records its requested and actual states.
The runtime also sets the D435i dynamic gyro-sensitivity index before streaming
and reads it back exactly. The selected serial then multiplies the SDK gyro
rad/s vector by its explicit `gyro_scale_factor: 1.0` before synchronization.
Sensitivity and project scale are separate provenance fields; replay does not
reapply the factor to already recorded samples.
The gyroscope stream defines the bootstrap IMU axes. The accelerometer CSV
retains the SDK-delivered value before project-owned frame rotation; it is not
uncalibrated register/ADC data. The synchronized value is rotated into gyro
axes using the actual accelerometer-to-gyro `get_extrinsics_to` rotation
reported by the connected device. Translation is not applied to a free vector;
lever-arm acceleration is not modeled in bootstrap mode. Factory transforms
exported through `get_extrinsics_to` map the gyro/IMU stream into each IR
optical frame; see `docs/calibration.md`.

## Stereo and dispatcher

IR1/IR2 must share the same frameset number and differ by no more than the
configured tolerance (2 ms default). The pair timestamp is their mean.

The dispatcher buffers stereo until IMU spans its timestamp. It feeds IMU up
through the first sample at or after the image, then the stereo pair. This
provides the bracketing data required by OpenVINS propagation while retaining a
deterministic sequence. Closely spaced camera pairs may share an already-fed
IMU bracket; the dispatcher retains the first dispatched IMU timestamp so this
valid reuse is not mistaken for missing earlier coverage. Remaining IMU is
drained on clean shutdown; stereo without IMU coverage is rejected.

Default limits are 4096 synchronized IMU samples and 32 stereo pairs in live
mode. RealSense callback queues and recording queues are also bounded.
