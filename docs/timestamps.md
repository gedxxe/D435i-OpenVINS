# Timestamp and buffering contract

librealsense sensor timestamps are retained in milliseconds and normalized to
double-precision seconds from the first accepted timestamp in a session. Every
stream records its timestamp domain. The normalizer rejects a domain change,
duplicate timestamp, or per-stream regression. It does not silently mix
hardware clock, system time, or global time.

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
