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

`LiveViewer` is opt-in. The capture/dispatcher threads publish only
reference-counted owned image buffers and copied estimator states behind a
mutex. The application main thread alone calls OpenCV HighGUI. Trajectory
history has a configurable bound, window closure requests the same clean
shutdown path as Ctrl+C, and headless operation does not create a window.

`RunWriter` serializes state, diagnostics, metadata log, close, and finalize
operations. This is required because live state output originates on the
dispatcher worker while periodic diagnostics originate on the main thread.

## Dependency boundaries

`ovrs_core` has no OpenCV, RealSense, Eigen, Ceres, or OpenVINS dependency.
`ovrs_realsense` owns RealSense types. `ovrs_openvins` owns all upstream types.
`ovrs_viewer` owns OpenCV HighGUI rendering. Application and log interfaces
use project-owned structs.

OpenVINS is built from its `ov_msckf` directory with `ENABLE_ROS=OFF` and
`ENABLE_ARUCO_TAGS=OFF`; `catkin` and `ament_cmake` package discovery are
disabled as well. Ubuntu 24.04 uses repository-local Ceres 2.1.0 because
OpenVINS v2.7 still uses `LocalParameterization`.

librealsense v2.56.5 is pinned because its release line explicitly includes
Ubuntu 24.04. The official package route is acceptable when the exact version
is available. The repository-local RSUSB build is the fallback and does not
patch kernel modules.

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
