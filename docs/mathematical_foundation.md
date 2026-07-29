# Mathematical foundation

This document summarizes the mathematical contract implemented by this
repository. It is an orientation guide, not a replacement for the complete
[OpenVINS derivations](https://docs.openvins.com/pages.html).

GitHub renders inline expressions between single dollar delimiters. Display
equations use fenced `math` blocks, which avoids exposing raw delimiter
markers when Markdown is viewed on GitHub.

## IMU measurement and propagation

Let ${}^{G}\!R_I$ rotate an IMU-frame vector into the global frame. With
gyroscope bias $b_g$, accelerometer bias $b_a$, white measurement noises
$n_g,n_a$, and global gravity ${}^{G}\!g$, the basic calibrated IMU model is

```math
\omega_m = \omega + b_g + n_g,
\qquad
a_m = {}^{I}\!R_G\left({}^{G}\!a_I-{}^{G}\!g\right)+b_a+n_a.
```

The continuous navigation state evolves as

```math
\dot{{}^{G}\!R_I}
  = {}^{G}\!R_I[\omega_m-b_g-n_g]_\times,
\qquad
\dot{{}^{G}\!p_I} = {}^{G}\!v_I,
```

```math
\dot{{}^{G}\!v_I}
  = {}^{G}\!R_I(a_m-b_a-n_a)+{}^{G}\!g,
\qquad
\dot b_g=n_{wg},\quad \dot b_a=n_{wa}.
```

Here $[\cdot]_\times$ is the skew-symmetric cross-product matrix, while
$n_{wg}$ and $n_{wa}$ drive bias random walks. The runtime uses OpenVINS RK4
mean propagation and covariance propagation over the 15-element IMU error
state: orientation, position, velocity, gyro bias, and accelerometer bias.

The fuller OpenVINS intrinsic model additionally admits invertible scale and
cross-axis matrices $T_w,T_a$, sensor-frame rotations, and gyro
gravity-sensitivity $T_g$. A stationary Allan recording estimates stochastic
noise terms; it cannot identify all deterministic intrinsic matrices from one
orientation. Intrinsic calibration needs well-excited motion to avoid
degeneracy.

## D435i sampling and time alignment

Librealsense supplies motion values in rad/s and m/s². The generic
`StreamConfig` preserves SDK gyro values with scale `1.0`; the selected serial
explicitly uses the post-calibration factor $s_g=1.0$:

```math
\omega_{\mathrm{used}}=s_g\,\omega_{\mathrm{SDK}}.
```

The accelerometer measurement is rotated using the device-reported
accel-to-gyro rotation:

```math
a_{\mathrm{gyro}}^{\text{sample}}
=R_{\mathrm{gyro}\leftarrow\mathrm{accel}}\,
 a_{\mathrm{accel}}^{\text{sample}}.
```

Because the configured accelerometer and gyro rates differ, acceleration at a
gyro timestamp $t_g\in[t_{a0},t_{a1}]$ is

```math
\alpha=\frac{t_g-t_{a0}}{t_{a1}-t_{a0}},
\qquad
a(t_g)=(1-\alpha)a(t_{a0})+\alpha a(t_{a1}).
```

Raw device timestamps remain beside normalized seconds so synchronization can
be audited. The runtime requires the configured motion profiles rather than
silently substituting the nearest available rates.

## Stereo geometry

For a rectified pinhole stereo pair with focal length $f_x$, baseline $b$, and
disparity $d=u_L-u_R$, approximate depth is

```math
Z \approx \frac{f_x b}{d}.
```

First-order uncertainty grows approximately as

```math
\sigma_Z \approx \frac{Z^2}{f_x b}\,\sigma_d.
```

The D435i's short stereo baseline means distant, low-disparity features have
weak depth conditioning. Fast blur, repeated texture, poor exposure, bad
intrinsics, or a wrong stereo transform can damage the visual constraint even
when the IMU stream is numerically smooth.

The runtime key `T_imu_cam` maps a homogeneous point from camera coordinates
into IMU coordinates:

```math
\begin{bmatrix}p_I\\1\end{bmatrix}
=T_{\mathrm{imu}\leftarrow\mathrm{cam}}
\begin{bmatrix}p_C\\1\end{bmatrix}.
```

Kalibr emits the opposite transform direction. Its result is inverted only at
the reviewed promotion boundary.

## MSCKF visual constraint

For a tracked feature, the linearized pixel residual over its observations is

```math
r \approx H_x\tilde x + H_f\tilde p_f+n.
```

MSCKF does not retain that feature as a permanent map landmark. It finds a
left-nullspace basis $N$ satisfying $N^\mathsf{T}H_f=0$, then projects

```math
r_o=N^\mathsf{T}r
\approx N^\mathsf{T}H_x\tilde x+N^\mathsf{T}n.
```

The resulting constraint updates the navigation state while eliminating the
unknown feature error. For $H=N^\mathsf{T}H_x$ and projected noise covariance
$R$, the EKF update follows

```math
S=HPH^\mathsf{T}+R,\qquad
K=PH^\mathsf{T}S^{-1},
```

```math
\delta x=Kr_o,\qquad
P^+=(I-KH)P^-.
```

The project keeps OpenVINS First-Estimate Jacobians enabled to preserve its
intended observability and consistency behavior.

## ZUPT and fail-closed stationarity

The reviewed OpenVINS patch adds a velocity observation only after inertial and
visual stationarity agree. For a zero-velocity residual,

```math
r_v=0-\hat v_I,\qquad H_v=
\begin{bmatrix}0&0&I_3&0&0\end{bmatrix}.
```

Low feature disparity is an additional gate. Missing visual tracks mean
unknown, not stationary. The serial-specific selected runtime permits
post-motion recovery only after one second of consecutive frames with more
than 20 common tracks and mean disparity below 2 px; any moving or unknown
frame resets the candidate. The generic estimator template remains more
conservative.

ZUPT constrains velocity. It cannot reconstruct a position already corrupted
by drift.

## IMU noise parameters

For sample interval $\Delta t$, a continuous white-noise density $\sigma_n$
maps approximately to per-sample standard deviation
$\sigma_n/\sqrt{\Delta t}$, while bias random-walk increments scale as
$\sigma_w\sqrt{\Delta t}$ under the convention used by the calibration tool.
Recording longer extends the Allan time scales available to fitting; it does
not correct wrong units, axes, timestamps, motion contamination, or a bad fit.

## Primary references

- [OpenVINS IMU propagation derivations](https://docs.openvins.com/propagation.html)
- [OpenVINS camera measurement model](https://docs.openvins.com/update-feat.html)
- [OpenVINS MSCKF nullspace projection](https://docs.openvins.com/update-null.html)
- [OpenVINS zero-velocity update](https://docs.openvins.com/update-zerovelocity.html)
- [Librealsense D400 IMU coordinate system](https://github.com/IntelRealSense/librealsense/blob/master/doc/rs400/rs400_imu_coordinates.md)
