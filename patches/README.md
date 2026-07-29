# Reviewed dependency patches

## librealsense RSUSB gyro sensitivity

`librealsense-rsusb-gyro-sensitivity.patch` fixes the pinned SDK's RSUSB
encoding of `RS2_OPTION_GYRO_SENSITIVITY`. The SDK maps levels 1 through 4 to
`0.1` through `0.4`, but the unpatched RSUSB backend casts those values
directly to an unsigned feature-report field and sends zero. The patch encodes
the field in its 0.1 units.

This is a host-side runtime patch. It does not write calibration, firmware, or
EEPROM. The Ubuntu build always builds this exact patched librealsense checkout
and refuses an unreviewed source diff or a system library, even if the system
library reports the pinned version. Its SHA-256 is centralized in
`cmake/DependencyVersions.cmake` and checked before patch application,
preflight, and the registered repository test.

Hardware evidence for D435i `843212070146` on firmware `5.17.3.10`:

- unpatched capture `vio_pitch_up_20260729T120744Z` measured gyro/visual
  rotation ratios of 1.986, 1.979, and 1.944 over three 0.5-second windows;
- after the patch, capture `vio_rsusb_sensitivity_fix_20260729T1315Z` measured
  5.05/5.01, 13.29/13.50, and 12.08/12.08 degrees of gyro/visual rotation;
- both captures used sensitivity level 1, runtime gyro scale 1.0, motion
  correction, and Global Time.

The captures remain local and ignored by Git. These comparisons establish the
session's angular-scale correction, not absolute trajectory accuracy.

## OpenVINS ZUPT

OpenVINS remains pinned at v2.7 commit
`93adc241390d13e99232652cf05cbe18a93c7bea`. The Ubuntu build applies
`openvins-zupt-velocity-constraint.patch` to an ignored local clone under
`.deps/src/open_vins` and verifies that clone exactly matches the patch. The
tracked `third_party/open_vins` submodule remains clean at the pinned commit.

Pinned OpenVINS' active ZUPT path updates attitude and IMU biases but does not
observe velocity. A stopped rig can therefore retain a stale nonzero velocity.
The project patch adds an opt-in zero-velocity pseudo measurement, gated by
sustained accumulated low visual disparity. Visual disparity can reject a
candidate but can never bypass the IMU innovation or estimated-velocity gates.
When `zupt_constrain_velocity` is absent or false, ZUPT uses the mandatory IMU
and estimated-velocity gates without the additional velocity observation.

The generic D435i template uses a 0.25-second confirmation interval, 0.05 m/s
measurement noise, and initialization-only ZUPT. The serial-specific
diagnostic runtime uses a separately documented one-second visual confirmation
for continuous stop recovery. Missing or insufficient visual tracks are
treated as unknown motion and reject ZUPT. Do not edit the submodule manually;
change the reviewed patch and its corresponding SHA-256 pin, then rerun replay
and live gates.
