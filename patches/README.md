# OpenVINS patches

OpenVINS remains pinned at v2.7 commit
`93adc241390d13e99232652cf05cbe18a93c7bea`. The Ubuntu build applies
`openvins-zupt-velocity-constraint.patch` and then verifies that the submodule
working tree exactly matches that patch.

Pinned OpenVINS' active ZUPT path updates attitude and IMU biases but does not
observe velocity. A stopped rig can therefore retain a stale nonzero velocity.
The project patch adds an opt-in zero-velocity pseudo measurement, gated by
sustained accumulated low visual disparity. Visual disparity can reject a
candidate but can never bypass the IMU innovation or estimated-velocity gates.
When `zupt_constrain_velocity` is absent or false, ZUPT uses the mandatory IMU
and estimated-velocity gates without the additional velocity observation.

The D435i template enables the patch with a 0.25-second confirmation interval
and 0.05 m/s measurement noise, but conservatively limits ZUPT to the initial
stationary phase. Continuous ZUPT may be enabled only after calibrated,
ground-truthed replay demonstrates that slow handheld motion is not classified
as a stop. Missing or insufficient visual tracks are treated as unknown motion
and reject ZUPT. Do not edit the submodule manually; change the reviewed patch
and rerun the replay and live gates.
