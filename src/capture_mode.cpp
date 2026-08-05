#include "ovrs/capture_mode.hpp"

#include <stdexcept>

namespace ovrs {

std::vector<std::string> capture_mode_names() {
  return {"vio", "imu-allan", "stereo-calibration",
          "imu-camera-calibration"};
}

CapturePlan capture_plan(CaptureMode mode) {
  switch (mode) {
  case CaptureMode::Vio:
    return {mode,
            true,
            true,
            true,
            true,
            false,
            false,
            true,
            "vio",
            "ovrs-euroc-like-v1",
            "standalone stereo visual-inertial dataset"};
  case CaptureMode::ImuAllan:
    return {mode,
            false,
            false,
            true,
            true,
            true,
            false,
            false,
            "imu-allan",
            "ovrs-calibration-capture-v1",
            "stationary IMU noise characterization"};
  case CaptureMode::StereoCalibration:
    return {mode,
            true,
            true,
            false,
            false,
            false,
            true,
            false,
            "stereo-calibration",
            "ovrs-calibration-capture-v1",
            "stereo camera intrinsics and extrinsics calibration"};
  case CaptureMode::ImuCameraCalibration:
    return {mode,
            true,
            true,
            true,
            true,
            false,
            true,
            false,
            "imu-camera-calibration",
            "ovrs-calibration-capture-v1",
            "camera-IMU spatial and temporal calibration"};
  }
  throw std::invalid_argument("unknown capture mode");
}

std::optional<CapturePlan> capture_plan(const std::string &name) {
  for (const auto mode : {CaptureMode::Vio, CaptureMode::ImuAllan,
                          CaptureMode::StereoCalibration,
                          CaptureMode::ImuCameraCalibration}) {
    auto plan = capture_plan(mode);
    if (plan.name == name) {
      return plan;
    }
  }
  return std::nullopt;
}

} // namespace ovrs
