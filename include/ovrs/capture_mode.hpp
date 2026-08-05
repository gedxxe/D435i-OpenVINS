#pragma once

#include <optional>
#include <string>
#include <vector>

namespace ovrs {

enum class CaptureMode {
  Vio,
  ImuAllan,
  StereoCalibration,
  ImuCameraCalibration,
};

struct CapturePlan {
  CaptureMode mode = CaptureMode::Vio;
  bool enable_stereo = true;
  bool supports_preview = true;
  bool enable_motion = true;
  bool write_synchronized_imu = true;
  bool requires_stationary_sensor = false;
  bool requires_calibration_target = false;
  bool replay_compatible = true;
  std::string name;
  std::string format;
  std::string purpose;
};

std::vector<std::string> capture_mode_names();
std::optional<CapturePlan> capture_plan(const std::string &name);
CapturePlan capture_plan(CaptureMode mode);

} // namespace ovrs
