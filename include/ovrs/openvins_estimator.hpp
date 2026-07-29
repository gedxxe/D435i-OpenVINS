#pragma once

#include "ovrs/tracking_health.hpp"
#include "ovrs/types.hpp"

#include <memory>
#include <optional>
#include <string>

namespace ovrs {

struct OpenVinsEstimatorOptions {
  // When unset, preserve calib_cam_timeoffset from the OpenVINS YAML.
  std::optional<bool> camera_imu_time_offset_online_override;
};

class OpenVinsEstimator {
public:
  explicit OpenVinsEstimator(
      const std::string &configuration_path,
      OpenVinsEstimatorOptions options = {});
  ~OpenVinsEstimator();
  OpenVinsEstimator(const OpenVinsEstimator &) = delete;
  OpenVinsEstimator &operator=(const OpenVinsEstimator &) = delete;

  void feed_imu(const ImuSample &sample);
  void feed_stereo(const StereoFrame &frame);
  bool initialized() const;
  double initialization_time() const;
  bool camera_imu_time_offset_online() const;
  TrackingHealthGateConfig tracking_health_gate_config() const;
  std::optional<EstimatorState> latest_state(double processing_latency_ms = 0.0);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace ovrs
