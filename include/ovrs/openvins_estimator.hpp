#pragma once

#include "ovrs/types.hpp"

#include <memory>
#include <optional>
#include <string>

namespace ovrs {

class OpenVinsEstimator {
public:
  explicit OpenVinsEstimator(const std::string &configuration_path);
  ~OpenVinsEstimator();
  OpenVinsEstimator(const OpenVinsEstimator &) = delete;
  OpenVinsEstimator &operator=(const OpenVinsEstimator &) = delete;

  void feed_imu(const ImuSample &sample);
  void feed_stereo(const StereoFrame &frame);
  bool initialized() const;
  double initialization_time() const;
  std::optional<EstimatorState> latest_state(double processing_latency_ms = 0.0);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace ovrs
