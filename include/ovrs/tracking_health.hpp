#pragma once

#include "ovrs/types.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace ovrs {

struct TrackingHealthGateConfig {
  bool enabled = true;
  std::uint64_t minimum_visual_support_features = 12;
  double degrade_after_s = 1.0;
  double recover_after_s = 1.5;
  double warmup_timeout_s = 3.0;
};

struct TrackingHealthResult {
  TrackingHealthStatus status = TrackingHealthStatus::disabled;
  std::uint64_t visual_support_features = 0;
  double good_duration_s = 0.0;
  double bad_duration_s = 0.0;
  bool healthy = true;
};

const char *tracking_health_status_name(TrackingHealthStatus status) noexcept;
std::string tracking_health_metadata_yaml(
    const TrackingHealthGateConfig &config);
std::string tracking_health_transition_message(const EstimatorState &state);

class TrackingHealthGate {
public:
  explicit TrackingHealthGate(TrackingHealthGateConfig config = {});

  TrackingHealthResult update(double timestamp,
                              std::uint64_t visual_support_features);
  const TrackingHealthGateConfig &config() const noexcept { return config_; }
  void reset() noexcept;

private:
  TrackingHealthResult result() const noexcept;

  TrackingHealthGateConfig config_;
  TrackingHealthStatus status_ = TrackingHealthStatus::disabled;
  std::optional<double> previous_timestamp_;
  double good_duration_s_ = 0.0;
  double bad_duration_s_ = 0.0;
  double evaluation_duration_s_ = 0.0;
  std::uint64_t visual_support_features_ = 0;
};

} // namespace ovrs
