#include "ovrs/tracking_health.hpp"

#include <cmath>
#include <stdexcept>

namespace ovrs {
namespace {

// A discontinuity this large is not normal frame jitter at any supported
// camera rate. Reset temporal evidence instead of carrying it across a gap.
constexpr double maximum_sample_gap_s = 0.5;

} // namespace

const char *tracking_health_status_name(
    TrackingHealthStatus status) noexcept {
  switch (status) {
  case TrackingHealthStatus::disabled:
    return "DISABLED";
  case TrackingHealthStatus::warming_up:
    return "WARMING_UP";
  case TrackingHealthStatus::healthy:
    return "HEALTHY";
  case TrackingHealthStatus::degraded:
    return "DEGRADED";
  }
  return "INVALID";
}

std::string tracking_health_metadata_yaml(
    const TrackingHealthGateConfig &config) {
  return "tracking_health_gate_enabled: " +
         std::string(config.enabled ? "true" : "false") +
         "\ntracking_health_min_visual_support_features: " +
         std::to_string(config.minimum_visual_support_features) +
         "\ntracking_health_degrade_after_s: " +
         std::to_string(config.degrade_after_s) +
         "\ntracking_health_recover_after_s: " +
         std::to_string(config.recover_after_s) +
         "\ntracking_health_warmup_timeout_s: " +
         std::to_string(config.warmup_timeout_s) + '\n';
}

std::string tracking_health_transition_message(const EstimatorState &state) {
  return "tracking_health_transition timestamp=" +
         std::to_string(state.timestamp) + " status=" +
         tracking_health_status_name(state.tracking_health_status) +
         " visual_support_features=" +
         std::to_string(state.visual_support_features);
}

TrackingHealthGate::TrackingHealthGate(TrackingHealthGateConfig config)
    : config_(config) {
  if (config_.minimum_visual_support_features == 0 ||
      config_.minimum_visual_support_features > 1000000) {
    throw std::invalid_argument(
        "tracking-health minimum visual support must be in [1,1000000]");
  }
  if (!std::isfinite(config_.degrade_after_s) ||
      !std::isfinite(config_.recover_after_s) ||
      !std::isfinite(config_.warmup_timeout_s) ||
      config_.degrade_after_s <= 0.0 || config_.recover_after_s <= 0.0 ||
      config_.warmup_timeout_s <= 0.0 ||
      config_.degrade_after_s > 60.0 || config_.recover_after_s > 60.0 ||
      config_.warmup_timeout_s > 60.0) {
    throw std::invalid_argument(
        "tracking-health timing values must be finite and in (0,60]");
  }
  reset();
}

void TrackingHealthGate::reset() noexcept {
  status_ = config_.enabled ? TrackingHealthStatus::warming_up
                            : TrackingHealthStatus::disabled;
  previous_timestamp_.reset();
  good_duration_s_ = 0.0;
  bad_duration_s_ = 0.0;
  evaluation_duration_s_ = 0.0;
  visual_support_features_ = 0;
}

TrackingHealthResult TrackingHealthGate::update(
    double timestamp, std::uint64_t visual_support_features) {
  if (!std::isfinite(timestamp)) {
    throw std::invalid_argument(
        "tracking-health timestamp must be finite");
  }
  visual_support_features_ = visual_support_features;
  if (!config_.enabled) {
    status_ = TrackingHealthStatus::disabled;
    previous_timestamp_ = timestamp;
    return result();
  }

  if (!previous_timestamp_) {
    previous_timestamp_ = timestamp;
    return result();
  }
  const double elapsed_s = timestamp - *previous_timestamp_;
  if (!std::isfinite(elapsed_s) || elapsed_s <= 0.0) {
    throw std::invalid_argument(
        "tracking-health timestamps must increase strictly");
  }
  previous_timestamp_ = timestamp;
  if (elapsed_s > maximum_sample_gap_s) {
    status_ = TrackingHealthStatus::warming_up;
    good_duration_s_ = 0.0;
    bad_duration_s_ = 0.0;
    evaluation_duration_s_ = 0.0;
    return result();
  }
  evaluation_duration_s_ += elapsed_s;

  const bool visually_supported =
      visual_support_features_ >= config_.minimum_visual_support_features;
  if (visually_supported) {
    good_duration_s_ += elapsed_s;
    bad_duration_s_ = 0.0;
    if (status_ != TrackingHealthStatus::healthy &&
        good_duration_s_ >= config_.recover_after_s) {
      status_ = TrackingHealthStatus::healthy;
    }
  } else {
    bad_duration_s_ += elapsed_s;
    good_duration_s_ = 0.0;
    if (bad_duration_s_ >= config_.degrade_after_s) {
      status_ = TrackingHealthStatus::degraded;
    }
  }
  if (status_ == TrackingHealthStatus::warming_up &&
      evaluation_duration_s_ >= config_.warmup_timeout_s) {
    status_ = TrackingHealthStatus::degraded;
  }
  return result();
}

TrackingHealthResult TrackingHealthGate::result() const noexcept {
  TrackingHealthResult value;
  value.status = status_;
  value.visual_support_features = visual_support_features_;
  value.good_duration_s = good_duration_s_;
  value.bad_duration_s = bad_duration_s_;
  value.healthy = status_ == TrackingHealthStatus::disabled ||
                  status_ == TrackingHealthStatus::healthy;
  return value;
}

} // namespace ovrs
