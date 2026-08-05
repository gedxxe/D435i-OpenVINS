#pragma once

#include <cstdint>
#include <optional>

namespace ovrs {

enum class OrbTrackingContinuityState {
  NotReady,
  PoseValid,
  Lost,
};

struct OrbTrajectoryGateStatus {
  bool accept_pose = false;
  bool acceptance_started = false;
  bool discontinuity_detected = false;
  double stable_gate_elapsed_s = 0.0;
};

class OrbTrajectoryGate {
public:
  OrbTrajectoryGate(double minimum_stable_inertial_seconds,
                    double maximum_tracking_interval_seconds,
                    std::uint64_t maximum_preacceptance_map_resets = 0);

  OrbTrajectoryGateStatus update(double timestamp_s,
                                 bool inertial_initialized,
                                 bool inertial_ba2_finished,
                                 std::uint64_t active_map_reset_count,
                                 std::uint64_t active_map_change_index,
                                 OrbTrackingContinuityState tracking_state,
                                 bool reset_pending = false,
                                 bool visual_support_sufficient = true);

  bool ever_inertial_initialized() const;
  bool inertial_initialized() const;
  bool ever_inertial_ba2_finished() const;
  bool inertial_ba2_finished() const;
  bool acceptance_started() const;
  bool discontinuity_detected() const;
  std::uint64_t active_map_reset_count() const;
  std::uint64_t preacceptance_map_reset_count() const;
  std::uint64_t postacceptance_map_reset_count() const;
  bool preacceptance_reset_limit_exceeded() const;
  std::uint64_t active_map_change_index() const;
  bool map_change_after_acceptance() const;
  bool pending_reset_observed() const;
  bool pending_reset_after_acceptance_observed() const;
  std::uint64_t inertial_regression_count() const;
  std::uint64_t inertial_ba2_regression_count() const;
  std::uint64_t tracking_loss_after_acceptance_count() const;
  std::uint64_t tracking_gap_after_acceptance_count() const;
  std::uint64_t visual_support_failure_after_acceptance_count() const;
  double maximum_observed_tracking_interval_seconds() const;

private:
  double minimum_stable_inertial_seconds_ = 0.0;
  double maximum_tracking_interval_seconds_ = 0.0;
  std::uint64_t maximum_preacceptance_map_resets_ = 0;
  double maximum_observed_tracking_interval_seconds_ = 0.0;
  std::optional<double> last_timestamp_s_;
  std::optional<double> stable_inertial_started_at_s_;
  bool previous_inertial_initialized_ = false;
  bool ever_inertial_initialized_ = false;
  bool previous_inertial_ba2_finished_ = false;
  bool ever_inertial_ba2_finished_ = false;
  bool acceptance_started_ = false;
  bool discontinuity_detected_ = false;
  bool map_change_after_acceptance_ = false;
  bool pending_reset_observed_ = false;
  bool pending_reset_after_acceptance_observed_ = false;
  std::uint64_t active_map_reset_count_ = 0;
  std::uint64_t preacceptance_map_reset_count_ = 0;
  std::uint64_t postacceptance_map_reset_count_ = 0;
  std::uint64_t active_map_change_index_ = 0;
  std::uint64_t inertial_regression_count_ = 0;
  std::uint64_t inertial_ba2_regression_count_ = 0;
  std::uint64_t tracking_loss_after_acceptance_count_ = 0;
  std::uint64_t tracking_gap_after_acceptance_count_ = 0;
  std::uint64_t visual_support_failure_after_acceptance_count_ = 0;
};

} // namespace ovrs
