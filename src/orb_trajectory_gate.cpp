#include "ovrs/orb_trajectory_gate.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ovrs {

OrbTrajectoryGate::OrbTrajectoryGate(
    double minimum_stable_inertial_seconds,
    double maximum_tracking_interval_seconds,
    std::uint64_t maximum_preacceptance_map_resets)
    : minimum_stable_inertial_seconds_(
          minimum_stable_inertial_seconds),
      maximum_tracking_interval_seconds_(
          maximum_tracking_interval_seconds),
      maximum_preacceptance_map_resets_(
          maximum_preacceptance_map_resets) {
  if (!std::isfinite(minimum_stable_inertial_seconds_) ||
      minimum_stable_inertial_seconds_ <= 0.0) {
    throw std::invalid_argument(
        "minimum stable inertial time must be finite and positive");
  }
  if (!std::isfinite(maximum_tracking_interval_seconds_) ||
      maximum_tracking_interval_seconds_ <= 0.0) {
    throw std::invalid_argument(
        "maximum tracking interval must be finite and positive");
  }
}

OrbTrajectoryGateStatus
OrbTrajectoryGate::update(double timestamp_s, bool inertial_initialized,
                          bool inertial_ba2_finished,
                          std::uint64_t active_map_reset_count,
                          std::uint64_t active_map_change_index,
                          OrbTrackingContinuityState tracking_state,
                          bool reset_pending) {
  if (!std::isfinite(timestamp_s)) {
    throw std::invalid_argument("ORB trajectory timestamp must be finite");
  }
  if (last_timestamp_s_ && timestamp_s <= *last_timestamp_s_) {
    throw std::invalid_argument(
        "ORB trajectory timestamps must be strictly increasing");
  }
  const double tracking_interval_s =
      last_timestamp_s_ ? timestamp_s - *last_timestamp_s_ : 0.0;
  maximum_observed_tracking_interval_seconds_ =
      std::max(maximum_observed_tracking_interval_seconds_,
               tracking_interval_s);
  const bool tracking_gap =
      last_timestamp_s_ &&
      tracking_interval_s > maximum_tracking_interval_seconds_;
  if (active_map_reset_count < active_map_reset_count_) {
    throw std::invalid_argument(
        "ORB active-map reset counter must not regress");
  }
  const bool active_map_reset_advanced =
      active_map_reset_count > active_map_reset_count_;
  if (active_map_change_index < active_map_change_index_ &&
      !active_map_reset_advanced) {
    throw std::invalid_argument(
        "ORB active-map change index must not regress");
  }

  if (active_map_reset_count != active_map_reset_count_) {
    const auto reset_delta =
        active_map_reset_count - active_map_reset_count_;
    if (acceptance_started_) {
      discontinuity_detected_ = true;
      postacceptance_map_reset_count_ += reset_delta;
    } else {
      preacceptance_map_reset_count_ += reset_delta;
      stable_inertial_started_at_s_.reset();
    }
    active_map_reset_count_ = active_map_reset_count;
  }
  if (active_map_change_index != active_map_change_index_) {
    if (acceptance_started_) {
      discontinuity_detected_ = true;
      map_change_after_acceptance_ = true;
    }
    active_map_change_index_ = active_map_change_index;
  }
  if (reset_pending) {
    pending_reset_observed_ = true;
    if (acceptance_started_) {
      discontinuity_detected_ = true;
      pending_reset_after_acceptance_observed_ = true;
    } else {
      stable_inertial_started_at_s_.reset();
    }
  }

  if (inertial_initialized) {
    ever_inertial_initialized_ = true;
  } else {
    if (previous_inertial_initialized_) {
      ++inertial_regression_count_;
      if (acceptance_started_) {
        discontinuity_detected_ = true;
      }
    }
  }

  if (inertial_ba2_finished) {
    ever_inertial_ba2_finished_ = true;
  } else if (previous_inertial_ba2_finished_) {
    ++inertial_ba2_regression_count_;
    if (acceptance_started_) {
      discontinuity_detected_ = true;
    }
  }

  const bool tracking_ready =
      tracking_state == OrbTrackingContinuityState::PoseValid;
  if (tracking_gap && acceptance_started_) {
    discontinuity_detected_ = true;
    ++tracking_gap_after_acceptance_count_;
  }
  if (acceptance_started_ && !tracking_ready) {
    discontinuity_detected_ = true;
    if (tracking_state == OrbTrackingContinuityState::Lost) {
      ++tracking_loss_after_acceptance_count_;
    }
  }

  const bool gate_ready =
      inertial_initialized && inertial_ba2_finished && tracking_ready;
  if (gate_ready &&
      (!stable_inertial_started_at_s_ || tracking_gap)) {
    stable_inertial_started_at_s_ = timestamp_s;
  } else if (!gate_ready) {
    stable_inertial_started_at_s_.reset();
  }

  previous_inertial_initialized_ = inertial_initialized;
  previous_inertial_ba2_finished_ = inertial_ba2_finished;
  last_timestamp_s_ = timestamp_s;
  const double stable_elapsed_s =
      gate_ready && stable_inertial_started_at_s_
          ? std::max(0.0, timestamp_s - *stable_inertial_started_at_s_)
          : 0.0;

  if (!acceptance_started_ && !discontinuity_detected_ &&
      !preacceptance_reset_limit_exceeded() && !reset_pending &&
      stable_elapsed_s >= minimum_stable_inertial_seconds_) {
    acceptance_started_ = true;
  }

  return {
      acceptance_started_ && !discontinuity_detected_ &&
          gate_ready && !reset_pending &&
          postacceptance_map_reset_count_ == 0 &&
          !pending_reset_after_acceptance_observed_,
      acceptance_started_,
      discontinuity_detected_,
      stable_elapsed_s,
  };
}

bool OrbTrajectoryGate::ever_inertial_initialized() const {
  return ever_inertial_initialized_;
}

bool OrbTrajectoryGate::inertial_initialized() const {
  return previous_inertial_initialized_;
}

bool OrbTrajectoryGate::ever_inertial_ba2_finished() const {
  return ever_inertial_ba2_finished_;
}

bool OrbTrajectoryGate::inertial_ba2_finished() const {
  return previous_inertial_ba2_finished_;
}

bool OrbTrajectoryGate::acceptance_started() const {
  return acceptance_started_;
}

bool OrbTrajectoryGate::discontinuity_detected() const {
  return discontinuity_detected_;
}

std::uint64_t OrbTrajectoryGate::active_map_reset_count() const {
  return active_map_reset_count_;
}

std::uint64_t OrbTrajectoryGate::preacceptance_map_reset_count() const {
  return preacceptance_map_reset_count_;
}

std::uint64_t OrbTrajectoryGate::postacceptance_map_reset_count() const {
  return postacceptance_map_reset_count_;
}

bool OrbTrajectoryGate::preacceptance_reset_limit_exceeded() const {
  return preacceptance_map_reset_count_ > maximum_preacceptance_map_resets_;
}

std::uint64_t OrbTrajectoryGate::active_map_change_index() const {
  return active_map_change_index_;
}

bool OrbTrajectoryGate::map_change_after_acceptance() const {
  return map_change_after_acceptance_;
}

bool OrbTrajectoryGate::pending_reset_observed() const {
  return pending_reset_observed_;
}

bool OrbTrajectoryGate::pending_reset_after_acceptance_observed() const {
  return pending_reset_after_acceptance_observed_;
}

std::uint64_t OrbTrajectoryGate::inertial_regression_count() const {
  return inertial_regression_count_;
}

std::uint64_t OrbTrajectoryGate::inertial_ba2_regression_count() const {
  return inertial_ba2_regression_count_;
}

std::uint64_t
OrbTrajectoryGate::tracking_loss_after_acceptance_count() const {
  return tracking_loss_after_acceptance_count_;
}

std::uint64_t
OrbTrajectoryGate::tracking_gap_after_acceptance_count() const {
  return tracking_gap_after_acceptance_count_;
}

double
OrbTrajectoryGate::maximum_observed_tracking_interval_seconds() const {
  return maximum_observed_tracking_interval_seconds_;
}

} // namespace ovrs
