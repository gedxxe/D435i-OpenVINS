#include "ovrs/imu_startup_gate.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ovrs {
namespace {

double magnitude(const Vec3 &value) {
  return std::sqrt(value.x * value.x + value.y * value.y +
                   value.z * value.z);
}

} // namespace

ImuStartupGate::ImuStartupGate(
    double expected_gravity_m_s2, double maximum_gravity_error_m_s2,
    double stationary_window_seconds, double stationary_timeout_seconds,
    double maximum_acceleration_stddev_m_s2,
    double maximum_gyro_magnitude_rad_s)
    : expected_gravity_m_s2_(expected_gravity_m_s2),
      maximum_gravity_error_m_s2_(maximum_gravity_error_m_s2),
      stationary_window_seconds_(stationary_window_seconds),
      stationary_timeout_seconds_(stationary_timeout_seconds),
      maximum_acceleration_stddev_m_s2_(
          maximum_acceleration_stddev_m_s2),
      maximum_gyro_magnitude_rad_s_(maximum_gyro_magnitude_rad_s) {
  if (!std::isfinite(expected_gravity_m_s2_) ||
      expected_gravity_m_s2_ <= 0.0 ||
      !std::isfinite(maximum_gravity_error_m_s2_) ||
      maximum_gravity_error_m_s2_ <= 0.0 ||
      maximum_gravity_error_m_s2_ >= expected_gravity_m_s2_ ||
      !std::isfinite(stationary_window_seconds_) ||
      stationary_window_seconds_ <= 0.0 ||
      !std::isfinite(stationary_timeout_seconds_) ||
      stationary_timeout_seconds_ < stationary_window_seconds_ ||
      !std::isfinite(maximum_acceleration_stddev_m_s2_) ||
      maximum_acceleration_stddev_m_s2_ <= 0.0 ||
      !std::isfinite(maximum_gyro_magnitude_rad_s_) ||
      maximum_gyro_magnitude_rad_s_ <= 0.0) {
    throw std::invalid_argument("invalid IMU startup-gate thresholds");
  }
}

ImuStartupGateStatus ImuStartupGate::add(const ImuSample &sample) {
  if (state_ != ImuStartupGateState::Collecting) {
    return status();
  }
  if (!std::isfinite(sample.timestamp) ||
      (previous_timestamp_s_ && sample.timestamp <= *previous_timestamp_s_)) {
    throw std::invalid_argument(
        "IMU startup-gate timestamps must be finite and strictly increasing");
  }
  const double acceleration_magnitude =
      magnitude(sample.linear_acceleration_m_s2);
  const double gyro_magnitude = magnitude(sample.angular_velocity_rad_s);
  if (!std::isfinite(acceleration_magnitude) ||
      !std::isfinite(gyro_magnitude)) {
    throw std::invalid_argument("IMU startup-gate sample must be finite");
  }
  if (!first_timestamp_s_) {
    first_timestamp_s_ = sample.timestamp;
  }
  previous_timestamp_s_ = sample.timestamp;

  if (gyro_magnitude > maximum_gyro_magnitude_rad_s_) {
    if (window_samples_ > 0) {
      ++rejected_dynamic_windows_;
    }
    reset_window();
  } else {
    add_to_window(sample.timestamp, acceleration_magnitude, gyro_magnitude);
    const double window_duration =
        sample.timestamp - *window_started_at_s_;
    if (window_samples_ >= 2 &&
        window_duration >= stationary_window_seconds_) {
      const double stddev = std::sqrt(
          acceleration_m2_m2_s4_ /
          static_cast<double>(window_samples_ - 1));
      if (stddev > maximum_acceleration_stddev_m_s2_) {
        ++rejected_dynamic_windows_;
        reset_window();
      } else if (std::abs(acceleration_mean_m_s2_ -
                          expected_gravity_m_s2_) >
                 maximum_gravity_error_m_s2_) {
        state_ = ImuStartupGateState::GravityMismatch;
      } else {
        state_ = ImuStartupGateState::Passed;
      }
    }
  }

  if (state_ == ImuStartupGateState::Collecting &&
      sample.timestamp - *first_timestamp_s_ >= stationary_timeout_seconds_) {
    state_ = ImuStartupGateState::StationaryTimeout;
  }
  return status();
}

ImuStartupGateStatus ImuStartupGate::status() const {
  const double duration =
      window_started_at_s_ && previous_timestamp_s_
          ? std::max(0.0, *previous_timestamp_s_ - *window_started_at_s_)
          : 0.0;
  const double stddev =
      window_samples_ >= 2
          ? std::sqrt(acceleration_m2_m2_s4_ /
                      static_cast<double>(window_samples_ - 1))
          : 0.0;
  return {state_,
          window_samples_,
          rejected_dynamic_windows_,
          duration,
          acceleration_mean_m_s2_,
          stddev,
          maximum_window_gyro_magnitude_rad_s_,
          window_samples_ > 0
              ? std::abs(acceleration_mean_m_s2_ - expected_gravity_m_s2_)
              : 0.0};
}

void ImuStartupGate::reset_window() {
  window_started_at_s_.reset();
  window_samples_ = 0;
  acceleration_mean_m_s2_ = 0.0;
  acceleration_m2_m2_s4_ = 0.0;
  maximum_window_gyro_magnitude_rad_s_ = 0.0;
}

void ImuStartupGate::add_to_window(double timestamp_s,
                                   double acceleration_magnitude_m_s2,
                                   double gyro_magnitude_rad_s) {
  if (!window_started_at_s_) {
    window_started_at_s_ = timestamp_s;
  }
  ++window_samples_;
  const double delta = acceleration_magnitude_m_s2 - acceleration_mean_m_s2_;
  acceleration_mean_m_s2_ += delta / static_cast<double>(window_samples_);
  const double delta_after_mean =
      acceleration_magnitude_m_s2 - acceleration_mean_m_s2_;
  acceleration_m2_m2_s4_ += delta * delta_after_mean;
  maximum_window_gyro_magnitude_rad_s_ =
      std::max(maximum_window_gyro_magnitude_rad_s_, gyro_magnitude_rad_s);
}

} // namespace ovrs
