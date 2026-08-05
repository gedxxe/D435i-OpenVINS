#pragma once

#include "ovrs/types.hpp"

#include <cstdint>
#include <optional>

namespace ovrs {

enum class ImuStartupGateState {
  Collecting,
  Passed,
  GravityMismatch,
  StationaryTimeout,
};

struct ImuStartupGateStatus {
  ImuStartupGateState state = ImuStartupGateState::Collecting;
  std::uint64_t samples = 0;
  std::uint64_t rejected_dynamic_windows = 0;
  double window_duration_s = 0.0;
  double acceleration_magnitude_mean_m_s2 = 0.0;
  double acceleration_magnitude_stddev_m_s2 = 0.0;
  double maximum_gyro_magnitude_rad_s = 0.0;
  double gravity_error_m_s2 = 0.0;
};

class ImuStartupGate {
public:
  ImuStartupGate(double expected_gravity_m_s2,
                 double maximum_gravity_error_m_s2,
                 double stationary_window_seconds,
                 double stationary_timeout_seconds,
                 double maximum_acceleration_stddev_m_s2,
                 double maximum_gyro_magnitude_rad_s);

  ImuStartupGateStatus add(const ImuSample &sample);
  ImuStartupGateStatus status() const;

private:
  void reset_window();
  void add_to_window(double timestamp_s, double acceleration_magnitude_m_s2,
                     double gyro_magnitude_rad_s);

  double expected_gravity_m_s2_ = 0.0;
  double maximum_gravity_error_m_s2_ = 0.0;
  double stationary_window_seconds_ = 0.0;
  double stationary_timeout_seconds_ = 0.0;
  double maximum_acceleration_stddev_m_s2_ = 0.0;
  double maximum_gyro_magnitude_rad_s_ = 0.0;
  ImuStartupGateState state_ = ImuStartupGateState::Collecting;
  std::optional<double> first_timestamp_s_;
  std::optional<double> previous_timestamp_s_;
  std::optional<double> window_started_at_s_;
  std::uint64_t window_samples_ = 0;
  std::uint64_t rejected_dynamic_windows_ = 0;
  double acceleration_mean_m_s2_ = 0.0;
  double acceleration_m2_m2_s4_ = 0.0;
  double maximum_window_gyro_magnitude_rad_s_ = 0.0;
};

} // namespace ovrs
