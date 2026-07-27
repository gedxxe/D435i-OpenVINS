#pragma once

#include "ovrs/types.hpp"

#include <cstddef>
#include <deque>
#include <optional>

namespace ovrs {

class ImuSynchronizer {
public:
  struct Stats {
    std::uint64_t received_accel = 0;
    std::uint64_t received_gyro = 0;
    std::uint64_t generated = 0;
    std::uint64_t duplicate_timestamps = 0;
    std::uint64_t regressing_timestamps = 0;
    std::uint64_t invalid_values = 0;
    std::uint64_t missing_brackets = 0;
    std::uint64_t dropped_capacity = 0;
    double maximum_interpolation_delay_s = 0.0;
  };

  explicit ImuSynchronizer(std::size_t capacity = 1024);
  bool add_accelerometer(const TimedVec3 &sample);
  bool add_gyroscope(const TimedVec3 &sample);
  std::optional<ImuSample> take_ready();
  void shutdown();
  bool stopped() const { return stopped_; }
  const Stats &stats() const { return stats_; }
  std::size_t accel_buffer_size() const { return accel_.size(); }
  std::size_t gyro_buffer_size() const { return gyro_.size(); }

private:
  bool validate_timestamp(double timestamp, std::optional<double> &last);
  void process();
  void trim_to_capacity();

  std::size_t capacity_;
  std::deque<TimedVec3> accel_;
  std::deque<TimedVec3> gyro_;
  std::deque<ImuSample> ready_;
  std::optional<double> last_accel_;
  std::optional<double> last_gyro_;
  Stats stats_;
  bool stopped_ = false;
};

} // namespace ovrs
