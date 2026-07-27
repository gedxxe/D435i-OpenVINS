#include "ovrs/imu_synchronizer.hpp"

#include <algorithm>
#include <cmath>

namespace ovrs {
namespace {
bool finite(const Vec3 &value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}
} // namespace

ImuSynchronizer::ImuSynchronizer(std::size_t capacity)
    : capacity_(std::max<std::size_t>(capacity, 2)) {}

bool ImuSynchronizer::validate_timestamp(double timestamp,
                                         std::optional<double> &last) {
  if (!std::isfinite(timestamp)) {
    ++stats_.regressing_timestamps;
    return false;
  }
  if (last && timestamp == *last) {
    ++stats_.duplicate_timestamps;
    return false;
  }
  if (last && timestamp < *last) {
    ++stats_.regressing_timestamps;
    return false;
  }
  last = timestamp;
  return true;
}

bool ImuSynchronizer::add_accelerometer(const TimedVec3 &sample) {
  if (stopped_) {
    return false;
  }
  if (!std::isfinite(sample.raw_timestamp_ms) || !finite(sample.value) ||
      (sample.original_sensor_value_available &&
       !finite(sample.original_sensor_value))) {
    ++stats_.invalid_values;
    return false;
  }
  if (!validate_timestamp(sample.timestamp, last_accel_)) {
    return false;
  }
  ++stats_.received_accel;
  accel_.push_back(sample);
  trim_to_capacity();
  process();
  return true;
}

bool ImuSynchronizer::add_gyroscope(const TimedVec3 &sample) {
  if (stopped_) {
    return false;
  }
  if (!std::isfinite(sample.raw_timestamp_ms) || !finite(sample.value) ||
      (sample.original_sensor_value_available &&
       !finite(sample.original_sensor_value))) {
    ++stats_.invalid_values;
    return false;
  }
  if (!validate_timestamp(sample.timestamp, last_gyro_)) {
    return false;
  }
  ++stats_.received_gyro;
  gyro_.push_back(sample);
  trim_to_capacity();
  process();
  return true;
}

void ImuSynchronizer::process() {
  while (!gyro_.empty() && accel_.size() >= 2) {
    const auto &g = gyro_.front();
    while (accel_.size() >= 2 && accel_[1].timestamp < g.timestamp) {
      accel_.pop_front();
    }
    if (accel_.size() < 2) {
      return;
    }
    if (g.timestamp < accel_[0].timestamp) {
      gyro_.pop_front();
      ++stats_.missing_brackets;
      continue;
    }
    if (g.timestamp > accel_[1].timestamp) {
      return;
    }
    const double interval = accel_[1].timestamp - accel_[0].timestamp;
    if (!(interval > 0.0)) {
      accel_.pop_front();
      ++stats_.duplicate_timestamps;
      continue;
    }
    const double alpha = (g.timestamp - accel_[0].timestamp) / interval;
    ImuSample combined;
    combined.timestamp = g.timestamp;
    combined.raw_gyro_timestamp_ms = g.raw_timestamp_ms;
    combined.angular_velocity_rad_s = g.value;
    combined.linear_acceleration_m_s2 =
        lerp(accel_[0].value, accel_[1].value, alpha);
    combined.interpolation_delay_s = accel_[1].timestamp - g.timestamp;
    stats_.maximum_interpolation_delay_s =
        std::max(stats_.maximum_interpolation_delay_s,
                 combined.interpolation_delay_s);
    ready_.push_back(combined);
    gyro_.pop_front();
    ++stats_.generated;
    if (ready_.size() > capacity_) {
      ready_.pop_front();
      ++stats_.dropped_capacity;
    }
  }
}

void ImuSynchronizer::trim_to_capacity() {
  while (accel_.size() > capacity_) {
    accel_.pop_front();
    ++stats_.dropped_capacity;
  }
  while (gyro_.size() > capacity_) {
    gyro_.pop_front();
    ++stats_.dropped_capacity;
  }
}

std::optional<ImuSample> ImuSynchronizer::take_ready() {
  if (ready_.empty()) {
    return std::nullopt;
  }
  auto result = ready_.front();
  ready_.pop_front();
  return result;
}

void ImuSynchronizer::shutdown() {
  stopped_ = true;
  stats_.missing_brackets += gyro_.size();
  accel_.clear();
  gyro_.clear();
}

} // namespace ovrs
