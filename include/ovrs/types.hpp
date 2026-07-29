#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace ovrs {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

inline Vec3 lerp(const Vec3 &a, const Vec3 &b, double alpha) {
  return {a.x + (b.x - a.x) * alpha, a.y + (b.y - a.y) * alpha,
          a.z + (b.z - a.z) * alpha};
}

struct TimedVec3 {
  double timestamp = 0.0;
  double raw_timestamp_ms = 0.0;
  Vec3 value;
  Vec3 original_sensor_value;
  bool original_sensor_value_available = false;
};

struct ImuSample {
  double timestamp = 0.0;
  double raw_gyro_timestamp_ms = 0.0;
  Vec3 angular_velocity_rad_s;
  Vec3 linear_acceleration_m_s2;
  double interpolation_delay_s = 0.0;
};

struct ImageFrame {
  double timestamp = 0.0;
  double raw_timestamp_ms = 0.0;
  std::uint64_t frameset_number = 0;
  int camera_id = -1;
  int width = 0;
  int height = 0;
  int stride_bytes = 0;
  std::string format;
  std::shared_ptr<std::vector<std::uint8_t>> pixels;
};

struct StereoFrame {
  double timestamp = 0.0;
  ImageFrame camera0;
  ImageFrame camera1;
};

enum class TrackingHealthStatus : std::uint8_t {
  disabled = 0,
  warming_up = 1,
  healthy = 2,
  degraded = 3,
};

struct EstimatorState {
  double timestamp = 0.0;
  Vec3 position_world_m;
  Vec3 velocity_world_m_s;
  // OpenVINS JPL quaternion q_GtoI, serialized in x,y,z,w order.
  std::array<double, 4> q_world_to_imu_xyzw{{0.0, 0.0, 0.0, 1.0}};
  Vec3 gyro_bias_rad_s;
  Vec3 accel_bias_m_s2;
  // Legacy visualization count from OpenVINS' last accepted MSCKF set.
  std::uint64_t msckf_update_features = 0;
  std::uint64_t msckf_candidate_features = 0;
  std::uint64_t msckf_accepted_features = 0;
  double msckf_acceptance_ratio =
      std::numeric_limits<double>::quiet_NaN();
  double msckf_update_age_s =
      std::numeric_limits<double>::quiet_NaN();
  bool msckf_update_quality_available = false;
  std::uint64_t slam_features = 0;
  std::uint64_t visual_support_features = 0;
  double tracking_health_good_duration_s = 0.0;
  double tracking_health_bad_duration_s = 0.0;
  TrackingHealthStatus tracking_health_status =
      TrackingHealthStatus::disabled;
  bool tracking_health_gate_enabled = false;
  double camera_imu_time_offset_s = 0.0;
  double camera_imu_time_offset_variance_s2 =
      std::numeric_limits<double>::quiet_NaN();
  bool camera_imu_time_offset_online = false;
  bool camera_imu_time_offset_variance_available = false;
  // Error-state order: orientation, position, velocity, gyro bias, accel bias.
  std::array<double, 15> covariance_diagonal{};
  bool covariance_available = false;
  bool initialized = false;
  bool healthy = false;
  double processing_latency_ms = 0.0;

  EstimatorState() {
    covariance_diagonal.fill(std::numeric_limits<double>::quiet_NaN());
  }
};

struct DiagnosticsSnapshot {
  double timestamp = 0.0;
  std::uint64_t received_camera_frames = 0;
  std::uint64_t valid_stereo_pairs = 0;
  std::uint64_t received_gyro_samples = 0;
  std::uint64_t received_accel_samples = 0;
  std::uint64_t synchronized_imu_samples = 0;
  std::uint64_t rejected_timestamps = 0;
  std::uint64_t dropped_frames = 0;
  std::size_t imu_queue_depth = 0;
  std::size_t camera_queue_depth = 0;
  double camera_rate_hz = 0.0;
  double imu_rate_hz = 0.0;
  double estimator_rate_hz = 0.0;
  double processing_latency_ms = 0.0;
  bool initialized = false;
  bool healthy = false;
  std::uint64_t msckf_candidate_features = 0;
  std::uint64_t msckf_accepted_features = 0;
  double msckf_acceptance_ratio = 0.0;
  double msckf_update_age_s = 0.0;
  bool msckf_update_quality_available = false;
  std::uint64_t visual_support_features = 0;
  TrackingHealthStatus tracking_health_status =
      TrackingHealthStatus::disabled;
};

} // namespace ovrs
