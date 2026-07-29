#include "ovrs/openvins_estimator.hpp"
#include "ovrs/trajectory.hpp"

#include "core/VioManager.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/sensor_data.h"

#include <opencv2/core.hpp>

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace ovrs {

class OpenVinsEstimator::Impl {
public:
  explicit Impl(const std::string &path,
                const OpenVinsEstimatorOptions &overrides) {
    auto parser = std::make_shared<ov_core::YamlParser>(path);
    std::string verbosity = "INFO";
    parser->parse_config("verbosity", verbosity);
    ov_core::Printer::setPrintLevel(verbosity);
    parser->parse_config("max_estimated_speed_m_s", max_estimated_speed_m_s,
                        false);
    parser->parse_config("max_accel_bias_m_s2", max_accel_bias_m_s2, false);
    parser->parse_config("tracking_health_gate_enabled",
                         tracking_health_config.enabled, false);
    int minimum_visual_support_features =
        static_cast<int>(
            tracking_health_config.minimum_visual_support_features);
    parser->parse_config("tracking_health_min_visual_support_features",
                         minimum_visual_support_features, false);
    if (minimum_visual_support_features <= 0) {
      throw std::runtime_error(
          "tracking_health_min_visual_support_features must be positive");
    }
    tracking_health_config.minimum_visual_support_features =
        static_cast<std::uint64_t>(minimum_visual_support_features);
    parser->parse_config("tracking_health_degrade_after_s",
                         tracking_health_config.degrade_after_s, false);
    parser->parse_config("tracking_health_recover_after_s",
                         tracking_health_config.recover_after_s, false);
    parser->parse_config("tracking_health_warmup_timeout_s",
                         tracking_health_config.warmup_timeout_s, false);
    tracking_health_gate =
        std::make_unique<TrackingHealthGate>(tracking_health_config);

    ov_msckf::VioManagerOptions options;
    options.print_and_load(parser);
    if (!parser->successful()) {
      throw std::runtime_error("OpenVINS configuration is incomplete: " +
                               path);
    }
    if (overrides.camera_imu_time_offset_online_override.has_value()) {
      options.state_options.do_calib_camera_timeoffset =
          *overrides.camera_imu_time_offset_online_override;
    }
    if (options.state_options.num_cameras != 2 || !options.use_stereo) {
      throw std::runtime_error(
          "OpenVINS configuration must set max_cameras: 2 and "
          "use_stereo: true");
    }
    camera_imu_time_offset_online =
        options.state_options.do_calib_camera_timeoffset;
    manager = std::make_shared<ov_msckf::VioManager>(options);
  }

  std::shared_ptr<ov_msckf::VioManager> manager;
  bool camera_imu_time_offset_online = false;
  double max_estimated_speed_m_s = 0.0;
  double max_accel_bias_m_s2 = 0.0;
  TrackingHealthGateConfig tracking_health_config;
  std::unique_ptr<TrackingHealthGate> tracking_health_gate;
};

OpenVinsEstimator::OpenVinsEstimator(
    const std::string &configuration_path, OpenVinsEstimatorOptions options)
    : impl_(std::make_unique<Impl>(configuration_path, options)) {}
OpenVinsEstimator::~OpenVinsEstimator() = default;

void OpenVinsEstimator::feed_imu(const ImuSample &sample) {
  ov_core::ImuData measurement;
  measurement.timestamp = sample.timestamp;
  measurement.wm << sample.angular_velocity_rad_s.x,
      sample.angular_velocity_rad_s.y, sample.angular_velocity_rad_s.z;
  measurement.am << sample.linear_acceleration_m_s2.x,
      sample.linear_acceleration_m_s2.y, sample.linear_acceleration_m_s2.z;
  impl_->manager->feed_measurement_imu(measurement);
}

void OpenVinsEstimator::feed_stereo(const StereoFrame &frame) {
  const auto make_mat = [](const ImageFrame &image) {
    const std::size_t required =
        static_cast<std::size_t>(image.stride_bytes) * image.height;
    if (!image.pixels || image.pixels->size() < required ||
        image.format != "Y8") {
      throw std::runtime_error("invalid Y8 image passed to OpenVINS");
    }
    return cv::Mat(image.height, image.width, CV_8UC1,
                   image.pixels->data(), image.stride_bytes);
  };
  ov_core::CameraData measurement;
  measurement.timestamp = frame.timestamp;
  measurement.sensor_ids = {0, 1};
  measurement.images = {make_mat(frame.camera0), make_mat(frame.camera1)};
  measurement.masks = {
      cv::Mat::zeros(frame.camera0.height, frame.camera0.width, CV_8UC1),
      cv::Mat::zeros(frame.camera1.height, frame.camera1.width, CV_8UC1)};
  impl_->manager->feed_measurement_camera(measurement);
}

bool OpenVinsEstimator::initialized() const {
  return impl_->manager->initialized();
}

double OpenVinsEstimator::initialization_time() const {
  return impl_->manager->initialized_time();
}

bool OpenVinsEstimator::camera_imu_time_offset_online() const {
  return impl_->camera_imu_time_offset_online;
}

TrackingHealthGateConfig
OpenVinsEstimator::tracking_health_gate_config() const {
  return impl_->tracking_health_config;
}

std::optional<EstimatorState>
OpenVinsEstimator::latest_state(double processing_latency_ms) {
  if (!initialized()) {
    return std::nullopt;
  }
  auto state = impl_->manager->get_state();
  std::lock_guard<std::mutex> lock(state->_mutex_state);
  EstimatorState result;
  result.timestamp = state->_timestamp;
  const auto position = state->_imu->pos();
  const auto velocity = state->_imu->vel();
  const auto quaternion = state->_imu->quat();
  const auto gyro_bias = state->_imu->bias_g();
  const auto accel_bias = state->_imu->bias_a();
  result.position_world_m = {position(0), position(1), position(2)};
  result.velocity_world_m_s = {velocity(0), velocity(1), velocity(2)};
  for (int i = 0; i < 4; ++i) {
    result.q_world_to_imu_xyzw[static_cast<std::size_t>(i)] = quaternion(i);
  }
  result.gyro_bias_rad_s = {gyro_bias(0), gyro_bias(1), gyro_bias(2)};
  result.accel_bias_m_s2 = {accel_bias(0), accel_bias(1), accel_bias(2)};
  result.msckf_update_features =
      static_cast<std::uint64_t>(impl_->manager->get_good_features_MSCKF().size());
  const auto msckf_stats =
      impl_->manager->get_last_msckf_update_stats();
  if (msckf_stats.timestamp >= 0.0 &&
      msckf_stats.candidate_features > 0 &&
      msckf_stats.timestamp <= result.timestamp) {
    result.msckf_candidate_features =
        static_cast<std::uint64_t>(msckf_stats.candidate_features);
    result.msckf_accepted_features =
        static_cast<std::uint64_t>(msckf_stats.accepted_features);
    result.msckf_acceptance_ratio =
        static_cast<double>(msckf_stats.accepted_features) /
        static_cast<double>(msckf_stats.candidate_features);
    result.msckf_update_age_s =
        result.timestamp - msckf_stats.timestamp;
    result.msckf_update_quality_available = true;
  }
  result.slam_features =
      static_cast<std::uint64_t>(impl_->manager->get_features_SLAM().size());
  result.visual_support_features = static_cast<std::uint64_t>(
      impl_->manager->get_active_track_count(0));
  result.camera_imu_time_offset_s =
      state->_calib_dt_CAMtoIMU->value()(0);
  result.camera_imu_time_offset_online =
      state->_options.do_calib_camera_timeoffset;
  if (result.camera_imu_time_offset_online) {
    const Eigen::MatrixXd time_offset_covariance =
        ov_msckf::StateHelper::get_marginal_covariance(
            state, {state->_calib_dt_CAMtoIMU});
    if (time_offset_covariance.rows() == 1 &&
        time_offset_covariance.cols() == 1) {
      result.camera_imu_time_offset_variance_available = true;
      result.camera_imu_time_offset_variance_s2 =
          time_offset_covariance(0, 0);
    }
  }
  const Eigen::MatrixXd covariance =
      ov_msckf::StateHelper::get_marginal_covariance(state, {state->_imu});
  if (covariance.rows() == 15 && covariance.cols() == 15) {
    result.covariance_available = true;
    for (int i = 0; i < 15; ++i) {
      result.covariance_diagonal[static_cast<std::size_t>(i)] =
          covariance(i, i);
    }
  }
  result.initialized = true;
  result.processing_latency_ms = processing_latency_ms;
  if (!finite_state(result)) {
    throw std::runtime_error(
        "OpenVINS produced a nonfinite initialized state; stopping");
  }
  const TrackingHealthResult tracking_health =
      impl_->tracking_health_gate->update(
          result.timestamp, result.visual_support_features);
  result.visual_support_features =
      tracking_health.visual_support_features;
  result.tracking_health_good_duration_s =
      tracking_health.good_duration_s;
  result.tracking_health_bad_duration_s =
      tracking_health.bad_duration_s;
  result.tracking_health_status = tracking_health.status;
  result.tracking_health_gate_enabled =
      impl_->tracking_health_config.enabled;
  result.healthy = tracking_health.healthy;
  const double speed = std::sqrt(
      result.velocity_world_m_s.x * result.velocity_world_m_s.x +
      result.velocity_world_m_s.y * result.velocity_world_m_s.y +
      result.velocity_world_m_s.z * result.velocity_world_m_s.z);
  const double accel_bias_norm = std::sqrt(
      result.accel_bias_m_s2.x * result.accel_bias_m_s2.x +
      result.accel_bias_m_s2.y * result.accel_bias_m_s2.y +
      result.accel_bias_m_s2.z * result.accel_bias_m_s2.z);
  if (impl_->max_estimated_speed_m_s > 0.0 &&
      speed > impl_->max_estimated_speed_m_s) {
    throw std::runtime_error(
        "OpenVINS estimated speed exceeded configured limit: " +
        std::to_string(speed) + " m/s");
  }
  if (impl_->max_accel_bias_m_s2 > 0.0 &&
      accel_bias_norm > impl_->max_accel_bias_m_s2) {
    throw std::runtime_error(
        "OpenVINS accelerometer bias exceeded configured limit: " +
        std::to_string(accel_bias_norm) + " m/s^2");
  }
  return result;
}

} // namespace ovrs
