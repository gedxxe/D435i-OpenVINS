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
  explicit Impl(const std::string &path) {
    auto parser = std::make_shared<ov_core::YamlParser>(path);
    std::string verbosity = "INFO";
    parser->parse_config("verbosity", verbosity);
    ov_core::Printer::setPrintLevel(verbosity);
    parser->parse_config("max_estimated_speed_m_s", max_estimated_speed_m_s,
                        false);
    parser->parse_config("max_accel_bias_m_s2", max_accel_bias_m_s2, false);

    ov_msckf::VioManagerOptions options;
    options.print_and_load(parser);
    if (!parser->successful()) {
      throw std::runtime_error("OpenVINS configuration is incomplete: " +
                               path);
    }
    if (options.state_options.num_cameras != 2 || !options.use_stereo) {
      throw std::runtime_error(
          "OpenVINS configuration must set max_cameras: 2 and "
          "use_stereo: true");
    }
    manager = std::make_shared<ov_msckf::VioManager>(options);
  }

  std::shared_ptr<ov_msckf::VioManager> manager;
  double max_estimated_speed_m_s = 0.0;
  double max_accel_bias_m_s2 = 0.0;
};

OpenVinsEstimator::OpenVinsEstimator(const std::string &configuration_path)
    : impl_(std::make_unique<Impl>(configuration_path)) {}
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
  result.healthy = finite_state(result);
  if (!result.healthy) {
    throw std::runtime_error(
        "OpenVINS produced a nonfinite initialized state; stopping");
  }
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
