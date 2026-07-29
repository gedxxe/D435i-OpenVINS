#include "test_harness.hpp"

#include "ovrs/app_support.hpp"
#include "ovrs/bounded_queue.hpp"
#include "ovrs/capture_mode.hpp"
#include "ovrs/config.hpp"
#include "ovrs/imu_synchronizer.hpp"
#include "ovrs/measurement_dispatcher.hpp"
#include "ovrs/realsense_source.hpp"
#include "ovrs/stereo_synchronizer.hpp"
#include "ovrs/timestamp_normalizer.hpp"
#include "ovrs/tracking_health.hpp"
#include "ovrs/trajectory.hpp"
#include "ovrs/trajectory_view.hpp"
#include "ovrs/yaml_utils.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <thread>
#include <vector>

namespace {

ovrs::ImageFrame image(int id, double timestamp, std::uint64_t frameset) {
  ovrs::ImageFrame value;
  value.timestamp = timestamp;
  value.frameset_number = frameset;
  value.camera_id = id;
  value.width = 2;
  value.height = 2;
  value.stride_bytes = 2;
  value.format = "Y8";
  value.pixels = std::make_shared<std::vector<std::uint8_t>>(4, std::uint8_t{7});
  return value;
}

ovrs::TimedVec3 timed(double timestamp, double raw_timestamp_ms, ovrs::Vec3 value = {}) {
  ovrs::TimedVec3 sample;
  sample.timestamp = timestamp;
  sample.raw_timestamp_ms = raw_timestamp_ms;
  sample.value = value;
  return sample;
}

} // namespace

TEST_CASE("timestamp normalization and domain enforcement") {
  ovrs::TimestampNormalizer normalizer;
  auto first = normalizer.normalize("gyro", 1000.0, "hardware_clock");
  auto second = normalizer.normalize("gyro", 1005.0, "hardware_clock");
  REQUIRE(first.accepted);
  REQUIRE_NEAR(first.seconds, 0.0, 1e-12);
  REQUIRE_NEAR(second.seconds, 0.005, 1e-12);
  REQUIRE(!normalizer.normalize("gyro", 1005.0, "hardware_clock").accepted);
  REQUIRE(!normalizer.normalize("gyro", 1004.0, "hardware_clock").accepted);
  REQUIRE(!normalizer.normalize("camera", 1006.0, "system_time").accepted);
  REQUIRE(normalizer.rejected() == 3);
}

TEST_CASE("accelerometer interpolation at gyro time") {
  ovrs::ImuSynchronizer sync;
  REQUIRE(sync.add_accelerometer(timed(0.0, 0.0, {0.0, 2.0, 4.0})));
  REQUIRE(sync.add_gyroscope(timed(0.5, 500.0, {1.0, 2.0, 3.0})));
  REQUIRE(!sync.take_ready());
  REQUIRE(sync.add_accelerometer(timed(1.0, 1000.0, {2.0, 4.0, 6.0})));
  const auto sample = sync.take_ready();
  REQUIRE(sample.has_value());
  REQUIRE_NEAR(sample->linear_acceleration_m_s2.x, 1.0, 1e-12);
  REQUIRE_NEAR(sample->linear_acceleration_m_s2.y, 3.0, 1e-12);
  REQUIRE_NEAR(sample->linear_acceleration_m_s2.z, 5.0, 1e-12);
  REQUIRE_NEAR(sample->interpolation_delay_s, 0.5, 1e-12);
}

TEST_CASE("IMU missing brackets duplicates regression trimming and shutdown") {
  ovrs::ImuSynchronizer sync(2);
  REQUIRE(sync.add_gyroscope(timed(0.0, 0.0)));
  REQUIRE(sync.add_accelerometer(timed(1.0, 1000.0)));
  REQUIRE(sync.add_accelerometer(timed(2.0, 2000.0)));
  REQUIRE(sync.stats().missing_brackets == 1);
  REQUIRE(!sync.add_accelerometer(timed(2.0, 2000.0)));
  REQUIRE(!sync.add_accelerometer(timed(1.5, 1500.0)));
  REQUIRE(!sync.add_gyroscope(
      timed(2.5, 2500.0, {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0})));
  REQUIRE(sync.stats().invalid_values == 1);
  REQUIRE(sync.add_gyroscope(timed(3.0, 3000.0)));
  REQUIRE(sync.add_gyroscope(timed(4.0, 4000.0)));
  REQUIRE(sync.add_gyroscope(timed(5.0, 5000.0)));
  REQUIRE(sync.stats().dropped_capacity >= 1);
  sync.shutdown();
  REQUIRE(sync.stopped());
  REQUIRE(!sync.add_gyroscope(timed(6.0, 6000.0)));
}

TEST_CASE("bounded queue drops oldest and closes cleanly") {
  ovrs::BoundedQueue<int> queue(2);
  REQUIRE(queue.push(1));
  REQUIRE(queue.push(2));
  REQUIRE(queue.push(3));
  REQUIRE(queue.dropped() == 1);
  REQUIRE(queue.try_pop() == 2);
  queue.close();
  REQUIRE(!queue.push(4));
  REQUIRE(queue.wait_pop() == 3);
  REQUIRE(!queue.wait_pop());
}

TEST_CASE("stereo pairing enforces identifiers frameset format and tolerance") {
  ovrs::StereoSynchronizer sync(0.002);
  auto valid = sync.pair(image(0, 1.0, 7), image(1, 1.001, 7));
  REQUIRE(valid.has_value());
  REQUIRE_NEAR(valid->timestamp, 1.0005, 1e-12);
  REQUIRE(!sync.pair(image(0, 2.0, 8), image(1, 2.0, 9)));
  REQUIRE(!sync.pair(image(0, 3.0, 10), image(1, 3.01, 10)));
  auto bad = image(1, 4.0, 11);
  bad.format = "RGB8";
  REQUIRE(!sync.pair(image(0, 4.0, 11), bad));
  bad = image(1, std::numeric_limits<double>::quiet_NaN(), 12);
  REQUIRE(!sync.pair(image(0, 5.0, 12), bad));
  bad = image(1, 6.0, 13);
  bad.pixels->resize(1);
  REQUIRE(!sync.pair(image(0, 6.0, 13), bad));
  REQUIRE(sync.stats().accepted_pairs == 1);
  REQUIRE(sync.stats().rejected_pairs == 5);
}

TEST_CASE("configuration validation") {
  REQUIRE(ovrs::is_d435i_device_name("Intel RealSense D435I"));
  REQUIRE(ovrs::is_d435i_device_name("intel realsense d435i"));
  REQUIRE(!ovrs::is_d435i_device_name("Intel RealSense D455"));
  REQUIRE(!ovrs::is_d435i_device_name("unknown"));

  ovrs::StreamConfig config;
  REQUIRE(ovrs::validate(config).empty());
  config.width = 0;
  config.gyro_fps = 0;
  config.stereo_tolerance_ms = 50.0;
  REQUIRE(ovrs::validate(config).size() == 3);
  config = ovrs::StreamConfig{};
  config.gyro_sensitivity = 5;
  REQUIRE(ovrs::validate(config).size() == 1);
  config = ovrs::StreamConfig{};
  config.gyro_scale_factor = 0.0;
  REQUIRE(ovrs::validate(config).size() == 1);
}

TEST_CASE("capture modes have explicit non-ambiguous stream plans") {
  const auto names = ovrs::capture_mode_names();
  REQUIRE(names.size() == 4);
  REQUIRE(std::count(names.begin(), names.end(), "vio") == 1);
  REQUIRE(std::count(names.begin(), names.end(), "imu-allan") == 1);
  REQUIRE(std::count(names.begin(), names.end(), "stereo-calibration") == 1);
  REQUIRE(std::count(names.begin(), names.end(), "imu-camera-calibration") == 1);

  const auto vio = ovrs::capture_plan("vio");
  REQUIRE(vio.has_value());
  REQUIRE(vio->enable_stereo);
  REQUIRE(vio->enable_motion);
  REQUIRE(vio->write_synchronized_imu);
  REQUIRE(vio->replay_compatible);
  REQUIRE(!vio->requires_stationary_sensor);
  REQUIRE(!vio->requires_calibration_target);

  const auto allan = ovrs::capture_plan("imu-allan");
  REQUIRE(allan.has_value());
  REQUIRE(!allan->enable_stereo);
  REQUIRE(allan->enable_motion);
  REQUIRE(allan->write_synchronized_imu);
  REQUIRE(allan->requires_stationary_sensor);
  REQUIRE(!allan->requires_calibration_target);
  REQUIRE(!allan->replay_compatible);

  const auto stereo = ovrs::capture_plan("stereo-calibration");
  REQUIRE(stereo.has_value());
  REQUIRE(stereo->enable_stereo);
  REQUIRE(!stereo->enable_motion);
  REQUIRE(!stereo->write_synchronized_imu);
  REQUIRE(stereo->requires_calibration_target);
  REQUIRE(!stereo->replay_compatible);

  const auto imucam = ovrs::capture_plan("imu-camera-calibration");
  REQUIRE(imucam.has_value());
  REQUIRE(imucam->enable_stereo);
  REQUIRE(imucam->enable_motion);
  REQUIRE(imucam->requires_calibration_target);
  REQUIRE(!imucam->requires_stationary_sensor);
  REQUIRE(!imucam->replay_compatible);

  REQUIRE(!ovrs::capture_plan("allan"));
  REQUIRE(!ovrs::capture_plan(""));
}

TEST_CASE("stream configuration YAML is validated and applied") {
  ovrs::StreamConfig config;
  std::string error;
  REQUIRE(ovrs::apply_stream_config_yaml("width: 640\nheight: 480\ncamera_fps: 15\n"
                                         "gyro_fps: 100\naccelerometer_fps: 63\n"
                                         "gyro_sensitivity: 2\n"
                                         "gyro_scale_factor: 0.5\n"
                                         "emitter_enabled: true\nauto_exposure: false\n"
                                         "motion_correction_enabled: false\n"
                                         "global_time_enabled: false\n"
                                         "imu_queue_size: 64\nstereo_queue_size: 8\n"
                                         "stereo_tolerance_ms: 1.5\nserial: \"unit-1\"\n",
                                         &config, &error));
  REQUIRE(config.width == 640);
  REQUIRE(config.gyro_fps == 100);
  REQUIRE(config.gyro_sensitivity == 2);
  REQUIRE_NEAR(config.gyro_scale_factor, 0.5, 1e-12);
  REQUIRE(config.accel_fps == 63);
  REQUIRE(config.emitter_enabled);
  REQUIRE(!config.auto_exposure);
  REQUIRE(!config.motion_correction_enabled);
  REQUIRE(!config.global_time_enabled);
  REQUIRE(config.serial == "unit-1");
  REQUIRE_NEAR(config.stereo_tolerance_ms, 1.5, 1e-12);
  REQUIRE(!ovrs::apply_stream_config_yaml("auto_exposure: maybe\n", &config, &error));
  REQUIRE(!ovrs::apply_stream_config_yaml("width: 640pixels\n", &config, &error));
  REQUIRE(!ovrs::apply_stream_config_yaml("width: 640\nwidth: 848\n", &config, &error));
}

TEST_CASE("stream configuration CLI and serialization are strict") {
  const auto stream_options = ovrs::stream_cli_value_options();
  const std::vector<std::string> expected_options{"--serial",
                                                  "--width",
                                                  "--height",
                                                  "--camera-fps",
                                                  "--gyro-fps",
                                                  "--gyro-sensitivity",
                                                  "--gyro-scale-factor",
                                                  "--accel-fps",
                                                  "--emitter",
                                                  "--auto-exposure",
                                                  "--motion-correction",
                                                  "--global-time",
                                                  "--imu-queue",
                                                  "--stereo-queue",
                                                  "--stereo-tolerance-ms"};
  REQUIRE(stream_options.size() == expected_options.size());
  for (const auto &option : expected_options) {
    REQUIRE(std::count(stream_options.begin(), stream_options.end(), option) == 1);
  }

  ovrs::StreamConfig config;
  std::string error;
  char program[] = "test";
  char width[] = "--width";
  char width_value[] = "640";
  char serial[] = "--serial";
  char serial_value[] = "unit-1";
  char global_time[] = "--global-time";
  char global_time_value[] = "off";
  char gyro_sensitivity[] = "--gyro-sensitivity";
  char gyro_sensitivity_value[] = "1";
  char gyro_scale[] = "--gyro-scale-factor";
  char gyro_scale_value[] = "0.5";
  char *valid_argv[] = {
      program,          width,       width_value,
      serial,           serial_value,
      global_time,      global_time_value,
      gyro_sensitivity, gyro_sensitivity_value,
      gyro_scale,       gyro_scale_value};
  REQUIRE(ovrs::apply_stream_config_cli(11, valid_argv, &config, &error));
  REQUIRE(config.width == 640);
  REQUIRE(config.serial == "unit-1");
  REQUIRE(!config.global_time_enabled);
  REQUIRE(config.gyro_sensitivity == 1);
  REQUIRE_NEAR(config.gyro_scale_factor, 0.5, 1e-12);

  ovrs::StreamConfig round_trip;
  REQUIRE(ovrs::apply_stream_config_yaml(ovrs::stream_config_yaml(config), &round_trip, &error));
  REQUIRE(round_trip.width == config.width);
  REQUIRE(round_trip.gyro_fps == config.gyro_fps);
  REQUIRE(round_trip.accel_fps == config.accel_fps);
  REQUIRE(round_trip.gyro_sensitivity == config.gyro_sensitivity);
  REQUIRE_NEAR(round_trip.gyro_scale_factor, config.gyro_scale_factor, 1e-12);
  REQUIRE(round_trip.motion_correction_enabled == config.motion_correction_enabled);
  REQUIRE(round_trip.global_time_enabled == config.global_time_enabled);
  REQUIRE(round_trip.serial == config.serial);

  char *missing_argv[] = {program, width};
  REQUIRE(!ovrs::apply_stream_config_cli(2, missing_argv, &config, &error));
  char duplicate_width[] = "--width";
  char duplicate_value[] = "800";
  char *duplicate_argv[] = {program, width, width_value, duplicate_width, duplicate_value};
  REQUIRE(!ovrs::apply_stream_config_cli(5, duplicate_argv, &config, &error));
}

TEST_CASE("application option validation rejects ambiguity") {
  std::string error;
  char program[] = "test";
  char output[] = "--output";
  char output_value[] = "run";
  char *valid_argv[] = {program, output, output_value};
  REQUIRE(ovrs::validate_cli_arguments(3, valid_argv, {"--output"}, {}, &error));
  char unknown[] = "--outpt";
  char *unknown_argv[] = {program, unknown, output_value};
  REQUIRE(!ovrs::validate_cli_arguments(3, unknown_argv, {"--output"}, {}, &error));
  char *missing_argv[] = {program, output};
  REQUIRE(!ovrs::validate_cli_arguments(2, missing_argv, {"--output"}, {}, &error));
  char history[] = "--viewer-history";
  char history_value[] = "42";
  char *history_argv[] = {program, history, history_value};
  REQUIRE(ovrs::bounded_size_option(3, history_argv, "--viewer-history", 6000, 2, 1000000) == 42);
  char excessive_value[] = "1000001";
  char *excessive_argv[] = {program, history, excessive_value};
  bool rejected_excessive_history = false;
  try {
    (void)ovrs::bounded_size_option(3, excessive_argv, "--viewer-history", 6000, 2, 1000000);
  } catch (const std::exception &) {
    rejected_excessive_history = true;
  }
  REQUIRE(rejected_excessive_history);
}

TEST_CASE("interruptible wait observes the shutdown flag") {
  ovrs::stop_requested_flag() = 1;
  REQUIRE(!ovrs::wait_until_or_stop(std::chrono::steady_clock::now() + std::chrono::seconds(1)));
  ovrs::stop_requested_flag() = 0;
  REQUIRE(ovrs::wait_until_or_stop(std::chrono::steady_clock::now()));
}

TEST_CASE("calibration identity and dependency paths fail closed") {
  std::string error;
  REQUIRE(!ovrs::validate_calibration_identity("calibration_state: BOOTSTRAP_UNVERIFIED\n"
                                               "calibrated_serial: REPLACE_WITH_DEVICE_SERIAL\n",
                                               &error));
  REQUIRE(ovrs::validate_calibration_identity("calibration_state: BOOTSTRAP_UNVERIFIED\n"
                                              "calibrated_serial: 123456\n",
                                              &error));
  REQUIRE(!ovrs::validate_estimation_calibration_state("calibration_state: BOOTSTRAP_UNVERIFIED\n",
                                                       false, &error));
  REQUIRE(ovrs::validate_estimation_calibration_state("calibration_state: BOOTSTRAP_UNVERIFIED\n",
                                                      true, &error));
  REQUIRE(ovrs::validate_estimation_calibration_state("calibration_state: KALIBR_VERIFIED\n", false,
                                                      &error));
  REQUIRE(!ovrs::validate_calibration_identity("calibration_state: BOOTSTRAP_UNVERIFIED\n"
                                               "calibrated_serial: not-a-device\n",
                                               &error));
  REQUIRE(!ovrs::validate_calibration_identity("calibration_state: BOOTSTRAP_UNVERIFIED\n"
                                               "calibrated_serial: 123456\n"
                                               "calibrated_serial: 654321\n",
                                               &error));
  REQUIRE(ovrs::simple_yaml_scalar("serial:evil\n", "serial").empty());
  REQUIRE(ovrs::safe_relative_config_path("calibration/cam.yaml"));
  REQUIRE(!ovrs::safe_relative_config_path("../cam.yaml"));
  REQUIRE(!ovrs::safe_relative_config_path("/tmp/cam.yaml"));
  REQUIRE(!ovrs::safe_relative_config_path("C:\\temp\\cam.yaml"));

  const auto suffix =
      std::to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count());
  const auto root = std::filesystem::temp_directory_path() / ("ovrs_config_validation_" + suffix);
  std::filesystem::create_directories(root);
  const std::string main_yaml = "calibration_state: BOOTSTRAP_UNVERIFIED\n"
                                "calibrated_serial: 123456\n"
                                "max_estimated_speed_m_s: 3.0\n"
                                "max_accel_bias_m_s2: 2.0\n"
                                "relative_config_imu: imu.yaml\n"
                                "relative_config_imucam: cameras.yaml\n";
  REQUIRE(ovrs::write_text(root / "imu.yaml",
                           "%YAML:1.0\ncalibration_state: BOOTSTRAP_UNVERIFIED\n"
                           "calibrated_serial: 123456\n"
                           "imu0:\n  update_rate: 200\n"
                           "  realsense_motion_correction_enabled: true\n"
                           "  realsense_global_time_enabled: true\n"
                           "  model: kalibr\n"
                           "  accelerometer_noise_density: 0.01\n"
                           "  accelerometer_random_walk: 0.001\n"
                           "  gyroscope_noise_density: 0.001\n"
                           "  gyroscope_random_walk: 0.0001\n",
                           &error));
  REQUIRE(ovrs::write_text(root / "cameras.yaml",
                           "%YAML:1.0\ncalibration_state: BOOTSTRAP_UNVERIFIED\n"
                           "calibrated_serial: 123456\n"
                           "T_gyro_accel:\n"
                           "  - [1, 0, 0, 0]\n"
                           "cam0:\n  T_imu_cam:\n"
                           "    - [1, 0, 0, 0]\n"
                           "    - [0, 1, 0, 0]\n"
                           "    - [0, 0, 1, 0]\n"
                           "    - [0, 0, 0, 1]\n"
                           "  camera_model: pinhole\n"
                           "  realsense_distortion_model: Brown Conrady\n"
                           "  realsense_distortion_coeffs: [0, 0, 0, 0, 0]\n"
                           "  distortion_model: radtan\n"
                           "  distortion_coeffs: [0, 0, 0, 0]\n"
                           "  intrinsics: [400, 400, 424, 240]\n"
                           "  resolution: [848, 480]\n"
                           "  timeshift_cam_imu: 0.0\n"
                           "cam1:\n  T_imu_cam:\n"
                           "    - [1, 0, 0, -0.05]\n"
                           "    - [0, 1, 0, 0]\n"
                           "    - [0, 0, 1, 0]\n"
                           "    - [0, 0, 0, 1]\n"
                           "  camera_model: pinhole\n"
                           "  realsense_distortion_model: Brown Conrady\n"
                           "  realsense_distortion_coeffs: [0, 0, 0, 0, 0]\n"
                           "  distortion_model: radtan\n"
                           "  distortion_coeffs: [0, 0, 0, 0]\n"
                           "  intrinsics: [400, 400, 424, 240]\n"
                           "  resolution: [848, 480]\n"
                           "  timeshift_cam_imu: 0.0\n",
                           &error));
  REQUIRE(ovrs::validate_estimator_configuration(root / "main.yaml", main_yaml, &error));
  REQUIRE(!ovrs::validate_estimator_configuration(
      root / "main.yaml", main_yaml + "max_estimated_speed_m_s: 4.0\n", &error));
  auto missing_speed_limit_yaml = main_yaml;
  const std::string speed_limit_line = "max_estimated_speed_m_s: 3.0\n";
  missing_speed_limit_yaml.erase(missing_speed_limit_yaml.find(speed_limit_line),
                                 speed_limit_line.size());
  REQUIRE(!ovrs::validate_estimator_configuration(root / "main.yaml", missing_speed_limit_yaml,
                                                  &error));
  REQUIRE(ovrs::validate_camera_calibration_resolution(root / "main.yaml", main_yaml, 848, 480,
                                                       &error));
  REQUIRE(!ovrs::validate_camera_calibration_resolution(root / "main.yaml", main_yaml, 640, 480,
                                                        &error));
  int calibration_width = 0;
  int calibration_height = 0;
  REQUIRE(ovrs::parse_camera_resolution("[848, 480]", &calibration_width, &calibration_height));
  REQUIRE(calibration_width == 848);
  REQUIRE(calibration_height == 480);
  REQUIRE(!ovrs::parse_camera_resolution("[848, 480] trailing", &calibration_width,
                                         &calibration_height));
  REQUIRE(ovrs::validate_runtime_imu_rate(root / "main.yaml", main_yaml, "gyro_rate_hz: 200\n",
                                          &error));
  REQUIRE(!ovrs::validate_runtime_imu_rate(root / "main.yaml", main_yaml, "gyro_rate_hz: 400\n",
                                           &error));
  REQUIRE(ovrs::validate_runtime_sensor_policy(
      root / "main.yaml", main_yaml, "motion_correction_active: true\nglobal_time_active: true\n",
      "motion_correction_enabled: true\nglobal_time_enabled: true\n", &error));
  REQUIRE(ovrs::validate_runtime_sensor_policy(
      root / "main.yaml", main_yaml,
      "motion_correction_active: true\nglobal_time_active: true\n"
      "gyro_sensitivity_requested: 1\ngyro_sensitivity_available: true\n"
      "gyro_sensitivity_active: 1\n"
      "gyro_scale_factor_configured: 0.5\n"
      "gyro_scale_factor_applied: 0.5\n",
      "motion_correction_enabled: true\nglobal_time_enabled: true\n"
      "gyro_sensitivity: 1\ngyro_scale_factor: 0.5\n",
      &error));
  REQUIRE(!ovrs::validate_runtime_sensor_policy(
      root / "main.yaml", main_yaml,
      "motion_correction_active: true\nglobal_time_active: true\n"
      "gyro_sensitivity_requested: 1\ngyro_sensitivity_available: true\n"
      "gyro_sensitivity_active: 0\n"
      "gyro_scale_factor_configured: 0.5\n"
      "gyro_scale_factor_applied: 0.5\n",
      "motion_correction_enabled: true\nglobal_time_enabled: true\n"
      "gyro_sensitivity: 1\ngyro_scale_factor: 0.5\n",
      &error));
  REQUIRE(!ovrs::validate_runtime_sensor_policy(
      root / "main.yaml", main_yaml,
      "motion_correction_active: true\nglobal_time_active: true\n"
      "gyro_scale_factor_configured: 0.5\n"
      "gyro_scale_factor_applied: 1.0\n",
      "motion_correction_enabled: true\nglobal_time_enabled: true\n"
      "gyro_scale_factor: 0.5\n",
      &error));
  REQUIRE(!ovrs::validate_runtime_sensor_policy(
      root / "main.yaml", main_yaml, "motion_correction_active: false\nglobal_time_active: true\n",
      "motion_correction_enabled: true\nglobal_time_enabled: true\n", &error));
  REQUIRE(!ovrs::validate_runtime_sensor_policy(
      root / "main.yaml", main_yaml, "motion_correction_active: true\nglobal_time_active: false\n",
      "motion_correction_enabled: true\nglobal_time_enabled: true\n", &error));
  REQUIRE(!ovrs::validate_estimator_configuration(
      root / "main.yaml", main_yaml + "relative_config_imu: other.yaml\n", &error));
  REQUIRE(ovrs::write_text(root / "imu.yaml",
                           "%YAML:1.0\nimu0:\n  update_rate: 200\n  update_rate: 400\n", &error));
  REQUIRE(!ovrs::validate_runtime_imu_rate(root / "main.yaml", main_yaml, "gyro_rate_hz: 200\n",
                                           &error));
  REQUIRE(ovrs::write_text(root / "imu.yaml",
                           "%YAML:1.0\ncalibration_state: BOOTSTRAP_UNVERIFIED\n"
                           "calibrated_serial: 123456\n"
                           "imu0:\n  update_rate: 200\n"
                           "  realsense_motion_correction_enabled: true\n"
                           "  realsense_global_time_enabled: true\n"
                           "  model: kalibr\n"
                           "  accelerometer_noise_density: 0.01\n"
                           "  accelerometer_random_walk: 0.001\n"
                           "  gyroscope_noise_density: 0.001\n"
                           "  gyroscope_random_walk: 0.0001\n",
                           &error));
  const std::string mismatched_imu_yaml = "%YAML:1.0\ncalibration_state: KALIBR_VERIFIED\n"
                                          "calibrated_serial: 123456\n"
                                          "imu0:\n  update_rate: 200\n"
                                          "  realsense_motion_correction_enabled: true\n"
                                          "  realsense_global_time_enabled: true\n"
                                          "  model: kalibr\n"
                                          "  accelerometer_noise_density: 0.01\n"
                                          "  accelerometer_random_walk: 0.001\n"
                                          "  gyroscope_noise_density: 0.001\n"
                                          "  gyroscope_random_walk: 0.0001\n";
  REQUIRE(ovrs::write_text(root / "imu.yaml", mismatched_imu_yaml, &error));
  REQUIRE(!ovrs::validate_estimator_configuration(root / "main.yaml", main_yaml, &error));
  REQUIRE(ovrs::write_text(root / "imu.yaml",
                           "%YAML:1.0\ncalibration_state: BOOTSTRAP_UNVERIFIED\n"
                           "calibrated_serial: 123456\n"
                           "imu0:\n  update_rate: 200\n"
                           "  realsense_motion_correction_enabled: true\n"
                           "  realsense_global_time_enabled: true\n"
                           "  model: kalibr\n"
                           "  accelerometer_noise_density: 0.01\n"
                           "  accelerometer_random_walk: 0.001\n"
                           "  gyroscope_noise_density: 0.001\n"
                           "  gyroscope_random_walk: 0.0001\n",
                           &error));
  std::string incompatible_camera_yaml;
  {
    std::ifstream camera_input(root / "cameras.yaml");
    incompatible_camera_yaml.assign(std::istreambuf_iterator<char>(camera_input),
                                    std::istreambuf_iterator<char>());
  }
  REQUIRE(ovrs::validate_camera_calibration_geometry(incompatible_camera_yaml, &error));
  auto kalibr_candidate_yaml =
      std::string("camera_calibration_method: KALIBR_REPEATABILITY_CANDIDATE\n"
                  "kalibr_primary_camchain_sha256: "
                  "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n"
                  "kalibr_repeat_camchain_sha256: "
                  "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n"
                  "kalibr_repeatability_review_sha256: "
                  "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc\n") +
      incompatible_camera_yaml;
  for (const std::string factory_line : {"  realsense_distortion_model: Brown Conrady\n",
                                         "  realsense_distortion_coeffs: [0, 0, 0, 0, 0]\n"}) {
    for (int camera = 0; camera < 2; ++camera) {
      const auto position = kalibr_candidate_yaml.find(factory_line);
      REQUIRE(position != std::string::npos);
      kalibr_candidate_yaml.erase(position, factory_line.size());
    }
  }
  REQUIRE(ovrs::validate_bootstrap_camera_calibration(kalibr_candidate_yaml, &error));
  auto malformed_candidate_yaml = kalibr_candidate_yaml;
  const auto candidate_hash = malformed_candidate_yaml.find(std::string(64, 'a'));
  REQUIRE(candidate_hash != std::string::npos);
  malformed_candidate_yaml.replace(candidate_hash, 64, "not-a-sha256");
  REQUIRE(!ovrs::validate_bootstrap_camera_calibration(malformed_candidate_yaml, &error));
  auto invalid_rotation_yaml = incompatible_camera_yaml;
  const auto rotation_entry = invalid_rotation_yaml.find("    - [1, 0, 0, 0]\n");
  REQUIRE(rotation_entry != std::string::npos);
  invalid_rotation_yaml.replace(rotation_entry, std::string("    - [1, 0, 0, 0]\n").size(),
                                "    - [2, 0, 0, 0]\n");
  REQUIRE(!ovrs::validate_camera_calibration_geometry(invalid_rotation_yaml, &error));
  auto mismatched_time_yaml = incompatible_camera_yaml;
  const auto second_time = mismatched_time_yaml.rfind("timeshift_cam_imu: 0.0");
  REQUIRE(second_time != std::string::npos);
  mismatched_time_yaml.replace(second_time, std::string("timeshift_cam_imu: 0.0").size(),
                               "timeshift_cam_imu: 0.01");
  REQUIRE(!ovrs::validate_camera_calibration_geometry(mismatched_time_yaml, &error));
  const auto fifth_coefficient = incompatible_camera_yaml.find("[0, 0, 0, 0, 0]");
  REQUIRE(fifth_coefficient != std::string::npos);
  incompatible_camera_yaml.replace(fifth_coefficient, std::string("[0, 0, 0, 0, 0]").size(),
                                   "[0, 0, 0, 0, 0.1]");
  REQUIRE(!ovrs::validate_bootstrap_camera_calibration(incompatible_camera_yaml, &error));
  REQUIRE(ovrs::write_text(root / "cameras.yaml",
                           "%YAML:1.0\ncalibration_state: BOOTSTRAP_UNVERIFIED\n"
                           "calibrated_serial: wrong\n",
                           &error));
  REQUIRE(!ovrs::validate_estimator_configuration(root / "main.yaml", main_yaml, &error));
  std::error_code cleanup_error;
  std::filesystem::remove_all(root, cleanup_error);
}

TEST_CASE("finite state validation and trajectory serialization") {
  ovrs::EstimatorState state;
  state.timestamp = 1.25;
  state.position_world_m = {1.0, 2.0, 3.0};
  state.camera_imu_time_offset_s = -0.005;
  state.camera_imu_time_offset_variance_s2 = 1e-8;
  state.camera_imu_time_offset_online = true;
  state.camera_imu_time_offset_variance_available = true;
  state.msckf_candidate_features = 20;
  state.msckf_accepted_features = 15;
  state.msckf_acceptance_ratio = 0.75;
  state.msckf_update_age_s = 0.02;
  state.msckf_update_quality_available = true;
  state.initialized = true;
  state.healthy = true;
  REQUIRE(ovrs::finite_state(state));
  REQUIRE(ovrs::serialize_tum(state) == "1.250000000 1.000000000 2.000000000 3.000000000 "
                                        "0.000000000 0.000000000 0.000000000 1.000000000");
  state.position_world_m.x = std::numeric_limits<double>::infinity();
  REQUIRE(!ovrs::finite_state(state));
  state.position_world_m.x = 0.0;
  state.q_world_to_imu_xyzw = {0.0, 0.0, 0.0, 0.0};
  REQUIRE(!ovrs::finite_state(state));
  state.q_world_to_imu_xyzw = {0.0, 0.0, 0.0, 1.0};
  state.camera_imu_time_offset_variance_s2 = -1e-8;
  REQUIRE(!ovrs::finite_state(state));
  state.camera_imu_time_offset_variance_s2 = 1e-8;
  state.camera_imu_time_offset_s = std::numeric_limits<double>::infinity();
  REQUIRE(!ovrs::finite_state(state));
  state.camera_imu_time_offset_s = -0.005;
  state.msckf_acceptance_ratio = 1.1;
  REQUIRE(!ovrs::finite_state(state));
  state.msckf_acceptance_ratio = 0.75;
  state.msckf_accepted_features = 21;
  REQUIRE(!ovrs::finite_state(state));
}

TEST_CASE("tracking health gate uses time hysteresis and fails closed") {
  ovrs::TrackingHealthGateConfig config;
  config.minimum_visual_support_features = 12;
  config.degrade_after_s = 1.0;
  config.recover_after_s = 1.5;
  ovrs::TrackingHealthGate gate(config);

  auto health = gate.update(10.0, 0);
  REQUIRE(health.status == ovrs::TrackingHealthStatus::warming_up);
  REQUIRE(!health.healthy);
  health = gate.update(10.5, 12);
  REQUIRE(health.status == ovrs::TrackingHealthStatus::warming_up);
  REQUIRE_NEAR(health.good_duration_s, 0.5, 1e-12);
  health = gate.update(11.0, 12);
  REQUIRE(health.status == ovrs::TrackingHealthStatus::warming_up);
  health = gate.update(11.5, 12);
  REQUIRE(health.status == ovrs::TrackingHealthStatus::healthy);
  REQUIRE(health.healthy);

  health = gate.update(12.0, 11);
  REQUIRE(health.status == ovrs::TrackingHealthStatus::healthy);
  REQUIRE_NEAR(health.bad_duration_s, 0.5, 1e-12);
  health = gate.update(12.5, 11);
  REQUIRE(health.status == ovrs::TrackingHealthStatus::degraded);
  REQUIRE(!health.healthy);
  health = gate.update(13.0, 12);
  REQUIRE(health.status == ovrs::TrackingHealthStatus::degraded);
  health = gate.update(13.5, 12);
  REQUIRE(health.status == ovrs::TrackingHealthStatus::degraded);
  health = gate.update(14.0, 12);
  REQUIRE(health.status == ovrs::TrackingHealthStatus::healthy);

  health = gate.update(15.0, 12);
  REQUIRE(health.status == ovrs::TrackingHealthStatus::warming_up);
  REQUIRE(!health.healthy);

  config.enabled = false;
  ovrs::TrackingHealthGate disabled(config);
  health = disabled.update(1.0, 0);
  REQUIRE(health.status == ovrs::TrackingHealthStatus::disabled);
  REQUIRE(health.healthy);

  config.enabled = true;
  config.recover_after_s = 5.0;
  config.warmup_timeout_s = 2.0;
  ovrs::TrackingHealthGate flickering(config);
  REQUIRE(flickering.update(20.0, 12).status ==
          ovrs::TrackingHealthStatus::warming_up);
  REQUIRE(flickering.update(20.5, 11).status ==
          ovrs::TrackingHealthStatus::warming_up);
  REQUIRE(flickering.update(21.0, 12).status ==
          ovrs::TrackingHealthStatus::warming_up);
  REQUIRE(flickering.update(21.5, 11).status ==
          ovrs::TrackingHealthStatus::warming_up);
  health = flickering.update(22.0, 12);
  REQUIRE(health.status == ovrs::TrackingHealthStatus::degraded);
  REQUIRE(!health.healthy);
}

TEST_CASE("tracking health metadata and transition formatting are shared") {
  ovrs::TrackingHealthGateConfig config;
  config.minimum_visual_support_features = 19;
  config.degrade_after_s = 0.75;
  config.recover_after_s = 1.25;
  config.warmup_timeout_s = 2.5;
  const std::string metadata = ovrs::tracking_health_metadata_yaml(config);
  REQUIRE(metadata.find("tracking_health_gate_enabled: true\n") !=
          std::string::npos);
  REQUIRE(metadata.find(
              "tracking_health_min_visual_support_features: 19\n") !=
          std::string::npos);
  REQUIRE(metadata.find("tracking_health_degrade_after_s: 0.750000\n") !=
          std::string::npos);
  REQUIRE(metadata.find("tracking_health_recover_after_s: 1.250000\n") !=
          std::string::npos);
  REQUIRE(metadata.find("tracking_health_warmup_timeout_s: 2.500000\n") !=
          std::string::npos);

  ovrs::EstimatorState state;
  state.timestamp = 4.5;
  state.tracking_health_status = ovrs::TrackingHealthStatus::healthy;
  state.visual_support_features = 31;
  REQUIRE(ovrs::tracking_health_transition_message(state) ==
          "tracking_health_transition timestamp=4.500000 status=HEALTHY "
          "visual_support_features=31");
}

TEST_CASE("trajectory view supports deterministic orbit pan zoom and reset") {
  ovrs::TrajectoryViewController view;

  const auto projected_x = view.project({1.0, 0.0, 0.0});
  const auto projected_y = view.project({0.0, 1.0, 0.0});
  const auto projected_z = view.project({0.0, 0.0, 1.0});
  REQUIRE_NEAR(projected_x.x, 0.7071067811865475, 1e-12);
  REQUIRE_NEAR(projected_x.y, -0.4082482904638630, 1e-12);
  REQUIRE_NEAR(projected_y.x, -0.7071067811865475, 1e-12);
  REQUIRE_NEAR(projected_y.y, -0.4082482904638630, 1e-12);
  REQUIRE_NEAR(projected_z.x, 0.0, 1e-12);
  REQUIRE_NEAR(projected_z.y, 0.8164965809277260, 1e-12);

  const ovrs::ViewPoint2d origin = {0.0, 0.0};
  const ovrs::ViewPoint2d viewport_centre = {320.0, 240.0};
  const auto screen_x = view.screen_point(projected_x, origin, 100.0, viewport_centre);
  const auto screen_y = view.screen_point(projected_y, origin, 100.0, viewport_centre);
  const auto screen_z = view.screen_point(projected_z, origin, 100.0, viewport_centre);
  REQUIRE(screen_x.y > viewport_centre.y);
  REQUIRE(screen_y.y > viewport_centre.y);
  REQUIRE(screen_z.y < viewport_centre.y);

  view.orbit(20.0, -30.0);
  REQUIRE(view.yaw_rad() != ovrs::TrajectoryViewController::default_yaw_rad);
  REQUIRE(view.elevation_rad() != ovrs::TrajectoryViewController::default_elevation_rad);
  view.orbit(0.0, -100000.0);
  REQUIRE_NEAR(view.elevation_rad(), ovrs::TrajectoryViewController::maximum_elevation_rad, 1e-12);

  const ovrs::ViewPoint2d projected = {1.5, -0.75};
  const ovrs::ViewPoint2d projected_centre = {0.25, 0.5};
  const ovrs::ViewPoint2d interaction_viewport_centre = {320.0, 240.0};
  constexpr double base_scale = 90.0;
  const auto before_pan =
      view.screen_point(projected, projected_centre, base_scale, interaction_viewport_centre);
  view.pan(18.0, -11.0);
  const auto after_pan =
      view.screen_point(projected, projected_centre, base_scale, interaction_viewport_centre);
  REQUIRE_NEAR(after_pan.x - before_pan.x, 18.0, 1e-12);
  REQUIRE_NEAR(after_pan.y - before_pan.y, -11.0, 1e-12);

  view.zoom_at(1.0, after_pan, interaction_viewport_centre);
  const auto after_zoom =
      view.screen_point(projected, projected_centre, base_scale, interaction_viewport_centre);
  REQUIRE_NEAR(after_zoom.x, after_pan.x, 1e-12);
  REQUIRE_NEAR(after_zoom.y, after_pan.y, 1e-12);

  const double fitted_yaw = view.yaw_rad();
  const double fitted_elevation = view.elevation_rad();
  view.zoom_at(100000.0, interaction_viewport_centre, interaction_viewport_centre);
  REQUIRE_NEAR(view.zoom(), ovrs::TrajectoryViewController::maximum_zoom, 1e-12);
  view.fit();
  REQUIRE_NEAR(view.zoom(), 1.0, 1e-12);
  REQUIRE_NEAR(view.pan_x_pixels(), 0.0, 1e-12);
  REQUIRE_NEAR(view.pan_y_pixels(), 0.0, 1e-12);
  REQUIRE_NEAR(view.yaw_rad(), fitted_yaw, 1e-12);
  REQUIRE_NEAR(view.elevation_rad(), fitted_elevation, 1e-12);

  view.reset();
  REQUIRE_NEAR(view.yaw_rad(), ovrs::TrajectoryViewController::default_yaw_rad, 1e-12);
  REQUIRE_NEAR(view.elevation_rad(), ovrs::TrajectoryViewController::default_elevation_rad, 1e-12);
  REQUIRE_NEAR(view.zoom(), 1.0, 1e-12);
}

TEST_CASE("trajectory view framing stays world anchored and follows viewport shape") {
  ovrs::TrajectoryViewController view;
  ovrs::TrajectoryViewFrame frame;
  const ovrs::Vec3 minimum = {-1.0, -2.0, -0.5};
  const ovrs::Vec3 maximum = {3.0, 2.0, 0.5};
  frame.fit(minimum, maximum);

  REQUIRE_NEAR(frame.focus_world_m().x, 1.0, 1e-12);
  REQUIRE_NEAR(frame.focus_world_m().y, 0.0, 1e-12);
  REQUIRE_NEAR(frame.focus_world_m().z, 0.0, 1e-12);
  REQUIRE_NEAR(frame.world_span_m(), 1.15 * std::hypot(4.0, 4.0, 1.0), 1e-12);

  const auto landscape_focus = frame.screen_point(view, frame.focus_world_m(), 1280.0, 400.0);
  REQUIRE_NEAR(landscape_focus.x, 640.0, 1e-12);
  REQUIRE_NEAR(landscape_focus.y, 221.0, 1e-12);
  const auto portrait_focus = frame.screen_point(view, frame.focus_world_m(), 400.0, 900.0);
  REQUIRE_NEAR(portrait_focus.x, 200.0, 1e-12);
  REQUIRE_NEAR(portrait_focus.y, 471.0, 1e-12);

  const ovrs::Vec3 fixed_world_point = {2.0, 1.0, 0.25};
  const auto before = frame.screen_point(view, fixed_world_point, 1280.0, 400.0);
  const auto again = frame.screen_point(view, fixed_world_point, 1280.0, 400.0);
  REQUIRE_NEAR(again.x, before.x, 1e-12);
  REQUIRE_NEAR(again.y, before.y, 1e-12);
}

TEST_CASE("run writer rejects nonmonotonic state output") {
  const auto suffix =
      std::to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count());
  const auto root = std::filesystem::temp_directory_path() / ("ovrs_writer_validation_" + suffix);
  ovrs::RunWriter writer;
  std::string error;
  REQUIRE(writer.open(root, &error));
  ovrs::EstimatorState state;
  state.timestamp = 1.0;
  state.msckf_update_features = 17;
  state.msckf_candidate_features = 20;
  state.msckf_accepted_features = 17;
  state.msckf_acceptance_ratio = 0.85;
  state.msckf_update_age_s = 0.02;
  state.msckf_update_quality_available = true;
  state.slam_features = 8;
  state.visual_support_features = 25;
  state.tracking_health_good_duration_s = 1.5;
  state.tracking_health_status = ovrs::TrackingHealthStatus::healthy;
  state.tracking_health_gate_enabled = true;
  state.camera_imu_time_offset_s = -0.005;
  state.camera_imu_time_offset_variance_s2 = 4e-8;
  state.camera_imu_time_offset_online = true;
  state.camera_imu_time_offset_variance_available = true;
  state.initialized = true;
  state.healthy = true;
  REQUIRE(writer.write_state(state, &error));
  REQUIRE(!writer.write_state(state, &error));
  REQUIRE(std::filesystem::exists(root / "INCOMPLETE"));
  REQUIRE(writer.finalize(&error));
  REQUIRE(!std::filesystem::exists(root / "INCOMPLETE"));
  REQUIRE(!writer.finalize(&error));
  std::ifstream state_input(root / "state.csv");
  const std::string state_text{std::istreambuf_iterator<char>(state_input),
                               std::istreambuf_iterator<char>()};
  REQUIRE(state_text.find("camera_imu_time_offset_s,"
                          "camera_imu_time_offset_std_s,"
                          "camera_imu_time_offset_online") != std::string::npos);
  REQUIRE(state_text.find(
              "msckf_update_features,msckf_candidate_features,"
              "msckf_accepted_features,msckf_acceptance_ratio,"
              "msckf_update_age_s,slam_features,"
              "visual_support_features,tracking_health_status") !=
          std::string::npos);
  REQUIRE(state_text.find(",17,20,17,0.84999999999999998,"
                          "0.02,8,25,HEALTHY,1.5,0,1,"
                          "-0.0050000000000000001,") != std::string::npos);
  REQUIRE(state_text.find("-0.0050000000000000001,"
                          "0.00020000000000000001,1,1,1") != std::string::npos);
  std::ifstream log_input(root / "application.log");
  const std::string log_text{std::istreambuf_iterator<char>(log_input),
                             std::istreambuf_iterator<char>()};
  REQUIRE(log_text.find("summary.final_camera_imu_time_offset_s="
                        "-0.0050000000000000001") != std::string::npos);
  REQUIRE(log_text.find("summary.final_camera_imu_time_offset_std_s="
                        "0.00020000000000000001") != std::string::npos);
  REQUIRE(log_text.find("summary.camera_imu_time_offset_online=true") != std::string::npos);
  REQUIRE(log_text.find("summary.unhealthy_state_count=0") !=
          std::string::npos);
  REQUIRE(log_text.find("summary.final_tracking_health_status=HEALTHY") !=
          std::string::npos);
  ovrs::RunWriter partial_writer;
  const auto partial_root = root / "partial";
  REQUIRE(partial_writer.open(partial_root, &error));
  REQUIRE(!partial_writer.finalize(&error));
  REQUIRE(partial_writer.close(&error));
  REQUIRE(std::filesystem::exists(partial_root / "INCOMPLETE"));
  std::error_code cleanup_error;
  std::filesystem::remove_all(root, cleanup_error);
}

TEST_CASE("run writer serializes concurrent state and diagnostics writes") {
  const auto suffix =
      std::to_string(std::chrono::high_resolution_clock::now().time_since_epoch().count());
  const auto root = std::filesystem::temp_directory_path() / ("ovrs_writer_concurrency_" + suffix);
  ovrs::RunWriter writer;
  std::string error;
  REQUIRE(writer.open(root, &error));
  std::atomic<bool> diagnostics_ok{true};
  std::thread diagnostics_thread([&] {
    for (int index = 0; index < 200; ++index) {
      ovrs::DiagnosticsSnapshot snapshot;
      snapshot.timestamp = index * 0.01;
      std::string write_error;
      if (!writer.write_diagnostics(snapshot, &write_error)) {
        diagnostics_ok.store(false);
        return;
      }
    }
  });
  bool states_ok = true;
  for (int index = 0; index < 200; ++index) {
    ovrs::EstimatorState state;
    state.timestamp = 1.0 + index * 0.01;
    state.initialized = true;
    state.healthy = true;
    if (!writer.write_state(state, &error)) {
      states_ok = false;
      break;
    }
  }
  diagnostics_thread.join();
  REQUIRE(states_ok);
  REQUIRE(diagnostics_ok.load());
  REQUIRE(writer.finalize(&error));
  std::error_code cleanup_error;
  std::filesystem::remove_all(root, cleanup_error);
}

TEST_CASE("ordered dispatcher waits for IMU coverage and shuts down") {
  std::vector<double> order;
  ovrs::MeasurementDispatcher dispatcher(
      8, 2, [&](const ovrs::ImuSample &sample) { order.push_back(sample.timestamp); },
      [&](const ovrs::StereoFrame &frame) { order.push_back(100.0 + frame.timestamp); });
  dispatcher.start();
  ovrs::StereoFrame stereo;
  stereo.timestamp = 0.015;
  REQUIRE(dispatcher.push_stereo(stereo));
  ovrs::ImuSample before;
  before.timestamp = 0.01;
  ovrs::ImuSample after;
  after.timestamp = 0.02;
  REQUIRE(dispatcher.push_imu(before));
  REQUIRE(dispatcher.push_imu(after));
  dispatcher.stop();
  REQUIRE(order.size() == 3);
  REQUIRE_NEAR(order[0], 0.01, 1e-12);
  REQUIRE_NEAR(order[1], 0.02, 1e-12);
  REQUIRE_NEAR(order[2], 100.015, 1e-12);
  REQUIRE(dispatcher.stats().dispatched_stereo == 1);
}

TEST_CASE("dispatcher rejects an image without an earlier IMU sample") {
  std::size_t camera_count = 0;
  ovrs::MeasurementDispatcher dispatcher(
      4, 2, [](const ovrs::ImuSample &) {}, [&](const ovrs::StereoFrame &) { ++camera_count; });
  dispatcher.start();
  ovrs::StereoFrame frame;
  frame.timestamp = 0.01;
  REQUIRE(dispatcher.push_stereo(frame));
  ovrs::ImuSample after;
  after.timestamp = 0.02;
  REQUIRE(dispatcher.push_imu(after));
  dispatcher.stop();
  REQUIRE(camera_count == 0);
  REQUIRE(dispatcher.stats().stereo_without_imu_coverage == 1);
}

TEST_CASE("dispatcher reuses an existing IMU bracket for close images") {
  std::vector<double> camera_times;
  ovrs::MeasurementDispatcher dispatcher(
      8, 4, [](const ovrs::ImuSample &) {},
      [&](const ovrs::StereoFrame &frame) { camera_times.push_back(frame.timestamp); });
  dispatcher.start();
  ovrs::StereoFrame first;
  first.timestamp = 0.015;
  ovrs::StereoFrame second;
  second.timestamp = 0.018;
  REQUIRE(dispatcher.push_stereo(first));
  REQUIRE(dispatcher.push_stereo(second));
  ovrs::ImuSample before;
  before.timestamp = 0.01;
  ovrs::ImuSample after;
  after.timestamp = 0.02;
  REQUIRE(dispatcher.push_imu(before));
  REQUIRE(dispatcher.push_imu(after));
  dispatcher.stop();
  REQUIRE(camera_times.size() == 2);
  REQUIRE_NEAR(camera_times[0], 0.015, 1e-12);
  REQUIRE_NEAR(camera_times[1], 0.018, 1e-12);
  REQUIRE(dispatcher.stats().stereo_without_imu_coverage == 0);
}

int main() { return test::run(); }
