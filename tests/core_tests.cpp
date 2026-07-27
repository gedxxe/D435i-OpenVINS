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
#include "ovrs/trajectory.hpp"

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
  value.pixels =
      std::make_shared<std::vector<std::uint8_t>>(4, std::uint8_t{7});
  return value;
}

ovrs::TimedVec3 timed(double timestamp, double raw_timestamp_ms,
                     ovrs::Vec3 value = {}) {
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
  REQUIRE(
      !normalizer.normalize("camera", 1006.0, "system_time").accepted);
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
      timed(2.5, 2500.0,
            {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0})));
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
}

TEST_CASE("capture modes have explicit non-ambiguous stream plans") {
  const auto names = ovrs::capture_mode_names();
  REQUIRE(names.size() == 4);
  REQUIRE(std::count(names.begin(), names.end(), "vio") == 1);
  REQUIRE(std::count(names.begin(), names.end(), "imu-allan") == 1);
  REQUIRE(std::count(names.begin(), names.end(), "stereo-calibration") == 1);
  REQUIRE(std::count(names.begin(), names.end(),
                     "imu-camera-calibration") == 1);

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
  REQUIRE(ovrs::apply_stream_config_yaml(
      "width: 640\nheight: 480\ncamera_fps: 15\n"
      "gyro_fps: 100\naccelerometer_fps: 63\n"
      "emitter_enabled: true\nauto_exposure: false\n"
      "motion_correction_enabled: false\n"
      "imu_queue_size: 64\nstereo_queue_size: 8\n"
      "stereo_tolerance_ms: 1.5\nserial: \"unit-1\"\n",
      &config, &error));
  REQUIRE(config.width == 640);
  REQUIRE(config.gyro_fps == 100);
  REQUIRE(config.accel_fps == 63);
  REQUIRE(config.emitter_enabled);
  REQUIRE(!config.auto_exposure);
  REQUIRE(!config.motion_correction_enabled);
  REQUIRE(config.serial == "unit-1");
  REQUIRE_NEAR(config.stereo_tolerance_ms, 1.5, 1e-12);
  REQUIRE(!ovrs::apply_stream_config_yaml("auto_exposure: maybe\n",
                                          &config, &error));
  REQUIRE(!ovrs::apply_stream_config_yaml("width: 640pixels\n",
                                          &config, &error));
  REQUIRE(!ovrs::apply_stream_config_yaml("width: 640\nwidth: 848\n",
                                          &config, &error));
}

TEST_CASE("stream configuration CLI and serialization are strict") {
  const auto stream_options = ovrs::stream_cli_value_options();
  const std::vector<std::string> expected_options{
      "--serial",        "--width",          "--height",
      "--camera-fps",    "--gyro-fps",       "--accel-fps",
      "--emitter",       "--auto-exposure",  "--motion-correction",
      "--imu-queue",     "--stereo-queue",   "--stereo-tolerance-ms"};
  REQUIRE(stream_options.size() == expected_options.size());
  for (const auto &option : expected_options) {
    REQUIRE(std::count(stream_options.begin(), stream_options.end(), option) ==
            1);
  }

  ovrs::StreamConfig config;
  std::string error;
  char program[] = "test";
  char width[] = "--width";
  char width_value[] = "640";
  char serial[] = "--serial";
  char serial_value[] = "unit-1";
  char *valid_argv[] = {program, width, width_value, serial, serial_value};
  REQUIRE(ovrs::apply_stream_config_cli(5, valid_argv, &config, &error));
  REQUIRE(config.width == 640);
  REQUIRE(config.serial == "unit-1");

  ovrs::StreamConfig round_trip;
  REQUIRE(ovrs::apply_stream_config_yaml(ovrs::stream_config_yaml(config),
                                         &round_trip, &error));
  REQUIRE(round_trip.width == config.width);
  REQUIRE(round_trip.gyro_fps == config.gyro_fps);
  REQUIRE(round_trip.accel_fps == config.accel_fps);
  REQUIRE(round_trip.motion_correction_enabled ==
          config.motion_correction_enabled);
  REQUIRE(round_trip.serial == config.serial);

  char *missing_argv[] = {program, width};
  REQUIRE(!ovrs::apply_stream_config_cli(2, missing_argv, &config, &error));
  char duplicate_width[] = "--width";
  char duplicate_value[] = "800";
  char *duplicate_argv[] = {program, width, width_value, duplicate_width,
                            duplicate_value};
  REQUIRE(!ovrs::apply_stream_config_cli(5, duplicate_argv, &config, &error));
}

TEST_CASE("application option validation rejects ambiguity") {
  std::string error;
  char program[] = "test";
  char output[] = "--output";
  char output_value[] = "run";
  char *valid_argv[] = {program, output, output_value};
  REQUIRE(ovrs::validate_cli_arguments(3, valid_argv, {"--output"}, {},
                                       &error));
  char unknown[] = "--outpt";
  char *unknown_argv[] = {program, unknown, output_value};
  REQUIRE(!ovrs::validate_cli_arguments(3, unknown_argv, {"--output"}, {},
                                        &error));
  char *missing_argv[] = {program, output};
  REQUIRE(!ovrs::validate_cli_arguments(2, missing_argv, {"--output"}, {},
                                        &error));
  char history[] = "--viewer-history";
  char history_value[] = "42";
  char *history_argv[] = {program, history, history_value};
  REQUIRE(ovrs::bounded_size_option(
              3, history_argv, "--viewer-history", 6000, 2, 1000000) ==
          42);
  char excessive_value[] = "1000001";
  char *excessive_argv[] = {program, history, excessive_value};
  bool rejected_excessive_history = false;
  try {
    (void)ovrs::bounded_size_option(
        3, excessive_argv, "--viewer-history", 6000, 2, 1000000);
  } catch (const std::exception &) {
    rejected_excessive_history = true;
  }
  REQUIRE(rejected_excessive_history);
}

TEST_CASE("interruptible wait observes the shutdown flag") {
  ovrs::stop_requested_flag() = 1;
  REQUIRE(!ovrs::wait_until_or_stop(std::chrono::steady_clock::now() +
                                    std::chrono::seconds(1)));
  ovrs::stop_requested_flag() = 0;
  REQUIRE(ovrs::wait_until_or_stop(std::chrono::steady_clock::now()));
}

TEST_CASE("calibration identity and dependency paths fail closed") {
  std::string error;
  REQUIRE(!ovrs::validate_calibration_identity(
      "calibration_state: BOOTSTRAP_UNVERIFIED\n"
      "calibrated_serial: REPLACE_WITH_DEVICE_SERIAL\n",
      &error));
  REQUIRE(ovrs::validate_calibration_identity(
      "calibration_state: BOOTSTRAP_UNVERIFIED\n"
      "calibrated_serial: 123456\n",
      &error));
  REQUIRE(!ovrs::validate_estimation_calibration_state(
      "calibration_state: BOOTSTRAP_UNVERIFIED\n", false, &error));
  REQUIRE(ovrs::validate_estimation_calibration_state(
      "calibration_state: BOOTSTRAP_UNVERIFIED\n", true, &error));
  REQUIRE(ovrs::validate_estimation_calibration_state(
      "calibration_state: KALIBR_VERIFIED\n", false, &error));
  REQUIRE(!ovrs::validate_calibration_identity(
      "calibration_state: BOOTSTRAP_UNVERIFIED\n"
      "calibrated_serial: not-a-device\n",
      &error));
  REQUIRE(!ovrs::validate_calibration_identity(
      "calibration_state: BOOTSTRAP_UNVERIFIED\n"
      "calibrated_serial: 123456\n"
      "calibrated_serial: 654321\n",
      &error));
  REQUIRE(ovrs::simple_yaml_scalar("serial:evil\n", "serial").empty());
  REQUIRE(ovrs::safe_relative_config_path("calibration/cam.yaml"));
  REQUIRE(!ovrs::safe_relative_config_path("../cam.yaml"));
  REQUIRE(!ovrs::safe_relative_config_path("/tmp/cam.yaml"));
  REQUIRE(!ovrs::safe_relative_config_path("C:\\temp\\cam.yaml"));

  const auto suffix = std::to_string(
      std::chrono::high_resolution_clock::now().time_since_epoch().count());
  const auto root = std::filesystem::temp_directory_path() /
                    ("ovrs_config_validation_" + suffix);
  std::filesystem::create_directories(root);
  const std::string main_yaml =
      "calibration_state: BOOTSTRAP_UNVERIFIED\n"
      "calibrated_serial: 123456\n"
      "relative_config_imu: imu.yaml\n"
      "relative_config_imucam: cameras.yaml\n";
  REQUIRE(ovrs::write_text(
      root / "imu.yaml",
      "%YAML:1.0\ncalibration_state: BOOTSTRAP_UNVERIFIED\n"
      "calibrated_serial: 123456\n"
      "imu0:\n  update_rate: 200\n"
      "  realsense_motion_correction_enabled: true\n"
      "  model: kalibr\n"
      "  accelerometer_noise_density: 0.01\n"
      "  accelerometer_random_walk: 0.001\n"
      "  gyroscope_noise_density: 0.001\n"
      "  gyroscope_random_walk: 0.0001\n",
      &error));
  REQUIRE(ovrs::write_text(
      root / "cameras.yaml",
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
  REQUIRE(ovrs::validate_estimator_configuration(root / "main.yaml",
                                                  main_yaml, &error));
  REQUIRE(ovrs::validate_camera_calibration_resolution(
      root / "main.yaml", main_yaml, 848, 480, &error));
  REQUIRE(!ovrs::validate_camera_calibration_resolution(
      root / "main.yaml", main_yaml, 640, 480, &error));
  int calibration_width = 0;
  int calibration_height = 0;
  REQUIRE(ovrs::parse_camera_resolution(
      "[848, 480]", &calibration_width, &calibration_height));
  REQUIRE(calibration_width == 848);
  REQUIRE(calibration_height == 480);
  REQUIRE(!ovrs::parse_camera_resolution(
      "[848, 480] trailing", &calibration_width, &calibration_height));
  REQUIRE(ovrs::validate_runtime_imu_rate(
      root / "main.yaml", main_yaml, "gyro_rate_hz: 200\n", &error));
  REQUIRE(!ovrs::validate_runtime_imu_rate(
      root / "main.yaml", main_yaml, "gyro_rate_hz: 400\n", &error));
  REQUIRE(ovrs::validate_runtime_motion_correction(
      root / "main.yaml", main_yaml,
      "motion_correction_active: true\n",
      "motion_correction_enabled: true\n", &error));
  REQUIRE(!ovrs::validate_runtime_motion_correction(
      root / "main.yaml", main_yaml,
      "motion_correction_active: false\n",
      "motion_correction_enabled: true\n", &error));
  REQUIRE(!ovrs::validate_estimator_configuration(
      root / "main.yaml",
      main_yaml + "relative_config_imu: other.yaml\n", &error));
  REQUIRE(ovrs::write_text(
      root / "imu.yaml",
      "%YAML:1.0\nimu0:\n  update_rate: 200\n  update_rate: 400\n",
      &error));
  REQUIRE(!ovrs::validate_runtime_imu_rate(
      root / "main.yaml", main_yaml, "gyro_rate_hz: 200\n", &error));
  REQUIRE(ovrs::write_text(
      root / "imu.yaml",
      "%YAML:1.0\ncalibration_state: BOOTSTRAP_UNVERIFIED\n"
      "calibrated_serial: 123456\n"
      "imu0:\n  update_rate: 200\n"
      "  realsense_motion_correction_enabled: true\n"
      "  model: kalibr\n"
      "  accelerometer_noise_density: 0.01\n"
      "  accelerometer_random_walk: 0.001\n"
      "  gyroscope_noise_density: 0.001\n"
      "  gyroscope_random_walk: 0.0001\n",
      &error));
  const std::string mismatched_imu_yaml =
      "%YAML:1.0\ncalibration_state: KALIBR_VERIFIED\n"
      "calibrated_serial: 123456\n"
      "imu0:\n  update_rate: 200\n"
      "  realsense_motion_correction_enabled: true\n"
      "  model: kalibr\n"
      "  accelerometer_noise_density: 0.01\n"
      "  accelerometer_random_walk: 0.001\n"
      "  gyroscope_noise_density: 0.001\n"
      "  gyroscope_random_walk: 0.0001\n";
  REQUIRE(ovrs::write_text(root / "imu.yaml", mismatched_imu_yaml, &error));
  REQUIRE(!ovrs::validate_estimator_configuration(root / "main.yaml",
                                                   main_yaml, &error));
  REQUIRE(ovrs::write_text(
      root / "imu.yaml",
      "%YAML:1.0\ncalibration_state: BOOTSTRAP_UNVERIFIED\n"
      "calibrated_serial: 123456\n"
      "imu0:\n  update_rate: 200\n"
      "  realsense_motion_correction_enabled: true\n"
      "  model: kalibr\n"
      "  accelerometer_noise_density: 0.01\n"
      "  accelerometer_random_walk: 0.001\n"
      "  gyroscope_noise_density: 0.001\n"
      "  gyroscope_random_walk: 0.0001\n",
      &error));
  std::string incompatible_camera_yaml;
  {
    std::ifstream camera_input(root / "cameras.yaml");
    incompatible_camera_yaml.assign(
        std::istreambuf_iterator<char>(camera_input),
        std::istreambuf_iterator<char>());
  }
  REQUIRE(ovrs::validate_camera_calibration_geometry(
      incompatible_camera_yaml, &error));
  auto invalid_rotation_yaml = incompatible_camera_yaml;
  const auto rotation_entry = invalid_rotation_yaml.find(
      "    - [1, 0, 0, 0]\n");
  REQUIRE(rotation_entry != std::string::npos);
  invalid_rotation_yaml.replace(
      rotation_entry, std::string("    - [1, 0, 0, 0]\n").size(),
      "    - [2, 0, 0, 0]\n");
  REQUIRE(!ovrs::validate_camera_calibration_geometry(
      invalid_rotation_yaml, &error));
  auto mismatched_time_yaml = incompatible_camera_yaml;
  const auto second_time =
      mismatched_time_yaml.rfind("timeshift_cam_imu: 0.0");
  REQUIRE(second_time != std::string::npos);
  mismatched_time_yaml.replace(
      second_time, std::string("timeshift_cam_imu: 0.0").size(),
      "timeshift_cam_imu: 0.01");
  REQUIRE(!ovrs::validate_camera_calibration_geometry(
      mismatched_time_yaml, &error));
  const auto fifth_coefficient =
      incompatible_camera_yaml.find("[0, 0, 0, 0, 0]");
  REQUIRE(fifth_coefficient != std::string::npos);
  incompatible_camera_yaml.replace(fifth_coefficient,
                                   std::string("[0, 0, 0, 0, 0]").size(),
                                   "[0, 0, 0, 0, 0.1]");
  REQUIRE(!ovrs::validate_bootstrap_camera_calibration(
      incompatible_camera_yaml, &error));
  REQUIRE(ovrs::write_text(
      root / "cameras.yaml",
      "%YAML:1.0\ncalibration_state: BOOTSTRAP_UNVERIFIED\n"
      "calibrated_serial: wrong\n",
      &error));
  REQUIRE(!ovrs::validate_estimator_configuration(root / "main.yaml",
                                                   main_yaml, &error));
  std::error_code cleanup_error;
  std::filesystem::remove_all(root, cleanup_error);
}

TEST_CASE("finite state validation and trajectory serialization") {
  ovrs::EstimatorState state;
  state.timestamp = 1.25;
  state.position_world_m = {1.0, 2.0, 3.0};
  state.initialized = true;
  state.healthy = true;
  REQUIRE(ovrs::finite_state(state));
  REQUIRE(ovrs::serialize_tum(state) ==
          "1.250000000 1.000000000 2.000000000 3.000000000 "
          "0.000000000 0.000000000 0.000000000 1.000000000");
  state.position_world_m.x = std::numeric_limits<double>::infinity();
  REQUIRE(!ovrs::finite_state(state));
  state.position_world_m.x = 0.0;
  state.q_world_to_imu_xyzw = {0.0, 0.0, 0.0, 0.0};
  REQUIRE(!ovrs::finite_state(state));
}

TEST_CASE("run writer rejects nonmonotonic state output") {
  const auto suffix = std::to_string(
      std::chrono::high_resolution_clock::now().time_since_epoch().count());
  const auto root = std::filesystem::temp_directory_path() /
                    ("ovrs_writer_validation_" + suffix);
  ovrs::RunWriter writer;
  std::string error;
  REQUIRE(writer.open(root, &error));
  ovrs::EstimatorState state;
  state.timestamp = 1.0;
  state.initialized = true;
  state.healthy = true;
  REQUIRE(writer.write_state(state, &error));
  REQUIRE(!writer.write_state(state, &error));
  REQUIRE(std::filesystem::exists(root / "INCOMPLETE"));
  REQUIRE(writer.finalize(&error));
  REQUIRE(!std::filesystem::exists(root / "INCOMPLETE"));
  REQUIRE(!writer.finalize(&error));
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
  const auto suffix = std::to_string(
      std::chrono::high_resolution_clock::now().time_since_epoch().count());
  const auto root = std::filesystem::temp_directory_path() /
                    ("ovrs_writer_concurrency_" + suffix);
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
      8, 2,
      [&](const ovrs::ImuSample &sample) { order.push_back(sample.timestamp); },
      [&](const ovrs::StereoFrame &frame) {
        order.push_back(100.0 + frame.timestamp);
      });
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
      4, 2, [](const ovrs::ImuSample &) {},
      [&](const ovrs::StereoFrame &) { ++camera_count; });
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
      [&](const ovrs::StereoFrame &frame) {
        camera_times.push_back(frame.timestamp);
      });
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
