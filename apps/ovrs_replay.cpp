#include "ovrs/app_support.hpp"
#include "ovrs/config.hpp"
#include "ovrs/live_viewer.hpp"
#include "ovrs/openvins_estimator.hpp"
#include "ovrs/trajectory.hpp"
#include "ovrs/types.hpp"
#include "ovrs/version.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifdef OVRS_HAS_OPENVINS
#include <opencv2/imgcodecs.hpp>
#endif

namespace {
void help() {
  std::cout << "Usage: ovrs_replay --dataset DATASET_DIR --config "
               "ESTIMATOR.yaml\n"
               "                   [--rate 0|FACTOR] [--output RUN_DIR]\n"
               "                   [--viewer] [--viewer-history COUNT]\n"
               "                   [--allow-unverified-calibration]\n"
               "                   [--version]\n"
               "A rate of 0 replays as fast as possible; 1 is real time. "
               "--viewer shows the recorded stereo feed and live X-Y "
               "estimate.\n"
               "Estimation requires KALIBR_VERIFIED unless the explicit "
               "diagnostic override is supplied.\n"
               "The camera serial is read from the dataset and must match "
               "the calibration; --serial is not accepted.\n";
}

#ifdef OVRS_HAS_OPENVINS
std::vector<std::string> split(const std::string &line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}
#endif
} // namespace

int main(int argc, char **argv) {
  if (ovrs::has_flag(argc, argv, "--version")) {
    std::cout << ovrs::version_summary("ovrs_replay");
    return 0;
  }
  if (ovrs::has_flag(argc, argv, "--help") ||
      ovrs::has_flag(argc, argv, "-h")) {
    help();
    return 0;
  }
  if (ovrs::has_flag(argc, argv, "--serial")) {
    std::cerr
        << "--serial is not accepted by replay. Replay reads the camera "
           "serial from dataset/device_report.yaml and requires --config to "
           "reference calibration for that same serial.\n";
    return 2;
  }
  std::string argument_error;
  if (!ovrs::validate_cli_arguments(
          argc, argv,
          {"--dataset", "--config", "--rate", "--output",
           "--viewer-history"},
          {"--viewer", "--allow-unverified-calibration"},
          &argument_error)) {
    std::cerr << argument_error << '\n';
    return 2;
  }
#ifndef OVRS_HAS_OPENVINS
  std::cerr << "ovrs_replay was built without OpenVINS/OpenCV. Use "
               "scripts/build_ubuntu.sh.\n";
  return 3;
#else
  const std::filesystem::path dataset =
      ovrs::value_after(argc, argv, "--dataset");
  const std::string config = ovrs::value_after(argc, argv, "--config");
  if (dataset.empty() || config.empty() ||
      !std::filesystem::exists(config)) {
    std::cerr << "--dataset and --config are required\n";
    return 2;
  }
  std::ifstream config_input(config);
  if (!config_input) {
    std::cerr << "Cannot open estimator configuration: " << config << '\n';
    return 2;
  }
  const std::string config_text(
      (std::istreambuf_iterator<char>(config_input)),
      std::istreambuf_iterator<char>());
  const bool allow_unverified =
      ovrs::has_flag(argc, argv, "--allow-unverified-calibration");
  const std::string calibration_state =
      ovrs::simple_yaml_scalar(config_text, "calibration_state");
  const bool unverified_override_active =
      allow_unverified && calibration_state == "BOOTSTRAP_UNVERIFIED";
  std::string identity_error;
  if (!ovrs::validate_estimator_configuration(
          config, config_text, &identity_error)) {
    std::cerr << identity_error
              << ". Replay requires a reviewed serial-specific "
                 "configuration prepared after ovrs_inspect; see "
                 "scripts/prepare_bootstrap_config.sh.\n";
    return 2;
  }
  if (!ovrs::validate_estimation_calibration_state(
          config_text, allow_unverified, &identity_error)) {
    std::cerr << identity_error << '\n';
    return 2;
  }
  if (unverified_override_active) {
    std::cerr << "WARNING: replaying with BOOTSTRAP_UNVERIFIED calibration; "
                 "trajectory accuracy is not certified.\n";
  }
  if (!std::filesystem::is_directory(dataset)) {
    std::cerr << "--dataset must name an existing dataset directory\n";
    return 2;
  }
  if (std::filesystem::exists(dataset / "INCOMPLETE")) {
    std::cerr << "Dataset is marked INCOMPLETE; recover/review it before "
                 "replay.\n";
    return 2;
  }
  double rate = -1.0;
  try {
    rate = ovrs::parse_double_strict(
        ovrs::value_after(argc, argv, "--rate", "1"), "--rate");
  } catch (const std::exception &) {
    std::cerr << "--rate must be nonnegative\n";
    return 2;
  }
  if (!std::isfinite(rate) || rate < 0.0) {
    std::cerr << "--rate must be nonnegative\n";
    return 2;
  }
  const std::filesystem::path output = ovrs::value_after(
      argc, argv, "--output",
      (std::filesystem::path("runs") / ("replay_" + ovrs::utc_timestamp()))
          .string());
  ovrs::install_signal_handlers();
  try {
    const bool viewer_enabled = ovrs::has_flag(argc, argv, "--viewer");
    const std::size_t viewer_history = ovrs::bounded_size_option(
        argc, argv, "--viewer-history",
        ovrs::LiveViewer::Options::default_trajectory_points,
        ovrs::LiveViewer::Options::minimum_trajectory_points,
        ovrs::LiveViewer::Options::maximum_allowed_trajectory_points);
    const auto read_required_text =
        [](const std::filesystem::path &path,
           const std::string &description) -> std::string {
      std::ifstream input(path, std::ios::binary);
      if (!input) {
        throw std::runtime_error("cannot open required " + description +
                                 ": " + path.string());
      }
      return {std::istreambuf_iterator<char>(input),
              std::istreambuf_iterator<char>()};
    };
    const std::string dataset_metadata = read_required_text(
        dataset / "dataset_metadata.yaml", "dataset metadata");
    if (ovrs::simple_yaml_scalar(dataset_metadata, "format") !=
            "ovrs-euroc-like-v1" ||
        ovrs::simple_yaml_scalar(dataset_metadata, "complete") != "true") {
      throw std::runtime_error(
          "dataset metadata does not declare a complete "
          "ovrs-euroc-like-v1 recording");
    }
    const std::string device_report_text =
        read_required_text(dataset / "device_report.yaml", "device report");
    const auto dataset_serial =
        ovrs::simple_yaml_scalar(device_report_text, "serial");
    const auto calibrated_serial =
        ovrs::simple_yaml_scalar(config_text, "calibrated_serial");
    if (dataset_serial.empty() || dataset_serial != calibrated_serial) {
      throw std::runtime_error("dataset serial " + dataset_serial +
                               " does not match calibrated serial " +
                               calibrated_serial);
    }
    std::string rate_error;
    if (!ovrs::validate_runtime_imu_rate(
            config, config_text, device_report_text, &rate_error)) {
      throw std::runtime_error(rate_error);
    }
    const auto resolved_stream_path = dataset / "resolved_stream_config.yaml";
    const auto resolved_stream_text =
        read_required_text(resolved_stream_path, "resolved stream config");
    if (!ovrs::validate_runtime_motion_correction(
            config, config_text, device_report_text, resolved_stream_text,
            &rate_error)) {
      throw std::runtime_error(rate_error);
    }
    const auto tolerance = ovrs::simple_yaml_scalar(
        resolved_stream_text, "stereo_tolerance_ms");
    if (tolerance.empty()) {
      throw std::runtime_error(
          "resolved stream config is missing stereo_tolerance_ms");
    }
    ovrs::StreamConfig recorded_stream;
    std::string stream_error;
    if (!ovrs::apply_stream_config_yaml(
            resolved_stream_text, &recorded_stream, &stream_error)) {
      throw std::runtime_error(stream_error);
    }
    if (recorded_stream.serial.empty() ||
        recorded_stream.serial != dataset_serial) {
      throw std::runtime_error(
          "resolved stream serial does not match the device report");
    }
    if (!ovrs::validate_camera_calibration_resolution(
            config, config_text, recorded_stream.width,
            recorded_stream.height, &stream_error)) {
      throw std::runtime_error(stream_error);
    }
    const double tolerance_ms = recorded_stream.stereo_tolerance_ms;
    const double stereo_tolerance_s = tolerance_ms * 1e-3;
    std::vector<ovrs::ImuSample> imu;
    std::ifstream imu_file(dataset / "imu" / "synchronized.csv");
    if (!imu_file) {
      throw std::runtime_error("cannot open synchronized IMU CSV");
    }
    std::string line;
    std::getline(imu_file, line);
    std::optional<double> last_imu_timestamp;
    while (std::getline(imu_file, line)) {
      const auto f = split(line);
      if (f.size() != 9) {
        throw std::runtime_error("malformed synchronized IMU CSV row");
      }
      ovrs::ImuSample sample;
      sample.timestamp = ovrs::parse_double_strict(f[0], "IMU timestamp");
      sample.raw_gyro_timestamp_ms =
          ovrs::parse_double_strict(f[1], "raw IMU timestamp");
      sample.angular_velocity_rad_s = {
          ovrs::parse_double_strict(f[2], "IMU angular velocity"),
          ovrs::parse_double_strict(f[3], "IMU angular velocity"),
          ovrs::parse_double_strict(f[4], "IMU angular velocity")};
      sample.linear_acceleration_m_s2 = {
          ovrs::parse_double_strict(f[5], "IMU acceleration"),
          ovrs::parse_double_strict(f[6], "IMU acceleration"),
          ovrs::parse_double_strict(f[7], "IMU acceleration")};
      sample.interpolation_delay_s =
          ovrs::parse_double_strict(f[8], "IMU interpolation delay");
      if (!std::isfinite(sample.timestamp) ||
          !std::isfinite(sample.raw_gyro_timestamp_ms) ||
          !std::isfinite(sample.angular_velocity_rad_s.x) ||
          !std::isfinite(sample.angular_velocity_rad_s.y) ||
          !std::isfinite(sample.angular_velocity_rad_s.z) ||
          !std::isfinite(sample.linear_acceleration_m_s2.x) ||
          !std::isfinite(sample.linear_acceleration_m_s2.y) ||
          !std::isfinite(sample.linear_acceleration_m_s2.z) ||
          !std::isfinite(sample.interpolation_delay_s) ||
          sample.interpolation_delay_s < 0.0 ||
          (last_imu_timestamp &&
           sample.timestamp <= *last_imu_timestamp)) {
        throw std::runtime_error(
            "synchronized IMU values are nonfinite, unordered, or invalid");
      }
      last_imu_timestamp = sample.timestamp;
      imu.push_back(sample);
    }
    if (imu.empty()) {
      throw std::runtime_error("dataset has no synchronized IMU samples");
    }
    std::ifstream left_file(dataset / "cam0" / "data.csv");
    std::ifstream right_file(dataset / "cam1" / "data.csv");
    if (!left_file || !right_file) {
      throw std::runtime_error("cannot open stereo camera CSV files");
    }
    const auto count_rows = [](std::ifstream &stream) {
      std::size_t count = 0;
      std::string row;
      std::getline(stream, row);
      while (std::getline(stream, row)) {
        ++count;
      }
      stream.clear();
      stream.seekg(0);
      std::getline(stream, row);
      return count;
    };
    const auto left_rows = count_rows(left_file);
    const auto right_rows = count_rows(right_file);
    if (left_rows == 0 || left_rows != right_rows) {
      throw std::runtime_error(
          "stereo camera CSV files are empty or have different row counts");
    }
    std::unique_ptr<ovrs::LiveViewer> viewer;
    if (viewer_enabled) {
      ovrs::LiveViewer::Options viewer_options;
      viewer_options.maximum_trajectory_points = viewer_history;
      viewer_options.calibration_state = calibration_state;
      viewer = std::make_unique<ovrs::LiveViewer>(viewer_options);
      if (!viewer->open(&stream_error)) {
        throw std::runtime_error(stream_error);
      }
    }
    ovrs::OpenVinsEstimator estimator(config);
    ovrs::RunWriter writer;
    std::string error;
    if (!writer.open(output, &error)) {
      throw std::runtime_error(error);
    }
    if (!writer.log("ovrs_replay started", &error)) {
      throw std::runtime_error(error);
    }
    std::filesystem::copy_file(
        config, output / "resolved_config.yaml",
        std::filesystem::copy_options::overwrite_existing);
    if (!ovrs::copy_config_dependency(config, config_text,
                                      "relative_config_imu",
                                      output / "resolved_imu.yaml", &error) ||
        !ovrs::copy_config_dependency(
            config, config_text, "relative_config_imucam",
            output / "resolved_imucam.yaml", &error)) {
      throw std::runtime_error(error);
    }
    std::filesystem::copy_file(
        dataset / "device_report.yaml", output / "device_report.yaml",
        std::filesystem::copy_options::overwrite_existing);
    std::filesystem::copy_file(
        resolved_stream_path, output / "resolved_stream_config.yaml",
        std::filesystem::copy_options::overwrite_existing);
    if (!ovrs::write_text(
        output / "run_metadata.yaml",
        "%YAML:1.0\nmode: replay\ncreated_utc: \"" + ovrs::utc_timestamp() +
            "\"\nproject_version: \"" + ovrs::project_version +
            "\"\ncalibration_state: \"" +
            calibration_state +
            "\"\nunverified_calibration_override: " +
            std::string(unverified_override_active ? "true" : "false") +
            "\nbuild_type: \"" + ovrs::build_type +
            "\"\ncompiler: \"" + ovrs::compiler_version +
            "\"\nopenvins_tag: \"" + ovrs::openvins_tag +
            "\"\nopenvins_commit: \"" + ovrs::openvins_commit +
            "\"\nceres_version: \"" + ovrs::ceres_version +
            "\"\nopencv_version: \"" + ovrs::opencv_version +
            "\"\nlibrealsense_version: \"" + ovrs::realsense_version +
            "\"\ninfrared_profile: \"" +
            ovrs::simple_yaml_scalar(device_report_text,
                                     "infrared_profile") +
            "\"\ngyro_rate_hz: " +
            ovrs::simple_yaml_scalar(device_report_text, "gyro_rate_hz") +
            "\naccelerometer_rate_hz: " +
            ovrs::simple_yaml_scalar(device_report_text,
                                     "accelerometer_rate_hz") +
            "\n",
        &error)) {
      throw std::runtime_error(error);
    }
    std::size_t imu_index = 0;
    std::size_t camera_count = 0;
    double latest_latency_ms = 0.0;
    std::optional<double> first_timestamp;
    const auto wall_start = std::chrono::steady_clock::now();
    std::string left_line;
    std::string right_line;
    std::optional<double> previous_camera_timestamp;
    std::optional<double> previous_left_raw_timestamp;
    std::optional<double> previous_right_raw_timestamp;
    std::optional<std::uint64_t> previous_frameset_number;
    while (std::getline(left_file, left_line) &&
           std::getline(right_file, right_line) &&
           !ovrs::stop_requested()) {
      const auto left = split(left_line);
      const auto right = split(right_line);
      if (left.size() != 4 || right.size() != 4) {
        throw std::runtime_error("malformed camera CSV row");
      }
      const double left_timestamp =
          ovrs::parse_double_strict(left[0], "left camera timestamp");
      const double right_timestamp =
          ovrs::parse_double_strict(right[0], "right camera timestamp");
      const double timestamp = 0.5 * (left_timestamp + right_timestamp);
      const double left_raw_timestamp = ovrs::parse_double_strict(
          left[1], "left raw camera timestamp");
      const double right_raw_timestamp = ovrs::parse_double_strict(
          right[1], "right raw camera timestamp");
      const std::uint64_t frameset_number =
          ovrs::parse_uint64_strict(left[2], "camera frameset number");
      if (!std::isfinite(left_timestamp) ||
          !std::isfinite(right_timestamp) || !std::isfinite(timestamp) ||
          std::abs(left_timestamp - right_timestamp) >
              stereo_tolerance_s ||
          left[2] != right[2] ||
          (previous_camera_timestamp &&
           timestamp <= *previous_camera_timestamp) ||
          (previous_left_raw_timestamp &&
           left_raw_timestamp <= *previous_left_raw_timestamp) ||
          (previous_right_raw_timestamp &&
           right_raw_timestamp <= *previous_right_raw_timestamp) ||
          (previous_frameset_number &&
           frameset_number <= *previous_frameset_number)) {
        throw std::runtime_error(
            "stereo rows violate timestamp, raw-clock, frameset, or ordering "
            "contract");
      }
      previous_camera_timestamp = timestamp;
      previous_left_raw_timestamp = left_raw_timestamp;
      previous_right_raw_timestamp = right_raw_timestamp;
      previous_frameset_number = frameset_number;
      if (!first_timestamp) {
        first_timestamp = timestamp;
      }
      if (rate > 0.0) {
        const auto target =
            wall_start + std::chrono::duration_cast<
                             std::chrono::steady_clock::duration>(
                             std::chrono::duration<double>(
                                 (timestamp - *first_timestamp) / rate));
        if (!ovrs::wait_until_or_stop(target)) {
          break;
        }
      }
      while (imu_index < imu.size() &&
             imu[imu_index].timestamp <= timestamp) {
        estimator.feed_imu(imu[imu_index++]);
      }
      if (imu_index > 0 && imu[imu_index - 1].timestamp < timestamp &&
          imu_index < imu.size()) {
        estimator.feed_imu(imu[imu_index++]);
      }
      if (imu_index == 0 || imu[imu_index - 1].timestamp < timestamp) {
        throw std::runtime_error("dataset IMU does not bracket camera time");
      }
      const auto make_image = [&](const std::vector<std::string> &row,
                                  int camera_id) {
        const std::string expected_filename =
            std::to_string(frameset_number) + ".png";
        if (row[3] != expected_filename) {
          throw std::runtime_error(
              "camera CSV filename does not match its frameset number");
        }
        const auto path =
            dataset / ("cam" + std::to_string(camera_id)) / "data" / row[3];
        if (std::filesystem::path(row[3]).filename().string() != row[3]) {
          throw std::runtime_error("camera CSV contains an unsafe image path");
        }
        cv::Mat image = cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
        if (image.empty()) {
          throw std::runtime_error("cannot read " + path.string());
        }
        if (image.cols != recorded_stream.width ||
            image.rows != recorded_stream.height ||
            image.type() != CV_8UC1) {
          throw std::runtime_error(
              "camera image dimensions/format do not match the recorded "
              "stream configuration");
        }
        if (!image.isContinuous()) {
          image = image.clone();
        }
        ovrs::ImageFrame frame;
        frame.timestamp =
            ovrs::parse_double_strict(row[0], "camera timestamp");
        frame.raw_timestamp_ms =
            camera_id == 0 ? left_raw_timestamp : right_raw_timestamp;
        if (!std::isfinite(frame.timestamp) ||
            !std::isfinite(frame.raw_timestamp_ms) ||
            (!row[2].empty() && row[2].front() == '-')) {
          throw std::runtime_error("camera CSV contains invalid numeric data");
        }
        frame.frameset_number = frameset_number;
        frame.camera_id = camera_id;
        frame.width = image.cols;
        frame.height = image.rows;
        frame.stride_bytes = image.cols;
        frame.format = "Y8";
        frame.pixels = std::make_shared<std::vector<std::uint8_t>>(
            image.data, image.data + image.total());
        return frame;
      };
      ovrs::StereoFrame stereo;
      stereo.timestamp = timestamp;
      stereo.camera0 = make_image(left, 0);
      stereo.camera1 = make_image(right, 1);
      if (viewer) {
        viewer->publish_stereo(stereo);
      }
      const auto begin = std::chrono::steady_clock::now();
      estimator.feed_stereo(stereo);
      ++camera_count;
      const double latency =
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - begin)
              .count();
      latest_latency_ms = latency;
      if (auto state = estimator.latest_state(latency)) {
        if (!writer.write_state(*state, &error)) {
          throw std::runtime_error(error);
        }
        if (viewer) {
          viewer->publish_state(*state);
        }
      }
      if (viewer) {
        std::string poll_error;
        if (!viewer->poll(&poll_error)) {
          if (!poll_error.empty()) {
            throw std::runtime_error(poll_error);
          }
          ovrs::request_stop();
          break;
        }
      }
    }
    ovrs::DiagnosticsSnapshot diagnostics;
    diagnostics.synchronized_imu_samples = imu_index;
    diagnostics.received_camera_frames = camera_count * 2;
    diagnostics.valid_stereo_pairs = camera_count;
    if (first_timestamp && !imu.empty() &&
        imu.back().timestamp > *first_timestamp) {
      const double sensor_duration = imu.back().timestamp - *first_timestamp;
      diagnostics.camera_rate_hz =
          static_cast<double>(camera_count) / sensor_duration;
      diagnostics.imu_rate_hz =
          static_cast<double>(imu_index) / sensor_duration;
      diagnostics.estimator_rate_hz = diagnostics.camera_rate_hz;
    }
    const bool replay_initialized = estimator.initialized();
    diagnostics.initialized = replay_initialized;
    diagnostics.processing_latency_ms = latest_latency_ms;
    const bool interrupted = ovrs::stop_requested();
    if (viewer) {
      viewer->close();
    }
    if (!writer.write_diagnostics(diagnostics, &error) ||
        !writer.log("initialization_time_s=" +
                        std::to_string(estimator.initialization_time()),
                    &error) ||
        !writer.log(interrupted
                        ? "ovrs_replay interrupted by user"
                        : (replay_initialized
                               ? "ovrs_replay stopped cleanly"
                               : "ovrs_replay completed without "
                                 "initialization"),
                    &error)) {
      throw std::runtime_error(error);
    }
    if (interrupted) {
      if (!writer.close(&error)) {
        throw std::runtime_error(error);
      }
      std::cout << "Replay interrupted by user; partial run retained: "
                << output << '\n';
      return 130;
    }
    if (!replay_initialized) {
      if (!writer.close(&error)) {
        throw std::runtime_error(error);
      }
      std::cerr << "OpenVINS did not initialize during replay; partial run "
                   "retained: "
                << output << '\n';
      return 5;
    }
    if (!writer.finalize(&error)) {
      throw std::runtime_error(error);
    }
    std::cout << "Replay complete: " << output << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 4;
  }
#endif
}
