#pragma once

#include "ovrs/version.hpp"
#include "ovrs/yaml_utils.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

namespace ovrs {

inline volatile std::sig_atomic_t &stop_requested_flag() {
  static volatile std::sig_atomic_t value = 0;
  return value;
}

inline bool stop_requested() { return stop_requested_flag() != 0; }

inline void request_stop() { stop_requested_flag() = 1; }

inline void signal_handler(int) { stop_requested_flag() = 1; }

inline void install_signal_handlers() {
  stop_requested_flag() = 0;
  std::signal(SIGINT, signal_handler);
#ifdef SIGTERM
  std::signal(SIGTERM, signal_handler);
#endif
}

inline bool wait_until_or_stop(
    const std::chrono::steady_clock::time_point &target) {
  while (!stop_requested()) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= target) {
      return true;
    }
    std::this_thread::sleep_until(
        std::min(target, now + std::chrono::milliseconds(20)));
  }
  return false;
}

inline std::string utc_timestamp() {
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  gmtime_s(&tm, &time);
#else
  gmtime_r(&time, &tm);
#endif
  std::ostringstream out;
  out << std::put_time(&tm, "%Y%m%dT%H%M%SZ");
  return out.str();
}

inline bool write_text(const std::filesystem::path &path,
                       const std::string &contents, std::string *error) {
  std::ofstream out(path, std::ios::binary);
  out << contents;
  if (!out) {
    if (error) {
      *error = "cannot write " + path.string();
    }
    return false;
  }
  return true;
}

inline std::string value_after(int argc, char **argv,
                               const std::string &option,
                               const std::string &fallback = {}) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == option) {
      return argv[i + 1];
    }
  }
  return fallback;
}

inline bool has_flag(int argc, char **argv, const std::string &flag) {
  for (int i = 1; i < argc; ++i) {
    if (argv[i] == flag) {
      return true;
    }
  }
  return false;
}

inline std::size_t bounded_size_option(
    int argc, char **argv, const std::string &option,
    std::size_t default_value, std::size_t minimum,
    std::size_t maximum) {
  if (minimum > maximum || default_value < minimum ||
      default_value > maximum) {
    throw std::logic_error("invalid bounds for " + option);
  }
  const auto text = value_after(argc, argv, option);
  if (text.empty()) {
    return default_value;
  }
  const auto parsed = parse_uint64_strict(text, option);
  if (parsed < minimum || parsed > maximum) {
    throw std::runtime_error(
        option + " must be in [" + std::to_string(minimum) + "," +
        std::to_string(maximum) + "]");
  }
  return static_cast<std::size_t>(parsed);
}

inline std::string version_summary(const std::string &application) {
  return application + " " + project_version + "\nOpenVINS " +
         openvins_tag + " (" + openvins_commit + ")\nCeres " +
         ceres_version + "\nOpenCV " + opencv_version +
         "\nlibrealsense " + realsense_version + "\nSource fingerprint " +
         source_fingerprint + "\n";
}

inline bool validate_cli_arguments(
    int argc, char **argv, const std::vector<std::string> &value_options,
    const std::vector<std::string> &flag_options, std::string *error) {
  std::unordered_set<std::string> seen;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    const bool takes_value =
        std::find(value_options.begin(), value_options.end(), argument) !=
        value_options.end();
    const bool is_flag =
        std::find(flag_options.begin(), flag_options.end(), argument) !=
        flag_options.end();
    if (!takes_value && !is_flag) {
      if (error) {
        *error = "unknown option: " + argument;
      }
      return false;
    }
    if (!seen.insert(argument).second) {
      if (error) {
        *error = argument + " may be specified only once";
      }
      return false;
    }
    if (takes_value) {
      if (index + 1 >= argc ||
          std::string(argv[index + 1]).rfind("--", 0) == 0) {
        if (error) {
          *error = argument + " requires a value";
        }
        return false;
      }
      ++index;
    }
  }
  return true;
}

inline bool validate_calibration_identity(const std::string &yaml,
                                          std::string *error) {
  if (simple_yaml_key_count(yaml, "calibration_state") != 1 ||
      simple_yaml_key_count(yaml, "calibrated_serial") != 1) {
    if (error) {
      *error = "calibration_state and calibrated_serial must each appear "
               "exactly once";
    }
    return false;
  }
  const auto state = simple_yaml_scalar(yaml, "calibration_state");
  if (state != "BOOTSTRAP_UNVERIFIED" && state != "KALIBR_VERIFIED") {
    if (error) {
      *error = "calibration_state must be BOOTSTRAP_UNVERIFIED or "
               "KALIBR_VERIFIED";
    }
    return false;
  }
  const auto serial = simple_yaml_scalar(yaml, "calibrated_serial");
  const bool numeric_serial =
      !serial.empty() &&
      std::all_of(serial.begin(), serial.end(), [](unsigned char character) {
        return std::isdigit(character) != 0;
      });
  if (!numeric_serial || serial == "REPLACE_WITH_DEVICE_SERIAL") {
    if (error) {
      *error = "calibrated_serial must be the numeric serial of the reviewed "
               "D435i unit";
    }
    return false;
  }
  return true;
}

inline bool validate_estimation_calibration_state(
    const std::string &yaml, bool allow_unverified, std::string *error) {
  const auto state = simple_yaml_scalar(yaml, "calibration_state");
  if (state == "KALIBR_VERIFIED") {
    return true;
  }
  if (state == "BOOTSTRAP_UNVERIFIED" && allow_unverified) {
    return true;
  }
  if (error) {
    *error =
        "estimation requires calibration_state KALIBR_VERIFIED. "
        "BOOTSTRAP_UNVERIFIED contains factory camera data plus unmeasured "
        "camera-IMU timing and IMU parameters; use "
        "--allow-unverified-calibration only for explicitly labelled "
        "diagnostics";
  }
  return false;
}

inline bool safe_relative_config_path(const std::string &text) {
  const std::filesystem::path path(text);
  const bool windows_drive =
      text.size() >= 2 &&
      ((text[0] >= 'A' && text[0] <= 'Z') ||
       (text[0] >= 'a' && text[0] <= 'z')) &&
      text[1] == ':';
  if (text.empty() || text.front() == '/' ||
      text.find('\\') != std::string::npos ||
      windows_drive || path.is_absolute() || path.has_root_name()) {
    return false;
  }
  for (const auto &component : path) {
    if (component == "..") {
      return false;
    }
  }
  return true;
}

inline bool resolve_estimator_dependency_path(
    const std::filesystem::path &main_config, const std::string &main_yaml,
    const std::string &key, std::filesystem::path *resolved,
    std::string *error) {
  if (simple_yaml_key_count(main_yaml, key) != 1) {
    if (error) {
      *error = key + " must appear exactly once";
    }
    return false;
  }
  const auto relative = simple_yaml_scalar(main_yaml, key);
  if (!safe_relative_config_path(relative)) {
    if (error) {
      *error = key + " must be a safe relative path";
    }
    return false;
  }
  std::error_code ec;
  const auto configured_parent = main_config.parent_path().empty()
                                     ? std::filesystem::path(".")
                                     : main_config.parent_path();
  const auto base =
      std::filesystem::weakly_canonical(configured_parent, ec);
  if (ec) {
    if (error) {
      *error = "cannot resolve estimator configuration directory: " +
               ec.message();
    }
    return false;
  }
  const auto candidate =
      std::filesystem::weakly_canonical(base / relative, ec);
  if (ec) {
    if (error) {
      *error = "cannot resolve estimator dependency: " + ec.message();
    }
    return false;
  }
  auto base_component = base.begin();
  auto candidate_component = candidate.begin();
  for (; base_component != base.end() &&
         candidate_component != candidate.end();
       ++base_component, ++candidate_component) {
    if (*base_component != *candidate_component) {
      if (error) {
        *error = key + " resolves outside the estimator configuration "
                       "directory";
      }
      return false;
    }
  }
  if (base_component != base.end() ||
      !std::filesystem::is_regular_file(candidate, ec) || ec) {
    if (error) {
      *error = key + " must resolve to a regular file inside the estimator "
                     "configuration directory";
    }
    return false;
  }
  if (resolved) {
    *resolved = candidate;
  }
  return true;
}

inline bool read_estimator_dependency(
    const std::filesystem::path &main_config, const std::string &main_yaml,
    const std::string &key, std::string *contents, std::string *error) {
  std::filesystem::path path;
  if (!resolve_estimator_dependency_path(main_config, main_yaml, key, &path,
                                         error)) {
    return false;
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    if (error) {
      *error = "cannot open estimator dependency: " + path.string();
    }
    return false;
  }
  if (contents) {
    contents->assign(std::istreambuf_iterator<char>(input),
                     std::istreambuf_iterator<char>());
  }
  return true;
}

inline bool configured_imu_rate(const std::filesystem::path &main_config,
                                const std::string &main_yaml, int *rate,
                                std::string *error) {
  std::string imu_yaml;
  if (!read_estimator_dependency(main_config, main_yaml,
                                 "relative_config_imu", &imu_yaml, error)) {
    return false;
  }
  if (simple_yaml_key_count(imu_yaml, "update_rate") != 1) {
    if (error) {
      *error = "IMU calibration update_rate must appear exactly once";
    }
    return false;
  }
  const auto text = simple_yaml_scalar(imu_yaml, "update_rate");
  try {
    std::size_t consumed = 0;
    const int value = std::stoi(text, &consumed);
    if (text.empty() || consumed != text.size() || value <= 0) {
      throw std::invalid_argument("not a positive integer");
    }
    if (rate) {
      *rate = value;
    }
  } catch (const std::exception &) {
    if (error) {
      *error = "IMU calibration update_rate must be a positive integer";
    }
    return false;
  }
  return true;
}

inline bool validate_runtime_imu_rate(
    const std::filesystem::path &main_config, const std::string &main_yaml,
    const std::string &device_report_yaml, std::string *error) {
  int expected = 0;
  if (!configured_imu_rate(main_config, main_yaml, &expected, error)) {
    return false;
  }
  const auto actual_text =
      simple_yaml_scalar(device_report_yaml, "gyro_rate_hz");
  try {
    std::size_t consumed = 0;
    const int actual = std::stoi(actual_text, &consumed);
    if (actual_text.empty() || consumed != actual_text.size() ||
        actual != expected) {
      throw std::invalid_argument("rate mismatch");
    }
  } catch (const std::exception &) {
    if (error) {
      *error = "actual gyro rate does not match IMU calibration update_rate " +
               std::to_string(expected);
    }
    return false;
  }
  return true;
}

inline bool validate_runtime_motion_correction(
    const std::filesystem::path &main_config,
    const std::string &main_yaml, const std::string &device_report_yaml,
    const std::string &stream_yaml, std::string *error) {
  std::filesystem::path imu_path;
  if (!resolve_estimator_dependency_path(
          main_config, main_yaml, "relative_config_imu", &imu_path, error)) {
    return false;
  }
  std::ifstream input(imu_path, std::ios::binary);
  if (!input) {
    if (error) {
      *error = "cannot read IMU calibration: " + imu_path.string();
    }
    return false;
  }
  const std::string imu_yaml((std::istreambuf_iterator<char>(input)),
                             std::istreambuf_iterator<char>());
  const auto read_bool = [&](const std::string &yaml,
                             const std::string &key,
                             bool *value) -> bool {
    if (simple_yaml_key_count(yaml, key) != 1) {
      if (error) {
        *error = key + " must appear exactly once";
      }
      return false;
    }
    const auto text = simple_yaml_scalar(yaml, key);
    if (text != "true" && text != "false") {
      if (error) {
        *error = key + " must be true or false";
      }
      return false;
    }
    *value = text == "true";
    return true;
  };
  bool calibration_enabled = false;
  bool requested_enabled = false;
  bool actual_enabled = false;
  if (!read_bool(imu_yaml, "realsense_motion_correction_enabled",
                 &calibration_enabled) ||
      !read_bool(stream_yaml, "motion_correction_enabled",
                 &requested_enabled) ||
      !read_bool(device_report_yaml, "motion_correction_active",
                 &actual_enabled)) {
    return false;
  }
  if (calibration_enabled != requested_enabled ||
      requested_enabled != actual_enabled) {
    if (error) {
      *error =
          "RealSense motion-correction policy differs between the IMU "
          "calibration, stream configuration, and captured device report";
    }
    return false;
  }
  return true;
}

inline bool parse_camera_resolution(const std::string &text, int *width,
                                    int *height) {
  std::istringstream input(text);
  char open = '\0';
  char comma = '\0';
  char close = '\0';
  int parsed_width = 0;
  int parsed_height = 0;
  input >> std::ws >> open >> parsed_width >> std::ws >> comma >>
      parsed_height >> std::ws >> close >> std::ws;
  if (!input.eof() || open != '[' || comma != ',' || close != ']' ||
      parsed_width <= 0 || parsed_height <= 0) {
    return false;
  }
  if (width) {
    *width = parsed_width;
  }
  if (height) {
    *height = parsed_height;
  }
  return true;
}

inline bool parse_transform_matrices(
    const std::string &yaml, const std::string &key,
    std::vector<std::array<double, 16>> *matrices) {
  if (!matrices) {
    return false;
  }
  matrices->clear();
  std::istringstream input(yaml);
  std::string line;
  while (std::getline(input, line)) {
    const auto first = line.find_first_not_of(" \t");
    if (first == std::string::npos ||
        line.compare(first, key.size() + 1, key + ":") != 0) {
      continue;
    }
    const auto after_key = first + key.size() + 1;
    const auto remainder = line.find_first_not_of(" \t", after_key);
    if (remainder != std::string::npos && line[remainder] != '#') {
      return false;
    }
    std::array<double, 16> matrix{};
    for (std::size_t row = 0; row < 4; ++row) {
      std::string row_line;
      do {
        if (!std::getline(input, row_line)) {
          return false;
        }
        const auto comment = row_line.find('#');
        if (comment != std::string::npos) {
          row_line.erase(comment);
        }
      } while (row_line.find_first_not_of(" \t") == std::string::npos);
      const auto dash = row_line.find_first_not_of(" \t");
      if (dash == std::string::npos || row_line[dash] != '-') {
        return false;
      }
      std::vector<double> values;
      if (!parse_double_list_strict(row_line.substr(dash + 1), 4,
                                    &values)) {
        return false;
      }
      for (std::size_t column = 0; column < 4; ++column) {
        matrix[row * 4 + column] = values[column];
      }
    }
    matrices->push_back(matrix);
  }
  return true;
}

inline bool validate_camera_calibration_geometry(
    const std::string &camera_yaml, std::string *error) {
  const auto fail = [&](const std::string &message) {
    if (error) {
      *error = message;
    }
    return false;
  };
  const auto camera_models =
      simple_yaml_scalars(camera_yaml, "camera_model");
  const auto distortion_models =
      simple_yaml_scalars(camera_yaml, "distortion_model");
  const auto distortion_coefficients =
      simple_yaml_scalars(camera_yaml, "distortion_coeffs");
  const auto intrinsics = simple_yaml_scalars(camera_yaml, "intrinsics");
  const auto resolutions = simple_yaml_scalars(camera_yaml, "resolution");
  const auto time_offsets =
      simple_yaml_scalars(camera_yaml, "timeshift_cam_imu");
  std::vector<std::array<double, 16>> transforms;
  if (camera_models.size() != 2 || distortion_models.size() != 2 ||
      distortion_coefficients.size() != 2 || intrinsics.size() != 2 ||
      resolutions.size() != 2 || time_offsets.size() != 2 ||
      !parse_transform_matrices(camera_yaml, "T_imu_cam", &transforms) ||
      transforms.size() != 2) {
    return fail("camera calibration requires two complete camera models, "
                "intrinsics, time offsets, and 4x4 T_imu_cam matrices");
  }
  std::array<std::array<double, 3>, 2> camera_centres{};
  std::array<double, 2> parsed_time_offsets{};
  constexpr double matrix_tolerance = 1e-6;
  for (std::size_t camera_id = 0; camera_id < 2; ++camera_id) {
    int width = 0;
    int height = 0;
    std::vector<double> intrinsic_values;
    std::vector<double> distortion_values;
    if (camera_models[camera_id] != "pinhole" ||
        (distortion_models[camera_id] != "radtan" &&
         distortion_models[camera_id] != "equidistant") ||
        !parse_camera_resolution(resolutions[camera_id], &width, &height) ||
        !parse_double_list_strict(intrinsics[camera_id], 4,
                                  &intrinsic_values) ||
        !parse_double_list_strict(distortion_coefficients[camera_id], 4,
                                  &distortion_values)) {
      return fail("cam" + std::to_string(camera_id) +
                  " has an invalid OpenVINS camera model");
    }
    if (intrinsic_values[0] <= 0.0 || intrinsic_values[1] <= 0.0 ||
        intrinsic_values[2] < 0.0 ||
        intrinsic_values[2] >= static_cast<double>(width) ||
        intrinsic_values[3] < 0.0 ||
        intrinsic_values[3] >= static_cast<double>(height)) {
      return fail("cam" + std::to_string(camera_id) +
                  " intrinsics are outside the declared image");
    }
    try {
      parsed_time_offsets[camera_id] = parse_double_strict(
          time_offsets[camera_id], "timeshift_cam_imu");
    } catch (const std::exception &) {
      return fail("cam" + std::to_string(camera_id) +
                  " has an invalid timeshift_cam_imu");
    }
    const auto &transform = transforms[camera_id];
    if (std::abs(transform[12]) > matrix_tolerance ||
        std::abs(transform[13]) > matrix_tolerance ||
        std::abs(transform[14]) > matrix_tolerance ||
        std::abs(transform[15] - 1.0) > matrix_tolerance) {
      return fail("cam" + std::to_string(camera_id) +
                  " T_imu_cam has an invalid homogeneous row");
    }
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        double dot = 0.0;
        for (std::size_t index = 0; index < 3; ++index) {
          dot += transform[index * 4 + row] *
                 transform[index * 4 + column];
        }
        const double expected = row == column ? 1.0 : 0.0;
        if (std::abs(dot - expected) > matrix_tolerance) {
          return fail("cam" + std::to_string(camera_id) +
                      " T_imu_cam rotation is not orthonormal");
        }
      }
    }
    const double determinant =
        transform[0] *
            (transform[5] * transform[10] -
             transform[6] * transform[9]) -
        transform[1] *
            (transform[4] * transform[10] -
             transform[6] * transform[8]) +
        transform[2] *
            (transform[4] * transform[9] -
             transform[5] * transform[8]);
    if (std::abs(determinant - 1.0) > matrix_tolerance) {
      return fail("cam" + std::to_string(camera_id) +
                  " T_imu_cam rotation determinant is not +1");
    }
    camera_centres[camera_id] = {
        transform[3], transform[7], transform[11]};
  }
  if (std::abs(parsed_time_offsets[0] - parsed_time_offsets[1]) >
      1e-9) {
    return fail("OpenVINS v2.7 uses one camera-IMU time offset; cam0 and "
                "cam1 timeshift_cam_imu must match");
  }
  double baseline_squared = 0.0;
  for (std::size_t axis = 0; axis < 3; ++axis) {
    const double difference =
        camera_centres[1][axis] - camera_centres[0][axis];
    baseline_squared += difference * difference;
  }
  if (!std::isfinite(baseline_squared) || baseline_squared <= 1e-12) {
    return fail("stereo T_imu_cam matrices produce a degenerate baseline");
  }
  return true;
}

inline bool validate_bootstrap_camera_calibration(
    const std::string &camera_yaml, std::string *error) {
  const auto fail = [&](const std::string &message) {
    if (error) {
      *error = message;
    }
    return false;
  };
  const auto camera_models =
      simple_yaml_scalars(camera_yaml, "camera_model");
  const auto realsense_models =
      simple_yaml_scalars(camera_yaml, "realsense_distortion_model");
  const auto realsense_coefficients =
      simple_yaml_scalars(camera_yaml, "realsense_distortion_coeffs");
  const auto openvins_models =
      simple_yaml_scalars(camera_yaml, "distortion_model");
  const auto openvins_coefficients =
      simple_yaml_scalars(camera_yaml, "distortion_coeffs");
  const auto intrinsics = simple_yaml_scalars(camera_yaml, "intrinsics");
  const auto resolutions = simple_yaml_scalars(camera_yaml, "resolution");
  const auto time_offsets =
      simple_yaml_scalars(camera_yaml, "timeshift_cam_imu");
  if (camera_models.size() != 2 || realsense_models.size() != 2 ||
      realsense_coefficients.size() != 2 ||
      openvins_models.size() != 2 || openvins_coefficients.size() != 2 ||
      intrinsics.size() != 2 || resolutions.size() != 2 ||
      time_offsets.size() != 2 ||
      simple_yaml_key_count(camera_yaml, "T_gyro_accel") != 1 ||
      simple_yaml_key_count(camera_yaml, "T_imu_cam") != 2 ||
      simple_yaml_key_count(camera_yaml, "T_cam_imu") != 0) {
    return fail(
        "bootstrap camera calibration is missing required factory fields");
  }
  constexpr double coefficient_tolerance = 1e-12;
  for (std::size_t camera_id = 0; camera_id < 2; ++camera_id) {
    int width = 0;
    int height = 0;
    std::vector<double> intrinsic_values;
    std::vector<double> realsense_values;
    std::vector<double> openvins_values;
    if (!parse_camera_resolution(resolutions[camera_id], &width, &height) ||
        !parse_double_list_strict(intrinsics[camera_id], 4,
                                  &intrinsic_values) ||
        !parse_double_list_strict(realsense_coefficients[camera_id], 5,
                                  &realsense_values) ||
        !parse_double_list_strict(openvins_coefficients[camera_id], 4,
                                  &openvins_values)) {
      return fail("bootstrap cam" + std::to_string(camera_id) +
                  " contains a malformed numeric camera field");
    }
    if (camera_models[camera_id] != "pinhole" ||
        openvins_models[camera_id] != "radtan") {
      return fail("bootstrap cam" + std::to_string(camera_id) +
                  " must use the reviewed pinhole/radtan mapping");
    }
    if (realsense_models[camera_id] != "Brown Conrady" &&
        realsense_models[camera_id] != "None") {
      return fail("bootstrap cam" + std::to_string(camera_id) +
                  " has an unsupported RealSense distortion model");
    }
    if (intrinsic_values[0] <= 0.0 || intrinsic_values[1] <= 0.0 ||
        intrinsic_values[2] < 0.0 ||
        intrinsic_values[2] >= static_cast<double>(width) ||
        intrinsic_values[3] < 0.0 ||
        intrinsic_values[3] >= static_cast<double>(height)) {
      return fail("bootstrap cam" + std::to_string(camera_id) +
                  " intrinsics are outside the declared image");
    }
    if (std::abs(realsense_values[4]) > coefficient_tolerance) {
      return fail("bootstrap cam" + std::to_string(camera_id) +
                  " has a nonzero fifth Brown-Conrady coefficient that "
                  "cannot be represented by OpenVINS radtan");
    }
    for (std::size_t coefficient = 0; coefficient < 4; ++coefficient) {
      if (std::abs(realsense_values[coefficient] -
                   openvins_values[coefficient]) >
          coefficient_tolerance) {
        return fail("bootstrap cam" + std::to_string(camera_id) +
                    " RealSense/OpenVINS distortion coefficients differ");
      }
    }
    if (realsense_models[camera_id] == "None" &&
        std::any_of(realsense_values.begin(), realsense_values.end(),
                    [](double value) {
                      return std::abs(value) > coefficient_tolerance;
                    })) {
      return fail("bootstrap cam" + std::to_string(camera_id) +
                  " declares no distortion but has nonzero coefficients");
    }
    try {
      (void)parse_double_strict(time_offsets[camera_id],
                                "timeshift_cam_imu");
    } catch (const std::exception &) {
      return fail("bootstrap cam" + std::to_string(camera_id) +
                  " has an invalid timeshift_cam_imu");
    }
  }
  return true;
}

inline bool validate_camera_calibration_resolution(
    const std::filesystem::path &main_config, const std::string &main_yaml,
    int expected_width, int expected_height, std::string *error) {
  if (expected_width <= 0 || expected_height <= 0) {
    if (error) {
      *error = "expected stream resolution must be positive";
    }
    return false;
  }
  std::string camera_yaml;
  if (!read_estimator_dependency(main_config, main_yaml,
                                 "relative_config_imucam", &camera_yaml,
                                 error)) {
    return false;
  }
  const auto resolutions = simple_yaml_scalars(camera_yaml, "resolution");
  if (resolutions.size() != 2) {
    if (error) {
      *error =
          "camera calibration must contain exactly two resolution entries";
    }
    return false;
  }
  for (std::size_t camera_id = 0; camera_id < resolutions.size();
       ++camera_id) {
    int width = 0;
    int height = 0;
    if (!parse_camera_resolution(resolutions[camera_id], &width, &height)) {
      if (error) {
        *error = "camera calibration cam" + std::to_string(camera_id) +
                 " has an invalid resolution";
      }
      return false;
    }
    if (width != expected_width || height != expected_height) {
      if (error) {
        *error = "camera calibration cam" + std::to_string(camera_id) +
                 " resolution " + std::to_string(width) + "x" +
                 std::to_string(height) +
                 " does not match stream resolution " +
                 std::to_string(expected_width) + "x" +
                 std::to_string(expected_height);
      }
      return false;
    }
  }
  return true;
}

inline bool validate_estimator_configuration(
    const std::filesystem::path &main_config, const std::string &main_yaml,
    std::string *error) {
  if (!validate_calibration_identity(main_yaml, error)) {
    return false;
  }
  int imu_rate = 0;
  if (!configured_imu_rate(main_config, main_yaml, &imu_rate, error)) {
    return false;
  }
  std::string imu_yaml;
  if (!read_estimator_dependency(main_config, main_yaml,
                                 "relative_config_imu", &imu_yaml, error)) {
    return false;
  }
  std::string camera_yaml;
  if (!read_estimator_dependency(main_config, main_yaml,
                                 "relative_config_imucam", &camera_yaml,
                                 error)) {
    return false;
  }
  const auto main_state = simple_yaml_scalar(main_yaml, "calibration_state");
  const auto main_serial = simple_yaml_scalar(main_yaml, "calibrated_serial");
  const auto imu_state = simple_yaml_scalar(imu_yaml, "calibration_state");
  const auto imu_serial = simple_yaml_scalar(imu_yaml, "calibrated_serial");
  const auto camera_state =
      simple_yaml_scalar(camera_yaml, "calibration_state");
  const auto camera_serial =
      simple_yaml_scalar(camera_yaml, "calibrated_serial");
  if (simple_yaml_key_count(imu_yaml, "calibration_state") != 1 ||
      simple_yaml_key_count(imu_yaml, "calibrated_serial") != 1) {
    if (error) {
      *error = "IMU calibration state and serial must each appear exactly "
               "once";
    }
    return false;
  }
  if (imu_state != main_state || imu_serial != main_serial) {
    if (error) {
      *error = "IMU calibration state/serial does not match the main "
               "estimator configuration";
    }
    return false;
  }
  if (simple_yaml_key_count(imu_yaml, "model") != 1 ||
      simple_yaml_scalar(imu_yaml, "model") != "kalibr") {
    if (error) {
      *error = "IMU calibration model must appear exactly once and be kalibr";
    }
    return false;
  }
  for (const std::string key :
       {"accelerometer_noise_density", "accelerometer_random_walk",
        "gyroscope_noise_density", "gyroscope_random_walk"}) {
    if (simple_yaml_key_count(imu_yaml, key) != 1) {
      if (error) {
        *error = "IMU calibration " + key + " must appear exactly once";
      }
      return false;
    }
    try {
      if (parse_double_strict(simple_yaml_scalar(imu_yaml, key), key) <=
          0.0) {
        throw std::invalid_argument("not positive");
      }
    } catch (const std::exception &) {
      if (error) {
        *error = "IMU calibration " + key +
                 " must be a positive finite value";
      }
      return false;
    }
  }
  if (simple_yaml_key_count(camera_yaml, "calibration_state") != 1 ||
      simple_yaml_key_count(camera_yaml, "calibrated_serial") != 1) {
    if (error) {
      *error = "camera calibration state and serial must each appear "
               "exactly once";
    }
    return false;
  }
  if (camera_state != main_state || camera_serial != main_serial) {
    if (error) {
      *error = "camera calibration state/serial does not match the main "
               "estimator configuration";
    }
    return false;
  }
  if (camera_yaml.find("INVALID PLACEHOLDER") != std::string::npos ||
      simple_yaml_key_count(camera_yaml, "cam0") != 1 ||
      simple_yaml_key_count(camera_yaml, "cam1") != 1 ||
      simple_yaml_key_count(camera_yaml, "T_imu_cam") != 2 ||
      simple_yaml_key_count(camera_yaml, "T_cam_imu") != 0 ||
      simple_yaml_key_count(camera_yaml, "intrinsics") != 2 ||
      simple_yaml_key_count(camera_yaml, "resolution") != 2) {
    if (error) {
      *error = "camera calibration is missing reviewed cam0/cam1 values or "
               "is still marked as a placeholder";
    }
    return false;
  }
  if (!validate_camera_calibration_geometry(camera_yaml, error)) {
    return false;
  }
  if (main_state == "BOOTSTRAP_UNVERIFIED" &&
      !validate_bootstrap_camera_calibration(camera_yaml, error)) {
    return false;
  }
  return true;
}

inline bool copy_config_dependency(const std::filesystem::path &main_config,
                                   const std::string &main_yaml,
                                   const std::string &key,
                                   const std::filesystem::path &destination,
                                   std::string *error) {
  std::filesystem::path source;
  if (!resolve_estimator_dependency_path(main_config, main_yaml, key,
                                         &source, error)) {
    return false;
  }
  std::error_code ec;
  std::filesystem::copy_file(
      source, destination, std::filesystem::copy_options::overwrite_existing,
      ec);
  if (ec) {
    if (error) {
      *error = "cannot copy " + source.string() + ": " + ec.message();
    }
    return false;
  }
  return true;
}

} // namespace ovrs
