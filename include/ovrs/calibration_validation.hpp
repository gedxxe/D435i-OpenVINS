#pragma once

#include <filesystem>
#include <string>

namespace ovrs {

bool validate_calibration_identity(const std::string &yaml,
                                   std::string *error);
bool validate_estimation_calibration_state(
    const std::string &yaml, bool allow_unverified, std::string *error);

bool safe_relative_config_path(const std::string &text);
bool copy_config_dependency(
    const std::filesystem::path &main_config,
    const std::string &main_yaml, const std::string &key,
    const std::filesystem::path &destination, std::string *error);

bool validate_runtime_imu_rate(
    const std::filesystem::path &main_config,
    const std::string &main_yaml,
    const std::string &device_report_yaml, std::string *error);
bool validate_runtime_sensor_policy(
    const std::filesystem::path &main_config,
    const std::string &main_yaml,
    const std::string &device_report_yaml,
    const std::string &stream_yaml, std::string *error);

bool parse_camera_resolution(const std::string &text, int *width,
                             int *height);
bool validate_camera_calibration_geometry(
    const std::string &camera_yaml, std::string *error);
bool validate_bootstrap_camera_calibration(
    const std::string &camera_yaml, std::string *error);
bool validate_camera_calibration_resolution(
    const std::filesystem::path &main_config,
    const std::string &main_yaml, int expected_width,
    int expected_height, std::string *error);
bool validate_estimator_configuration(
    const std::filesystem::path &main_config,
    const std::string &main_yaml, std::string *error);

} // namespace ovrs
