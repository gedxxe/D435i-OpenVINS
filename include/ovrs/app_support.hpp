#pragma once

#include <array>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace ovrs {

volatile std::sig_atomic_t &stop_requested_flag();
bool stop_requested();
void request_stop();
void signal_handler(int signal);
void install_signal_handlers();

bool wait_until_or_stop(
    const std::chrono::steady_clock::time_point &target);
std::string utc_timestamp();
bool write_text(const std::filesystem::path &path,
                const std::string &contents, std::string *error);

std::string value_after(int argc, char **argv,
                        const std::string &option,
                        const std::string &fallback = {});
bool has_flag(int argc, char **argv, const std::string &flag);
std::size_t bounded_size_option(
    int argc, char **argv, const std::string &option,
    std::size_t default_value, std::size_t minimum,
    std::size_t maximum);
std::string version_summary(const std::string &application);
bool validate_cli_arguments(
    int argc, char **argv,
    const std::vector<std::string> &value_options,
    const std::vector<std::string> &flag_options, std::string *error);

bool validate_calibration_identity(const std::string &yaml,
                                   std::string *error);
bool validate_estimation_calibration_state(
    const std::string &yaml, bool allow_unverified, std::string *error);
bool safe_relative_config_path(const std::string &text);
bool resolve_estimator_dependency_path(
    const std::filesystem::path &main_config,
    const std::string &main_yaml, const std::string &key,
    std::filesystem::path *resolved, std::string *error);
bool read_estimator_dependency(
    const std::filesystem::path &main_config,
    const std::string &main_yaml, const std::string &key,
    std::string *contents, std::string *error);
bool configured_imu_rate(const std::filesystem::path &main_config,
                         const std::string &main_yaml, int *rate,
                         std::string *error);
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
bool parse_transform_matrices(
    const std::string &yaml, const std::string &key,
    std::vector<std::array<double, 16>> *matrices);
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
bool copy_config_dependency(
    const std::filesystem::path &main_config,
    const std::string &main_yaml, const std::string &key,
    const std::filesystem::path &destination, std::string *error);

} // namespace ovrs
