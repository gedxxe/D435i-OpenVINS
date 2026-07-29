#pragma once

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
std::string version_summary(const std::string &application,
                            const std::string &ceres,
                            const std::string &opencv,
                            const std::string &realsense);
bool validate_cli_arguments(
    int argc, char **argv,
    const std::vector<std::string> &value_options,
    const std::vector<std::string> &flag_options, std::string *error);

} // namespace ovrs
