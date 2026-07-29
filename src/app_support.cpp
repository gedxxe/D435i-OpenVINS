#include "ovrs/app_support.hpp"

#include "ovrs/version.hpp"
#include "ovrs/yaml_utils.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

namespace ovrs {

volatile std::sig_atomic_t &stop_requested_flag() {
  static volatile std::sig_atomic_t value = 0;
  return value;
}

bool stop_requested() { return stop_requested_flag() != 0; }

void request_stop() { stop_requested_flag() = 1; }

void signal_handler(int) { stop_requested_flag() = 1; }

void install_signal_handlers() {
  stop_requested_flag() = 0;
  std::signal(SIGINT, signal_handler);
#ifdef SIGTERM
  std::signal(SIGTERM, signal_handler);
#endif
}

bool wait_until_or_stop(
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

std::string utc_timestamp() {
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

bool write_text(const std::filesystem::path &path,
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

std::string value_after(int argc, char **argv,
                        const std::string &option,
                        const std::string &fallback) {
  for (int i = 1; i + 1 < argc; ++i) {
    if (argv[i] == option) {
      return argv[i + 1];
    }
  }
  return fallback;
}

bool has_flag(int argc, char **argv, const std::string &flag) {
  for (int i = 1; i < argc; ++i) {
    if (argv[i] == flag) {
      return true;
    }
  }
  return false;
}

std::size_t bounded_size_option(
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

std::string version_summary(const std::string &application,
                            const std::string &ceres,
                            const std::string &opencv,
                            const std::string &realsense) {
  return application + " " + project_version + "\nOpenVINS " +
         openvins_tag + " (" + openvins_commit + ")\nCeres " +
         ceres + "\nOpenCV " + opencv +
         "\nlibrealsense " + realsense + "\nSource fingerprint " +
         source_fingerprint + "\n";
}

bool validate_cli_arguments(
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

} // namespace ovrs
