#include "ovrs/config.hpp"
#include "ovrs/yaml_utils.hpp"

#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>

namespace {

std::size_t parse_size_exact(const std::string &text,
                             const std::string &name) {
  const auto value = ovrs::parse_uint64_strict(text, name);
  if (value > std::numeric_limits<std::size_t>::max()) {
    throw std::invalid_argument(name + " must be a nonnegative integer");
  }
  return static_cast<std::size_t>(value);
}

std::optional<std::string> cli_value(int argc, char **argv,
                                     const std::string &option) {
  std::optional<std::string> value;
  for (int i = 1; i < argc; ++i) {
    if (argv[i] != option) {
      continue;
    }
    if (value) {
      throw std::invalid_argument(option + " may be specified only once");
    }
    if (i + 1 >= argc) {
      throw std::invalid_argument(option + " requires a value");
    }
    value = argv[i + 1];
  }
  return value;
}

bool safe_serial(const std::string &serial) {
  if (serial.empty()) {
    return true;
  }
  for (const unsigned char character : serial) {
    if (!std::isalnum(character) && character != '-' && character != '_' &&
        character != '.') {
      return false;
    }
  }
  return true;
}

} // namespace

namespace ovrs {

std::vector<std::string> validate(const StreamConfig &config) {
  std::vector<std::string> errors;
  if (config.width <= 0 || config.height <= 0) {
    errors.emplace_back("camera resolution must be positive");
  }
  if (config.width > 4096 || config.height > 4096) {
    errors.emplace_back("camera resolution exceeds the supported safety bound");
  }
  if (config.camera_fps <= 0 || config.gyro_fps <= 0 ||
      config.accel_fps <= 0) {
    errors.emplace_back("stream rates must be positive");
  }
  if (config.gyro_sensitivity < 0 || config.gyro_sensitivity > 4) {
    errors.emplace_back("gyro sensitivity must be an index in [0,4]");
  }
  if (!std::isfinite(config.gyro_scale_factor) ||
      config.gyro_scale_factor <= 0.0 ||
      config.gyro_scale_factor > 100.0) {
    errors.emplace_back("gyro scale factor must be finite and in (0,100]");
  }
  if (config.imu_queue_size < 2 || config.stereo_queue_size < 1 ||
      config.imu_queue_size > 1000000 ||
      config.stereo_queue_size > 1000000) {
    errors.emplace_back("queue sizes are outside the supported safety bounds");
  }
  if (!std::isfinite(config.stereo_tolerance_ms) ||
      config.stereo_tolerance_ms < 0.0 ||
      config.stereo_tolerance_ms > 20.0) {
    errors.emplace_back("stereo tolerance must be in [0,20] ms");
  }
  if (!safe_serial(config.serial)) {
    errors.emplace_back(
        "serial may contain only letters, digits, dot, dash, and underscore");
  }
  return errors;
}

bool apply_stream_config_yaml(const std::string &yaml, StreamConfig *config,
                              std::string *error) {
  if (!config) {
    if (error) {
      *error = "stream configuration destination is null";
    }
    return false;
  }
  try {
    for (const char *key :
         {"width", "height", "camera_fps", "gyro_fps",
          "gyro_sensitivity", "gyro_scale_factor",
          "accelerometer_fps", "emitter_enabled", "auto_exposure",
          "motion_correction_enabled", "global_time_enabled",
          "imu_queue_size", "stereo_queue_size", "stereo_tolerance_ms",
          "serial"}) {
      if (simple_yaml_key_count(yaml, key) > 1) {
        throw std::invalid_argument(
            std::string("duplicate stream key: ") + key);
      }
    }
    const auto apply_int = [&](const std::string &key, int *value) {
      const auto text = simple_yaml_scalar(yaml, key);
      if (!text.empty()) {
        *value = parse_int_strict(text, key);
      }
    };
    const auto apply_size = [&](const std::string &key, std::size_t *value) {
      const auto text = simple_yaml_scalar(yaml, key);
      if (!text.empty()) {
        *value = parse_size_exact(text, key);
      }
    };
    const auto apply_bool = [&](const std::string &key, bool *value) {
      const auto text = simple_yaml_scalar(yaml, key);
      if (text.empty()) {
        return;
      }
      if (text == "true") {
        *value = true;
      } else if (text == "false") {
        *value = false;
      } else {
        throw std::invalid_argument(key + " must be true or false");
      }
    };

    apply_int("width", &config->width);
    apply_int("height", &config->height);
    apply_int("camera_fps", &config->camera_fps);
    apply_int("gyro_fps", &config->gyro_fps);
    apply_int("gyro_sensitivity", &config->gyro_sensitivity);
    const auto gyro_scale = simple_yaml_scalar(yaml, "gyro_scale_factor");
    if (!gyro_scale.empty()) {
      config->gyro_scale_factor =
          parse_double_strict(gyro_scale, "gyro_scale_factor");
    }
    apply_int("accelerometer_fps", &config->accel_fps);
    apply_bool("emitter_enabled", &config->emitter_enabled);
    apply_bool("auto_exposure", &config->auto_exposure);
    apply_bool("motion_correction_enabled",
               &config->motion_correction_enabled);
    apply_bool("global_time_enabled", &config->global_time_enabled);
    apply_size("imu_queue_size", &config->imu_queue_size);
    apply_size("stereo_queue_size", &config->stereo_queue_size);
    const auto tolerance = simple_yaml_scalar(yaml, "stereo_tolerance_ms");
    if (!tolerance.empty()) {
      config->stereo_tolerance_ms =
          parse_double_strict(tolerance, "stereo_tolerance_ms");
    }
    const auto serial = simple_yaml_scalar(yaml, "serial");
    if (!serial.empty()) {
      config->serial = serial;
    }
  } catch (const std::exception &e) {
    if (error) {
      *error = std::string("invalid stream configuration: ") + e.what();
    }
    return false;
  }
  const auto errors = validate(*config);
  if (!errors.empty()) {
    if (error) {
      *error = "invalid stream configuration: " + errors.front();
    }
    return false;
  }
  return true;
}

bool load_stream_config(const std::filesystem::path &path,
                        StreamConfig *config, std::string *error) {
  std::ifstream input(path);
  if (!input) {
    if (error) {
      *error = "cannot open stream configuration: " + path.string();
    }
    return false;
  }
  const std::string yaml((std::istreambuf_iterator<char>(input)),
                         std::istreambuf_iterator<char>());
  return apply_stream_config_yaml(yaml, config, error);
}

std::vector<std::string> stream_cli_value_options() {
  return {"--serial",
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
}

bool apply_stream_config_cli(int argc, char **argv, StreamConfig *config,
                             std::string *error) {
  if (!config) {
    if (error) {
      *error = "stream configuration destination is null";
    }
    return false;
  }
  try {
    if (const auto value = cli_value(argc, argv, "--serial")) {
      config->serial = *value;
    }
    if (const auto value = cli_value(argc, argv, "--width")) {
      config->width = parse_int_strict(*value, "--width");
    }
    if (const auto value = cli_value(argc, argv, "--height")) {
      config->height = parse_int_strict(*value, "--height");
    }
    if (const auto value = cli_value(argc, argv, "--camera-fps")) {
      config->camera_fps = parse_int_strict(*value, "--camera-fps");
    }
    if (const auto value = cli_value(argc, argv, "--gyro-fps")) {
      config->gyro_fps = parse_int_strict(*value, "--gyro-fps");
    }
    if (const auto value = cli_value(argc, argv, "--gyro-sensitivity")) {
      config->gyro_sensitivity =
          parse_int_strict(*value, "--gyro-sensitivity");
    }
    if (const auto value = cli_value(argc, argv, "--gyro-scale-factor")) {
      config->gyro_scale_factor =
          parse_double_strict(*value, "--gyro-scale-factor");
    }
    if (const auto value = cli_value(argc, argv, "--accel-fps")) {
      config->accel_fps = parse_int_strict(*value, "--accel-fps");
    }
    if (const auto value = cli_value(argc, argv, "--emitter")) {
      if (*value != "on" && *value != "off") {
        throw std::invalid_argument("--emitter must be on or off");
      }
      config->emitter_enabled = *value == "on";
    }
    if (const auto value = cli_value(argc, argv, "--auto-exposure")) {
      if (*value != "on" && *value != "off") {
        throw std::invalid_argument("--auto-exposure must be on or off");
      }
      config->auto_exposure = *value == "on";
    }
    if (const auto value = cli_value(argc, argv, "--motion-correction")) {
      if (*value != "on" && *value != "off") {
        throw std::invalid_argument("--motion-correction must be on or off");
      }
      config->motion_correction_enabled = *value == "on";
    }
    if (const auto value = cli_value(argc, argv, "--global-time")) {
      if (*value != "on" && *value != "off") {
        throw std::invalid_argument("--global-time must be on or off");
      }
      config->global_time_enabled = *value == "on";
    }
    if (const auto value = cli_value(argc, argv, "--imu-queue")) {
      config->imu_queue_size = parse_size_exact(*value, "--imu-queue");
    }
    if (const auto value = cli_value(argc, argv, "--stereo-queue")) {
      config->stereo_queue_size =
          parse_size_exact(*value, "--stereo-queue");
    }
    if (const auto value =
            cli_value(argc, argv, "--stereo-tolerance-ms")) {
      config->stereo_tolerance_ms =
          parse_double_strict(*value, "--stereo-tolerance-ms");
    }
  } catch (const std::exception &exception) {
    if (error) {
      *error = std::string("invalid stream command line: ") + exception.what();
    }
    return false;
  }
  const auto errors = validate(*config);
  if (!errors.empty()) {
    if (error) {
      *error = "invalid stream configuration: " + errors.front();
    }
    return false;
  }
  return true;
}

std::string stream_config_yaml(const StreamConfig &config) {
  std::ostringstream output;
  output << std::setprecision(std::numeric_limits<double>::max_digits10)
         << "%YAML:1.0\n"
         << "serial: \"" << config.serial << "\"\n"
         << "width: " << config.width << '\n'
         << "height: " << config.height << '\n'
         << "camera_fps: " << config.camera_fps << '\n'
         << "gyro_fps: " << config.gyro_fps << '\n'
         << "gyro_sensitivity: " << config.gyro_sensitivity << '\n'
         << "gyro_scale_factor: " << config.gyro_scale_factor << '\n'
         << "accelerometer_fps: " << config.accel_fps << '\n'
         << "emitter_enabled: "
         << (config.emitter_enabled ? "true" : "false") << '\n'
         << "auto_exposure: "
         << (config.auto_exposure ? "true" : "false") << '\n'
         << "motion_correction_enabled: "
         << (config.motion_correction_enabled ? "true" : "false") << '\n'
         << "global_time_enabled: "
         << (config.global_time_enabled ? "true" : "false") << '\n'
         << "imu_queue_size: " << config.imu_queue_size << '\n'
         << "stereo_queue_size: " << config.stereo_queue_size << '\n'
         << "stereo_tolerance_ms: " << config.stereo_tolerance_ms << '\n';
  return output.str();
}

} // namespace ovrs
