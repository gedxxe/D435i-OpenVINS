#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace ovrs {

struct StreamConfig {
  int width = 848;
  int height = 480;
  int camera_fps = 30;
  int gyro_fps = 200;
  int accel_fps = 250;
  bool emitter_enabled = false;
  bool auto_exposure = true;
  bool motion_correction_enabled = true;
  std::size_t imu_queue_size = 2048;
  std::size_t stereo_queue_size = 16;
  double stereo_tolerance_ms = 2.0;
  std::string serial;
};

std::vector<std::string> validate(const StreamConfig &config);
bool apply_stream_config_yaml(const std::string &yaml, StreamConfig *config,
                              std::string *error);
bool load_stream_config(const std::filesystem::path &path,
                        StreamConfig *config, std::string *error);
std::vector<std::string> stream_cli_value_options();
bool apply_stream_config_cli(int argc, char **argv, StreamConfig *config,
                             std::string *error);
std::string stream_config_yaml(const StreamConfig &config);

} // namespace ovrs
