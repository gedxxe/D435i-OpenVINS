#pragma once

#include "ovrs/config.hpp"
#include "ovrs/types.hpp"

#include <cctype>
#include <functional>
#include <memory>
#include <string>

namespace ovrs {

inline bool is_d435i_device_name(std::string name) {
  for (char &character : name) {
    character = static_cast<char>(
        std::toupper(static_cast<unsigned char>(character)));
  }
  return name.find("D435I") != std::string::npos;
}

class RealSenseSource {
public:
  using StereoCallback = std::function<void(StereoFrame)>;
  using MotionCallback = std::function<void(TimedVec3)>;

  struct Callbacks {
    StereoCallback stereo;
    MotionCallback gyro;
    MotionCallback accel;
  };

  struct Stats {
    std::uint64_t received_framesets = 0;
    std::uint64_t valid_stereo_pairs = 0;
    std::uint64_t received_gyro = 0;
    std::uint64_t received_accel = 0;
    std::uint64_t malformed_frames = 0;
    std::uint64_t dropped_camera_frames = 0;
    std::uint64_t rejected_timestamps = 0;
    std::uint64_t callback_errors = 0;
  };

  struct StreamSelection {
    bool stereo = true;
    bool motion = true;
  };

  explicit RealSenseSource(StreamConfig config);
  RealSenseSource(StreamConfig config, StreamSelection selection);
  ~RealSenseSource();
  RealSenseSource(const RealSenseSource &) = delete;
  RealSenseSource &operator=(const RealSenseSource &) = delete;

  bool start(const Callbacks &callbacks, std::string *error);
  void stop();
  bool running() const;
  bool disconnected() const;
  std::string failure() const;
  Stats stats() const;
  std::string device_report_yaml() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace ovrs
