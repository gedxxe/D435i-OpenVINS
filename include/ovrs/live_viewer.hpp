#pragma once

#include "ovrs/types.hpp"

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <string>

namespace ovrs {

class LiveViewer {
public:
  struct Options {
    static constexpr std::size_t minimum_trajectory_points = 2;
    static constexpr std::size_t default_trajectory_points = 6000;
    static constexpr std::size_t maximum_allowed_trajectory_points = 1000000;

    std::size_t maximum_trajectory_points = default_trajectory_points;
    int trajectory_width = 640;
    int trajectory_height = 480;
    std::string calibration_state;
  };

  explicit LiveViewer(Options options);
  ~LiveViewer();

  LiveViewer(const LiveViewer &) = delete;
  LiveViewer &operator=(const LiveViewer &) = delete;

  bool open(std::string *error);
  void publish_stereo(const StereoFrame &frame);
  void publish_state(const EstimatorState &state);
  bool poll(std::string *error);
  void close() noexcept;

private:
  Options options_;
  std::mutex mutex_;
  std::optional<StereoFrame> latest_stereo_;
  std::optional<EstimatorState> latest_state_;
  std::deque<Vec3> trajectory_;
  std::optional<Vec3> first_position_;
  std::optional<Vec3> previous_position_;
  double total_path_length_m_ = 0.0;
  bool open_ = false;
};

} // namespace ovrs
