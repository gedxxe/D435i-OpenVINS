#pragma once

#include "ovrs/trajectory_view.hpp"
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
    int trajectory_width = 960;
    int trajectory_height = 600;
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
  enum class DragMode {
    none,
    orbit,
    pan,
  };

  static void mouse_callback(int event, int x, int y, int flags, void *context);
  void handle_mouse(int event, int x, int y, int flags) noexcept;

  Options options_;
  std::mutex mutex_;
  std::optional<StereoFrame> latest_stereo_;
  std::optional<EstimatorState> latest_state_;
  std::deque<Vec3> trajectory_;
  std::optional<Vec3> first_position_;
  std::optional<Vec3> previous_position_;
  double total_path_length_m_ = 0.0;
  TrajectoryViewController view_controller_;
  TrajectoryViewFrame view_frame_;
  bool view_frame_initialized_ = false;
  bool fit_requested_ = false;
  DragMode drag_mode_ = DragMode::none;
  int last_mouse_x_ = 0;
  int last_mouse_y_ = 0;
  int viewport_width_ = 960;
  int viewport_height_ = 600;
  bool open_ = false;
};

} // namespace ovrs
