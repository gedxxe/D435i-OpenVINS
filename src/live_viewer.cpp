#include "ovrs/live_viewer.hpp"
#include "ovrs/tracking_health.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ovrs {
namespace {

constexpr const char *stereo_window = "OVRS IR1 / IR2";
constexpr const char *trajectory_window = "OVRS trajectory (interactive global XYZ)";
constexpr double radians_to_degrees = 57.2957795130823208768;

cv::Mat image_view(const ImageFrame &frame) {
  const std::size_t required = static_cast<std::size_t>(frame.stride_bytes) * frame.height;
  if (frame.width <= 0 || frame.height <= 0 || frame.stride_bytes < frame.width ||
      frame.format != "Y8" || !frame.pixels || frame.pixels->size() < required) {
    throw std::runtime_error("viewer received a malformed Y8 image");
  }
  return {frame.height, frame.width, CV_8UC1, frame.pixels->data(),
          static_cast<std::size_t>(frame.stride_bytes)};
}

std::string position_text(const EstimatorState &state) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << "t=" << state.timestamp << "  p=["
      << state.position_world_m.x << ", " << state.position_world_m.y << ", "
      << state.position_world_m.z << "] m";
  return out.str();
}

double nice_grid_step(double requested) {
  if (!std::isfinite(requested) || requested <= 0.0) {
    return 0.1;
  }
  const double exponent = std::floor(std::log10(requested));
  const double magnitude = std::pow(10.0, exponent);
  const double normalized = requested / magnitude;
  double multiplier = 1.0;
  if (normalized > 5.0) {
    multiplier = 10.0;
  } else if (normalized > 2.0) {
    multiplier = 5.0;
  } else if (normalized > 1.0) {
    multiplier = 2.0;
  }
  return multiplier * magnitude;
}

int bounded_pixel_coordinate(double value) {
  if (!std::isfinite(value)) {
    throw std::runtime_error("viewer pixel coordinate became nonfinite");
  }
  constexpr double safe_limit = static_cast<double>(std::numeric_limits<int>::max() / 4);
  return static_cast<int>(std::lround(std::clamp(value, -safe_limit, safe_limit)));
}

cv::Scalar path_colour(double fraction) {
  const double bounded = std::clamp(fraction, 0.0, 1.0);
  return {
      255.0 * (1.0 - bounded),
      190.0 - 80.0 * bounded,
      70.0 + 185.0 * bounded,
  };
}

cv::Size drawable_size(const char *window, int fallback_width, int fallback_height) {
  const cv::Rect rectangle = cv::getWindowImageRect(window);
  if (rectangle.width >= 320 && rectangle.height >= 240 && rectangle.width <= 8192 &&
      rectangle.height <= 8192) {
    return rectangle.size();
  }
  return {fallback_width, fallback_height};
}

} // namespace

LiveViewer::LiveViewer(Options options) : options_(std::move(options)) {
  if (options_.maximum_trajectory_points < Options::minimum_trajectory_points ||
      options_.maximum_trajectory_points > Options::maximum_allowed_trajectory_points ||
      options_.trajectory_width < 320 || options_.trajectory_height < 240) {
    throw std::invalid_argument("invalid live-viewer safety bounds");
  }
  viewport_width_ = options_.trajectory_width;
  viewport_height_ = options_.trajectory_height;
}

LiveViewer::~LiveViewer() { close(); }

bool LiveViewer::open(std::string *error) {
  if (open_) {
    return true;
  }
  try {
    // Let the window remain freely resizable while HighGUI letterboxes the
    // combined stereo image instead of distorting its pixel aspect ratio.
    cv::namedWindow(stereo_window, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
    // Mark the partial resource as open so the catch path destroys it if
    // creation of the second window or callback registration fails.
    open_ = true;
    cv::namedWindow(trajectory_window, cv::WINDOW_NORMAL | cv::WINDOW_FREERATIO);
    cv::resizeWindow(stereo_window, 1120, 360);
    cv::resizeWindow(trajectory_window, options_.trajectory_width, options_.trajectory_height);
    cv::setMouseCallback(trajectory_window, &LiveViewer::mouse_callback, this);
    return true;
  } catch (const cv::Exception &exception) {
    if (error) {
      *error = std::string("cannot open the OpenCV viewer: ") + exception.what() +
               ". Start a graphical desktop session or run with --headless.";
    }
    close();
    return false;
  }
}

void LiveViewer::publish_stereo(const StereoFrame &frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_stereo_ = frame;
}

void LiveViewer::publish_state(const EstimatorState &state) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_state_ = state;
  if (!first_position_) {
    // WARMING_UP and DEGRADED poses remain available in the status overlay
    // and run logs, but must not define the viewer's visual origin.
    if (!state.healthy) {
      return;
    }
    first_position_ = state.position_world_m;
    previous_position_ = state.position_world_m;
    trajectory_.push_back(state.position_world_m);
    return;
  }
  if (previous_position_) {
    const double dx = state.position_world_m.x - previous_position_->x;
    const double dy = state.position_world_m.y - previous_position_->y;
    const double dz = state.position_world_m.z - previous_position_->z;
    const double step = std::hypot(dx, dy, dz);
    if (!std::isfinite(step) || !std::isfinite(total_path_length_m_ + step)) {
      throw std::runtime_error("viewer trajectory metric became nonfinite");
    }
    total_path_length_m_ += step;
  }
  previous_position_ = state.position_world_m;
  trajectory_.push_back(state.position_world_m);
  while (trajectory_.size() > options_.maximum_trajectory_points) {
    trajectory_.pop_front();
  }
}

void LiveViewer::mouse_callback(int event, int x, int y, int flags, void *context) {
  if (context != nullptr) {
    static_cast<LiveViewer *>(context)->handle_mouse(event, x, y, flags);
  }
}

void LiveViewer::handle_mouse(int event, int x, int y, int flags) noexcept {
  if (event == cv::EVENT_LBUTTONDBLCLK) {
    view_controller_.reset();
    fit_requested_ = true;
    drag_mode_ = DragMode::none;
    return;
  }
  if (event == cv::EVENT_LBUTTONDOWN) {
    drag_mode_ = DragMode::orbit;
    last_mouse_x_ = x;
    last_mouse_y_ = y;
    return;
  }
  if (event == cv::EVENT_MBUTTONDOWN || event == cv::EVENT_RBUTTONDOWN) {
    drag_mode_ = DragMode::pan;
    last_mouse_x_ = x;
    last_mouse_y_ = y;
    return;
  }
  if (event == cv::EVENT_LBUTTONUP || event == cv::EVENT_MBUTTONUP ||
      event == cv::EVENT_RBUTTONUP) {
    drag_mode_ = DragMode::none;
    return;
  }
  if (event == cv::EVENT_MOUSEMOVE && drag_mode_ != DragMode::none) {
    const double dx = static_cast<double>(x - last_mouse_x_);
    const double dy = static_cast<double>(y - last_mouse_y_);
    if (drag_mode_ == DragMode::orbit) {
      view_controller_.orbit(dx, dy);
    } else {
      view_controller_.pan(dx, dy);
    }
    last_mouse_x_ = x;
    last_mouse_y_ = y;
    return;
  }
  if (event == cv::EVENT_MOUSEWHEEL) {
    const double steps = static_cast<double>(cv::getMouseWheelDelta(flags)) / 120.0;
    view_controller_.zoom_at(steps, {static_cast<double>(x), static_cast<double>(y)},
                             {0.5 * viewport_width_, 0.5 * (viewport_height_ + 42.0)});
  }
}

bool LiveViewer::poll(std::string *error) {
  if (!open_) {
    if (error) {
      *error = "live viewer is not open";
    }
    return false;
  }
  std::optional<StereoFrame> stereo;
  std::optional<EstimatorState> state;
  std::deque<Vec3> trajectory;
  std::optional<Vec3> first_position;
  double total_path_length_m = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stereo = latest_stereo_;
    state = latest_state_;
    trajectory = trajectory_;
    first_position = first_position_;
    total_path_length_m = total_path_length_m_;
  }

  try {
    if (stereo) {
      cv::Mat cameras;
      cv::hconcat(image_view(stereo->camera0), image_view(stereo->camera1), cameras);
      cv::Mat display;
      cv::cvtColor(cameras, display, cv::COLOR_GRAY2BGR);
      const std::string status =
          state ? position_text(*state) : "Waiting for OpenVINS initialization";
      const cv::Scalar status_colour =
          state && !state->healthy ? cv::Scalar(40, 40, 255)
                                   : cv::Scalar(0, 255, 0);
      cv::putText(display, status, {16, 28},
                  cv::FONT_HERSHEY_SIMPLEX, 0.65, status_colour, 2,
                  cv::LINE_AA);
      if (!options_.calibration_state.empty()) {
        cv::putText(display, options_.calibration_state, {16, 56}, cv::FONT_HERSHEY_SIMPLEX, 0.65,
                    {0, 215, 255}, 2, cv::LINE_AA);
      }
      if (state && state->tracking_health_gate_enabled) {
        std::ostringstream tracking;
        tracking << "VISUAL SUPPORT "
                 << tracking_health_status_name(
                        state->tracking_health_status)
                 << "  features " << state->visual_support_features;
        cv::putText(display, tracking.str(), {16, 84},
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, status_colour, 2,
                    cv::LINE_AA);
      }
      cv::imshow(stereo_window, display);
    }

    const cv::Size trajectory_size =
        drawable_size(trajectory_window, options_.trajectory_width, options_.trajectory_height);
    viewport_width_ = trajectory_size.width;
    viewport_height_ = trajectory_size.height;
    cv::Mat plot(viewport_height_, viewport_width_, CV_8UC3, cv::Scalar(17, 20, 26));
    if (!trajectory.empty()) {
      Vec3 minimum = trajectory.front();
      Vec3 maximum = trajectory.front();
      for (const auto &point : trajectory) {
        minimum.x = std::min(minimum.x, point.x);
        minimum.y = std::min(minimum.y, point.y);
        minimum.z = std::min(minimum.z, point.z);
        maximum.x = std::max(maximum.x, point.x);
        maximum.y = std::max(maximum.y, point.y);
        maximum.z = std::max(maximum.z, point.z);
      }
      const double raw_x_span = maximum.x - minimum.x;
      const double raw_y_span = maximum.y - minimum.y;
      const double raw_z_span = maximum.z - minimum.z;
      if (!std::isfinite(raw_x_span) || !std::isfinite(raw_y_span) || !std::isfinite(raw_z_span)) {
        throw std::runtime_error("viewer trajectory extent became nonfinite");
      }
      if (!view_frame_initialized_ || fit_requested_) {
        view_frame_.fit(minimum, maximum);
        view_controller_.fit();
        view_frame_initialized_ = true;
        fit_requested_ = false;
      }

      const Vec3 axis_origin = first_position.value_or(trajectory.front());
      const double displayed_scale =
          view_frame_.base_scale(viewport_width_, viewport_height_) * view_controller_.zoom();
      const double axis_length = 44.0 / std::max(displayed_scale, 1e-6);
      const Vec3 axis_x = {axis_origin.x + axis_length, axis_origin.y, axis_origin.z};
      const Vec3 axis_y = {axis_origin.x, axis_origin.y + axis_length, axis_origin.z};
      const Vec3 axis_z = {axis_origin.x, axis_origin.y, axis_origin.z + axis_length};

      const double visible_world_span = view_frame_.world_span_m() / view_controller_.zoom();
      const double grid_step = nice_grid_step(visible_world_span / 10.0);
      const double grid_radius = visible_world_span;
      const Vec3 focus = view_frame_.focus_world_m();
      const double grid_min_x =
          axis_origin.x +
          std::floor((focus.x - grid_radius - axis_origin.x) / grid_step) * grid_step;
      const double grid_max_x =
          axis_origin.x +
          std::ceil((focus.x + grid_radius - axis_origin.x) / grid_step) * grid_step;
      const double grid_min_y =
          axis_origin.y +
          std::floor((focus.y - grid_radius - axis_origin.y) / grid_step) * grid_step;
      const double grid_max_y =
          axis_origin.y +
          std::ceil((focus.y + grid_radius - axis_origin.y) / grid_step) * grid_step;
      const int grid_x_count =
          std::min(40, static_cast<int>(std::lround((grid_max_x - grid_min_x) / grid_step)));
      const int grid_y_count =
          std::min(40, static_cast<int>(std::lround((grid_max_y - grid_min_y) / grid_step)));

      const auto pixel = [&](const Vec3 &point) {
        const ViewPoint2d screen =
            view_frame_.screen_point(view_controller_, point, viewport_width_, viewport_height_);
        return cv::Point(bounded_pixel_coordinate(screen.x), bounded_pixel_coordinate(screen.y));
      };

      for (int index = 0; index <= grid_x_count; ++index) {
        const double x = grid_min_x + index * grid_step;
        const bool origin_line = std::abs(x - axis_origin.x) < 0.25 * grid_step;
        cv::line(plot, pixel({x, grid_min_y, axis_origin.z}), pixel({x, grid_max_y, axis_origin.z}),
                 origin_line ? cv::Scalar(58, 68, 80) : cv::Scalar(31, 38, 48), origin_line ? 2 : 1,
                 cv::LINE_AA);
      }
      for (int index = 0; index <= grid_y_count; ++index) {
        const double y = grid_min_y + index * grid_step;
        const bool origin_line = std::abs(y - axis_origin.y) < 0.25 * grid_step;
        cv::line(plot, pixel({grid_min_x, y, axis_origin.z}), pixel({grid_max_x, y, axis_origin.z}),
                 origin_line ? cv::Scalar(58, 68, 80) : cv::Scalar(31, 38, 48), origin_line ? 2 : 1,
                 cv::LINE_AA);
      }

      cv::arrowedLine(plot, pixel(axis_origin), pixel(axis_x), {80, 95, 255}, 1, cv::LINE_AA, 0,
                      0.12);
      cv::arrowedLine(plot, pixel(axis_origin), pixel(axis_y), {100, 225, 105}, 1, cv::LINE_AA, 0,
                      0.12);
      cv::arrowedLine(plot, pixel(axis_origin), pixel(axis_z), {255, 175, 65}, 1, cv::LINE_AA, 0,
                      0.12);
      cv::putText(plot, "X", pixel(axis_x), cv::FONT_HERSHEY_SIMPLEX, 0.42, {80, 95, 255}, 1,
                  cv::LINE_AA);
      cv::putText(plot, "Y", pixel(axis_y), cv::FONT_HERSHEY_SIMPLEX, 0.42, {100, 225, 105}, 1,
                  cv::LINE_AA);
      cv::putText(plot, "Z", pixel(axis_z), cv::FONT_HERSHEY_SIMPLEX, 0.42, {255, 175, 65}, 1,
                  cv::LINE_AA);

      for (std::size_t index = 1; index < trajectory.size(); ++index) {
        const double fraction =
            static_cast<double>(index) / static_cast<double>(trajectory.size() - 1);
        cv::line(plot, pixel(trajectory[index - 1]), pixel(trajectory[index]),
                 path_colour(fraction), 2, cv::LINE_AA);
      }
      cv::circle(plot, pixel(axis_origin), 4, {255, 190, 75}, 1, cv::LINE_AA);
      cv::circle(plot, pixel(trajectory.back()), 6, {245, 245, 245}, 1, cv::LINE_AA);
      cv::circle(plot, pixel(trajectory.back()), 3, {65, 105, 255}, cv::FILLED, cv::LINE_AA);

      cv::rectangle(plot, {0, 0}, {plot.cols, 78}, cv::Scalar(12, 15, 21), cv::FILLED);
      std::ostringstream extent;
      extent << std::fixed << std::setprecision(2) << "WORLD LOCKED   XYZ span [" << raw_x_span
             << ", " << raw_y_span << ", " << raw_z_span << "] m   grid " << grid_step << " m";
      cv::putText(plot, extent.str(), {14, 23}, cv::FONT_HERSHEY_SIMPLEX, 0.50, {210, 220, 232}, 1,
                  cv::LINE_AA);
      if (state && first_position) {
        const double dx = state->position_world_m.x - first_position->x;
        const double dy = state->position_world_m.y - first_position->y;
        const double dz = state->position_world_m.z - first_position->z;
        const double displacement = std::hypot(dx, dy, dz);
        if (!std::isfinite(displacement)) {
          throw std::runtime_error("viewer displacement became nonfinite");
        }
        std::ostringstream metrics;
        metrics << std::fixed << std::setprecision(2) << "estimated path "
                << total_path_length_m << " m   displacement " << displacement
                << " m   p=[" << state->position_world_m.x << ", "
                << state->position_world_m.y << ", "
                << state->position_world_m.z << "]";
        cv::putText(plot, metrics.str(), {14, 47}, cv::FONT_HERSHEY_SIMPLEX, 0.50, {210, 220, 232},
                    1, cv::LINE_AA);
      }
      std::ostringstream view_status;
      if (state) {
        if (state->msckf_update_quality_available) {
          view_status << "MSCKF " << state->msckf_accepted_features << "/"
                      << state->msckf_candidate_features << " ("
                      << std::fixed << std::setprecision(0)
                      << 100.0 * state->msckf_acceptance_ratio << "%)   ";
        } else {
          view_status << "MSCKF n/a   ";
        }
        view_status << "SLAM " << state->slam_features << "   SUPPORT "
                    << state->visual_support_features << "   "
                    << tracking_health_status_name(
                           state->tracking_health_status)
                    << "   |   ";
      }
      view_status << std::fixed << std::setprecision(1) << "yaw "
                  << view_controller_.yaw_rad() * radians_to_degrees << " deg   elevation "
                  << view_controller_.elevation_rad() * radians_to_degrees << " deg   zoom "
                  << std::setprecision(2) << view_controller_.zoom() << "x";
      const cv::Scalar view_status_colour =
          state && !state->healthy ? cv::Scalar(70, 100, 255)
                                   : cv::Scalar(135, 190, 235);
      cv::putText(plot, view_status.str(), {14, 69},
                  cv::FONT_HERSHEY_SIMPLEX, 0.46, view_status_colour, 1,
                  cv::LINE_AA);
    } else {
      const std::string waiting =
          state && state->tracking_health_gate_enabled
              ? "Waiting for visual-support gate to anchor viewer origin"
              : "Waiting for initialized pose";
      cv::putText(plot, waiting, {20, 40}, cv::FONT_HERSHEY_SIMPLEX,
                  0.7, {220, 220, 220}, 1, cv::LINE_AA);
    }

    cv::rectangle(plot, {0, plot.rows - 28}, {plot.cols, plot.rows}, cv::Scalar(12, 15, 21),
                  cv::FILLED);
    cv::putText(plot, "Orbit: left-drag   Pan: middle/right-drag   Zoom: wheel   Fit: F   Reset: R",
                {14, plot.rows - 9}, cv::FONT_HERSHEY_SIMPLEX, 0.43, {165, 177, 195}, 1,
                cv::LINE_AA);
    cv::imshow(trajectory_window, plot);

    const int key = cv::waitKey(1) & 0xff;
    if (key == 27 || key == 'q' || key == 'Q') {
      return false;
    }
    if (key == 'f' || key == 'F') {
      fit_requested_ = true;
    } else if (key == 'r' || key == 'R' || key == '0') {
      view_controller_.reset();
      fit_requested_ = true;
    }
    const double stereo_visible = cv::getWindowProperty(stereo_window, cv::WND_PROP_VISIBLE);
    const double trajectory_visible =
        cv::getWindowProperty(trajectory_window, cv::WND_PROP_VISIBLE);
    // Some HighGUI backends report -1 when WND_PROP_VISIBLE is unsupported.
    // Only an explicit zero means the user closed a window.
    return stereo_visible != 0.0 && trajectory_visible != 0.0;
  } catch (const cv::Exception &exception) {
    if (error) {
      *error = std::string("OpenCV viewer failed: ") + exception.what();
    }
    return false;
  } catch (const std::exception &exception) {
    if (error) {
      *error = exception.what();
    }
    return false;
  }
}

void LiveViewer::close() noexcept {
  if (!open_) {
    return;
  }
  try {
    cv::setMouseCallback(trajectory_window, nullptr, nullptr);
  } catch (...) {
  }
  try {
    cv::destroyWindow(stereo_window);
  } catch (...) {
  }
  try {
    cv::destroyWindow(trajectory_window);
  } catch (...) {
  }
  try {
    cv::waitKey(1);
  } catch (...) {
  }
  drag_mode_ = DragMode::none;
  open_ = false;
}

} // namespace ovrs
