#include "ovrs/live_viewer.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ovrs {
namespace {

constexpr const char *stereo_window = "OVRS IR1 / IR2";
constexpr const char *trajectory_window =
    "OVRS trajectory (global XYZ isometric)";

struct IsometricPoint {
  double horizontal = 0.0;
  double vertical = 0.0;
};

IsometricPoint isometric_projection(const Vec3 &point) {
  constexpr double inverse_sqrt_two = 0.7071067811865475;
  constexpr double inverse_sqrt_six = 0.4082482904638631;
  return {(point.x - point.y) * inverse_sqrt_two,
          (point.x + point.y - 2.0 * point.z) * inverse_sqrt_six};
}

cv::Mat image_view(const ImageFrame &frame) {
  const std::size_t required =
      static_cast<std::size_t>(frame.stride_bytes) * frame.height;
  if (frame.width <= 0 || frame.height <= 0 ||
      frame.stride_bytes < frame.width || frame.format != "Y8" ||
      !frame.pixels || frame.pixels->size() < required) {
    throw std::runtime_error("viewer received a malformed Y8 image");
  }
  return {frame.height, frame.width, CV_8UC1, frame.pixels->data(),
          static_cast<std::size_t>(frame.stride_bytes)};
}

std::string position_text(const EstimatorState &state) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << "t=" << state.timestamp
      << "  p=[" << state.position_world_m.x << ", "
      << state.position_world_m.y << ", " << state.position_world_m.z
      << "] m";
  return out.str();
}

} // namespace

LiveViewer::LiveViewer(Options options) : options_(std::move(options)) {
  if (options_.maximum_trajectory_points <
          Options::minimum_trajectory_points ||
      options_.maximum_trajectory_points >
          Options::maximum_allowed_trajectory_points ||
      options_.trajectory_width < 320 || options_.trajectory_height < 240) {
    throw std::invalid_argument("invalid live-viewer safety bounds");
  }
}

LiveViewer::~LiveViewer() { close(); }

bool LiveViewer::open(std::string *error) {
  if (open_) {
    return true;
  }
  try {
    cv::namedWindow(stereo_window, cv::WINDOW_NORMAL);
    // Mark the partial resource as open so the catch path destroys it if
    // creation of the second window fails.
    open_ = true;
    cv::namedWindow(trajectory_window, cv::WINDOW_NORMAL);
    cv::resizeWindow(trajectory_window, options_.trajectory_width,
                     options_.trajectory_height);
    return true;
  } catch (const cv::Exception &exception) {
    if (error) {
      *error = std::string("cannot open the OpenCV viewer: ") +
               exception.what() +
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
    first_position_ = state.position_world_m;
  }
  if (previous_position_) {
    const double dx = state.position_world_m.x - previous_position_->x;
    const double dy = state.position_world_m.y - previous_position_->y;
    const double dz = state.position_world_m.z - previous_position_->z;
    const double step = std::hypot(dx, dy, dz);
    if (!std::isfinite(step) ||
        !std::isfinite(total_path_length_m_ + step)) {
      throw std::runtime_error(
          "viewer trajectory metric became nonfinite");
    }
    total_path_length_m_ += step;
  }
  previous_position_ = state.position_world_m;
  trajectory_.push_back(state.position_world_m);
  while (trajectory_.size() > options_.maximum_trajectory_points) {
    trajectory_.pop_front();
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
      cv::hconcat(image_view(stereo->camera0), image_view(stereo->camera1),
                  cameras);
      cv::Mat display;
      cv::cvtColor(cameras, display, cv::COLOR_GRAY2BGR);
      const std::string status =
          state ? position_text(*state) : "Waiting for OpenVINS initialization";
      cv::putText(display, status, {16, 28}, cv::FONT_HERSHEY_SIMPLEX, 0.65,
                  {0, 255, 0}, 2, cv::LINE_AA);
      if (!options_.calibration_state.empty()) {
        cv::putText(display, options_.calibration_state, {16, 56},
                    cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 215, 255}, 2,
                    cv::LINE_AA);
      }
      cv::imshow(stereo_window, display);
    }

    cv::Mat plot(options_.trajectory_height, options_.trajectory_width,
                 CV_8UC3, cv::Scalar(24, 24, 24));
    const int margin = 36;
    if (!trajectory.empty()) {
      double minimum_x = trajectory.front().x;
      double maximum_x = trajectory.front().x;
      double minimum_y = trajectory.front().y;
      double maximum_y = trajectory.front().y;
      double minimum_z = trajectory.front().z;
      double maximum_z = trajectory.front().z;
      for (const auto &point : trajectory) {
        minimum_x = std::min(minimum_x, point.x);
        maximum_x = std::max(maximum_x, point.x);
        minimum_y = std::min(minimum_y, point.y);
        maximum_y = std::max(maximum_y, point.y);
        minimum_z = std::min(minimum_z, point.z);
        maximum_z = std::max(maximum_z, point.z);
      }
      const double raw_x_span = maximum_x - minimum_x;
      const double raw_y_span = maximum_y - minimum_y;
      const double raw_z_span = maximum_z - minimum_z;
      if (!std::isfinite(raw_x_span) || !std::isfinite(raw_y_span) ||
          !std::isfinite(raw_z_span)) {
        throw std::runtime_error("viewer trajectory extent became nonfinite");
      }

      const double world_span =
          std::max({raw_x_span, raw_y_span, raw_z_span, 1e-3});
      const double axis_length = std::max(0.1, 0.2 * world_span);
      const Vec3 axis_origin = trajectory.front();
      const Vec3 axis_x = {axis_origin.x + axis_length, axis_origin.y,
                           axis_origin.z};
      const Vec3 axis_y = {axis_origin.x, axis_origin.y + axis_length,
                           axis_origin.z};
      const Vec3 axis_z = {axis_origin.x, axis_origin.y,
                           axis_origin.z + axis_length};

      IsometricPoint minimum = isometric_projection(trajectory.front());
      IsometricPoint maximum = minimum;
      const auto include_projection = [&](const Vec3 &point) {
        const auto projected = isometric_projection(point);
        minimum.horizontal =
            std::min(minimum.horizontal, projected.horizontal);
        maximum.horizontal =
            std::max(maximum.horizontal, projected.horizontal);
        minimum.vertical = std::min(minimum.vertical, projected.vertical);
        maximum.vertical = std::max(maximum.vertical, projected.vertical);
      };
      for (const auto &point : trajectory) {
        include_projection(point);
      }
      include_projection(axis_x);
      include_projection(axis_y);
      include_projection(axis_z);

      const double horizontal_span =
          std::max(maximum.horizontal - minimum.horizontal, 1e-3);
      const double vertical_span =
          std::max(maximum.vertical - minimum.vertical, 1e-3);
      const double horizontal_scale =
          static_cast<double>(options_.trajectory_width - 2 * margin) /
          horizontal_span;
      const double vertical_scale =
          static_cast<double>(options_.trajectory_height - 2 * margin) /
          vertical_span;
      const double scale = std::min(horizontal_scale, vertical_scale);
      const double centre_horizontal =
          0.5 * (minimum.horizontal + maximum.horizontal);
      const double centre_vertical =
          0.5 * (minimum.vertical + maximum.vertical);
      const auto pixel = [&](const Vec3 &point) {
        const auto projected = isometric_projection(point);
        return cv::Point(
            static_cast<int>(std::lround(
                0.5 * options_.trajectory_width +
                (projected.horizontal - centre_horizontal) * scale)),
            static_cast<int>(std::lround(
                0.5 * options_.trajectory_height -
                (projected.vertical - centre_vertical) * scale)));
      };

      cv::arrowedLine(plot, pixel(axis_origin), pixel(axis_x), {0, 0, 255}, 2,
                      cv::LINE_AA, 0, 0.08);
      cv::arrowedLine(plot, pixel(axis_origin), pixel(axis_y), {0, 220, 0}, 2,
                      cv::LINE_AA, 0, 0.08);
      cv::arrowedLine(plot, pixel(axis_origin), pixel(axis_z), {255, 120, 0}, 2,
                      cv::LINE_AA, 0, 0.08);
      cv::putText(plot, "X", pixel(axis_x), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                  {0, 0, 255}, 2, cv::LINE_AA);
      cv::putText(plot, "Y", pixel(axis_y), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                  {0, 220, 0}, 2, cv::LINE_AA);
      cv::putText(plot, "Z", pixel(axis_z), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                  {255, 120, 0}, 2, cv::LINE_AA);

      for (std::size_t index = 1; index < trajectory.size(); ++index) {
        cv::line(plot, pixel(trajectory[index - 1]),
                 pixel(trajectory[index]), {0, 215, 255}, 2, cv::LINE_AA);
      }
      cv::circle(plot, pixel(trajectory.front()), 5, {255, 160, 0},
                 cv::FILLED, cv::LINE_AA);
      cv::circle(plot, pixel(trajectory.back()), 5, {0, 0, 255}, cv::FILLED,
                 cv::LINE_AA);
      std::ostringstream extent;
      extent << std::fixed << std::setprecision(2) << "XYZ span ["
             << raw_x_span << ", " << raw_y_span << ", " << raw_z_span
             << "] m";
      cv::putText(plot, extent.str(), {12, 24}, cv::FONT_HERSHEY_SIMPLEX,
                  0.55, {220, 220, 220}, 1, cv::LINE_AA);
      if (state && first_position) {
        const double dx = state->position_world_m.x - first_position->x;
        const double dy = state->position_world_m.y - first_position->y;
        const double dz = state->position_world_m.z - first_position->z;
        const double displacement = std::hypot(dx, dy, dz);
        if (!std::isfinite(displacement)) {
          throw std::runtime_error(
              "viewer displacement became nonfinite");
        }
        std::ostringstream metrics;
        metrics << std::fixed << std::setprecision(2) << "3D path "
                << total_path_length_m << " m, displacement " << displacement
                << " m";
        cv::putText(plot, metrics.str(), {12, 48},
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, {220, 220, 220}, 1,
                    cv::LINE_AA);
      }
    } else {
      cv::putText(plot, "Waiting for initialized pose", {20, 40},
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, {220, 220, 220}, 1,
                  cv::LINE_AA);
    }
    cv::putText(plot, "Esc or q: clean shutdown", {12, plot.rows - 12},
                cv::FONT_HERSHEY_SIMPLEX, 0.5, {180, 180, 180}, 1,
                cv::LINE_AA);
    cv::imshow(trajectory_window, plot);

    const int key = cv::waitKey(1) & 0xff;
    if (key == 27 || key == 'q' || key == 'Q') {
      return false;
    }
    const double stereo_visible =
        cv::getWindowProperty(stereo_window, cv::WND_PROP_VISIBLE);
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
  open_ = false;
}

} // namespace ovrs
