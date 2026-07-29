#include "ovrs/trajectory_view.hpp"

#include <algorithm>
#include <cmath>

namespace ovrs {
namespace {

constexpr double orbit_radians_per_pixel = 0.006;
constexpr double full_turn_rad = 6.28318530717958647692;
constexpr double zoom_per_wheel_step = 1.15;

bool finite_point(const ViewPoint2d &point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

} // namespace

ViewPoint2d TrajectoryViewController::project(const Vec3 &point) const noexcept {
  const double sin_yaw = std::sin(yaw_rad_);
  const double cos_yaw = std::cos(yaw_rad_);
  const double sin_elevation = std::sin(elevation_rad_);
  const double cos_elevation = std::cos(elevation_rad_);

  const Vec3 right = {sin_yaw, -cos_yaw, 0.0};
  // Use the conventional above-ground default view: the horizontal X/Y plane
  // recedes down-screen while positive Z points up-screen. This changes only
  // the viewer camera basis; estimator coordinates and serialized states stay
  // in the native OpenVINS global frame.
  const Vec3 up = {-sin_elevation * cos_yaw, -sin_elevation * sin_yaw, cos_elevation};
  return {point.x * right.x + point.y * right.y + point.z * right.z,
          point.x * up.x + point.y * up.y + point.z * up.z};
}

ViewPoint2d
TrajectoryViewController::screen_point(const ViewPoint2d &projected,
                                       const ViewPoint2d &projected_centre, double base_scale,
                                       const ViewPoint2d &viewport_centre) const noexcept {
  const double scale = base_scale * zoom_;
  return {
      viewport_centre.x + pan_x_pixels_ + (projected.x - projected_centre.x) * scale,
      viewport_centre.y + pan_y_pixels_ - (projected.y - projected_centre.y) * scale,
  };
}

void TrajectoryViewController::orbit(double horizontal_pixels, double vertical_pixels) noexcept {
  if (!std::isfinite(horizontal_pixels) || !std::isfinite(vertical_pixels)) {
    return;
  }
  yaw_rad_ = std::remainder(yaw_rad_ + horizontal_pixels * orbit_radians_per_pixel, full_turn_rad);
  elevation_rad_ = std::clamp(elevation_rad_ - vertical_pixels * orbit_radians_per_pixel,
                              minimum_elevation_rad, maximum_elevation_rad);
}

void TrajectoryViewController::pan(double horizontal_pixels, double vertical_pixels) noexcept {
  if (!std::isfinite(horizontal_pixels) || !std::isfinite(vertical_pixels)) {
    return;
  }
  pan_x_pixels_ += horizontal_pixels;
  pan_y_pixels_ += vertical_pixels;
}

void TrajectoryViewController::zoom_at(double wheel_steps, const ViewPoint2d &cursor,
                                       const ViewPoint2d &viewport_centre) noexcept {
  if (!std::isfinite(wheel_steps) || !finite_point(cursor) || !finite_point(viewport_centre)) {
    return;
  }
  const double previous_zoom = zoom_;
  const double requested_zoom = previous_zoom * std::pow(zoom_per_wheel_step, wheel_steps);
  zoom_ = std::clamp(requested_zoom, minimum_zoom, maximum_zoom);
  const double ratio = zoom_ / previous_zoom;

  pan_x_pixels_ =
      cursor.x - viewport_centre.x - ratio * (cursor.x - viewport_centre.x - pan_x_pixels_);
  pan_y_pixels_ =
      cursor.y - viewport_centre.y - ratio * (cursor.y - viewport_centre.y - pan_y_pixels_);
}

void TrajectoryViewController::fit() noexcept {
  zoom_ = 1.0;
  pan_x_pixels_ = 0.0;
  pan_y_pixels_ = 0.0;
}

void TrajectoryViewController::reset() noexcept {
  yaw_rad_ = default_yaw_rad;
  elevation_rad_ = default_elevation_rad;
  fit();
}

void TrajectoryViewFrame::fit(const Vec3 &minimum, const Vec3 &maximum) noexcept {
  if (!std::isfinite(minimum.x) || !std::isfinite(minimum.y) || !std::isfinite(minimum.z) ||
      !std::isfinite(maximum.x) || !std::isfinite(maximum.y) || !std::isfinite(maximum.z)) {
    return;
  }
  focus_world_m_ = {
      0.5 * (minimum.x + maximum.x),
      0.5 * (minimum.y + maximum.y),
      0.5 * (minimum.z + maximum.z),
  };
  const double dx = maximum.x - minimum.x;
  const double dy = maximum.y - minimum.y;
  const double dz = maximum.z - minimum.z;
  const double diagonal = std::hypot(dx, dy, dz);
  world_span_m_ = std::max(minimum_world_span_m, fit_padding * diagonal);
}

double TrajectoryViewFrame::base_scale(double viewport_width,
                                       double viewport_height) const noexcept {
  if (!std::isfinite(viewport_width) || !std::isfinite(viewport_height) ||
      viewport_width <= 2.0 * horizontal_margin_pixels ||
      viewport_height <= vertical_margin_pixels) {
    return 1.0;
  }
  const double horizontal_pixels = viewport_width - 2.0 * horizontal_margin_pixels;
  const double vertical_pixels = viewport_height - vertical_margin_pixels;
  return std::max(1e-6, std::min(horizontal_pixels, vertical_pixels) / world_span_m_);
}

ViewPoint2d TrajectoryViewFrame::screen_point(const TrajectoryViewController &controller,
                                              const Vec3 &point, double viewport_width,
                                              double viewport_height) const noexcept {
  const ViewPoint2d viewport_centre = {
      0.5 * viewport_width,
      0.5 * (viewport_height + 42.0),
  };
  return controller.screen_point(controller.project(point), controller.project(focus_world_m_),
                                 base_scale(viewport_width, viewport_height), viewport_centre);
}

} // namespace ovrs
