#pragma once

#include "ovrs/types.hpp"

namespace ovrs {

struct ViewPoint2d {
  double x = 0.0;
  double y = 0.0;
};

class TrajectoryViewController {
public:
  static constexpr double default_yaw_rad = 0.7853981633974483;
  static constexpr double default_elevation_rad = 0.6154797086703874;
  static constexpr double minimum_elevation_rad = -1.4835298641951802;
  static constexpr double maximum_elevation_rad = 1.4835298641951802;
  static constexpr double minimum_zoom = 0.1;
  static constexpr double maximum_zoom = 20.0;

  ViewPoint2d project(const Vec3 &point) const noexcept;
  ViewPoint2d screen_point(const ViewPoint2d &projected, const ViewPoint2d &projected_centre,
                           double base_scale, const ViewPoint2d &viewport_centre) const noexcept;

  void orbit(double horizontal_pixels, double vertical_pixels) noexcept;
  void pan(double horizontal_pixels, double vertical_pixels) noexcept;
  void zoom_at(double wheel_steps, const ViewPoint2d &cursor,
               const ViewPoint2d &viewport_centre) noexcept;
  void fit() noexcept;
  void reset() noexcept;

  double yaw_rad() const noexcept { return yaw_rad_; }
  double elevation_rad() const noexcept { return elevation_rad_; }
  double zoom() const noexcept { return zoom_; }
  double pan_x_pixels() const noexcept { return pan_x_pixels_; }
  double pan_y_pixels() const noexcept { return pan_y_pixels_; }

private:
  double yaw_rad_ = default_yaw_rad;
  double elevation_rad_ = default_elevation_rad;
  double zoom_ = 1.0;
  double pan_x_pixels_ = 0.0;
  double pan_y_pixels_ = 0.0;
};

class TrajectoryViewFrame {
public:
  static constexpr double minimum_world_span_m = 0.5;
  static constexpr double fit_padding = 1.15;
  static constexpr double horizontal_margin_pixels = 48.0;
  static constexpr double vertical_margin_pixels = 92.0;

  void fit(const Vec3 &minimum, const Vec3 &maximum) noexcept;
  ViewPoint2d screen_point(const TrajectoryViewController &controller, const Vec3 &point,
                           double viewport_width, double viewport_height) const noexcept;
  double base_scale(double viewport_width, double viewport_height) const noexcept;

  const Vec3 &focus_world_m() const noexcept { return focus_world_m_; }
  double world_span_m() const noexcept { return world_span_m_; }

private:
  Vec3 focus_world_m_;
  double world_span_m_ = minimum_world_span_m;
};

} // namespace ovrs
