#pragma once

#include "ovrs/types.hpp"

#include <string>

namespace ovrs {

enum class CapturePreviewAction { none, start, abort };

// Main-thread-only OpenCV preview used before and during calibration capture.
// RealSense callbacks publish owned StereoFrame objects through a bounded
// queue; they never call HighGUI.
class StereoCapturePreview {
public:
  StereoCapturePreview() = default;
  ~StereoCapturePreview();

  StereoCapturePreview(const StereoCapturePreview &) = delete;
  StereoCapturePreview &operator=(const StereoCapturePreview &) = delete;

  bool open(std::string *error);
  bool show(const StereoFrame &frame, const std::string &status,
            const std::string &instruction, std::string *error);
  CapturePreviewAction poll(bool allow_start, std::string *error);
  void close() noexcept;

private:
  bool open_ = false;
};

} // namespace ovrs
