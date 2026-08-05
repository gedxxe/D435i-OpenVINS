#include "ovrs/stereo_capture_preview.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cstddef>
#include <stdexcept>

namespace ovrs {
namespace {

constexpr const char *window_name = "OVRS stereo capture preview";

cv::Mat image_view(const ImageFrame &frame) {
  const std::size_t required =
      static_cast<std::size_t>(frame.stride_bytes) * frame.height;
  if (frame.width <= 0 || frame.height <= 0 ||
      frame.stride_bytes < frame.width || frame.format != "Y8" ||
      !frame.pixels || frame.pixels->size() < required) {
    throw std::runtime_error("preview received a malformed Y8 image");
  }
  return {frame.height, frame.width, CV_8UC1, frame.pixels->data(),
          static_cast<std::size_t>(frame.stride_bytes)};
}

} // namespace

StereoCapturePreview::~StereoCapturePreview() { close(); }

bool StereoCapturePreview::open(std::string *error) {
  if (open_) {
    return true;
  }
  try {
    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    open_ = true;
    return true;
  } catch (const cv::Exception &exception) {
    if (error) {
      *error = std::string("cannot open stereo preview: ") +
               exception.what() +
               ". Use a graphical desktop session or omit --preview.";
    }
    close();
    return false;
  }
}

bool StereoCapturePreview::show(const StereoFrame &frame,
                                const std::string &status,
                                const std::string &instruction,
                                std::string *error) {
  if (!open_) {
    if (error) {
      *error = "stereo preview is not open";
    }
    return false;
  }
  try {
    cv::Mat cameras;
    cv::hconcat(image_view(frame.camera0), image_view(frame.camera1), cameras);
    cv::Mat display;
    cv::cvtColor(cameras, display, cv::COLOR_GRAY2BGR);
    const int second_camera_x = frame.camera0.width + 12;
    cv::putText(display, "IR1 / cam0", {12, 28},
                cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 255, 0}, 2,
                cv::LINE_AA);
    cv::putText(display, "IR2 / cam1", {second_camera_x, 28},
                cv::FONT_HERSHEY_SIMPLEX, 0.65, {0, 255, 0}, 2,
                cv::LINE_AA);
    cv::putText(display, status, {12, display.rows - 42},
                cv::FONT_HERSHEY_SIMPLEX, 0.58, {0, 215, 255}, 2,
                cv::LINE_AA);
    cv::putText(display, instruction, {12, display.rows - 14},
                cv::FONT_HERSHEY_SIMPLEX, 0.58, {255, 255, 255}, 1,
                cv::LINE_AA);
    cv::imshow(window_name, display);
    return true;
  } catch (const cv::Exception &exception) {
    if (error) {
      *error = std::string("stereo preview failed: ") + exception.what();
    }
    return false;
  } catch (const std::exception &exception) {
    if (error) {
      *error = exception.what();
    }
    return false;
  }
}

CapturePreviewAction StereoCapturePreview::poll(bool allow_start,
                                                std::string *error) {
  if (!open_) {
    if (error) {
      *error = "stereo preview is not open";
    }
    return CapturePreviewAction::abort;
  }
  try {
    const int key = cv::waitKey(1) & 0xff;
    if (key == 27 || key == 'q' || key == 'Q') {
      return CapturePreviewAction::abort;
    }
    if (allow_start && key == ' ') {
      return CapturePreviewAction::start;
    }
    const double visible =
        cv::getWindowProperty(window_name, cv::WND_PROP_VISIBLE);
    return visible == 0.0 ? CapturePreviewAction::abort
                          : CapturePreviewAction::none;
  } catch (const cv::Exception &exception) {
    if (error) {
      *error = std::string("stereo preview failed: ") + exception.what();
    }
    return CapturePreviewAction::abort;
  }
}

void StereoCapturePreview::close() noexcept {
  if (!open_) {
    return;
  }
  try {
    cv::destroyWindow(window_name);
    cv::waitKey(1);
  } catch (...) {
  }
  open_ = false;
}

} // namespace ovrs
