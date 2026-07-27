#include "ovrs/stereo_synchronizer.hpp"

#include <cmath>

namespace ovrs {

std::optional<StereoFrame> StereoSynchronizer::pair(ImageFrame camera0,
                                                    ImageFrame camera1) {
  const auto valid_buffer = [](const ImageFrame &image) {
    if (image.width <= 0 || image.height <= 0 ||
        image.stride_bytes < image.width || !image.pixels ||
        !std::isfinite(image.timestamp)) {
      return false;
    }
    const auto required = static_cast<std::size_t>(image.stride_bytes) *
                          static_cast<std::size_t>(image.height);
    return image.pixels->size() >= required;
  };
  if (camera0.camera_id != 0 || camera1.camera_id != 1 ||
      camera0.format != "Y8" || camera1.format != "Y8" ||
      camera0.width != camera1.width || camera0.height != camera1.height ||
      !valid_buffer(camera0) || !valid_buffer(camera1)) {
    ++stats_.rejected_pairs;
    ++stats_.malformed_frames;
    return std::nullopt;
  }
  if (camera0.frameset_number != camera1.frameset_number) {
    ++stats_.rejected_pairs;
    ++stats_.frameset_mismatches;
    return std::nullopt;
  }
  if (std::abs(camera0.timestamp - camera1.timestamp) >
      maximum_difference_s_) {
    ++stats_.rejected_pairs;
    ++stats_.timestamp_mismatches;
    return std::nullopt;
  }
  StereoFrame result;
  result.timestamp = 0.5 * (camera0.timestamp + camera1.timestamp);
  result.camera0 = std::move(camera0);
  result.camera1 = std::move(camera1);
  ++stats_.accepted_pairs;
  return result;
}

} // namespace ovrs
