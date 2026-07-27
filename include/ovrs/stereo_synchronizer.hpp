#pragma once

#include "ovrs/types.hpp"

#include <cstdint>
#include <optional>

namespace ovrs {

class StereoSynchronizer {
public:
  struct Stats {
    std::uint64_t accepted_pairs = 0;
    std::uint64_t rejected_pairs = 0;
    std::uint64_t timestamp_mismatches = 0;
    std::uint64_t frameset_mismatches = 0;
    std::uint64_t malformed_frames = 0;
  };

  explicit StereoSynchronizer(double maximum_difference_s = 0.002)
      : maximum_difference_s_(maximum_difference_s) {}

  std::optional<StereoFrame> pair(ImageFrame camera0, ImageFrame camera1);
  const Stats &stats() const { return stats_; }

private:
  double maximum_difference_s_;
  Stats stats_;
};

} // namespace ovrs
