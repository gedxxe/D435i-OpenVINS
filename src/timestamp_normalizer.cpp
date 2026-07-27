#include "ovrs/timestamp_normalizer.hpp"

#include <cmath>

namespace ovrs {

TimestampNormalizer::Result
TimestampNormalizer::normalize(const std::string &stream, double raw_timestamp_ms,
                               const std::string &domain) {
  if (stream.empty() || domain.empty() || !std::isfinite(raw_timestamp_ms)) {
    ++rejected_;
    return {false, 0.0, "stream, domain, and timestamp must be valid"};
  }
  if (!domain_.empty() && domain != domain_) {
    ++rejected_;
    return {false, 0.0, "timestamp domain changed from " + domain_ + " to " +
                            domain};
  }
  const auto previous = last_by_stream_ms_.find(stream);
  if (previous != last_by_stream_ms_.end() &&
      raw_timestamp_ms <= previous->second) {
    ++rejected_;
    return {false, 0.0, "duplicate or regressing timestamp on stream " +
                            stream};
  }
  if (!origin_ms_) {
    origin_ms_ = raw_timestamp_ms;
    domain_ = domain;
  }
  last_by_stream_ms_[stream] = raw_timestamp_ms;
  return {true, (raw_timestamp_ms - *origin_ms_) * 1e-3, {}};
}

void TimestampNormalizer::reset() {
  origin_ms_.reset();
  domain_.clear();
  last_by_stream_ms_.clear();
  rejected_ = 0;
}

std::vector<std::string> TimestampNormalizer::observed_streams() const {
  std::vector<std::string> result;
  result.reserve(last_by_stream_ms_.size());
  for (const auto &entry : last_by_stream_ms_) {
    result.push_back(entry.first);
  }
  return result;
}

} // namespace ovrs
