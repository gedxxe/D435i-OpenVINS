#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace ovrs {

class TimestampNormalizer {
public:
  struct Result {
    bool accepted = false;
    double seconds = 0.0;
    std::string error;
  };

  Result normalize(const std::string &stream, double raw_timestamp_ms,
                   const std::string &domain);
  void reset();
  std::optional<double> origin_ms() const { return origin_ms_; }
  const std::string &domain() const { return domain_; }
  std::vector<std::string> observed_streams() const;
  std::size_t rejected() const { return rejected_; }

private:
  std::optional<double> origin_ms_;
  std::string domain_;
  std::map<std::string, double> last_by_stream_ms_;
  std::size_t rejected_ = 0;
};

} // namespace ovrs
