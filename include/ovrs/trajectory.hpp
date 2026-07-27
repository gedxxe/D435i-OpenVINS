#pragma once

#include "ovrs/types.hpp"

#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>

namespace ovrs {

bool finite_state(const EstimatorState &state);
std::string serialize_tum(const EstimatorState &state);

class RunWriter {
public:
  RunWriter() = default;
  ~RunWriter();
  bool open(const std::filesystem::path &directory, std::string *error);
  bool write_state(const EstimatorState &state, std::string *error);
  bool write_diagnostics(const DiagnosticsSnapshot &snapshot,
                         std::string *error);
  bool log(const std::string &message, std::string *error = nullptr);
  bool close(std::string *error = nullptr);
  bool finalize(std::string *error = nullptr);
  const std::filesystem::path &directory() const { return directory_; }

private:
  bool close_locked(std::string *error);

  std::mutex mutex_;
  std::filesystem::path directory_;
  std::ofstream trajectory_;
  std::ofstream state_;
  std::ofstream diagnostics_;
  std::ofstream application_log_;
  bool output_open_ = false;
  bool finalized_ = false;
  bool have_state_ = false;
  Vec3 first_position_;
  Vec3 previous_position_;
  double first_timestamp_ = 0.0;
  double last_timestamp_ = 0.0;
  double path_length_m_ = 0.0;
  double latency_sum_ms_ = 0.0;
  double latency_max_ms_ = 0.0;
  std::uint64_t state_count_ = 0;
  std::uint64_t rejected_nonfinite_states_ = 0;
};

} // namespace ovrs
