#include "ovrs/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace ovrs {
namespace {
bool finite(const Vec3 &v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
} // namespace

bool finite_state(const EstimatorState &state) {
  if (!std::isfinite(state.timestamp) || !finite(state.position_world_m) ||
      !finite(state.velocity_world_m_s) || !finite(state.gyro_bias_rad_s) ||
      !finite(state.accel_bias_m_s2) ||
      !std::isfinite(state.processing_latency_ms) ||
      state.processing_latency_ms < 0.0) {
    return false;
  }
  double quaternion_norm_squared = 0.0;
  for (const double q : state.q_world_to_imu_xyzw) {
    if (!std::isfinite(q)) {
      return false;
    }
    quaternion_norm_squared += q * q;
  }
  constexpr double quaternion_norm_tolerance = 1e-3;
  if (!std::isfinite(quaternion_norm_squared) ||
      std::abs(quaternion_norm_squared - 1.0) >
          quaternion_norm_tolerance) {
    return false;
  }
  if (state.covariance_available) {
    for (const double covariance : state.covariance_diagonal) {
      if (!std::isfinite(covariance) || covariance < 0.0) {
        return false;
      }
    }
  }
  return true;
}

std::string serialize_tum(const EstimatorState &state) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(9) << state.timestamp << ' '
      << state.position_world_m.x << ' ' << state.position_world_m.y << ' '
      << state.position_world_m.z << ' ' << state.q_world_to_imu_xyzw[0]
      << ' ' << state.q_world_to_imu_xyzw[1] << ' '
      << state.q_world_to_imu_xyzw[2] << ' '
      << state.q_world_to_imu_xyzw[3];
  return out.str();
}

RunWriter::~RunWriter() { close(nullptr); }

bool RunWriter::open(const std::filesystem::path &directory,
                     std::string *error) {
  std::lock_guard<std::mutex> lock(mutex_);
  close_locked(nullptr);
  std::error_code ec;
  if (std::filesystem::exists(directory, ec) &&
      !std::filesystem::is_empty(directory, ec)) {
    if (error) {
      *error = "run directory already exists and is not empty: " +
               directory.string();
    }
    return false;
  }
  std::filesystem::create_directories(directory, ec);
  if (ec) {
    if (error) {
      *error = "cannot create run directory: " + ec.message();
    }
    return false;
  }
  directory_ = directory;
  finalized_ = false;
  output_open_ = false;
  have_state_ = false;
  path_length_m_ = 0.0;
  latency_sum_ms_ = 0.0;
  latency_max_ms_ = 0.0;
  state_count_ = 0;
  rejected_nonfinite_states_ = 0;
  std::ofstream incomplete(directory / "INCOMPLETE",
                           std::ios::binary | std::ios::trunc);
  incomplete << "ovrs run output is incomplete until clean finalization\n";
  incomplete.flush();
  if (!incomplete) {
    if (error) {
      *error = "cannot create run INCOMPLETE marker";
    }
    return false;
  }
  incomplete.close();
  trajectory_.open(directory / "trajectory_tum.txt");
  state_.open(directory / "state.csv");
  diagnostics_.open(directory / "diagnostics.csv");
  application_log_.open(directory / "application.log");
  if (!trajectory_ || !state_ || !diagnostics_ || !application_log_) {
    if (error) {
      *error = "cannot create one or more run output files";
    }
    close_locked(nullptr);
    return false;
  }
  trajectory_ << "# timestamp p_IinG_x p_IinG_y p_IinG_z q_GtoI_x "
                 "q_GtoI_y q_GtoI_z q_GtoI_w; native OpenVINS JPL state\n";
  state_ << "timestamp,px,py,pz,vx,vy,vz,qx,qy,qz,qw,bgx,bgy,bgz,bax,"
            "bay,baz,cov_ori_x,cov_ori_y,cov_ori_z,cov_pos_x,cov_pos_y,"
            "cov_pos_z,cov_vel_x,cov_vel_y,cov_vel_z,cov_bg_x,cov_bg_y,"
            "cov_bg_z,cov_ba_x,cov_ba_y,cov_ba_z,initialized,healthy,"
            "processing_latency_ms\n";
  diagnostics_
      << "timestamp,received_camera_frames,valid_stereo_pairs,"
         "received_gyro_samples,received_accel_samples,"
         "synchronized_imu_samples,rejected_timestamps,dropped_frames,"
         "imu_queue_depth,camera_queue_depth,camera_rate_hz,imu_rate_hz,"
         "estimator_rate_hz,processing_latency_ms,initialized\n";
  output_open_ = true;
  return true;
}

bool RunWriter::write_state(const EstimatorState &s, std::string *error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!finite_state(s)) {
    ++rejected_nonfinite_states_;
    if (error) {
      *error = "refusing to serialize a non-finite estimator state";
    }
    return false;
  }
  if (have_state_ && s.timestamp <= last_timestamp_) {
    if (error) {
      *error = "refusing to serialize a duplicate or regressing state "
               "timestamp";
    }
    return false;
  }
  trajectory_ << serialize_tum(s) << '\n';
  if (!have_state_) {
    have_state_ = true;
    first_position_ = s.position_world_m;
    previous_position_ = s.position_world_m;
    first_timestamp_ = s.timestamp;
  } else {
    const double dx = s.position_world_m.x - previous_position_.x;
    const double dy = s.position_world_m.y - previous_position_.y;
    const double dz = s.position_world_m.z - previous_position_.z;
    path_length_m_ += std::sqrt(dx * dx + dy * dy + dz * dz);
    previous_position_ = s.position_world_m;
  }
  last_timestamp_ = s.timestamp;
  latency_sum_ms_ += s.processing_latency_ms;
  latency_max_ms_ = std::max(latency_max_ms_, s.processing_latency_ms);
  ++state_count_;
  state_ << std::setprecision(17) << s.timestamp << ',' << s.position_world_m.x
         << ',' << s.position_world_m.y << ',' << s.position_world_m.z << ','
         << s.velocity_world_m_s.x << ',' << s.velocity_world_m_s.y << ','
         << s.velocity_world_m_s.z;
  for (const auto q : s.q_world_to_imu_xyzw) {
    state_ << ',' << q;
  }
  state_ << ',' << s.gyro_bias_rad_s.x << ',' << s.gyro_bias_rad_s.y << ','
         << s.gyro_bias_rad_s.z << ',' << s.accel_bias_m_s2.x << ','
         << s.accel_bias_m_s2.y << ',' << s.accel_bias_m_s2.z;
  for (const auto covariance : s.covariance_diagonal) {
    if (s.covariance_available) {
      state_ << ',' << covariance;
    } else {
      state_ << ",NA";
    }
  }
  state_ << ',' << (s.initialized ? 1 : 0) << ',' << (s.healthy ? 1 : 0)
         << ',' << s.processing_latency_ms << '\n';
  if (!trajectory_ || !state_) {
    if (error) {
      *error = "failed to write trajectory or state output";
    }
    return false;
  }
  return true;
}

bool RunWriter::write_diagnostics(const DiagnosticsSnapshot &d,
                                  std::string *error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!std::isfinite(d.timestamp) || !std::isfinite(d.camera_rate_hz) ||
      !std::isfinite(d.imu_rate_hz) ||
      !std::isfinite(d.estimator_rate_hz) ||
      !std::isfinite(d.processing_latency_ms)) {
    if (error) {
      *error = "refusing to serialize nonfinite diagnostics";
    }
    return false;
  }
  diagnostics_ << std::setprecision(17) << d.timestamp << ','
               << d.received_camera_frames << ',' << d.valid_stereo_pairs
               << ',' << d.received_gyro_samples << ','
               << d.received_accel_samples << ','
               << d.synchronized_imu_samples << ',' << d.rejected_timestamps
               << ',' << d.dropped_frames << ',' << d.imu_queue_depth << ','
               << d.camera_queue_depth << ',' << d.camera_rate_hz << ','
               << d.imu_rate_hz << ',' << d.estimator_rate_hz << ','
               << d.processing_latency_ms << ',' << (d.initialized ? 1 : 0)
               << '\n';
  if (!diagnostics_ && error) {
    *error = "failed to write diagnostics.csv";
  }
  return static_cast<bool>(diagnostics_);
}

bool RunWriter::log(const std::string &message, std::string *error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!application_log_) {
    if (error) {
      *error = "application log is not open";
    }
    return false;
  }
  application_log_ << message << '\n';
  if (!application_log_) {
    if (error) {
      *error = "failed to write application.log";
    }
    return false;
  }
  return true;
}

bool RunWriter::close(std::string *error) {
  std::lock_guard<std::mutex> lock(mutex_);
  return close_locked(error);
}

bool RunWriter::close_locked(std::string *error) {
  if (application_log_.is_open()) {
    application_log_ << std::setprecision(17);
    if (have_state_) {
      const double dx = previous_position_.x - first_position_.x;
      const double dy = previous_position_.y - first_position_.y;
      const double dz = previous_position_.z - first_position_.z;
      application_log_
          << "summary.total_trajectory_duration_s="
          << (last_timestamp_ - first_timestamp_) << '\n'
          << "summary.estimated_path_length_m=" << path_length_m_ << '\n'
          << "summary.final_displacement_m="
          << std::sqrt(dx * dx + dy * dy + dz * dz) << '\n'
          << "summary.average_processing_latency_ms="
          << latency_sum_ms_ / static_cast<double>(state_count_) << '\n'
          << "summary.maximum_processing_latency_ms=" << latency_max_ms_
          << '\n';
    }
    application_log_ << "summary.nonfinite_states_rejected="
                     << rejected_nonfinite_states_ << '\n';
  }
  trajectory_.flush();
  state_.flush();
  diagnostics_.flush();
  application_log_.flush();
  const bool success =
      (!trajectory_.is_open() || static_cast<bool>(trajectory_)) &&
      (!state_.is_open() || static_cast<bool>(state_)) &&
      (!diagnostics_.is_open() || static_cast<bool>(diagnostics_)) &&
      (!application_log_.is_open() || static_cast<bool>(application_log_));
  trajectory_.close();
  state_.close();
  diagnostics_.close();
  application_log_.close();
  output_open_ = false;
  if (!success && error) {
    *error = "one or more run output files failed to flush";
  }
  return success;
}

bool RunWriter::finalize(std::string *error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (finalized_) {
    if (error) {
      *error = "run output was already finalized";
    }
    return false;
  }
  if (!output_open_) {
    if (error) {
      *error = "run output is not open for finalization";
    }
    return false;
  }
  if (!have_state_) {
    if (error) {
      *error = "run output has no initialized estimator state";
    }
    return false;
  }
  if (!close_locked(error)) {
    return false;
  }
  std::error_code ec;
  const auto marker = directory_ / "INCOMPLETE";
  const bool removed = std::filesystem::remove(marker, ec);
  if (ec || !removed) {
    if (error) {
      *error = "cannot remove run INCOMPLETE marker: " +
               (ec ? ec.message() : std::string("marker is missing"));
    }
    return false;
  }
  finalized_ = true;
  return true;
}

} // namespace ovrs
