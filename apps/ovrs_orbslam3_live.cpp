#include "ovrs/app_support.hpp"
#include "ovrs/calibration_validation.hpp"
#include "ovrs/config.hpp"
#include "ovrs/imu_startup_gate.hpp"
#include "ovrs/imu_synchronizer.hpp"
#include "ovrs/measurement_dispatcher.hpp"
#include "ovrs/orb_trajectory_gate.hpp"
#include "ovrs/realsense_source.hpp"
#include "ovrs/version.hpp"
#include "ovrs/yaml_utils.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#if defined(OVRS_HAS_ORBSLAM3)
#include <System.h>
#include <opencv2/core.hpp>
#include <openssl/evp.h>
#endif

namespace {

void help() {
  std::cout
      << "Usage: ovrs_orbslam3_live --settings ORB.yaml --vocabulary ORBvoc.txt\n"
         "       --config ESTIMATOR.yaml --stream-config STREAMS.yaml\n"
         "       --live-bundle-manifest LIVE_MANIFEST.yaml\n"
         "       [--serial SERIAL] [--output RUN_DIR] [--viewer|--headless]\n"
         "       [--allow-unverified-calibration] [RealSense stream overrides]\n"
         "\n"
         "Runs the experimental pure ORB-SLAM3 stereo-inertial live path. "
         "OpenVINS poses are not consumed and global corrections are not fed "
         "to OpenVINS. The generated ORB settings select camera stride and "
         "camera-to-IMU time offset. Headless is the default.\n";
}

#if defined(OVRS_HAS_ORBSLAM3)

class WorkingDirectoryGuard {
public:
  explicit WorkingDirectoryGuard(const std::filesystem::path &path)
      : original_(std::filesystem::current_path()) {
    std::filesystem::current_path(path);
  }
  ~WorkingDirectoryGuard() {
    std::error_code ignored;
    std::filesystem::current_path(original_, ignored);
  }

private:
  std::filesystem::path original_;
};

class OrbSystemShutdownGuard {
public:
  explicit OrbSystemShutdownGuard(ORB_SLAM3::System &system)
      : system_(system) {}
  ~OrbSystemShutdownGuard() noexcept {
    try {
      system_.Shutdown();
    } catch (...) {
      // Destructors cannot safely surface errors. Explicit shutdown remains
      // responsible for reporting failures on normal runtime paths.
    }
  }

private:
  ORB_SLAM3::System &system_;
};

std::string read_file(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("cannot open file: " + path.string());
  }
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

std::string sha256_file(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("cannot hash file: " + path.string());
  }
  using DigestContext =
      std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>;
  DigestContext context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("cannot initialize SHA-256");
  }
  std::array<char, 64 * 1024> buffer{};
  while (input) {
    input.read(buffer.data(),
               static_cast<std::streamsize>(buffer.size()));
    const auto count = input.gcount();
    if (count > 0 &&
        EVP_DigestUpdate(context.get(), buffer.data(),
                         static_cast<std::size_t>(count)) != 1) {
      throw std::runtime_error("cannot update SHA-256 for " + path.string());
    }
  }
  if (!input.eof()) {
    throw std::runtime_error("cannot read file while hashing: " +
                             path.string());
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0;
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_size) != 1 ||
      digest_size != 32) {
    throw std::runtime_error("cannot finalize SHA-256 for " + path.string());
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < digest_size; ++index) {
    output << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return output.str();
}

std::filesystem::path current_executable_path(const char *argv0) {
  std::error_code error;
#if defined(__linux__)
  const auto proc_path = std::filesystem::read_symlink("/proc/self/exe", error);
  if (!error && std::filesystem::is_regular_file(proc_path)) {
    return std::filesystem::canonical(proc_path);
  }
  error.clear();
#endif
  const auto fallback = std::filesystem::absolute(argv0);
  if (!std::filesystem::is_regular_file(fallback)) {
    throw std::runtime_error("cannot resolve current executable path");
  }
  return std::filesystem::canonical(fallback);
}

void require_yaml_value(const std::string &yaml, const std::string &key,
                        const std::string &expected,
                        const std::string &label) {
  const auto actual = ovrs::simple_yaml_scalar(yaml, key);
  if (actual != expected) {
    throw std::runtime_error(label + " " + key + " differs: expected " +
                             expected + ", got " + actual);
  }
}

struct OrbLiveSettings {
  std::string serial;
  std::string calibration_state;
  std::string backend_commit;
  std::string backend_patch_sha256;
  int width = 0;
  int height = 0;
  int source_fps = 0;
  int orb_fps = 0;
  int camera_stride = 0;
  double camera_imu_offset_s = 0.0;
  double imu_init_acceleration_threshold_m_s2 = 0.0;
  double imu_init_low_motion_reset_seconds = 0.0;
  double minimum_stable_inertial_seconds = 0.0;
  double maximum_tracking_interval_seconds = 0.0;
  double maximum_tracking_interval_factor = 0.0;
  std::uint64_t minimum_tracked_map_points = 0;
  double maximum_pose_linear_speed_m_s = 0.0;
  double maximum_pose_angular_speed_rad_s = 0.0;
  std::uint64_t maximum_preacceptance_map_resets = 0;
  double gravity_m_s2 = 0.0;
  double startup_maximum_gravity_error_m_s2 = 0.0;
  double startup_stationary_seconds = 0.0;
  double startup_stationary_timeout_seconds = 0.0;
  double startup_maximum_acceleration_stddev_m_s2 = 0.0;
  double startup_maximum_gyro_magnitude_rad_s = 0.0;
  double maximum_input_stall_seconds = 0.0;
};

template <typename Value>
Value required_setting(const cv::FileStorage &storage, const char *key) {
  const cv::FileNode node = storage[key];
  if (node.empty()) {
    throw std::runtime_error(std::string("ORB settings lack ") + key);
  }
  Value value{};
  node >> value;
  return value;
}

OrbLiveSettings load_orb_settings(const std::filesystem::path &path) {
  cv::FileStorage storage(path.string(), cv::FileStorage::READ);
  if (!storage.isOpened()) {
    throw std::runtime_error("cannot parse ORB settings: " + path.string());
  }
  OrbLiveSettings result;
  const auto mode = required_setting<std::string>(storage, "OVRS.Mode");
  if (mode != "EXPERIMENTAL_PURE_ORB_SLAM3_LIVE") {
    throw std::runtime_error(
        "ORB settings were not generated for the project live adapter");
  }
  result.serial =
      required_setting<std::string>(storage, "OVRS.CalibratedSerial");
  result.calibration_state =
      required_setting<std::string>(storage, "OVRS.CalibrationState");
  result.backend_commit =
      required_setting<std::string>(storage, "OVRS.BackendCommit");
  result.backend_patch_sha256 =
      required_setting<std::string>(storage, "OVRS.BackendPatchSHA256");
  result.width = required_setting<int>(storage, "Camera.width");
  result.height = required_setting<int>(storage, "Camera.height");
  result.source_fps = required_setting<int>(storage, "OVRS.SourceCameraFPS");
  result.orb_fps = required_setting<int>(storage, "Camera.fps");
  result.camera_stride =
      required_setting<int>(storage, "OVRS.CameraStride");
  result.camera_imu_offset_s = required_setting<double>(
      storage, "OVRS.CameraImuTimeOffsetSeconds");
  result.imu_init_acceleration_threshold_m_s2 =
      required_setting<double>(storage, "IMU.InitAccelerationThreshold");
  result.imu_init_low_motion_reset_seconds =
      required_setting<double>(storage, "IMU.InitLowMotionResetSeconds");
  result.minimum_stable_inertial_seconds = required_setting<double>(
      storage, "OVRS.MinimumStableInertialSeconds");
  result.maximum_tracking_interval_seconds = required_setting<double>(
      storage, "OVRS.MaximumTrackingIntervalSeconds");
  result.maximum_tracking_interval_factor = required_setting<double>(
      storage, "OVRS.MaximumTrackingIntervalFactor");
  const int minimum_tracked_map_points =
      required_setting<int>(storage, "OVRS.MinimumTrackedMapPoints");
  if (minimum_tracked_map_points <= 0) {
    throw std::runtime_error(
        "ORB minimum tracked map points must be positive");
  }
  result.minimum_tracked_map_points =
      static_cast<std::uint64_t>(minimum_tracked_map_points);
  result.maximum_pose_linear_speed_m_s =
      required_setting<double>(storage, "OVRS.MaximumPoseLinearSpeed");
  result.maximum_pose_angular_speed_rad_s =
      required_setting<double>(storage, "OVRS.MaximumPoseAngularSpeed");
  const int maximum_preacceptance_map_resets = required_setting<int>(
      storage, "OVRS.MaximumPreacceptanceMapResets");
  if (maximum_preacceptance_map_resets < 0) {
    throw std::runtime_error(
        "ORB maximum preacceptance map resets must be nonnegative");
  }
  result.maximum_preacceptance_map_resets =
      static_cast<std::uint64_t>(maximum_preacceptance_map_resets);
  result.gravity_m_s2 =
      required_setting<double>(storage, "OVRS.GravityMagnitudeMps2");
  result.startup_maximum_gravity_error_m_s2 = required_setting<double>(
      storage, "OVRS.StartupMaximumGravityErrorMps2");
  result.startup_stationary_seconds = required_setting<double>(
      storage, "OVRS.StartupStationarySeconds");
  result.startup_stationary_timeout_seconds = required_setting<double>(
      storage, "OVRS.StartupStationaryTimeoutSeconds");
  result.startup_maximum_acceleration_stddev_m_s2 = required_setting<double>(
      storage, "OVRS.StartupMaximumAccelerationStddevMps2");
  result.startup_maximum_gyro_magnitude_rad_s = required_setting<double>(
      storage, "OVRS.StartupMaximumGyroMagnitudeRadps");
  result.maximum_input_stall_seconds = required_setting<double>(
      storage, "OVRS.MaximumInputStallSeconds");
  if (result.serial.empty() || result.backend_commit.size() != 40 ||
      result.backend_patch_sha256.size() != 64 || result.width <= 0 ||
      result.height <= 0 || result.source_fps <= 0 || result.orb_fps <= 0 ||
      result.camera_stride <= 0 ||
      result.source_fps / result.camera_stride != result.orb_fps ||
      result.source_fps % result.camera_stride != 0 ||
      !std::isfinite(result.camera_imu_offset_s) ||
      std::abs(result.camera_imu_offset_s) > 0.1 ||
      !std::isfinite(result.imu_init_acceleration_threshold_m_s2) ||
      result.imu_init_acceleration_threshold_m_s2 <= 0.0 ||
      !std::isfinite(result.imu_init_low_motion_reset_seconds) ||
      result.imu_init_low_motion_reset_seconds <= 0.0 ||
      result.imu_init_low_motion_reset_seconds > 10.0 ||
      !std::isfinite(result.minimum_stable_inertial_seconds) ||
      result.minimum_stable_inertial_seconds <= 0.0 ||
      result.minimum_stable_inertial_seconds > 60.0 ||
      !std::isfinite(result.maximum_tracking_interval_seconds) ||
      result.maximum_tracking_interval_seconds <=
          1.0 / static_cast<double>(result.orb_fps) ||
      result.maximum_tracking_interval_seconds > 1.0 ||
      !std::isfinite(result.maximum_tracking_interval_factor) ||
      result.maximum_tracking_interval_factor <= 1.0 ||
      result.maximum_tracking_interval_factor > 10.0 ||
      result.minimum_tracked_map_points > 10000 ||
      !std::isfinite(result.maximum_pose_linear_speed_m_s) ||
      result.maximum_pose_linear_speed_m_s <= 0.0 ||
      !std::isfinite(result.maximum_pose_angular_speed_rad_s) ||
      result.maximum_pose_angular_speed_rad_s <= 0.0 ||
      result.maximum_preacceptance_map_resets > 20 ||
      !std::isfinite(result.gravity_m_s2) || result.gravity_m_s2 <= 0.0 ||
      !std::isfinite(result.startup_maximum_gravity_error_m_s2) ||
      result.startup_maximum_gravity_error_m_s2 <= 0.0 ||
      result.startup_maximum_gravity_error_m_s2 >= result.gravity_m_s2 ||
      !std::isfinite(result.startup_stationary_seconds) ||
      result.startup_stationary_seconds <= 0.0 ||
      !std::isfinite(result.startup_stationary_timeout_seconds) ||
      result.startup_stationary_timeout_seconds <
          result.startup_stationary_seconds ||
      result.startup_stationary_timeout_seconds > 60.0 ||
      !std::isfinite(
          result.startup_maximum_acceleration_stddev_m_s2) ||
      result.startup_maximum_acceleration_stddev_m_s2 <= 0.0 ||
      !std::isfinite(result.startup_maximum_gyro_magnitude_rad_s) ||
      result.startup_maximum_gyro_magnitude_rad_s <= 0.0 ||
      !std::isfinite(result.maximum_input_stall_seconds) ||
      result.maximum_input_stall_seconds <= 0.0 ||
      result.maximum_input_stall_seconds > 10.0 ||
      std::abs(result.maximum_tracking_interval_seconds -
               result.maximum_tracking_interval_factor /
                   static_cast<double>(result.orb_fps)) > 1e-9) {
    throw std::runtime_error("ORB live settings contain invalid provenance");
  }
  if (result.backend_commit != OVRS_ORBSLAM3_COMMIT_VALUE ||
      result.backend_patch_sha256 !=
          OVRS_ORBSLAM3_PATCH_SHA256_VALUE) {
    throw std::runtime_error(
        "ORB bundle backend identity differs from the compiled adapter");
  }
  return result;
}

const char *tracking_state_name(int state) {
  switch (state) {
  case ORB_SLAM3::Tracking::SYSTEM_NOT_READY:
    return "SYSTEM_NOT_READY";
  case ORB_SLAM3::Tracking::NO_IMAGES_YET:
    return "NO_IMAGES_YET";
  case ORB_SLAM3::Tracking::NOT_INITIALIZED:
    return "NOT_INITIALIZED";
  case ORB_SLAM3::Tracking::OK:
    return "OK";
  case ORB_SLAM3::Tracking::RECENTLY_LOST:
    return "RECENTLY_LOST";
  case ORB_SLAM3::Tracking::LOST:
    return "LOST";
  case ORB_SLAM3::Tracking::OK_KLT:
    return "OK_KLT";
  default:
    return "UNKNOWN";
  }
}

bool valid_image(const ovrs::ImageFrame &image, int width, int height) {
  if (!image.pixels || image.width != width || image.height != height ||
      image.format != "Y8" || image.stride_bytes < width) {
    return false;
  }
  const auto required =
      static_cast<std::size_t>(image.stride_bytes) * image.height;
  return image.pixels->size() >= required;
}

double magnitude(const ovrs::Vec3 &value) {
  return std::sqrt(value.x * value.x + value.y * value.y +
                   value.z * value.z);
}

std::int64_t steady_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

struct FrameImuMetrics {
  std::size_t samples = 0;
  ovrs::Vec3 acceleration_sum;
  double gyro_magnitude_sum = 0.0;
  double gyro_magnitude_max = 0.0;

  void add(const ovrs::ImuSample &sample) {
    ++samples;
    acceleration_sum.x += sample.linear_acceleration_m_s2.x;
    acceleration_sum.y += sample.linear_acceleration_m_s2.y;
    acceleration_sum.z += sample.linear_acceleration_m_s2.z;
    const double gyro_magnitude =
        magnitude(sample.angular_velocity_rad_s);
    gyro_magnitude_sum += gyro_magnitude;
    gyro_magnitude_max =
        std::max(gyro_magnitude_max, gyro_magnitude);
  }
};

struct ImuExcitationStats {
  std::uint64_t samples = 0;
  std::uint64_t consecutive_sample_deltas = 0;
  std::uint64_t orb_observed_frames = 0;
  std::uint64_t orb_pass_frames = 0;
  double acceleration_magnitude_sum = 0.0;
  double acceleration_magnitude_min =
      std::numeric_limits<double>::infinity();
  double acceleration_magnitude_max = 0.0;
  double gyro_magnitude_sum = 0.0;
  double gyro_magnitude_max = 0.0;
  double acceleration_sample_delta_squared_sum = 0.0;
  double acceleration_sample_delta_max = 0.0;
  double orb_delta_sum = 0.0;
  double orb_delta_max = 0.0;
  std::optional<ovrs::Vec3> previous_acceleration;

  void add_sample(const ovrs::ImuSample &sample) {
    const double acceleration_magnitude =
        magnitude(sample.linear_acceleration_m_s2);
    const double gyro_magnitude =
        magnitude(sample.angular_velocity_rad_s);
    ++samples;
    acceleration_magnitude_sum += acceleration_magnitude;
    acceleration_magnitude_min =
        std::min(acceleration_magnitude_min, acceleration_magnitude);
    acceleration_magnitude_max =
        std::max(acceleration_magnitude_max, acceleration_magnitude);
    gyro_magnitude_sum += gyro_magnitude;
    gyro_magnitude_max = std::max(gyro_magnitude_max, gyro_magnitude);
    if (previous_acceleration) {
      const ovrs::Vec3 delta{
          sample.linear_acceleration_m_s2.x - previous_acceleration->x,
          sample.linear_acceleration_m_s2.y - previous_acceleration->y,
          sample.linear_acceleration_m_s2.z - previous_acceleration->z};
      const double delta_magnitude = magnitude(delta);
      ++consecutive_sample_deltas;
      acceleration_sample_delta_squared_sum +=
          delta_magnitude * delta_magnitude;
      acceleration_sample_delta_max =
          std::max(acceleration_sample_delta_max, delta_magnitude);
    }
    previous_acceleration = sample.linear_acceleration_m_s2;
  }

  void add_orb_delta(double delta, double threshold) {
    ++orb_observed_frames;
    orb_delta_sum += delta;
    orb_delta_max = std::max(orb_delta_max, delta);
    if (delta >= threshold) {
      ++orb_pass_frames;
    }
  }
};

const char *startup_gate_state_name(ovrs::ImuStartupGateState state) {
  switch (state) {
  case ovrs::ImuStartupGateState::Collecting:
    return "COLLECTING";
  case ovrs::ImuStartupGateState::Passed:
    return "PASSED";
  case ovrs::ImuStartupGateState::GravityMismatch:
    return "GRAVITY_MISMATCH";
  case ovrs::ImuStartupGateState::StationaryTimeout:
    return "STATIONARY_TIMEOUT";
  }
  return "UNKNOWN";
}

struct TrackingLatencyStats {
  std::uint64_t samples = 0;
  std::uint64_t frame_budget_misses = 0;
  double sum_ms = 0.0;
  double maximum_ms = 0.0;

  void add(double latency_ms, double frame_budget_ms) {
    ++samples;
    sum_ms += latency_ms;
    maximum_ms = std::max(maximum_ms, latency_ms);
    if (latency_ms > frame_budget_ms) {
      ++frame_budget_misses;
    }
  }
};

#endif

} // namespace

int main(int argc, char **argv) {
  if (ovrs::has_flag(argc, argv, "--version")) {
    std::cout << ovrs::version_summary(
        "ovrs_orbslam3_live", ovrs::ceres_version, ovrs::opencv_version,
        ovrs::realsense_version);
    return 0;
  }
  if (ovrs::has_flag(argc, argv, "--help") ||
      ovrs::has_flag(argc, argv, "-h")) {
    help();
    return 0;
  }
  auto value_options = ovrs::stream_cli_value_options();
  value_options.insert(value_options.end(),
                       {"--settings", "--vocabulary", "--config",
                        "--stream-config", "--live-bundle-manifest",
                        "--output"});
  std::string argument_error;
  if (!ovrs::validate_cli_arguments(
          argc, argv, value_options,
          {"--viewer", "--headless", "--allow-unverified-calibration"},
          &argument_error)) {
    std::cerr << argument_error << '\n';
    return 2;
  }
  if (ovrs::has_flag(argc, argv, "--viewer") &&
      ovrs::has_flag(argc, argv, "--headless")) {
    std::cerr << "--viewer and --headless are mutually exclusive\n";
    return 2;
  }
#if !defined(OVRS_HAS_REALSENSE) || !defined(OVRS_HAS_ORBSLAM3)
  std::cerr << "ovrs_orbslam3_live was built without RealSense/ORB-SLAM3. "
               "Configure with OVRS_ENABLE_ORBSLAM3=ON and the pinned backend "
               "paths.\n";
  return 3;
#else
  try {
    const auto settings_path = std::filesystem::absolute(
        ovrs::value_after(argc, argv, "--settings"));
    const auto vocabulary_path = std::filesystem::absolute(
        ovrs::value_after(argc, argv, "--vocabulary"));
    const auto config_path = std::filesystem::absolute(
        ovrs::value_after(argc, argv, "--config"));
    const auto stream_path = std::filesystem::absolute(
        ovrs::value_after(argc, argv, "--stream-config"));
    const auto live_manifest_path = std::filesystem::absolute(
        ovrs::value_after(argc, argv, "--live-bundle-manifest"));
    for (const auto &path :
         {settings_path, vocabulary_path, config_path, stream_path,
          live_manifest_path}) {
      if (!std::filesystem::is_regular_file(path)) {
        throw std::runtime_error("required input is not a file: " +
                                 path.string());
      }
    }

    const OrbLiveSettings orb_settings = load_orb_settings(settings_path);
    const std::string live_manifest_text = read_file(live_manifest_path);
    const std::string settings_sha256 = sha256_file(settings_path);
    const std::string live_manifest_sha256 =
        sha256_file(live_manifest_path);
    const std::string vocabulary_sha256 = sha256_file(vocabulary_path);
    const auto executable_path = current_executable_path(argv[0]);
    const std::string executable_sha256 = sha256_file(executable_path);
    const std::filesystem::path backend_library_path =
        OVRS_ORBSLAM3_LIBRARY_PATH_VALUE;
    const std::string backend_library_sha256 =
        sha256_file(backend_library_path);
    if (backend_library_sha256 != OVRS_ORBSLAM3_LIBRARY_SHA256_VALUE) {
      throw std::runtime_error(
          "ORB-SLAM3 library changed after this executable was built");
    }
    for (const auto &[key, expected] :
         std::vector<std::pair<std::string, std::string>>{
             {"format", "ovrs-orbslam3-live-bundle-v6"},
             {"state", "PREPARED_NOT_RUN"},
             {"integration", "PURE_ORB_SLAM3_STEREO_INERTIAL"},
             {"openvins_pose_consumed", "false"},
             {"global_correction_fed_to_openvins", "false"},
             {"camera_serial", orb_settings.serial},
             {"calibration_state", orb_settings.calibration_state},
             {"backend_commit", orb_settings.backend_commit},
             {"backend_patch_sha256", orb_settings.backend_patch_sha256},
             {"camera_stride", std::to_string(orb_settings.camera_stride)},
             {"minimum_tracked_map_points",
              std::to_string(orb_settings.minimum_tracked_map_points)},
             {"maximum_pose_linear_speed_m_s",
              std::to_string(
                  orb_settings.maximum_pose_linear_speed_m_s)},
             {"maximum_pose_angular_speed_rad_s",
              std::to_string(
                  orb_settings.maximum_pose_angular_speed_rad_s)},
             {"settings_sha256", settings_sha256}}) {
      require_yaml_value(live_manifest_text, key, expected,
                         "ORB live bundle manifest");
    }
    const double manifest_stability = ovrs::parse_double_strict(
        ovrs::simple_yaml_scalar(live_manifest_text,
                                 "minimum_stable_inertial_seconds"),
        "live manifest minimum_stable_inertial_seconds");
    const double manifest_imu_init_low_motion_reset_seconds =
        ovrs::parse_double_strict(
            ovrs::simple_yaml_scalar(
                live_manifest_text,
                "imu_init_low_motion_reset_seconds"),
            "live manifest imu_init_low_motion_reset_seconds");
    const double manifest_offset = ovrs::parse_double_strict(
        ovrs::simple_yaml_scalar(live_manifest_text,
                                 "camera_imu_time_offset_s"),
        "live manifest camera_imu_time_offset_s");
    const double manifest_maximum_tracking_interval =
        ovrs::parse_double_strict(
            ovrs::simple_yaml_scalar(
                live_manifest_text,
                "maximum_tracking_interval_seconds"),
            "live manifest maximum_tracking_interval_seconds");
    const double manifest_maximum_tracking_interval_factor =
        ovrs::parse_double_strict(
            ovrs::simple_yaml_scalar(
                live_manifest_text,
                "maximum_tracking_interval_factor"),
            "live manifest maximum_tracking_interval_factor");
    const auto manifest_maximum_preacceptance_map_resets =
        ovrs::parse_uint64_strict(
            ovrs::simple_yaml_scalar(
                live_manifest_text,
                "maximum_preacceptance_map_resets"),
            "live manifest maximum_preacceptance_map_resets");
    const double manifest_gravity = ovrs::parse_double_strict(
        ovrs::simple_yaml_scalar(live_manifest_text, "gravity_m_s2"),
        "live manifest gravity_m_s2");
    const double manifest_startup_maximum_gravity_error =
        ovrs::parse_double_strict(
            ovrs::simple_yaml_scalar(
                live_manifest_text,
                "startup_maximum_gravity_error_m_s2"),
            "live manifest startup_maximum_gravity_error_m_s2");
    const double manifest_startup_stationary_seconds =
        ovrs::parse_double_strict(
            ovrs::simple_yaml_scalar(live_manifest_text,
                                     "startup_stationary_seconds"),
            "live manifest startup_stationary_seconds");
    const double manifest_startup_stationary_timeout_seconds =
        ovrs::parse_double_strict(
            ovrs::simple_yaml_scalar(
                live_manifest_text,
                "startup_stationary_timeout_seconds"),
            "live manifest startup_stationary_timeout_seconds");
    const double manifest_startup_maximum_acceleration_stddev =
        ovrs::parse_double_strict(
            ovrs::simple_yaml_scalar(
                live_manifest_text,
                "startup_maximum_acceleration_stddev_m_s2"),
            "live manifest startup_maximum_acceleration_stddev_m_s2");
    const double manifest_startup_maximum_gyro_magnitude =
        ovrs::parse_double_strict(
            ovrs::simple_yaml_scalar(
                live_manifest_text,
                "startup_maximum_gyro_magnitude_rad_s"),
            "live manifest startup_maximum_gyro_magnitude_rad_s");
    const double manifest_maximum_input_stall_seconds =
        ovrs::parse_double_strict(
            ovrs::simple_yaml_scalar(live_manifest_text,
                                     "maximum_input_stall_seconds"),
            "live manifest maximum_input_stall_seconds");
    if (std::abs(manifest_stability -
                 orb_settings.minimum_stable_inertial_seconds) > 1e-9 ||
        std::abs(manifest_imu_init_low_motion_reset_seconds -
                 orb_settings.imu_init_low_motion_reset_seconds) > 1e-9 ||
        std::abs(manifest_maximum_tracking_interval -
                 orb_settings.maximum_tracking_interval_seconds) > 1e-9 ||
        std::abs(manifest_maximum_tracking_interval_factor -
                 orb_settings.maximum_tracking_interval_factor) > 1e-9 ||
        manifest_maximum_preacceptance_map_resets !=
            orb_settings.maximum_preacceptance_map_resets ||
        std::abs(manifest_gravity - orb_settings.gravity_m_s2) > 1e-9 ||
        std::abs(manifest_startup_maximum_gravity_error -
                 orb_settings.startup_maximum_gravity_error_m_s2) > 1e-9 ||
        std::abs(manifest_startup_stationary_seconds -
                 orb_settings.startup_stationary_seconds) > 1e-9 ||
        std::abs(manifest_startup_stationary_timeout_seconds -
                 orb_settings.startup_stationary_timeout_seconds) > 1e-9 ||
        std::abs(manifest_startup_maximum_acceleration_stddev -
                 orb_settings.startup_maximum_acceleration_stddev_m_s2) >
            1e-9 ||
        std::abs(manifest_startup_maximum_gyro_magnitude -
                 orb_settings.startup_maximum_gyro_magnitude_rad_s) > 1e-9 ||
        std::abs(manifest_maximum_input_stall_seconds -
                 orb_settings.maximum_input_stall_seconds) > 1e-9 ||
        std::abs(manifest_offset - orb_settings.camera_imu_offset_s) > 1e-9) {
      throw std::runtime_error(
          "ORB live bundle timing/IMU gates differ from generated settings");
    }
    const std::string config_text = read_file(config_path);
    std::string error;
    if (!ovrs::validate_estimator_configuration(config_path, config_text,
                                                &error)) {
      throw std::runtime_error(error);
    }
    const bool allow_unverified =
        ovrs::has_flag(argc, argv, "--allow-unverified-calibration");
    if (!ovrs::validate_estimation_calibration_state(
            config_text, allow_unverified, &error)) {
      throw std::runtime_error(error);
    }
    const auto calibrated_serial =
        ovrs::simple_yaml_scalar(config_text, "calibrated_serial");
    const auto calibration_state =
        ovrs::simple_yaml_scalar(config_text, "calibration_state");
    if (calibrated_serial != orb_settings.serial ||
        calibration_state != orb_settings.calibration_state) {
      throw std::runtime_error(
          "ORB bundle identity differs from estimator calibration");
    }
    const double estimator_gravity = ovrs::parse_double_strict(
        ovrs::simple_yaml_scalar(config_text, "gravity_mag"),
        "estimator gravity_mag");
    const double estimator_maximum_accel_bias = ovrs::parse_double_strict(
        ovrs::simple_yaml_scalar(config_text, "max_accel_bias_m_s2"),
        "estimator max_accel_bias_m_s2");
    if (std::abs(estimator_gravity - orb_settings.gravity_m_s2) > 1e-9 ||
        std::abs(estimator_maximum_accel_bias -
                 orb_settings.startup_maximum_gravity_error_m_s2) > 1e-9) {
      throw std::runtime_error(
          "ORB startup IMU gate differs from estimator safety bounds");
    }

    ovrs::StreamConfig stream_config;
    stream_config.imu_queue_size = 4096;
    stream_config.stereo_queue_size = 32;
    if (!ovrs::load_stream_config(stream_path.string(), &stream_config,
                                  &error)) {
      throw std::runtime_error(error);
    }
    const auto requested_serial =
        ovrs::value_after(argc, argv, "--serial");
    stream_config.serial =
        requested_serial.empty() ? calibrated_serial : requested_serial;
    if (stream_config.serial != calibrated_serial) {
      throw std::runtime_error(
          "requested serial differs from calibrated serial");
    }
    if (!ovrs::apply_stream_config_cli(argc, argv, &stream_config, &error)) {
      throw std::runtime_error(error);
    }
    if (stream_config.width != orb_settings.width ||
        stream_config.height != orb_settings.height ||
        stream_config.camera_fps != orb_settings.source_fps) {
      throw std::runtime_error(
          "resolved stream geometry/rate differs from ORB live bundle");
    }
    if (!ovrs::validate_camera_calibration_resolution(
            config_path, config_text, stream_config.width,
            stream_config.height, &error)) {
      throw std::runtime_error(error);
    }

    const auto output = std::filesystem::absolute(ovrs::value_after(
        argc, argv, "--output",
        (std::filesystem::path("runs") /
         ("orbslam3_live_" + ovrs::utc_timestamp()))
            .string()));
    if (std::filesystem::exists(output)) {
      throw std::runtime_error("output already exists: " + output.string());
    }
    std::filesystem::create_directories(output);
    const auto incomplete = output / "INCOMPLETE";
    if (!ovrs::write_text(incomplete, "ORB-SLAM3 live run incomplete.\n",
                          &error)) {
      throw std::runtime_error(error);
    }
    std::filesystem::copy_file(
        settings_path, output / "resolved_orbslam3_settings.yaml");
    std::filesystem::copy_file(config_path,
                               output / "resolved_estimator_config.yaml");
    std::filesystem::copy_file(stream_path,
                               output / "requested_stream_config.yaml");
    std::filesystem::copy_file(
        live_manifest_path, output / "source_live_manifest.yaml");
    const auto launch_provenance_path = output / "launch_provenance.yaml";
    if (!ovrs::write_text(
            launch_provenance_path,
            std::string("%YAML:1.0\n"
            "format: \"ovrs-orbslam3-live-launch-provenance-v1\"\n"
            "state: \"LAUNCHED_NOT_CAPTURE_VALIDATED\"\n"
            "integration: \"PURE_ORB_SLAM3_STEREO_INERTIAL\"\n"
            "openvins_pose_consumed: false\n"
            "global_correction_fed_to_openvins: false\n"
            "source_fingerprint: \"") +
                ovrs::source_fingerprint +
                "\"\nlive_executable_sha256_at_start: \"" +
                executable_sha256 +
                "\"\nbackend_library_sha256_at_start: \"" +
                backend_library_sha256 +
                "\"\nvocabulary_sha256_at_start: \"" +
                vocabulary_sha256 +
                "\"\nsettings_sha256_at_start: \"" + settings_sha256 +
                "\"\nlive_bundle_manifest_sha256_at_start: \"" +
                live_manifest_sha256 + "\"\n",
            &error)) {
      throw std::runtime_error(error);
    }
    const std::string launch_provenance_sha256 =
        sha256_file(launch_provenance_path);

    const bool viewer = ovrs::has_flag(argc, argv, "--viewer");
    // Resolve librealsense/libusb before ORB launches worker threads. A
    // hardware-backend construction failure must leave no ORB lifecycle to
    // unwind.
    ovrs::RealSenseSource source(stream_config);
    // Vocabulary loading and ORB worker construction can take long enough for
    // an operator to request shutdown. Install the handler before entering
    // upstream construction so Ctrl+C is never delivered with default
    // process-termination semantics while ORB owns live threads.
    ovrs::install_signal_handlers();
    WorkingDirectoryGuard working_directory(output);
    ORB_SLAM3::System slam(vocabulary_path.string(), settings_path.string(),
                          ORB_SLAM3::System::IMU_STEREO, viewer);
    OrbSystemShutdownGuard slam_shutdown_guard(slam);
    if (ovrs::stop_requested()) {
      slam.Shutdown();
      throw std::runtime_error("shutdown requested during ORB startup");
    }

    const auto candidate_trajectory_path =
        output / "live_camera_trajectory_candidate_tum.txt";
    const auto accepted_trajectory_path =
        output / "live_camera_trajectory_tum.txt";
    std::ofstream trajectory(candidate_trajectory_path);
    std::ofstream visual_trajectory(
        output / "live_visual_tracking_trajectory_tum.txt");
    std::ofstream states(output / "live_tracking_states.csv");
    std::ofstream excitation(output / "live_imu_excitation.csv");
    if (!trajectory || !visual_trajectory || !states || !excitation) {
      throw std::runtime_error("cannot open ORB live output files");
    }
    trajectory << std::fixed << std::setprecision(9);
    visual_trajectory << std::fixed << std::setprecision(9);
    states << "timestamp_s,state,tracked_keypoints,tracked_map_points,"
              "tracking_latency_ms,imu_batch,startup_imu_gate_passed,"
              "inertial_initialized,"
              "inertial_ba2_finished,active_map_reset_count,"
              "active_map_change_index,reset_pending,"
              "pose_tx_m,pose_ty_m,pose_tz_m,"
              "pose_qx,pose_qy,pose_qz,pose_qw,"
              "stable_gate_elapsed_s,"
              "trajectory_candidate_accepted\n";
    excitation
        << "timestamp_s,imu_batch,mean_ax_m_s2,mean_ay_m_s2,mean_az_m_s2,"
           "mean_accel_magnitude_m_s2,mean_gyro_magnitude_rad_s,"
           "max_gyro_magnitude_rad_s,orb_init_accel_delta_m_s2,"
           "orb_init_accel_threshold_m_s2,orb_init_accel_gate_passed,"
           "inertial_initialized,inertial_ba2_finished,"
           "active_map_reset_count,active_map_change_index,reset_pending\n";

    ovrs::ImuSynchronizer synchronizer(stream_config.imu_queue_size);
    std::mutex synchronizer_mutex;
    std::deque<ovrs::ImuSample> orb_imu;
    std::uint64_t source_stereo_index = 0;
    std::uint64_t submitted_stereo = 0;
    std::uint64_t candidate_pose_count = 0;
    std::uint64_t visual_pose_count = 0;
    std::uint64_t lost_frame_count = 0;
    std::uint64_t invalid_image_count = 0;
    std::atomic<bool> inertial_initialized{false};
    std::atomic<bool> inertial_ba2_finished{false};
    ovrs::OrbTrajectoryGate trajectory_gate(
        orb_settings.minimum_stable_inertial_seconds,
        orb_settings.maximum_tracking_interval_seconds,
        orb_settings.maximum_preacceptance_map_resets);
    ovrs::OrbPoseRateGate pose_rate_gate(
        orb_settings.maximum_pose_linear_speed_m_s,
        orb_settings.maximum_pose_angular_speed_rad_s);
    std::optional<double> previous_tracking_timestamp_s;
    bool trajectory_acceptance_announced = false;
    std::uint64_t announced_active_map_reset_count = 0;
    bool weak_visual_support_announced = false;
    bool implausible_pose_rate_announced = false;
    bool terminal_gate_stop_requested = false;
    ImuExcitationStats excitation_stats;
    ovrs::ImuStartupGate startup_gate(
        orb_settings.gravity_m_s2,
        orb_settings.startup_maximum_gravity_error_m_s2,
        orb_settings.startup_stationary_seconds,
        orb_settings.startup_stationary_timeout_seconds,
        orb_settings.startup_maximum_acceleration_stddev_m_s2,
        orb_settings.startup_maximum_gyro_magnitude_rad_s);
    bool startup_gate_pass_announced = false;
    TrackingLatencyStats tracking_latency_stats;
    const double tracking_frame_budget_ms =
        1000.0 / static_cast<double>(orb_settings.orb_fps);
    std::atomic<std::int64_t> last_stereo_callback_ns{steady_now_ns()};
    std::atomic<std::int64_t> last_imu_callback_ns{steady_now_ns()};
    double latest_latency_ms = 0.0;
    int latest_state = ORB_SLAM3::Tracking::NO_IMAGES_YET;

    ovrs::MeasurementDispatcher dispatcher(
        stream_config.imu_queue_size, stream_config.stereo_queue_size,
        [&](const ovrs::ImuSample &sample) {
          excitation_stats.add_sample(sample);
          const auto startup_status = startup_gate.add(sample);
          if (startup_status.state == ovrs::ImuStartupGateState::Passed &&
              !startup_gate_pass_announced) {
            startup_gate_pass_announced = true;
            std::cout
                << "ORB startup IMU gate PASS: stationary acceleration "
                   "matches configured gravity. Begin smooth continuous "
                   "textured translation now, with modest rotation while "
                   "translating. Do not pause until the canonical trajectory "
                   "gate opens.\n"
                << std::flush;
          } else if (startup_status.state ==
                     ovrs::ImuStartupGateState::GravityMismatch) {
            throw std::runtime_error(
                "D435i startup IMU gravity mismatch; power-cycle or "
                "hardware-reset the device and recapture before SLAM");
          } else if (startup_status.state ==
                     ovrs::ImuStartupGateState::StationaryTimeout) {
            throw std::runtime_error(
                "D435i startup stationary IMU window was not established; "
                "restart and keep the camera still during the startup cue");
          }
          orb_imu.push_back(sample);
        },
        [&](const ovrs::StereoFrame &frame) {
          if (!valid_image(frame.camera0, orb_settings.width,
                           orb_settings.height) ||
              !valid_image(frame.camera1, orb_settings.width,
                           orb_settings.height)) {
            ++invalid_image_count;
            return;
          }
          std::vector<ORB_SLAM3::IMU::Point> imu_batch;
          FrameImuMetrics frame_imu;
          while (!orb_imu.empty() &&
                 orb_imu.front().timestamp <= frame.timestamp) {
            const auto sample = orb_imu.front();
            orb_imu.pop_front();
            frame_imu.add(sample);
            imu_batch.emplace_back(
                static_cast<float>(sample.linear_acceleration_m_s2.x),
                static_cast<float>(sample.linear_acceleration_m_s2.y),
                static_cast<float>(sample.linear_acceleration_m_s2.z),
                static_cast<float>(sample.angular_velocity_rad_s.x),
                static_cast<float>(sample.angular_velocity_rad_s.y),
                static_cast<float>(sample.angular_velocity_rad_s.z),
                sample.timestamp);
          }
          const cv::Mat left(
              frame.camera0.height, frame.camera0.width, CV_8UC1,
              frame.camera0.pixels->data(), frame.camera0.stride_bytes);
          const cv::Mat right(
              frame.camera1.height, frame.camera1.width, CV_8UC1,
              frame.camera1.pixels->data(), frame.camera1.stride_bytes);
          const auto begin = std::chrono::steady_clock::now();
          Sophus::SE3f t_camera_world;
          try {
            t_camera_world =
                slam.TrackStereo(left, right, frame.timestamp, imu_batch);
          } catch (const std::exception &exception) {
            throw std::runtime_error(
                std::string("ORB-SLAM3 TrackStereo failed: ") +
                exception.what());
          }
          latest_latency_ms =
              std::chrono::duration<double, std::milli>(
                  std::chrono::steady_clock::now() - begin)
                  .count();
          tracking_latency_stats.add(latest_latency_ms,
                                     tracking_frame_budget_ms);
          latest_state = slam.GetTrackingState();
          const bool current_inertial_initialized =
              slam.IsImuInitialized();
          const bool current_inertial_ba2_finished =
              slam.IsInertialBA2Finished();
          const std::uint64_t current_active_map_reset_count =
              slam.GetActiveMapResetCount();
          const std::uint64_t current_active_map_change_index =
              slam.GetActiveMapChangeIndex();
          const bool current_reset_pending = slam.IsResetPending();
          const bool current_tracking_pose_valid =
              latest_state == ORB_SLAM3::Tracking::OK ||
              latest_state == ORB_SLAM3::Tracking::OK_KLT;
          const bool current_tracking_lost =
              latest_state == ORB_SLAM3::Tracking::LOST ||
              latest_state == ORB_SLAM3::Tracking::RECENTLY_LOST;
          const auto keypoints = slam.GetTrackedKeyPointsUn().size();
          const auto map_points = slam.GetTrackedMapPoints();
          const auto tracked_map_points = static_cast<std::size_t>(
              std::count_if(map_points.begin(), map_points.end(),
                            [](const auto *point) { return point != nullptr; }));
          const bool visual_support_sufficient =
              current_tracking_pose_valid &&
              tracked_map_points >= orb_settings.minimum_tracked_map_points;
          const bool current_tracking_gap =
              previous_tracking_timestamp_s &&
              frame.timestamp - *previous_tracking_timestamp_s >
                  orb_settings.maximum_tracking_interval_seconds;
          const bool pose_baseline_boundary =
              !current_tracking_pose_valid || current_reset_pending ||
              current_tracking_gap ||
              current_active_map_reset_count !=
                  trajectory_gate.active_map_reset_count() ||
              current_active_map_change_index !=
                  trajectory_gate.active_map_change_index();
          if (pose_baseline_boundary) {
            pose_rate_gate.reset();
          }
          std::optional<Sophus::SE3f> t_world_camera;
          ovrs::OrbPoseRateStatus pose_rate_status;
          if (current_tracking_pose_valid) {
            t_world_camera = t_camera_world.inverse();
            const auto translation = t_world_camera->translation();
            const auto quaternion = t_world_camera->unit_quaternion();
            pose_rate_status = pose_rate_gate.update(
                frame.timestamp,
                {static_cast<double>(translation.x()),
                 static_cast<double>(translation.y()),
                 static_cast<double>(translation.z())},
                {static_cast<double>(quaternion.x()),
                 static_cast<double>(quaternion.y()),
                 static_cast<double>(quaternion.z()),
                 static_cast<double>(quaternion.w())});
          }
          previous_tracking_timestamp_s = frame.timestamp;
          const bool pose_rate_sufficient =
              current_tracking_pose_valid &&
              pose_rate_status.pose_finite_and_normalized &&
              pose_rate_status.within_limits;
          const bool startup_imu_gate_passed =
              startup_gate.status().state ==
              ovrs::ImuStartupGateState::Passed;
          const auto current_tracking_continuity =
              startup_imu_gate_passed && current_tracking_pose_valid
                  ? ovrs::OrbTrackingContinuityState::PoseValid
                  : (current_tracking_lost
                         ? ovrs::OrbTrackingContinuityState::Lost
                         : ovrs::OrbTrackingContinuityState::NotReady);
          inertial_initialized.store(current_inertial_initialized);
          inertial_ba2_finished.store(current_inertial_ba2_finished);
          const auto gate_status = trajectory_gate.update(
              frame.timestamp, current_inertial_initialized,
              current_inertial_ba2_finished,
              current_active_map_reset_count,
              current_active_map_change_index,
              current_tracking_continuity, current_reset_pending,
              visual_support_sufficient, pose_rate_sufficient);
          if (current_active_map_reset_count >
              announced_active_map_reset_count) {
            announced_active_map_reset_count =
                current_active_map_reset_count;
            if (!gate_status.acceptance_started) {
              std::cerr
                  << "ORB-SLAM3 initialization retry "
                  << trajectory_gate.preacceptance_map_reset_count() << '/'
                  << orb_settings.maximum_preacceptance_map_resets
                  << ": active map reset before canonical acceptance. "
                     "Continue smooth translation through a textured rigid "
                     "scene; rotation alone or pausing between keyframes is "
                     "insufficient.\n"
                  << std::flush;
            }
          }
          if (gate_status.accept_pose && !trajectory_acceptance_announced) {
            trajectory_acceptance_announced = true;
            std::cout
                << "ORB-SLAM3 canonical trajectory gate OPEN: inertial BA2 "
                   "is stable after bounded initialization retries, with no "
                   "post-acceptance reset, sustained visual map support, and "
                   "bounded pose rates. "
                   "Begin any "
                   "closed-loop return-to-start motion only now.\n"
                << std::flush;
          }
          if (!gate_status.acceptance_started &&
              current_inertial_ba2_finished &&
              current_tracking_pose_valid && !visual_support_sufficient &&
              !weak_visual_support_announced) {
            weak_visual_support_announced = true;
            std::cerr
                << "ORB-SLAM3 initialization WAIT: tracking is pose-valid "
                   "but visual support is "
                << tracked_map_points << '/'
                << orb_settings.minimum_tracked_map_points
                << " tracked map points. Keep a rigid textured scene in both "
                   "IR cameras and continue smooth translation.\n"
                << std::flush;
          } else if (visual_support_sufficient) {
            weak_visual_support_announced = false;
          }
          if (!gate_status.acceptance_started &&
              current_inertial_ba2_finished &&
              current_tracking_pose_valid && !pose_rate_sufficient &&
              !implausible_pose_rate_announced) {
            implausible_pose_rate_announced = true;
            std::cerr
                << "ORB-SLAM3 initialization WAIT: pose output is invalid "
                   "or exceeded the pinned rate envelope ("
                << pose_rate_status.linear_speed_m_s << " m/s, "
                << pose_rate_status.angular_speed_rad_s
                << " rad/s). Continue smooth motion; canonical stability "
                   "must be re-established.\n"
                << std::flush;
          } else if (pose_rate_sufficient) {
            implausible_pose_rate_announced = false;
          }
          if (!terminal_gate_stop_requested &&
              trajectory_gate.preacceptance_reset_limit_exceeded()) {
            terminal_gate_stop_requested = true;
            std::cerr
                << "ORB-SLAM3 initialization retry limit exceeded; stopping "
                   "this failed attempt automatically.\n"
                << std::flush;
            ovrs::request_stop();
          } else if (!terminal_gate_stop_requested &&
                     gate_status.acceptance_started &&
                     gate_status.discontinuity_detected) {
            terminal_gate_stop_requested = true;
            std::cerr
                << "ORB-SLAM3 canonical trajectory continuity failed; "
                   "stopping this attempt automatically.\n"
                << std::flush;
            ovrs::request_stop();
          }
          float orb_acceleration_delta = 0.0f;
          float orb_acceleration_threshold = 0.0f;
          const bool orb_acceleration_available =
              slam.GetImuInitAccelerationDiagnostic(
                  orb_acceleration_delta, orb_acceleration_threshold);
          if (orb_acceleration_available) {
            if (std::abs(
                    static_cast<double>(orb_acceleration_threshold) -
                    orb_settings.imu_init_acceleration_threshold_m_s2) >
                1e-6) {
              throw std::runtime_error(
                  "ORB runtime IMU initialization threshold differs from "
                  "the generated settings");
            }
            excitation_stats.add_orb_delta(
                orb_acceleration_delta, orb_acceleration_threshold);
          }
          states << std::fixed << std::setprecision(9) << frame.timestamp
                 << ',' << tracking_state_name(latest_state) << ','
                 << keypoints << ',' << tracked_map_points << ','
                 << latest_latency_ms << ',' << imu_batch.size() << ','
                 << (startup_imu_gate_passed ? 1 : 0) << ','
                 << (current_inertial_initialized ? 1 : 0) << ','
                 << (current_inertial_ba2_finished ? 1 : 0) << ','
                 << current_active_map_reset_count << ','
                 << current_active_map_change_index << ','
                 << (current_reset_pending ? 1 : 0);
          if (t_world_camera) {
            const auto translation = t_world_camera->translation();
            const auto quaternion = t_world_camera->unit_quaternion();
            states << ',' << translation.x() << ',' << translation.y() << ','
                   << translation.z() << ',' << quaternion.x() << ','
                   << quaternion.y() << ',' << quaternion.z() << ','
                   << quaternion.w();
          } else {
            states << ",,,,,,,";
          }
          states << ',' << gate_status.stable_gate_elapsed_s << ','
                 << (gate_status.accept_pose ? 1 : 0) << '\n';
          excitation << std::fixed << std::setprecision(9)
                     << frame.timestamp << ',' << frame_imu.samples;
          if (frame_imu.samples > 0) {
            const double divisor =
                static_cast<double>(frame_imu.samples);
            const ovrs::Vec3 mean_acceleration{
                frame_imu.acceleration_sum.x / divisor,
                frame_imu.acceleration_sum.y / divisor,
                frame_imu.acceleration_sum.z / divisor};
            excitation << ',' << mean_acceleration.x << ','
                       << mean_acceleration.y << ','
                       << mean_acceleration.z << ','
                       << magnitude(mean_acceleration) << ','
                       << frame_imu.gyro_magnitude_sum / divisor << ','
                       << frame_imu.gyro_magnitude_max;
          } else {
            excitation << ",,,,,,";
          }
          if (orb_acceleration_available) {
            excitation
                << ',' << orb_acceleration_delta << ','
                << orb_acceleration_threshold << ','
                << (orb_acceleration_delta >=
                            orb_acceleration_threshold
                        ? 1
                        : 0);
          } else {
            excitation << ",,,";
          }
          excitation << ','
                     << (current_inertial_initialized ? 1 : 0) << ','
                     << (current_inertial_ba2_finished ? 1 : 0) << ','
                     << current_active_map_reset_count << ','
                     << current_active_map_change_index << ','
                     << (current_reset_pending ? 1 : 0) << '\n';
          if (current_tracking_pose_valid && t_world_camera &&
              pose_rate_status.pose_finite_and_normalized) {
            const auto translation = t_world_camera->translation();
            const auto quaternion = t_world_camera->unit_quaternion();
            visual_trajectory
                << frame.timestamp << ' ' << translation.x() << ' '
                << translation.y() << ' ' << translation.z() << ' '
                << quaternion.x() << ' ' << quaternion.y() << ' '
                << quaternion.z() << ' ' << quaternion.w() << '\n';
            ++visual_pose_count;
            if (gate_status.accept_pose) {
              trajectory << frame.timestamp << ' ' << translation.x() << ' '
                         << translation.y() << ' ' << translation.z() << ' '
                         << quaternion.x() << ' ' << quaternion.y() << ' '
                         << quaternion.z() << ' ' << quaternion.w() << '\n';
              ++candidate_pose_count;
            }
          } else if (current_tracking_lost) {
            ++lost_frame_count;
          }
          ++submitted_stereo;
          if (orb_acceleration_available &&
              !inertial_initialized.load() &&
              submitted_stereo %
                      static_cast<std::uint64_t>(orb_settings.orb_fps) ==
                  0) {
            std::cout
                << "ORB IMU excitation: delta="
                << orb_acceleration_delta << " m/s^2, threshold="
                << orb_acceleration_threshold << " m/s^2, gate="
                << (orb_acceleration_delta >=
                            orb_acceleration_threshold
                        ? "PASS"
                        : "LOW")
                << '\n';
          }
        });
    dispatcher.start();
    std::cout << "ORB startup IMU gate: keep the D435i stationary for "
              << orb_settings.startup_stationary_seconds
              << " second(s), for up to "
              << orb_settings.startup_stationary_timeout_seconds
              << " seconds.\n"
              << std::flush;

    const auto motion_callback =
        [&](const ovrs::TimedVec3 &sample, bool gyroscope) {
          last_imu_callback_ns.store(steady_now_ns());
          std::lock_guard<std::mutex> lock(synchronizer_mutex);
          if (gyroscope) {
            synchronizer.add_gyroscope(sample);
          } else {
            synchronizer.add_accelerometer(sample);
          }
          while (const auto ready = synchronizer.take_ready()) {
            dispatcher.push_imu(*ready);
          }
        };
    if (!source.start(
            {[&](ovrs::StereoFrame frame) {
               last_stereo_callback_ns.store(steady_now_ns());
               const auto index = source_stereo_index++;
               if (index %
                       static_cast<std::uint64_t>(
                           orb_settings.camera_stride) !=
                   0) {
                 return;
               }
               frame.timestamp += orb_settings.camera_imu_offset_s;
               frame.camera0.timestamp = frame.timestamp;
               frame.camera1.timestamp = frame.timestamp;
               dispatcher.push_stereo(std::move(frame));
             },
             [&](ovrs::TimedVec3 sample) { motion_callback(sample, true); },
             [&](ovrs::TimedVec3 sample) { motion_callback(sample, false); }},
            &error)) {
      dispatcher.stop();
      slam.Shutdown();
      throw std::runtime_error(error);
    }
    const auto capture_started = std::chrono::steady_clock::now();
    const auto device_report = source.device_report_yaml();
    const auto actual_serial =
        ovrs::simple_yaml_scalar(device_report, "serial");
    if (actual_serial != calibrated_serial) {
      source.stop();
      dispatcher.stop();
      slam.Shutdown();
      throw std::runtime_error(
          "connected D435i serial differs from calibrated serial");
    }
    if (!ovrs::validate_runtime_imu_rate(config_path, config_text,
                                         device_report, &error) ||
        !ovrs::validate_runtime_sensor_policy(
            config_path, config_text, device_report,
            ovrs::stream_config_yaml(stream_config), &error)) {
      source.stop();
      dispatcher.stop();
      slam.Shutdown();
      throw std::runtime_error(error);
    }
    if (!ovrs::write_text(output / "device_report.yaml", device_report,
                          &error) ||
        !ovrs::write_text(output / "resolved_stream_config.yaml",
                          ovrs::stream_config_yaml(stream_config), &error)) {
      source.stop();
      dispatcher.stop();
      slam.Shutdown();
      throw std::runtime_error(error);
    }
    if (!ovrs::write_text(
            output / "run_metadata.yaml",
            "%YAML:1.0\n"
        "mode: \"experimental_pure_orbslam3_live\"\n"
        "runtime_provenance_format: "
        "\"ovrs-orbslam3-live-runtime-provenance-v7\"\n"
        "openvins_pose_consumed: false\n"
        "global_correction_fed_to_openvins: false\n"
        "viewer: " +
            std::string(viewer ? "true\n" : "false\n") +
            "calibration_state: \"" + calibration_state +
            "\"\ncamera_serial: \"" + calibrated_serial +
            "\"\nbackend_commit: \"" + orb_settings.backend_commit +
            "\"\nbackend_patch_sha256: \"" +
            orb_settings.backend_patch_sha256 +
            "\"\nbackend_library_sha256_at_build: \"" +
            OVRS_ORBSLAM3_LIBRARY_SHA256_VALUE +
            "\"\nbackend_library_sha256_at_start: \"" +
            backend_library_sha256 +
            "\"\nlive_executable_sha256_at_start: \"" +
            executable_sha256 +
            "\"\nvocabulary_sha256_at_start: \"" + vocabulary_sha256 +
            "\"\nsettings_sha256_at_start: \"" + settings_sha256 +
            "\"\nlive_bundle_manifest_sha256_at_start: \"" +
            live_manifest_sha256 +
            "\"\nlaunch_provenance_sha256: \"" +
            launch_provenance_sha256 +
            "\"\nsource_fingerprint_at_start: \"" +
            ovrs::source_fingerprint +
            "\"\ncamera_stride: " +
            std::to_string(orb_settings.camera_stride) +
            "\ncamera_imu_time_offset_s: " +
            std::to_string(orb_settings.camera_imu_offset_s) +
            "\nimu_init_acceleration_threshold_m_s2: " +
            std::to_string(
                orb_settings.imu_init_acceleration_threshold_m_s2) +
            "\nimu_init_low_motion_reset_seconds: " +
            std::to_string(
                orb_settings.imu_init_low_motion_reset_seconds) +
            "\nminimum_stable_inertial_seconds: " +
            std::to_string(
                orb_settings.minimum_stable_inertial_seconds) +
            "\nmaximum_tracking_interval_seconds: " +
            std::to_string(
                orb_settings.maximum_tracking_interval_seconds) +
            "\nmaximum_tracking_interval_factor: " +
            std::to_string(
                orb_settings.maximum_tracking_interval_factor) +
            "\nminimum_tracked_map_points: " +
            std::to_string(orb_settings.minimum_tracked_map_points) +
            "\nmaximum_pose_linear_speed_m_s: " +
            std::to_string(
                orb_settings.maximum_pose_linear_speed_m_s) +
            "\nmaximum_pose_angular_speed_rad_s: " +
            std::to_string(
                orb_settings.maximum_pose_angular_speed_rad_s) +
            "\nmaximum_preacceptance_map_resets: " +
            std::to_string(
                orb_settings.maximum_preacceptance_map_resets) +
            "\ngravity_m_s2: " +
            std::to_string(orb_settings.gravity_m_s2) +
            "\nstartup_maximum_gravity_error_m_s2: " +
            std::to_string(
                orb_settings.startup_maximum_gravity_error_m_s2) +
            "\nstartup_stationary_seconds: " +
            std::to_string(orb_settings.startup_stationary_seconds) +
            "\nstartup_stationary_timeout_seconds: " +
            std::to_string(
                orb_settings.startup_stationary_timeout_seconds) +
            "\nstartup_maximum_acceleration_stddev_m_s2: " +
            std::to_string(
                orb_settings.startup_maximum_acceleration_stddev_m_s2) +
            "\nstartup_maximum_gyro_magnitude_rad_s: " +
            std::to_string(
                orb_settings.startup_maximum_gyro_magnitude_rad_s) +
            "\nmaximum_input_stall_seconds: " +
            std::to_string(orb_settings.maximum_input_stall_seconds) +
            "\ntrajectory_acceptance_policy: "
            "\"startup_imu_pass_post_inertial_ba2_stable_tracking_minimum_"
            "visual_support_bounded_pose_rates_continuous_"
            "bounded_preacceptance_resets_zero_postacceptance_resets_"
            "no_postacceptance_map_correction\"\n"
            "closed_loop_reference_start_policy: "
            "\"post_acceptance_gate_open_operator_cue\"\n"
            "visual_tracking_trajectory_is_diagnostic_only: true"
            "\n",
            &error)) {
      source.stop();
      dispatcher.stop();
      slam.Shutdown();
      throw std::runtime_error(error);
    }

    std::cout << "ORB-SLAM3 live started (" << orb_settings.source_fps << "/"
              << orb_settings.camera_stride << "=" << orb_settings.orb_fps
              << " Hz). Press Ctrl+C to stop.\n";
    bool input_stall_detected = false;
    double maximum_observed_stereo_wall_gap_seconds = 0.0;
    double maximum_observed_imu_wall_gap_seconds = 0.0;
    while (!ovrs::stop_requested() && source.failure().empty() &&
           !source.disconnected() && dispatcher.failure().empty() &&
           !slam.isShutDown()) {
      const auto now_ns = steady_now_ns();
      const double stereo_wall_gap_seconds =
          static_cast<double>(now_ns - last_stereo_callback_ns.load()) / 1e9;
      const double imu_wall_gap_seconds =
          static_cast<double>(now_ns - last_imu_callback_ns.load()) / 1e9;
      maximum_observed_stereo_wall_gap_seconds =
          std::max(maximum_observed_stereo_wall_gap_seconds,
                   stereo_wall_gap_seconds);
      maximum_observed_imu_wall_gap_seconds =
          std::max(maximum_observed_imu_wall_gap_seconds,
                   imu_wall_gap_seconds);
      if (stereo_wall_gap_seconds >
              orb_settings.maximum_input_stall_seconds ||
          imu_wall_gap_seconds > orb_settings.maximum_input_stall_seconds) {
        input_stall_detected = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    const auto capture_stopped = std::chrono::steady_clock::now();
    source.stop();
    {
      std::lock_guard<std::mutex> lock(synchronizer_mutex);
      synchronizer.shutdown();
    }
    dispatcher.stop();
    slam.Shutdown();
    const double shutdown_duration_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      capture_stopped)
            .count();
    const bool reset_pending_at_shutdown = slam.IsResetPending();
    trajectory.flush();
    visual_trajectory.flush();
    states.flush();
    excitation.flush();
    trajectory.close();
    visual_trajectory.close();
    states.close();
    excitation.close();

    const auto source_stats = source.stats();
    const auto sync_stats = synchronizer.stats();
    const auto dispatch_stats = dispatcher.stats();
    const auto startup_status = startup_gate.status();
    const double capture_duration_s =
        std::chrono::duration<double>(capture_stopped - capture_started)
            .count();
    if (!ovrs::write_text(output / "device_report.yaml",
                          source.device_report_yaml(), &error)) {
      throw std::runtime_error(error);
    }
    std::string runtime_failure;
    if (!source.failure().empty()) {
      runtime_failure = "RealSense source failure: " + source.failure();
    } else if (source.disconnected()) {
      runtime_failure = "selected RealSense device disconnected";
    } else if (input_stall_detected) {
      runtime_failure =
          "live RealSense stereo or IMU input exceeded the wall-clock stall "
          "limit";
    } else if (startup_status.state ==
               ovrs::ImuStartupGateState::GravityMismatch) {
      runtime_failure =
          "D435i startup IMU gravity mismatch; device state is unsafe for "
          "stereo-inertial SLAM";
    } else if (startup_status.state ==
               ovrs::ImuStartupGateState::StationaryTimeout) {
      runtime_failure =
          "D435i startup stationary IMU window was not established";
    } else if (!dispatcher.failure().empty()) {
      runtime_failure =
          "measurement dispatcher failure: " + dispatcher.failure();
    } else if (source_stats.malformed_frames ||
               source_stats.dropped_camera_frames ||
               source_stats.rejected_timestamps ||
               source_stats.callback_errors || dispatch_stats.dropped_imu ||
               dispatch_stats.dropped_stereo ||
               dispatch_stats.rejected_nonmonotonic ||
               dispatch_stats.stereo_without_imu_coverage ||
               sync_stats.duplicate_timestamps ||
               sync_stats.regressing_timestamps || sync_stats.invalid_values ||
               sync_stats.dropped_capacity || invalid_image_count) {
      runtime_failure = "capture/synchronization integrity counters are nonzero";
    } else if (reset_pending_at_shutdown ||
               trajectory_gate.pending_reset_after_acceptance_observed()) {
      runtime_failure =
          "ORB-SLAM3 active-map reset was requested during live run";
    } else if (trajectory_gate.preacceptance_reset_limit_exceeded()) {
      runtime_failure =
          "ORB-SLAM3 exceeded the bounded preacceptance map-reset limit";
    } else if (
        trajectory_gate.tracking_loss_after_acceptance_count() != 0) {
      runtime_failure =
          "ORB-SLAM3 tracking was lost after trajectory acceptance";
    } else if (
        trajectory_gate.tracking_gap_after_acceptance_count() != 0) {
      runtime_failure =
          "ORB-SLAM3 frame interval exceeded the continuity limit "
          "after trajectory acceptance";
    } else if (
        trajectory_gate.visual_support_failure_after_acceptance_count() != 0) {
      runtime_failure =
          "ORB-SLAM3 visual map support fell below the canonical continuity "
          "floor after trajectory acceptance";
    } else if (
        trajectory_gate.pose_rate_failure_after_acceptance_count() != 0) {
      runtime_failure =
          "ORB-SLAM3 pose rate exceeded the canonical continuity envelope "
          "after trajectory acceptance";
    } else if (trajectory_gate.discontinuity_detected()) {
      runtime_failure =
          "ORB-SLAM3 trajectory discontinuity was detected";
    } else if (trajectory_gate.postacceptance_map_reset_count() != 0) {
      runtime_failure =
          "ORB-SLAM3 active map reset after trajectory acceptance";
    } else if (startup_status.state != ovrs::ImuStartupGateState::Passed) {
      runtime_failure = "D435i startup IMU gate did not pass";
    } else if (!inertial_initialized.load()) {
      runtime_failure = "ORB-SLAM3 inertial initialization was not established";
    } else if (!inertial_ba2_finished.load()) {
      runtime_failure =
          "ORB-SLAM3 second inertial bundle adjustment was not completed";
    } else if (!trajectory_gate.acceptance_started()) {
      runtime_failure =
          "ORB-SLAM3 inertial state did not satisfy the stability window";
    } else if (candidate_pose_count == 0) {
      runtime_failure =
          "ORB-SLAM3 produced no accepted stable tracking pose";
    }
    const bool passed = runtime_failure.empty();
    if (passed) {
      std::filesystem::rename(candidate_trajectory_path,
                              accepted_trajectory_path);
    }
    const double acceleration_magnitude_mean =
        excitation_stats.samples > 0
            ? excitation_stats.acceleration_magnitude_sum /
                  static_cast<double>(excitation_stats.samples)
            : 0.0;
    const double gyro_magnitude_mean =
        excitation_stats.samples > 0
            ? excitation_stats.gyro_magnitude_sum /
                  static_cast<double>(excitation_stats.samples)
            : 0.0;
    const double acceleration_sample_delta_rms =
        excitation_stats.consecutive_sample_deltas > 0
            ? std::sqrt(
                  excitation_stats.acceleration_sample_delta_squared_sum /
                  static_cast<double>(
                      excitation_stats.consecutive_sample_deltas))
            : 0.0;
    const double orb_acceleration_delta_mean =
        excitation_stats.orb_observed_frames > 0
            ? excitation_stats.orb_delta_sum /
                  static_cast<double>(
                      excitation_stats.orb_observed_frames)
            : 0.0;
    const double orb_acceleration_pass_ratio =
        excitation_stats.orb_observed_frames > 0
            ? static_cast<double>(excitation_stats.orb_pass_frames) /
                  static_cast<double>(
                      excitation_stats.orb_observed_frames)
            : 0.0;
    const double tracking_latency_mean_ms =
        tracking_latency_stats.samples > 0
            ? tracking_latency_stats.sum_ms /
                  static_cast<double>(tracking_latency_stats.samples)
            : 0.0;
    const double tracking_latency_frame_budget_miss_ratio =
        tracking_latency_stats.samples > 0
            ? static_cast<double>(
                  tracking_latency_stats.frame_budget_misses) /
                  static_cast<double>(tracking_latency_stats.samples)
            : 0.0;
    std::string excitation_state = "UNAVAILABLE";
    if (excitation_stats.orb_observed_frames > 0) {
      if (inertial_ba2_finished.load()) {
        excitation_state = "INERTIAL_BA2_FINISHED";
      } else if (inertial_initialized.load()) {
        excitation_state = "INERTIAL_INITIALIZED";
      } else if (excitation_stats.orb_pass_frames == 0) {
        excitation_state = "BELOW_ORB_ACCELERATION_GATE";
      } else {
        excitation_state =
            "ACCELERATION_GATE_INTERMITTENT_INIT_NOT_ESTABLISHED";
      }
    }
    const std::string summary =
        "%YAML:1.0\n"
        "state: \"" +
        std::string(passed ? "EXPERIMENTAL_RUN_COMPLETE"
                           : "EXPERIMENTAL_RUN_FAILED") +
        "\"\nsubmitted_stereo: " + std::to_string(submitted_stereo) +
        "\nsource_stereo_pairs: " +
        std::to_string(source_stats.valid_stereo_pairs) +
        "\nsynchronized_imu_samples: " +
        std::to_string(sync_stats.generated) +
        "\nvisual_pose_count: " + std::to_string(visual_pose_count) +
        "\ncandidate_pose_count: " +
        std::to_string(candidate_pose_count) +
        "\naccepted_pose_count: " +
        std::to_string(passed ? candidate_pose_count : 0) +
        "\nlost_frame_count: " + std::to_string(lost_frame_count) +
        "\ncapture_duration_s: " + std::to_string(capture_duration_s) +
        "\nshutdown_duration_s: " + std::to_string(shutdown_duration_s) +
        "\nmaximum_input_stall_seconds: " +
        std::to_string(orb_settings.maximum_input_stall_seconds) +
        "\nmaximum_observed_stereo_wall_gap_seconds: " +
        std::to_string(maximum_observed_stereo_wall_gap_seconds) +
        "\nmaximum_observed_imu_wall_gap_seconds: " +
        std::to_string(maximum_observed_imu_wall_gap_seconds) +
        "\ninput_stall_detected: " +
        std::string(input_stall_detected ? "true" : "false") +
        "\nsource_camera_rate_hz: " +
        std::to_string(capture_duration_s > 0.0
                           ? source_stats.valid_stereo_pairs /
                                 capture_duration_s
                           : 0.0) +
        "\norb_submission_rate_hz: " +
        std::to_string(capture_duration_s > 0.0
                           ? submitted_stereo / capture_duration_s
                           : 0.0) +
        "\nsynchronized_imu_rate_hz: " +
        std::to_string(capture_duration_s > 0.0
                           ? sync_stats.generated / capture_duration_s
                           : 0.0) +
        "\ndropped_imu: " + std::to_string(dispatch_stats.dropped_imu) +
        "\ndropped_stereo: " +
        std::to_string(dispatch_stats.dropped_stereo) +
        "\nrejected_nonmonotonic: " +
        std::to_string(dispatch_stats.rejected_nonmonotonic) +
        "\nstereo_before_imu_start: " +
        std::to_string(dispatch_stats.stereo_before_imu_start) +
        "\nstereo_without_imu_coverage: " +
        std::to_string(dispatch_stats.stereo_without_imu_coverage) +
        "\nstereo_discarded_on_shutdown: " +
        std::to_string(dispatch_stats.stereo_discarded_on_shutdown) +
        "\nimu_acceleration_magnitude_mean_m_s2: " +
        std::to_string(acceleration_magnitude_mean) +
        "\nimu_acceleration_magnitude_min_m_s2: " +
        std::to_string(excitation_stats.samples > 0
                           ? excitation_stats.acceleration_magnitude_min
                           : 0.0) +
        "\nimu_acceleration_magnitude_max_m_s2: " +
        std::to_string(excitation_stats.acceleration_magnitude_max) +
        "\nimu_acceleration_sample_delta_rms_m_s2: " +
        std::to_string(acceleration_sample_delta_rms) +
        "\nimu_acceleration_sample_delta_max_m_s2: " +
        std::to_string(excitation_stats.acceleration_sample_delta_max) +
        "\nimu_gyro_magnitude_mean_rad_s: " +
        std::to_string(gyro_magnitude_mean) +
        "\nimu_gyro_magnitude_max_rad_s: " +
        std::to_string(excitation_stats.gyro_magnitude_max) +
        "\nstartup_imu_gate_state: \"" +
        startup_gate_state_name(startup_status.state) + "\"" +
        "\nstartup_imu_gate_passed: " +
        std::string(startup_status.state == ovrs::ImuStartupGateState::Passed
                        ? "true"
                        : "false") +
        "\nstartup_imu_gate_samples: " +
        std::to_string(startup_status.samples) +
        "\nstartup_imu_gate_rejected_dynamic_windows: " +
        std::to_string(startup_status.rejected_dynamic_windows) +
        "\nstartup_imu_gate_window_duration_s: " +
        std::to_string(startup_status.window_duration_s) +
        "\nstartup_imu_acceleration_magnitude_mean_m_s2: " +
        std::to_string(
            startup_status.acceleration_magnitude_mean_m_s2) +
        "\nstartup_imu_acceleration_magnitude_stddev_m_s2: " +
        std::to_string(
            startup_status.acceleration_magnitude_stddev_m_s2) +
        "\nstartup_imu_maximum_gyro_magnitude_rad_s: " +
        std::to_string(startup_status.maximum_gyro_magnitude_rad_s) +
        "\nstartup_imu_gravity_error_m_s2: " +
        std::to_string(startup_status.gravity_error_m_s2) +
        "\ngravity_m_s2: " + std::to_string(orb_settings.gravity_m_s2) +
        "\nstartup_maximum_gravity_error_m_s2: " +
        std::to_string(
            orb_settings.startup_maximum_gravity_error_m_s2) +
        "\nstartup_stationary_seconds: " +
        std::to_string(orb_settings.startup_stationary_seconds) +
        "\nstartup_stationary_timeout_seconds: " +
        std::to_string(orb_settings.startup_stationary_timeout_seconds) +
        "\nstartup_maximum_acceleration_stddev_m_s2: " +
        std::to_string(
            orb_settings.startup_maximum_acceleration_stddev_m_s2) +
        "\nstartup_maximum_gyro_magnitude_rad_s: " +
        std::to_string(
            orb_settings.startup_maximum_gyro_magnitude_rad_s) +
        "\norb_imu_init_acceleration_threshold_m_s2: " +
        std::to_string(
            orb_settings.imu_init_acceleration_threshold_m_s2) +
        "\norb_imu_init_low_motion_reset_seconds: " +
        std::to_string(
            orb_settings.imu_init_low_motion_reset_seconds) +
        "\norb_imu_init_acceleration_observed_frames: " +
        std::to_string(excitation_stats.orb_observed_frames) +
        "\norb_imu_init_acceleration_pass_frames: " +
        std::to_string(excitation_stats.orb_pass_frames) +
        "\norb_imu_init_acceleration_pass_ratio: " +
        std::to_string(orb_acceleration_pass_ratio) +
        "\norb_imu_init_acceleration_delta_mean_m_s2: " +
        std::to_string(orb_acceleration_delta_mean) +
        "\norb_imu_init_acceleration_delta_max_m_s2: " +
        std::to_string(excitation_stats.orb_delta_max) +
        "\norb_imu_init_excitation_state: \"" + excitation_state + "\"" +
        "\nminimum_stable_inertial_seconds: " +
        std::to_string(
            orb_settings.minimum_stable_inertial_seconds) +
        "\nmaximum_tracking_interval_seconds: " +
        std::to_string(
            orb_settings.maximum_tracking_interval_seconds) +
        "\nmaximum_observed_tracking_interval_seconds: " +
        std::to_string(
            trajectory_gate
                .maximum_observed_tracking_interval_seconds()) +
        "\nmaximum_pose_linear_speed_m_s: " +
        std::to_string(orb_settings.maximum_pose_linear_speed_m_s) +
        "\nmaximum_observed_pose_linear_speed_m_s: " +
        std::to_string(
            pose_rate_gate.maximum_observed_linear_speed_m_s()) +
        "\nmaximum_pose_angular_speed_rad_s: " +
        std::to_string(orb_settings.maximum_pose_angular_speed_rad_s) +
        "\nmaximum_observed_pose_angular_speed_rad_s: " +
        std::to_string(
            pose_rate_gate.maximum_observed_angular_speed_rad_s()) +
        "\npose_rate_gate_failure_count: " +
        std::to_string(pose_rate_gate.failure_count()) +
        "\ntracking_latency_samples: " +
        std::to_string(tracking_latency_stats.samples) +
        "\ntracking_latency_mean_ms: " +
        std::to_string(tracking_latency_mean_ms) +
        "\ntracking_latency_maximum_ms: " +
        std::to_string(tracking_latency_stats.maximum_ms) +
        "\ntracking_frame_budget_ms: " +
        std::to_string(tracking_frame_budget_ms) +
        "\ntracking_latency_frame_budget_miss_count: " +
        std::to_string(tracking_latency_stats.frame_budget_misses) +
        "\ntracking_latency_frame_budget_miss_ratio: " +
        std::to_string(tracking_latency_frame_budget_miss_ratio) +
        "\never_inertial_initialized: " +
        std::string(trajectory_gate.ever_inertial_initialized() ? "true"
                                                               : "false") +
        "\ninertial_initialized: " +
        std::string(inertial_initialized.load() ? "true" : "false") +
        "\never_inertial_ba2_finished: " +
        std::string(trajectory_gate.ever_inertial_ba2_finished() ? "true"
                                                                : "false") +
        "\ninertial_ba2_finished: " +
        std::string(inertial_ba2_finished.load() ? "true" : "false") +
        "\nactive_map_reset_count: " +
        std::to_string(trajectory_gate.active_map_reset_count()) +
        "\nmaximum_preacceptance_map_resets: " +
        std::to_string(orb_settings.maximum_preacceptance_map_resets) +
        "\npreacceptance_map_reset_count: " +
        std::to_string(trajectory_gate.preacceptance_map_reset_count()) +
        "\npostacceptance_map_reset_count: " +
        std::to_string(trajectory_gate.postacceptance_map_reset_count()) +
        "\npreacceptance_reset_limit_exceeded: " +
        std::string(trajectory_gate.preacceptance_reset_limit_exceeded()
                        ? "true"
                        : "false") +
        "\nactive_map_change_index: " +
        std::to_string(trajectory_gate.active_map_change_index()) +
        "\nmap_change_after_acceptance: " +
        std::string(trajectory_gate.map_change_after_acceptance() ? "true"
                                                                 : "false") +
        "\npending_reset_observed: " +
        std::string(trajectory_gate.pending_reset_observed() ? "true"
                                                             : "false") +
        "\npending_reset_after_acceptance_observed: " +
        std::string(
            trajectory_gate.pending_reset_after_acceptance_observed()
                ? "true"
                : "false") +
        "\nreset_pending_at_shutdown: " +
        std::string(reset_pending_at_shutdown ? "true" : "false") +
        "\ninertial_regression_count: " +
        std::to_string(trajectory_gate.inertial_regression_count()) +
        "\ninertial_ba2_regression_count: " +
        std::to_string(
            trajectory_gate.inertial_ba2_regression_count()) +
        "\ntracking_loss_after_acceptance_count: " +
        std::to_string(
            trajectory_gate.tracking_loss_after_acceptance_count()) +
        "\ntracking_gap_after_acceptance_count: " +
        std::to_string(
            trajectory_gate.tracking_gap_after_acceptance_count()) +
        "\nminimum_tracked_map_points: " +
        std::to_string(orb_settings.minimum_tracked_map_points) +
        "\nvisual_support_failure_after_acceptance_count: " +
        std::to_string(
            trajectory_gate
                .visual_support_failure_after_acceptance_count()) +
        "\npose_rate_failure_after_acceptance_count: " +
        std::to_string(
            trajectory_gate
                .pose_rate_failure_after_acceptance_count()) +
        "\ntrajectory_acceptance_started: " +
        std::string(trajectory_gate.acceptance_started() ? "true"
                                                         : "false") +
        "\ntrajectory_discontinuity_detected: " +
        std::string(trajectory_gate.discontinuity_detected() ? "true"
                                                             : "false") +
        "\naccepted_trajectory_file: \"" +
        std::string(passed ? "live_camera_trajectory_tum.txt" : "") +
        "\"\nrejected_candidate_trajectory_file: \"" +
        std::string(passed ? ""
                           : "live_camera_trajectory_candidate_tum.txt") +
        "\"\nvisual_diagnostic_trajectory_file: "
        "\"live_visual_tracking_trajectory_tum.txt\"" +
        "\nlast_tracking_state: \"" + tracking_state_name(latest_state) +
        "\"\nlast_tracking_latency_ms: " +
        std::to_string(latest_latency_ms) + "\nruntime_failure: \"" +
        runtime_failure + "\"\n";
    if (!ovrs::write_text(output / "run_summary.yaml", summary, &error)) {
      throw std::runtime_error(error);
    }
    if (passed) {
      std::filesystem::remove(incomplete);
      std::cout << "ORB-SLAM3 live stopped cleanly: " << output << '\n';
      return 0;
    }
    std::cerr << "ORB-SLAM3 live failed: " << runtime_failure << '\n';
    return 5;
  } catch (const std::exception &exception) {
    std::cerr << "ORB-SLAM3 live error: " << exception.what() << '\n';
    return 4;
  }
#endif
}
