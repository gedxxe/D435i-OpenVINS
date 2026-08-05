#include "ovrs/realsense_source.hpp"

#include "ovrs/stereo_synchronizer.hpp"
#include "ovrs/timestamp_normalizer.hpp"

#include <librealsense2/rs.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace ovrs {
namespace {

std::string info_or_unknown(const rs2::device &device, rs2_camera_info key) {
  return device.supports(key) ? device.get_info(key) : "unknown";
}

std::string domain_name(rs2_timestamp_domain domain) {
  return rs2_timestamp_domain_to_string(domain);
}

const char *gyro_sensitivity_description(int index) {
  switch (index) {
  case 0:
    return "61.0 mDeg/s per LSB";
  case 1:
    return "30.5 mDeg/s per LSB";
  case 2:
    return "15.3 mDeg/s per LSB";
  case 3:
    return "7.6 mDeg/s per LSB";
  case 4:
    return "3.8 mDeg/s per LSB";
  default:
    return "invalid";
  }
}

struct MotionChoice {
  int fps = 0;
  bool found = false;
};

MotionChoice choose_motion_rate(const rs2::device &device,
                                rs2_stream stream, int requested) {
  for (const auto &sensor : device.query_sensors()) {
    for (const auto &profile : sensor.get_stream_profiles()) {
      if (profile.stream_type() == stream &&
          profile.format() == RS2_FORMAT_MOTION_XYZ32F &&
          profile.fps() == requested) {
        return {requested, true};
      }
    }
  }
  return {};
}

std::optional<rs2::stream_profile>
find_motion_profile(const rs2::device &device, rs2_stream stream, int fps) {
  for (const auto &sensor : device.query_sensors()) {
    for (const auto &profile : sensor.get_stream_profiles()) {
      if (profile.stream_type() == stream && profile.fps() == fps &&
          profile.format() == RS2_FORMAT_MOTION_XYZ32F) {
        return profile;
      }
    }
  }
  return std::nullopt;
}

bool has_video_profile(const rs2::device &device, int index, int width,
                       int height, int fps) {
  for (const auto &sensor : device.query_sensors()) {
    for (const auto &profile : sensor.get_stream_profiles()) {
      if (profile.stream_type() != RS2_STREAM_INFRARED ||
          profile.stream_index() != index || profile.fps() != fps ||
          profile.format() != RS2_FORMAT_Y8) {
        continue;
      }
      const auto video = profile.as<rs2::video_stream_profile>();
      if (video.width() == width && video.height() == height) {
        return true;
      }
    }
  }
  return false;
}

std::string describe_ir_profiles(const rs2::device &device) {
  std::ostringstream out;
  for (const auto &sensor : device.query_sensors()) {
    for (const auto &profile : sensor.get_stream_profiles()) {
      if (profile.stream_type() != RS2_STREAM_INFRARED ||
          profile.format() != RS2_FORMAT_Y8) {
        continue;
      }
      const auto video = profile.as<rs2::video_stream_profile>();
      out << " IR" << profile.stream_index() << "=" << video.width() << "x"
          << video.height() << "@" << profile.fps();
    }
  }
  return out.str();
}

bool sensor_is_selected(
    const rs2::sensor &sensor,
    const RealSenseSource::StreamSelection &selection) {
  for (const auto &profile : sensor.get_stream_profiles()) {
    const auto stream = profile.stream_type();
    if ((selection.stereo && stream == RS2_STREAM_INFRARED) ||
        (selection.motion &&
         (stream == RS2_STREAM_GYRO || stream == RS2_STREAM_ACCEL))) {
      return true;
    }
  }
  return false;
}

bool finite_motion_intrinsics(
    const rs2_motion_device_intrinsic &intrinsics) {
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 4; ++column) {
      if (!std::isfinite(intrinsics.data[row][column])) {
        return false;
      }
    }
    if (!std::isfinite(intrinsics.noise_variances[row]) ||
        !std::isfinite(intrinsics.bias_variances[row])) {
      return false;
    }
  }
  return true;
}

void write_motion_intrinsics(
    std::ostream &out, const char *name,
    const rs2_motion_device_intrinsic &intrinsics) {
  out << name << ":\n  scale_cross_axis_bias:\n";
  for (int row = 0; row < 3; ++row) {
    out << "    - [";
    for (int column = 0; column < 4; ++column) {
      if (column != 0) {
        out << ", ";
      }
      out << intrinsics.data[row][column];
    }
    out << "]\n";
  }
  out << "  noise_variances: [" << intrinsics.noise_variances[0] << ", "
      << intrinsics.noise_variances[1] << ", "
      << intrinsics.noise_variances[2] << "]\n"
      << "  bias_variances: [" << intrinsics.bias_variances[0] << ", "
      << intrinsics.bias_variances[1] << ", "
      << intrinsics.bias_variances[2] << "]\n";
}

} // namespace

class RealSenseSource::Impl {
public:
  explicit Impl(StreamConfig config, StreamSelection selection)
      : config(std::move(config)),
        selection(selection),
        pipeline(context),
        stereo_sync(config.stereo_tolerance_ms * 1e-3) {
    if (!selection.stereo && !selection.motion) {
      throw std::invalid_argument(
          "RealSense stream selection must enable stereo or motion");
    }
    context.set_devices_changed_callback(
        [this](rs2::event_information &information) noexcept {
          handle_devices_changed(information);
        });
  }

  void handle_devices_changed(rs2::event_information &information) noexcept {
    try {
      std::lock_guard<std::mutex> lock(mutex);
      if (!active || !information.was_removed(device)) {
        return;
      }
      disconnected_flag = true;
      if (failure_message.empty()) {
        failure_message = "selected RealSense device disconnected";
      }
    } catch (const std::exception &e) {
      std::lock_guard<std::mutex> lock(mutex);
      if (!active) {
        return;
      }
      if (failure_message.empty()) {
        failure_message =
            std::string("RealSense hotplug callback failed: ") + e.what();
      }
      ++statistics.callback_errors;
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex);
      if (!active) {
        return;
      }
      if (failure_message.empty()) {
        failure_message = "unknown RealSense hotplug callback error";
      }
      ++statistics.callback_errors;
    }
  }

  bool start(const Callbacks &new_callbacks, std::string *error) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (active) {
        if (error) {
          *error = "RealSense source is already running";
        }
        return false;
      }
    }
    try {
      const auto devices = context.query_devices();
      if (devices.size() == 0) {
        if (error) {
          *error = "no RealSense device found; connect the D435i over USB 3 "
                   "and check udev permissions";
        }
        return false;
      }
      bool selected = false;
      for (auto &&candidate : devices) {
        const std::string candidate_serial =
            info_or_unknown(candidate, RS2_CAMERA_INFO_SERIAL_NUMBER);
        const std::string candidate_name =
            info_or_unknown(candidate, RS2_CAMERA_INFO_NAME);
        if (is_d435i_device_name(candidate_name) &&
            (config.serial.empty() || candidate_serial == config.serial)) {
          device = candidate;
          selected = true;
          break;
        }
      }
      if (!selected) {
        if (error) {
          *error = config.serial.empty()
                       ? "no Intel RealSense D435i was found"
                       : "requested D435i serial was not found: " +
                             config.serial;
        }
        return false;
      }
      selected_serial = info_or_unknown(device, RS2_CAMERA_INFO_SERIAL_NUMBER);
      if (selection.stereo &&
          (!has_video_profile(device, 1, config.width, config.height,
                              config.camera_fps) ||
           !has_video_profile(device, 2, config.width, config.height,
                              config.camera_fps))) {
        if (error) {
          *error = "requested stereo Y8 profile is unavailable: " +
                   std::to_string(config.width) + "x" +
                   std::to_string(config.height) + "@" +
                   std::to_string(config.camera_fps) +
                   "; supported Y8 profiles:" + describe_ir_profiles(device) +
                   "; USB descriptor=" +
                   info_or_unknown(
                       device, RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR) +
                   ". Connect through USB 3 for the requested bandwidth or "
                   "explicitly select one common supported profile. Automatic "
                   "fallback is disabled to preserve calibration consistency.";
        }
        return false;
      }
      if (selection.motion) {
        const auto gyro =
            choose_motion_rate(device, RS2_STREAM_GYRO, config.gyro_fps);
        const auto accel =
            choose_motion_rate(device, RS2_STREAM_ACCEL, config.accel_fps);
        if (!gyro.found || !accel.found) {
          if (error) {
            *error =
                "requested D435i gyro or accelerometer rate is unavailable; "
                "calibration/runtime rate fallback is disabled";
          }
          return false;
        }
        selected_gyro_fps = gyro.fps;
        selected_accel_fps = accel.fps;
        const auto gyro_profile =
            find_motion_profile(device, RS2_STREAM_GYRO, selected_gyro_fps);
        const auto accel_profile =
            find_motion_profile(device, RS2_STREAM_ACCEL, selected_accel_fps);
        if (!gyro_profile || !accel_profile) {
          if (error) {
            *error = "could not resolve selected motion stream profiles";
          }
          return false;
        }
        gyro_intrinsics =
            gyro_profile->as<rs2::motion_stream_profile>()
                .get_motion_intrinsics();
        accel_intrinsics =
            accel_profile->as<rs2::motion_stream_profile>()
                .get_motion_intrinsics();
        if (!finite_motion_intrinsics(gyro_intrinsics) ||
            !finite_motion_intrinsics(accel_intrinsics)) {
          if (error) {
            *error = "selected D435i motion intrinsics contain nonfinite "
                     "values";
          }
          return false;
        }
        accel_to_gyro = accel_profile->get_extrinsics_to(*gyro_profile);
      }
      {
        std::lock_guard<std::mutex> lock(mutex);
        callbacks = {};
      }
      rs_config.enable_device(selected_serial);
      if (selection.stereo) {
        rs_config.enable_stream(RS2_STREAM_INFRARED, 1, config.width,
                                config.height, RS2_FORMAT_Y8,
                                config.camera_fps);
        rs_config.enable_stream(RS2_STREAM_INFRARED, 2, config.width,
                                config.height, RS2_FORMAT_Y8,
                                config.camera_fps);
      }
      if (selection.motion) {
        rs_config.enable_stream(RS2_STREAM_GYRO,
                                RS2_FORMAT_MOTION_XYZ32F,
                                selected_gyro_fps);
        rs_config.enable_stream(RS2_STREAM_ACCEL,
                                RS2_FORMAT_MOTION_XYZ32F,
                                selected_accel_fps);
        // Resolve first so the option is changed on the exact device instance
        // that pipeline.start() will open. A separately enumerated device has
        // its own option object and cannot prove the pipeline acquisition
        // state.
        const auto resolved_profile = rs_config.resolve(pipeline);
        configure_gyro_sensitivity(resolved_profile.get_device());
      }
      {
        std::lock_guard<std::mutex> lock(mutex);
        failure_message.clear();
        disconnected_flag = false;
        statistics = {};
        last_frameset_number = 0;
        reset_infrared_frame_metadata();
        normalizer.reset();
      }
      profile = pipeline.start(rs_config, [this](rs2::frame frame) {
        handle_frame(std::move(frame));
      });
      {
        std::lock_guard<std::mutex> lock(mutex);
        active = true;
      }
      configure_sensor_options(profile.get_device());
      if (selection.motion) {
        verify_active_gyro_sensitivity(profile.get_device());
      }
      {
        // Keep consuming SDK callbacks during option configuration, but do
        // not expose those warm-up measurements to the caller. Quiesce the
        // callback before resetting timestamp/statistics state and publishing
        // the requested callbacks so the first delivered sample starts a
        // clean capture epoch.
        std::lock_guard<std::mutex> callback_lock(callback_mutex);
        {
          std::lock_guard<std::mutex> timestamp_lock(timestamp_mutex);
          normalizer.reset();
        }
        {
          std::lock_guard<std::mutex> lock(mutex);
          callbacks = new_callbacks;
          statistics = {};
          last_frameset_number = 0;
          reset_infrared_frame_metadata();
          build_report();
        }
      }
      return true;
    } catch (const rs2::error &e) {
      stop();
      if (error) {
        *error = std::string("librealsense error: ") + e.what();
      }
      return false;
    } catch (const std::exception &e) {
      stop();
      if (error) {
        *error = e.what();
      }
      return false;
    }
  }

  void configure_gyro_sensitivity(const rs2::device &selected_device) {
    gyro_sensitivity_available = false;
    active_gyro_sensitivity = -1;
    for (const auto &sensor : selected_device.query_sensors()) {
      if (!sensor.supports(RS2_OPTION_GYRO_SENSITIVITY)) {
        continue;
      }
      if (gyro_sensitivity_available) {
        throw std::runtime_error(
            "multiple D435i sensors expose RS2_OPTION_GYRO_SENSITIVITY");
      }
      gyro_sensitivity_available = true;
      const auto range =
          sensor.get_option_range(RS2_OPTION_GYRO_SENSITIVITY);
      const float requested =
          static_cast<float>(config.gyro_sensitivity);
      if (requested < range.min || requested > range.max ||
          range.step <= 0.0F ||
          std::abs(std::round((requested - range.min) / range.step) *
                           range.step +
                       range.min -
                       requested) >
              1e-4F) {
        throw std::runtime_error(
            "requested gyro sensitivity is outside the D435i option range");
      }
      // This option becomes read-only once streaming starts. Set it
      // explicitly so separate processes cannot inherit an ambiguous SDK or
      // firmware state.
      sensor.set_option(RS2_OPTION_GYRO_SENSITIVITY, requested);
      const float active =
          sensor.get_option(RS2_OPTION_GYRO_SENSITIVITY);
      if (std::abs(active - requested) > 1e-4F) {
        throw std::runtime_error(
            "RealSense gyro sensitivity did not accept the requested value "
            "before pipeline start");
      }
      active_gyro_sensitivity =
          static_cast<int>(std::lround(active));
    }
    if (!gyro_sensitivity_available) {
      throw std::runtime_error(
          "D435i motion streaming requires "
          "RS2_OPTION_GYRO_SENSITIVITY; use the pinned firmware and "
          "librealsense versions");
    }
  }

  void verify_active_gyro_sensitivity(const rs2::device &active_device) {
    int matching_sensors = 0;
    for (const auto &sensor : active_device.query_sensors()) {
      if (!sensor.supports(RS2_OPTION_GYRO_SENSITIVITY)) {
        continue;
      }
      ++matching_sensors;
      const float active = sensor.get_option(RS2_OPTION_GYRO_SENSITIVITY);
      const float requested = static_cast<float>(config.gyro_sensitivity);
      if (std::abs(active - requested) > 1e-4F) {
        throw std::runtime_error(
            "RealSense gyro sensitivity changed when streaming started");
      }
      active_gyro_sensitivity = static_cast<int>(std::lround(active));
    }
    if (matching_sensors != 1) {
      throw std::runtime_error(
          "streaming D435i must expose gyro sensitivity on exactly one "
          "sensor");
    }
  }

  void configure_sensor_options(const rs2::device &active_device) {
    motion_correction_available = false;
    motion_correction_active = false;
    global_time_available = false;
    global_time_active = false;
    selected_timestamp_sensors = 0;
    global_time_supported_sensors = 0;
    for (const auto &sensor : active_device.query_sensors()) {
      if (sensor_is_selected(sensor, selection)) {
        ++selected_timestamp_sensors;
        if (!sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
          throw std::runtime_error(
              "a selected D435i sensor does not expose "
              "RS2_OPTION_GLOBAL_TIME_ENABLED; timestamp policy cannot be "
              "verified");
        }
        ++global_time_supported_sensors;
        sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED,
                          config.global_time_enabled ? 1.0F : 0.0F);
        const bool enabled =
            sensor.get_option(RS2_OPTION_GLOBAL_TIME_ENABLED) > 0.5F;
        if (enabled != config.global_time_enabled) {
          throw std::runtime_error(
              "RealSense global-time option did not accept the requested "
              "state after pipeline start");
        }
      }
      if (selection.stereo && sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
        sensor.set_option(RS2_OPTION_EMITTER_ENABLED,
                          config.emitter_enabled ? 1.0F : 0.0F);
      }
      if (selection.stereo &&
          sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
        sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE,
                          config.auto_exposure ? 1.0F : 0.0F);
      }
      if (selection.motion &&
          sensor.supports(RS2_OPTION_ENABLE_MOTION_CORRECTION)) {
        motion_correction_available = true;
        sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION,
                          config.motion_correction_enabled ? 1.0F : 0.0F);
        const bool enabled =
            sensor.get_option(RS2_OPTION_ENABLE_MOTION_CORRECTION) > 0.5F;
        if (enabled != config.motion_correction_enabled) {
          throw std::runtime_error(
              "RealSense motion-correction option did not accept the "
              "requested state after pipeline start");
        }
        motion_correction_active = enabled;
      }
    }
    if (selected_timestamp_sensors == 0) {
      throw std::runtime_error(
          "no selected RealSense sensor was found for timestamp policy");
    }
    global_time_available =
        global_time_supported_sensors == selected_timestamp_sensors;
    global_time_active = config.global_time_enabled;
    if (selection.motion && config.motion_correction_enabled &&
        !motion_correction_available) {
      throw std::runtime_error(
          "D435i motion correction was requested but the selected device "
          "does not expose RS2_OPTION_ENABLE_MOTION_CORRECTION");
    }
  }

  void reset_infrared_frame_metadata() noexcept {
    infrared_metadata_samples = 0;
    infrared_exposure_us_min = std::numeric_limits<double>::infinity();
    infrared_exposure_us_max = 0.0;
    infrared_exposure_us_sum = 0.0;
    infrared_exposure_us_last = 0.0;
    infrared_gain_level_min = std::numeric_limits<double>::infinity();
    infrared_gain_level_max = 0.0;
    infrared_gain_level_sum = 0.0;
    infrared_gain_level_last = 0.0;
  }

  void handle_frame(rs2::frame frame) noexcept {
    std::lock_guard<std::mutex> callback_lock(callback_mutex);
    try {
      if (selection.stereo) {
        if (auto frames = frame.as<rs2::frameset>()) {
          const auto left = frames.get_infrared_frame(1);
          const auto right = frames.get_infrared_frame(2);
          if (!left || !right) {
            std::lock_guard<std::mutex> lock(mutex);
            ++statistics.malformed_frames;
            return;
          }
          std::optional<double> exposure_us;
          std::optional<double> gain_level;
          try {
            if (left.supports_frame_metadata(
                    RS2_FRAME_METADATA_ACTUAL_EXPOSURE) &&
                left.supports_frame_metadata(RS2_FRAME_METADATA_GAIN_LEVEL)) {
              exposure_us = static_cast<double>(left.get_frame_metadata(
                  RS2_FRAME_METADATA_ACTUAL_EXPOSURE));
              gain_level = static_cast<double>(
                  left.get_frame_metadata(RS2_FRAME_METADATA_GAIN_LEVEL));
            }
          } catch (const rs2::error &) {
            // Per-frame exposure diagnostics must never make acquisition fail.
            exposure_us.reset();
            gain_level.reset();
          }
          auto make_image = [this, &frames](const rs2::video_frame &video,
                                            int camera_id,
                                            const char *stream_name) {
            ImageFrame result;
            result.raw_timestamp_ms = video.get_timestamp();
            const std::string timestamp_domain =
                domain_name(video.get_frame_timestamp_domain());
            TimestampNormalizer::Result normalized;
            {
              std::lock_guard<std::mutex> timestamp_lock(timestamp_mutex);
              normalized = normalizer.normalize(
                  stream_name, result.raw_timestamp_ms, timestamp_domain);
            }
            if (!normalized.accepted) {
              throw std::runtime_error(normalized.error);
            }
            result.timestamp = normalized.seconds;
            result.frameset_number = frames.get_frame_number();
            result.camera_id = camera_id;
            result.width = video.get_width();
            result.height = video.get_height();
            result.stride_bytes = video.get_stride_in_bytes();
            result.format = "Y8";
            if (result.width <= 0 || result.height <= 0 ||
                result.stride_bytes < result.width || !video.get_data() ||
                static_cast<std::size_t>(result.stride_bytes) >
                    std::numeric_limits<std::size_t>::max() /
                        static_cast<std::size_t>(result.height)) {
              throw std::runtime_error(
                  std::string("malformed Y8 buffer on ") + stream_name);
            }
            const std::size_t bytes =
                static_cast<std::size_t>(result.stride_bytes) * result.height;
            result.pixels =
                std::make_shared<std::vector<std::uint8_t>>(bytes);
            std::memcpy(result.pixels->data(), video.get_data(), bytes);
            return result;
          };
          auto pair = stereo_sync.pair(make_image(left, 0, "infrared_1"),
                                       make_image(right, 1, "infrared_2"));
          StereoCallback callback;
          {
            std::lock_guard<std::mutex> lock(mutex);
            ++statistics.received_framesets;
            const auto frameset_number = frames.get_frame_number();
            if (last_frameset_number != 0 &&
                frameset_number > last_frameset_number + 1) {
              statistics.dropped_camera_frames +=
                  2 * (frameset_number - last_frameset_number - 1);
            }
            last_frameset_number = frameset_number;
            if (exposure_us && gain_level && std::isfinite(*exposure_us) &&
                std::isfinite(*gain_level) && *exposure_us > 0.0 &&
                *gain_level >= 0.0) {
              ++infrared_metadata_samples;
              infrared_exposure_us_min =
                  std::min(infrared_exposure_us_min, *exposure_us);
              infrared_exposure_us_max =
                  std::max(infrared_exposure_us_max, *exposure_us);
              infrared_exposure_us_sum += *exposure_us;
              infrared_exposure_us_last = *exposure_us;
              infrared_gain_level_min =
                  std::min(infrared_gain_level_min, *gain_level);
              infrared_gain_level_max =
                  std::max(infrared_gain_level_max, *gain_level);
              infrared_gain_level_sum += *gain_level;
              infrared_gain_level_last = *gain_level;
            }
            if (!pair) {
              ++statistics.malformed_frames;
            }
            callback = callbacks.stereo;
          }
          if (pair && callback) {
            {
              std::lock_guard<std::mutex> lock(mutex);
              ++statistics.valid_stereo_pairs;
            }
            callback(std::move(*pair));
          }
          return;
        }
      }
      if (selection.motion) {
        if (auto motion = frame.as<rs2::motion_frame>()) {
          const auto stream = motion.get_profile().stream_type();
          if (stream != RS2_STREAM_GYRO && stream != RS2_STREAM_ACCEL) {
            throw std::runtime_error(
                "unexpected motion stream in RealSense callback");
          }
          const char *name =
              stream == RS2_STREAM_GYRO ? "gyro" : "accelerometer";
          TimedVec3 sample;
          sample.raw_timestamp_ms = motion.get_timestamp();
          TimestampNormalizer::Result normalized;
          {
            std::lock_guard<std::mutex> timestamp_lock(timestamp_mutex);
            normalized = normalizer.normalize(
                name, sample.raw_timestamp_ms,
                domain_name(motion.get_frame_timestamp_domain()));
          }
          if (!normalized.accepted) {
            std::lock_guard<std::mutex> lock(mutex);
            ++statistics.rejected_timestamps;
            return;
          }
          sample.timestamp = normalized.seconds;
          const auto data = motion.get_motion_data();
          if (!std::isfinite(data.x) || !std::isfinite(data.y) ||
              !std::isfinite(data.z)) {
            throw std::runtime_error(
                std::string("nonfinite motion measurement on ") + name);
          }
          sample.original_sensor_value = {data.x, data.y, data.z};
          sample.original_sensor_value_available = true;
          sample.value = sample.original_sensor_value;
          if (stream == RS2_STREAM_GYRO) {
            sample.value = {
                sample.original_sensor_value.x * config.gyro_scale_factor,
                sample.original_sensor_value.y * config.gyro_scale_factor,
                sample.original_sensor_value.z * config.gyro_scale_factor};
          } else {
            const Vec3 raw = sample.original_sensor_value;
            sample.value = {
                accel_to_gyro.rotation[0] * raw.x +
                    accel_to_gyro.rotation[3] * raw.y +
                    accel_to_gyro.rotation[6] * raw.z,
                accel_to_gyro.rotation[1] * raw.x +
                    accel_to_gyro.rotation[4] * raw.y +
                    accel_to_gyro.rotation[7] * raw.z,
                accel_to_gyro.rotation[2] * raw.x +
                    accel_to_gyro.rotation[5] * raw.y +
                    accel_to_gyro.rotation[8] * raw.z};
          }
          MotionCallback callback;
          {
            std::lock_guard<std::mutex> lock(mutex);
            if (stream == RS2_STREAM_GYRO) {
              ++statistics.received_gyro;
              callback = callbacks.gyro;
            } else {
              ++statistics.received_accel;
              callback = callbacks.accel;
            }
          }
          if (callback) {
            callback(sample);
          }
          return;
        }
      }
    } catch (const rs2::error &e) {
      std::lock_guard<std::mutex> lock(mutex);
      failure_message =
          std::string("librealsense frame callback failed: ") + e.what();
      ++statistics.callback_errors;
    } catch (const std::exception &e) {
      std::lock_guard<std::mutex> lock(mutex);
      failure_message =
          std::string("RealSense frame callback failed: ") + e.what();
      ++statistics.callback_errors;
      ++statistics.rejected_timestamps;
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex);
      failure_message = "unknown RealSense callback error";
      ++statistics.callback_errors;
    }
  }

  void build_report() {
    std::ostringstream out;
    out << "%YAML:1.0\n"
        << "device_name: \"" << info_or_unknown(device, RS2_CAMERA_INFO_NAME)
        << "\"\nserial: \"" << selected_serial << "\"\nfirmware: \""
        << info_or_unknown(device, RS2_CAMERA_INFO_FIRMWARE_VERSION)
        << "\"\nusb_type: \""
        << info_or_unknown(device, RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)
        << "\"\nlibrealsense_version: \"" << RS2_API_VERSION_STR << "\"\n"
        << "stereo_stream_enabled: "
        << (selection.stereo ? "true" : "false") << "\n"
        << "motion_streams_enabled: "
        << (selection.motion ? "true" : "false") << "\n"
        << "infrared_profile: \"";
    if (selection.stereo) {
      out << config.width << "x" << config.height << " Y8 @"
          << config.camera_fps;
    } else {
      out << "disabled";
    }
    out << "\"\n"
        << "emitter_enabled: "
        << (selection.stereo && config.emitter_enabled ? "true" : "false")
        << "\n"
        << "auto_exposure: "
        << (selection.stereo && config.auto_exposure ? "true" : "false")
        << "\n"
        << "motion_correction_requested: "
        << (selection.motion && config.motion_correction_enabled ? "true"
                                                                 : "false")
        << "\n"
        << "motion_correction_available: "
        << (motion_correction_available ? "true" : "false") << "\n"
        << "motion_correction_active: "
        << (motion_correction_active ? "true" : "false") << "\n"
        << "global_time_requested: "
        << (config.global_time_enabled ? "true" : "false") << "\n"
        << "global_time_available: "
        << (global_time_available ? "true" : "false") << "\n"
        << "global_time_active: "
        << (global_time_active ? "true" : "false") << "\n"
        << "timestamp_sensor_count: " << selected_timestamp_sensors << "\n"
        << "global_time_supported_sensor_count: "
        << global_time_supported_sensors << "\n"
        << "gyro_requested_rate_hz: "
        << (selection.motion ? config.gyro_fps : 0) << "\n"
        << "gyro_rate_hz: " << selected_gyro_fps << "\n"
        << "gyro_sensitivity_requested: "
        << (selection.motion ? config.gyro_sensitivity : -1) << "\n"
        << "gyro_sensitivity_available: "
        << (gyro_sensitivity_available ? "true" : "false") << "\n"
        << "gyro_sensitivity_active: " << active_gyro_sensitivity << "\n"
        << "gyro_sensitivity_description: \""
        << gyro_sensitivity_description(active_gyro_sensitivity) << "\"\n"
        << "gyro_scale_factor_configured: "
        << config.gyro_scale_factor << "\n"
        << "gyro_scale_factor_applied: "
        << (selection.motion ? config.gyro_scale_factor : 1.0) << "\n"
        << "gyro_profile_fallback: "
        << (!selection.motion || selected_gyro_fps == config.gyro_fps
                ? "false"
                : "true")
        << "\n"
        << "accelerometer_requested_rate_hz: "
        << (selection.motion ? config.accel_fps : 0) << "\n"
        << "accelerometer_rate_hz: " << selected_accel_fps << "\n"
        << "accelerometer_profile_fallback: "
        << (!selection.motion || selected_accel_fps == config.accel_fps
                ? "false"
                : "true")
        << "\n";
    if (selection.motion) {
      out << "imu_frame: \"gyroscope stream coordinates\"\n"
          << "gyroscope_value_unit: "
             "\"rad/s after configured scale factor is applied to the "
             "librealsense motion API value\"\n"
          << "accelerometer_value_unit: "
             "\"m/s^2 from librealsense motion API\"\n"
          << "accelerometer_axis_policy: "
             "\"rotation from librealsense accel-to-gyro extrinsics\"\n";
      write_motion_intrinsics(out, "gyroscope_factory_intrinsics",
                              gyro_intrinsics);
      write_motion_intrinsics(out, "accelerometer_factory_intrinsics",
                              accel_intrinsics);
    }
    report = out.str();
  }

  void stop() {
    bool should_stop = false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      should_stop = active;
      active = false;
    }
    if (should_stop) {
      try {
        pipeline.stop();
      } catch (const std::exception &e) {
        std::lock_guard<std::mutex> lock(mutex);
        if (failure_message.empty()) {
          failure_message =
              std::string("failed to stop RealSense pipeline: ") + e.what();
        }
      } catch (...) {
        std::lock_guard<std::mutex> lock(mutex);
        if (failure_message.empty()) {
          failure_message = "unknown failure while stopping RealSense pipeline";
        }
      }
    }
  }

  bool is_disconnected() {
    std::lock_guard<std::mutex> lock(mutex);
    return disconnected_flag;
  }

  StreamConfig config;
  StreamSelection selection;
  Callbacks callbacks;
  mutable std::mutex mutex;
  std::mutex timestamp_mutex;
  std::mutex callback_mutex;
  // The callback owned by context uses device and mutex. Their declaration
  // order keeps them alive until after pipeline and context are destroyed.
  rs2::device device;
  rs2::context context;
  rs2::pipeline pipeline;
  rs2::config rs_config;
  rs2::pipeline_profile profile;
  rs2_extrinsics accel_to_gyro{};
  rs2_motion_device_intrinsic gyro_intrinsics{};
  rs2_motion_device_intrinsic accel_intrinsics{};
  TimestampNormalizer normalizer;
  StereoSynchronizer stereo_sync;
  Stats statistics;
  std::string selected_serial;
  int selected_gyro_fps = 0;
  int selected_accel_fps = 0;
  std::uint64_t last_frameset_number = 0;
  std::uint64_t infrared_metadata_samples = 0;
  double infrared_exposure_us_min = std::numeric_limits<double>::infinity();
  double infrared_exposure_us_max = 0.0;
  double infrared_exposure_us_sum = 0.0;
  double infrared_exposure_us_last = 0.0;
  double infrared_gain_level_min = std::numeric_limits<double>::infinity();
  double infrared_gain_level_max = 0.0;
  double infrared_gain_level_sum = 0.0;
  double infrared_gain_level_last = 0.0;
  std::string failure_message;
  std::string report;
  bool active = false;
  bool disconnected_flag = false;
  bool motion_correction_available = false;
  bool motion_correction_active = false;
  bool gyro_sensitivity_available = false;
  int active_gyro_sensitivity = -1;
  bool global_time_available = false;
  bool global_time_active = false;
  int selected_timestamp_sensors = 0;
  int global_time_supported_sensors = 0;
};

RealSenseSource::RealSenseSource(StreamConfig config)
    : RealSenseSource(std::move(config), StreamSelection{true, true}) {}
RealSenseSource::RealSenseSource(StreamConfig config,
                                 StreamSelection selection)
    : impl_(std::make_unique<Impl>(std::move(config), selection)) {}
RealSenseSource::~RealSenseSource() { stop(); }
bool RealSenseSource::start(const Callbacks &callbacks, std::string *error) {
  return impl_->start(callbacks, error);
}
void RealSenseSource::stop() { impl_->stop(); }
bool RealSenseSource::running() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->active;
}
bool RealSenseSource::disconnected() const {
  return impl_->is_disconnected();
}
std::string RealSenseSource::failure() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->failure_message;
}
RealSenseSource::Stats RealSenseSource::stats() const {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->statistics;
}
std::string RealSenseSource::device_report_yaml() const {
  std::string report;
  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    report = impl_->report;
    std::ostringstream metadata;
    metadata << "infrared_frame_metadata_available: "
             << (impl_->infrared_metadata_samples > 0 ? "true" : "false")
             << "\n"
             << "infrared_frame_metadata_samples: "
             << impl_->infrared_metadata_samples << "\n";
    if (impl_->infrared_metadata_samples > 0) {
      const double samples =
          static_cast<double>(impl_->infrared_metadata_samples);
      metadata << "infrared_exposure_us_last: "
               << impl_->infrared_exposure_us_last << "\n"
               << "infrared_exposure_us_min: "
               << impl_->infrared_exposure_us_min << "\n"
               << "infrared_exposure_us_max: "
               << impl_->infrared_exposure_us_max << "\n"
               << "infrared_exposure_us_mean: "
               << impl_->infrared_exposure_us_sum / samples << "\n"
               << "infrared_gain_level_last: "
               << impl_->infrared_gain_level_last << "\n"
               << "infrared_gain_level_min: "
               << impl_->infrared_gain_level_min << "\n"
               << "infrared_gain_level_max: "
               << impl_->infrared_gain_level_max << "\n"
               << "infrared_gain_level_mean: "
               << impl_->infrared_gain_level_sum / samples << "\n";
    }
    report += metadata.str();
  }
  {
    std::lock_guard<std::mutex> lock(impl_->timestamp_mutex);
    report += "timestamp_domains:\n";
    const auto streams = impl_->normalizer.observed_streams();
    if (streams.empty()) {
      report += "  observed: \"not sampled yet\"\n";
    } else {
      for (const auto &stream : streams) {
        report += "  " + stream + ": \"" + impl_->normalizer.domain() +
                  "\"\n";
      }
    }
  }
  return report;
}

} // namespace ovrs
