#include "ovrs/app_support.hpp"
#include "ovrs/config.hpp"
#include "ovrs/realsense_source.hpp"
#include "ovrs/yaml_utils.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <thread>

#ifdef OVRS_HAS_REALSENSE
#include <librealsense2/rs.hpp>
#endif

namespace {
void help() {
  std::cout
      << "Usage: ovrs_inspect [--serial SERIAL] [--duration SECONDS]\n"
         "                    [--stream-config STREAMS.yaml]\n"
         "                    [--width PIXELS --height PIXELS "
         "--camera-fps HZ]\n"
         "                    [--gyro-fps HZ --accel-fps HZ]\n"
         "                    [--gyro-sensitivity 0..4]\n"
         "                    [--gyro-scale-factor FACTOR]\n"
         "                    [--emitter on|off] "
         "[--auto-exposure on|off]\n"
         "                    [--motion-correction on|off]\n"
         "                    [--global-time on|off]\n"
         "                    [--imu-queue COUNT --stereo-queue COUNT]\n"
         "                    [--stereo-tolerance-ms MILLISECONDS]\n"
         "                    [--export REPORT.yaml]\n"
         "                    [--export-calibration CALIBRATION.yaml]\n"
         "                    [--version]\n"
         "Enumerates D435i stereo IR and IMU capabilities and samples the "
         "requested streams.\n";
}
#ifdef OVRS_HAS_REALSENSE
void write_transform(std::ostream &out, const rs2_extrinsics &extrinsics,
                     const std::string &indent) {
  for (int row = 0; row < 3; ++row) {
    out << indent << "- [";
    for (int column = 0; column < 3; ++column) {
      // librealsense stores this rotation matrix column-major.
      out << extrinsics.rotation[row + column * 3] << ", ";
    }
    out << extrinsics.translation[row] << "]\n";
  }
  out << indent << "- [0.0, 0.0, 0.0, 1.0]\n";
}

struct CameraCalibrationProfile {
  rs2::video_stream_profile profile;
  rs2_intrinsics intrinsics;
};

bool factory_model_is_openvins_compatible(
    const CameraCalibrationProfile &camera, std::string *error) {
  const auto &intrinsics = camera.intrinsics;
  if (intrinsics.model != RS2_DISTORTION_BROWN_CONRADY &&
      intrinsics.model != RS2_DISTORTION_NONE) {
    if (error) {
      *error = std::string("RealSense distortion model ") +
               rs2_distortion_to_string(intrinsics.model) +
               " cannot be represented safely as OpenVINS radtan";
    }
    return false;
  }
  constexpr float coefficient_tolerance = 1e-12F;
  if (std::abs(intrinsics.coeffs[4]) > coefficient_tolerance) {
    if (error) {
      *error = "the fifth Brown-Conrady coefficient is nonzero, but "
               "OpenVINS radtan accepts only four coefficients";
    }
    return false;
  }
  if (intrinsics.model == RS2_DISTORTION_NONE &&
      std::any_of(std::begin(intrinsics.coeffs),
                  std::end(intrinsics.coeffs), [](float value) {
                    return std::abs(value) > coefficient_tolerance;
                  })) {
    if (error) {
      *error = "the profile declares no distortion but reports nonzero "
               "coefficients";
    }
    return false;
  }
  return true;
}

std::optional<rs2_intrinsics>
read_intrinsics(const rs2::video_stream_profile &profile,
                std::string *error) {
  try {
    const auto intrinsics = profile.get_intrinsics();
    const bool finite_coefficients =
        std::all_of(std::begin(intrinsics.coeffs),
                    std::end(intrinsics.coeffs),
                    [](float value) { return std::isfinite(value); });
    if (intrinsics.width != profile.width() ||
        intrinsics.height != profile.height() || intrinsics.fx <= 0.0F ||
        intrinsics.fy <= 0.0F || !std::isfinite(intrinsics.fx) ||
        !std::isfinite(intrinsics.fy) || !std::isfinite(intrinsics.ppx) ||
        !std::isfinite(intrinsics.ppy) || !finite_coefficients) {
      if (error) {
        *error = "librealsense returned invalid intrinsics";
      }
      return std::nullopt;
    }
    return intrinsics;
  } catch (const rs2::error &e) {
    if (error) {
      *error = e.what();
    }
    return std::nullopt;
  }
}

void write_camera(std::ostream &out, const std::string &name,
                  const CameraCalibrationProfile &camera,
                  const rs2::stream_profile &gyro,
                  const std::string &overlap) {
  const auto &intrinsics = camera.intrinsics;
  out << name << ":\n"
      << "  # OpenVINS v2.7 contract: T_imu_cam maps this IR camera into\n"
      << "  # gyro/IMU coordinates (rotation camera-to-IMU; camera origin in "
         "IMU).\n"
      << "  T_imu_cam:\n";
  write_transform(out, camera.profile.get_extrinsics_to(gyro), "    ");
  out << "  cam_overlaps: [" << overlap << "]\n"
      << "  camera_model: pinhole\n"
      << "  realsense_distortion_model: \""
      << rs2_distortion_to_string(intrinsics.model) << "\"\n"
      << "  realsense_distortion_coeffs: [" << intrinsics.coeffs[0] << ", "
      << intrinsics.coeffs[1] << ", " << intrinsics.coeffs[2] << ", "
      << intrinsics.coeffs[3] << ", " << intrinsics.coeffs[4] << "]\n"
      << "  # Review compatibility before using OpenVINS radtan.\n"
      << "  distortion_model: radtan\n"
      << "  distortion_coeffs: [" << intrinsics.coeffs[0] << ", "
      << intrinsics.coeffs[1] << ", " << intrinsics.coeffs[2] << ", "
      << intrinsics.coeffs[3] << "]\n"
      << "  intrinsics: [" << intrinsics.fx << ", " << intrinsics.fy << ", "
      << intrinsics.ppx << ", " << intrinsics.ppy << "]\n"
      << "  resolution: [" << intrinsics.width << ", " << intrinsics.height
      << "]\n"
      << "  timeshift_cam_imu: 0.0 # factory API does not calibrate time offset\n";
}
#endif
} // namespace

int main(int argc, char **argv) {
  if (ovrs::has_flag(argc, argv, "--version")) {
    std::cout << ovrs::version_summary("ovrs_inspect");
    return 0;
  }
  if (ovrs::has_flag(argc, argv, "--help") ||
      ovrs::has_flag(argc, argv, "-h")) {
    help();
    return 0;
  }
  std::string argument_error;
  auto value_options = ovrs::stream_cli_value_options();
  value_options.insert(value_options.end(),
                       {"--duration", "--stream-config", "--export",
                        "--export-calibration"});
  if (!ovrs::validate_cli_arguments(argc, argv, value_options, {},
                                    &argument_error)) {
    std::cerr << argument_error << '\n';
    return 2;
  }
#ifndef OVRS_HAS_REALSENSE
  std::cerr << "ovrs_inspect was built without librealsense2. Reconfigure "
               "with OVRS_ENABLE_HARDWARE=ON on Ubuntu.\n";
  return 3;
#else
  try {
    ovrs::StreamConfig config;
    const auto stream_config_path =
        ovrs::value_after(argc, argv, "--stream-config");
    std::string error;
    if (!stream_config_path.empty() &&
        !ovrs::load_stream_config(stream_config_path, &config, &error)) {
      throw std::runtime_error(error);
    }
    if (!ovrs::apply_stream_config_cli(argc, argv, &config, &error)) {
      throw std::runtime_error(error);
    }
    const std::string requested_serial = config.serial;
    const double duration =
        ovrs::parse_double_strict(
            ovrs::value_after(argc, argv, "--duration", "5"), "--duration");
    if (!std::isfinite(duration) || duration <= 0.0) {
      std::cerr << "--duration must be a positive finite number\n";
      return 2;
    }
    ovrs::install_signal_handlers();
    rs2::context context;
    const auto devices = context.query_devices();
    if (devices.size() == 0) {
      std::cerr << "No RealSense device found. Connect the D435i over USB 3, "
                   "then check rs-enumerate-devices and udev permissions.\n";
      return 2;
    }
    rs2::device selected;
    std::string selected_serial;
    std::optional<CameraCalibrationProfile> selected_ir0;
    std::optional<CameraCalibrationProfile> selected_ir1;
    std::optional<rs2::stream_profile> selected_gyro;
    std::optional<rs2::stream_profile> selected_accel;
    std::string requested_intrinsics_error;
    for (auto &&device : devices) {
      const std::string serial =
          device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)
              ? device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
              : "unknown";
      std::cout << "Device: "
                << (device.supports(RS2_CAMERA_INFO_NAME)
                        ? device.get_info(RS2_CAMERA_INFO_NAME)
                        : "unknown")
                << "\n  serial: " << serial << "\n  firmware: "
                << (device.supports(RS2_CAMERA_INFO_FIRMWARE_VERSION)
                        ? device.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION)
                        : "unknown")
                << "\n  USB: "
                << (device.supports(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)
                        ? device.get_info(RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR)
                        : "unknown")
                << '\n';
      const bool choose_device =
          ovrs::is_d435i_device_name(
              device.supports(RS2_CAMERA_INFO_NAME)
                  ? device.get_info(RS2_CAMERA_INFO_NAME)
                  : "") &&
          (requested_serial.empty() ? !selected
                                    : serial == requested_serial);
      if (choose_device) {
        selected = device;
        selected_serial = serial;
      }
      for (const auto &sensor : device.query_sensors()) {
        std::cout << "  sensor: " << sensor.get_info(RS2_CAMERA_INFO_NAME)
                  << '\n';
        for (const auto &profile : sensor.get_stream_profiles()) {
          if (profile.stream_type() != RS2_STREAM_INFRARED &&
              profile.stream_type() != RS2_STREAM_GYRO &&
              profile.stream_type() != RS2_STREAM_ACCEL) {
            continue;
          }
          std::cout << "    " << profile.stream_name() << " index "
                    << profile.stream_index() << " "
                    << rs2_format_to_string(profile.format()) << " @"
                    << profile.fps();
          if (auto video = profile.as<rs2::video_stream_profile>()) {
            std::cout << " " << video.width() << "x" << video.height();
            const bool is_requested_camera =
                serial == selected_serial &&
                profile.stream_type() == RS2_STREAM_INFRARED &&
                profile.format() == RS2_FORMAT_Y8 &&
                video.width() == config.width &&
                video.height() == config.height &&
                profile.fps() == config.camera_fps;
            if (profile.stream_type() == RS2_STREAM_INFRARED &&
                profile.format() == RS2_FORMAT_Y8) {
              std::string intrinsics_error;
              const auto intrinsics =
                  read_intrinsics(video, &intrinsics_error);
              if (intrinsics) {
                std::cout << " fx=" << intrinsics->fx
                          << " fy=" << intrinsics->fy
                          << " ppx=" << intrinsics->ppx
                          << " ppy=" << intrinsics->ppy << " distortion="
                          << rs2_distortion_to_string(intrinsics->model)
                          << " coeffs=[" << intrinsics->coeffs[0] << ","
                          << intrinsics->coeffs[1] << ","
                          << intrinsics->coeffs[2] << ","
                          << intrinsics->coeffs[3] << ","
                          << intrinsics->coeffs[4] << "]";
              } else {
                std::cout << " intrinsics=unavailable";
              }
              if (is_requested_camera && intrinsics) {
                const CameraCalibrationProfile selected_profile{video,
                                                                *intrinsics};
                if (profile.stream_index() == 1) {
                  selected_ir0 = selected_profile;
                } else if (profile.stream_index() == 2) {
                  selected_ir1 = selected_profile;
                }
              } else if (is_requested_camera) {
                if (!requested_intrinsics_error.empty()) {
                  requested_intrinsics_error += "; ";
                }
                requested_intrinsics_error +=
                    std::string("IR") +
                    std::to_string(profile.stream_index()) + ": " +
                    intrinsics_error;
              }
            }
          } else if (serial == selected_serial &&
                     profile.stream_type() == RS2_STREAM_GYRO &&
                     profile.fps() == config.gyro_fps) {
            selected_gyro = profile;
          } else if (serial == selected_serial &&
                     profile.stream_type() == RS2_STREAM_ACCEL &&
                     profile.fps() == config.accel_fps) {
            selected_accel = profile;
          }
          std::cout << '\n';
        }
      }
    }
    if (!selected) {
      std::cerr << (requested_serial.empty()
                        ? "No Intel RealSense D435i was found."
                        : "Requested D435i serial was not found: " +
                              requested_serial)
                << '\n';
      return 2;
    }
    if (selected_ir0 && selected_ir1 && selected_gyro) {
      const auto stereo_extrinsics =
          selected_ir0->profile.get_extrinsics_to(selected_ir1->profile);
      const auto imu0_extrinsics =
          selected_gyro->get_extrinsics_to(selected_ir0->profile);
      const auto imu1_extrinsics =
          selected_gyro->get_extrinsics_to(selected_ir1->profile);
      std::cout << "T_ir2_ir1 (IR1 coordinates to IR2 coordinates):\n";
      write_transform(std::cout, stereo_extrinsics, "  ");
      std::cout << "T_ir1_imu (gyro/IMU coordinates to IR1):\n";
      write_transform(std::cout, imu0_extrinsics, "  ");
      std::cout << "T_ir2_imu (gyro/IMU coordinates to IR2):\n";
      write_transform(std::cout, imu1_extrinsics, "  ");
      if (selected_accel) {
        std::cout << "T_gyro_accel (accelerometer coordinates to gyro):\n";
        write_transform(std::cout,
                        selected_accel->get_extrinsics_to(*selected_gyro),
                        "  ");
      }
    }

    config.serial = selected_serial;
    ovrs::RealSenseSource source(config);
    std::atomic<std::uint64_t> stereo{0}, gyro{0}, accel{0};
    if (!source.start({[&](ovrs::StereoFrame) { ++stereo; },
                       [&](ovrs::TimedVec3) { ++gyro; },
                       [&](ovrs::TimedVec3) { ++accel; }},
                      &error)) {
      std::cerr << error << '\n';
      return 2;
    }
    const auto start = std::chrono::steady_clock::now();
    const auto deadline =
        start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(duration));
    while (!ovrs::stop_requested() &&
           std::chrono::steady_clock::now() < deadline &&
           source.failure().empty() && !source.disconnected()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    source.stop();
    const bool interrupted = ovrs::stop_requested();
    const bool disconnected = source.disconnected();
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
    const auto stats = source.stats();
    std::cout << "Sampled " << elapsed << " s: stereo=" << stereo / elapsed
              << " Hz, gyro=" << gyro / elapsed
              << " Hz, accel=" << accel / elapsed << " Hz\n"
              << "Malformed frames: " << stats.malformed_frames
              << ", dropped camera frames: "
              << stats.dropped_camera_frames
              << ", rejected timestamps: " << stats.rejected_timestamps
              << ", callback errors: " << stats.callback_errors << '\n'
              << "Timestamp monotonic/domain check: "
              << (stats.rejected_timestamps == 0 ? "PASS" : "FAILED")
              << '\n';
    if (!source.failure().empty()) {
      std::cerr << "Sampling failure: " << source.failure() << '\n';
      return 4;
    }
    if (disconnected) {
      std::cerr << "Sampling failed: selected D435i disconnected; "
                   "report/calibration exports were not written.\n";
      return 4;
    }
    if (interrupted) {
      std::cerr << "Sampling interrupted; report/calibration exports were "
                   "not written.\n";
      return 130;
    }
    if (stats.valid_stereo_pairs == 0 || stats.received_gyro == 0 ||
        stats.received_accel == 0 || stats.malformed_frames != 0 ||
        stats.dropped_camera_frames != 0 ||
        stats.rejected_timestamps != 0 || stats.callback_errors != 0) {
      std::cerr << "Sampling failed integrity checks; report/calibration "
                   "exports were not written.\n";
      return 4;
    }
    const std::string sampled_report = source.device_report_yaml();
    std::cout << "Timestamp domains: IR1="
              << ovrs::simple_yaml_scalar(sampled_report, "infrared_1")
              << ", IR2="
              << ovrs::simple_yaml_scalar(sampled_report, "infrared_2")
              << ", gyro="
              << ovrs::simple_yaml_scalar(sampled_report, "gyro")
              << ", accelerometer="
              << ovrs::simple_yaml_scalar(sampled_report, "accelerometer")
              << '\n';
    std::ostringstream factory_sections;
    factory_sections
        << "calibration_state: \"BOOTSTRAP_UNVERIFIED\"\n"
        << "calibrated_serial: \""
        << selected_serial << "\"\n";
    std::string factory_compatibility_error;
    const bool factory_profiles_complete =
        selected_ir0 && selected_ir1 && selected_gyro && selected_accel;
    const bool factory_models_compatible =
        factory_profiles_complete &&
        factory_model_is_openvins_compatible(
            *selected_ir0, &factory_compatibility_error) &&
        factory_model_is_openvins_compatible(
            *selected_ir1, &factory_compatibility_error);
    if (factory_models_compatible) {
      factory_sections << "T_gyro_accel:\n";
      write_transform(factory_sections,
                      selected_accel->get_extrinsics_to(*selected_gyro),
                      "  ");
      write_camera(factory_sections, "cam0", *selected_ir0, *selected_gyro,
                   "1");
      write_camera(factory_sections, "cam1", *selected_ir1, *selected_gyro,
                   "0");
    }
    const std::string complete_report =
        sampled_report + factory_sections.str();
    const std::string report_path =
        ovrs::value_after(argc, argv, "--export");
    if (!report_path.empty() &&
        !ovrs::write_text(report_path, complete_report, &error)) {
      std::cerr << error << '\n';
      return 4;
    }
    if (!report_path.empty()) {
      std::cout << "Wrote device report: " << report_path << '\n';
    }
    const std::string calibration_path =
        ovrs::value_after(argc, argv, "--export-calibration");
    if (!calibration_path.empty()) {
      std::ostringstream calibration;
      calibration
          << "%YAML:1.0\n"
          << "# Reviewable factory report only. This is "
             "BOOTSTRAP_UNVERIFIED, not flight-ready Kalibr calibration.\n"
          << factory_sections.str();
      if (!factory_profiles_complete) {
        std::cerr << "Cannot export calibration: requested " << config.width
                  << 'x' << config.height << '@' << config.camera_fps
                  << " Y8 stereo, " << config.gyro_fps << " Hz gyro, or "
                  << config.accel_fps
                  << " Hz accelerometer profile was not found or did not "
                     "provide valid calibration.";
        if (!requested_intrinsics_error.empty()) {
          std::cerr << " Intrinsics query failed: "
                    << requested_intrinsics_error;
        }
        std::cerr << '\n';
        return 4;
      }
      if (!factory_models_compatible) {
        std::cerr << "Cannot export calibration: "
                  << factory_compatibility_error
                  << ". Preserve the device report and use a supported "
                     "profile or a reviewed Kalibr conversion.\n";
        return 4;
      }
      if (!ovrs::write_text(calibration_path, calibration.str(), &error)) {
        std::cerr << error << '\n';
        return 4;
      }
      std::cout << "Wrote BOOTSTRAP_UNVERIFIED factory calibration: "
                << calibration_path << '\n';
    }
    return 0;
  } catch (const rs2::error &e) {
    std::cerr << "librealsense error: " << e.what() << '\n';
    return 4;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 4;
  }
#endif
}
