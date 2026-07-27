#include "ovrs/app_support.hpp"
#include "ovrs/bounded_queue.hpp"
#include "ovrs/config.hpp"
#include "ovrs/imu_synchronizer.hpp"
#include "ovrs/live_viewer.hpp"
#include "ovrs/measurement_dispatcher.hpp"
#include "ovrs/openvins_estimator.hpp"
#include "ovrs/realsense_source.hpp"
#include "ovrs/trajectory.hpp"
#include "ovrs/version.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

namespace {
void help() {
  std::cout << "Usage: ovrs_live --config ESTIMATOR.yaml [--serial SERIAL]\n"
               "                 [--output RUN_DIR] [--headless|--viewer]\n"
               "                 [--viewer-history COUNT]\n"
               "                 [--allow-unverified-calibration]\n"
               "                 [--stream-config STREAMS.yaml]\n"
               "                 [--width PIXELS --height PIXELS "
               "--camera-fps HZ]\n"
               "                 [--gyro-fps HZ --accel-fps HZ]\n"
               "                 [--emitter on|off] [--auto-exposure on|off]\n"
               "                 [--motion-correction on|off]\n"
               "                 [--imu-queue COUNT --stereo-queue COUNT]\n"
               "                 [--stereo-tolerance-ms MILLISECONDS]\n"
               "                 [--logging-config LOGGING.yaml]\n"
               "                 [--pose-print-rate-hz 1]\n"
               "                 [--diagnostics-rate-hz 1]\n"
               "                 [--version]\n"
               "Runs stereo-inertial OpenVINS. Headless is the default. "
               "--viewer shows IR1/IR2 and the live X-Y trajectory; close it "
               "with q or Esc. Estimation requires KALIBR_VERIFIED unless "
               "the explicit diagnostic override is supplied.\n";
}
} // namespace

int main(int argc, char **argv) {
  if (ovrs::has_flag(argc, argv, "--version")) {
    std::cout << ovrs::version_summary("ovrs_live");
    return 0;
  }
  if (ovrs::has_flag(argc, argv, "--help") ||
      ovrs::has_flag(argc, argv, "-h")) {
    help();
    return 0;
  }
  std::string argument_error;
  auto value_options = ovrs::stream_cli_value_options();
  value_options.insert(
      value_options.end(),
      {"--config", "--output", "--stream-config", "--logging-config",
       "--pose-print-rate-hz", "--diagnostics-rate-hz",
       "--viewer-history"});
  if (!ovrs::validate_cli_arguments(
          argc, argv, value_options,
          {"--headless", "--viewer", "--allow-unverified-calibration"},
                                    &argument_error)) {
    std::cerr << argument_error << '\n';
    return 2;
  }
  if (ovrs::has_flag(argc, argv, "--headless") &&
      ovrs::has_flag(argc, argv, "--viewer")) {
    std::cerr << "--headless and --viewer are mutually exclusive\n";
    return 2;
  }
#if !defined(OVRS_HAS_REALSENSE) || !defined(OVRS_HAS_OPENVINS)
  std::cerr << "ovrs_live was built without the RealSense/OpenVINS "
               "integration. Use scripts/build_ubuntu.sh.\n";
  return 3;
#else
  const std::string config_path =
      ovrs::value_after(argc, argv, "--config");
  if (config_path.empty() || !std::filesystem::exists(config_path)) {
    std::cerr << "--config must name an existing OpenVINS YAML file\n";
    return 2;
  }
  std::ifstream config_in(config_path);
  if (!config_in) {
    std::cerr << "Cannot open configuration: " << config_path << '\n';
    return 2;
  }
  const std::string config_text((std::istreambuf_iterator<char>(config_in)),
                                std::istreambuf_iterator<char>());
  const bool allow_unverified =
      ovrs::has_flag(argc, argv, "--allow-unverified-calibration");
  const std::string calibration_state =
      ovrs::simple_yaml_scalar(config_text, "calibration_state");
  const bool unverified_override_active =
      allow_unverified && calibration_state == "BOOTSTRAP_UNVERIFIED";
  std::string identity_error;
  if (!ovrs::validate_estimator_configuration(
          config_path, config_text, &identity_error)) {
    std::cerr << identity_error << ". Run ovrs_inspect and create a "
                 "serial-specific reviewed calibration, then prepare a local "
                 "bootstrap configuration with "
                 "scripts/prepare_bootstrap_config.sh.\n";
    return 2;
  }
  if (!ovrs::validate_estimation_calibration_state(
          config_text, allow_unverified, &identity_error)) {
    std::cerr << identity_error << '\n';
    return 2;
  }
  if (unverified_override_active) {
    std::cerr << "WARNING: running BOOTSTRAP_UNVERIFIED estimator "
                 "diagnostics; trajectory accuracy is not certified.\n";
  }
  const std::string requested_serial =
      ovrs::value_after(argc, argv, "--serial");
  const std::string calibrated_serial =
      ovrs::simple_yaml_scalar(config_text, "calibrated_serial");
  if (!requested_serial.empty() && !calibrated_serial.empty() &&
      requested_serial != calibrated_serial) {
    std::cerr << "Connected/requested serial " << requested_serial
              << " does not match calibrated_serial " << calibrated_serial
              << ". Refusing to run.\n";
    return 2;
  }
  const auto output = ovrs::value_after(
      argc, argv, "--output",
      (std::filesystem::path("runs") / ("live_" + ovrs::utc_timestamp()))
          .string());
  ovrs::install_signal_handlers();
  try {
    const bool viewer_enabled = ovrs::has_flag(argc, argv, "--viewer");
    const std::size_t viewer_history = ovrs::bounded_size_option(
        argc, argv, "--viewer-history",
        ovrs::LiveViewer::Options::default_trajectory_points,
        ovrs::LiveViewer::Options::minimum_trajectory_points,
        ovrs::LiveViewer::Options::maximum_allowed_trajectory_points);
    double pose_print_rate_hz = 1.0;
    double diagnostics_rate_hz = 1.0;
    const std::string logging_config_path =
        ovrs::value_after(argc, argv, "--logging-config");
    std::string logging_config_text;
    if (!logging_config_path.empty()) {
      std::ifstream logging_config_input(logging_config_path);
      if (!logging_config_input) {
        throw std::runtime_error("cannot open logging configuration: " +
                                 logging_config_path);
      }
      logging_config_text.assign(
          std::istreambuf_iterator<char>(logging_config_input),
          std::istreambuf_iterator<char>());
      const auto pose_rate =
          ovrs::simple_yaml_scalar(logging_config_text, "pose_print_rate_hz");
      const auto diagnostics_rate = ovrs::simple_yaml_scalar(
          logging_config_text, "diagnostics_rate_hz");
      if (!pose_rate.empty()) {
        pose_print_rate_hz =
            ovrs::parse_double_strict(pose_rate, "pose_print_rate_hz");
      }
      if (!diagnostics_rate.empty()) {
        diagnostics_rate_hz = ovrs::parse_double_strict(
            diagnostics_rate, "diagnostics_rate_hz");
      }
    }
    const auto pose_rate_option =
        ovrs::value_after(argc, argv, "--pose-print-rate-hz");
    const auto diagnostics_rate_option =
        ovrs::value_after(argc, argv, "--diagnostics-rate-hz");
    if (!pose_rate_option.empty()) {
      pose_print_rate_hz = ovrs::parse_double_strict(
          pose_rate_option, "--pose-print-rate-hz");
    }
    if (!diagnostics_rate_option.empty()) {
      diagnostics_rate_hz = ovrs::parse_double_strict(
          diagnostics_rate_option, "--diagnostics-rate-hz");
    }
    if (!std::isfinite(pose_print_rate_hz) || pose_print_rate_hz <= 0.0 ||
        !std::isfinite(diagnostics_rate_hz) ||
        diagnostics_rate_hz <= 0.0) {
      throw std::runtime_error(
          "pose and diagnostics rates must be positive finite values");
    }
    const std::chrono::duration<double> pose_print_period(
        1.0 / pose_print_rate_hz);
    const std::chrono::duration<double> diagnostics_period(
        1.0 / diagnostics_rate_hz);
    ovrs::StreamConfig stream_config;
    stream_config.imu_queue_size = 4096;
    stream_config.stereo_queue_size = 32;
    const auto stream_config_path =
        ovrs::value_after(argc, argv, "--stream-config");
    std::string stream_config_error;
    if (!stream_config_path.empty() &&
        !ovrs::load_stream_config(stream_config_path, &stream_config,
                                  &stream_config_error)) {
      throw std::runtime_error(stream_config_error);
    }
    stream_config.serial =
        requested_serial.empty() ? calibrated_serial : requested_serial;
    if (!ovrs::apply_stream_config_cli(
            argc, argv, &stream_config, &stream_config_error)) {
      throw std::runtime_error(stream_config_error);
    }
    if (!ovrs::validate_camera_calibration_resolution(
            config_path, config_text, stream_config.width,
            stream_config.height, &stream_config_error)) {
      throw std::runtime_error(stream_config_error);
    }
    std::unique_ptr<ovrs::LiveViewer> viewer;
    if (viewer_enabled) {
      ovrs::LiveViewer::Options viewer_options;
      viewer_options.maximum_trajectory_points = viewer_history;
      viewer_options.calibration_state = calibration_state;
      viewer = std::make_unique<ovrs::LiveViewer>(viewer_options);
      if (!viewer->open(&stream_config_error)) {
        throw std::runtime_error(stream_config_error);
      }
    }
    ovrs::OpenVinsEstimator estimator(config_path);
    ovrs::RunWriter writer;
    std::string error;
    if (!writer.open(output, &error)) {
      std::cerr << error << '\n';
      return 4;
    }
    if (!writer.log("ovrs_live started", &error)) {
      throw std::runtime_error(error);
    }
    std::filesystem::copy_file(
        config_path, std::filesystem::path(output) / "resolved_config.yaml",
        std::filesystem::copy_options::overwrite_existing);
    if (!ovrs::copy_config_dependency(
            config_path, config_text, "relative_config_imu",
            std::filesystem::path(output) / "resolved_imu.yaml", &error) ||
        !ovrs::copy_config_dependency(
            config_path, config_text, "relative_config_imucam",
            std::filesystem::path(output) / "resolved_imucam.yaml", &error)) {
      throw std::runtime_error(error);
    }
    const std::string resolved_logging_config =
        "%YAML:1.0\nheadless: " +
        std::string(viewer_enabled ? "false" : "true") +
        "\nviewer: " + std::string(viewer_enabled ? "true" : "false") +
        "\nviewer_history: " + std::to_string(viewer_history) +
        "\npose_print_rate_hz: " +
        std::to_string(pose_print_rate_hz) +
        "\ndiagnostics_rate_hz: " +
        std::to_string(diagnostics_rate_hz) + "\n";
    if (!ovrs::write_text(
            std::filesystem::path(output) /
                "resolved_logging_config.yaml",
            resolved_logging_config, &error)) {
      throw std::runtime_error(error);
    }
    ovrs::ImuSynchronizer synchronizer(stream_config.imu_queue_size);
    std::mutex sync_mutex;
    std::atomic<bool> initialized_flag{false};
    std::atomic<double> latest_latency_ms{0.0};
    auto last_print = std::chrono::steady_clock::now();
    ovrs::MeasurementDispatcher dispatcher(
        stream_config.imu_queue_size, stream_config.stereo_queue_size,
        [&](const ovrs::ImuSample &sample) { estimator.feed_imu(sample); },
        [&](const ovrs::StereoFrame &frame) {
          const auto begin = std::chrono::steady_clock::now();
          estimator.feed_stereo(frame);
          initialized_flag.store(estimator.initialized());
          const double latency =
              std::chrono::duration<double, std::milli>(
                  std::chrono::steady_clock::now() - begin)
                  .count();
          latest_latency_ms.store(latency);
          if (auto state = estimator.latest_state(latency)) {
            std::string write_error;
            if (!writer.write_state(*state, &write_error)) {
              throw std::runtime_error(write_error);
            }
            if (viewer) {
              viewer->publish_state(*state);
            }
            const auto now = std::chrono::steady_clock::now();
            if (now - last_print >= pose_print_period) {
              std::cout << "t=" << state->timestamp << " p=["
                        << state->position_world_m.x << ", "
                        << state->position_world_m.y << ", "
                        << state->position_world_m.z << "] initialized\n";
              last_print = now;
            }
          }
        });
    dispatcher.start();
    ovrs::RealSenseSource source(stream_config);
    const auto motion_callback = [&](const ovrs::TimedVec3 &sample,
                                     bool gyro) {
      std::lock_guard<std::mutex> lock(sync_mutex);
      if (gyro) {
        synchronizer.add_gyroscope(sample);
      } else {
        synchronizer.add_accelerometer(sample);
      }
      while (auto ready = synchronizer.take_ready()) {
        dispatcher.push_imu(*ready);
      }
    };
    if (!source.start({[&](ovrs::StereoFrame frame) {
                         if (viewer) {
                           viewer->publish_stereo(frame);
                         }
                         dispatcher.push_stereo(std::move(frame));
                       },
                       [&](ovrs::TimedVec3 sample) {
                         motion_callback(sample, true);
                       },
                       [&](ovrs::TimedVec3 sample) {
                         motion_callback(sample, false);
                       }},
                      &error)) {
      std::cerr << error << '\n';
      dispatcher.stop();
      return 2;
    }
    if (!ovrs::write_text(
            std::filesystem::path(output) / "resolved_stream_config.yaml",
            ovrs::stream_config_yaml(stream_config),
            &error)) {
      throw std::runtime_error(error);
    }
    const std::string actual_serial =
        ovrs::simple_yaml_scalar(source.device_report_yaml(), "serial");
    if (!calibrated_serial.empty() && actual_serial != calibrated_serial) {
      source.stop();
      dispatcher.stop();
      std::cerr << "Connected serial " << actual_serial
                << " does not match calibrated_serial " << calibrated_serial
                << ". Refusing to run.\n";
      return 2;
    }
    if (!ovrs::validate_runtime_imu_rate(
            config_path, config_text, source.device_report_yaml(), &error)) {
      throw std::runtime_error(error);
    }
    if (!ovrs::validate_runtime_motion_correction(
            config_path, config_text, source.device_report_yaml(),
            ovrs::stream_config_yaml(stream_config), &error)) {
      throw std::runtime_error(error);
    }
    const auto capture_started = std::chrono::steady_clock::now();
    const std::string initial_device_report = source.device_report_yaml();
    if (!ovrs::write_text(
            std::filesystem::path(output) / "device_report.yaml",
            initial_device_report, &error) ||
        !ovrs::write_text(
            std::filesystem::path(output) / "run_metadata.yaml",
            "%YAML:1.0\nmode: live\ncreated_utc: \"" +
                ovrs::utc_timestamp() +
            "\"\nproject_version: \"" + ovrs::project_version +
            "\"\ncalibration_state: \"" +
            calibration_state +
            "\"\nunverified_calibration_override: " +
            std::string(unverified_override_active ? "true" : "false") +
            "\nbuild_type: \"" + ovrs::build_type +
            "\"\ncompiler: \"" + ovrs::compiler_version +
            "\"\nopenvins_tag: \"" + ovrs::openvins_tag +
            "\"\nopenvins_commit: \"" + ovrs::openvins_commit +
            "\"\nceres_version: \"" + ovrs::ceres_version +
            "\"\nopencv_version: \"" + ovrs::opencv_version +
            "\"\nlibrealsense_version: \"" + ovrs::realsense_version +
            "\"\ninfrared_profile: \"" +
            ovrs::simple_yaml_scalar(initial_device_report,
                                     "infrared_profile") +
            "\"\ngyro_rate_hz: " +
            ovrs::simple_yaml_scalar(initial_device_report, "gyro_rate_hz") +
            "\naccelerometer_rate_hz: " +
            ovrs::simple_yaml_scalar(initial_device_report,
                                     "accelerometer_rate_hz") +
            "\n",
            &error)) {
      throw std::runtime_error(error);
    }
    std::cout << "Waiting for OpenVINS initialization. Press Ctrl+C to stop.\n";
    auto last_diagnostics = std::chrono::steady_clock::now();
    std::string viewer_failure;
    while (!ovrs::stop_requested() && source.failure().empty() &&
           !source.disconnected() &&
           dispatcher.failure().empty()) {
      if (viewer) {
        std::string poll_error;
        if (!viewer->poll(&poll_error)) {
          if (!poll_error.empty()) {
            viewer_failure = poll_error;
          }
          ovrs::request_stop();
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }
      const auto now = std::chrono::steady_clock::now();
      if (now - last_diagnostics >= diagnostics_period) {
        const auto source_snapshot = source.stats();
        ovrs::ImuSynchronizer::Stats sync_snapshot;
        {
          std::lock_guard<std::mutex> lock(sync_mutex);
          sync_snapshot = synchronizer.stats();
        }
        const auto dispatcher_snapshot = dispatcher.stats();
        const double elapsed =
            std::chrono::duration<double>(now - capture_started).count();
        ovrs::DiagnosticsSnapshot diagnostics;
        diagnostics.timestamp = elapsed;
        diagnostics.received_camera_frames =
            source_snapshot.received_framesets * 2;
        diagnostics.valid_stereo_pairs =
            source_snapshot.valid_stereo_pairs;
        diagnostics.received_gyro_samples = source_snapshot.received_gyro;
        diagnostics.received_accel_samples = source_snapshot.received_accel;
        diagnostics.synchronized_imu_samples = sync_snapshot.generated;
        diagnostics.rejected_timestamps =
            source_snapshot.rejected_timestamps +
            dispatcher_snapshot.rejected_nonmonotonic +
            sync_snapshot.duplicate_timestamps +
            sync_snapshot.regressing_timestamps +
            sync_snapshot.invalid_values;
        diagnostics.dropped_frames =
            source_snapshot.malformed_frames +
            source_snapshot.dropped_camera_frames +
            dispatcher_snapshot.dropped_stereo;
        diagnostics.imu_queue_depth = dispatcher.imu_queue_depth();
        diagnostics.camera_queue_depth = dispatcher.stereo_queue_depth();
        if (elapsed > 0.0) {
          diagnostics.camera_rate_hz =
              source_snapshot.valid_stereo_pairs / elapsed;
          diagnostics.imu_rate_hz = sync_snapshot.generated / elapsed;
          diagnostics.estimator_rate_hz =
              dispatcher_snapshot.dispatched_stereo / elapsed;
        }
        diagnostics.processing_latency_ms = latest_latency_ms.load();
        diagnostics.initialized = initialized_flag.load();
        if (!writer.write_diagnostics(diagnostics, &error)) {
          throw std::runtime_error(error);
        }
        last_diagnostics = now;
      }
    }
    source.stop();
    if (viewer) {
      viewer->close();
    }
    {
      std::lock_guard<std::mutex> lock(sync_mutex);
      synchronizer.shutdown();
    }
    dispatcher.stop();
    const auto source_stats = source.stats();
    const auto sync_stats = synchronizer.stats();
    const auto dispatch_stats = dispatcher.stats();
    const double capture_duration_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      capture_started)
            .count();
    ovrs::DiagnosticsSnapshot final_diagnostics;
    final_diagnostics.received_camera_frames =
        source_stats.received_framesets * 2;
    final_diagnostics.valid_stereo_pairs = source_stats.valid_stereo_pairs;
    final_diagnostics.received_gyro_samples = source_stats.received_gyro;
    final_diagnostics.received_accel_samples = source_stats.received_accel;
    final_diagnostics.synchronized_imu_samples = sync_stats.generated;
    final_diagnostics.rejected_timestamps =
        source_stats.rejected_timestamps +
        dispatch_stats.rejected_nonmonotonic +
        sync_stats.duplicate_timestamps +
        sync_stats.regressing_timestamps + sync_stats.invalid_values;
    final_diagnostics.dropped_frames =
        source_stats.malformed_frames + source_stats.dropped_camera_frames +
        dispatch_stats.dropped_stereo;
    if (capture_duration_s > 0.0) {
      final_diagnostics.camera_rate_hz =
          static_cast<double>(source_stats.valid_stereo_pairs) /
          capture_duration_s;
      final_diagnostics.imu_rate_hz =
          static_cast<double>(sync_stats.generated) / capture_duration_s;
      final_diagnostics.estimator_rate_hz =
          static_cast<double>(dispatch_stats.dispatched_stereo) /
          capture_duration_s;
    }
    final_diagnostics.initialized = estimator.initialized();
    final_diagnostics.processing_latency_ms = latest_latency_ms.load();
    if (!writer.write_diagnostics(final_diagnostics, &error) ||
        !ovrs::write_text(
            std::filesystem::path(output) / "device_report.yaml",
            source.device_report_yaml(), &error)) {
      throw std::runtime_error(error);
    }
    if (!writer.log("initialization_time_s=" +
                        std::to_string(estimator.initialization_time()),
                    &error) ||
        !writer.log("dropped_imu=" +
                        std::to_string(dispatch_stats.dropped_imu),
                    &error) ||
        !writer.log("dropped_stereo=" +
                        std::to_string(dispatch_stats.dropped_stereo),
                    &error) ||
        !writer.log("maximum_interpolation_delay_s=" +
                        std::to_string(
                            sync_stats.maximum_interpolation_delay_s),
                    &error)) {
      throw std::runtime_error(error);
    }
    std::cout << "Final rates: camera=" << final_diagnostics.camera_rate_hz
              << " Hz, synchronized IMU=" << final_diagnostics.imu_rate_hz
              << " Hz, estimator=" << final_diagnostics.estimator_rate_hz
              << " Hz; dropped stereo=" << dispatch_stats.dropped_stereo
              << ", dropped IMU=" << dispatch_stats.dropped_imu << '\n';
    std::string runtime_failure;
    if (!viewer_failure.empty()) {
      runtime_failure = viewer_failure;
    } else if (!source.failure().empty()) {
      runtime_failure = source.failure();
    } else if (source.disconnected()) {
      runtime_failure = "selected RealSense device disconnected";
    } else if (source_stats.malformed_frames != 0 ||
               source_stats.dropped_camera_frames != 0 ||
               source_stats.rejected_timestamps != 0 ||
               source_stats.callback_errors != 0) {
      runtime_failure = "RealSense capture integrity counters are nonzero";
    } else if (!dispatcher.failure().empty()) {
      runtime_failure = dispatcher.failure();
    } else if (dispatch_stats.dropped_imu != 0 ||
               dispatch_stats.dropped_stereo != 0 ||
               dispatch_stats.rejected_nonmonotonic != 0) {
      runtime_failure = "ordered dispatcher integrity counters are nonzero";
    } else if (sync_stats.duplicate_timestamps != 0 ||
               sync_stats.regressing_timestamps != 0 ||
               sync_stats.invalid_values != 0 ||
               sync_stats.dropped_capacity != 0) {
      runtime_failure = "IMU synchronization integrity counters are nonzero";
    } else if (!estimator.initialized()) {
      runtime_failure = "OpenVINS stopped before initialization";
    }
    const bool runtime_failed = !runtime_failure.empty();
    if (!writer.log(runtime_failed ? "ovrs_live stopped after runtime failure"
                                   : "ovrs_live stopped cleanly",
                    &error)) {
      throw std::runtime_error(error);
    }
    if (runtime_failed) {
      if (!writer.close(&error)) {
        throw std::runtime_error(error);
      }
      std::cerr << runtime_failure << '\n';
      return 5;
    }
    if (!writer.finalize(&error)) {
      throw std::runtime_error(error);
    }
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 4;
  }
#endif
}
