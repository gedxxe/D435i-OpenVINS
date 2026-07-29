#include "ovrs/app_support.hpp"
#include "ovrs/bounded_queue.hpp"
#include "ovrs/capture_mode.hpp"
#include "ovrs/config.hpp"
#include "ovrs/imu_synchronizer.hpp"
#include "ovrs/realsense_source.hpp"
#include "ovrs/stereo_capture_preview.hpp"
#include "ovrs/yaml_utils.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <sstream>
#include <thread>

#ifdef OVRS_HAS_REALSENSE
#include <opencv2/imgcodecs.hpp>
#endif

namespace {
void help() {
  std::cout << "Usage: ovrs_record [--serial SERIAL] [--duration SECONDS]\n"
               "                   --output DATASET_DIR\n"
               "                   [--capture-mode MODE]\n"
               "                   [--confirm-stationary]\n"
               "                   [--calibration-target APRILGRID.yaml]\n"
               "                   [--preview]\n"
               "                   [--stream-config STREAMS.yaml]\n"
               "                   [--width PIXELS --height PIXELS "
               "--camera-fps HZ]\n"
               "                   [--gyro-fps HZ --accel-fps HZ]\n"
               "                   [--gyro-sensitivity 0..4]\n"
               "                   [--gyro-scale-factor FACTOR]\n"
               "                   [--emitter on|off] "
               "[--auto-exposure on|off]\n"
               "                   [--motion-correction on|off]\n"
               "                   [--global-time on|off]\n"
               "                   [--imu-queue COUNT --stereo-queue COUNT]\n"
               "                   [--stereo-tolerance-ms MILLISECONDS]\n"
               "                   [--version]\n"
               "MODE is vio (default), imu-allan, stereo-calibration, or\n"
               "imu-camera-calibration. Calibration captures are deliberately\n"
               "not replay-compatible and never become KALIBR_VERIFIED.\n";
}
} // namespace

int main(int argc, char **argv) {
  if (ovrs::has_flag(argc, argv, "--version")) {
    std::cout << ovrs::version_summary("ovrs_record");
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
                       {"--duration", "--output", "--stream-config",
                        "--capture-mode", "--calibration-target"});
  if (!ovrs::validate_cli_arguments(argc, argv, value_options,
                                    {"--confirm-stationary", "--preview"},
                                    &argument_error)) {
    std::cerr << argument_error << '\n';
    return 2;
  }
#ifndef OVRS_HAS_REALSENSE
  std::cerr << "ovrs_record was built without librealsense2/OpenCV. Use "
               "scripts/build_ubuntu.sh.\n";
  return 3;
#else
  const auto output = ovrs::value_after(argc, argv, "--output");
  if (output.empty()) {
    std::cerr << "--output is required\n";
    return 2;
  }
  double duration = 0.0;
  try {
    duration =
        ovrs::parse_double_strict(
            ovrs::value_after(argc, argv, "--duration", "60"), "--duration");
  } catch (const std::exception &) {
    std::cerr << "--duration must be a positive number\n";
    return 2;
  }
  if (!std::isfinite(duration) || duration <= 0.0) {
    std::cerr << "--duration must be a positive finite number\n";
    return 2;
  }
  const auto plan = ovrs::capture_plan(
      ovrs::value_after(argc, argv, "--capture-mode", "vio"));
  if (!plan) {
    std::cerr << "--capture-mode must be vio, imu-allan, "
                 "stereo-calibration, or imu-camera-calibration\n";
    return 2;
  }
  const bool stationary_confirmed =
      ovrs::has_flag(argc, argv, "--confirm-stationary");
  const bool preview_requested = ovrs::has_flag(argc, argv, "--preview");
  if (plan->requires_stationary_sensor && !stationary_confirmed) {
    std::cerr << "imu-allan requires --confirm-stationary; rigidly secure the "
                 "camera before recording\n";
    return 2;
  }
  if (!plan->requires_stationary_sensor && stationary_confirmed) {
    std::cerr << "--confirm-stationary is valid only with "
                 "--capture-mode imu-allan\n";
    return 2;
  }
  const auto calibration_target =
      ovrs::value_after(argc, argv, "--calibration-target");
  if (plan->requires_calibration_target && calibration_target.empty()) {
    std::cerr << plan->name
              << " requires --calibration-target created from measured "
                 "AprilGrid dimensions\n";
    return 2;
  }
  if (!plan->requires_calibration_target && !calibration_target.empty()) {
    std::cerr << "--calibration-target is valid only for stereo-calibration "
                 "or imu-camera-calibration\n";
    return 2;
  }
  if (preview_requested && !plan->requires_calibration_target) {
    std::cerr << "--preview is valid only with stereo-calibration or "
                 "imu-camera-calibration\n";
    return 2;
  }
  const std::filesystem::path root(output);
  ovrs::install_signal_handlers();
  try {
    if (std::filesystem::exists(root) && !std::filesystem::is_empty(root)) {
      std::cerr << "Output directory already exists and is not empty: "
                << root << '\n';
      return 2;
    }
    std::string calibration_target_yaml;
    if (plan->requires_calibration_target) {
      std::ifstream target_input(calibration_target);
      if (!target_input) {
        throw std::runtime_error(
            "cannot open calibration target: " + calibration_target);
      }
      calibration_target_yaml.assign(
          std::istreambuf_iterator<char>(target_input),
          std::istreambuf_iterator<char>());
      if (calibration_target_yaml.empty()) {
        throw std::runtime_error("calibration target file is empty");
      }
    }
    ovrs::StreamConfig config;
    config.imu_queue_size = 4096;
    config.stereo_queue_size = 64;
    const auto stream_config =
        ovrs::value_after(argc, argv, "--stream-config");
    if (!stream_config.empty()) {
      std::string load_error;
      if (!ovrs::load_stream_config(stream_config, &config, &load_error)) {
        throw std::runtime_error(load_error);
      }
    }
    std::string cli_error;
    if (!ovrs::apply_stream_config_cli(argc, argv, &config, &cli_error)) {
      throw std::runtime_error(cli_error);
    }
    std::string error;
    std::unique_ptr<ovrs::StereoCapturePreview> preview;
    if (preview_requested) {
      preview = std::make_unique<ovrs::StereoCapturePreview>();
      if (!preview->open(&error)) {
        throw std::runtime_error(error);
      }
      ovrs::BoundedQueue<ovrs::StereoFrame> preview_queue(2);
      ovrs::RealSenseSource preview_source(config, {true, false});
      ovrs::RealSenseSource::Callbacks preview_callbacks;
      preview_callbacks.stereo = [&](ovrs::StereoFrame frame) {
        preview_queue.push(std::move(frame));
      };
      if (!preview_source.start(preview_callbacks, &error)) {
        throw std::runtime_error(error);
      }
      config.serial = ovrs::simple_yaml_scalar(
          preview_source.device_report_yaml(), "serial");
      std::cout
          << "Preview opened for serial " << config.serial << ".\n"
          << "Verify the whole AprilGrid is sharp and visible in IR1 and IR2; "
             "press Space to start a clean capture, or q/Esc to cancel.\n";
      bool have_frame = false;
      bool start_capture = false;
      while (!ovrs::stop_requested() && preview_source.failure().empty() &&
             !preview_source.disconnected()) {
        std::optional<ovrs::StereoFrame> latest;
        while (auto frame = preview_queue.try_pop()) {
          latest = std::move(*frame);
        }
        if (latest) {
          have_frame = true;
          if (!preview->show(
                  *latest,
                  "PREVIEW ONLY - frames are not saved",
                  "Space: start clean capture   q/Esc: cancel",
                  &error)) {
            preview_source.stop();
            throw std::runtime_error(error);
          }
        }
        const auto action = preview->poll(have_frame, &error);
        if (!error.empty()) {
          preview_source.stop();
          throw std::runtime_error(error);
        }
        if (action == ovrs::CapturePreviewAction::start) {
          start_capture = true;
          break;
        }
        if (action == ovrs::CapturePreviewAction::abort) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      const auto preview_failure = preview_source.failure();
      const bool preview_disconnected = preview_source.disconnected();
      preview_source.stop();
      if (!preview_failure.empty()) {
        throw std::runtime_error(preview_failure);
      }
      if (preview_disconnected) {
        throw std::runtime_error(
            "selected RealSense device disconnected during preview");
      }
      if (!start_capture) {
        std::cerr << "Calibration preview cancelled; no dataset was "
                     "initialized.\n";
        return ovrs::stop_requested() ? 130 : 2;
      }
    }
    ovrs::BoundedQueue<ovrs::StereoFrame> stereo_queue(
        config.stereo_queue_size);
    ovrs::BoundedQueue<ovrs::TimedVec3> gyro_queue(config.imu_queue_size);
    ovrs::BoundedQueue<ovrs::TimedVec3> accel_queue(config.imu_queue_size);
    ovrs::RealSenseSource source(
        config, {plan->enable_stereo, plan->enable_motion});
    ovrs::RealSenseSource::Callbacks callbacks;
    if (plan->enable_stereo) {
      callbacks.stereo = [&](ovrs::StereoFrame frame) {
        stereo_queue.push(std::move(frame));
      };
    }
    if (plan->enable_motion) {
      callbacks.gyro = [&](ovrs::TimedVec3 sample) {
        gyro_queue.push(sample);
      };
      callbacks.accel = [&](ovrs::TimedVec3 sample) {
        accel_queue.push(sample);
      };
    }
    if (!source.start(callbacks, &error)) {
      throw std::runtime_error(error);
    }
    const auto started = std::chrono::steady_clock::now();
    config.serial =
        ovrs::simple_yaml_scalar(source.device_report_yaml(), "serial");
    if (plan->enable_stereo) {
      std::filesystem::create_directories(root / "cam0" / "data");
      std::filesystem::create_directories(root / "cam1" / "data");
    }
    if (plan->enable_motion) {
      std::filesystem::create_directories(root / "imu");
    }
    std::ofstream incomplete(root / "INCOMPLETE");
    incomplete << "Recording was not closed cleanly yet.\n";
    incomplete.close();
    std::ofstream cam0_csv;
    std::ofstream cam1_csv;
    std::ofstream gyro_csv;
    std::ofstream accel_csv;
    std::ofstream synced_csv;
    if (plan->enable_stereo) {
      cam0_csv.open(root / "cam0" / "data.csv");
      cam1_csv.open(root / "cam1" / "data.csv");
    }
    if (plan->enable_motion) {
      gyro_csv.open(root / "imu" / "gyro.csv");
      accel_csv.open(root / "imu" / "accelerometer.csv");
    }
    if (plan->write_synchronized_imu) {
      synced_csv.open(root / "imu" / "synchronized.csv");
    }
    if (!incomplete ||
        (plan->enable_stereo && (!cam0_csv || !cam1_csv)) ||
        (plan->enable_motion && (!gyro_csv || !accel_csv)) ||
        (plan->write_synchronized_imu && !synced_csv)) {
      throw std::runtime_error("cannot open one or more dataset output files");
    }
    if (plan->enable_stereo) {
      cam0_csv << "timestamp_s,raw_timestamp_ms,frameset_number,file\n";
      cam1_csv << "timestamp_s,raw_timestamp_ms,frameset_number,file\n";
    }
    if (plan->enable_motion) {
      gyro_csv
          << "timestamp_s,raw_timestamp_ms,wx_rad_s,wy_rad_s,wz_rad_s\n";
      accel_csv
          << "timestamp_s,raw_timestamp_ms,ax_m_s2,ay_m_s2,az_m_s2\n";
    }
    if (plan->write_synchronized_imu) {
      synced_csv << "timestamp_s,raw_gyro_timestamp_ms,wx_rad_s,wy_rad_s,"
                    "wz_rad_s,ax_m_s2,ay_m_s2,az_m_s2,"
                    "interpolation_delay_s\n";
    }
    const auto capture_created_utc = ovrs::utc_timestamp();
    const auto initial_metadata =
        "%YAML:1.0\nformat: \"" + plan->format +
        "\"\ncapture_mode: \"" + plan->name +
        "\"\npurpose: \"" + plan->purpose +
        "\"\ncreated_utc: \"" + capture_created_utc +
        "\"\ncomplete: false\nreplay_compatible: " +
        (plan->replay_compatible ? std::string("true")
                                 : std::string("false")) +
        "\nrequires_stationary_sensor: " +
        (plan->requires_stationary_sensor ? std::string("true")
                                          : std::string("false")) +
        "\noperator_confirmed_stationary: " +
        (stationary_confirmed ? std::string("true") : std::string("false")) +
        "\ncalibration_target_present: " +
        (plan->requires_calibration_target ? std::string("true")
                                           : std::string("false")) +
        "\ncalibration_state: \"BOOTSTRAP_UNVERIFIED\"\n";
    if (!ovrs::write_text(root / "device_report.yaml",
                          source.device_report_yaml(), &error) ||
        (plan->requires_calibration_target &&
         !ovrs::write_text(root / "calibration_target.yaml",
                           calibration_target_yaml, &error)) ||
        !ovrs::write_text(
            root / "dataset_metadata.yaml",
            initial_metadata,
            &error) ||
        !ovrs::write_text(
            root / "resolved_stream_config.yaml",
            ovrs::stream_config_yaml(config),
            &error)) {
      throw std::runtime_error(error);
    }
    ovrs::ImuSynchronizer synchronizer(config.imu_queue_size);
    std::optional<ovrs::StereoFrame> latest_preview_frame;
    const auto write_synchronized = [&](const ovrs::ImuSample &sample) {
      synced_csv << std::setprecision(17) << sample.timestamp << ','
                 << sample.raw_gyro_timestamp_ms << ','
                 << sample.angular_velocity_rad_s.x << ','
                 << sample.angular_velocity_rad_s.y << ','
                 << sample.angular_velocity_rad_s.z << ','
                 << sample.linear_acceleration_m_s2.x << ','
                 << sample.linear_acceleration_m_s2.y << ','
                 << sample.linear_acceleration_m_s2.z << ','
                 << sample.interpolation_delay_s << '\n';
    };
    const auto drain_queues = [&]() {
      bool did_work = false;
      while (auto accel = accel_queue.try_pop()) {
        did_work = true;
        const auto raw = accel->original_sensor_value_available
                             ? accel->original_sensor_value
                             : accel->value;
        accel_csv << std::setprecision(17) << accel->timestamp << ','
                  << accel->raw_timestamp_ms << ',' << raw.x << ',' << raw.y
                  << ',' << raw.z << '\n';
        synchronizer.add_accelerometer(*accel);
      }
      while (auto gyro = gyro_queue.try_pop()) {
        did_work = true;
        gyro_csv << std::setprecision(17) << gyro->timestamp << ','
                 << gyro->raw_timestamp_ms << ',' << gyro->value.x << ','
                 << gyro->value.y << ',' << gyro->value.z << '\n';
        synchronizer.add_gyroscope(*gyro);
      }
      while (auto synced = synchronizer.take_ready()) {
        did_work = true;
        write_synchronized(*synced);
      }
      while (auto stereo = stereo_queue.try_pop()) {
        did_work = true;
        if (preview) {
          latest_preview_frame = *stereo;
        }
        const auto filename =
            std::to_string(stereo->camera0.frameset_number) + ".png";
        const auto write_image = [&](const ovrs::ImageFrame &image,
                                     const std::filesystem::path &folder) {
          cv::Mat view(image.height, image.width, CV_8UC1,
                       image.pixels->data(), image.stride_bytes);
          const auto temporary = folder / ("tmp_" + filename);
          if (!cv::imwrite(temporary.string(), view)) {
            throw std::runtime_error("failed to write image " +
                                     temporary.string());
          }
          std::filesystem::rename(temporary, folder / filename);
        };
        write_image(stereo->camera0, root / "cam0" / "data");
        write_image(stereo->camera1, root / "cam1" / "data");
        cam0_csv << std::setprecision(17) << stereo->camera0.timestamp << ','
                 << stereo->camera0.raw_timestamp_ms << ','
                 << stereo->camera0.frameset_number << ',' << filename
                 << '\n';
        cam1_csv << std::setprecision(17) << stereo->camera1.timestamp << ','
                 << stereo->camera1.raw_timestamp_ms << ','
                 << stereo->camera1.frameset_number << ',' << filename
                 << '\n';
      }
      return did_work;
    };
    bool preview_aborted = false;
    while (!ovrs::stop_requested() && source.failure().empty() &&
           !source.disconnected() &&
           std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         started)
                   .count() < duration) {
      const bool did_work = drain_queues();
      if (preview && latest_preview_frame) {
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          started)
                .count();
        std::ostringstream status;
        status << "RECORDING " << std::fixed << std::setprecision(1)
               << elapsed << " / " << duration << " s";
        if (!preview->show(*latest_preview_frame, status.str(),
                           "Keep target visible and sharp   q/Esc: abort",
                           &error)) {
          throw std::runtime_error(error);
        }
        latest_preview_frame.reset();
      }
      if (preview) {
        const auto action = preview->poll(false, &error);
        if (!error.empty()) {
          throw std::runtime_error(error);
        }
        if (action == ovrs::CapturePreviewAction::abort) {
          preview_aborted = true;
          break;
        }
      }
      if (!did_work) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
    const bool interrupted = ovrs::stop_requested() || preview_aborted;
    source.stop();
    if (preview) {
      preview->close();
    }
    while (drain_queues()) {
    }
    synchronizer.shutdown();
    while (auto synced = synchronizer.take_ready()) {
      write_synchronized(*synced);
    }
    if (plan->enable_stereo) {
      cam0_csv.close();
      cam1_csv.close();
    }
    if (plan->enable_motion) {
      gyro_csv.close();
      accel_csv.close();
    }
    if (plan->write_synchronized_imu) {
      synced_csv.close();
    }
    if ((plan->enable_stereo && (!cam0_csv || !cam1_csv)) ||
        (plan->enable_motion && (!gyro_csv || !accel_csv)) ||
        (plan->write_synchronized_imu && !synced_csv)) {
      throw std::runtime_error("one or more dataset files failed to flush");
    }
    const auto source_stats = source.stats();
    const auto synchronizer_stats = synchronizer.stats();
    const bool queue_drops =
        stereo_queue.dropped() != 0 || gyro_queue.dropped() != 0 ||
        accel_queue.dropped() != 0;
    const bool synchronizer_integrity_failure =
        synchronizer_stats.duplicate_timestamps != 0 ||
        synchronizer_stats.regressing_timestamps != 0 ||
        synchronizer_stats.invalid_values != 0 ||
        synchronizer_stats.dropped_capacity != 0;
    const bool missing_required_stream =
        (plan->enable_stereo && source_stats.valid_stereo_pairs == 0) ||
        (plan->enable_motion &&
         (source_stats.received_gyro == 0 ||
          source_stats.received_accel == 0)) ||
        (plan->write_synchronized_imu &&
         synchronizer_stats.generated == 0);
    if (interrupted) {
      std::cerr << "Recording interrupted; partial capture remains marked "
                   "INCOMPLETE.\n";
      return 130;
    }
    if (!source.failure().empty() || source.disconnected() ||
        (plan->enable_stereo &&
         source_stats.dropped_camera_frames != 0) ||
        source_stats.malformed_frames != 0 ||
        source_stats.rejected_timestamps != 0 ||
        source_stats.callback_errors != 0 || queue_drops ||
        synchronizer_integrity_failure || missing_required_stream) {
      const std::string failure =
          !source.failure().empty()
              ? source.failure()
              : (source.disconnected()
                     ? "selected RealSense device disconnected"
                     : (missing_required_stream
                            ? "one or more required streams produced no data"
                            : "capture integrity counters are nonzero"));
      std::cerr << "Capture failure: " << failure
                << ". Dataset remains marked INCOMPLETE.\n";
      return 5;
    }
    if (!ovrs::write_text(root / "device_report.yaml",
                          source.device_report_yaml(), &error)) {
      throw std::runtime_error(error);
    }
    const double recording_duration_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      started)
            .count();
    const auto final_metadata =
        "%YAML:1.0\nformat: \"" + plan->format +
        "\"\ncapture_mode: \"" + plan->name +
        "\"\npurpose: \"" + plan->purpose +
        "\"\ncreated_utc: \"" + capture_created_utc +
        "\"\ncompleted_utc: \"" + ovrs::utc_timestamp() +
        "\"\ncomplete: true\nreplay_compatible: " +
        (plan->replay_compatible ? std::string("true")
                                 : std::string("false")) +
        "\nrequires_stationary_sensor: " +
        (plan->requires_stationary_sensor ? std::string("true")
                                          : std::string("false")) +
        "\noperator_confirmed_stationary: " +
        (stationary_confirmed ? std::string("true") : std::string("false")) +
        "\ncalibration_target_present: " +
        (plan->requires_calibration_target ? std::string("true")
                                           : std::string("false")) +
        "\ncalibration_state: \"BOOTSTRAP_UNVERIFIED\"\n"
        "calibrated_serial: \"" +
        config.serial + "\"\ninfrared_profile: \"" +
        ovrs::simple_yaml_scalar(source.device_report_yaml(),
                                 "infrared_profile") +
        "\"\ngyro_rate_hz: " +
        ovrs::simple_yaml_scalar(source.device_report_yaml(),
                                 "gyro_rate_hz") +
        "\ngyro_sensitivity_active: " +
        ovrs::simple_yaml_scalar(source.device_report_yaml(),
                                 "gyro_sensitivity_active") +
        "\ngyro_scale_factor_applied: " +
        ovrs::simple_yaml_scalar(source.device_report_yaml(),
                                 "gyro_scale_factor_applied") +
        "\naccelerometer_rate_hz: " +
        ovrs::simple_yaml_scalar(source.device_report_yaml(),
                                 "accelerometer_rate_hz") +
        "\nmotion_correction_active: " +
        ovrs::simple_yaml_scalar(source.device_report_yaml(),
                                 "motion_correction_active") +
        "\nglobal_time_active: " +
        ovrs::simple_yaml_scalar(source.device_report_yaml(),
                                 "global_time_active") +
        "\n";
    if (!ovrs::write_text(
            root / "recording_summary.yaml",
            "%YAML:1.0\nreceived_framesets: " +
                std::to_string(source_stats.received_framesets) +
                "\nvalid_stereo_pairs: " +
                std::to_string(source_stats.valid_stereo_pairs) +
                "\ndropped_camera_frames: " +
                std::to_string(source_stats.dropped_camera_frames) +
                "\nmalformed_frames: " +
                std::to_string(source_stats.malformed_frames) +
                "\nrejected_timestamps: " +
                std::to_string(source_stats.rejected_timestamps) +
                "\ncallback_errors: " +
                std::to_string(source_stats.callback_errors) +
                "\nreceived_gyro: " +
                std::to_string(source_stats.received_gyro) +
                "\nreceived_accelerometer: " +
                std::to_string(source_stats.received_accel) +
                "\nstereo_queue_drops: " +
                std::to_string(stereo_queue.dropped()) +
                "\ngyro_queue_drops: " +
                std::to_string(gyro_queue.dropped()) +
                "\naccelerometer_queue_drops: " +
                std::to_string(accel_queue.dropped()) +
                "\nsynchronized_imu: " +
                std::to_string(synchronizer_stats.generated) +
                "\nimu_duplicate_timestamps: " +
                std::to_string(
                    synchronizer_stats.duplicate_timestamps) +
                "\nimu_regressing_timestamps: " +
                std::to_string(
                    synchronizer_stats.regressing_timestamps) +
                "\nimu_invalid_values: " +
                std::to_string(synchronizer_stats.invalid_values) +
                "\nimu_missing_interpolation_brackets: " +
                std::to_string(synchronizer_stats.missing_brackets) +
                "\nimu_synchronizer_capacity_drops: " +
                std::to_string(synchronizer_stats.dropped_capacity) +
                "\nmaximum_interpolation_delay_s: " +
                std::to_string(
                    synchronizer_stats.maximum_interpolation_delay_s) +
                "\nrecording_duration_s: " +
                std::to_string(recording_duration_s) +
                "\neffective_camera_rate_hz: " +
                std::to_string(source_stats.valid_stereo_pairs /
                               recording_duration_s) +
                "\neffective_synchronized_imu_rate_hz: " +
                std::to_string(synchronizer_stats.generated /
                               recording_duration_s) +
                "\n",
            &error) ||
        !ovrs::write_text(
            root / "dataset_metadata.yaml",
            final_metadata,
            &error)) {
      throw std::runtime_error(error);
    }
    std::error_code remove_error;
    if (!std::filesystem::remove(root / "INCOMPLETE", remove_error) ||
        remove_error) {
      throw std::runtime_error("cannot remove INCOMPLETE marker: " +
                               remove_error.message());
    }
    std::cout << "Recording complete: " << root << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::error_code marker_error;
    const bool has_incomplete =
        std::filesystem::exists(root / "INCOMPLETE", marker_error) &&
        !marker_error;
    std::cerr << e.what()
              << (has_incomplete ? "\nDataset remains marked INCOMPLETE.\n"
                                 : "\nNo dataset was initialized.\n");
    return 4;
  }
#endif
}
