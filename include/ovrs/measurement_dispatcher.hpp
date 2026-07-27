#pragma once

#include "ovrs/types.hpp"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace ovrs {

class MeasurementDispatcher {
public:
  using ImuHandler = std::function<void(const ImuSample &)>;
  using CameraHandler = std::function<void(const StereoFrame &)>;

  struct Stats {
    std::uint64_t dispatched_imu = 0;
    std::uint64_t dispatched_stereo = 0;
    std::uint64_t rejected_nonmonotonic = 0;
    std::uint64_t dropped_imu = 0;
    std::uint64_t dropped_stereo = 0;
    std::uint64_t stereo_without_imu_coverage = 0;
  };

  MeasurementDispatcher(std::size_t imu_capacity, std::size_t stereo_capacity,
                        ImuHandler imu_handler,
                        CameraHandler camera_handler);
  ~MeasurementDispatcher();

  MeasurementDispatcher(const MeasurementDispatcher &) = delete;
  MeasurementDispatcher &operator=(const MeasurementDispatcher &) = delete;

  void start();
  bool push_imu(ImuSample sample);
  bool push_stereo(StereoFrame frame);
  void stop();
  bool running() const;
  std::string failure() const;
  Stats stats() const;
  std::size_t imu_queue_depth() const;
  std::size_t stereo_queue_depth() const;

private:
  void run();
  bool dispatch_ready_locked(std::unique_lock<std::mutex> &lock);

  const std::size_t imu_capacity_;
  const std::size_t stereo_capacity_;
  ImuHandler imu_handler_;
  CameraHandler camera_handler_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<ImuSample> imu_;
  std::deque<StereoFrame> stereo_;
  std::optional<double> last_imu_received_;
  std::optional<double> last_stereo_received_;
  std::optional<double> first_dispatched_imu_timestamp_;
  std::optional<double> last_dispatched_timestamp_;
  Stats stats_;
  std::thread worker_;
  std::string failure_;
  bool running_ = false;
  bool stopping_ = false;
};

} // namespace ovrs
