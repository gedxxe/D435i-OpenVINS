#include "ovrs/measurement_dispatcher.hpp"

#include <cmath>
#include <exception>

namespace ovrs {

MeasurementDispatcher::MeasurementDispatcher(
    std::size_t imu_capacity, std::size_t stereo_capacity,
    ImuHandler imu_handler, CameraHandler camera_handler)
    : imu_capacity_(imu_capacity), stereo_capacity_(stereo_capacity),
      imu_handler_(std::move(imu_handler)),
      camera_handler_(std::move(camera_handler)) {}

MeasurementDispatcher::~MeasurementDispatcher() { stop(); }

void MeasurementDispatcher::start() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_) {
    return;
  }
  imu_.clear();
  stereo_.clear();
  last_imu_received_.reset();
  last_stereo_received_.reset();
  first_dispatched_imu_timestamp_.reset();
  last_dispatched_timestamp_.reset();
  stats_ = {};
  stopping_ = false;
  failure_.clear();
  running_ = true;
  worker_ = std::thread(&MeasurementDispatcher::run, this);
}

bool MeasurementDispatcher::push_imu(ImuSample sample) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_ || stopping_ || !std::isfinite(sample.timestamp)) {
    return false;
  }
  if (last_imu_received_ && sample.timestamp <= *last_imu_received_) {
    ++stats_.rejected_nonmonotonic;
    return false;
  }
  last_imu_received_ = sample.timestamp;
  if (imu_capacity_ == 0) {
    ++stats_.dropped_imu;
    return false;
  }
  if (imu_.size() == imu_capacity_) {
    imu_.pop_front();
    ++stats_.dropped_imu;
  }
  imu_.push_back(std::move(sample));
  cv_.notify_one();
  return true;
}

bool MeasurementDispatcher::push_stereo(StereoFrame frame) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_ || stopping_ || !std::isfinite(frame.timestamp)) {
    return false;
  }
  if (last_stereo_received_ &&
      frame.timestamp <= *last_stereo_received_) {
    ++stats_.rejected_nonmonotonic;
    return false;
  }
  last_stereo_received_ = frame.timestamp;
  if (stereo_capacity_ == 0) {
    ++stats_.dropped_stereo;
    return false;
  }
  if (stereo_.size() == stereo_capacity_) {
    stereo_.pop_front();
    ++stats_.dropped_stereo;
  }
  stereo_.push_back(std::move(frame));
  cv_.notify_one();
  return true;
}

bool MeasurementDispatcher::dispatch_ready_locked(
    std::unique_lock<std::mutex> &lock) {
  if (stereo_.empty() || !last_imu_received_ ||
      *last_imu_received_ < stereo_.front().timestamp) {
    return false;
  }
  const double image_time = stereo_.front().timestamp;
  bool has_imu_at_or_before =
      first_dispatched_imu_timestamp_ &&
      *first_dispatched_imu_timestamp_ <= image_time;
  while (!imu_.empty() && imu_.front().timestamp <= image_time) {
    ImuSample sample = std::move(imu_.front());
    imu_.pop_front();
    if (!first_dispatched_imu_timestamp_) {
      first_dispatched_imu_timestamp_ = sample.timestamp;
    }
    last_dispatched_timestamp_ = sample.timestamp;
    lock.unlock();
    imu_handler_(sample);
    lock.lock();
    ++stats_.dispatched_imu;
    has_imu_at_or_before = true;
  }
  // OpenVINS propagation interpolates at the image time, so it needs the
  // first IMU sample at or after that image as well as the sample before it.
  if ((!last_dispatched_timestamp_ ||
       *last_dispatched_timestamp_ < image_time) &&
      !imu_.empty()) {
    ImuSample sample = std::move(imu_.front());
    imu_.pop_front();
    if (!first_dispatched_imu_timestamp_) {
      first_dispatched_imu_timestamp_ = sample.timestamp;
    }
    last_dispatched_timestamp_ = sample.timestamp;
    lock.unlock();
    imu_handler_(sample);
    lock.lock();
    ++stats_.dispatched_imu;
  }
  if (!has_imu_at_or_before) {
    ++stats_.stereo_before_imu_start;
    stereo_.pop_front();
    return true;
  }
  if (!last_dispatched_timestamp_ ||
      *last_dispatched_timestamp_ < image_time) {
    ++stats_.stereo_without_imu_coverage;
    stereo_.pop_front();
    return true;
  }
  StereoFrame frame = std::move(stereo_.front());
  stereo_.pop_front();
  lock.unlock();
  camera_handler_(frame);
  lock.lock();
  ++stats_.dispatched_stereo;
  return true;
}

void MeasurementDispatcher::run() {
  try {
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
      cv_.wait(lock, [this] {
        return stopping_ ||
               (!stereo_.empty() && last_imu_received_ &&
                *last_imu_received_ >= stereo_.front().timestamp);
      });
      while (dispatch_ready_locked(lock)) {
      }
      if (stopping_) {
        while (!imu_.empty()) {
          ImuSample sample = std::move(imu_.front());
          imu_.pop_front();
          if (!first_dispatched_imu_timestamp_) {
            first_dispatched_imu_timestamp_ = sample.timestamp;
          }
          lock.unlock();
          imu_handler_(sample);
          lock.lock();
          ++stats_.dispatched_imu;
        }
        stats_.stereo_discarded_on_shutdown += stereo_.size();
        stereo_.clear();
        running_ = false;
        return;
      }
    }
  } catch (const std::exception &e) {
    std::lock_guard<std::mutex> lock(mutex_);
    failure_ = e.what();
    running_ = false;
    stopping_ = true;
  } catch (...) {
    std::lock_guard<std::mutex> lock(mutex_);
    failure_ = "unknown dispatcher exception";
    running_ = false;
    stopping_ = true;
  }
}

void MeasurementDispatcher::stop() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_ && !worker_.joinable()) {
      return;
    }
    stopping_ = true;
    cv_.notify_all();
  }
  if (worker_.joinable()) {
    worker_.join();
  }
}

bool MeasurementDispatcher::running() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return running_;
}

std::string MeasurementDispatcher::failure() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return failure_;
}

MeasurementDispatcher::Stats MeasurementDispatcher::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

std::size_t MeasurementDispatcher::imu_queue_depth() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return imu_.size();
}

std::size_t MeasurementDispatcher::stereo_queue_depth() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stereo_.size();
}

} // namespace ovrs
