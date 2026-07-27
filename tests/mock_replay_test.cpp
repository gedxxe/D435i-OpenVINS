#include "test_harness.hpp"

#include "ovrs/measurement_dispatcher.hpp"
#include "ovrs/trajectory.hpp"

#include <chrono>
#include <atomic>
#include <filesystem>

TEST_CASE("synthetic replay reaches the shared estimator interface") {
  std::atomic<std::size_t> imu_count{0};
  std::atomic<std::size_t> camera_count{0};
  ovrs::MeasurementDispatcher dispatcher(
      32, 8, [&](const ovrs::ImuSample &) { ++imu_count; },
      [&](const ovrs::StereoFrame &) { ++camera_count; });
  dispatcher.start();
  for (int i = 0; i < 10; ++i) {
    ovrs::ImuSample sample;
    sample.timestamp = 0.01 * i;
    REQUIRE(dispatcher.push_imu(sample));
    if (i == 5) {
      ovrs::StereoFrame frame;
      frame.timestamp = 0.045;
      REQUIRE(dispatcher.push_stereo(frame));
    }
  }
  dispatcher.stop();
  REQUIRE(imu_count.load() == 10);
  REQUIRE(camera_count.load() == 1);

  const auto unique_suffix = std::to_string(
      std::chrono::high_resolution_clock::now().time_since_epoch().count());
  const auto path = std::filesystem::temp_directory_path() /
                    ("ovrs_mock_replay_" + unique_suffix);
  std::error_code ec;
  std::filesystem::remove_all(path, ec);
  ovrs::RunWriter writer;
  std::string error;
  REQUIRE(writer.open(path, &error));
  ovrs::EstimatorState state;
  state.timestamp = 0.045;
  state.initialized = true;
  state.healthy = true;
  REQUIRE(writer.write_state(state, &error));
  REQUIRE(std::filesystem::exists(path / "INCOMPLETE"));
  REQUIRE(writer.finalize(&error));
  REQUIRE(!std::filesystem::exists(path / "INCOMPLETE"));
  REQUIRE(std::filesystem::exists(path / "trajectory_tum.txt"));
  REQUIRE(std::filesystem::exists(path / "state.csv"));
  REQUIRE(std::filesystem::exists(path / "diagnostics.csv"));
  REQUIRE(std::filesystem::exists(path / "application.log"));
  std::filesystem::remove_all(path, ec);
}

int main() { return test::run(); }
