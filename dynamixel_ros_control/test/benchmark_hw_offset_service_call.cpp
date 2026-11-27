#include <benchmark/benchmark.h>

#include <algorithm>
#include <chrono>
#include <optional>
#include <stop_token>
#include <thread>

#include <hector_transmission_interface_msgs/srv/adjust_transmission_offsets.hpp>
#include <hector_transmission_interface_msgs/srv/flip_by_pi.hpp>

#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {
namespace {

using AdjustOffsets = hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets;
using FlipByPi = hector_transmission_interface_msgs::srv::FlipByPi;

// How long an offset service call holds the bus. Wall clock bound, so a shared runner cannot
// assert it, see the README for running it.
class OffsetServiceCall : public DirectHardwareTest
{
public:
  void TestBody() override {}

  bool start()
  {
    SetUp();
    startComponent("athena_flipper_interface");
    node_ = std::make_shared<rclcpp::Node>("offset_service_call_benchmark");
    adjust_ = node_->create_client<AdjustOffsets>("/athena_flipper_interface_node/adjust_transmission_offsets");
    flip_ = node_->create_client<FlipByPi>("/athena_flipper_interface_node/flip_by_pi");
    return !HasFailure() && adjust_->wait_for_service(std::chrono::seconds(10)) &&
           flip_->wait_for_service(std::chrono::seconds(10));
  }

  void stop()
  {
    node_.reset();
    TearDown();
  }

  /// One adjustment and one flip, false if one did not succeed
  bool callBoth(int i)
  {
    auto adjust_request = std::make_shared<AdjustOffsets::Request>();
    adjust_request->external_joint_measurements.name = {"flipper_fl_joint"};
    adjust_request->external_joint_measurements.position = {0.1 * (i % 2)};
    auto flip_request = std::make_shared<FlipByPi::Request>();
    flip_request->joint_names = {"flipper_fl_joint"};
    return call(adjust_, adjust_request) && call(flip_, flip_request);
  }

  uint64_t skipped() const
  {
    return hardware_->overlapSkippedCycles();
  }

  /// read() alone, which skips while the bus is held
  void read()
  {
    const rclcpp::Duration period = rclcpp::Duration::from_seconds(kPeriod);
    time_ += period;
    (void) hardware_->read(time_, period);
  }

private:
  template <typename Client, typename Request>
  bool call(Client& client, Request request)
  {
    auto future = client->async_send_request(request);
    return rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(10)) ==
               rclcpp::FutureReturnCode::SUCCESS &&
           future.get()->success;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<AdjustOffsets>::SharedPtr adjust_;
  rclcpp::Client<FlipByPi>::SharedPtr flip_;
};

// How long read() finds the bus taken per service call, sampled by read() back to back. Includes
// the wait of the service for the read() in flight, which is what the loop sees.
void BM_BusHoldTime(benchmark::State& state)
{
  OffsetServiceCall bench;
  if (!bench.start()) {
    state.SkipWithError("the component or its services did not come up");
    bench.stop();
    return;
  }
  using Clock = std::chrono::steady_clock;
  // Only the probe writes them, and they are read after it joined
  int64_t hold_ns_total = 0;
  int64_t hold_ns_max = 0;
  int64_t holds = 0;
  std::jthread probe([&](const std::stop_token& stop) {
    std::optional<Clock::time_point> held_since;
    uint64_t skipped = bench.skipped();
    while (!stop.stop_requested()) {
      const auto now = Clock::now();
      bench.read();
      const uint64_t skipped_now = bench.skipped();
      if (skipped_now != skipped && !held_since) {
        held_since = now;
      } else if (skipped_now == skipped && held_since) {
        const int64_t hold = std::chrono::duration_cast<std::chrono::nanoseconds>(now - *held_since).count();
        hold_ns_total += hold;
        hold_ns_max = std::max(hold_ns_max, hold);
        ++holds;
        held_since.reset();
      }
      skipped = skipped_now;
    }
  });
  int i = 0;
  for (auto _ : state) {
    if (!bench.callBoth(i++)) {
      state.SkipWithError("an offset service call failed");
      break;
    }
  }
  probe.request_stop();
  probe.join();
  state.counters["holds_seen"] = static_cast<double>(holds);
  state.counters["hold_mean_us"] =
      static_cast<double>(hold_ns_total) / static_cast<double>(std::max<int64_t>(holds, 1)) / 1e3;
  state.counters["hold_max_us"] = static_cast<double>(hold_ns_max) / 1e3;
  bench.stop();
}
BENCHMARK(BM_BusHoldTime)->Iterations(100)->Unit(benchmark::kMicrosecond)->UseRealTime();

}  // namespace
}  // namespace dynamixel_ros_control::test
