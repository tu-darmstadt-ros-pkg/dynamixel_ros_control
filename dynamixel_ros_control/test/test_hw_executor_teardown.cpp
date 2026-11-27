#include <atomic>
#include <chrono>
#include <thread>

#include <std_srvs/srv/trigger.hpp>

#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {

// The component spins its own node on an executor thread. Destroying it must not rebuild that
// executor's wait set while the thread still evaluates it, which aborts the process with
// "ExecutorNotifyWaitable::is_ready: Internal error, got index out of range". The race is narrow,
// so the executor is kept busy with reboot requests (refused while not unconfigured) and the
// teardown repeated.
using ExecutorTeardownTest = DirectHardwareTest;

namespace {
/// Repetitions bounded by wall clock, since one costs tens of ms and far more under load or a sanitizer
bool withinBudget(const std::chrono::steady_clock::time_point start, const std::chrono::seconds budget)
{
  return std::chrono::steady_clock::now() - start < scaled(budget);
}
}  // namespace

TEST_F(ExecutorTeardownTest, DestroyingTheComponentNeverAborts)
{
  auto node = std::make_shared<rclcpp::Node>("executor_teardown_client");
  auto client = node->create_client<std_srvs::srv::Trigger>("/athena_arm_interface_node/reboot");
  std::atomic<bool> publishing{true};
  std::thread traffic([&] {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    while (publishing) {
      (void) client->async_send_request(request);
      client->prune_pending_requests();
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  });

  const auto start = std::chrono::steady_clock::now();
  int iterations = 0;
  for (; iterations < 5000 && withinBudget(start, std::chrono::seconds(120)); ++iterations) {
    initComponent("athena_arm_interface");
    ASSERT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::SUCCESS);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    hardware_.reset();
  }
  publishing = false;
  traffic.join();
  // The old destructor aborted once in about 4000 teardowns, so a run cut short by the budget shows here
  RecordProperty("iterations", iterations);
}

// A component destroyed right after init stops its executor before the thread has entered spin().
// The stop must not get lost there, or the destructor waits for the thread forever.
TEST_F(ExecutorTeardownTest, DestroyingRightAfterInitNeverHangs)
{
  const auto start = std::chrono::steady_clock::now();
  int iterations = 0;
  for (; iterations < 2000 && withinBudget(start, std::chrono::seconds(120)); ++iterations) {
    initComponent("athena_arm_interface");
    ASSERT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::SUCCESS);
    hardware_.reset();
  }
  RecordProperty("iterations", iterations);
}

}  // namespace dynamixel_ros_control::test
