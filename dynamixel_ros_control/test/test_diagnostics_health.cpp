#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <dynamixel_ros_control/dynamixel_diagnostics.hpp>
#include <dynamixel_ros_control/sync_read_manager.hpp>
#include <dynamixel_ros_control/sync_write_manager.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dynamixel_ros_control::test {

namespace {
/// Spins @p executor until @p done holds or @p timeout, stretched for sanitizers, has passed
bool spinUntil(rclcpp::Executor& executor, const std::function<bool()>& done,
               const std::chrono::seconds timeout = std::chrono::seconds(10))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout * DXL_TEST_TIMEOUT_FACTOR;
  while (!done()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      return false;
    }
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}
}  // namespace

// on_error publishes the fault on the controller manager's thread while the health timer may publish
// on the component's executor, and both fill the same message. Run under TSan to see a race.
TEST(DiagnosticsHealth, ConcurrentPublishesDoNotRace)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("diagnostics_health_test");
    const std::unordered_map<std::string, Joint> joints;
    const std::vector<std::string> joint_names;
    const SyncReadManager read_manager;
    const SyncWriteManager write_manager;
    DynamixelDiagnostics diagnostics(node, "test_interface", joints, joint_names, read_manager, write_manager);
    diagnostics.snapshotHealth(node->now(), true, false, node->now(), 0.0, 0);

    std::thread other([&diagnostics]() {
      for (int i = 0; i < 2000; ++i) {
        diagnostics.publishHealth();
      }
    });
    for (int i = 0; i < 2000; ++i) {
      diagnostics.publishHealth();
    }
    other.join();
    diagnostics.stopHealthTimer();
  }
  rclcpp::shutdown();
}

// A tick the executor dispatched just before stopHealthTimer() runs after it. on_error publishes the
// fault right after the stop, so such a tick must publish nothing or it would overwrite the fault.
TEST(DiagnosticsHealth, ATickDispatchedBeforeTheStopPublishesNothing)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("diagnostics_stopped_timer_test");
    const std::unordered_map<std::string, Joint> joints;
    const std::vector<std::string> joint_names;
    const SyncReadManager read_manager;
    const SyncWriteManager write_manager;
    DynamixelDiagnostics diagnostics(node, "test_interface", joints, joint_names, read_manager, write_manager);
    diagnostics.snapshotHealth(node->now(), true, false, node->now(), 0.0, 0);

    auto listener = std::make_shared<rclcpp::Node>("diagnostics_stopped_timer_listener");
    std::atomic<int> received{0};
    auto subscription = listener->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics_stopped_timer_test/diagnostics", rclcpp::QoS(10),
        [&received](const diagnostic_msgs::msg::DiagnosticArray&) { ++received; });

    diagnostics.startHealthTimer();
    const auto timer = node->get_node_base_interface()->get_default_callback_group()->find_timer_ptrs_if(
        [](const rclcpp::TimerBase::SharedPtr&) { return true; });
    ASSERT_NE(timer, nullptr);
    const auto dispatch = [&timer]() {
      std::shared_ptr<void> data;
      for (int i = 0; i < 300 && !data; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        data = timer->call();
      }
      return data;
    };
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(listener);

    // The listener sees a tick of a running timer, so silence below is the stop's doing
    auto running_tick = dispatch();
    ASSERT_NE(running_tick, nullptr);
    timer->execute_callback(running_tick);
    ASSERT_TRUE(spinUntil(executor, [&received]() { return received >= 1; }));
    ASSERT_EQ(received, 1);

    auto late_tick = dispatch();
    ASSERT_NE(late_tick, nullptr);
    diagnostics.stopHealthTimer();
    timer->execute_callback(late_tick);
    // A window to watch, since nothing is expected to arrive in it
    spinUntil(executor, [&received]() { return received >= 2; }, std::chrono::seconds(2));
    EXPECT_EQ(received, 1) << "a tick dispatched before the stop published after it";
  }
  rclcpp::shutdown();
}

// Every publish is a message, since on_error's last word and the manifest are each sent only once
TEST(DiagnosticsHealth, BackToBackPublishesAreAllSent)
{
  rclcpp::init(0, nullptr);
  {
    using diagnostic_msgs::msg::DiagnosticArray;
    auto node = std::make_shared<rclcpp::Node>("diagnostics_back_to_back_test");
    const std::unordered_map<std::string, Joint> joints;
    const std::vector<std::string> joint_names;
    const SyncReadManager read_manager;
    const SyncWriteManager write_manager;
    DynamixelDiagnostics diagnostics(node, "test_interface", joints, joint_names, read_manager, write_manager);
    diagnostics.snapshotHealth(node->now(), true, false, node->now(), 0.0, 0);

    auto listener = std::make_shared<rclcpp::Node>("diagnostics_back_to_back_listener");
    std::atomic<int> received{0};
    auto health =
        listener->create_subscription<DiagnosticArray>("/diagnostics_back_to_back_test/diagnostics", rclcpp::QoS(10),
                                                       [&received](const DiagnosticArray&) { ++received; });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(listener);
    spinUntil(executor, [&health]() { return health->get_publisher_count() > 0; });
    ASSERT_GT(health->get_publisher_count(), 0u);

    constexpr int PUBLISHES = 5;
    for (int i = 0; i < PUBLISHES; ++i) {
      diagnostics.publishHealth();
    }
    spinUntil(executor, [&received]() { return received >= PUBLISHES; });
    EXPECT_EQ(received, PUBLISHES) << "a health publish was dropped";

    diagnostics.publishManifest(rclcpp::Time(1, 0));
    diagnostics.publishManifest(rclcpp::Time(2, 0));
    std::atomic<int32_t> manifest_stamp{0};
    auto manifest = listener->create_subscription<DiagnosticArray>(
        "/diagnostics_back_to_back_test/manifest", rclcpp::QoS(1).transient_local(),
        [&manifest_stamp](const DiagnosticArray& msg) { manifest_stamp = msg.header.stamp.sec; });
    spinUntil(executor, [&manifest_stamp]() { return manifest_stamp != 0; });
    EXPECT_EQ(manifest_stamp, 2) << "the latched manifest is not the latest one";
  }
  rclcpp::shutdown();
}

}  // namespace dynamixel_ros_control::test
