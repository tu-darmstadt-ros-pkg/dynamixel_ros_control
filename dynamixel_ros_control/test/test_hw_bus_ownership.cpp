#include <atomic>
#include <chrono>
#include <thread>
#include <tuple>

#include <std_srvs/srv/trigger.hpp>

#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {

// read() and write() only try the bus, every other caller waits for it. read() runs while the
// component is inactive too, so a reboot there would wipe the motors' RAM under it, and read()
// reports that as an error.
using BusOwnershipTest = DirectHardwareTest;

TEST_F(BusOwnershipTest, ARebootNeverRunsUnderReadAndWrite)
{
  startComponent("athena_arm_interface");
  ASSERT_EQ(hardware_->on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  // A motor that cannot be asked for its error is rebooted, so a reboot here would do real work
  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_3_ID);
  ASSERT_NE(motor, nullptr);
  motor->setRegisterReadFault("hardware_error_status");

  const auto response = callWhileCycling<std_srvs::srv::Trigger>("/athena_arm_interface_node/reboot",
                                                                 std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success) << "a reboot ran while the component was inactive";
  EXPECT_EQ(motor->getRebootCount(), 0);
}

// A reboot request that arrives while on_configure holds the bus waits for it, and then finds the
// component configured. The lifecycle is checked under the bus lock, so it is refused.
TEST_F(BusOwnershipTest, ARebootRequestedDuringConfigureIsRefused)
{
  initComponent("athena_arm_interface");
  ASSERT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::SUCCESS);
  std::ignore = hardware_->on_export_state_interfaces();
  std::ignore = hardware_->on_export_command_interfaces();
  auto node = std::make_shared<rclcpp::Node>("bus_ownership_test_client");
  auto client = node->create_client<std_srvs::srv::Trigger>("/athena_arm_interface_node/reboot");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(10)));

  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_3_ID);
  ASSERT_NE(motor, nullptr);
  std::atomic<bool> requested{false};
  std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr> response;
  motor->setRegisterReadHook("torque_enable", [&]() {
    if (requested.exchange(true)) {
      return;
    }
    response = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>()).future.share();
    // Long enough for the service to receive the request while configure still holds the bus
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  });
  ASSERT_EQ(hardware_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  configured_ = true;
  motor->clearRegisterReadHooks();
  ASSERT_TRUE(requested) << "precondition: configure reads torque_enable";

  ASSERT_EQ(rclcpp::spin_until_future_complete(node, response, std::chrono::seconds(10)),
            rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_FALSE(response.get()->success) << "a reboot ran on a configured component";
}

// A refused reboot does not wait for the bus, so it never costs the control loop a cycle
TEST_F(BusOwnershipTest, ARefusedRebootDoesNotWaitForTheBus)
{
  startComponent("athena_flipper_interface");
  auto node = std::make_shared<rclcpp::Node>("bus_ownership_test_client");
  auto client = node->create_client<std_srvs::srv::Trigger>("/athena_flipper_interface_node/reboot");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(10)));
  auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_NE(motor, nullptr);
  bool held = false;
  bool answered_while_held = false;
  motor->setRegisterReadHook("operating_mode", [&]() {
    if (held) {
      return;
    }
    held = true;
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    answered_while_held = rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds(500)) ==
                              rclcpp::FutureReturnCode::SUCCESS &&
                          !future.get()->success;
  });
  ASSERT_EQ(hardware_->perform_command_mode_switch({"flipper_fl_joint/velocity"}, {}),
            hardware_interface::return_type::OK);
  motor->clearRegisterReadHooks();
  ASSERT_TRUE(held) << "precondition: the switch reads the operating mode";
  EXPECT_TRUE(answered_while_held) << "the refusal waited for the bus a switch held";
}

// The stock controller manager runs a switch requested without activate_asap on its service thread
// while the control thread keeps calling read() and write(). Those wait out the switch: they skip
// the cycle and count it, instead of driving the bus alongside it.
TEST_F(BusOwnershipTest, ReadAndWriteDuringAModeSwitchSkipTheCycle)
{
  startComponent("athena_flipper_interface");
  auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_NE(motor, nullptr);
  bool overlapped = false;
  hardware_interface::return_type read_result = hardware_interface::return_type::ERROR;
  hardware_interface::return_type write_result = hardware_interface::return_type::ERROR;
  motor->setRegisterReadHook("operating_mode", [&]() {
    if (overlapped) {
      return;
    }
    overlapped = true;
    // The control thread of the stock controller manager
    std::thread([&]() {
      const rclcpp::Duration period = rclcpp::Duration::from_seconds(kPeriod);
      read_result = hardware_->read(time_, period);
      write_result = hardware_->write(time_, period);
    }).join();
  });
  ASSERT_EQ(hardware_->perform_command_mode_switch({"flipper_fl_joint/velocity"}, {}),
            hardware_interface::return_type::OK);
  motor->clearRegisterReadHooks();
  ASSERT_TRUE(overlapped) << "precondition: the switch reads the operating mode";
  EXPECT_EQ(read_result, hardware_interface::return_type::OK);
  EXPECT_EQ(write_result, hardware_interface::return_type::OK);
  EXPECT_EQ(hardware_->overlapSkippedCycles(), 2u) << "the overlap was not noticed";
  cycles(5);
}

// The other order: a switch that starts while read() drives the bus waits for read() to finish
TEST_F(BusOwnershipTest, AModeSwitchWaitsForARunningRead)
{
  startComponent("athena_flipper_interface");
  auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_NE(motor, nullptr);
  std::atomic<bool> switch_done{false};
  std::atomic<bool> started{false};
  bool done_during_read = true;
  std::thread switcher;
  motor->setRegisterReadHook("present_position", [&]() {
    if (started.exchange(true)) {
      return;
    }
    switcher = std::thread([&]() {
      std::ignore = hardware_->perform_command_mode_switch({"flipper_fl_joint/velocity"}, {});
      switch_done = true;
    });
    waitUntil([&]() { return hardware_->busWaiters() > 0 || switch_done; });
    done_during_read = switch_done;
  });
  ASSERT_EQ(cycle(), hardware_interface::return_type::OK);
  motor->clearRegisterReadHooks();
  ASSERT_TRUE(started) << "precondition: read() reads the present position";
  switcher.join();
  EXPECT_FALSE(done_during_read) << "the switch drove the bus while read() did";
  EXPECT_TRUE(switch_done);
}

// A switch that waits for read() comes before the control loop's next call, so it waits for at most
// the one call in flight
TEST_F(BusOwnershipTest, AWaitingModeSwitchMakesTheNextWriteSkip)
{
  startComponent("athena_flipper_interface");
  auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_NE(motor, nullptr);
  std::atomic<bool> started{false};
  std::thread switcher;
  motor->setRegisterReadHook("present_position", [&]() {
    if (started.exchange(true)) {
      return;
    }
    switcher =
        std::thread([&]() { std::ignore = hardware_->perform_command_mode_switch({"flipper_fl_joint/velocity"}, {}); });
    waitUntil([&]() { return hardware_->busWaiters() > 0; });
  });
  const uint64_t skipped_before = hardware_->overlapSkippedCycles();
  ASSERT_EQ(cycle(), hardware_interface::return_type::OK);
  motor->clearRegisterReadHooks();
  ASSERT_TRUE(started) << "precondition: read() reads the present position";
  switcher.join();
  EXPECT_EQ(hardware_->overlapSkippedCycles(), skipped_before + 1) << "write() took the bus before the waiting switch";
}

}  // namespace dynamixel_ros_control::test
