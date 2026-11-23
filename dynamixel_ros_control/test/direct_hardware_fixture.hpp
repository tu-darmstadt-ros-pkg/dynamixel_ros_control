#ifndef DYNAMIXEL_ROS_CONTROL_TEST_DIRECT_HARDWARE_FIXTURE_HPP
#define DYNAMIXEL_ROS_CONTROL_TEST_DIRECT_HARDWARE_FIXTURE_HPP

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <dynamixel_ros_control/dynamixel_hardware_interface.hpp>
#include <dynamixel_ros_control/mock_dynamixel.hpp>
#include <hardware_interface/component_parser.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

/// Drives one hardware component of the test robot directly, without a controller manager: one
/// read and one write per simulated cycle against the mock bus. For behaviour a controller manager
/// cannot provoke, such as a NaN command (ForwardCommandController drops it, the legacy controller
/// manager never writes one) or a lifecycle call in a chosen state.
class DirectHardwareTest : public ::testing::Test
{
protected:
  static constexpr double kPeriod = 0.02;

  void SetUp() override
  {
    test_home_.emplace();
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    if (hardware_ && configured_) {
      hardware_->on_deactivate(rclcpp_lifecycle::State());
    }
    hardware_.reset();
    MockDynamixelManager::instance().reset();
    // rcl crashes at exit after a reinit that is never shut down, and HardwareInterfaceTest shuts down per test
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    test_home_.reset();
  }

  /// Loads @p component from the test robot, after @p transform_urdf changed the description.
  void initComponent(
      const std::string& component,
      const std::function<std::string(std::string)>& transform_urdf = [](std::string urdf) { return urdf; })
  {
    const auto infos = hardware_interface::parse_control_resources_from_urdf(
        transform_urdf(load_file(std::string(TEST_CONFIG_DIR) + "/athena.urdf")));
    for (const auto& info : infos) {
      if (info.name == component) {
        params_.hardware_info = info;
      }
    }
    ASSERT_EQ(params_.hardware_info.name, component);
    adjustInfo(params_.hardware_info);
    addDummyMotors({params_.hardware_info});
    params_.hardware_info.rw_rate = static_cast<unsigned int>(std::lround(1.0 / kPeriod));
    params_.clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    params_.logger = rclcpp::get_logger("direct_hardware_test");

    hardware_ = std::make_unique<DynamixelHardwareInterface>();
  }

  /// Changes the parsed component before the plugin sees it
  virtual void adjustInfo(hardware_interface::HardwareInfo& /*info*/) {}

  /// The motors' physics over one period
  virtual void step()
  {
    MockDynamixelManager::instance().update(kPeriod);
  }

  /// Initialises, configures and activates @p component, and runs a few cycles.
  void startComponent(
      const std::string& component,
      const std::function<std::string(std::string)>& transform_urdf = [](std::string urdf) { return urdf; })
  {
    initComponent(component, transform_urdf);
    ASSERT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::SUCCESS);
    for (const auto& state : hardware_->on_export_state_interfaces()) {
      states_[state->get_name()] = state;
    }
    for (const auto& command : hardware_->on_export_command_interfaces()) {
      commands_[command->get_name()] = command;
    }
    ASSERT_EQ(hardware_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
    configured_ = true;
    ASSERT_EQ(hardware_->on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
    cycles(5);
  }

  void startInterface(const std::string& name, double command)
  {
    ASSERT_EQ(hardware_->perform_command_mode_switch({name}, {}), hardware_interface::return_type::OK);
    setCommand(name, command);
  }

  void setCommand(const std::string& name, double value)
  {
    ASSERT_TRUE(commands_.at(name)->set_value(value, true));
  }

  /// One control cycle: read, write, then the mock's physics. Returns the first result that is not OK.
  hardware_interface::return_type cycle()
  {
    const rclcpp::Duration period = rclcpp::Duration::from_seconds(kPeriod);
    time_ += period;
    auto result = hardware_->read(time_, period);
    if (result == hardware_interface::return_type::OK) {
      result = hardware_->write(time_, period);
    }
    step();
    return result;
  }

  void cycles(int count)
  {
    for (int i = 0; i < count; ++i) {
      ASSERT_EQ(cycle(), hardware_interface::return_type::OK);
    }
  }

  /// Waits until @p done holds, and fails the test if it does not within @p timeout
  static void waitUntil(const std::function<bool()>& done, const std::chrono::seconds timeout = std::chrono::seconds(10))
  {
    const auto deadline = std::chrono::steady_clock::now() + scaled(timeout);
    while (!done()) {
      if (std::chrono::steady_clock::now() >= deadline) {
        ADD_FAILURE() << "the condition was not met in time";
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  /// Calls a service of the component's own node, which its executor thread serves.
  /// Returns nullptr if it did not answer within @p timeout.
  template <typename Service>
  typename Service::Response::SharedPtr callService(const std::string& service,
                                                    typename Service::Request::SharedPtr request,
                                                    const std::chrono::seconds timeout = std::chrono::seconds(30))
  {
    auto node = std::make_shared<rclcpp::Node>("direct_hardware_test_service_client");
    auto client = node->create_client<Service>(service);
    if (!client->wait_for_service(scaled(std::chrono::seconds(10)))) {
      ADD_FAILURE() << service << " is not offered";
      return nullptr;
    }
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future, scaled(timeout)) != rclcpp::FutureReturnCode::SUCCESS) {
      return nullptr;
    }
    return future.get();
  }

  /// Calls a service of the component's own node and runs control cycles until it answers, for a
  /// service that read() serves. Every cycle must succeed. Returns nullptr if it never answered.
  template <typename Service>
  typename Service::Response::SharedPtr callWhileCycling(const std::string& service,
                                                         typename Service::Request::SharedPtr request)
  {
    auto node = std::make_shared<rclcpp::Node>("direct_hardware_test_client");
    auto client = node->create_client<Service>(service);
    EXPECT_TRUE(client->wait_for_service(scaled(std::chrono::seconds(10))));
    auto future = client->async_send_request(request);
    for (int i = 0; i < 500; ++i) {
      if (rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds(10)) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        return future.get();
      }
      EXPECT_EQ(cycle(), hardware_interface::return_type::OK) << "a cycle failed while serving " << service;
    }
    ADD_FAILURE() << service << " never answered";
    return nullptr;
  }

  std::optional<TestHome> test_home_;
  hardware_interface::HardwareComponentParams params_;
  std::unique_ptr<DynamixelHardwareInterface> hardware_;
  std::unordered_map<std::string, hardware_interface::CommandInterface::SharedPtr> commands_;
  std::unordered_map<std::string, hardware_interface::StateInterface::ConstSharedPtr> states_;
  rclcpp::Time time_{0, 0, RCL_ROS_TIME};
  /// Only a configured component may be deactivated when the test ends.
  bool configured_{false};
};

}  // namespace dynamixel_ros_control::test

#endif
