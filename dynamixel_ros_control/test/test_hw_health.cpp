// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace dynamixel_ros_control::test {

namespace {

const diagnostic_msgs::msg::KeyValue* findValue(const diagnostic_msgs::msg::DiagnosticStatus& status,
                                                const std::string& key)
{
  for (const auto& kv : status.values) {
    if (kv.key == key)
      return &kv;
  }
  return nullptr;
}

// Find the bus-level status (`<hw_iface>/bus`).
const diagnostic_msgs::msg::DiagnosticStatus* findBus(const diagnostic_msgs::msg::DiagnosticArray& msg)
{
  for (const auto& s : msg.status) {
    if (s.name.size() >= 4 && s.name.substr(s.name.size() - 4) == "/bus")
      return &s;
  }
  return nullptr;
}

}  // namespace

// ============================================================================
// Diagnostics tests (~/diagnostics, 1 Hz timer, runtime state)
// ============================================================================

TEST_F(HardwareInterfaceTest, Health_PublishedPeriodically)
{
  std::vector<diagnostic_msgs::msg::DiagnosticArray::SharedPtr> messages;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface_node/diagnostics", rclcpp::QoS(10),
      [&messages](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { messages.push_back(m); });

  // 1 Hz nominal rate. Waits well beyond the 3 s that 3 messages take, for a loaded machine
  auto deadline = std::chrono::steady_clock::now() + scaled(15s);
  while (messages.size() < 3 && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_GE(messages.size(), 3u) << "Diagnostics publisher should produce at least 3 messages at 1 Hz";
}

// A hardware error takes the component to UNCONFIGURED, where read() no longer refreshes the
// snapshot. A timer left running keeps republishing the last healthy sample at 1 Hz. That is a
// fault reported as the current state of a bus nobody is reading any more.
TEST_F(HardwareInterfaceTest, Health_StopsPublishingOnceTheComponentIsUnconfigured)
{
  std::vector<diagnostic_msgs::msg::DiagnosticArray::SharedPtr> messages;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface_node/diagnostics", rclcpp::QoS(10),
      [&messages](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { messages.push_back(m); });

  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);
  motor->setHardwareError(ERROR_OVERHEATING);

  // Wait for the error to take the component out of ACTIVE.
  auto deadline = std::chrono::steady_clock::now() + 10s;
  while (hardwareComponentState("athena_arm_interface") == "active" && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(100ms);
  }
  ASSERT_NE(hardwareComponentState("athena_arm_interface"), "active")
      << "Test premise: the hardware error should have taken the component out of ACTIVE";

  // Let anything already in flight drain, then require silence.
  std::this_thread::sleep_for(1500ms);
  executor_->spin_some();
  messages.clear();

  deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_TRUE(messages.empty()) << "The diagnostics timer kept publishing " << messages.size()
                                << " message(s) after the component left ACTIVE";
}

TEST_F(HardwareInterfaceTest, Health_ContainsExpectedFields)
{
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface_node/diagnostics", rclcpp::QoS(10),
      [&msg](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { msg = m; });

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!msg && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_NE(msg, nullptr);

  // Bus-level keys.
  const auto* bus = findBus(*msg);
  ASSERT_NE(bus, nullptr) << "Missing bus-level health status";
  for (const auto* key :
       {"read_consecutive_errors", "write_consecutive_errors", "last_successful_read", "mode_switch_failed"}) {
    EXPECT_NE(findValue(*bus, key), nullptr) << "Missing bus key '" << key << "'";
  }

  // Per-joint keys.
  const diagnostic_msgs::msg::DiagnosticStatus* joint_status = nullptr;
  for (const auto& s : msg->status) {
    if (s.name.find("arm_joint_1") != std::string::npos) {
      joint_status = &s;
      break;
    }
  }
  ASSERT_NE(joint_status, nullptr);
  for (const auto* key :
       {"motor_id", "torque_desired", "torque_enable", "hardware_error_status", "operating_mode_desired"}) {
    EXPECT_NE(findValue(*joint_status, key), nullptr) << "Missing joint key '" << key << "'";
  }
}

TEST_F(HardwareInterfaceTest, Health_ReflectsTorqueState)
{
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface_node/diagnostics", rclcpp::QoS(10),
      [&msg](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { msg = m; });

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!msg && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_NE(msg, nullptr);

  auto value_for = [&](const std::string& joint, const std::string& key) -> std::string {
    for (const auto& s : msg->status) {
      if (s.name.find(joint) != std::string::npos) {
        if (const auto* v = findValue(s, key))
          return v->value;
      }
    }
    return {};
  };
  auto wait_for = [&](const std::string& key, const std::string& value) {
    const auto until = std::chrono::steady_clock::now() + 3s;
    while (value_for("arm_joint_1", key) != value && std::chrono::steady_clock::now() < until) {
      executor_->spin_some();
      std::this_thread::sleep_for(50ms);
    }
    return value_for("arm_joint_1", key);
  };
  // Torque is on after on_activate
  EXPECT_EQ(value_for("arm_joint_1", "torque_desired"), "true");
  EXPECT_EQ(wait_for("torque_enable", "on"), "on");

  ASSERT_TRUE(limp());
  EXPECT_EQ(wait_for("torque_desired", "false"), "false");
  EXPECT_EQ(wait_for("torque_enable", "off"), "off");

  // torque_enable is what the motor reports, also when that differs from what the plugin wants
  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);
  motor->write1Byte(motor->getAddress("torque_enable"), 1);
  EXPECT_EQ(wait_for("torque_enable", "on"), "on");
  EXPECT_EQ(value_for("arm_joint_1", "torque_desired"), "false");
  motor->write1Byte(motor->getAddress("torque_enable"), 0);

  ASSERT_TRUE(releaseLimp());
}

}  // namespace dynamixel_ros_control::test
