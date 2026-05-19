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
      "/athena_arm_interface/diagnostics", rclcpp::QoS(10),
      [&messages](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { messages.push_back(m); });

  // 1 Hz nominal rate. Wait up to 4 s for ≥3 messages, leaving margin for startup delay.
  auto deadline = std::chrono::steady_clock::now() + 4s;
  while (messages.size() < 3 && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_GE(messages.size(), 3u) << "Diagnostics publisher should produce ≥3 messages within 4 s at 1 Hz";
}

TEST_F(HardwareInterfaceTest, Health_ContainsExpectedFields)
{
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface/diagnostics", rclcpp::QoS(10),
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
  for (const auto* key : {"e_stop_active", "read_consecutive_errors", "write_consecutive_errors",
                          "last_successful_read", "mode_switch_failed"}) {
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
  for (const auto* key : {"motor_id", "torque_desired", "hardware_error_status", "operating_mode_desired"}) {
    EXPECT_NE(findValue(*joint_status, key), nullptr) << "Missing joint key '" << key << "'";
  }
}

TEST_F(HardwareInterfaceTest, Health_ReflectsEStopState)
{
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface/diagnostics", rclcpp::QoS(10),
      [&msg](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { msg = m; });

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!msg && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_NE(msg, nullptr);

  const auto* bus = findBus(*msg);
  ASSERT_NE(bus, nullptr);
  const auto* e_stop = findValue(*bus, "e_stop_active");
  ASSERT_NE(e_stop, nullptr);
  EXPECT_EQ(e_stop->value, "false");

  // Engage e-stop and wait for the next health tick.
  auto estop_pub = createEStopPublisher();
  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub->publish(estop_msg);

  deadline = std::chrono::steady_clock::now() + 3s;
  bool saw_warn = false;
  while (!saw_warn && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
    if (msg) {
      const auto* b = findBus(*msg);
      const auto* v = b ? findValue(*b, "e_stop_active") : nullptr;
      if (b && b->level == diagnostic_msgs::msg::DiagnosticStatus::WARN && v && v->value == "true") {
        saw_warn = true;
      }
    }
  }
  EXPECT_TRUE(saw_warn) << "Health should report WARN with e_stop_active=true after e-stop publish";

  // Cleanup.
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(300ms);
}

TEST_F(HardwareInterfaceTest, Health_ReflectsTorqueState)
{
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface/diagnostics", rclcpp::QoS(10),
      [&msg](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { msg = m; });

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!msg && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_NE(msg, nullptr);

  // Initially torque ON (set on_activate). Find any arm joint and inspect `torque_desired`.
  auto torque_value_for = [&](const diagnostic_msgs::msg::DiagnosticArray& m, const std::string& joint) -> std::string {
    for (const auto& s : m.status) {
      if (s.name.find(joint) != std::string::npos) {
        if (const auto* v = findValue(s, "torque_desired"))
          return v->value;
      }
    }
    return {};
  };
  EXPECT_EQ(torque_value_for(*msg, "arm_joint_1"), "true");

  // Disable torque via service.
  auto torque_client = createTorqueClient();
  ASSERT_TRUE(setTorque(torque_client, false));

  deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
    if (msg && torque_value_for(*msg, "arm_joint_1") == "false") {
      break;
    }
  }
  EXPECT_EQ(torque_value_for(*msg, "arm_joint_1"), "false");

  ASSERT_TRUE(setTorque(torque_client, true));
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
