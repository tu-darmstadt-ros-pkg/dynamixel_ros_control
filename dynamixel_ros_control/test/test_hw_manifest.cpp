// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

namespace dynamixel_ros_control::test {

namespace {

// Find a KeyValue by key in a DiagnosticStatus. Returns nullptr if absent.
const diagnostic_msgs::msg::KeyValue* findValue(const diagnostic_msgs::msg::DiagnosticStatus& status,
                                                const std::string& key)
{
  for (const auto& kv : status.values) {
    if (kv.key == key)
      return &kv;
  }
  return nullptr;
}

}  // namespace

// ============================================================================
// Manifest tests (~/manifest, transient_local, one-shot per configure / after reboot)
// ============================================================================

TEST_F(HardwareInterfaceTest, Manifest_PublishedOnConfigure)
{
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface/manifest", rclcpp::QoS(1).transient_local(),
      [&msg](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { msg = m; });

  auto deadline = std::chrono::steady_clock::now() + 10s;
  while (!msg && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }

  ASSERT_NE(msg, nullptr) << "No manifest message received";
  // One bus-level status + one per joint. The URDF defines at least 7 arm joints.
  EXPECT_GE(msg->status.size(), 8u) << "Expected bus + 7 joint statuses";
  EXPECT_EQ(msg->status[0].level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST_F(HardwareInterfaceTest, Manifest_ContainsExpectedJointFields)
{
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface/manifest", rclcpp::QoS(1).transient_local(),
      [&msg](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { msg = m; });

  auto deadline = std::chrono::steady_clock::now() + 10s;
  while (!msg && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_NE(msg, nullptr);

  // Locate the arm_joint_1 status.
  const diagnostic_msgs::msg::DiagnosticStatus* joint_status = nullptr;
  for (const auto& s : msg->status) {
    if (s.name.find("arm_joint_1") != std::string::npos) {
      joint_status = &s;
      break;
    }
  }
  ASSERT_NE(joint_status, nullptr) << "arm_joint_1 not present in manifest";

  // Required keys. firmware_version may be absent on mock motors but the key must exist.
  for (const auto* key :
       {"motor_id", "model_number", "firmware_version", "operating_mode", "command_interfaces", "state_interfaces"}) {
    EXPECT_NE(findValue(*joint_status, key), nullptr) << "Missing key '" << key << "'";
  }
}

TEST_F(HardwareInterfaceTest, Manifest_RepublishedAfterReboot)
{
  // Latch the first manifest with transient_local QoS.
  std::vector<diagnostic_msgs::msg::DiagnosticArray::SharedPtr> messages;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface/manifest", rclcpp::QoS(1).transient_local(),
      [&messages](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { messages.push_back(m); });

  auto deadline = std::chrono::steady_clock::now() + 10s;
  while (messages.empty() && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_GE(messages.size(), 1u);
  const size_t before = messages.size();

  // Inject a hardware error and trigger reboot.
  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);
  motor->setHardwareError(dynamixel_ros_control::ERROR_OVERLOAD);
  std::this_thread::sleep_for(500ms);

  auto reboot_client = tester_node_->create_test_client<std_srvs::srv::Trigger>("/athena_arm_interface/reboot");
  ASSERT_TRUE(reboot_client->wait_for_service(*executor_, 5s));
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 10s;
  options.response_timeout = 10s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::Trigger>(reboot_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success) << resp->message;

  deadline = std::chrono::steady_clock::now() + 5s;
  while (messages.size() <= before && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_GT(messages.size(), before) << "Manifest should be republished after a successful reboot";
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
