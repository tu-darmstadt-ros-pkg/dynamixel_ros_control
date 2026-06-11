// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <algorithm>
#include <optional>

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

// Locate a joint-level DiagnosticStatus by joint name substring. Returns nullptr if absent.
const diagnostic_msgs::msg::DiagnosticStatus* findJointStatus(const diagnostic_msgs::msg::DiagnosticArray& msg,
                                                              const std::string& joint_name)
{
  for (const auto& s : msg.status) {
    if (s.name.find(joint_name) != std::string::npos)
      return &s;
  }
  return nullptr;
}

// Read a register by name from a mock motor as a raw int32, reproducing the exact sign-extension
// the driver applies in DynamixelDriver::readRegister (1 byte -> int8, 2 bytes -> int16, 4 bytes ->
// int32). This is the ground truth the manifest's readInt() must match: the manifest reports raw
// register ints, so comparing against the mock's stored register validates that the published value
// reflects what the driver actually wrote/read on the hardware. Returns std::nullopt if the model
// does not declare the register (address 0), mirroring the manifest's omission behavior.
std::optional<int32_t> readMockRegisterRaw(const std::shared_ptr<MockDynamixel>& motor, const std::string& name)
{
  const uint16_t address = motor->getAddress(name);
  if (address == 0)
    return std::nullopt;  // Register not present in this model's control table.
  switch (motor->getLength(name)) {
    case 1:
      return static_cast<int32_t>(static_cast<int8_t>(motor->read1Byte(address)));
    case 2:
      return static_cast<int32_t>(static_cast<int16_t>(motor->read2Byte(address)));
    case 4:
      return motor->read4ByteSigned(address);
    default:
      return std::nullopt;
  }
}

// Registers the manifest exposes only "when available" — a model that does not declare the register
// has the key omitted entirely (add_int_if() silently skips it in DynamixelDiagnostics).
inline const std::vector<std::string> kProfileRegisters = {"profile_acceleration", "profile_velocity"};
inline const std::vector<std::string> kControlGainRegisters = {"velocity_i_gain",     "velocity_p_gain",
                                                               "position_d_gain",     "position_i_gain",
                                                               "position_p_gain",     "feedforward_2nd_gain",
                                                               "feedforward_1st_gain"};

}  // namespace

// ============================================================================
// Manifest tests (~/manifest, transient_local, one-shot per configure / after reboot)
// ============================================================================

TEST_F(HardwareInterfaceTest, Manifest_PublishedOnConfigure)
{
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface_node/manifest", rclcpp::QoS(1).transient_local(),
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
      "/athena_arm_interface_node/manifest", rclcpp::QoS(1).transient_local(),
      [&msg](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { msg = m; });

  auto deadline = std::chrono::steady_clock::now() + 10s;
  while (!msg && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_NE(msg, nullptr);

  // Locate the arm_joint_1 status.
  const diagnostic_msgs::msg::DiagnosticStatus* joint_status = findJointStatus(*msg, "arm_joint_1");
  ASSERT_NE(joint_status, nullptr) << "arm_joint_1 not present in manifest";

  // Required keys. firmware_version may be absent on mock motors but the key must exist.
  for (const auto* key :
       {"motor_id", "model_number", "firmware_version", "operating_mode", "command_interfaces", "state_interfaces"}) {
    EXPECT_NE(findValue(*joint_status, key), nullptr) << "Missing key '" << key << "'";
  }
}

// homing_offset is configured per joint in the URDF (registers.homing_offset, in radians). The driver
// converts it to a raw register tick value and writes it to the motor; the manifest republishes that
// raw readback. This test asserts the key is present for every arm joint and that its published value
// matches the raw register the mock motor actually holds — i.e. the manifest reflects what the driver
// configured from the URDF, not a hardcoded or stale value.
TEST_F(HardwareInterfaceTest, Manifest_HomingOffsetMatchesConfigured)
{
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface_node/manifest", rclcpp::QoS(1).transient_local(),
      [&msg](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { msg = m; });

  auto deadline = std::chrono::steady_clock::now() + 10s;
  while (!msg && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_NE(msg, nullptr);

  // arm_joint_N -> motor id (N + 10), per the URDF.
  for (int joint_index = 1; joint_index <= 7; ++joint_index) {
    const std::string joint_name = "arm_joint_" + std::to_string(joint_index);
    const auto motor_id = static_cast<uint8_t>(ARM_JOINT_1_ID + (joint_index - 1));

    const diagnostic_msgs::msg::DiagnosticStatus* joint_status = findJointStatus(*msg, joint_name);
    ASSERT_NE(joint_status, nullptr) << joint_name << " not present in manifest";

    auto motor = MockDynamixelManager::instance().getMotor(motor_id);
    ASSERT_NE(motor, nullptr) << "Mock motor " << static_cast<int>(motor_id) << " missing";

    const auto expected = readMockRegisterRaw(motor, "homing_offset");
    ASSERT_TRUE(expected.has_value()) << joint_name << " mock lacks homing_offset register";

    const auto* kv = findValue(*joint_status, "homing_offset");
    ASSERT_NE(kv, nullptr) << joint_name << " manifest missing homing_offset";
    EXPECT_EQ(kv->value, std::to_string(*expected))
        << joint_name << " manifest homing_offset (" << kv->value << ") does not match the configured register value ("
        << *expected << ")";
  }

  // arm_joint_1 is configured with homing_offset 0.0 rad -> 0 ticks; arm_joint_2 with a non-zero
  // offset. Assert the non-zero case so a conversion that silently collapses to 0 would be caught.
  {
    auto motor2 = MockDynamixelManager::instance().getMotor(ARM_JOINT_2_ID);
    ASSERT_NE(motor2, nullptr);
    const auto raw = readMockRegisterRaw(motor2, "homing_offset");
    ASSERT_TRUE(raw.has_value());
    EXPECT_NE(*raw, 0) << "arm_joint_2 URDF configures a non-zero homing_offset; expected non-zero register ticks";
  }
}

// Motion-profile setpoints and control-gain registers are exposed "when available": the manifest
// includes the key iff the model declares the register, and the published value must equal the raw
// register the mock motor holds. This validates both the value plumbing and the omission contract.
TEST_F(HardwareInterfaceTest, Manifest_ProfileAndGainValuesMatchRegisters)
{
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface_node/manifest", rclcpp::QoS(1).transient_local(),
      [&msg](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { msg = m; });

  auto deadline = std::chrono::steady_clock::now() + 10s;
  while (!msg && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_NE(msg, nullptr);

  // At least one arm joint must actually exercise each register family, otherwise the test would
  // vacuously pass if a model stopped declaring them.
  size_t profile_keys_checked = 0;
  size_t gain_keys_checked = 0;

  for (int joint_index = 1; joint_index <= 7; ++joint_index) {
    const std::string joint_name = "arm_joint_" + std::to_string(joint_index);
    const auto motor_id = static_cast<uint8_t>(ARM_JOINT_1_ID + (joint_index - 1));

    const diagnostic_msgs::msg::DiagnosticStatus* joint_status = findJointStatus(*msg, joint_name);
    ASSERT_NE(joint_status, nullptr) << joint_name << " not present in manifest";

    auto motor = MockDynamixelManager::instance().getMotor(motor_id);
    ASSERT_NE(motor, nullptr);

    std::vector<std::string> registers = kProfileRegisters;
    registers.insert(registers.end(), kControlGainRegisters.begin(), kControlGainRegisters.end());

    for (const auto& reg : registers) {
      const auto expected = readMockRegisterRaw(motor, reg);
      const auto* kv = findValue(*joint_status, reg);

      if (!expected.has_value()) {
        // Advertised omission: a model that does not declare the register must not list the key.
        EXPECT_EQ(kv, nullptr) << joint_name << " manifest exposes '" << reg
                               << "' but the model does not declare that register";
        continue;
      }

      ASSERT_NE(kv, nullptr) << joint_name << " manifest missing available register '" << reg << "'";
      EXPECT_EQ(kv->value, std::to_string(*expected)) << joint_name << " manifest '" << reg << "' (" << kv->value
                                                      << ") does not match the register value (" << *expected << ")";

      if (std::find(kProfileRegisters.begin(), kProfileRegisters.end(), reg) != kProfileRegisters.end())
        ++profile_keys_checked;
      else
        ++gain_keys_checked;
    }
  }

  EXPECT_GT(profile_keys_checked, 0u) << "No arm joint exposed any profile register; coverage would be vacuous";
  EXPECT_GT(gain_keys_checked, 0u) << "No arm joint exposed any control-gain register; coverage would be vacuous";
}

TEST_F(HardwareInterfaceTest, Manifest_RepublishedAfterReboot)
{
  // Latch the first manifest with transient_local QoS.
  std::vector<diagnostic_msgs::msg::DiagnosticArray::SharedPtr> messages;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface_node/manifest", rclcpp::QoS(1).transient_local(),
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

  auto reboot_client = tester_node_->create_test_client<std_srvs::srv::Trigger>("/athena_arm_interface_node/reboot");
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
