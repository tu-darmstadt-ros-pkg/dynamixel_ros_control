// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Strict initialization
// ============================================================================
//
// The configured initial register values are the safety-relevant ones: current limits, drive mode,
// motion profiles, homing offsets. A motor that cannot have them applied must not come up, and must
// not be energised.

// A joint whose configured initial value cannot be applied must prevent the interface coming up.
// An unparsable value is the cheapest way to make writeInitialValues() fail for a real reason
// (it is a plain operator typo) without needing to fault the bus.
class BadInitialValueTest : public HardwareInterfaceTest
{
protected:
  void SetUp() override
  {
    require_hardware_active_ = false;  // the arm interface is expected to refuse to configure
    HardwareInterfaceTest::SetUp();
  }

  std::string transformUrdf(std::string urdf) override
  {
    const std::string anchor = "<param name=\"id\">" + std::to_string(static_cast<int>(ARM_JOINT_1_ID)) + "</param>";
    const auto pos = urdf.find(anchor);
    if (pos == std::string::npos) {
      ADD_FAILURE() << "Could not find arm_joint_1 id anchor";
      return urdf;
    }
    // led_red is not already configured for this joint — a duplicate <param> would be ignored by
    // the URDF parser (first one wins), silently making this fixture a no-op.
    const std::string injected = anchor + "\n      <param name=\"registers.led_red\">not_a_number</param>";
    urdf.replace(pos, anchor.size(), injected);
    return urdf;
  }
};

TEST_F(BadInitialValueTest, UnappliableInitialValueBlocksActivation)
{
  std::this_thread::sleep_for(500ms);

  const std::string arm_state = hardwareComponentState("athena_arm_interface");
  const std::string flipper_state = hardwareComponentState("athena_flipper_interface");

  EXPECT_NE(arm_state, "active") << "The arm interface reached '" << arm_state
                                 << "' even though a configured initial register value could not be applied. "
                                    "Torque may be enabled with unintended settings.";
  // The flipper bus is independent and must be unaffected — a strict failure must not be a
  // sledgehammer that takes down healthy hardware.
  EXPECT_EQ(flipper_state, "active") << "The unrelated flipper interface was taken down too";

  // Nothing on the arm may be energised.
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 0)
        << "Motor " << static_cast<int>(id) << " is torqued despite a failed initialization";
  }
}

// A reboot restores the configured registers from scratch (the reboot wipes RAM). If that
// restoration fails, the reboot must report failure and must not go on to re-enable torque.
TEST_F(HardwareInterfaceTest, RebootWithFailedInitialValuesDoesNotRestoreTorque)
{
  std::this_thread::sleep_for(300ms);

  constexpr uint8_t FAULTED_ID = ARM_JOINT_1_ID;
  auto motor = MockDynamixelManager::instance().getMotor(FAULTED_ID);
  ASSERT_NE(motor, nullptr);

  // Torque must be ON going in (torque_on_startup=true), so the assertion below is load-bearing:
  // a reboot power-cycles the motor into torque-off, and it is reboot()'s *restore* step that would
  // put it back on. If the restore runs despite the failed register write, the motor comes back
  // energised with unrestored settings, which must not happen.
  ASSERT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 1) << "Test premise: the arm should start torqued";

  // Provoke the reboot, and make the post-reboot restoration of profile_velocity impossible.
  motor->setHardwareError(ERROR_OVERHEATING);
  motor->setRegisterWriteFault("profile_velocity", RegisterWriteFault::TIMEOUT);
  // A reboot refused while the component is still active would pass this test for the wrong reason
  ASSERT_TRUE(waitForComponentState("athena_arm_interface", "unconfigured"));

  auto resp = rebootMotors();
  ASSERT_NE(resp, nullptr);
  EXPECT_FALSE(resp->success) << "Reboot reported success even though a configured initial register value could "
                                 "not be restored: "
                              << resp->message;

  // Only the motor that reported an error is rebooted, so only it was power-cycled into torque-off.
  // The others keep the torque they already had and are deliberately not asserted on here.
  std::this_thread::sleep_for(300ms);
  EXPECT_GT(motor->getRebootCount(), 0) << "Test premise: the faulted motor should have been rebooted";
  EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 0)
      << "Motor " << static_cast<int>(FAULTED_ID)
      << " was re-energised by a reboot that could not restore its configured registers";

  motor->clearRegisterWriteFaults();
  motor->setHardwareError(0);
}

// A warm start finds the motors still torqued from the previous session, and the firmware refuses
// EEPROM writes while a motor is energised. Failing configuration there would refuse to bring the
// bus up, and the arm stays energised with nobody managing it, respawn after respawn. The
// write is retried de-energised instead, and the torque state put back.
TEST_F(HardwareInterfaceTest, WarmStart_EepromInitialValueIsWrittenDespiteTorque)
{
  std::this_thread::sleep_for(400ms);

  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_4_ID);
  ASSERT_NE(motor, nullptr);
  ASSERT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 1) << "Test premise: the motors must be energised";
  std::this_thread::sleep_for(300ms);

  // From here the motor behaves like real hardware: EEPROM is locked while it is energised.
  motor->setRejectEepromWritesWhileTorqued(true);
  // Make the configured current_limit (22 A) differ from what the motor holds, so on_configure has
  // a genuine EEPROM write to perform rather than skipping it as already correct.
  motor->write2Byte(motor->getAddress("current_limit"), 1000);

  auto hw_state_client =
      tester_node_->create_test_client<SetHardwareComponentState>("/controller_manager/set_hardware_component_state");
  ASSERT_TRUE(hw_state_client->wait_for_service(*executor_, 5s));
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 15s;
  options.response_timeout = 15s;
  const auto transition = [&](uint8_t state_id, const std::string& label) {
    auto request = std::make_shared<SetHardwareComponentState::Request>();
    request->name = "athena_arm_interface";
    request->target_state.id = state_id;
    request->target_state.label = label;
    return hector_testing_utils::call_service<SetHardwareComponentState>(hw_state_client->get(), request, *executor_,
                                                                         options);
  };

  ASSERT_NE(transition(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive"), nullptr);
  std::this_thread::sleep_for(300ms);
  ASSERT_NE(transition(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured"), nullptr);
  std::this_thread::sleep_for(300ms);

  // Reconfigure with the motors still torqued, which is the warm start.
  transition(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  std::this_thread::sleep_for(500ms);

  EXPECT_EQ(hardwareComponentState("athena_arm_interface"), "inactive")
      << "Configuration failed because an EEPROM write was refused while the motor was energised, "
         "which leaves the arm live with no interface managing it.";
  EXPECT_EQ(motor->read2Byte(motor->getAddress("current_limit")), 22000)
      << "The configured current limit never reached EEPROM, so the motor is running with a limit "
         "nobody chose.";
}

// The indirect address pointers are torque-locked too, but unlike an initial value they are not
// retried de-energised: a layout rewrite touches every slot of every motor, and an arm joint without
// a brake falls while it is off. A warm start whose stored layout differs (a new branch, a changed
// interface list) must therefore fail configuration before writing anything, and say what to do.
// A de-energised motor takes the same layout without complaint.
TEST_F(HardwareInterfaceTest, WarmStart_ChangedIndirectLayoutFailsConfigureWithoutTouchingIt)
{
  std::this_thread::sleep_for(400ms);

  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_4_ID);
  ASSERT_NE(motor, nullptr);
  ASSERT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 1) << "Test premise: the motors must be energised";
  std::this_thread::sleep_for(300ms);

  auto hw_state_client =
      tester_node_->create_test_client<SetHardwareComponentState>("/controller_manager/set_hardware_component_state");
  ASSERT_TRUE(hw_state_client->wait_for_service(*executor_, 5s));
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 15s;
  options.response_timeout = 15s;
  const auto transition = [&](uint8_t state_id, const std::string& label) {
    auto request = std::make_shared<SetHardwareComponentState::Request>();
    request->name = "athena_arm_interface";
    request->target_state.id = state_id;
    request->target_state.label = label;
    return hector_testing_utils::call_service<SetHardwareComponentState>(hw_state_client->get(), request, *executor_,
                                                                         options);
  };

  ASSERT_NE(transition(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive"), nullptr);
  std::this_thread::sleep_for(300ms);
  ASSERT_NE(transition(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured"), nullptr);
  std::this_thread::sleep_for(300ms);
  ASSERT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 1) << "Test premise: still energised";

  // A layout from another configuration: the first pointer aims somewhere this one does not.
  constexpr uint16_t FIRST_POINTER = 168;
  const uint16_t configured_target = motor->read2Byte(FIRST_POINTER);
  const uint16_t foreign_target = motor->getAddress("led_green");
  ASSERT_NE(configured_target, foreign_target);
  motor->write2Byte(FIRST_POINTER, foreign_target);

  transition(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  std::this_thread::sleep_for(500ms);

  EXPECT_NE(hardwareComponentState("athena_arm_interface"), "inactive")
      << "Configuration went through with a layout the energised motor refuses to store.";
  EXPECT_EQ(motor->read2Byte(FIRST_POINTER), foreign_target)
      << "A pointer was written to an energised motor. The firmware refuses that, and the slots that "
         "did land would leave a mixed layout behind.";
  EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 1)
      << "The failed configuration de-energised the motor. An arm joint without a brake falls.";

  // De-energised, as after a power cycle, the same configuration writes the layout.
  for (const uint8_t id : ARM_MOTOR_IDS) {
    auto arm_motor = MockDynamixelManager::instance().getMotor(id);
    arm_motor->write1Byte(arm_motor->getAddress("torque_enable"), 0);
  }
  transition(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  std::this_thread::sleep_for(500ms);

  EXPECT_EQ(hardwareComponentState("athena_arm_interface"), "inactive");
  EXPECT_EQ(motor->read2Byte(FIRST_POINTER), configured_target);
}

// A joint with state interfaces and no command interfaces is a motor that is only ever read: a
// passive encoder, or one an operator has stopped commanding. The control write manager already
// skips it. The command read group has to skip it too, since a group needs one value per motor it
// was told about, and a motor with no goal register to read fails the group's own count check with
// a message that reads like an internal error rather than a configuration that is not supported.
class CommandlessJointTest : public HardwareInterfaceTest
{
protected:
  std::string transformUrdf(std::string urdf) override
  {
    const std::string anchor = R"(      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <command_interface name="max_current"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="gripper_joint_r1">)";
    const auto pos = urdf.find(anchor);
    if (pos == std::string::npos) {
      ADD_FAILURE() << "Could not find the gripper joint's command interfaces";
      return urdf;
    }
    const std::string replacement = R"(      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    <joint name="gripper_joint_r1">)";
    urdf.replace(pos, anchor.size(), replacement);
    return urdf;
  }
};

TEST_F(CommandlessJointTest, AJointThatTakesNoCommandsDoesNotBlockTheOthers)
{
  std::this_thread::sleep_for(500ms);

  EXPECT_EQ(hardwareComponentState("athena_arm_interface"), "active")
      << "A joint that declares no command interfaces kept the whole component from coming up, "
         "although nothing about it is unsupported: it is simply only ever read.";

  // The joints that do take commands must still be driven.
  for (uint8_t id : ARM_MOTOR_IDS) {
    if (id == GRIPPER_ID) {
      continue;
    }
    auto motor = MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 1)
        << "Motor " << static_cast<int>(id) << " should be energised";
  }
}

}  // namespace dynamixel_ros_control::test
