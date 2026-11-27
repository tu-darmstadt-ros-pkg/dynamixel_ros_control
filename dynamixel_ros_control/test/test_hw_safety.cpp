// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Critical Safety Tests: Torque Enable Must Fail If Goal Write Fails
// ============================================================================

TEST_F(HardwareInterfaceTest, Safety_TorqueEnableFailsWhenGoalWriteFails)
{
  // CRITICAL SAFETY TEST: When enabling torque, the system must first write
  // the current position as goal position. If this write fails, torque MUST
  // NOT be enabled to prevent sudden, uncontrolled movement.

  // 1. Activate position controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
  std::this_thread::sleep_for(300ms);

  // 2. Record initial motor positions
  std::map<uint8_t, double> initial_positions;
  std::vector<uint8_t> arm_ids = {ARM_JOINT_1_ID, ARM_JOINT_2_ID, ARM_JOINT_3_ID, ARM_JOINT_4_ID,
                                  ARM_JOINT_5_ID, ARM_JOINT_6_ID, ARM_JOINT_7_ID};
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions[id] = motor->getCurrentPosition();
  }

  // 3. Disable torque
  ASSERT_TRUE(limp()) << "Should be able to disable torque";

  std::this_thread::sleep_for(300ms);

  // Verify torque is off
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    ASSERT_EQ(motor->read1Byte(torque_addr), 0) << "Motor " << (int) id << " torque should be disabled";
  }

  // 4. Inject communication error on motor 1 to make goal position write/verify fail
  // This simulates the scenario where the hardware interface cannot properly write/verify
  // goal positions before enabling torque, a dangerous situation if allowed to proceed
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  motor1->setCommunicationFault(dynamixel_ros_control::CommunicationFault::DISCONNECTED);

  // 5. Attempt to enable torque. This must fail
  // The resetGoalStateAndVerify() function requires successful read/write/verify cycle
  EXPECT_FALSE(releaseLimp()) << "Torque enable MUST fail when goal position write fails!";

  std::this_thread::sleep_for(300ms);

  // 6. CRITICAL CHECK: Verify torque is still OFF on all motors
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 0)
        << "CRITICAL: Motor " << (int) id << " torque MUST remain disabled when goal write failed! "
        << "Enabling torque without proper goal position could cause dangerous, sudden movement!";
  }

  motor1->setCommunicationFault(dynamixel_ros_control::CommunicationFault::NONE);
}

TEST_F(HardwareInterfaceTest, Safety_NoMovementOnFailedTorqueEnable)
{
  // Test that even when torque enable fails, no movement occurs during the failed attempt

  // 1. Set up controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  waitForSubscribers(pub, 5s);

  // Move to known position
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // 2. Disable torque
  ASSERT_TRUE(limp());
  std::this_thread::sleep_for(300ms);

  // 3. Record positions after torque off
  std::map<uint8_t, double> positions_after_torque_off;
  std::vector<uint8_t> arm_ids = {ARM_JOINT_1_ID, ARM_JOINT_2_ID, ARM_JOINT_3_ID, ARM_JOINT_4_ID,
                                  ARM_JOINT_5_ID, ARM_JOINT_6_ID, ARM_JOINT_7_ID};
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_after_torque_off[id] = motor->getCurrentPosition();
  }

  // 4. Inject communication error on multiple motors
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    motor->setCommunicationFault(dynamixel_ros_control::CommunicationFault::DISCONNECTED);
  }

  // 5. Attempt torque enable (should fail)
  EXPECT_FALSE(releaseLimp()) << "Torque enable should fail with communication errors";

  std::this_thread::sleep_for(500ms);

  // 6. Verify NO movement occurred during failed torque enable attempt
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double position_change = std::abs(motor->getCurrentPosition() - positions_after_torque_off[id]);
    EXPECT_LT(position_change, 0.05) << "Motor " << (int) id << " should NOT have moved during failed torque enable";
  }

  // Cleanup
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    motor->setCommunicationFault(dynamixel_ros_control::CommunicationFault::NONE);
  }
}

}  // namespace dynamixel_ros_control::test
