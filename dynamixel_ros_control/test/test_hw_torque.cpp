// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Torque Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, Torque_DisableTorqueChangesLEDToGreen)
{
  // 1. Verify initial state (torque on, LED blue)
  std::this_thread::sleep_for(200ms);
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "Initial LED should be blue";
    }
  }

  // 2. Claim torque_off for the whole arm
  EXPECT_TRUE(limp()) << "Torque disable should succeed";

  // 3. Wait for LED update and verify LED is green
  std::this_thread::sleep_for(500ms);

  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      EXPECT_EQ(motor->getLedRed(), COLOR_GREEN_R) << "Motor " << (int) id << " LED should be green (R)";
      EXPECT_EQ(motor->getLedGreen(), COLOR_GREEN_G) << "Motor " << (int) id << " LED should be green (G)";
      EXPECT_EQ(motor->getLedBlue(), COLOR_GREEN_B) << "Motor " << (int) id << " LED should be green (B)";
    }
  }

  // 4. Verify torque is actually disabled in motors
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      uint16_t torque_addr = motor->getAddress("torque_enable");
      EXPECT_EQ(motor->read1Byte(torque_addr), 0) << "Motor " << (int) id << " torque should be disabled";
    }
  }
}

TEST_F(HardwareInterfaceTest, Torque_EnableTorqueChangesLEDToBlue)
{
  // 1. First disable torque
  ASSERT_TRUE(limp());

  std::this_thread::sleep_for(300ms);

  // Verify LED is green (torque off)
  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  EXPECT_EQ(motor->getLedGreen(), COLOR_GREEN_G) << "LED should be green when torque is off";

  // 2. Enable torque
  EXPECT_TRUE(releaseLimp());

  std::this_thread::sleep_for(300ms);

  // 3. Verify LED is blue
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      EXPECT_EQ(motor->getLedRed(), COLOR_BLUE_R) << "Motor " << (int) id << " LED should be blue (R)";
      EXPECT_EQ(motor->getLedGreen(), COLOR_BLUE_G) << "Motor " << (int) id << " LED should be blue (G)";
      EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "Motor " << (int) id << " LED should be blue (B)";
    }
  }

  // 4. Verify torque is enabled in motors
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      uint16_t torque_addr = motor->getAddress("torque_enable");
      EXPECT_EQ(motor->read1Byte(torque_addr), 1) << "Motor " << (int) id << " torque should be enabled";
    }
  }
}

TEST_F(HardwareInterfaceTest, Torque_CommandsNotExecutedWhenTorqueOff)
{
  // 1. Disable torque
  ASSERT_TRUE(limp());

  std::this_thread::sleep_for(300ms);

  // 2. Record initial positions
  std::vector<double> initial_positions;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions.push_back(motor->getCurrentPosition());
  }

  // 3. A joint held limp cannot be commanded, so the position controller is refused
  EXPECT_FALSE(loadAndActivateController("arm_position_controller", 3s));
  std::this_thread::sleep_for(1s);

  // 5. Verify motors didn't move (torque is off, physics simulation shouldn't move them)
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    // With torque off, motors should not move toward goal
    EXPECT_NEAR(motor->getCurrentPosition(), initial_positions[i], 0.01)
        << "Motor " << (ARM_JOINT_1_ID + i) << " should not have moved with torque off";
  }
}

TEST_F(HardwareInterfaceTest, Torque_OnStartupVerification)
{
  // Verify that the torque_on_startup parameter works as expected.
  // The test URDF has torque_on_startup: true, so torque should be enabled on activation.
  // This test verifies the initial state set up by the test fixture.

  // Hardware interface is active and torque should be on (torque_on_startup: true in URDF)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Motor " << (int) id << " should exist";
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 1)
        << "Motor " << (int) id << " torque should be ON due to torque_on_startup: true";
  }

  // LED should be blue (torque on, active state)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B)
        << "Motor " << (int) id << " LED should be blue when torque is on at startup";
  }
}

TEST_F(HardwareInterfaceTest, Torque_ReEnableRefusesAStaleSmallerGoalCurrent)
{
  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_4_ID);
  ASSERT_NE(motor, nullptr);
  ASSERT_TRUE(limp()) << "Precondition: torque must go off";
  std::this_thread::sleep_for(300ms);

  // 5 A of the configured 22 A, left behind by something else, and the reset write does not land.
  motor->write2Byte(motor->getAddress("goal_current"), 5000);
  motor->setRegisterWriteFault("goal_current", RegisterWriteFault::DROP);

  EXPECT_FALSE(releaseLimp())
      << "A goal current smaller than the one written was accepted as if the motor had clamped it.";
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 0)
      << "arm_joint_4 must not be energised on a goal current that was never verified";
}

// The same failure with nothing left behind: a goal current that did not reach the register reads
// back as 0. A latched bus watchdog, wiped indirect pointers after a reboot and a dropped packet all
// produce this, and energising a joint whose torque limit did not land leaves it unable to hold.
TEST_F(HardwareInterfaceTest, Torque_ReEnableRefusesWhenTheGoalCurrentNeverLands)
{
  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_4_ID);
  ASSERT_NE(motor, nullptr);
  ASSERT_TRUE(limp()) << "Precondition: torque must go off";
  std::this_thread::sleep_for(300ms);

  // Clear the register first: a dropped write leaves whatever was there before, and the previous
  // activation already wrote the correct goal. That would verify legitimately and prove nothing.
  // Zero is what the register reads after a reboot wipes RAM or when it was not written.
  motor->write2Byte(motor->getAddress("goal_current"), 0);
  // From here the motor acknowledges the write and does not apply it. That is the silent rejection
  // a latched watchdog performs on every goal-area write.
  motor->setRegisterWriteFault("goal_current", RegisterWriteFault::DROP);

  EXPECT_FALSE(releaseLimp())
      << "A goal current that never landed must abort the torque-on, not pass verification as if "
         "the motor had clamped it.";
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 0)
      << "arm_joint_4 must not be energised when its goal current could not be written";
}

}  // namespace dynamixel_ros_control::test
