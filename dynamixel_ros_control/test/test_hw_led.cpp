// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// LED Color Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, LED_BlueWhenActiveAndTorqueOn)
{
  // Hardware interface is active with torque on by default (torque_on_startup: true)
  // LED should be blue
  std::this_thread::sleep_for(500ms);  // Allow time for LED update

  // Check arm motors (hardware interface athena_arm_interface)
  for (uint8_t id : ARM_MOTOR_IDS) {
    verifyLEDColor(id, COLOR_BLUE_R, COLOR_BLUE_G, COLOR_BLUE_B);
  }
}

TEST_F(HardwareInterfaceTest, LED_PinkWhenHardwareInterfaceInactive)
{
  // Test that LED is pink when hardware interface is deactivated (inactive state)
  // Pink indicates the hardware interface is not active (safe but not operational)

  // 1. Verify initial state. The LED should be blue (active, torque on)
  std::this_thread::sleep_for(300ms);
  for (uint8_t id : ARM_MOTOR_IDS) {
    verifyLEDColor(id, COLOR_BLUE_R, COLOR_BLUE_G, COLOR_BLUE_B);
  }

  // 2. Deactivate the hardware interface
  EXPECT_TRUE(
      setHardwareComponentState("athena_arm_interface", lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive"))
      << "Hardware interface deactivation should succeed";

  // 3. Wait for LED update
  std::this_thread::sleep_for(500ms);

  // 4. Verify LED is pink (inactive state)
  for (uint8_t id : ARM_MOTOR_IDS) {
    verifyLEDColor(id, COLOR_PINK_R, COLOR_PINK_G, COLOR_PINK_B);
  }
}

TEST_F(HardwareInterfaceTest, LED_BlueAfterReactivation)
{
  // Test that LED returns to blue after hardware interface is reactivated

  // 1. Deactivate the hardware interface
  ASSERT_TRUE(
      setHardwareComponentState("athena_arm_interface", lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive"))
      << "Deactivation should succeed";

  std::this_thread::sleep_for(300ms);

  // Verify LED is pink
  verifyLEDColor(ARM_JOINT_1_ID, COLOR_PINK_R, COLOR_PINK_G, COLOR_PINK_B);

  // 2. Reactivate the hardware interface
  EXPECT_TRUE(
      setHardwareComponentState("athena_arm_interface", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active"))
      << "Reactivation should succeed";

  std::this_thread::sleep_for(500ms);

  // 3. Verify LED is blue (active state with torque on, torque_on_startup: true)
  for (uint8_t id : ARM_MOTOR_IDS) {
    verifyLEDColor(id, COLOR_BLUE_R, COLOR_BLUE_G, COLOR_BLUE_B);
  }
}

}  // namespace dynamixel_ros_control::test
