// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Bus Watchdog Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, BusWatchdog_ConfiguredOnStartup)
{
  // The hardware interface configures the bus watchdog during on_configure.
  // With update_rate=50 Hz, dt=20 ms, watchdog = 4 * 20 ms = 80 ms.
  // The bus_watchdog register uses ms_20 unit (20 ms per tick), so the
  // expected register value is 80 / 20 = 4 ticks.
  constexpr uint8_t EXPECTED_WATCHDOG_TICKS = 4;

  // Check all arm motors
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Motor ID " << static_cast<int>(id) << " not found";
    EXPECT_EQ(motor->getBusWatchdog(), EXPECTED_WATCHDOG_TICKS)
        << "Bus watchdog not correctly configured for arm motor " << static_cast<int>(id);
  }

  // Check gripper motor
  auto gripper = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  ASSERT_NE(gripper, nullptr) << "Gripper motor not found";
  EXPECT_EQ(gripper->getBusWatchdog(), EXPECTED_WATCHDOG_TICKS)
      << "Bus watchdog not correctly configured for gripper motor";

  // Check all flipper motors
  for (uint8_t id : FLIPPER_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Motor ID " << static_cast<int>(id) << " not found";
    EXPECT_EQ(motor->getBusWatchdog(), EXPECTED_WATCHDOG_TICKS)
        << "Bus watchdog not correctly configured for flipper motor " << static_cast<int>(id);
  }
}

TEST_F(HardwareInterfaceTest, BusWatchdog_RegisterAddressCorrect)
{
  // Verify that the bus watchdog register address is correctly loaded from the YAML.
  // PH series (model 2020) has bus_watchdog at address 546.
  constexpr uint16_t EXPECTED_BUS_WATCHDOG_ADDR = 546;

  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);

  uint16_t addr = motor->getAddress("bus_watchdog");
  EXPECT_EQ(addr, EXPECTED_BUS_WATCHDOG_ADDR) << "Bus watchdog register address mismatch";
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
