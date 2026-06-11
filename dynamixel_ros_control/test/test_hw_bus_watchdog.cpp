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

// ============================================================================
// bus_watchdog_cycles parameter tests
//
// The watchdog timeout is computed as
//   ticks = clamp(ceil(dt_ms * cycles / ms_per_tick), MIN_TICKS=1, MAX_TICKS=127)
// where dt_ms = 1000 / rw_rate. The test rig runs the arm interface at 50 Hz (dt_ms = 20) and the
// PH-series bus_watchdog register uses the ms_20 unit (ms_per_tick = 20), so on the arm motors the
// formula collapses to ticks = clamp(ceil(cycles), 1, 127). The default cycles is 4 (-> 4 ticks),
// which BusWatchdog_ConfiguredOnStartup already covers. These tests inject a non-default
// bus_watchdog_cycles hardware parameter into the arm interface and verify the override, the
// upper clamp, the disable behavior (cycles == 0 -> 0 ticks), and the invalid (< 0) fallback.
// ============================================================================

// Fixture that injects a configurable bus_watchdog_cycles parameter into the arm <hardware> block.
class BusWatchdogCyclesTest : public HardwareInterfaceTest
{
protected:
  // The literal text written into the URDF for <param name="bus_watchdog_cycles">. Set by each test
  // before SetUp() runs. A string (not a number) so the invalid "0"/"-1" cases can be exercised
  // exactly as a user would configure them.
  std::string bus_watchdog_cycles_param_ = "8";

  std::string transformUrdf(std::string urdf) override
  {
    // Anchor on the arm interface's unique port_name so only the arm <hardware> block is touched.
    const std::string anchor = "<param name=\"port_name\">/dev/ttyUSB_manipulator_arm</param>";
    const auto pos = urdf.find(anchor);
    if (pos == std::string::npos) {
      ADD_FAILURE() << "Could not find arm interface anchor to inject bus_watchdog_cycles";
      return urdf;
    }
    const std::string injected =
        anchor + "\n      <param name=\"bus_watchdog_cycles\">" + bus_watchdog_cycles_param_ + "</param>";
    urdf.replace(pos, anchor.size(), injected);
    return urdf;
  }

  // Convenience: assert every arm + gripper motor holds the expected watchdog tick value.
  void expectArmWatchdogTicks(uint8_t expected_ticks)
  {
    for (uint8_t id : ARM_MOTOR_IDS) {
      auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
      ASSERT_NE(motor, nullptr) << "Motor ID " << static_cast<int>(id) << " not found";
      EXPECT_EQ(motor->getBusWatchdog(), expected_ticks)
          << "Unexpected watchdog ticks for arm motor " << static_cast<int>(id);
    }
    auto gripper = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
    ASSERT_NE(gripper, nullptr) << "Gripper motor not found";
    EXPECT_EQ(gripper->getBusWatchdog(), expected_ticks) << "Unexpected watchdog ticks for gripper motor";
  }
};

// (1) A non-default bus_watchdog_cycles must override the default and change the computed ticks.
// cycles = 8 -> ceil(20 * 8 / 20) = 8 ticks, distinct from the default of 4.
TEST_F(BusWatchdogCyclesTest, OverridesDefaultMultiplier)
{
  // bus_watchdog_cycles_param_ defaults to "8" above; assert the resulting ticks.
  constexpr uint8_t EXPECTED_TICKS = 8;
  expectArmWatchdogTicks(EXPECTED_TICKS);
}

// A large value must be clamped to MAX_TICKS (127) rather than overflowing the register.
class BusWatchdogCyclesClampTest : public BusWatchdogCyclesTest
{
protected:
  void SetUp() override
  {
    bus_watchdog_cycles_param_ = "1000";  // ceil(1000) = 1000, clamped to 127.
    BusWatchdogCyclesTest::SetUp();
  }
};

TEST_F(BusWatchdogCyclesClampTest, ClampsToMaxTicks)
{
  constexpr uint8_t EXPECTED_TICKS = 127;  // DXL_BUS_WATCHDOG_MAX_TICKS
  expectArmWatchdogTicks(EXPECTED_TICKS);
}

// (2) A zero bus_watchdog_cycles disables the watchdog: the register is written as 0, NOT clamped
// up to MIN_TICKS and NOT replaced with the default. This is the documented way to turn the feature
// off.
class BusWatchdogCyclesZeroTest : public BusWatchdogCyclesTest
{
protected:
  void SetUp() override
  {
    bus_watchdog_cycles_param_ = "0";
    BusWatchdogCyclesTest::SetUp();
  }
};

TEST_F(BusWatchdogCyclesZeroTest, ZeroDisablesWatchdog)
{
  constexpr uint8_t DISABLED_TICKS = 0;  // 0 = watchdog disabled on Dynamixel hardware
  expectArmWatchdogTicks(DISABLED_TICKS);
}

// (2) A negative bus_watchdog_cycles is invalid and must fall back to the default.
class BusWatchdogCyclesNegativeTest : public BusWatchdogCyclesTest
{
protected:
  void SetUp() override
  {
    bus_watchdog_cycles_param_ = "-5";
    BusWatchdogCyclesTest::SetUp();
  }
};

TEST_F(BusWatchdogCyclesNegativeTest, NegativeFallsBackToDefault)
{
  constexpr uint8_t EXPECTED_DEFAULT_TICKS = 4;
  expectArmWatchdogTicks(EXPECTED_DEFAULT_TICKS);
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
