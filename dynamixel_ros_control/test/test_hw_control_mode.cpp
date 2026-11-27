// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

#include <dynamixel_ros_control/common.hpp>
#include <dynamixel_ros_control/dynamixel.hpp>

#include <set>

namespace dynamixel_ros_control::test {

// ============================================================================
// Control-mode cache coherence
// ============================================================================

// Joint::updateControlMode() must not commit control_mode_ until the operating-mode write lands.
// resetGoalState() derives its goals from that cache: for the velocity interface it writes 0 only
// while the cached mode is VELOCITY, and otherwise falls back to default_goal_values_[velocity],
// which is the motor's *velocity limit*. A joint left in velocity mode whose cache claims POSITION
// would therefore be commanded to full speed by the emergency stop meant to halt it.
//
// The default athena.urdf leaves velocity_limit at 0, which would make both branches write the same
// value and this test vacuous, so the fixture gives the arm motors a real limit.
class ControlModeVelocityLimitTest : public HardwareInterfaceTest
{
protected:
  // rad/s, converted to raw ticks by the initial-value write. Large enough that the resulting goal
  // velocity is unmistakably non-zero if the buggy branch is taken.
  static constexpr const char* ARM_VELOCITY_LIMIT = "3.0";

  std::string transformUrdf(std::string urdf) override
  {
    for (uint8_t id : ARM_MOTOR_IDS) {
      const std::string anchor = "<param name=\"id\">" + std::to_string(static_cast<int>(id)) + "</param>";
      const auto pos = urdf.find(anchor);
      if (pos == std::string::npos) {
        ADD_FAILURE() << "Could not find id anchor for motor " << static_cast<int>(id);
        return urdf;
      }
      const std::string injected =
          anchor + "\n      <param name=\"registers.velocity_limit\">" + ARM_VELOCITY_LIMIT + "</param>";
      urdf.replace(pos, anchor.size(), injected);
    }
    return urdf;
  }
};

// Guard against the test above silently becoming vacuous again: if velocity_limit is ever 0, the
// buggy and fixed branches write the same value and the assertion proves nothing.
TEST_F(ControlModeVelocityLimitTest, VelocityLimitIsNonZero)
{
  std::this_thread::sleep_for(200ms);
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_GT(motor->read4ByteSigned(motor->getAddress("velocity_limit")), 0)
        << "Motor " << static_cast<int>(id)
        << " has a zero velocity limit, so ModeSwitchFailure_DoesNotCommandVelocityLimit cannot "
           "distinguish the safe branch from the unsafe one.";
  }
}

TEST_F(ControlModeVelocityLimitTest, ModeSwitchFailure_DoesNotCommandVelocityLimit)
{
  std::this_thread::sleep_for(300ms);

  // 1. Put the arm into velocity mode for real.
  ASSERT_TRUE(loadAndActivateController("arm_velocity_controller"));
  std::this_thread::sleep_for(200ms);

  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    ASSERT_EQ(motor->read1Byte(motor->getAddress("operating_mode")),
              static_cast<uint8_t>(OperatingMode::VELOCITY_CONTROL))
        << "Motor " << static_cast<int>(id) << " is not in velocity mode; the test premise does not hold";
  }

  // 2. Make the operating-mode register unwritable. The motor stays otherwise healthy, so this
  //    isolates a failed mode switch from a dead motor.
  for (uint8_t id : ARM_MOTOR_IDS) {
    MockDynamixelManager::instance().getMotor(id)->setRegisterWriteFault("operating_mode", RegisterWriteFault::TIMEOUT);
  }

  // 3. Ask for position control. perform_command_mode_switch cannot write the mode, returns ERROR,
  //    the next write() returns ERROR, and on_error secures the component.
  ASSERT_TRUE(loadController("arm_position_controller"));
  switch_controllers({"arm_position_controller"}, {"arm_velocity_controller"});
  std::this_thread::sleep_for(700ms);  // let the failed switch and securing settle

  // 4. Every arm motor is still physically in velocity mode. Its goal velocity must be zero.
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(motor->read1Byte(motor->getAddress("operating_mode")),
              static_cast<uint8_t>(OperatingMode::VELOCITY_CONTROL))
        << "Motor " << static_cast<int>(id) << " changed mode even though the write was faulted";

    const int32_t goal_velocity_ticks = motor->read4ByteSigned(motor->getAddress("goal_velocity"));
    EXPECT_EQ(goal_velocity_ticks, 0)
        << "Motor " << static_cast<int>(id) << " was commanded to a non-zero goal velocity (" << goal_velocity_ticks
        << " ticks) after a failed mode switch. A stale control-mode cache made resetGoalState() "
        << "write the velocity limit to a motor still in velocity mode.";
  }

  // 5. The physical consequence: nothing may be accelerating.
  std::this_thread::sleep_for(300ms);
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    EXPECT_NEAR(motor->getCurrentVelocity(), 0.0, 0.05)
        << "Motor " << static_cast<int>(id) << " is still moving after the emergency stop";
  }

  for (uint8_t id : ARM_MOTOR_IDS) {
    MockDynamixelManager::instance().getMotor(id)->clearRegisterWriteFaults();
  }
}

// A mode switch that succeeds must still update the cache. Without this test, an implementation
// that ignores the cache entirely would "pass" the one above.
TEST_F(HardwareInterfaceTest, ModeSwitchSuccess_UpdatesCache)
{
  std::this_thread::sleep_for(300ms);

  ASSERT_TRUE(loadAndActivateController("arm_velocity_controller"));
  std::this_thread::sleep_for(200ms);

  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(motor->read1Byte(motor->getAddress("operating_mode")),
              static_cast<uint8_t>(OperatingMode::VELOCITY_CONTROL))
        << "Motor " << static_cast<int>(id) << " did not switch to velocity mode";
  }

  // Switching back must land. Each joint returns to *its own* configured preferred position mode:
  // the URDF gives 11/13/15/17 extended_position, the rest default to plain position.
  ASSERT_TRUE(loadController("arm_position_controller"));
  ASSERT_TRUE(switch_controllers({"arm_position_controller"}, {"arm_velocity_controller"}));
  std::this_thread::sleep_for(400ms);

  const std::set<uint8_t> extended_position_ids = {ARM_JOINT_1_ID, ARM_JOINT_3_ID, ARM_JOINT_5_ID, ARM_JOINT_7_ID};
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    const auto expected = extended_position_ids.count(id) != 0 ? OperatingMode::EXTENDED_POSITION_CONTROL :
                                                                 OperatingMode::POSITION_CONTROL;
    EXPECT_EQ(motor->read1Byte(motor->getAddress("operating_mode")), static_cast<uint8_t>(expected))
        << "Motor " << static_cast<int>(id) << " did not switch back to its preferred position mode";
  }
}

// The stock controller manager's default switch (no activate_asap), as the spawner and the
// orchestrator send it: perform runs on the service thread beside read() and write(). The switch
// still lands, and the overlap costs skipped cycles, not a fault.
TEST_F(HardwareInterfaceTest, ADefaultSwitchBesideTheControlLoopLands)
{
  std::this_thread::sleep_for(300ms);
  ASSERT_TRUE(loadController("arm_velocity_controller"));
  auto request = switchRequest();
  request->activate_asap = false;
  request->activate_controllers = {"arm_velocity_controller"};
  const auto response = callService<SwitchController>(switch_client_, request, 10s);
  ASSERT_TRUE(response && response->ok);
  std::this_thread::sleep_for(200ms);

  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->read1Byte(motor->getAddress("operating_mode")),
              static_cast<uint8_t>(OperatingMode::VELOCITY_CONTROL))
        << "Motor " << static_cast<int>(id) << " did not switch to velocity mode";
  }
  EXPECT_EQ(hardwareComponentState("athena_arm_interface"), "active") << "the overlap faulted the arm";
}

// ============================================================================
// A dropped status packet during activation
// ============================================================================
//
// Every other transaction on this bus retries a lost status packet. The control-mode read did not,
// so one dropped packet failed the activation and left a warm-started motor energised with the
// watchdog already cleared, waiting for a full configure-and-activate round to recover.
class ControlModeReadDropsOnceTest : public HardwareInterfaceTest
{
protected:
  void beforeControllerManager() override
  {
    auto& manager = MockDynamixelManager::instance();
    manager.addMotor(ARM_JOINT_3_ID, DEFAULT_MOCK_MODEL_NUMBER);
    // Two: the diagnostics manifest reads this register once during configuration, and the miss
    // under test is the one activation's own read then hits. A single miss on a shared bus is
    // ordinary, which is why every other transaction here retries it.
    manager.getMotor(ARM_JOINT_3_ID)->setRegisterReadFaultCount(DXL_REGISTER_CONTROL_MODE, 2);
  }
};

TEST_F(ControlModeReadDropsOnceTest, OneLostControlModeReadDoesNotFailActivation)
{
  EXPECT_EQ(hardwareComponentState("athena_arm_interface"), "active")
      << "A single dropped control-mode status packet aborted the activation.";
}

}  // namespace dynamixel_ros_control::test
