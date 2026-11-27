// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

/**
 * Activation must not energise a motor before its control mode is known and safe.
 *
 * A motor that survived the last shutdown keeps its operating mode. Until on_activate has read that
 * mode the cached mode is stale, and resetGoalState() falls back to the *limit* registers for the
 * velocity and current goals. In position mode those are profile limits. In velocity or current
 * mode they are commands. So the order in which activation reads the mode, writes goals and enables
 * torque decides whether a warm-started actuator holds station or runs away at its velocity limit.
 *
 * Every test here de-energises the motor before activating: torque-on is the branch that runs the
 * goal reset, and setTorqueForJoints() skips the whole sequence for a motor that is already on.
 * The arm interface is configured with torque_on_startup=true, so this branch is reachable.
 */
class WarmStartTest : public HardwareInterfaceTest
{
protected:
  /// @brief Drive a component to a state, failing the test if the transition is refused.
  void requireHardwareComponentState(const std::string& component_name, uint8_t state_id, const std::string& label)
  {
    ASSERT_TRUE(setHardwareComponentState(component_name, state_id, label))
        << "Could not drive " << component_name << " to '" << label << "'";
  }

  void deactivateArm()
  {
    requireHardwareComponentState("athena_arm_interface", lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                  "inactive");
  }
  void activateArm()
  {
    requireHardwareComponentState("athena_arm_interface", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  }

  /// Leave the motor the way a shutdown in `mode` does: that operating mode, de-energised.
  void warmStartIn(const std::shared_ptr<MockDynamixel>& motor, OperatingMode mode)
  {
    motor->write1Byte(motor->getAddress("torque_enable"), 0);
    motor->write1Byte(motor->getAddress("operating_mode"), static_cast<uint8_t>(mode));
    ASSERT_EQ(motor->read1Byte(motor->getAddress("operating_mode")), static_cast<uint8_t>(mode));
    ASSERT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 0);
  }

  /// Assert that `mode` was written before the motor was first energised during the traced window.
  void expectModeSetBeforeTorqueOn(const std::shared_ptr<MockDynamixel>& motor, OperatingMode mode,
                                   const std::string& what)
  {
    const int mode_index = motor->firstWriteIndex("operating_mode", static_cast<uint8_t>(mode));
    const int torque_on_index = motor->firstWriteIndex("torque_enable", 1);

    ASSERT_GE(mode_index, 0) << "Activation never switched the warm-started motor into " << what << ".";
    ASSERT_GE(torque_on_index, 0) << "Test premise: torque_on_startup=true and the motor started de-energised, so "
                                     "activation must energise it.";
    EXPECT_LT(mode_index, torque_on_index)
        << "Torque was enabled before the motor was switched into " << what
        << ". With the mode cache still claiming a position mode, resetGoalState() writes goal velocity = "
           "velocity_limit and goal current = current_limit, so energising first commands a motor that is "
           "physically in velocity or current mode to run at full speed or full current.";
  }
};

// arm_joint_2 declares no position_control_mode, so its preferred mode is plain POSITION.
TEST_F(WarmStartTest, ActivationSwitchesModeBeforeEnergisingVelocityWarmStart)
{
  std::this_thread::sleep_for(300ms);
  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_2_ID);
  ASSERT_NE(motor, nullptr);

  deactivateArm();
  warmStartIn(motor, OperatingMode::VELOCITY_CONTROL);

  motor->startWriteTrace();
  activateArm();
  motor->stopWriteTrace();

  expectModeSetBeforeTorqueOn(motor, OperatingMode::POSITION_CONTROL, "position mode");
  EXPECT_EQ(motor->read1Byte(motor->getAddress("operating_mode")), static_cast<uint8_t>(OperatingMode::POSITION_CONTROL))
      << "The motor should end activation in its configured position mode.";
}

// Same hazard through the current-goal branch: in current mode a nonzero goal current is torque
// straight into the joint, and the fallback goal is the motor's current *limit*.
TEST_F(WarmStartTest, ActivationSwitchesModeBeforeEnergisingCurrentWarmStart)
{
  std::this_thread::sleep_for(300ms);
  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_2_ID);
  ASSERT_NE(motor, nullptr);

  deactivateArm();
  warmStartIn(motor, OperatingMode::CURRENT_CONTROL);

  motor->startWriteTrace();
  activateArm();
  motor->stopWriteTrace();

  expectModeSetBeforeTorqueOn(motor, OperatingMode::POSITION_CONTROL, "position mode");
}

// A joint configured for extended_position must be brought to its own preferred mode, not to plain
// POSITION, and still before it is energised.
TEST_F(WarmStartTest, ActivationRestoresConfiguredExtendedPositionMode)
{
  std::this_thread::sleep_for(300ms);
  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);

  deactivateArm();
  warmStartIn(motor, OperatingMode::VELOCITY_CONTROL);

  motor->startWriteTrace();
  activateArm();
  motor->stopWriteTrace();

  expectModeSetBeforeTorqueOn(motor, OperatingMode::EXTENDED_POSITION_CONTROL, "extended position mode");
  EXPECT_EQ(motor->read1Byte(motor->getAddress("operating_mode")),
            static_cast<uint8_t>(OperatingMode::EXTENDED_POSITION_CONTROL))
      << "arm_joint_1 declares position_control_mode=extended_position; activation must restore that mode.";
}

// The reordering must not have bought mode safety by making every activation limp first: dropping
// torque on a holding, gravity-loaded joint that is already correctly configured is its own hazard.
TEST_F(WarmStartTest, ActivationDoesNotDropTorqueOnCorrectlyConfiguredJoint)
{
  std::this_thread::sleep_for(300ms);
  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_2_ID);
  ASSERT_NE(motor, nullptr);

  deactivateArm();
  // Exactly what a normal shutdown leaves behind: the configured mode, still energised.
  ASSERT_EQ(motor->read1Byte(motor->getAddress("operating_mode")),
            static_cast<uint8_t>(OperatingMode::POSITION_CONTROL));
  motor->write1Byte(motor->getAddress("torque_enable"), 1);

  motor->startWriteTrace();
  activateArm();
  motor->stopWriteTrace();

  EXPECT_EQ(motor->lastWriteIndex("operating_mode"), -1)
      << "A joint already in its configured mode should not have its operating mode rewritten at all.";
  EXPECT_EQ(motor->firstWriteIndex("torque_enable", 0), -1)
      << "Activation de-energised a joint that was already correctly configured and holding.";
  EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 1);
}

// An activation that finds every motor energised writes no torque: a torque on would write and
// verify each motor, and one verify miss would roll the whole arm back to off.
TEST_F(WarmStartTest, ActivationOfAnEnergisedArmWritesNoTorque)
{
  std::this_thread::sleep_for(300ms);
  deactivateArm();

  std::vector<std::shared_ptr<MockDynamixel>> motors;
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    ASSERT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 1)
        << "Test premise: the arm keeps its torque on deactivation (torque_off_on_shutdown=false).";
    motor->startWriteTrace();
    motors.push_back(motor);
  }
  activateArm();

  for (const auto& motor : motors) {
    motor->stopWriteTrace();
    EXPECT_EQ(motor->lastWriteIndex("torque_enable"), -1)
        << "Activation wrote torque_enable on motor " << static_cast<int>(motor->getId())
        << " although it was already energised.";
  }
}

}  // namespace dynamixel_ros_control::test
