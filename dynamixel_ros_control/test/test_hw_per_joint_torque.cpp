// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// Torque state is per joint, so a partially-torqued arm is representable: safety paths must not
// stand down while any motor is still live, and a reboot must restore each joint's own intent.

namespace {
constexpr const char* GRIPPER_JOINT = "gripper_servo_joint";

bool motorTorqued(uint8_t id)
{
  auto motor = MockDynamixelManager::instance().getMotor(id);
  EXPECT_NE(motor, nullptr) << "Motor " << static_cast<int>(id) << " missing";
  return motor != nullptr && motor->read1Byte(motor->getAddress("torque_enable")) == 1;
}
}  // namespace

// A partial torque-off must survive a reboot: the joints the operator switched off stay off, and
// the joint they deliberately kept on stays on.
TEST_F(HardwareInterfaceTest, PartialTorqueOff_SurvivesReboot)
{
  std::this_thread::sleep_for(300ms);

  ASSERT_TRUE(limp("arm_joints_torque_off_controller"));
  std::this_thread::sleep_for(300ms);
  ASSERT_TRUE(motorTorqued(GRIPPER_ID)) << "Test premise: the gripper must still be torqued";

  // Provoke a reboot of one of the switched-off arm motors.
  constexpr uint8_t REBOOTED_ID = ARM_JOINT_2_ID;
  auto rebooted = MockDynamixelManager::instance().getMotor(REBOOTED_ID);
  ASSERT_NE(rebooted, nullptr);
  rebooted->setHardwareError(ERROR_OVERHEATING);
  ASSERT_TRUE(waitForComponentState("athena_arm_interface", "unconfigured"));

  auto resp = rebootMotors();
  ASSERT_NE(resp, nullptr);
  std::this_thread::sleep_for(600ms);

  // The operator's partial state must be intact on both sides.
  for (uint8_t id : ARM_MOTOR_IDS) {
    EXPECT_FALSE(motorTorqued(id)) << "Arm motor " << static_cast<int>(id)
                                   << " was re-energised by a reboot even though the operator had switched it off.";
  }
  EXPECT_TRUE(motorTorqued(GRIPPER_ID)) << "The gripper the operator deliberately kept torqued was switched off";

  rebooted->setHardwareError(0);
}

// ignore_joints exists for the joint an operator cannot or must not command, so the goal reset that
// precedes a torque-on has to address only the joints being energised. While it read and wrote the
// whole bus, an excluded joint whose goal registers could not be read back failed the reset for the
// healthy joints too: naming it changed nothing about whether the rest could be brought up.
//
// The fault is scoped to the goal register rather than the whole motor because a motor that answers
// nothing takes the shared state read down regardless, and the component with it. No per-joint
// request can rescue that.
TEST_F(HardwareInterfaceTest, TorqueOnSucceedsWhileAnUnclaimedJointsGoalsAreUnreadable)
{
  std::this_thread::sleep_for(300ms);
  ASSERT_TRUE(limp("arm_joints_torque_off_controller")) << "Precondition: the arm must start de-energised";
  std::this_thread::sleep_for(300ms);

  auto excluded = MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  ASSERT_NE(excluded, nullptr);
  excluded->setRegisterReadFault("goal_position");

  EXPECT_TRUE(releaseLimp("arm_joints_torque_off_controller"))
      << "A torque-on that excluded the unreadable joint was still refused, so the joints the "
         "operator asked for stayed de-energised because of a motor they had deliberately left out.";
  std::this_thread::sleep_for(300ms);

  for (uint8_t id : ARM_MOTOR_IDS) {
    EXPECT_TRUE(motorTorqued(id)) << "Motor " << static_cast<int>(id) << " should be energised";
  }
}

// Ending a torque_off claim re-arms the watchdog and proves it on the energised motors only. A
// proof over the whole bus lets a limp joint whose watchdog cannot be read fail the torque-on, and
// the rollback then leaves the arm limp because of a motor nobody energised.
TEST_F(HardwareInterfaceTest, TorqueOnAfterTorqueOffSucceedsWhileALimpJointsWatchdogIsUnreadable)
{
  std::this_thread::sleep_for(300ms);
  ASSERT_TRUE(limp("arm_joints_torque_off_controller")) << "Precondition: the arm must start de-energised";
  ASSERT_TRUE(limp("gripper_torque_off_controller")) << "Precondition: the gripper must stay de-energised";
  std::this_thread::sleep_for(300ms);

  auto limp_motor = MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  ASSERT_NE(limp_motor, nullptr);
  limp_motor->setRegisterReadFault("bus_watchdog");

  EXPECT_TRUE(releaseLimp("arm_joints_torque_off_controller"))
      << "The torque-on was rolled back because of the watchdog of a joint it left limp";
  std::this_thread::sleep_for(300ms);

  for (uint8_t id : ARM_MOTOR_IDS) {
    EXPECT_TRUE(motorTorqued(id)) << "Motor " << static_cast<int>(id) << " should be energised";
  }
}

}  // namespace dynamixel_ros_control::test
