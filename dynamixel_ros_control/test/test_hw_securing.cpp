#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {

/// Securing runs in on_error, when the component is given up: it holds what can hold, commands zero
/// where it cannot, and turns torque off where neither is proven. Driven directly, so no control
/// loop spends the faults a test injects before on_error sees them.
class SecuringTest : public DirectHardwareTest
{
protected:
  static std::shared_ptr<MockDynamixel> motor(uint8_t id)
  {
    auto m = MockDynamixelManager::instance().getMotor(id);
    EXPECT_NE(m, nullptr) << "Motor " << static_cast<int>(id) << " missing";
    return m;
  }

  static bool torqued(uint8_t id)
  {
    auto m = motor(id);
    return m != nullptr && m->read1Byte(m->getAddress("torque_enable")) == 1;
  }

  /// A hardware error on @p id, run until read() reports it, so on_error takes the securing branch.
  void fault(uint8_t id)
  {
    motor(id)->setHardwareError(ERROR_OVERHEATING);
    for (int i = 0; i < 10; ++i) {
      if (cycle() != hardware_interface::return_type::OK) {
        return;
      }
    }
    FAIL() << "read() never reported the hardware error of motor " << static_cast<int>(id);
  }

  void handleError()
  {
    ASSERT_EQ(hardware_->on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  }
};

// A motor whose torque read-back fails is not holding. It is unproven, so it gets the fallback
// torque-off. Counting it as holding would report an arm secured while a gravity-loaded joint may
// hang limp.
TEST_F(SecuringTest, TorqueThatCannotBeReadBackIsNotReportedAsHolding)
{
  startComponent("athena_arm_interface");
  fault(ARM_JOINT_7_ID);
  auto unprovable = motor(ARM_JOINT_2_ID);
  unprovable->setRegisterReadFault("torque_enable");

  unprovable->startWriteTrace();
  handleError();
  unprovable->stopWriteTrace();

  EXPECT_GE(unprovable->lastWriteIndex("torque_enable", 0), 0)
      << "A joint whose torque could not be read back was counted as holding.";
}

// Securing must not force torque onto a joint that was de-energised when it began.
TEST_F(SecuringTest, SecuringNeverEnergisesAJointThatWasTorqueOff)
{
  startComponent("athena_arm_interface");
  std::vector<std::string> torque_off;
  for (const auto* joint : {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5", "arm_joint_6",
                            "arm_joint_7", "gripper_servo_joint"}) {
    torque_off.push_back(std::string(joint) + "/torque_off");
  }
  ASSERT_EQ(hardware_->perform_command_mode_switch(torque_off, {}), hardware_interface::return_type::OK);
  cycles(5);
  for (uint8_t id : ARM_MOTOR_IDS) {
    ASSERT_FALSE(torqued(id)) << "Test premise: motor " << static_cast<int>(id) << " must start de-energised";
  }

  fault(ARM_JOINT_7_ID);
  handleError();

  for (uint8_t id : ARM_MOTOR_IDS) {
    EXPECT_FALSE(torqued(id)) << "Securing energised motor " << static_cast<int>(id)
                              << ", which was de-energised when it began.";
  }
}

// Torque off is the price of a mode write, not a step of securing. A joint already in its hold mode
// owes nothing, and de-energising it drops it for the length of the pipeline.
TEST_F(SecuringTest, AJointAlreadyInItsHoldModeIsNeverDeEnergised)
{
  startComponent("athena_arm_interface");
  ASSERT_TRUE(torqued(ARM_JOINT_2_ID)) << "Test premise: the joint must start energised";
  fault(ARM_JOINT_7_ID);
  auto healthy = motor(ARM_JOINT_2_ID);

  healthy->startWriteTrace();
  handleError();
  healthy->stopWriteTrace();

  ASSERT_EQ(healthy->lastWriteIndex("operating_mode"), -1)
      << "Test premise: this joint is already in its hold mode, so securing must not rewrite it.";
  EXPECT_EQ(healthy->firstWriteIndex("torque_enable", 0), -1)
      << "The joint was de-energised although no mode write followed.";
  EXPECT_TRUE(torqued(ARM_JOINT_2_ID)) << "The joint must still be holding once securing is done";
}

// A dropped status packet is ordinary on a faulted bus. Without a retry, one lost packet routes a
// holding joint into the torque-off fallback. readWhoeverAnswers() issues a group read and then
// probes each motor, so failing one observation of one motor costs two transactions.
TEST_F(SecuringTest, ATransientTorqueReadbackMissDoesNotDeEnergiseAHoldingJoint)
{
  startComponent("athena_arm_interface");
  ASSERT_TRUE(torqued(ARM_JOINT_2_ID)) << "Test premise: the joint must start energised";
  fault(ARM_JOINT_7_ID);

  motor(ARM_JOINT_2_ID)->setRegisterReadFaultCount("torque_enable", 2);
  handleError();

  EXPECT_TRUE(torqued(ARM_JOINT_2_ID)) << "A joint that missed one torque read-back was de-energised.";
}

// The same for the goal read-back that proves the hold landed.
TEST_F(SecuringTest, ATransientGoalReadbackMissDoesNotDeEnergiseAHoldingJoint)
{
  startComponent("athena_arm_interface");
  ASSERT_TRUE(torqued(ARM_JOINT_2_ID)) << "Test premise: the joint must start energised";
  fault(ARM_JOINT_7_ID);

  motor(ARM_JOINT_2_ID)->setRegisterReadFaultCount("goal_position", 2);
  handleError();

  EXPECT_TRUE(torqued(ARM_JOINT_2_ID)) << "A joint whose hold goal missed one verification read was de-energised.";
}

// The one path where securing de-energises a joint on purpose: it is in the wrong mode to hold, and
// the mode register takes writes only with torque off. On the retry the mode is already right and
// the torque reads Off because securing switched it off, and the joint must still be held again.
TEST_F(SecuringTest, AJointDeEnergisedForItsModeSwitchIsHeldAgainAfterARetry)
{
  startComponent("athena_arm_interface");
  auto gripper = motor(GRIPPER_ID);
  ASSERT_TRUE(torqued(GRIPPER_ID)) << "Test premise: the gripper must start energised";
  fault(ARM_JOINT_7_ID);

  // Velocity mode, moved under the interface: securing has to switch it back to hold
  gripper->write1Byte(gripper->getAddress("operating_mode"), 1);
  ASSERT_EQ(gripper->read1Byte(gripper->getAddress("operating_mode")), 1) << "Test premise: mode moved";
  gripper->setRegisterReadFaultCount("goal_position", 2);
  handleError();

  EXPECT_NE(gripper->read1Byte(gripper->getAddress("operating_mode")), 1)
      << "Test premise: securing should have switched the joint into a position mode";
  EXPECT_TRUE(torqued(GRIPPER_ID)) << "The joint securing de-energised for its own mode switch was never re-energised.";
}

// A joint proven limp for its mode switch gets no second torque-off, since on the degraded bus
// securing runs over every addressed transaction may cost the hold
TEST_F(SecuringTest, AJointProvenLimpForItsModeSwitchGetsOneTorqueOff)
{
  startComponent("athena_arm_interface");
  auto gripper = motor(GRIPPER_ID);
  fault(ARM_JOINT_7_ID);
  gripper->write1Byte(gripper->getAddress("operating_mode"), 1);
  gripper->startWriteTrace();
  handleError();
  gripper->stopWriteTrace();

  ASSERT_NE(gripper->read1Byte(gripper->getAddress("operating_mode")), 1)
      << "precondition: securing switched the joint into a position mode";
  ASSERT_GE(gripper->firstWriteIndex("torque_enable", 0), 0) << "precondition: securing de-energised the joint";
  EXPECT_EQ(gripper->firstWriteIndex("torque_enable", 0), gripper->lastWriteIndex("torque_enable", 0))
      << "the joint was sent a second torque-off";
  EXPECT_TRUE(torqued(GRIPPER_ID));
}

// A latched watchdog discards every goal write until a 0 clears it, so securing clears it first.
// Otherwise its hold goals read back stale and joints that could hold get the torque-off.
TEST_F(SecuringTest, SecuringClearsALatchedWatchdogSoItsHoldGoalsLand)
{
  startComponent("athena_arm_interface");
  fault(ARM_JOINT_7_ID);
  for (uint8_t id : ARM_MOTOR_IDS) {
    motor(id)->tripBusWatchdog();
  }
  ASSERT_TRUE(motor(ARM_JOINT_2_ID)->busWatchdogLatched()) << "Test premise: the watchdog must be latched";

  handleError();

  EXPECT_FALSE(motor(ARM_JOINT_2_ID)->busWatchdogLatched()) << "Securing wrote its hold goals into a latched watchdog.";
  EXPECT_TRUE(torqued(ARM_JOINT_2_ID)) << "The joint could hold, but was de-energised";
}

// A failed mode switch leaves joints in a mode their goals were not written for, so on_error
// secures the component even on a healthy bus.
TEST_F(SecuringTest, OnErrorSecuresAfterAFailedModeSwitch)
{
  startComponent("athena_arm_interface");
  motor(ARM_JOINT_1_ID)->setRegisterWriteFault("operating_mode", RegisterWriteFault::TIMEOUT);
  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/velocity"}, {}),
            hardware_interface::return_type::ERROR);
  ASSERT_NE(cycle(), hardware_interface::return_type::OK) << "write() must not go on after a failed mode switch";

  auto other = motor(ARM_JOINT_2_ID);
  other->startWriteTrace();
  handleError();
  other->stopWriteTrace();

  EXPECT_GE(other->lastWriteIndex("goal_position"), 0) << "on_error did not secure after a failed mode switch";
  EXPECT_TRUE(torqued(ARM_JOINT_2_ID)) << "An untouched joint should be holding";
  motor(ARM_JOINT_1_ID)->clearRegisterWriteFaults();
}

// A joint without a position command interface cannot hold station, and in a position mode no zero
// command is defined for it. So securing turns its torque off rather than leaving it as it is.
TEST_F(SecuringTest, JointThatCannotHoldPositionIsDeEnergisedRatherThanLeftAsIs)
{
  size_t stripped = 0;
  startComponent("athena_flipper_interface", [&stripped](std::string urdf) {
    const std::string block_open = "<ros2_control name=\"athena_flipper_interface\"";
    const std::string anchor = "<command_interface name=\"position\"/>";
    const auto begin = urdf.find(block_open);
    const auto end = begin == std::string::npos ? std::string::npos : urdf.find("</ros2_control>", begin);
    if (end == std::string::npos) {
      return urdf;
    }
    std::string block = urdf.substr(begin, end - begin);
    for (auto pos = block.find(anchor); pos != std::string::npos; pos = block.find(anchor, pos)) {
      block.erase(pos, anchor.size());
      ++stripped;
    }
    urdf.replace(begin, end - begin, block);
    return urdf;
  });
  ASSERT_EQ(stripped, FLIPPER_MOTOR_IDS.size()) << "Test premise: every flipper must lose its position interface";
  for (uint8_t id : FLIPPER_MOTOR_IDS) {
    ASSERT_TRUE(torqued(id)) << "Test premise: flipper " << static_cast<int>(id) << " must start energised";
  }

  fault(FLIPPER_FL_ID);
  handleError();

  for (uint8_t id : FLIPPER_MOTOR_IDS) {
    EXPECT_FALSE(torqued(id)) << "Flipper " << static_cast<int>(id)
                              << " is still energised although no hold or zero command is defined for it.";
  }
}

}  // namespace dynamixel_ros_control::test
