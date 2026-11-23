#include <array>
#include <chrono>
#include <string>

#include <std_srvs/srv/trigger.hpp>

#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {

// Torque off is a command mode: a controller claims <joint>/torque_off and the joint goes limp until
// the claim ends. The switch itself does the bus work, so no value is ever written to the interface.
class TorqueOffModeTest : public DirectHardwareTest
{
protected:
  // The test robot exports torque_off on every motor joint. arm_joint_3 loses it here, so one
  // joint shows the refusal of a joint without it.
  static std::string withoutTorqueOffOnJoint3(std::string urdf)
  {
    const std::string anchor = "<param name=\"id\">" + std::to_string(static_cast<int>(ARM_JOINT_3_ID)) +
                               "</param>\n      <command_interface name=\"torque_off\"/>";
    const auto pos = urdf.find(anchor);
    EXPECT_NE(pos, std::string::npos);
    urdf.replace(pos, anchor.size(),
                 "<param name=\"id\">" + std::to_string(static_cast<int>(ARM_JOINT_3_ID)) + "</param>");
    return urdf;
  }

  void SetUp() override
  {
    DirectHardwareTest::SetUp();
    startComponent("athena_arm_interface", withoutTorqueOffOnJoint3);
    motor_ = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
    ASSERT_NE(motor_, nullptr);
  }

  [[nodiscard]] bool torqueOn() const
  {
    return motor_->read1Byte(motor_->getAddress("torque_enable")) == 1;
  }

  hardware_interface::return_type prepare(const std::vector<std::string>& start, const std::vector<std::string>& stop)
  {
    return hardware_->prepare_command_mode_switch(start, stop);
  }

  std::shared_ptr<MockDynamixel> motor_;
};

TEST_F(TorqueOffModeTest, TheSwitchTakesTheTorqueAndGivesItBackWithoutAJump)
{
  startInterface("arm_joint_1/position", std::numeric_limits<double>::quiet_NaN());
  cycles(10);
  ASSERT_TRUE(torqueOn());

  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/torque_off"}, {"arm_joint_1/position"}),
            hardware_interface::return_type::OK);
  EXPECT_FALSE(torqueOn());
  cycles(5);
  motor_->moveByHand(0.3);
  cycles(5);
  const double moved_to = motor_->getJointPosition();

  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/position"}, {"arm_joint_1/torque_off"}),
            hardware_interface::return_type::OK);
  EXPECT_TRUE(torqueOn());
  cycles(50);
  EXPECT_NEAR(motor_->getJointPosition(), moved_to, 0.01) << "the joint jumped when its torque came back";
}

// A stop of torque_off alone, as the controller manager's hold after a fault sends it, leaves the
// joint limp. A later motion start energises it where the hand left it.
TEST_F(TorqueOffModeTest, AStoppedTorqueOffStaysLimpUntilAMotionStart)
{
  startInterface("arm_joint_2/position", std::numeric_limits<double>::quiet_NaN());
  cycles(10);
  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/torque_off"}, {}), hardware_interface::return_type::OK);
  cycles(5);

  ASSERT_EQ(hardware_->perform_command_mode_switch({}, {"arm_joint_1/torque_off", "arm_joint_2/position"}),
            hardware_interface::return_type::OK);
  EXPECT_FALSE(torqueOn()) << "stopping torque_off without a motion start energised the joint";
  cycles(5);
  motor_->moveByHand(0.3);
  cycles(5);
  const double moved_to = motor_->getJointPosition();

  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/position"}, {}), hardware_interface::return_type::OK);
  EXPECT_TRUE(torqueOn());
  cycles(50);
  EXPECT_NEAR(motor_->getJointPosition(), moved_to, 0.01) << "the joint jumped when its torque came back";
}

// The claim ends with a fault, so after the recovery the joint must accept a motion claim again
TEST_F(TorqueOffModeTest, AfterAFaultWithTorqueOffClaimedTheJointCanBeCommandedAgain)
{
  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/torque_off"}, {}), hardware_interface::return_type::OK);
  cycles(5);
  ASSERT_EQ(hardware_->on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware_->on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  cycles(5);
  EXPECT_TRUE(torqueOn());
  EXPECT_EQ(prepare({"arm_joint_1/position"}, {}), hardware_interface::return_type::OK)
      << "a torque_off claim from before the fault still blocks the joint";
}

// A stale goal register drives the motor the moment torque is on, so the goal must hold the present
// position before torque comes back. Only the order of the bus writes shows this.
TEST_F(TorqueOffModeTest, TorqueComesBackOnlyAfterTheGoalHoldsThePresentPosition)
{
  startInterface("arm_joint_1/position", std::numeric_limits<double>::quiet_NaN());
  cycles(10);
  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/torque_off"}, {"arm_joint_1/position"}),
            hardware_interface::return_type::OK);
  cycles(5);
  motor_->moveByHand(0.3);
  cycles(5);

  motor_->startWriteTrace();
  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/position"}, {"arm_joint_1/torque_off"}),
            hardware_interface::return_type::OK);
  motor_->stopWriteTrace();

  const int torque_on = motor_->firstWriteIndex("torque_enable", 1);
  ASSERT_GE(torque_on, 0);
  EXPECT_GT(torque_on, motor_->lastWriteIndex("goal_position")) << "torque came on before the goal was reset";
  EXPECT_NEAR(motor_->read4ByteSigned(motor_->getAddress("goal_position")),
              motor_->read4ByteSigned(motor_->getAddress("present_position")), 5);
}

// A joint going limp loses torque before its mode or goals are touched, and is never energised on
// the way, even when the switch changes its operating mode
TEST_F(TorqueOffModeTest, GoingLimpTurnsTorqueOffBeforeAnyModeOrGoalWrite)
{
  startInterface("arm_joint_1/velocity", 0.0);
  cycles(5);

  motor_->startWriteTrace();
  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/torque_off"}, {"arm_joint_1/velocity"}),
            hardware_interface::return_type::OK);
  motor_->stopWriteTrace();

  const int torque_off = motor_->firstWriteIndex("torque_enable", 0);
  ASSERT_GE(torque_off, 0);
  ASSERT_GE(motor_->firstWriteIndex("operating_mode"), 0) << "test premise: the switch changes the mode";
  EXPECT_EQ(motor_->firstWriteIndex("torque_enable", 1), -1) << "the joint was energised on its way to limp";
  for (const char* reg : {"operating_mode", "goal_position", "goal_velocity"}) {
    const int first = motor_->firstWriteIndex(reg);
    EXPECT_TRUE(first == -1 || first > torque_off) << reg << " was written before torque went off";
  }
}

// Torque that comes back on only some joints of a claim leaves a half limp arm, so it is rolled back
TEST_F(TorqueOffModeTest, APartialTorqueOnIsRolledBack)
{
  const auto motor_2 = MockDynamixelManager::instance().getMotor(ARM_JOINT_2_ID);
  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/torque_off", "arm_joint_2/torque_off"}, {}),
            hardware_interface::return_type::OK);
  cycles(5);

  motor_2->setRegisterWriteFault("torque_enable", RegisterWriteFault::DROP);
  EXPECT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/position", "arm_joint_2/position"},
                                                   {"arm_joint_1/torque_off", "arm_joint_2/torque_off"}),
            hardware_interface::return_type::ERROR);
  EXPECT_FALSE(torqueOn()) << "arm_joint_1 was left energised while arm_joint_2 stayed limp";
  motor_2->clearRegisterWriteFaults();
}

// A reboot restores the torque each joint desires, and a joint held limp desires none
TEST_F(TorqueOffModeTest, ARebootKeepsAJointHeldTorqueOffLimp)
{
  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/torque_off"}, {}), hardware_interface::return_type::OK);
  motor_->setHardwareError(0x20);  // overload, so the reboot actually reboots this motor
  ASSERT_NE(cycle(), hardware_interface::return_type::OK);
  ASSERT_EQ(hardware_->on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  const auto response = callService<std_srvs::srv::Trigger>("/athena_arm_interface_node/reboot",
                                                            std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success) << response->message;
  EXPECT_EQ(motor_->getRebootCount(), 1);
  EXPECT_FALSE(torqueOn()) << "the reboot energised a joint held limp";
}

// A torque_off that could not be verified still ends the intent to have torque, so no reboot
// energises the joint the operator asked to go limp
TEST_F(TorqueOffModeTest, AFailedTorqueOffLeavesNoTorqueForARebootToRestore)
{
  auto silent = MockDynamixelManager::instance().getMotor(ARM_JOINT_2_ID);
  ASSERT_NE(silent, nullptr);
  silent->setCommunicationFault(CommunicationFault::DROP_STATUS_PACKETS);
  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_2/torque_off"}, {}),
            hardware_interface::return_type::ERROR);
  silent->setCommunicationFault(CommunicationFault::NONE);

  silent->setHardwareError(0x20);  // overload, so the reboot actually reboots this motor
  ASSERT_EQ(hardware_->on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  const auto response = callService<std_srvs::srv::Trigger>("/athena_arm_interface_node/reboot",
                                                            std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success) << response->message;
  EXPECT_EQ(silent->getRebootCount(), 1);
  EXPECT_EQ(silent->read1Byte(silent->getAddress("torque_enable")), 0) << "the reboot energised a joint sent limp";
}

// Green means safe to touch, so a limp joint shows it and turns blue again with its torque
TEST_F(TorqueOffModeTest, TheLedsShowTheTorqueOffMode)
{
  const auto colour = [this] {
    return std::array<uint8_t, 3>{motor_->getLedRed(), motor_->getLedGreen(), motor_->getLedBlue()};
  };
  ASSERT_EQ(colour(), (std::array<uint8_t, 3>{COLOR_BLUE_R, COLOR_BLUE_G, COLOR_BLUE_B}));
  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/torque_off"}, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(colour(), (std::array<uint8_t, 3>{COLOR_GREEN_R, COLOR_GREEN_G, COLOR_GREEN_B}));
  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/position"}, {"arm_joint_1/torque_off"}),
            hardware_interface::return_type::OK);
  EXPECT_EQ(colour(), (std::array<uint8_t, 3>{COLOR_BLUE_R, COLOR_BLUE_G, COLOR_BLUE_B}));
}

// A motor whose torque off cannot be read back is not shown green, since nothing proves it safe
TEST_F(TorqueOffModeTest, AnUnverifiableTorqueOffIsNotShownAsSafe)
{
  auto silent = MockDynamixelManager::instance().getMotor(ARM_JOINT_2_ID);
  ASSERT_NE(silent, nullptr);
  silent->setCommunicationFault(CommunicationFault::DROP_STATUS_PACKETS);

  EXPECT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/torque_off", "arm_joint_2/torque_off"}, {}),
            hardware_interface::return_type::ERROR);

  EXPECT_EQ(motor_->getLedRed(), COLOR_GREEN_R) << "A verified limp motor should be green";
  EXPECT_EQ(motor_->getLedGreen(), COLOR_GREEN_G);
  EXPECT_EQ(motor_->getLedBlue(), COLOR_GREEN_B);
  const bool silent_is_green = silent->getLedRed() == COLOR_GREEN_R && silent->getLedGreen() == COLOR_GREEN_G &&
                               silent->getLedBlue() == COLOR_GREEN_B;
  EXPECT_FALSE(silent_is_green) << "A motor whose torque could not be read back is shown as safe to touch";
  silent->setCommunicationFault(CommunicationFault::NONE);
}

// Torque that came back cannot run without comm loss protection, so a watchdog that cannot be armed
// sends the joint back to limp
TEST_F(TorqueOffModeTest, TorqueOnRollsBackWhenTheWatchdogCannotBeRearmed)
{
  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/torque_off"}, {}), hardware_interface::return_type::OK);
  cycles(5);
  const auto other = MockDynamixelManager::instance().getMotor(ARM_JOINT_3_ID);
  other->setRegisterWriteFault("bus_watchdog", RegisterWriteFault::TIMEOUT);

  EXPECT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/position"}, {"arm_joint_1/torque_off"}),
            hardware_interface::return_type::ERROR);
  EXPECT_FALSE(torqueOn()) << "the joint was left energised without a verified bus watchdog";
  other->clearRegisterWriteFaults();
}

// Bringing the torque back arms the cleared watchdog once, not once for the torque and again for the switch
TEST_F(TorqueOffModeTest, TorqueComingBackArmsTheWatchdogOnce)
{
  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/torque_off"}, {}), hardware_interface::return_type::OK);
  cycles(5);
  motor_->startWriteTrace();
  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/position"}, {"arm_joint_1/torque_off"}),
            hardware_interface::return_type::OK);
  motor_->stopWriteTrace();
  ASSERT_TRUE(torqueOn());
  const auto armed = static_cast<uint8_t>(motor_->read1Byte(motor_->getAddress("bus_watchdog")));
  ASSERT_NE(armed, 0) << "precondition: the arm joint has a bus watchdog";
  EXPECT_GE(motor_->firstWriteIndex("bus_watchdog", armed), 0);
  EXPECT_EQ(motor_->firstWriteIndex("bus_watchdog", armed), motor_->lastWriteIndex("bus_watchdog", armed))
      << "the switch armed the watchdog more than once";
}

// Activation trusts no old torque reading. A motor energised behind the interface's back since it
// last read limp must not stay torqued in a mode nothing commands.
TEST_F(TorqueOffModeTest, ActivationTakesTheTorqueOfAMotorEnergisedBehindItsBack)
{
  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/torque_off"}, {}), hardware_interface::return_type::OK);
  cycles(5);
  ASSERT_EQ(hardware_->on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  constexpr uint8_t velocity_mode = 1;
  motor_->write1Byte(motor_->getAddress("operating_mode"), velocity_mode);
  motor_->write1Byte(motor_->getAddress("torque_enable"), 1);

  (void) hardware_->on_activate(rclcpp_lifecycle::State());
  EXPECT_FALSE(torqueOn() && motor_->read1Byte(motor_->getAddress("operating_mode")) == velocity_mode)
      << "the motor was left torqued in the mode it was found in";
}

TEST_F(TorqueOffModeTest, OtherJointsKeepTheirTorque)
{
  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_1/torque_off"}, {}), hardware_interface::return_type::OK);
  auto gripper = MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  EXPECT_EQ(gripper->read1Byte(gripper->getAddress("torque_enable")), 1);
  cycles(10);
}

TEST_F(TorqueOffModeTest, PrepareRefusesTorqueOffTogetherWithAMotionInterface)
{
  EXPECT_EQ(prepare({"arm_joint_1/torque_off", "arm_joint_1/position"}, {}), hardware_interface::return_type::ERROR);
  EXPECT_NE(hardware_->switchRefusal({"arm_joint_1/torque_off", "arm_joint_1/position"}, {})
                .find("cannot be torque_off and commanded at once"),
            std::string::npos);
  EXPECT_EQ(prepare({"arm_joint_1/torque_off"}, {}), hardware_interface::return_type::OK);
}

TEST_F(TorqueOffModeTest, PrepareRefusesTorqueOffOnAJointThatDoesNotExportIt)
{
  EXPECT_EQ(prepare({"arm_joint_3/torque_off"}, {}), hardware_interface::return_type::ERROR);
}

// With torque_on_startup false the component comes up limp, and starting a motion interface is
// what energises a joint, also one without torque_off
class DeEnergisedStartTest : public TorqueOffModeTest
{
protected:
  void SetUp() override
  {
    DirectHardwareTest::SetUp();
    startComponent("athena_arm_interface", [](std::string urdf) {
      const std::string on = "<param name=\"torque_on_startup\">true</param>";
      urdf.replace(urdf.find(on), on.size(), "<param name=\"torque_on_startup\">false</param>");
      return withoutTorqueOffOnJoint3(urdf);
    });
    motor_ = MockDynamixelManager::instance().getMotor(ARM_JOINT_3_ID);
    ASSERT_NE(motor_, nullptr);
  }
};

TEST_F(DeEnergisedStartTest, AMotionStartEnergisesAJointWithoutTorqueOff)
{
  ASSERT_FALSE(torqueOn());
  motor_->moveByHand(0.3);
  cycles(5);
  const double moved_to = motor_->getJointPosition();

  startInterface("arm_joint_3/position", std::numeric_limits<double>::quiet_NaN());
  EXPECT_TRUE(torqueOn()) << "a started position controller commands a limp joint";
  cycles(50);
  EXPECT_NEAR(motor_->getJointPosition(), moved_to, 0.01) << "the joint jumped when its torque came on";
}

}  // namespace dynamixel_ros_control::test
