#include <cmath>
#include <limits>
#include <string>

#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {

namespace {

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

/// @p urdf with @p text inserted after the id parameter of motor @p id
std::string afterId(std::string urdf, const int id, const std::string& text)
{
  const std::string anchor = "<param name=\"id\">" + std::to_string(id) + "</param>";
  const auto pos = urdf.find(anchor);
  EXPECT_NE(pos, std::string::npos) << "no motor " << id;
  urdf.insert(pos + anchor.size(), text);
  return urdf;
}

void replaceAll(std::string& text, const std::string& from, const std::string& to)
{
  for (auto pos = text.find(from); pos != std::string::npos; pos = text.find(from, pos + to.size())) {
    text.replace(pos, from.size(), to);
  }
}

/// EEPROM limits for arm_joint_2, since a fresh mock holds 0 in them
std::string withLimits(std::string urdf)
{
  return afterId(std::move(urdf), ARM_JOINT_2_ID,
                 "<param name=\"registers.current_limit\">5</param>"
                 "<param name=\"registers.velocity_limit\">2</param>");
}

/// Every arm joint declares max_velocity, and arm_joint_1 loses its transmission and its configured profile velocity
std::string withMaxVelocity(std::string urdf)
{
  replaceAll(urdf, "<command_interface name=\"max_current\"/>",
             "<command_interface name=\"max_current\"/><command_interface name=\"max_velocity\"/>");
  replaceAll(urdf, "<param name=\"registers.profile_velocity\">20</param>", "");
  const auto begin = urdf.find("<transmission name=\"arm_joint_1_transmission\">");
  const std::string end_tag = "</transmission>";
  urdf.erase(begin, urdf.find(end_tag, begin) + end_tag.size() - begin);
  return withLimits(std::move(urdf));
}

/// arm_joint_2 keeps its goal across controller changes
std::string keepingGoal(std::string urdf)
{
  return afterId(std::move(urdf), ARM_JOINT_2_ID, "<param name=\"do_not_reset_on_ctrl_change\">true</param>");
}

/// Like withMaxVelocity(), with every profile velocity based
std::string withVelocityBasedMaxVelocity(std::string urdf)
{
  replaceAll(urdf, "<param name=\"registers.drive_mode\">4</param>", "<param name=\"registers.drive_mode\">0</param>");
  return withMaxVelocity(std::move(urdf));
}

}  // namespace

class CapTest : public DirectHardwareTest
{
protected:
  int16_t goalCurrentTicks(const uint8_t id) const
  {
    const auto motor = MockDynamixelManager::instance().getMotor(id);
    return static_cast<int16_t>(motor->read2Byte(motor->getAddress("goal_current")));
  }

  int32_t profileVelocityTicks(const uint8_t id) const
  {
    const auto motor = MockDynamixelManager::instance().getMotor(id);
    return motor->read4ByteSigned(motor->getAddress("profile_velocity"));
  }

  int operatingMode(const uint8_t id) const
  {
    const auto motor = MockDynamixelManager::instance().getMotor(id);
    return motor->read1Byte(motor->getAddress("operating_mode"));
  }

  double position(const std::string& joint) const
  {
    return states_.at(joint + "/position")->get_optional().value();
  }

  /// Init and the exports a controller manager runs before configure
  void initAndExport()
  {
    ASSERT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::SUCCESS);
    hardware_->on_export_state_interfaces();
    hardware_->on_export_command_interfaces();
  }

  void claim(const std::vector<std::string>& interfaces)
  {
    ASSERT_EQ(hardware_->perform_command_mode_switch(interfaces, {}), hardware_interface::return_type::OK);
  }

  /// Holds arm_joint_2 where it is, with @p cap on max_current
  void holdWithMaxCurrent(const double cap)
  {
    claim({"arm_joint_2/position", "arm_joint_2/max_current"});
    setCommand("arm_joint_2/position", position("arm_joint_2"));
    setCommand("arm_joint_2/max_current", cap);
    cycles(3);
  }

  void holdWithMaxVelocity(const double cap)
  {
    claim({"arm_joint_2/position", "arm_joint_2/max_velocity"});
    setCommand("arm_joint_2/position", position("arm_joint_2"));
    setCommand("arm_joint_2/max_velocity", cap);
    cycles(3);
  }

  void expectRefused(const std::vector<std::string>& interfaces, const std::string& reason)
  {
    EXPECT_NE(hardware_->switchRefusal(interfaces, {}).find(reason), std::string::npos)
        << hardware_->switchRefusal(interfaces, {});
    EXPECT_EQ(hardware_->prepare_command_mode_switch(interfaces, {}), hardware_interface::return_type::ERROR);
  }

  /// One tick of PH Profile Velocity, 0.01 rev/min
  static constexpr double PH_RAD_PER_S_PER_TICK = 0.00104719755;
};

TEST_F(CapTest, ACapWithoutAMainInterfaceIsRefused)
{
  startComponent("athena_arm_interface", withVelocityBasedMaxVelocity);
  expectRefused({"arm_joint_2/max_current"}, "without position or velocity");
  expectRefused({"arm_joint_2/max_velocity"}, "without position");
}

// Clean break: a limit beside position goes through a cap, never through a second main interface
TEST_F(CapTest, TwoMainInterfacesAreRefused)
{
  startComponent("athena_arm_interface");
  expectRefused({"arm_joint_2/position", "arm_joint_2/velocity"}, "more than one of position, velocity and current");
}

TEST_F(CapTest, PositionWithCurrentIsRefused)
{
  startComponent("athena_flipper_interface");
  expectRefused({"flipper_fl_joint/position", "flipper_fl_joint/current"},
                "more than one of position, velocity and current");
}

TEST_F(CapTest, MaxVelocityBesideVelocityIsRefused)
{
  startComponent("athena_arm_interface", withVelocityBasedMaxVelocity);
  expectRefused({"arm_joint_2/velocity", "arm_joint_2/max_velocity"}, "without position");
}

// The configured position mode stays, the cap goes to Goal Current
TEST_F(CapTest, PositionWithMaxCurrentKeepsTheModeAndWritesGoalCurrent)
{
  startComponent("athena_arm_interface", withLimits);
  holdWithMaxCurrent(1.5);
  EXPECT_EQ(operatingMode(ARM_JOINT_2_ID), POSITION);
  EXPECT_EQ(goalCurrentTicks(ARM_JOINT_2_ID), 1500);
}

TEST_F(CapTest, VelocityWithMaxCurrentWritesGoalCurrentOnAModelThatHonoursIt)
{
  startComponent("athena_arm_interface", withLimits);
  claim({"arm_joint_2/velocity", "arm_joint_2/max_current"});
  setCommand("arm_joint_2/velocity", 0.0);
  setCommand("arm_joint_2/max_current", 1.5);
  cycles(3);
  EXPECT_EQ(operatingMode(ARM_JOINT_2_ID), VELOCITY);
  EXPECT_EQ(goalCurrentTicks(ARM_JOINT_2_ID), 1500);
}

TEST_F(CapTest, MaxCurrentAboveTheEepromLimitIsClampedToIt)
{
  startComponent("athena_arm_interface", withLimits);
  holdWithMaxCurrent(9.0);
  EXPECT_EQ(goalCurrentTicks(ARM_JOINT_2_ID), 5000);
}

TEST_F(CapTest, ANaNMaxCurrentIsTheEepromLimit)
{
  startComponent("athena_arm_interface", withLimits);
  holdWithMaxCurrent(1.5);
  setCommand("arm_joint_2/max_current", NaN);
  cycles(3);
  EXPECT_EQ(goalCurrentTicks(ARM_JOINT_2_ID), 5000);
}

// 0 would lift the cap on the PRO series, so the floor is one tick
TEST_F(CapTest, AZeroMaxCurrentIsOneTick)
{
  startComponent("athena_arm_interface", withLimits);
  holdWithMaxCurrent(0.0);
  EXPECT_EQ(goalCurrentTicks(ARM_JOINT_2_ID), 1);
}

TEST_F(CapTest, AReleasedMaxCurrentIsTheEepromLimit)
{
  startComponent("athena_arm_interface", withLimits);
  holdWithMaxCurrent(1.5);
  ASSERT_EQ(hardware_->perform_command_mode_switch({}, {"arm_joint_2/max_current"}),
            hardware_interface::return_type::OK);
  cycles(3);
  EXPECT_EQ(goalCurrentTicks(ARM_JOINT_2_ID), 5000);
}

// A joint that keeps its goal keeps the cap through the deactivation, so only activation resets it
TEST_F(CapTest, ActivationResetsMaxCurrentToTheEepromLimit)
{
  startComponent("athena_arm_interface", [](std::string urdf) { return keepingGoal(withLimits(std::move(urdf))); });
  holdWithMaxCurrent(1.5);
  ASSERT_EQ(hardware_->on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware_->on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  cycles(3);
  EXPECT_EQ(goalCurrentTicks(ARM_JOINT_2_ID), 5000);
}

// Like the goal, so a gripper keeps its grip force through a controller change
TEST_F(CapTest, AJointThatKeepsItsGoalKeepsItsCapAcrossAControllerChange)
{
  startComponent("athena_arm_interface", [](std::string urdf) { return keepingGoal(withLimits(std::move(urdf))); });
  holdWithMaxCurrent(1.5);
  ASSERT_EQ(hardware_->perform_command_mode_switch({}, {"arm_joint_2/position", "arm_joint_2/max_current"}),
            hardware_interface::return_type::OK);
  cycles(3);
  EXPECT_EQ(goalCurrentTicks(ARM_JOINT_2_ID), 1500);
  claim({"arm_joint_2/position", "arm_joint_2/max_current"});
  cycles(3);
  EXPECT_EQ(goalCurrentTicks(ARM_JOINT_2_ID), 1500);
}

// A restart without a power cycle leaves the last cap in Goal Current, which must not become the ceiling
TEST_F(CapTest, AnUnreadableLimitFailsConfigure)
{
  initComponent("athena_arm_interface", withLimits);
  const auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_2_ID);
  motor->write2Byte(motor->getAddress("goal_current"), 500);
  motor->setRegisterReadFaultCount("current_limit", 2);  // the first goes to the initial value check
  initAndExport();
  EXPECT_EQ(hardware_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::FAILURE);
}

// RH-P12-RN(A) limits by Goal Current only in current-based position mode
class GripperCapTest : public CapTest
{
protected:
  static std::string asRhP12Rna(std::string urdf)
  {
    return afterId(std::move(urdf), GRIPPER_ID, "<param name=\"model_number\">35074</param>");
  }
};

TEST_F(GripperCapTest, MaxCurrentIsAcceptedInCurrentBasedPosition)
{
  startComponent("athena_arm_interface", asRhP12Rna);
  EXPECT_EQ(hardware_->prepare_command_mode_switch({"gripper_servo_joint/position", "gripper_servo_joint/max_current"},
                                                   {}),
            hardware_interface::return_type::OK);
}

TEST_F(GripperCapTest, MaxCurrentIsRefusedWhereGoalCurrentDoesNotLimit)
{
  startComponent("athena_arm_interface", asRhP12Rna);
  expectRefused({"gripper_servo_joint/velocity", "gripper_servo_joint/max_current"}, "velocity mode");
}

TEST_F(GripperCapTest, MaxCurrentIsRefusedInAPositionModeWhereGoalCurrentDoesNotLimit)
{
  startComponent("athena_arm_interface", [](std::string urdf) {
    replaceAll(urdf, "<param name=\"position_control_mode\">current_based_position</param>",
               "<param name=\"position_control_mode\">extended_position</param>");
    return asRhP12Rna(std::move(urdf));
  });
  expectRefused({"gripper_servo_joint/position", "gripper_servo_joint/max_current"}, "extended_position mode");
}

TEST_F(CapTest, PositionWithMaxVelocityKeepsTheModeAndWritesProfileVelocity)
{
  startComponent("athena_arm_interface", withVelocityBasedMaxVelocity);
  holdWithMaxVelocity(1.0);
  EXPECT_EQ(operatingMode(ARM_JOINT_2_ID), POSITION);
  EXPECT_EQ(profileVelocityTicks(ARM_JOINT_2_ID), std::lround(1.0 / PH_RAD_PER_S_PER_TICK));
}

TEST_F(CapTest, MaxVelocityAboveTheEepromLimitIsClampedToIt)
{
  startComponent("athena_arm_interface", withVelocityBasedMaxVelocity);
  holdWithMaxVelocity(1.0);
  setCommand("arm_joint_2/max_velocity", 5.0);
  cycles(3);
  EXPECT_EQ(profileVelocityTicks(ARM_JOINT_2_ID), std::lround(2.0 / PH_RAD_PER_S_PER_TICK));
}

// 0 is an unlimited profile, so the floor is one tick
TEST_F(CapTest, AZeroMaxVelocityIsOneTick)
{
  startComponent("athena_arm_interface", withVelocityBasedMaxVelocity);
  holdWithMaxVelocity(0.0);
  EXPECT_EQ(profileVelocityTicks(ARM_JOINT_2_ID), 1);
}

TEST_F(CapTest, ANaNOrReleasedMaxVelocityIsTheEepromLimit)
{
  startComponent("athena_arm_interface", withVelocityBasedMaxVelocity);
  const int32_t limit = std::lround(2.0 / PH_RAD_PER_S_PER_TICK);
  holdWithMaxVelocity(1.0);
  setCommand("arm_joint_2/max_velocity", NaN);
  cycles(3);
  EXPECT_EQ(profileVelocityTicks(ARM_JOINT_2_ID), limit);

  setCommand("arm_joint_2/max_velocity", 1.0);
  cycles(3);
  ASSERT_EQ(hardware_->perform_command_mode_switch({}, {"arm_joint_2/max_velocity"}),
            hardware_interface::return_type::OK);
  cycles(3);
  EXPECT_EQ(profileVelocityTicks(ARM_JOINT_2_ID), limit);
}

// In a time-based profile Profile Velocity is a duration
TEST_F(CapTest, MaxVelocityOnATimeBasedProfileFailsConfigure)
{
  initComponent("athena_arm_interface", withMaxVelocity);
  initAndExport();
  EXPECT_EQ(hardware_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::FAILURE);
}

TEST_F(CapTest, MaxVelocityOnAModelWithoutProfileVelocityFailsConfigure)
{
  initComponent("athena_arm_interface", [](std::string urdf) {
    return afterId(withVelocityBasedMaxVelocity(std::move(urdf)), ARM_JOINT_3_ID,
                   "<param name=\"model_number\">51200</param>");
  });
  initAndExport();
  EXPECT_EQ(hardware_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::FAILURE);
}

// The cap overwrites its register every cycle, so a configured value there would never hold
TEST_F(CapTest, AConfiguredCapRegisterFailsInit)
{
  initComponent("athena_arm_interface", [](std::string urdf) {
    return afterId(withVelocityBasedMaxVelocity(std::move(urdf)), ARM_JOINT_2_ID,
                   "<param name=\"registers.profile_velocity\">100</param>");
  });
  EXPECT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::ERROR);
}

// A transmission would hand the motor a joint velocity off by the reduction
TEST_F(CapTest, MaxVelocityBehindATransmissionFailsInit)
{
  initComponent("athena_arm_interface", [](std::string urdf) {
    replaceAll(urdf, "<command_interface name=\"max_current\"/>",
               "<command_interface name=\"max_current\"/><command_interface name=\"max_velocity\"/>");
    return urdf;
  });
  EXPECT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::ERROR);
}

}  // namespace dynamixel_ros_control::test
