#include <string>

#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {

// A misspelt parameter must stop the component, not be ignored while its default takes effect.
using UnknownParametersTest = DirectHardwareTest;

namespace {
std::string insertAfter(std::string urdf, const std::string& anchor, const std::string& text)
{
  const auto pos = urdf.find(anchor);
  EXPECT_NE(pos, std::string::npos) << anchor;
  urdf.insert(pos + anchor.size(), text);
  return urdf;
}
}  // namespace

TEST_F(UnknownParametersTest, AnUnknownHardwareParameterFailsInit)
{
  initComponent("athena_arm_interface", [](std::string urdf) {
    return insertAfter(std::move(urdf), "<param name=\"use_dummy\">true</param>",
                       "<param name=\"torque_on_start\">true</param>");
  });
  EXPECT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::ERROR);
}

TEST_F(UnknownParametersTest, AnUnknownJointParameterFailsInit)
{
  initComponent("athena_arm_interface", [](std::string urdf) {
    return insertAfter(std::move(urdf),
                       "<param name=\"id\">" + std::to_string(static_cast<int>(GRIPPER_ID)) + "</param>",
                       "<param name=\"do_not_reset_on_controller_change\">true</param>");
  });
  EXPECT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::ERROR);
}

// Any register but these would be written with a goal of 0 on activation, current_limit into EEPROM
TEST_F(UnknownParametersTest, ACommandInterfaceOtherThanTheMotionOnesFailsInit)
{
  for (const std::string interface_name : {"current_limit", "goal_pwm", "present_position"}) {
    initComponent("athena_arm_interface", [&interface_name](std::string urdf) {
      return insertAfter(std::move(urdf),
                         "<param name=\"id\">" + std::to_string(static_cast<int>(GRIPPER_ID)) + "</param>",
                         "<command_interface name=\"" + interface_name + "\"/>");
    });
    EXPECT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::ERROR) << interface_name;
    hardware_.reset();
    MockDynamixelManager::instance().reset();
  }
}

TEST_F(UnknownParametersTest, AnInvalidOptionalHardwareParameterFailsInit)
{
  for (const std::string parameter :
       {"debug", "torque_on_startup", "torque_off_on_shutdown", "allow_velocity_without_watchdog", "use_dummy",
        "publish_goal_joint_states", "publish_read_joint_states", "publish_write_joint_states"}) {
    initComponent("athena_arm_interface", [&parameter](std::string urdf) {
      const std::string anchor = "<ros2_control name=\"athena_arm_interface\"";
      const auto block = urdf.find(anchor);
      const std::string existing = "<param name=\"" + parameter + "\">";
      const auto pos = urdf.find(existing, block);
      const auto block_end = urdf.find("</hardware>", block);
      // The first of two equal params wins, so a present one is replaced
      if (pos != std::string::npos && pos < block_end) {
        urdf.replace(pos, urdf.find("</param>", pos) - pos, existing + "yes");
        return urdf;
      }
      return insertAfter(std::move(urdf), "<param name=\"use_dummy\">true</param>", existing + "yes</param>");
    });
    EXPECT_EQ(params_.hardware_info.hardware_parameters.at(parameter), "yes");
    EXPECT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::ERROR) << parameter;
    hardware_.reset();
    MockDynamixelManager::instance().reset();
  }
}

TEST_F(UnknownParametersTest, AnInvalidOptionalJointParameterFailsInit)
{
  initComponent("athena_arm_interface", [](std::string urdf) {
    return insertAfter(std::move(urdf),
                       "<param name=\"id\">" + std::to_string(static_cast<int>(GRIPPER_ID)) + "</param>",
                       "<param name=\"do_not_reset_on_ctrl_change\">yes</param>");
  });
  EXPECT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::ERROR);
}

// A loader reports a configuration it cannot read by returning nothing, and the interfaces are
// exported through the transmission it did not build
TEST_F(UnknownParametersTest, ATransmissionThatCouldNotBeBuiltFailsInit)
{
  initComponent("athena_flipper_interface", [](std::string urdf) {
    // A value that is not a boolean, which the loader refuses rather than reading as false
    return insertAfter(std::move(urdf), "<plugin>hector_transmission_interface/AdjustableOffsetTransmission</plugin>",
                       "<param name=\"pass_through_effort\">maybe</param>");
  });
  EXPECT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::ERROR);
}

// Two joints on one id are two names for one motor, each writing its own configuration and goals
// to it. Refused in on_init, since the sync read group would refuse it only after both
// configurations reached the motor.
TEST_F(UnknownParametersTest, TwoJointsOnOneMotorIdFailInit)
{
  initComponent("athena_arm_interface", [](std::string urdf) {
    const std::string anchor = "<param name=\"id\">" + std::to_string(static_cast<int>(ARM_JOINT_2_ID)) + "</param>";
    const auto pos = urdf.find(anchor);
    EXPECT_NE(pos, std::string::npos);
    // Replaced rather than added, since the first of two equal params wins
    return urdf.replace(pos, anchor.size(),
                        "<param name=\"id\">" + std::to_string(static_cast<int>(ARM_JOINT_1_ID)) + "</param>");
  });
  const auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);
  motor->startWriteTrace();
  EXPECT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::ERROR);
  motor->stopWriteTrace();
  // The duplicate joint's own registers, and what any connect or activation writes
  for (const char* register_name : {"homing_offset", "led_blue", "torque_enable", "goal_position"}) {
    EXPECT_EQ(motor->firstWriteIndex(register_name), -1) << register_name << " was written before the refusal";
  }
}

// Every goal reset holds the present position. Without a position state the hold goal is 0, which
// verifies against itself, and torque-on drives every joint to position 0.
TEST_F(UnknownParametersTest, ADescriptionWithoutPositionStateFailsInit)
{
  initComponent("athena_arm_interface", [](std::string urdf) {
    const std::string position_state = "<state_interface name=\"position\"/>";
    for (auto pos = urdf.find(position_state); pos != std::string::npos; pos = urdf.find(position_state, pos)) {
      urdf.erase(pos, position_state.size());
    }
    return urdf;
  });
  EXPECT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::ERROR);
}

// position_control_mode is the mode a claimed position interface selects and the emergency hold
// uses. A non-position mode leaves the motor ignoring every goal position it is sent.
TEST_F(UnknownParametersTest, ANonPositionControlModeFailsInit)
{
  initComponent("athena_arm_interface", [](std::string urdf) {
    const std::string anchor = "<param name=\"position_control_mode\">extended_position</param>";
    const auto pos = urdf.find(anchor);
    EXPECT_NE(pos, std::string::npos);
    return urdf.replace(pos, anchor.size(), "<param name=\"position_control_mode\">velocity</param>");
  });
  EXPECT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::ERROR);
}

// A number may carry surrounding whitespace, so a bool may too
TEST(BoolParameter, SurroundingWhitespaceIsIgnored)
{
  bool value = false;
  EXPECT_TRUE(getParameter<bool>({{"flag", " True\n"}}, "flag", value));
  EXPECT_TRUE(value);
  EXPECT_TRUE(getParameter<bool>({{"flag", "\t0 "}}, "flag", value));
  EXPECT_FALSE(value);
  EXPECT_FALSE(getParameter<bool>({{"flag", "t rue"}}, "flag", value));
}

TEST_F(UnknownParametersTest, EveryDocumentedParameterIsAccepted)
{
  initComponent("athena_arm_interface", [](std::string urdf) {
    urdf = insertAfter(std::move(urdf), "<param name=\"use_dummy\">true</param>",
                       "<param name=\"publish_read_joint_states\">false</param>");
    return insertAfter(std::move(urdf),
                       "<param name=\"id\">" + std::to_string(static_cast<int>(GRIPPER_ID)) + "</param>",
                       "<param name=\"do_not_reset_on_ctrl_change\">true</param>");
  });
  EXPECT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::SUCCESS);
}

}  // namespace dynamixel_ros_control::test
