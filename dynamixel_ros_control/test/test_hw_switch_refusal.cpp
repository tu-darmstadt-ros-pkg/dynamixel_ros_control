#include <string>

#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {

// prepare_command_mode_switch refuses what the joints cannot carry out, before anything changes, so
// a controller manager rejects the switch instead of faulting the hardware in perform.
class SwitchRefusalTest : public DirectHardwareTest
{
protected:
  void SetUp() override
  {
    DirectHardwareTest::SetUp();
    startComponent("athena_arm_interface");
  }
};

TEST_F(SwitchRefusalTest, PrepareRefusesAnUnknownInterface)
{
  EXPECT_EQ(hardware_->prepare_command_mode_switch({"arm_joint_1/bogus"}, {}), hardware_interface::return_type::ERROR);
  EXPECT_EQ(hardware_->prepare_command_mode_switch({"no_joint/position"}, {}), hardware_interface::return_type::ERROR);
}

// A joint in current mode cannot be held without its current command, so the mode is refused until
// that hold exists. The flippers declare current, the arm joints only max_current.
class CurrentRefusalTest : public DirectHardwareTest
{
protected:
  void SetUp() override
  {
    DirectHardwareTest::SetUp();
    startComponent("athena_flipper_interface");
  }
};

TEST_F(CurrentRefusalTest, PrepareRefusesAPureCurrentClaim)
{
  EXPECT_EQ(hardware_->prepare_command_mode_switch({"flipper_fl_joint/current"}, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_NE(hardware_->switchRefusal({"flipper_fl_joint/current"}, {}).find("current mode, which is not supported"),
            std::string::npos);
}

// A controller manager may skip prepare. perform refuses the same switch before it changes anything.
TEST_F(CurrentRefusalTest, PerformRefusesWhatPrepareRefuses)
{
  EXPECT_EQ(hardware_->perform_command_mode_switch({"flipper_fl_joint/current"}, {}),
            hardware_interface::return_type::ERROR);
  auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  EXPECT_NE(motor->read1Byte(motor->getAddress("operating_mode")), CURRENT);
  cycles(5);
}

}  // namespace dynamixel_ros_control::test
