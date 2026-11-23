#include <string>

#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {

// do_not_reset_on_ctrl_change: a gripper holding an object keeps squeezing only while its goal stays
// beyond the object. A controller change or a missing command must not move the goal to where the
// fingers stand.
class KeepGoalTest : public DirectHardwareTest
{
protected:
  static std::string keepGripperGoal(std::string urdf)
  {
    const std::string anchor = "<param name=\"id\">" + std::to_string(static_cast<int>(GRIPPER_ID)) + "</param>";
    const auto pos = urdf.find(anchor);
    EXPECT_NE(pos, std::string::npos);
    urdf.insert(pos + anchor.size(), "<param name=\"do_not_reset_on_ctrl_change\">true</param>");
    return urdf;
  }

  void SetUp() override
  {
    DirectHardwareTest::SetUp();
    startComponent("athena_arm_interface", keepGripperGoal);
    motor_ = MockDynamixelManager::instance().getMotor(GRIPPER_ID);
    ASSERT_NE(motor_, nullptr);
    startInterface("gripper_servo_joint/position", 0.9);
    cycles(3);
    ASSERT_LT(motor_->getJointPosition(), 0.7) << "the gripper must still be closing when the test acts";
  }

  std::shared_ptr<MockDynamixel> motor_;
};

TEST_F(KeepGoalTest, AMissingCommandKeepsTheGoal)
{
  setCommand("gripper_servo_joint/position", std::numeric_limits<double>::quiet_NaN());
  cycles(100);
  EXPECT_NEAR(motor_->getJointPosition(), 0.9, 0.05) << "the goal fell back to where the gripper stood";
}

TEST_F(KeepGoalTest, AControllerChangeKeepsTheGoal)
{
  ASSERT_EQ(hardware_->perform_command_mode_switch({}, {"gripper_servo_joint/position"}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(hardware_->perform_command_mode_switch({"gripper_servo_joint/position"}, {}),
            hardware_interface::return_type::OK);
  cycles(100);
  EXPECT_NEAR(motor_->getJointPosition(), 0.9, 0.05) << "the goal fell back to where the gripper stood";
}

// A grasp from before a power cut must not come back after the fingers were opened by hand
TEST_F(KeepGoalTest, AGoalOfAnEarlierActivationDoesNotComeBack)
{
  cycles(100);
  ASSERT_NEAR(motor_->getJointPosition(), 0.9, 0.05);
  ASSERT_EQ(hardware_->on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  motor_->powerCycle();
  motor_->moveByHand(-0.6);
  ASSERT_EQ(hardware_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware_->on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  cycles(5);
  const double at_activation = motor_->getJointPosition();
  ASSERT_LT(at_activation, 0.5);

  startInterface("gripper_servo_joint/position", std::numeric_limits<double>::quiet_NaN());
  cycles(100);
  EXPECT_NEAR(motor_->getJointPosition(), at_activation, 0.05) << "a goal of the previous activation came back";
}

// A controller manager that leaves the last command in a released interface, as the upstream one
// does, must not hand that command to the next controller
TEST_F(KeepGoalTest, AStartedInterfaceGetsTheKeptGoalInsteadOfALeftOverCommand)
{
  cycles(100);
  ASSERT_EQ(hardware_->on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  motor_->powerCycle();
  motor_->moveByHand(-0.6);
  ASSERT_EQ(hardware_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware_->on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  cycles(5);
  const double at_activation = motor_->getJointPosition();
  ASSERT_LT(at_activation, 0.5);

  ASSERT_EQ(hardware_->perform_command_mode_switch({"gripper_servo_joint/position"}, {}),
            hardware_interface::return_type::OK);
  cycles(100);
  EXPECT_NEAR(motor_->getJointPosition(), at_activation, 0.05) << "the left-over command of the old activation ran";
}

// A kept goal is still dropped when torque comes back, otherwise the gripper would snap back to it
// after a hand moved the limp fingers
TEST_F(KeepGoalTest, TorqueBackAfterTorqueOffHoldsWhereTheHandLeftTheGripper)
{
  cycles(100);
  ASSERT_EQ(hardware_->perform_command_mode_switch({"gripper_servo_joint/torque_off"},
                                                   {"gripper_servo_joint/position"}),
            hardware_interface::return_type::OK);
  motor_->moveByHand(-0.6);
  cycles(5);
  const double moved_to = motor_->getJointPosition();
  ASSERT_LT(moved_to, 0.5);

  ASSERT_EQ(hardware_->perform_command_mode_switch({"gripper_servo_joint/position"},
                                                   {"gripper_servo_joint/torque_off"}),
            hardware_interface::return_type::OK);
  cycles(100);
  EXPECT_NEAR(motor_->getJointPosition(), moved_to, 0.05) << "the gripper drove back to its kept goal";
}

}  // namespace dynamixel_ros_control::test
