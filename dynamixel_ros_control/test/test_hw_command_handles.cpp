#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// A controller that starts without a command of its own must hold, not drive to the last command an
// earlier activation left in its interface. The mode switch writes the hold goal into the handle.
TEST_F(HardwareInterfaceTest, StartedInterfaceHoldsInsteadOfTheLeftOverCommand)
{
  ASSERT_TRUE(loadAndActivateController("gripper_position_controller"));
  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  ASSERT_NE(motor, nullptr);

  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/gripper_position_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(pub, 5s));
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.9};
  pub->publish(cmd);
  ASSERT_TRUE(executor_->spin_until([&] { return std::abs(motor->getJointPosition() - 0.9) < 0.05; }, scaled(10s)));

  // Head back, and stop the controller on the way: its last command stays in the interface.
  cmd.data = {0.1};
  pub->publish(cmd);
  ASSERT_TRUE(executor_->spin_until([&] { return motor->getJointPosition() < 0.85; }, scaled(10s)));
  ASSERT_TRUE(switch_controllers({}, {"gripper_position_controller"}));
  ASSERT_TRUE(executor_->spin_until([&] { return std::abs(motor->getCurrentVelocity()) < 1e-3; }, scaled(10s)))
      << "the stopped joint did not come to rest";
  const double held = motor->getJointPosition();
  ASSERT_GT(held, 0.4) << "the stop must have caught the joint well before the old target";

  // Started again, the controller writes nothing until it gets a new command.
  ASSERT_TRUE(switch_controllers({"gripper_position_controller"}, {}));
  // A window to watch, since nothing is expected to happen in it
  std::this_thread::sleep_for(1s);
  EXPECT_NEAR(motor->getJointPosition(), held, 0.05) << "the joint moved towards the left-over command";
}

}  // namespace dynamixel_ros_control::test
