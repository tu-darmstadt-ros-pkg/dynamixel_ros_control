#include <mutex>
#include <shared_mutex>

#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {

// NaN is how a controller says it has no command, and what hector_controller_manager leaves in an
// interface it takes from a controller. The joint must hold, never keep executing the last command.
using NaNHoldTest = DirectHardwareTest;

TEST_F(NaNHoldTest, PositionNaNHoldsWhereTheJointWas)
{
  startComponent("athena_arm_interface");
  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);

  startInterface("arm_joint_1/position", 0.8);
  cycles(150);
  ASSERT_NEAR(motor->getJointPosition(), 0.8, 0.05);

  setCommand("arm_joint_1/position", -0.8);
  cycles(10);
  setCommand("arm_joint_1/position", std::numeric_limits<double>::quiet_NaN());
  const double at_nan = motor->getJointPosition();
  ASSERT_GT(at_nan, -0.6) << "the joint must still be on its way when the NaN arrives";
  cycles(50);

  const double held = motor->getJointPosition();
  EXPECT_NEAR(held, at_nan, 0.1) << "the joint kept moving towards the last finite target";
  cycles(25);
  EXPECT_NEAR(motor->getJointPosition(), held, 0.005) << "a held joint must not drift";
}

// On an async component the controllers can hold a handle while write() wants it. That cycle keeps
// the previous goal, the hold itself, which must not count as a command that ends the hold.
TEST_F(NaNHoldTest, ABusyHandleDoesNotMoveTheHold)
{
  startComponent("athena_arm_interface");
  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_2_ID);
  ASSERT_NE(motor, nullptr);
  startInterface("arm_joint_2/position", std::numeric_limits<double>::quiet_NaN());
  cycles(10);
  const double held = motor->getJointPosition();

  motor->moveByHand(0.3);
  {
    std::unique_lock<std::shared_mutex> busy(commands_.at("arm_joint_2/position")->get_mutex());
    ASSERT_EQ(cycle(), hardware_interface::return_type::OK);
  }
  cycles(50);
  EXPECT_NEAR(motor->getJointPosition(), held, 0.02) << "the hold moved to where the joint was pushed";
}

TEST_F(NaNHoldTest, VelocityNaNStopsTheJoint)
{
  startComponent("athena_flipper_interface");
  auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_NE(motor, nullptr);

  startInterface("flipper_fl_joint/velocity", 1.0);
  cycles(25);
  ASSERT_GT(std::abs(motor->getCurrentVelocity()), 0.1) << "the flipper should be moving before the NaN";

  setCommand("flipper_fl_joint/velocity", std::numeric_limits<double>::quiet_NaN());
  cycles(15);
  const double stopped_at = motor->getCurrentPosition();
  cycles(25);
  EXPECT_NEAR(motor->getCurrentPosition(), stopped_at, 0.02) << "the flipper kept its last velocity";
}

}  // namespace dynamixel_ros_control::test
