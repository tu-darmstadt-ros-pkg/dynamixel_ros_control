#include "direct_hardware_fixture.hpp"

#include <rcl/time.h>
#include <std_srvs/srv/trigger.hpp>

namespace dynamixel_ros_control::test {

using OnErrorTest = DirectHardwareTest;

// After on_error nothing writes this component's goals. A hang fault, a failed mode switch or a
// DEACTIVATE reach on_error with the bus and every motor healthy, and a velocity joint must not run
// on from there.
TEST_F(OnErrorTest, OnErrorStopsAVelocityJointOnAHealthyBus)
{
  startComponent("athena_flipper_interface");
  auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_NE(motor, nullptr);
  startInterface("flipper_fl_joint/velocity", 1.0);
  cycles(25);
  ASSERT_GT(std::abs(motor->getCurrentVelocity()), 0.1) << "the flipper should be moving before the error";

  EXPECT_EQ(hardware_->on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  for (int i = 0; i < 10; ++i) {
    MockDynamixelManager::instance().update(kPeriod);
  }
  const double stopped_at = motor->getCurrentPosition();
  for (int i = 0; i < 25; ++i) {
    MockDynamixelManager::instance().update(kPeriod);
  }
  EXPECT_NEAR(motor->getCurrentPosition(), stopped_at, 0.02) << "the flipper kept its last velocity after on_error";
}

// A sync write carries no status, so a bus that only fails the writes shows nowhere but in write()
TEST_F(OnErrorTest, PersistentWriteFailuresFaultTheComponentAtTheThreshold)
{
  startComponent("athena_flipper_interface");
  MockDynamixelManager::instance().setSyncWriteTransmissionFailure(true);
  for (unsigned int i = 1; i < COMM_ERROR_THRESHOLD; ++i) {
    ASSERT_EQ(cycle(), hardware_interface::return_type::OK) << "cycle " << i << " is below the threshold";
  }
  EXPECT_EQ(cycle(), hardware_interface::return_type::ERROR);
  MockDynamixelManager::instance().setSyncWriteTransmissionFailure(false);
}

// A healthy on_error still leaves the component unconfigured, and the LEDs must say so
TEST_F(OnErrorTest, OnErrorOnAHealthyBusLeavesTheActiveColours)
{
  startComponent("athena_arm_interface");
  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);
  ASSERT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "test premise: an active torqued joint is blue";
  ASSERT_EQ(motor->getLedRed(), COLOR_BLUE_R) << "test premise: an active torqued joint is blue";

  EXPECT_EQ(hardware_->on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  for (const uint8_t id : ARM_MOTOR_IDS) {
    const auto arm_motor = MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(arm_motor->getLedRed(), COLOR_PINK_R)
        << "motor " << static_cast<int>(id) << " still shows an active colour";
    EXPECT_EQ(arm_motor->getLedGreen(), COLOR_PINK_G)
        << "motor " << static_cast<int>(id) << " still shows an active colour";
    EXPECT_EQ(arm_motor->getLedBlue(), COLOR_PINK_B)
        << "motor " << static_cast<int>(id) << " still shows an active colour";
  }
}

// The reboot holds the bus while it waits for the motors, so it must not wait on a sim clock that may be paused
TEST_F(OnErrorTest, ARebootAnswersWhileTheSimClockIsPaused)
{
  startComponent("athena_arm_interface");
  rcl_clock_t* clock = params_.clock->get_clock_handle();
  ASSERT_EQ(rcl_enable_ros_time_override(clock), RCL_RET_OK);
  ASSERT_EQ(rcl_set_ros_time_override(clock, RCUTILS_S_TO_NS(1)), RCL_RET_OK);

  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  motor->setHardwareError(ERROR_OVERHEATING);
  int cycles_until_fault = 1;
  while (cycle() == hardware_interface::return_type::OK) {
    ASSERT_LE(++cycles_until_fault, 10) << "the hardware error was never reported";
  }
  ASSERT_EQ(hardware_->on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);

  const auto response =
      callService<std_srvs::srv::Trigger>("/athena_arm_interface_node/reboot",
                                          std::make_shared<std_srvs::srv::Trigger::Request>(), std::chrono::seconds(5));
  if (response == nullptr) {
    // Releases a reboot stuck on the paused clock, so the teardown can take the bus
    ASSERT_EQ(rcl_set_ros_time_override(clock, RCUTILS_S_TO_NS(100)), RCL_RET_OK);
    FAIL() << "the reboot did not answer while the sim clock was paused";
  }
  EXPECT_TRUE(response->success) << response->message;
  EXPECT_GT(motor->getRebootCount(), 0);
}

}  // namespace dynamixel_ros_control::test
