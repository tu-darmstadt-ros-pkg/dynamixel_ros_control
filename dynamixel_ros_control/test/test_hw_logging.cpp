#include <rcutils/logging.h>

#include <string>
#include <tuple>

#include "direct_hardware_fixture.hpp"
#include "problem_capture.hpp"

namespace dynamixel_ros_control::test {

// A failure on the cyclic path repeats at the control rate, so it is reported once per second
using LoggingTest = DirectHardwareTest;

TEST_F(LoggingTest, DebugOnOneComponentLeavesTheOthersAlone)
{
  initComponent("athena_flipper_interface");
  params_.hardware_info.hardware_parameters["debug"] = "true";
  ASSERT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(rcutils_logging_get_logger_effective_level("dynamixel_ros_control.athena_flipper_interface"),
            RCUTILS_LOG_SEVERITY_DEBUG);
  EXPECT_NE(rcutils_logging_get_logger_effective_level("dynamixel_ros_control.athena_arm_interface"),
            RCUTILS_LOG_SEVERITY_DEBUG)
      << "debug on the flippers switched the arm to DEBUG too";
}

TEST_F(LoggingTest, AFailingBusIsReportedOncePerSecond)
{
  startComponent("athena_flipper_interface");
  const ProblemCapture capture;
  MockDynamixelManager::instance().setGlobalCommunicationError(true);
  // Below the consecutive error threshold, so every cycle still returns OK
  cycles(10);
  MockDynamixelManager::instance().setGlobalCommunicationError(false);
  // One line for the failed read. The sync write carries no status, so it does not fail.
  EXPECT_EQ(capture.count(), 1u) << "a failing bus logged at the control rate, or not at all";
  EXPECT_FALSE(capture.loggerOf("Sync read failed").empty()) << "the failed read was not reported";
}

TEST_F(LoggingTest, ACommandWithoutARawValueIsReportedOnce)
{
  startComponent("athena_flipper_interface");
  startInterface("flipper_fl_joint/position", 0.0);
  cycles(2);
  const ProblemCapture capture;
  setCommand("flipper_fl_joint/position", 1e15);
  cycles(10);
  EXPECT_EQ(capture.count(), 1u) << "an unencodable command logged at the control rate";
}

TEST_F(LoggingTest, AControlModeThatCannotBeReadIsReported)
{
  initComponent("athena_flipper_interface");
  ASSERT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::SUCCESS);
  std::ignore = hardware_->on_export_state_interfaces();
  std::ignore = hardware_->on_export_command_interfaces();
  ASSERT_EQ(hardware_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  configured_ = true;
  const auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  motor->setRegisterReadFaultCount("operating_mode", 100);
  const ProblemCapture capture;
  EXPECT_EQ(hardware_->on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
  EXPECT_FALSE(capture.loggerOf("attempts. Aborting activation").empty())
      << "the last failed control mode read was not logged";
}

TEST_F(LoggingTest, AnLedFailureNamesItsComponent)
{
  startComponent("athena_flipper_interface");
  const ProblemCapture capture;
  MockDynamixelManager::instance().setSyncWriteTransmissionFailure(true);
  EXPECT_EQ(hardware_->on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  MockDynamixelManager::instance().setSyncWriteTransmissionFailure(false);
  configured_ = false;
  EXPECT_EQ(capture.loggerOf("Failed to write LED colors"), "dynamixel_ros_control.athena_flipper_interface");
}

}  // namespace dynamixel_ros_control::test
