#include <rcutils/logging.h>

#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>
#include <vector>

#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {

namespace {

std::mutex warnings_mutex;
std::vector<std::string> warnings;

void captureWarnings(const rcutils_log_location_t* /*location*/, int severity, const char* /*name*/,
                     rcutils_time_point_value_t /*timestamp*/, const char* format, va_list* args)
{
  if (severity != RCUTILS_LOG_SEVERITY_WARN) {
    return;
  }
  char message[1024];
  va_list copy;
  va_copy(copy, *args);
  std::vsnprintf(message, sizeof(message), format, copy);
  va_end(copy);
  const std::lock_guard<std::mutex> lock(warnings_mutex);
  warnings.emplace_back(message);
}

/// Collects warnings while it lives
struct WarningCapture
{
  WarningCapture()
  {
    {
      const std::lock_guard<std::mutex> lock(warnings_mutex);
      warnings.clear();
    }
    rcutils_logging_set_output_handler(captureWarnings);
  }
  ~WarningCapture()
  {
    rcutils_logging_set_output_handler(previous);
  }
  WarningCapture(const WarningCapture&) = delete;
  WarningCapture& operator=(const WarningCapture&) = delete;

  rcutils_logging_output_handler_t previous{rcutils_logging_get_output_handler()};
};

/// The flippers with max_current in place of current
std::string withMaxCurrent(std::string urdf)
{
  const std::string from = "<command_interface name=\"current\"/>";
  for (auto pos = urdf.find(from); pos != std::string::npos; pos = urdf.find(from, pos)) {
    urdf.replace(pos, from.size(), "<command_interface name=\"max_current\"/>");
  }
  return urdf;
}

/// The captured warning containing @p text, empty if there is none
std::string warningWith(const std::string& text)
{
  const std::lock_guard<std::mutex> lock(warnings_mutex);
  for (const auto& message : warnings) {
    if (message.find(text) != std::string::npos) {
      return message;
    }
  }
  return "";
}

const std::vector<std::string> FLIPPER_JOINTS{"flipper_fl_joint", "flipper_fr_joint", "flipper_bl_joint",
                                              "flipper_br_joint"};

}  // namespace

// athena's flippers are PRO H54 motors, which have no bus watchdog, while the bus asks for one
class WatchdogUnsupportedTest : public DirectHardwareTest
{
protected:
  void adjustInfo(hardware_interface::HardwareInfo& info) override
  {
    for (auto& joint : info.joints) {
      joint.parameters["model_number"] = "54024";
    }
    if (allow_velocity_) {
      info.hardware_parameters["allow_velocity_without_watchdog"] = "true";
    }
  }

  bool allow_velocity_{false};
};

TEST_F(WatchdogUnsupportedTest, ConfigureWarnsAboutEveryJointWithoutAWatchdog)
{
  {
    const WarningCapture capture;
    // Fails unless configure and activate succeed, as a missing watchdog is no reason to refuse
    startComponent("athena_flipper_interface");
  }

  const std::string warning = warningWith("no bus watchdog");
  ASSERT_FALSE(warning.empty()) << "configure did not warn";
  for (const auto& joint : FLIPPER_JOINTS) {
    EXPECT_NE(warning.find(joint), std::string::npos) << joint << " is missing from: " << warning;
  }
}

// Nothing would stop the motor once the goals stop coming, so velocity mode is refused
TEST_F(WatchdogUnsupportedTest, VelocityModeIsRefused)
{
  startComponent("athena_flipper_interface");
  EXPECT_NE(hardware_->switchRefusal({"flipper_fl_joint/velocity"}, {}).find("velocity mode without a bus watchdog"),
            std::string::npos);
  EXPECT_EQ(hardware_->prepare_command_mode_switch({"flipper_fl_joint/velocity"}, {}),
            hardware_interface::return_type::ERROR);
  EXPECT_EQ(hardware_->perform_command_mode_switch({"flipper_fl_joint/velocity"}, {}),
            hardware_interface::return_type::ERROR);
}

// A cap does not make velocity mode safe
TEST_F(WatchdogUnsupportedTest, VelocityModeWithMaxCurrentIsRefused)
{
  startComponent("athena_flipper_interface", withMaxCurrent);
  EXPECT_NE(hardware_->switchRefusal({"flipper_fl_joint/velocity", "flipper_fl_joint/max_current"}, {})
                .find("velocity mode without a bus watchdog"),
            std::string::npos);
}

// A position mode holds its last goal when the bus goes silent, also with a cap beside it
TEST_F(WatchdogUnsupportedTest, PositionModesAreAccepted)
{
  startComponent("athena_flipper_interface", withMaxCurrent);
  EXPECT_EQ(hardware_->prepare_command_mode_switch({"flipper_fl_joint/position"}, {}),
            hardware_interface::return_type::OK);
  EXPECT_EQ(hardware_->prepare_command_mode_switch({"flipper_fl_joint/position", "flipper_fl_joint/max_current"}, {}),
            hardware_interface::return_type::OK);
}

TEST_F(WatchdogUnsupportedTest, OverrideAcceptsVelocityModeAndWarns)
{
  allow_velocity_ = true;
  {
    const WarningCapture capture;
    startComponent("athena_flipper_interface");
  }
  const std::string warning = warningWith("allow_velocity_without_watchdog");
  ASSERT_FALSE(warning.empty()) << "configure did not warn about the override";
  for (const auto& joint : FLIPPER_JOINTS) {
    EXPECT_NE(warning.find(joint), std::string::npos) << joint << " is missing from: " << warning;
  }

  startInterface("flipper_fl_joint/velocity", 0.0);
  auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  EXPECT_EQ(motor->read1Byte(motor->getAddress("operating_mode")), VELOCITY);
  cycles(5);
}

// athena's arm bus disables the watchdog, which leaves the same joints unprotected as a missing register
class WatchdogDisabledTest : public DirectHardwareTest
{
protected:
  void adjustInfo(hardware_interface::HardwareInfo& info) override
  {
    info.hardware_parameters["bus_watchdog_cycles"] = "0";
    if (allow_velocity_) {
      info.hardware_parameters["allow_velocity_without_watchdog"] = "true";
    }
  }

  bool allow_velocity_{false};
};

TEST_F(WatchdogDisabledTest, VelocityModeIsRefused)
{
  startComponent("athena_arm_interface");
  EXPECT_EQ(hardware_->prepare_command_mode_switch({"arm_joint_2/velocity"}, {}),
            hardware_interface::return_type::ERROR);
}

// The claims athena's arm and gripper controllers make, a position mode with a current cap, on
// athena's RH-P12-RN(A) gripper
TEST_F(WatchdogDisabledTest, PositionWithMaxCurrentIsAccepted)
{
  startComponent("athena_arm_interface", [](std::string urdf) {
    const std::string anchor = "<param name=\"id\">18</param>";
    return urdf.insert(urdf.find(anchor) + anchor.size(), "<param name=\"model_number\">35074</param>");
  });
  EXPECT_EQ(hardware_->prepare_command_mode_switch({"arm_joint_2/position", "arm_joint_2/max_current",
                                                    "gripper_servo_joint/position", "gripper_servo_joint/max_current"},
                                                   {}),
            hardware_interface::return_type::OK);
}

TEST_F(WatchdogDisabledTest, OverrideAcceptsVelocityMode)
{
  allow_velocity_ = true;
  startComponent("athena_arm_interface");
  startInterface("arm_joint_2/velocity", 0.0);
  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_2_ID);
  EXPECT_EQ(motor->read1Byte(motor->getAddress("operating_mode")), VELOCITY);
  cycles(5);
}

}  // namespace dynamixel_ros_control::test
