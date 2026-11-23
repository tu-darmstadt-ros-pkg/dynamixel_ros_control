#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {

using LifecycleClaimsTest = DirectHardwareTest;

// A DEACTIVATE from write(), or set_hardware_component_state to inactive, deactivates the component
// without a mode switch that stops its claimers first. Activated again, it has no claimer, so its
// joints must come up in their position mode, not in the mode the departed claimer used.
TEST_F(LifecycleClaimsTest, ReactivationForgetsTheClaimsOfTheLastActivation)
{
  startComponent("athena_flipper_interface");
  auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_NE(motor, nullptr);
  startInterface("flipper_fl_joint/velocity", 0.0);
  cycles(5);
  ASSERT_EQ(motor->read1Byte(motor->getAddress("operating_mode")), static_cast<uint8_t>(VELOCITY));

  ASSERT_EQ(hardware_->on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware_->on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);

  EXPECT_NE(motor->read1Byte(motor->getAddress("operating_mode")), static_cast<uint8_t>(VELOCITY))
      << "the joint came back in the velocity mode of a claimer that is gone";
}

// With torque_off_on_shutdown a healthy deactivation leaves every motor limp, and a cleanup leaves
// the component ready to be configured and activated again
TEST_F(LifecycleClaimsTest, TorqueOffOnShutdownDeEnergisesEveryMotorAndACleanupReconfigures)
{
  startComponent("athena_arm_interface", [](std::string urdf) {
    const std::string anchor = "<param name=\"torque_off_on_shutdown\">false</param>";
    const auto pos = urdf.find(anchor, urdf.find("<ros2_control name=\"athena_arm_interface\""));
    EXPECT_NE(pos, std::string::npos);
    return urdf.replace(pos, anchor.size(), "<param name=\"torque_off_on_shutdown\">true</param>");
  });
  std::vector<uint8_t> ids = ARM_MOTOR_IDS;
  ids.push_back(GRIPPER_ID);
  const auto torque = [](const uint8_t id) {
    const auto motor = MockDynamixelManager::instance().getMotor(id);
    return motor->read1Byte(motor->getAddress("torque_enable"));
  };
  for (const uint8_t id : ids) {
    ASSERT_EQ(torque(id), 1) << "test premise: torque_on_startup energised motor " << static_cast<int>(id);
  }

  ASSERT_EQ(hardware_->on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  for (const uint8_t id : ids) {
    EXPECT_EQ(torque(id), 0) << "motor " << static_cast<int>(id) << " kept its torque on deactivation";
  }

  ASSERT_EQ(hardware_->on_cleanup(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  configured_ = false;
  ASSERT_EQ(hardware_->on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  configured_ = true;
  ASSERT_EQ(hardware_->on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  cycles(5);
  for (const uint8_t id : ids) {
    EXPECT_EQ(torque(id), 1) << "motor " << static_cast<int>(id) << " did not come back after the cleanup";
  }
}

}  // namespace dynamixel_ros_control::test
