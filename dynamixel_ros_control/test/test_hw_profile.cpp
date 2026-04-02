// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Motion Profile Register Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, Profile_RegistersSetOnStartup)
{
  // Verify that profile_velocity and profile_acceleration registers are written
  // during on_configure when configured via registers.* URDF parameters.
  // The test URDF sets registers.profile_velocity=1.0 and registers.profile_acceleration=0.5
  // on arm_joint_1 (motor ID 11, PH model).
  std::this_thread::sleep_for(200ms);

  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);

  // Verify profile_velocity register was written
  uint16_t profile_vel_addr = motor->getAddress("profile_velocity");
  ASSERT_GT(profile_vel_addr, 0u) << "profile_velocity register should exist on PH model";
  int32_t profile_vel = motor->read4ByteSigned(profile_vel_addr);
  EXPECT_GT(profile_vel, 0) << "profile_velocity should be nonzero after initial write (set to 1.0 rad/s)";

  // Verify profile_acceleration register was written
  uint16_t profile_acc_addr = motor->getAddress("profile_acceleration");
  ASSERT_GT(profile_acc_addr, 0u) << "profile_acceleration register should exist on PH model";
  int32_t profile_acc = motor->read4ByteSigned(profile_acc_addr);
  EXPECT_GT(profile_acc, 0) << "profile_acceleration should be nonzero after initial write (set to 0.5 rad/s2)";

  // Verify motors without profile config have default (0) values
  auto motor2 = MockDynamixelManager::instance().getMotor(ARM_JOINT_2_ID);
  ASSERT_NE(motor2, nullptr);
  uint16_t motor2_vel_addr = motor2->getAddress("profile_velocity");
  if (motor2_vel_addr > 0) {
    EXPECT_EQ(motor2->read4ByteSigned(motor2_vel_addr), 0)
        << "arm_joint_2 should have default profile_velocity (no URDF config)";
  }
}

TEST_F(HardwareInterfaceTest, Profile_RegistersRestoredAfterReboot)
{
  // Verify that RAM registers configured via registers.* are restored after a motor reboot.
  // This tests the fix: writeInitialValues() is now called after reboot.
  std::this_thread::sleep_for(200ms);

  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);

  // 1. Record initial profile register values
  uint16_t profile_vel_addr = motor->getAddress("profile_velocity");
  uint16_t profile_acc_addr = motor->getAddress("profile_acceleration");
  ASSERT_GT(profile_vel_addr, 0u);
  ASSERT_GT(profile_acc_addr, 0u);

  int32_t initial_profile_vel = motor->read4ByteSigned(profile_vel_addr);
  int32_t initial_profile_acc = motor->read4ByteSigned(profile_acc_addr);
  EXPECT_GT(initial_profile_vel, 0) << "profile_velocity should be set from URDF";
  EXPECT_GT(initial_profile_acc, 0) << "profile_acceleration should be set from URDF";

  // 2. Inject hardware error to trigger reboot
  motor->setHardwareError(dynamixel_ros_control::ERROR_OVERLOAD);
  std::this_thread::sleep_for(1s);

  // 3. Call reboot service
  auto reboot_client = tester_node_->create_test_client<std_srvs::srv::Trigger>("/athena_arm_interface/reboot");
  ASSERT_TRUE(reboot_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 10s;
  options.response_timeout = 10s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::Trigger>(reboot_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success) << "Reboot should succeed: " << resp->message;

  std::this_thread::sleep_for(500ms);

  // 4. Verify profile registers were restored after reboot
  int32_t restored_profile_vel = motor->read4ByteSigned(profile_vel_addr);
  int32_t restored_profile_acc = motor->read4ByteSigned(profile_acc_addr);

  EXPECT_EQ(restored_profile_vel, initial_profile_vel)
      << "profile_velocity should be restored after reboot. Initial: " << initial_profile_vel
      << ", After reboot: " << restored_profile_vel;
  EXPECT_EQ(restored_profile_acc, initial_profile_acc)
      << "profile_acceleration should be restored after reboot. Initial: " << initial_profile_acc
      << ", After reboot: " << restored_profile_acc;
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
