// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Motion Profile Register Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, Profile_RegistersSetOnStartup)
{
  // Verify that profile and drive_mode registers are written during on_configure
  // when configured via registers.* URDF parameters. The test URDF sets
  // drive_mode=4 (time-based), profile_velocity=20 (ms), profile_acceleration=8 (ms)
  // on arm_joint_1 (motor ID 11, PH model).
  std::this_thread::sleep_for(200ms);

  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);

  // Verify drive_mode was written (time-based profile bit)
  uint16_t drive_mode_addr = motor->getAddress("drive_mode");
  ASSERT_GT(drive_mode_addr, 0u) << "drive_mode register should exist on PH model";
  EXPECT_EQ(motor->read1Byte(drive_mode_addr), 4) << "drive_mode should be 4 (time-based profile)";

  // Verify profile_velocity register was written
  uint16_t profile_vel_addr = motor->getAddress("profile_velocity");
  ASSERT_GT(profile_vel_addr, 0u) << "profile_velocity register should exist on PH model";
  EXPECT_EQ(motor->read4ByteSigned(profile_vel_addr), 20) << "profile_velocity should be 20 (ms, raw)";

  // Verify profile_acceleration register was written
  uint16_t profile_acc_addr = motor->getAddress("profile_acceleration");
  ASSERT_GT(profile_acc_addr, 0u) << "profile_acceleration register should exist on PH model";
  EXPECT_EQ(motor->read4ByteSigned(profile_acc_addr), 8) << "profile_acceleration should be 8 (ms, raw)";

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

TEST_F(HardwareInterfaceTest, Profile_IndirectMappingsValidAfterReboot)
{
  // Regression test for a bug where the post-reboot path in DynamixelHardwareInterface
  // forgot to re-apply the SyncReadManager / SyncWriteManager indirect address
  // mappings. A real Dynamixel reboot wipes RAM (including the indirect-address
  // pointer registers); without re-writing them the very first post-reboot read
  // resolves through stale mappings, isHardwareOk() returns wrong values, and the
  // reboot service itself ends up reporting failure.
  //
  // The load-bearing assertion is `resp->success`: with the bug present (verified
  // by temporarily disabling rewriteIndirectAddresses), the reboot service returns
  // false with "Hardware still reports errors after reboot." With the fix, the
  // service succeeds. The pointer-byte check below is a belt-and-braces secondary
  // assertion that nails down the exact mechanism.

  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);

  // 1. Inject a hardware error and wait for the read cycle to pick it up.
  // The 1s wait matches the existing Profile_RegistersRestoredAfterReboot precedent
  // and gives the controller_manager update loop ample time at any reasonable rate.
  motor->setHardwareError(dynamixel_ros_control::ERROR_OVERLOAD);
  std::this_thread::sleep_for(1s);
  ASSERT_NE(motor->getHardwareError(), 0) << "Hardware error must remain set until reboot.";

  // 2. Call the reboot service. The handler is synchronous: it reboots the motor,
  // re-applies indirect mappings, restores initial register values, performs a
  // verifying read, and only then returns. So `resp->success == true` is itself
  // a strong assertion that the post-reboot read succeeded through valid indirect
  // mappings.
  auto reboot_client = tester_node_->create_test_client<std_srvs::srv::Trigger>("/athena_arm_interface/reboot");
  ASSERT_TRUE(reboot_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 10s;
  options.response_timeout = 10s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::Trigger>(reboot_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success) << "Reboot service must succeed. With the bug, the post-reboot read goes through "
                             << "stale indirect mappings, isHardwareOk() reports false, and the service returns "
                             << "'Hardware still reports errors after reboot.'  Service message: " << resp->message;

  // 3. Direct mock-state assertion (no timing dependency): after a successful reboot,
  // the indirect-address pointer registers must be non-zero. resetRAMRegisters in
  // the mock clears them; rewriteIndirectAddresses in the production code restores
  // them. Both happen synchronously inside the reboot service handler, so this
  // does not race with the update loop.
  // Address 168 is indirect_address_1 on PH series (devices/models/PH.yaml).
  constexpr uint16_t INDIRECT_ADDRESS_1 = 168;
  EXPECT_NE(motor->read2Byte(INDIRECT_ADDRESS_1), 0)
      << "indirect_address_1 must be re-written after reboot; reading 0 means the "
      << "SyncReadManager mapping was not restored and post-reboot reads target "
      << "address 0 (model_number) instead of the intended register.";
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  dynamixel_ros_control::test::run_tests_and_exit(argc, argv);
}
