// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Hardware Error Handling Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, HardwareError_LEDTurnsRedOnError)
{
  // Test that when a motor has a hardware error, its LED turns red
  // while other motors indicate the error state (orange for e-stop).
  //
  // IMPORTANT: When read() returns ERROR due to hardware error:
  // 1. on_error() is called which sets red LED for error motor, orange for others (e-stop)
  // 2. Controller_manager tries to deactivate the HW interface
  // 3. on_deactivate may fail if it can't set torque (due to hardware error)
  //
  // The motor with error should have red LED, others should have orange (e-stop active).

  // 1. Verify initial LED state (blue - active with torque on)
  std::this_thread::sleep_for(500ms);
  for (uint8_t id : ARM_MOTOR_IDS) {
    verifyLEDColor(id, COLOR_BLUE_R, COLOR_BLUE_G, COLOR_BLUE_B);
  }

  // 2. Inject hardware error on one motor
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor1, nullptr);
  motor1->setHardwareError(dynamixel_ros_control::ERROR_OVERLOAD);

  // 3. Wait for error to be detected
  // The hardware interface's read() should detect the error
  std::this_thread::sleep_for(1s);

  // 4. Verify the error was detected - motor should have hardware error set
  EXPECT_NE(motor1->getHardwareError(), 0) << "Motor 1 should have hardware error set";

  // 5. Verify motor with error has red LED
  verifyLEDColor(ARM_JOINT_1_ID, COLOR_RED_R, COLOR_RED_G, COLOR_RED_B);

  // 6. Other motors should have orange LED (e-stop active)
  for (uint8_t id = ARM_JOINT_2_ID; id <= ARM_JOINT_7_ID; ++id) {
    verifyLEDColor(id, COLOR_ORANGE_R, COLOR_ORANGE_G, COLOR_ORANGE_B);
  }

  // Cleanup - clear the error
  motor1->clearHardwareError();
}

TEST_F(HardwareInterfaceTest, HardwareError_EStopActivatedOnError)
{
  // Test that when a hardware error is detected, e-stop is activated
  // to prevent other actuators from moving

  // 1. Activate position controller and send a command
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(pub, 5s));

  // Move to a known position
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // Record positions before error
  std::map<uint8_t, double> positions_before;
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_before[id] = motor->getCurrentPosition();
  }

  // 2. Inject hardware error on one motor
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor1, nullptr);
  motor1->setHardwareError(dynamixel_ros_control::ERROR_OVERLOAD);

  // 3. Wait for error to be detected
  std::this_thread::sleep_for(1s);

  // 4. Try to send a new command - motors should NOT move due to e-stop
  cmd.data = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // 5. Verify motors did NOT move significantly (e-stop should prevent movement)
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double position_change = std::abs(motor->getCurrentPosition() - positions_before[id]);
    EXPECT_LT(position_change, 0.1) << "Motor " << (int) id
                                    << " should not move while e-stop is active due to hardware error";
  }

  // Cleanup
  motor1->clearHardwareError();
}

TEST_F(HardwareInterfaceTest, HardwareError_RebootServiceClearsErrorAndReleasesEStop)
{
  // Test that calling the reboot service clears the hardware error
  // and releases the e-stop
  //
  // NOTE: After hardware error, the HW interface is deactivated by controller_manager.
  // The reboot service still works but the HW interface remains in inactive state.
  // LEDs will be pink (inactive) after reboot.

  // 1. Inject hardware error
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor1, nullptr);
  motor1->setHardwareError(dynamixel_ros_control::ERROR_OVERLOAD);

  // 2. Wait for error to be detected
  std::this_thread::sleep_for(1s);

  // Verify error was detected (HW interface will be deactivated, LED will be pink)
  EXPECT_NE(motor1->getHardwareError(), 0) << "Motor 1 should have hardware error set";

  // 3. Call reboot service
  auto reboot_client = tester_node_->create_test_client<std_srvs::srv::Trigger>("/athena_arm_interface/reboot");
  ASSERT_TRUE(reboot_client->wait_for_service(*executor_, 5s)) << "Reboot service not available";

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 10s;
  options.response_timeout = 10s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::Trigger>(reboot_client->get(), request, *executor_, options);

  ASSERT_NE(resp, nullptr) << "Reboot service call failed";
  EXPECT_TRUE(resp->success) << "Reboot should succeed: " << resp->message;

  // 4. Verify motor was rebooted
  EXPECT_GT(motor1->getRebootCount(), 0) << "Motor should have been rebooted";

  // 5. Wait for LED update
  std::this_thread::sleep_for(500ms);

  // 6. Verify hardware error is cleared
  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  EXPECT_EQ(motor->getHardwareError(), 0) << "Hardware error should be cleared after reboot";

  // LED will be pink because HW interface was deactivated by controller_manager
  // after the initial hardware error was detected
  verifyLEDColor(ARM_JOINT_1_ID, COLOR_PINK_R, COLOR_PINK_G, COLOR_PINK_B);
}

TEST_F(HardwareInterfaceTest, HardwareError_MultipleMotorsWithErrors)
{
  // Test that multiple motors with hardware errors all get red LEDs
  // while others get orange (e-stop active)

  // 1. Inject hardware errors on multiple motors
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  auto motor3 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_3_ID);
  auto motor5 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_5_ID);
  ASSERT_NE(motor1, nullptr);
  ASSERT_NE(motor3, nullptr);
  ASSERT_NE(motor5, nullptr);

  motor1->setHardwareError(dynamixel_ros_control::ERROR_OVERLOAD);
  motor3->setHardwareError(dynamixel_ros_control::ERROR_OVERHEATING);
  motor5->setHardwareError(dynamixel_ros_control::ERROR_INPUT_VOLTAGE);

  // 2. Wait for errors to be detected
  std::this_thread::sleep_for(1s);

  // 3. Verify motors have their hardware errors set
  EXPECT_NE(motor1->getHardwareError(), 0) << "Motor 1 should have hardware error set";
  EXPECT_NE(motor3->getHardwareError(), 0) << "Motor 3 should have hardware error set";
  EXPECT_NE(motor5->getHardwareError(), 0) << "Motor 5 should have hardware error set";

  // Motors without injected errors should have no hardware error
  auto motor2 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_2_ID);
  auto motor4 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_4_ID);
  EXPECT_EQ(motor2->getHardwareError(), 0) << "Motor 2 should have no hardware error";
  EXPECT_EQ(motor4->getHardwareError(), 0) << "Motor 4 should have no hardware error";

  // 4. Verify motors with errors have red LEDs
  verifyLEDColor(ARM_JOINT_1_ID, COLOR_RED_R, COLOR_RED_G, COLOR_RED_B);
  verifyLEDColor(ARM_JOINT_3_ID, COLOR_RED_R, COLOR_RED_G, COLOR_RED_B);
  verifyLEDColor(ARM_JOINT_5_ID, COLOR_RED_R, COLOR_RED_G, COLOR_RED_B);

  // 5. Motors without errors have orange LEDs (e-stop active)
  verifyLEDColor(ARM_JOINT_2_ID, COLOR_ORANGE_R, COLOR_ORANGE_G, COLOR_ORANGE_B);
  verifyLEDColor(ARM_JOINT_4_ID, COLOR_ORANGE_R, COLOR_ORANGE_G, COLOR_ORANGE_B);
  verifyLEDColor(ARM_JOINT_6_ID, COLOR_ORANGE_R, COLOR_ORANGE_G, COLOR_ORANGE_B);
  verifyLEDColor(ARM_JOINT_7_ID, COLOR_ORANGE_R, COLOR_ORANGE_G, COLOR_ORANGE_B);

  // Cleanup
  motor1->clearHardwareError();
  motor3->clearHardwareError();
  motor5->clearHardwareError();
}

TEST_F(HardwareInterfaceTest, HardwareError_HardwareInterfaceDoesNotCrash)
{
  // Test that the hardware interface continues to run after a hardware error
  // (i.e., on_error returns SUCCESS, not FAILURE)

  // 1. Record initial state
  auto list_hw_request = std::make_shared<ListHardwareInterfaces::Request>();
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto hw_resp = hector_testing_utils::call_service<ListHardwareInterfaces>(list_hw_client_->get(), list_hw_request,
                                                                            *executor_, options);
  ASSERT_NE(hw_resp, nullptr);

  // Find arm interface state
  bool arm_interface_active = false;
  for (const auto& iface : hw_resp->command_interfaces) {
    if (iface.name.find("arm_joint_1/position") != std::string::npos) {
      arm_interface_active = true;
      break;
    }
  }
  ASSERT_TRUE(arm_interface_active) << "Arm interface should be active before test";

  // 2. Inject hardware error
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor1, nullptr);
  motor1->setHardwareError(dynamixel_ros_control::ERROR_OVERLOAD);

  // 3. Wait for error to be detected and handled
  std::this_thread::sleep_for(2s);

  // 4. Verify hardware interface is still running (command interfaces still available)
  hw_resp = hector_testing_utils::call_service<ListHardwareInterfaces>(list_hw_client_->get(), list_hw_request,
                                                                       *executor_, options);
  ASSERT_NE(hw_resp, nullptr);

  arm_interface_active = false;
  for (const auto& iface : hw_resp->command_interfaces) {
    if (iface.name.find("arm_joint_1/position") != std::string::npos) {
      arm_interface_active = true;
      break;
    }
  }
  EXPECT_TRUE(arm_interface_active) << "Arm interface should still be running after hardware error";

  // 5. Verify we can still call services on the interface
  auto torque_client = createTorqueClient("athena_arm_interface");
  // Service should still be available
  EXPECT_TRUE(torque_client->wait_for_service(*executor_, 2s)) << "Torque service should still be available";

  // Cleanup
  motor1->clearHardwareError();
}

TEST_F(HardwareInterfaceTest, HardwareError_RebootOnlyAffectedMotors)
{
  // Test that reboot only reboots motors with hardware errors, not all motors

  // 1. Inject hardware error on specific motors
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  auto motor3 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_3_ID);
  auto motor2 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_2_ID);
  ASSERT_NE(motor1, nullptr);
  ASSERT_NE(motor2, nullptr);
  ASSERT_NE(motor3, nullptr);

  // Reset reboot counters
  motor1->resetRebootCount();
  motor2->resetRebootCount();
  motor3->resetRebootCount();

  // Only motors 1 and 3 have errors
  motor1->setHardwareError(dynamixel_ros_control::ERROR_OVERLOAD);
  motor3->setHardwareError(dynamixel_ros_control::ERROR_OVERLOAD);

  // 2. Wait for errors to be detected
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
  EXPECT_TRUE(resp->success);

  // 4. Verify only motors with errors were rebooted
  EXPECT_GT(motor1->getRebootCount(), 0) << "Motor 1 should have been rebooted (had error)";
  EXPECT_GT(motor3->getRebootCount(), 0) << "Motor 3 should have been rebooted (had error)";
  EXPECT_EQ(motor2->getRebootCount(), 0) << "Motor 2 should NOT have been rebooted (no error)";
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
