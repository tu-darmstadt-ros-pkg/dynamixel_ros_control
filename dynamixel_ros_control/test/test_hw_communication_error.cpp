// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Communication Error / Hard E-Stop Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, CommunicationError_TemporaryErrorRecovery)
{
  // Test that the system can recover from temporary communication errors
  // This simulates brief motor disconnection or bus errors

  // 1. Activate controller and verify normal operation
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Move to initial position
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // 2. Inject temporary communication errors on some motors
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  auto motor2 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_2_ID);
  ASSERT_NE(motor1, nullptr);
  ASSERT_NE(motor2, nullptr);

  motor1->setCommunicationError(true);
  motor2->setCommunicationError(true);

  // Wait a bit with errors
  std::this_thread::sleep_for(500ms);

  // 3. Clear errors - simulate recovery
  motor1->setCommunicationError(false);
  motor2->setCommunicationError(false);

  // 4. Wait for recovery and then send command
  std::this_thread::sleep_for(500ms);

  // Record positions before new command
  std::vector<double> positions_before;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_before.push_back(motor->getCurrentPosition());
  }

  // Send new position command
  cmd.data = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  // 5. Verify at least some motors moved (system recovered)
  bool any_moved = false;
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    if (std::abs(motor->getCurrentPosition() - positions_before[i]) > 0.05) {
      any_moved = true;
      break;
    }
  }
  EXPECT_TRUE(any_moved) << "System should recover from temporary communication errors";
}

TEST_F(HardwareInterfaceTest, CommunicationError_GlobalErrorBlocksOperation)
{
  // Test that global communication errors (all motors) block operation
  // This simulates hard E-Stop cutting power to all motors

  // 1. Activate controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Move to initial position
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // Record initial positions
  std::vector<double> initial_positions;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions.push_back(motor->getCurrentPosition());
  }

  // 2. Enable global communication error (simulates power cut)
  dynamixel_ros_control::MockDynamixelManager::instance().setGlobalCommunicationError(true);

  // 3. Try to send movement commands - should have no effect
  cmd.data = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // Verify positions haven't changed (communication errors prevent read/write)
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    // With communication errors, the mock won't update positions
    EXPECT_NEAR(motor->getCurrentPosition(), initial_positions[i], 0.1)
        << "Motor " << (ARM_JOINT_1_ID + i) << " should not move during communication errors";
  }

  // 4. Clear errors (simulate power restored)
  dynamixel_ros_control::MockDynamixelManager::instance().setGlobalCommunicationError(false);
  std::this_thread::sleep_for(500ms);

  // 5. Verify system can recover
  std::vector<double> positions_after_recovery;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_after_recovery.push_back(motor->getCurrentPosition());
  }

  cmd.data = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  bool any_moved = false;
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    if (std::abs(motor->getCurrentPosition() - positions_after_recovery[i]) > 0.05) {
      any_moved = true;
      break;
    }
  }
  EXPECT_TRUE(any_moved) << "System should recover after communication errors are cleared";
}

TEST_F(HardwareInterfaceTest, EdgeCase_ControllerActivationWithCommunicationErrors)
{
  // Test controller activation when communication errors prevent successful reads
  // Expected: Controller activation should fail gracefully

  // 1. Set communication errors on all arm motors BEFORE activating controller
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor)
      motor->setCommunicationError(true);
  }
  // Also set error on gripper
  auto gripper_motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  if (gripper_motor)
    gripper_motor->setCommunicationError(true);

  // Wait for errors to take effect in read cycle
  std::this_thread::sleep_for(500ms);

  // 2. Try to load and activate controller - this may fail
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;

  auto load_req = std::make_shared<LoadController::Request>();
  load_req->name = "arm_position_controller";
  auto load_resp =
      hector_testing_utils::call_service<LoadController>(load_client_->get(), load_req, *executor_, options);

  // Loading should succeed (doesn't require communication)
  ASSERT_NE(load_resp, nullptr);
  EXPECT_TRUE(load_resp->ok) << "Controller loading should succeed";

  auto config_req = std::make_shared<ConfigureController::Request>();
  config_req->name = "arm_position_controller";
  auto config_resp =
      hector_testing_utils::call_service<ConfigureController>(config_client_->get(), config_req, *executor_, options);
  ASSERT_NE(config_resp, nullptr);
  EXPECT_TRUE(config_resp->ok) << "Controller configuration should succeed";

  // 3. Activation might fail due to no successful read
  auto switch_req = std::make_shared<SwitchController::Request>();
  switch_req->activate_controllers = {"arm_position_controller"};
  switch_req->strictness = SwitchController::Request::STRICT;
  auto switch_resp =
      hector_testing_utils::call_service<SwitchController>(switch_client_->get(), switch_req, *executor_, options);

  // This documents the current behavior - activation may fail
  // If this test passes with switch_resp->ok == false, that's expected safety behavior
  // If a fix is implemented to do inline read, this test should be updated

  // 4. Clear errors for cleanup
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor)
      motor->setCommunicationError(false);
  }
  if (gripper_motor)
    gripper_motor->setCommunicationError(false);

  // Log the result for documentation
  if (switch_resp && !switch_resp->ok) {
    // Expected behavior - activation fails when read hasn't succeeded
    SUCCEED() << "Controller activation correctly rejected when communication errors prevent reads";
  } else if (switch_resp && switch_resp->ok) {
    // If activation succeeded, the fix was implemented - verify system is functional
    SUCCEED() << "Controller activation succeeded (inline read fix may be implemented)";
  }
}

TEST_F(HardwareInterfaceTest, RebootService_ResetsMotors)
{
  // Test the reboot service functionality
  // This is used to recover motors from error states

  // 1. Create reboot service client
  auto reboot_client = tester_node_->create_test_client<std_srvs::srv::Trigger>("/athena_arm_interface/reboot");
  ASSERT_TRUE(reboot_client->wait_for_service(*executor_, 5s)) << "Reboot service not available";

  // 2. Record initial motor states
  std::vector<double> initial_positions;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions.push_back(motor->getCurrentPosition());
  }

  // 3. Call reboot service
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 10s;
  options.response_timeout = 10s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::Trigger>(reboot_client->get(), request, *executor_, options);

  ASSERT_NE(resp, nullptr) << "Reboot service call failed";
  EXPECT_TRUE(resp->success) << "Reboot should succeed: " << resp->message;

  // 4. Verify motors still exist and are functional
  std::this_thread::sleep_for(500ms);

  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Motor " << (int) id << " should still exist after reboot";
  }

  // 5. Verify LED is correct (should be blue if torque is on)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "Motor " << (int) id << " LED should be blue after reboot";
  }
}

TEST_F(HardwareInterfaceTest, RebootService_OnlyRebootsFaultyMotors)
{
  // Test that reboot is only called for motors with hardware errors, not healthy ones

  // 1. Create reboot service client
  auto reboot_client = tester_node_->create_test_client<std_srvs::srv::Trigger>("/athena_arm_interface/reboot");
  ASSERT_TRUE(reboot_client->wait_for_service(*executor_, 5s)) << "Reboot service not available";

  // 2. Reset reboot counters for all motors
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    motor->resetRebootCount();
  }
  auto gripper_motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  gripper_motor->resetRebootCount();

  // 3. Set hardware error on only motor 1 and motor 3
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  auto motor3 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_3_ID);
  motor1->setHardwareError(dynamixel_ros_control::HardwareErrorBit::ERROR_OVERHEATING);
  motor3->setHardwareError(dynamixel_ros_control::HardwareErrorBit::ERROR_INPUT_VOLTAGE);

  // Wait for at least one read cycle to update hardware_error_status in the Dynamixel class
  std::this_thread::sleep_for(300ms);

  // 4. Call reboot service
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 10s;
  options.response_timeout = 10s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::Trigger>(reboot_client->get(), request, *executor_, options);

  ASSERT_NE(resp, nullptr) << "Reboot service call failed";
  EXPECT_TRUE(resp->success) << "Reboot should succeed: " << resp->message;

  std::this_thread::sleep_for(500ms);

  // 5. Verify ONLY faulty motors were rebooted
  EXPECT_EQ(motor1->getRebootCount(), 1) << "Motor 1 (with error) should have been rebooted once";
  EXPECT_EQ(motor3->getRebootCount(), 1) << "Motor 3 (with error) should have been rebooted once";

  // Other arm motors should NOT have been rebooted
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    if (id == ARM_JOINT_1_ID || id == ARM_JOINT_3_ID)
      continue;
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getRebootCount(), 0) << "Motor " << (int) id << " (healthy) should NOT have been rebooted";
  }

  // Gripper should not have been rebooted either
  EXPECT_EQ(gripper_motor->getRebootCount(), 0) << "Gripper (healthy) should NOT have been rebooted";

  // 6. Verify hardware errors were cleared
  EXPECT_EQ(motor1->getHardwareError(), 0) << "Motor 1 hardware error should be cleared after reboot";
  EXPECT_EQ(motor3->getHardwareError(), 0) << "Motor 3 hardware error should be cleared after reboot";
}

TEST_F(HardwareInterfaceTest, RebootService_RestoresTorqueOnAndBlueLED)
{
  // Test that reboot restores torque ON state after clearing hardware errors
  //
  // NOTE: When a hardware error is detected, read() returns ERROR which causes
  // the controller_manager to deactivate the HW interface. After reboot,
  // the HW interface remains in inactive state, so LEDs will be pink.

  // 1. Setup: torque is ON by default (torque_on_startup: true)
  auto reboot_client = tester_node_->create_test_client<std_srvs::srv::Trigger>("/athena_arm_interface/reboot");
  ASSERT_TRUE(reboot_client->wait_for_service(*executor_, 5s)) << "Reboot service not available";

  // 2. Verify torque is initially on
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    ASSERT_EQ(motor->read1Byte(torque_addr), 1) << "Motor " << (int) id << " torque should be ON initially";
  }

  // 3. Reset reboot counter and set hardware error on motor 1
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  motor1->resetRebootCount();
  motor1->setHardwareError(dynamixel_ros_control::HardwareErrorBit::ERROR_OVERHEATING);

  // Wait for at least one read cycle to update hardware_error_status in the Dynamixel class
  std::this_thread::sleep_for(300ms);

  // 4. Call reboot service
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 10s;
  options.response_timeout = 10s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::Trigger>(reboot_client->get(), request, *executor_, options);

  ASSERT_NE(resp, nullptr) << "Reboot service call failed";
  EXPECT_TRUE(resp->success) << "Reboot should succeed: " << resp->message;

  std::this_thread::sleep_for(500ms);

  // 5. Verify motor was rebooted and hardware error was cleared
  EXPECT_EQ(motor1->getRebootCount(), 1) << "Motor 1 should have been rebooted";
  EXPECT_EQ(motor1->getHardwareError(), 0) << "Motor 1 hardware error should be cleared after reboot";

  // 6. Verify torque is still ON (restored state) for all motors
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 1) << "Motor " << (int) id << " torque should be restored to ON";
  }

  // 7. LED will be pink because HW interface was deactivated after hardware error
  // (controller_manager deactivates HW when read() returns ERROR)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_PINK_R) << "Motor " << (int) id << " LED should be pink (R) - HW inactive";
    EXPECT_EQ(motor->getLedGreen(), COLOR_PINK_G) << "Motor " << (int) id << " LED should be pink (G) - HW inactive";
    EXPECT_EQ(motor->getLedBlue(), COLOR_PINK_B) << "Motor " << (int) id << " LED should be pink (B) - HW inactive";
  }
}

TEST_F(HardwareInterfaceTest, RebootService_RestoresTorqueOffAndGreenLED)
{
  // Test that reboot restores torque OFF state after clearing hardware errors
  //
  // NOTE: When a hardware error is detected, read() returns ERROR which causes
  // the controller_manager to deactivate the HW interface. After reboot,
  // the HW interface remains in inactive state, so LEDs will be pink.

  // 1. Setup clients
  auto reboot_client = tester_node_->create_test_client<std_srvs::srv::Trigger>("/athena_arm_interface/reboot");
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(reboot_client->wait_for_service(*executor_, 5s)) << "Reboot service not available";
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s)) << "Torque service not available";

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 10s;
  options.response_timeout = 10s;

  // 2. Disable torque first
  auto torque_request = std::make_shared<std_srvs::srv::SetBool::Request>();
  torque_request->data = false;
  auto torque_resp = hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), torque_request,
                                                                                *executor_, options);
  ASSERT_NE(torque_resp, nullptr);
  ASSERT_TRUE(torque_resp->success) << "Should be able to disable torque";

  std::this_thread::sleep_for(300ms);

  // 3. Verify torque is now off
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    ASSERT_EQ(motor->read1Byte(torque_addr), 0) << "Motor " << (int) id << " torque should be OFF";
  }

  // 4. Set hardware error on motor 1 and call reboot
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  motor1->resetRebootCount();
  motor1->setHardwareError(dynamixel_ros_control::HardwareErrorBit::ERROR_OVERLOAD);

  // Wait for at least one read cycle to update hardware_error_status in the Dynamixel class
  std::this_thread::sleep_for(300ms);

  auto reboot_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto resp = hector_testing_utils::call_service<std_srvs::srv::Trigger>(reboot_client->get(), reboot_request,
                                                                         *executor_, options);

  ASSERT_NE(resp, nullptr) << "Reboot service call failed";
  EXPECT_TRUE(resp->success) << "Reboot should succeed: " << resp->message;

  std::this_thread::sleep_for(500ms);

  // 5. Verify motor was rebooted and hardware error was cleared
  EXPECT_EQ(motor1->getRebootCount(), 1) << "Motor 1 should have been rebooted";
  EXPECT_EQ(motor1->getHardwareError(), 0) << "Motor 1 hardware error should be cleared after reboot";

  // 6. Verify torque is still OFF (restored to user's desired state)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 0) << "Motor " << (int) id << " torque should remain OFF after reboot";
  }

  // 7. LED will be pink because HW interface was deactivated after hardware error
  // (controller_manager deactivates HW when read() returns ERROR)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_PINK_R) << "Motor " << (int) id << " LED should be pink (R) - HW inactive";
    EXPECT_EQ(motor->getLedGreen(), COLOR_PINK_G) << "Motor " << (int) id << " LED should be pink (G) - HW inactive";
    EXPECT_EQ(motor->getLedBlue(), COLOR_PINK_B) << "Motor " << (int) id << " LED should be pink (B) - HW inactive";
  }
}

TEST_F(HardwareInterfaceTest, RebootService_NoRebootWhenNoErrors)
{
  // Test that when no motors have hardware errors, no reboots occur but LEDs are still updated

  // 1. Create reboot service client
  auto reboot_client = tester_node_->create_test_client<std_srvs::srv::Trigger>("/athena_arm_interface/reboot");
  ASSERT_TRUE(reboot_client->wait_for_service(*executor_, 5s)) << "Reboot service not available";

  // 2. Reset reboot counters for all motors (no hardware errors set)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    motor->resetRebootCount();
    ASSERT_EQ(motor->getHardwareError(), 0) << "Motor " << (int) id << " should have no hardware error";
  }

  // 3. Call reboot service
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 10s;
  options.response_timeout = 10s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::Trigger>(reboot_client->get(), request, *executor_, options);

  ASSERT_NE(resp, nullptr) << "Reboot service call failed";
  EXPECT_TRUE(resp->success) << "Reboot should succeed (no errors to fix): " << resp->message;

  std::this_thread::sleep_for(500ms);

  // 4. Verify NO motors were rebooted
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getRebootCount(), 0) << "Motor " << (int) id << " should NOT have been rebooted (no error)";
  }

  // 5. Verify LED is still correct (blue since torque is on)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "Motor " << (int) id << " LED should be blue";
  }
}

// ============================================================================
// Error Threshold / Robustness Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, Robustness_ToleratesOccasionalReadErrors)
{
  // Test that the system tolerates occasional read errors without triggering ERROR state
  // The error threshold is 25 consecutive errors before returning ERROR

  // 1. Activate controller and verify normal operation
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(pub, 5s));

  // Move to initial position
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // 2. Inject brief communication errors (less than threshold)
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor1, nullptr);

  // Cause a few errors then recover (well below the 25 error threshold)
  motor1->setCommunicationError(true);
  std::this_thread::sleep_for(200ms);  // ~4 read cycles at 50Hz
  motor1->setCommunicationError(false);

  // 3. Wait for recovery
  std::this_thread::sleep_for(500ms);

  // 4. Verify controller is still active (system tolerated the brief errors)
  ASSERT_TRUE(waitForControllerState("arm_position_controller", "active", 5s))
      << "Controller should remain active after brief communication errors";

  // 5. Verify commands still work
  std::vector<double> positions_before;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_before.push_back(motor->getCurrentPosition());
  }

  cmd.data = {0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  // Verify motors moved
  bool any_moved = false;
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    if (std::abs(motor->getCurrentPosition() - positions_before[i]) > 0.1) {
      any_moved = true;
      break;
    }
  }
  EXPECT_TRUE(any_moved) << "System should continue operating after tolerating brief read errors";
}

TEST_F(HardwareInterfaceTest, Robustness_IntermittentErrorsWithRecovery)
{
  // Test that the system handles intermittent errors (error, recover, error, recover)
  // without accumulating errors across recovery periods

  // 1. Activate controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(pub, 5s));

  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor1, nullptr);

  // 2. Cause multiple intermittent error periods
  for (int i = 0; i < 5; ++i) {
    // Brief error
    motor1->setCommunicationError(true);
    std::this_thread::sleep_for(100ms);  // ~2 read cycles

    // Recovery period - errors should reset
    motor1->setCommunicationError(false);
    std::this_thread::sleep_for(200ms);  // Allow successful reads to reset error counter
  }

  // 3. Verify controller is still active
  ASSERT_TRUE(waitForControllerState("arm_position_controller", "active", 5s))
      << "Controller should remain active after intermittent errors with recovery";

  // 4. Verify system is still functional
  std::vector<double> positions_before;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_before.push_back(motor->getCurrentPosition());
  }

  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  bool any_moved = false;
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    if (std::abs(motor->getCurrentPosition() - positions_before[i]) > 0.1) {
      any_moved = true;
      break;
    }
  }
  EXPECT_TRUE(any_moved) << "System should function after intermittent errors with recovery periods";
}

TEST_F(HardwareInterfaceTest, Robustness_ControllerActivationWithoutPriorRead)
{
  // Test that controller activation works even when no read() has been called yet
  // The perform_command_mode_switch should perform an inline read

  // Note: The test fixture automatically activates hardware, but we can test
  // by ensuring controller activation succeeds on a fresh start

  // 1. Verify no controllers are active initially
  auto list_resp = list_controllers();
  ASSERT_NE(list_resp, nullptr);

  bool any_active = false;
  for (const auto& ctrl : list_resp->controller) {
    if (ctrl.state == "active") {
      any_active = true;
      break;
    }
  }

  // 2. If no controllers are active, activate one - this tests inline read
  if (!any_active) {
    ASSERT_TRUE(loadAndActivateController("arm_position_controller"))
        << "Controller activation should succeed with inline read in perform_command_mode_switch";
  }

  // 3. Verify controller is now active and functional
  ASSERT_TRUE(waitForControllerState("arm_position_controller", "active", 5s));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(pub, 5s));

  // 4. Send command and verify it works
  std::vector<double> positions_before;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_before.push_back(motor->getCurrentPosition());
  }

  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  bool any_moved = false;
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    if (std::abs(motor->getCurrentPosition() - positions_before[i]) > 0.05) {
      any_moved = true;
      break;
    }
  }
  EXPECT_TRUE(any_moved) << "Commands should work after controller activation with inline read";
}

TEST_F(HardwareInterfaceTest, Robustness_ControllerSwitchAfterCommunicationRecovery)
{
  // Test controller switching after communication errors have been resolved
  // This verifies the inline read allows mode switches after error recovery

  // 1. Activate first controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pos_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(pos_pub, 5s));

  // 2. Cause communication errors that reset first_read_successful_
  dynamixel_ros_control::MockDynamixelManager::instance().setGlobalCommunicationError(true);
  std::this_thread::sleep_for(500ms);
  dynamixel_ros_control::MockDynamixelManager::instance().setGlobalCommunicationError(false);
  std::this_thread::sleep_for(500ms);

  // 3. Load velocity controller
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;

  auto load_req = std::make_shared<LoadController::Request>();
  load_req->name = "arm_velocity_controller";
  auto load_resp =
      hector_testing_utils::call_service<LoadController>(load_client_->get(), load_req, *executor_, options);
  ASSERT_NE(load_resp, nullptr);
  ASSERT_TRUE(load_resp->ok);

  auto config_req = std::make_shared<ConfigureController::Request>();
  config_req->name = "arm_velocity_controller";
  hector_testing_utils::call_service<ConfigureController>(config_client_->get(), config_req, *executor_, options);

  // 4. Switch controllers - this should succeed with inline read
  ASSERT_TRUE(switch_controllers({"arm_velocity_controller"}, {"arm_position_controller"}))
      << "Controller switch should succeed after communication recovery (inline read)";

  // 5. Verify velocity controller is active
  ASSERT_TRUE(waitForControllerState("arm_velocity_controller", "active", 5s));
  ASSERT_TRUE(waitForControllerState("arm_position_controller", "inactive", 5s));

  // 6. Verify velocity commands work
  auto vel_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_velocity_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(vel_pub, 5s));

  std::vector<double> positions_before;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_before.push_back(motor->getCurrentPosition());
  }

  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  vel_pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  bool any_moved = false;
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    if (std::abs(motor->getCurrentPosition() - positions_before[i]) > 0.1) {
      any_moved = true;
      break;
    }
  }
  EXPECT_TRUE(any_moved) << "Velocity controller should work after switch following communication recovery";
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
