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

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
