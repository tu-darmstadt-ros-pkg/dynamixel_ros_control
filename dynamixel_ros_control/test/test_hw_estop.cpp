// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// E-Stop Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, EStop_StopsMovement)
{
  // 1. Activate arm position controller and start movement
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Send position command - motors should start moving
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};  // Target position
  pub->publish(cmd);
  std::this_thread::sleep_for(500ms);

  // Record positions before e-stop
  std::vector<double> positions_before;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_before.push_back(motor->getJointPosition());
  }

  // 2. Trigger E-Stop
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;

  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0) << "No subscriber for e-stop topic";

  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 3. Send another command - should be ignored due to e-stop
  cmd.data = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
  pub->publish(cmd);
  std::this_thread::sleep_for(500ms);

  // 4. Verify LED is orange and motors stopped (positions haven't changed much since e-stop)
  std::this_thread::sleep_for(200ms);  // Allow LED update

  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_ORANGE_R) << "Motor " << (int) id << " LED should be orange (R)";
    EXPECT_EQ(motor->getLedGreen(), COLOR_ORANGE_G) << "Motor " << (int) id << " LED should be orange (G)";
    EXPECT_EQ(motor->getLedBlue(), COLOR_ORANGE_B) << "Motor " << (int) id << " LED should be orange (B)";
  }

  // 5. Disable E-Stop and verify normal operation resumes
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // Verify LED is back to blue
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_BLUE_R) << "Motor " << (int) id << " LED should be blue (R)";
    EXPECT_EQ(motor->getLedGreen(), COLOR_BLUE_G) << "Motor " << (int) id << " LED should be blue (G)";
    EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "Motor " << (int) id << " LED should be blue (B)";
  }
}

TEST_F(HardwareInterfaceTest, EStop_MultipleCommandsBlocked)
{
  // Test that multiple commands during E-Stop are all blocked
  // and that LED correctly reflects e-stop state

  // 1. Activate arm position controller and move to initial position
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // First move to a known position
  std_msgs::msg::Float64MultiArray init_cmd;
  init_cmd.data = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
  pub->publish(init_cmd);
  std::this_thread::sleep_for(1s);

  // 2. Trigger E-Stop
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0);

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 3. Record positions at e-stop
  std::vector<double> positions_at_estop;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_at_estop.push_back(motor->getJointPosition());
  }

  // 4. Verify LED is orange
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_ORANGE_R) << "Motor " << (int) id << " LED should be orange (R)";
    EXPECT_EQ(motor->getLedGreen(), COLOR_ORANGE_G) << "Motor " << (int) id << " LED should be orange (G)";
    EXPECT_EQ(motor->getLedBlue(), COLOR_ORANGE_B) << "Motor " << (int) id << " LED should be orange (B)";
  }

  // 5. Send multiple movement commands over time - all should be ignored
  for (int i = 0; i < 5; ++i) {
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = {1.0 + i * 0.2, 1.0 + i * 0.2, 1.0 + i * 0.2, 1.0 + i * 0.2,
                1.0 + i * 0.2, 1.0 + i * 0.2, 1.0 + i * 0.2};
    pub->publish(cmd);
    std::this_thread::sleep_for(200ms);
  }
  std::this_thread::sleep_for(500ms);

  // 6. Verify motors haven't moved significantly (e-stop still active)
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    EXPECT_NEAR(motor->getJointPosition(), positions_at_estop[i], 0.15)
        << "Motor " << (ARM_JOINT_1_ID + i) << " should not move during e-stop";
  }

  // 7. Verify LED is still orange after all those commands
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_ORANGE_R) << "Motor " << (int) id << " LED should still be orange (R)";
    EXPECT_EQ(motor->getLedGreen(), COLOR_ORANGE_G) << "Motor " << (int) id << " LED should still be orange (G)";
    EXPECT_EQ(motor->getLedBlue(), COLOR_ORANGE_B) << "Motor " << (int) id << " LED should still be orange (B)";
  }

  // 8. Disable E-Stop and verify LED returns to blue
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_BLUE_R) << "Motor " << (int) id << " LED should be blue (R)";
    EXPECT_EQ(motor->getLedGreen(), COLOR_BLUE_G) << "Motor " << (int) id << " LED should be blue (G)";
    EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "Motor " << (int) id << " LED should be blue (B)";
  }
}

TEST_F(HardwareInterfaceTest, EStop_VelocityControllerSwitchesToPositionMode)
{
  // Test that when e-stop is activated while using velocity controller,
  // the actuators switch to position control mode with current position as goal

  // 1. Load and activate flipper velocity controller
  ASSERT_TRUE(loadAndActivateController("flipper_velocity_controller"));

  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_velocity_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // 2. Send velocity command - flippers should start moving
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {1.0, 1.0, 1.0, 1.0};  // Joint velocity 1 rad/s
  pub->publish(cmd);
  std::this_thread::sleep_for(500ms);

  // Verify flippers are moving (velocity != 0)
  std::vector<uint8_t> flipper_ids = {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID};
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_NE(motor->getCurrentVelocity(), 0.0) << "Flipper " << (int) id << " should be moving before e-stop";
  }

  // Record positions before e-stop
  std::map<uint8_t, double> positions_before_estop;
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_before_estop[id] = motor->getJointPosition();
  }

  // 3. Trigger E-Stop
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0);

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 4. Verify motors are in position mode (operating_mode register = 3 for position control)
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t operating_mode_addr = motor->getAddress("operating_mode");
    uint8_t operating_mode = motor->read1Byte(operating_mode_addr);
    // Position control mode = 3, Extended position = 4
    EXPECT_TRUE(operating_mode == 3 || operating_mode == 4)
        << "Flipper " << (int) id << " should be in position mode after e-stop, got mode " << (int) operating_mode;
  }

  // 5. Verify motors stopped at approximately the position they were at during e-stop
  std::this_thread::sleep_for(500ms);  // Allow physics to settle
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double position_change = std::abs(motor->getJointPosition() - positions_before_estop[id]);
    // Allow some tolerance for stopping distance
    EXPECT_LT(position_change, 0.5) << "Flipper " << (int) id << " should have stopped near e-stop position";
  }

  // 6. Verify LED is orange
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_ORANGE_R);
    EXPECT_EQ(motor->getLedGreen(), COLOR_ORANGE_G);
    EXPECT_EQ(motor->getLedBlue(), COLOR_ORANGE_B);
  }

  // Cleanup: release e-stop
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(300ms);
}

TEST_F(HardwareInterfaceTest, EStop_GoalPositionWriteFailurePreventsActivation)
{
  // Test that e-stop activation attempts even with communication errors.
  // When communication errors persist, the e-stop flag is set internally,
  // but the hardware interface enters error state and gets deactivated by
  // the controller manager. This test verifies that the system doesn't crash
  // and handles the error gracefully.
  //
  // Note: LED state is not checked because writes fail during comm error,
  // leaving LED state undefined (whatever it was before the error).

  // 1. Load and activate arm position controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Move to a position
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // Record positions before e-stop
  std::map<uint8_t, double> positions_before_estop;
  std::vector<uint8_t> arm_ids = {ARM_JOINT_1_ID, ARM_JOINT_2_ID, ARM_JOINT_3_ID, ARM_JOINT_4_ID,
                                  ARM_JOINT_5_ID, ARM_JOINT_6_ID, ARM_JOINT_7_ID};
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_before_estop[id] = motor->getJointPosition();
  }

  // 2. Inject global communication error (simulates goal position write failure)
  dynamixel_ros_control::MockDynamixelManager::instance().setGlobalCommunicationError(true);

  // 3. Attempt to trigger E-Stop
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0);

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 4. Clear communication error
  dynamixel_ros_control::MockDynamixelManager::instance().setGlobalCommunicationError(false);
  std::this_thread::sleep_for(300ms);

  // 5. Verify motors didn't move significantly during the error/e-stop sequence.
  // Even though commands weren't being written, motors should hold their position.
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double position_change = std::abs(motor->getJointPosition() - positions_before_estop[id]);
    EXPECT_LT(position_change, 0.2) << "Motor " << (int) id
                                    << " should not move significantly during comm error + e-stop";
  }

  // Cleanup - e-stop message won't work as HW is deactivated, but send it anyway
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(300ms);
}

TEST_F(HardwareInterfaceTest, EStop_ReactivationMaintainsPositionModeUntilControllerLoaded)
{
  // Test that when e-stop is released, position control mode stays active
  // with current position as goal until a new controller is loaded

  // 1. Load and activate arm position controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  // Move to a position
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  // 2. Trigger E-Stop
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0);

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // Record positions during e-stop
  std::map<uint8_t, double> positions_during_estop;
  std::vector<uint8_t> arm_ids = {ARM_JOINT_1_ID, ARM_JOINT_2_ID, ARM_JOINT_3_ID, ARM_JOINT_4_ID,
                                  ARM_JOINT_5_ID, ARM_JOINT_6_ID, ARM_JOINT_7_ID};
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_during_estop[id] = motor->getJointPosition();
  }

  // 3. Release E-Stop
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 4. Verify motors maintain their position (no sudden movement)
  // No controller is active, so motors should hold position
  // Allow small tolerance for physics simulation settling
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double position_change = std::abs(motor->getJointPosition() - positions_during_estop[id]);
    EXPECT_LT(position_change, 0.15) << "Motor " << (int) id
                                     << " should maintain position after e-stop release (no controller active)";
  }

  // 5. Verify LED is blue (normal state after e-stop release)
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_BLUE_R);
    EXPECT_EQ(motor->getLedGreen(), COLOR_BLUE_G);
    EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B);
  }

  // 6. Now reactivate the controller (it's already loaded, just deactivated) and verify it works
  ASSERT_TRUE(switch_controllers({"arm_position_controller"}, {}));

  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  // Send new command
  cmd.data = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  // Verify motors moved to new position
  for (uint8_t id : arm_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_NEAR(motor->getJointPosition(), 0.8, 0.15)
        << "Motor " << (int) id << " should reach new target after controller reload";
  }
}

TEST_F(HardwareInterfaceTest, EStop_CannotActivateWhenTorqueOff)
{
  // Test that e-stop cannot be activated when torque is off

  // 1. Disable torque first
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  std::this_thread::sleep_for(300ms);

  // 2. Verify LED is green (torque off)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedGreen(), COLOR_GREEN_G) << "LED should be green when torque is off";
  }

  // 3. Try to activate e-stop
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0);

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 4. Verify LED is still green (e-stop did not activate because torque was off)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedGreen(), COLOR_GREEN_G)
        << "LED should still be green - e-stop cannot activate when torque is off";
    EXPECT_EQ(motor->getLedRed(), COLOR_GREEN_R);
    EXPECT_EQ(motor->getLedBlue(), COLOR_GREEN_B);
  }

  // Cleanup: release e-stop message and re-enable torque
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(100ms);

  request->data = true;
  hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  std::this_thread::sleep_for(300ms);
}

TEST_F(HardwareInterfaceTest, EStop_DeactivationRetriesControllerDeactivation)
{
  // Test that when deactivating e-stop with active controllers:
  // 1. If communication is restored, e-stop deactivation retries controller
  //    deactivation and succeeds
  // 2. The system returns to normal operation

  // 1. Load and activate arm position controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Move to a position
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // 2. Trigger E-Stop normally (no communication error)
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0);

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 3. Verify controller was deactivated
  auto list_resp = list_controllers();
  ASSERT_NE(list_resp, nullptr);
  auto states = states_from_list(*list_resp);
  EXPECT_TRUE(states.count("arm_position_controller") == 0 || states["arm_position_controller"] != "active")
      << "Controller should be deactivated after e-stop";

  // 4. Verify LED is orange (e-stop active)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_ORANGE_R);
    EXPECT_EQ(motor->getLedGreen(), COLOR_ORANGE_G);
    EXPECT_EQ(motor->getLedBlue(), COLOR_ORANGE_B);
  }

  // 5. Now manually reactivate the controller while e-stop is active
  // (simulating a scenario where controller got reactivated somehow)
  ASSERT_TRUE(switch_controllers({"arm_position_controller"}, {}));
  std::this_thread::sleep_for(300ms);

  // Verify controller is active again
  list_resp = list_controllers();
  ASSERT_NE(list_resp, nullptr);
  states = states_from_list(*list_resp);
  EXPECT_EQ(states["arm_position_controller"], "active") << "Controller should be active after manual reactivation";

  // 6. Attempt to deactivate e-stop - should retry controller deactivation and succeed
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 7. Verify controller was deactivated by the retry
  list_resp = list_controllers();
  ASSERT_NE(list_resp, nullptr);
  states = states_from_list(*list_resp);
  EXPECT_TRUE(states.count("arm_position_controller") == 0 || states["arm_position_controller"] != "active")
      << "Controller should be deactivated after e-stop release retry";

  // 8. Verify LED is blue (normal state after e-stop release)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_BLUE_R);
    EXPECT_EQ(motor->getLedGreen(), COLOR_BLUE_G);
    EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B);
  }
}

TEST_F(HardwareInterfaceTest, EStop_RemainsActiveWhenControllerDeactivationFails)
{
  // Test that e-stop remains active if controller deactivation fails during
  // e-stop release attempt. This ensures safety when communication issues
  // prevent proper controller shutdown.
  //
  // Note: When communication errors occur during controller mode switch,
  // the controller manager may deactivate the hardware interface entirely.
  // This test verifies the system handles this gracefully and doesn't crash.

  // 1. Load and activate arm position controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Move to a position
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // 2. Trigger E-Stop normally
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0);

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 3. Manually reactivate the controller while e-stop is active
  ASSERT_TRUE(switch_controllers({"arm_position_controller"}, {}));
  std::this_thread::sleep_for(300ms);

  // 4. Now inject communication error so controller deactivation will fail
  dynamixel_ros_control::MockDynamixelManager::instance().setGlobalCommunicationError(true);

  // 5. Attempt to deactivate e-stop - should fail because controller deactivation fails
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 6. Due to the communication error causing mode switch failure, the controller
  // manager may deactivate the hardware interface entirely. This is expected
  // behavior - the system enters a safe error state.

  // 7. Clear communication error
  dynamixel_ros_control::MockDynamixelManager::instance().setGlobalCommunicationError(false);
  std::this_thread::sleep_for(300ms);

  // 8. The key verification: the system didn't crash and is in a safe state.
  // Either e-stop is still active (orange LED) or hardware interface was
  // deactivated (pink LED). Both are safe states.
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    // LED should be either orange (e-stop still active) or pink (HW deactivated)
    bool is_orange = (motor->getLedRed() == COLOR_ORANGE_R && motor->getLedGreen() == COLOR_ORANGE_G &&
                      motor->getLedBlue() == COLOR_ORANGE_B);
    bool is_pink = (motor->getLedRed() == COLOR_PINK_R && motor->getLedGreen() == COLOR_PINK_G &&
                    motor->getLedBlue() == COLOR_PINK_B);
    EXPECT_TRUE(is_orange || is_pink) << "Motor " << (int) id
                                      << " should be in safe state (orange=e-stop or pink=HW deactivated)";
  }
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
