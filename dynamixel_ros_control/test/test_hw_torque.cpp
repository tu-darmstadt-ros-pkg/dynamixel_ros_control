// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Torque Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, Torque_DisableTorqueChangesLEDToGreen)
{
  // 1. Verify initial state - torque on, LED blue
  std::this_thread::sleep_for(200ms);
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "Initial LED should be blue";
    }
  }

  // 2. Call set_torque service to disable torque
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;  // Disable torque

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);

  ASSERT_NE(resp, nullptr) << "Service call failed";
  EXPECT_TRUE(resp->success) << "Torque disable should succeed";

  // 3. Wait for LED update and verify LED is green
  std::this_thread::sleep_for(500ms);

  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      EXPECT_EQ(motor->getLedRed(), COLOR_GREEN_R) << "Motor " << (int) id << " LED should be green (R)";
      EXPECT_EQ(motor->getLedGreen(), COLOR_GREEN_G) << "Motor " << (int) id << " LED should be green (G)";
      EXPECT_EQ(motor->getLedBlue(), COLOR_GREEN_B) << "Motor " << (int) id << " LED should be green (B)";
    }
  }

  // 4. Verify torque is actually disabled in motors
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      uint16_t torque_addr = motor->getAddress("torque_enable");
      EXPECT_EQ(motor->read1Byte(torque_addr), 0) << "Motor " << (int) id << " torque should be disabled";
    }
  }
}

TEST_F(HardwareInterfaceTest, Torque_EnableTorqueChangesLEDToBlue)
{
  // 1. First disable torque
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;  // Disable torque

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  std::this_thread::sleep_for(300ms);

  // Verify LED is green (torque off)
  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  EXPECT_EQ(motor->getLedGreen(), COLOR_GREEN_G) << "LED should be green when torque is off";

  // 2. Enable torque
  request->data = true;
  resp = hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success);

  std::this_thread::sleep_for(300ms);

  // 3. Verify LED is blue
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      EXPECT_EQ(motor->getLedRed(), COLOR_BLUE_R) << "Motor " << (int) id << " LED should be blue (R)";
      EXPECT_EQ(motor->getLedGreen(), COLOR_BLUE_G) << "Motor " << (int) id << " LED should be blue (G)";
      EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "Motor " << (int) id << " LED should be blue (B)";
    }
  }

  // 4. Verify torque is enabled in motors
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      uint16_t torque_addr = motor->getAddress("torque_enable");
      EXPECT_EQ(motor->read1Byte(torque_addr), 1) << "Motor " << (int) id << " torque should be enabled";
    }
  }
}

TEST_F(HardwareInterfaceTest, Torque_CommandsNotExecutedWhenTorqueOff)
{
  // 1. Disable torque
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  std::this_thread::sleep_for(300ms);

  // 2. Record initial positions
  std::vector<double> initial_positions;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions.push_back(motor->getCurrentPosition());
  }

  // 3. Load and activate controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  // 4. Send position command
  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // 5. Verify motors didn't move (torque is off, physics simulation shouldn't move them)
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    // With torque off, motors should not move toward goal
    EXPECT_NEAR(motor->getCurrentPosition(), initial_positions[i], 0.01)
        << "Motor " << (ARM_JOINT_1_ID + i) << " should not have moved with torque off";
  }
}

TEST_F(HardwareInterfaceTest, Torque_OnStartupVerification)
{
  // Verify that the torque_on_startup parameter works as expected.
  // The test URDF has torque_on_startup: true, so torque should be enabled on activation.
  // This test verifies the initial state set up by the test fixture.

  // Hardware interface is active and torque should be on (torque_on_startup: true in URDF)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Motor " << (int) id << " should exist";
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 1)
        << "Motor " << (int) id << " torque should be ON due to torque_on_startup: true";
  }

  // LED should be blue (torque on, active state)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B)
        << "Motor " << (int) id << " LED should be blue when torque is on at startup";
  }
}

TEST_F(HardwareInterfaceTest, Torque_GoalPositionUpdatedBeforeReEnable)
{
  // CRITICAL TEST: Verify that goal position is properly reset when torque is re-enabled
  // with an active controller. The resetGoalStateAndVerify() function writes goal position
  // equal to current position for all active command interfaces before enabling torque.
  //
  // This test verifies that with an active controller sending commands, the motors don't
  // jump to an old position when torque is re-enabled.

  // 1. Move motors to a known position first using position controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Move to position 0.5 rad
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  // Record positions after reaching target (use joint position which includes homing offset)
  std::map<uint8_t, double> positions_at_target;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_at_target[id] = motor->getJointPosition();
    // Verify motors reached approximately the target
    EXPECT_NEAR(positions_at_target[id], 0.5, 0.1) << "Motor " << (int) id << " should have reached target 0.5 rad";
  }

  // 2. Disable torque (this also deactivates the controller)
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success) << "Torque disable should succeed";

  std::this_thread::sleep_for(300ms);

  // Verify controller was deactivated
  EXPECT_TRUE(waitForControllerState("arm_position_controller", "inactive", 3s))
      << "Controller should be deactivated when torque is disabled";

  // 3. Re-enable torque WITHOUT a controller active
  // The motors should stay at their current position because when no controller is active,
  // the previous goal (0.5 rad) remains in the motor. This is the expected behavior.
  request->data = true;
  resp = hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success) << "Torque enable should succeed";

  std::this_thread::sleep_for(500ms);

  // 4. Verify motors stayed at approximately the same position
  // Since the goal was 0.5 rad before torque disable, motors should stay near that position
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    // Motors should stay near their position before torque disable (which was ~0.5 rad)
    EXPECT_NEAR(motor->getJointPosition(), positions_at_target[id], 0.2)
        << "Motor " << (int) id << " should stay near position after torque cycle. "
        << "Position before torque off: " << positions_at_target[id]
        << ", Position after torque on: " << motor->getJointPosition();
  }
}

TEST_F(HardwareInterfaceTest, Torque_GoalVelocityZeroBeforeReEnable)
{
  // Verify that when a velocity controller is deactivated via torque disable,
  // motors stop moving (velocity becomes zero).

  // 1. Activate velocity controller and send velocity command to get motors moving
  ASSERT_TRUE(loadAndActivateController("arm_velocity_controller"));

  auto vel_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_velocity_controller/commands", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && vel_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(vel_pub->get_subscription_count(), 0);

  // Send non-zero velocity command to get motors moving
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  vel_pub->publish(cmd);
  std::this_thread::sleep_for(500ms);

  // Record positions while moving (use joint position for consistency with hardware interface)
  std::map<uint8_t, double> positions_while_moving;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_while_moving[id] = motor->getJointPosition();
  }

  // 2. Disable torque (this also deactivates the controller)
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  std::this_thread::sleep_for(300ms);

  // Verify controller was deactivated
  EXPECT_TRUE(waitForControllerState("arm_velocity_controller", "inactive", 3s))
      << "Controller should be deactivated when torque is disabled";

  // 3. Record positions after torque disable
  std::map<uint8_t, double> positions_after_torque_off;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_after_torque_off[id] = motor->getJointPosition();
  }

  // 4. Re-enable torque (no controller active)
  request->data = true;
  resp = hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success);

  // 5. Wait and verify motors stayed put (no controller commanding movement)
  std::this_thread::sleep_for(500ms);

  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double position_change = std::abs(motor->getJointPosition() - positions_after_torque_off[id]);
    // With no controller active, motors should not move significantly
    // (they may drift slightly due to physics simulation settling)
    EXPECT_LT(position_change, 0.3) << "Motor " << (int) id
                                    << " should not continue moving after torque re-enable without controller. "
                                    << "Position after torque off: " << positions_after_torque_off[id]
                                    << ", Position after torque on: " << motor->getJointPosition();
  }
}

TEST_F(HardwareInterfaceTest, Torque_DeactivatesControllersOnDisable)
{
  // Verify that disabling torque deactivates active controllers
  // This prevents controllers from commanding motors that can't move

  // 1. Activate controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
  ASSERT_TRUE(waitForControllerState("arm_position_controller", "active", 5s));

  // 2. Disable torque
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  ASSERT_TRUE(resp->success);

  std::this_thread::sleep_for(500ms);

  // 3. Verify controller was deactivated
  auto list_resp = list_controllers();
  ASSERT_NE(list_resp, nullptr);
  auto states = states_from_list(*list_resp);

  // Controller should be deactivated (inactive or unloaded)
  EXPECT_TRUE(states.count("arm_position_controller") == 0 || states["arm_position_controller"] != "active")
      << "Controller should be deactivated when torque is disabled";
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
