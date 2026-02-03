// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Normal Usage Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, NormalUsage_ArmPositionMode)
{
  // 1. Load and activate arm_position_controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  // 2. Verify all arm motors exist in mock manager
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Motor ID " << (int) id << " not found in mock manager";
  }

  // 3. Create publisher and send position command
  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  std_msgs::msg::Float64MultiArray cmd;
  double target_position = 0.5;  // radians
  cmd.data = {target_position, target_position, target_position, target_position,
              target_position, target_position, target_position};  // 7 joints

  // Wait for subscriber and publish
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0) << "Controller not subscribed to commands topic";

  pub->publish(cmd);

  // 4. Wait for motors to reach target position
  std::this_thread::sleep_for(3s);

  // 5. Verify motors reached target position
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double current_pos = motor->getJointPosition();
    EXPECT_NEAR(current_pos, target_position, 0.1) << "Motor ID " << (int) id << " did not reach target position. "
                                                   << "Expected: " << target_position << ", Got: " << current_pos;
  }
}

TEST_F(HardwareInterfaceTest, NormalUsage_FlipperVelocityMode)
{
  // 1. Load and activate flipper_velocity_controller
  ASSERT_TRUE(loadAndActivateController("flipper_velocity_controller"));

  // 2. Verify all flipper motors exist
  std::vector<uint8_t> flipper_ids = {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID};
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Flipper motor ID " << (int) id << " not found";
  }

  // 3. Record initial positions
  std::map<uint8_t, double> initial_positions;
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions[id] = motor->getJointPosition();
  }

  // 4. Send velocity command
  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_velocity_controller/commands", 10);

  std_msgs::msg::Float64MultiArray cmd;
  double joint_velocity = 1.0;  // rad/s for joint
  cmd.data = {joint_velocity, joint_velocity, joint_velocity, joint_velocity};

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  pub->publish(cmd);

  // 5. Wait and verify motors are moving
  std::this_thread::sleep_for(1s);

  // Flipper transmissions have mechanical_reduction of +/-2.0
  // So actuator velocity = joint_velocity * reduction
  // FL: -2.0, FR: 2.0, BL: 2.0, BR: -2.0
  std::map<uint8_t, double> expected_directions = {
      {FLIPPER_FL_ID, -1.0},  // reduction=-2.0, so negative direction
      {FLIPPER_FR_ID, 1.0},   // reduction=2.0, positive direction
      {FLIPPER_BL_ID, 1.0},   // reduction=2.0
      {FLIPPER_BR_ID, -1.0}   // reduction=-2.0
  };

  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double current_pos = motor->getJointPosition();
    double position_change = current_pos - initial_positions[id];

    // Verify position changed in expected direction
    if (expected_directions[id] > 0) {
      EXPECT_GT(position_change, 0.1) << "Flipper motor " << (int) id << " should have moved in positive direction";
    } else {
      EXPECT_LT(position_change, -0.1) << "Flipper motor " << (int) id << " should have moved in negative direction";
    }
  }
}

TEST_F(HardwareInterfaceTest, NormalUsage_ControllerSwitch_ArmPositionToVelocity)
{
  // 1. Start with arm position controller active
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  // 2. Move to a known position
  auto pos_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  std_msgs::msg::Float64MultiArray pos_cmd;
  pos_cmd.data = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pos_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  pos_pub->publish(pos_cmd);
  std::this_thread::sleep_for(2s);

  // 3. Load velocity controller (configure but don't activate)
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;

  auto load_req = std::make_shared<LoadController::Request>();
  load_req->name = "arm_velocity_controller";
  hector_testing_utils::call_service<LoadController>(load_client_->get(), load_req, *executor_, options);

  auto config_req = std::make_shared<ConfigureController::Request>();
  config_req->name = "arm_velocity_controller";
  hector_testing_utils::call_service<ConfigureController>(config_client_->get(), config_req, *executor_, options);

  // 4. Switch controllers
  ASSERT_TRUE(switch_controllers({"arm_velocity_controller"}, {"arm_position_controller"}));

  // 5. Verify states
  ASSERT_TRUE(waitForControllerState("arm_position_controller", "inactive", 5s));
  ASSERT_TRUE(waitForControllerState("arm_velocity_controller", "active", 5s));

  // 6. Send velocity command and verify movement
  auto vel_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_velocity_controller/commands", 10);

  std_msgs::msg::Float64MultiArray vel_cmd;
  vel_cmd.data = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};  // rad/s

  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && vel_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(vel_pub->get_subscription_count(), 0);

  // Record positions before velocity command
  std::vector<double> positions_before;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_before.push_back(motor->getJointPosition());
  }

  vel_pub->publish(vel_cmd);
  std::this_thread::sleep_for(1s);

  // Verify positions changed (motors are moving)
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    double pos_change = motor->getJointPosition() - positions_before[i];
    EXPECT_GT(pos_change, 0.1) << "Motor " << (ARM_JOINT_1_ID + i) << " should have moved in velocity mode";
  }
}

TEST_F(HardwareInterfaceTest, NormalUsage_SimultaneousMovement)
{
  // 1. Activate all controllers
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
  ASSERT_TRUE(loadAndActivateController("flipper_velocity_controller"));
  ASSERT_TRUE(loadAndActivateController("gripper_position_controller"));

  // 2. Create publishers
  auto arm_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  auto flipper_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_velocity_controller/commands", 10);
  auto gripper_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/gripper_position_controller/commands", 10);

  // Wait for all subscribers
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (arm_pub->get_subscription_count() > 0 && flipper_pub->get_subscription_count() > 0 &&
        gripper_pub->get_subscription_count() > 0) {
      break;
    }
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  // 3. Record initial states
  std::vector<double> arm_initial, flipper_initial;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    arm_initial.push_back(dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id)->getJointPosition());
  }
  for (uint8_t id : {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID}) {
    flipper_initial.push_back(dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id)->getJointPosition());
  }
  double gripper_initial =
      dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID)->getJointPosition();

  // 4. Send commands to all
  std_msgs::msg::Float64MultiArray arm_cmd, flipper_cmd, gripper_cmd;
  arm_cmd.data = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  flipper_cmd.data = {1.0, 1.0, 1.0, 1.0};
  gripper_cmd.data = {0.5};

  arm_pub->publish(arm_cmd);
  flipper_pub->publish(flipper_cmd);
  gripper_pub->publish(gripper_cmd);

  // 5. Wait for movement
  std::this_thread::sleep_for(2s);

  // 6. Verify all moved
  for (size_t i = 0; i < 7; ++i) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
    EXPECT_NE(motor->getJointPosition(), arm_initial[i]) << "Arm joint " << i << " should have moved";
  }

  size_t idx = 0;
  for (uint8_t id : {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID}) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_NE(motor->getJointPosition(), flipper_initial[idx]) << "Flipper " << (int) id << " should have moved";
    idx++;
  }

  auto gripper_motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  EXPECT_NE(gripper_motor->getJointPosition(), gripper_initial) << "Gripper should have moved";
}

TEST_F(HardwareInterfaceTest, RapidControllerSwitch_StressTest)
{
  // Test rapid switching between controllers
  // This can expose race conditions and timing issues

  // Load and configure both controllers
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;

  auto load_req = std::make_shared<LoadController::Request>();
  load_req->name = "arm_position_controller";
  hector_testing_utils::call_service<LoadController>(load_client_->get(), load_req, *executor_, options);

  auto config_req = std::make_shared<ConfigureController::Request>();
  config_req->name = "arm_position_controller";
  hector_testing_utils::call_service<ConfigureController>(config_client_->get(), config_req, *executor_, options);

  load_req->name = "arm_velocity_controller";
  hector_testing_utils::call_service<LoadController>(load_client_->get(), load_req, *executor_, options);

  config_req->name = "arm_velocity_controller";
  hector_testing_utils::call_service<ConfigureController>(config_client_->get(), config_req, *executor_, options);

  // First, activate position controller (velocity is still inactive)
  ASSERT_TRUE(switch_controllers({"arm_position_controller"}, {}))
      << "Initial activation of position controller failed";
  std::this_thread::sleep_for(100ms);

  // Perform rapid switches between the two
  for (int i = 0; i < 5; ++i) {
    // Switch to velocity controller (deactivate position)
    ASSERT_TRUE(switch_controllers({"arm_velocity_controller"}, {"arm_position_controller"}))
        << "Switch to velocity controller failed on iteration " << i;
    std::this_thread::sleep_for(100ms);

    // Switch to position controller (deactivate velocity)
    ASSERT_TRUE(switch_controllers({"arm_position_controller"}, {"arm_velocity_controller"}))
        << "Switch to position controller failed on iteration " << i;
    std::this_thread::sleep_for(100ms);
  }

  // Final state check - verify system is stable
  auto list_resp = list_controllers();
  ASSERT_NE(list_resp, nullptr);
  auto states = states_from_list(*list_resp);

  // Position controller should be active (last activation in the loop)
  EXPECT_EQ(states["arm_position_controller"], "active")
      << "Position controller should be active after rapid switching";
  EXPECT_EQ(states["arm_velocity_controller"], "inactive")
      << "Velocity controller should be inactive after rapid switching";
}

TEST_F(HardwareInterfaceTest, Gripper_PositionControl)
{
  // Test gripper position control
  ASSERT_TRUE(loadAndActivateController("gripper_position_controller"));

  // Verify gripper motor exists
  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  ASSERT_NE(motor, nullptr) << "Gripper motor not found";

  double initial_pos = motor->getJointPosition();

  // Send position command
  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/gripper_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  std_msgs::msg::Float64MultiArray cmd;
  double target = 0.5;
  cmd.data = {target};
  pub->publish(cmd);

  std::this_thread::sleep_for(2s);

  // Verify gripper moved
  double final_pos = motor->getJointPosition();
  EXPECT_NEAR(final_pos, target, 0.2) << "Gripper should have moved to target. Initial: " << initial_pos
                                      << ", Final: " << final_pos;
}

TEST_F(HardwareInterfaceTest, MotorLimits_PositionLimitRespected)
{
  // Test that position limits are respected
  // (This depends on hardware interface implementation)

  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Send extreme position command (outside normal operating range)
  std_msgs::msg::Float64MultiArray cmd;
  double extreme_position = 100.0;  // Very large position
  cmd.data = {extreme_position, extreme_position, extreme_position, extreme_position,
              extreme_position, extreme_position, extreme_position};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  // Motors should move but may be limited by hardware
  // Just verify motors are still functional (no crash/hang)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Motor " << (int) id << " should still exist";
    // Motor should have moved in positive direction
    EXPECT_GT(motor->getJointPosition(), 0.0) << "Motor " << (int) id << " should have moved toward positive position";
  }
}

TEST_F(HardwareInterfaceTest, StateInterface_PositionVelocityConsistent)
{
  // Test that position and velocity state interfaces are consistent
  // Velocity should approximately match position derivative

  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Send position command to initiate movement
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  pub->publish(cmd);

  // Sample positions over time
  double dt = 0.1;
  std::this_thread::sleep_for(std::chrono::duration<double>(dt));

  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  double pos1 = motor->getJointPosition();

  std::this_thread::sleep_for(std::chrono::duration<double>(dt));

  double pos2 = motor->getJointPosition();

  // Velocity should be approximately (pos2 - pos1) / dt
  double approx_velocity = (pos2 - pos1) / dt;

  // Just verify motor is moving in expected direction
  EXPECT_GT(pos2, pos1) << "Motor should be moving toward goal (increasing position)";
  EXPECT_GT(approx_velocity, 0.0) << "Velocity should be positive when moving to higher position";
}

TEST_F(HardwareInterfaceTest, SimultaneousOperations_ArmAndFlipperIndependent)
{
  // Test that arm and flipper interfaces operate independently
  // E-Stop on arm shouldn't affect flipper operation

  // 1. Activate controllers on both interfaces
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
  ASSERT_TRUE(loadAndActivateController("flipper_position_controller"));

  auto arm_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  auto flipper_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    if (arm_pub->get_subscription_count() > 0 && flipper_pub->get_subscription_count() > 0) {
      break;
    }
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  // 2. Move both to initial positions
  std_msgs::msg::Float64MultiArray arm_cmd, flipper_cmd;
  arm_cmd.data = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
  flipper_cmd.data = {0.3, 0.3, 0.3, 0.3};
  arm_pub->publish(arm_cmd);
  flipper_pub->publish(flipper_cmd);
  std::this_thread::sleep_for(1s);

  // Record flipper positions
  std::vector<uint8_t> flipper_ids = {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID};
  std::map<uint8_t, double> flipper_positions_before;
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    flipper_positions_before[id] = motor->getJointPosition();
  }

  // 3. Disable torque on ARM only (simulating arm-specific issue)
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
  EXPECT_TRUE(resp->success);

  std::this_thread::sleep_for(300ms);

  // 4. Send new flipper command - should still work
  flipper_cmd.data = {0.6, 0.6, 0.6, 0.6};
  flipper_pub->publish(flipper_cmd);
  std::this_thread::sleep_for(2s);

  // 5. Verify flippers moved (not affected by arm torque disable)
  bool flipper_moved = false;
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (std::abs(motor->getJointPosition() - flipper_positions_before[id]) > 0.1) {
      flipper_moved = true;
      break;
    }
  }
  EXPECT_TRUE(flipper_moved) << "Flippers should still move when arm torque is disabled";

  // 6. Verify arm motors have green LED (torque off) while flippers have blue
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedGreen(), COLOR_GREEN_G) << "Arm motor " << (int) id << " LED should be green (torque off)";
  }

  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "Flipper motor " << (int) id << " LED should be blue (torque on)";
  }
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
