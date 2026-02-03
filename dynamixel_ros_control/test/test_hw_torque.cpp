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

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
