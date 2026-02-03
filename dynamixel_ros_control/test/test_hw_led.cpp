// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// LED Color Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, LED_BlueWhenActiveAndTorqueOn)
{
  // Hardware interface is active with torque on by default (torque_on_startup: true)
  // LED should be blue
  std::this_thread::sleep_for(500ms);  // Allow time for LED update

  // Check arm motors (hardware interface athena_arm_interface)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      EXPECT_EQ(motor->getLedRed(), COLOR_BLUE_R) << "Motor " << (int) id << " LED Red";
      EXPECT_EQ(motor->getLedGreen(), COLOR_BLUE_G) << "Motor " << (int) id << " LED Green";
      EXPECT_EQ(motor->getLedBlue(), COLOR_BLUE_B) << "Motor " << (int) id << " LED Blue";
    }
  }
}

TEST_F(HardwareInterfaceTest, LED_GreenWhenTorqueOff)
{
  // Disable torque and verify LED is green
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

  std::this_thread::sleep_for(500ms);

  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      EXPECT_EQ(motor->getLedRed(), COLOR_GREEN_R) << "Motor " << (int) id << " R";
      EXPECT_EQ(motor->getLedGreen(), COLOR_GREEN_G) << "Motor " << (int) id << " G";
      EXPECT_EQ(motor->getLedBlue(), COLOR_GREEN_B) << "Motor " << (int) id << " B";
    }
  }
}

TEST_F(HardwareInterfaceTest, LED_OrangeWhenEStopActive)
{
  // Trigger E-Stop and verify LED is orange
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0);

  std_msgs::msg::Bool msg;
  msg.data = true;
  estop_pub->publish(msg);

  std::this_thread::sleep_for(500ms);

  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor) {
      EXPECT_EQ(motor->getLedRed(), COLOR_ORANGE_R) << "Motor " << (int) id << " R";
      EXPECT_EQ(motor->getLedGreen(), COLOR_ORANGE_G) << "Motor " << (int) id << " G";
      EXPECT_EQ(motor->getLedBlue(), COLOR_ORANGE_B) << "Motor " << (int) id << " B";
    }
  }

  // Clean up - disable e-stop
  msg.data = false;
  estop_pub->publish(msg);
  std::this_thread::sleep_for(200ms);
}

TEST_F(HardwareInterfaceTest, MockMotor_VerifyPhysicsSimulation)
{
  // This test verifies the mock motor physics work correctly
  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);

  // Verify motor is in position mode with torque enabled
  EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 1);

  // Verify motor can read/write registers correctly
  uint16_t goal_pos_addr = motor->getAddress("goal_position");
  EXPECT_GT(goal_pos_addr, 0) << "goal_position address should be valid";

  // Write a goal position directly and verify physics simulation moves toward it
  int32_t target_ticks = 10000;  // Small positive position
  motor->write4Byte(goal_pos_addr, static_cast<uint32_t>(target_ticks));

  // Let physics update
  for (int i = 0; i < 100; ++i) {
    motor->update(0.01);
  }

  // Verify motor moved toward goal
  EXPECT_GT(motor->getCurrentPosition(), 0.0) << "Motor should have moved toward positive goal";
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
