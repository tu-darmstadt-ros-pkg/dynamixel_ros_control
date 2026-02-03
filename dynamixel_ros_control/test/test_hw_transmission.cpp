// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Transmission Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, Transmission_FlipperVelocityReduction)
{
  // Test that flipper transmission correctly applies mechanical reduction
  // FL: -2.0, FR: 2.0, BL: 2.0, BR: -2.0
  // When joint velocity is 1.0 rad/s, actuator velocity should be 2.0 rad/s (with sign)

  ASSERT_TRUE(loadAndActivateController("flipper_velocity_controller"));

  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_velocity_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Record initial positions
  std::vector<uint8_t> flipper_ids = {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID};
  std::map<uint8_t, double> initial_positions;
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions[id] = motor->getCurrentPosition();
  }

  // Send joint velocity command of 1.0 rad/s to all flippers
  std_msgs::msg::Float64MultiArray cmd;
  double joint_velocity = 1.0;
  cmd.data = {joint_velocity, joint_velocity, joint_velocity, joint_velocity};
  pub->publish(cmd);

  // Run for 1 second
  double test_duration = 1.0;
  std::this_thread::sleep_for(std::chrono::duration<double>(test_duration));

  // Calculate expected position changes
  // Actuator velocity = joint velocity * mechanical_reduction
  // Expected position change = actuator_velocity * time
  std::map<uint8_t, double> expected_reductions = {
      {FLIPPER_FL_ID, -2.0}, {FLIPPER_FR_ID, 2.0}, {FLIPPER_BL_ID, 2.0}, {FLIPPER_BR_ID, -2.0}};

  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double current_pos = motor->getCurrentPosition();
    double position_change = current_pos - initial_positions[id];
    double expected_actuator_velocity = joint_velocity * expected_reductions[id];
    double expected_position_change = expected_actuator_velocity * test_duration;

    // Allow some tolerance for timing and physics simulation
    EXPECT_NEAR(position_change, expected_position_change, 0.5)
        << "Flipper motor " << (int) id << " position change doesn't match transmission ratio. "
        << "Expected: " << expected_position_change << ", Got: " << position_change;
  }
}

TEST_F(HardwareInterfaceTest, Transmission_FlipperPositionReduction)
{
  // Test position control respects transmission ratio
  // For FL with reduction=-2.0: joint position change of 0.5 rad -> actuator change of -1.0 rad

  ASSERT_TRUE(loadAndActivateController("flipper_position_controller"));

  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // First send to position 0 and wait
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.0, 0.0, 0.0, 0.0};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  // Record positions at joint position 0
  std::map<uint8_t, double> positions_at_zero;
  std::vector<uint8_t> flipper_ids = {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID};
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_at_zero[id] = motor->getCurrentPosition();
  }

  // Send joint position command
  double joint_position = 0.5;  // rad
  cmd.data = {joint_position, joint_position, joint_position, joint_position};
  pub->publish(cmd);

  // Wait for motors to reach position
  std::this_thread::sleep_for(3s);

  // Verify actuator position changes match transmission ratios
  std::map<uint8_t, double> expected_reductions = {
      {FLIPPER_FL_ID, -2.0}, {FLIPPER_FR_ID, 2.0}, {FLIPPER_BL_ID, 2.0}, {FLIPPER_BR_ID, -2.0}};

  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double actuator_pos = motor->getCurrentPosition();
    double actuator_change = actuator_pos - positions_at_zero[id];
    double expected_actuator_change = joint_position * expected_reductions[id];

    EXPECT_NEAR(actuator_change, expected_actuator_change, 0.3)
        << "Flipper motor " << (int) id << " position change doesn't match transmission ratio. "
        << "Expected change: " << expected_actuator_change << ", Got: " << actuator_change;
  }
}

TEST_F(HardwareInterfaceTest, TransmissionOffset_AdjustFlipperOffset)
{
  // Test that calling adjust_transmission_offsets service:
  // 1. Deactivates active flipper controllers
  // 2. Adjusts offset so position jumps to external measurement
  // 3. Position should not change during service call

  using AdjustOffsets = hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets;

  // 1. First activate flipper controller and move to a position
  ASSERT_TRUE(loadAndActivateController("flipper_position_controller"));

  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // Move to initial position
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.3, 0.3, 0.3, 0.3};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  // 2. Record current actuator positions
  std::vector<uint8_t> flipper_ids = {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID};
  std::map<uint8_t, double> positions_before;
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_before[id] = motor->getCurrentPosition();
  }

  // 3. Call adjust_transmission_offsets service
  auto offset_client =
      tester_node_->create_test_client<AdjustOffsets>("/athena_flipper_interface/adjust_transmission_offsets");
  ASSERT_TRUE(offset_client->wait_for_service(*executor_, 5s)) << "adjust_transmission_offsets service not available";

  auto request = std::make_shared<AdjustOffsets::Request>();
  // Set external measurement - this is the "true" joint position from external sensor
  request->external_joint_measurements.name = {"flipper_fl_joint", "flipper_fr_joint", "flipper_bl_joint",
                                               "flipper_br_joint"};
  double external_position = 1.5708;  // ~90 degrees
  request->external_joint_measurements.position = {external_position, external_position, external_position,
                                                   external_position};

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 10s;
  options.response_timeout = 10s;
  auto resp = hector_testing_utils::call_service<AdjustOffsets>(offset_client->get(), request, *executor_, options);

  ASSERT_NE(resp, nullptr) << "Service call failed";
  EXPECT_TRUE(resp->success) << "Offset adjustment should succeed: " << resp->message;

  // 4. Verify controller was deactivated during adjustment
  // (The pre_callback deactivates controllers)
  auto list_resp = list_controllers();
  ASSERT_NE(list_resp, nullptr);
  auto states = states_from_list(*list_resp);
  // Controller should be inactive after offset adjustment (deactivated by pre_callback)
  EXPECT_EQ(states["flipper_position_controller"], "inactive")
      << "Controller should be deactivated after offset adjustment";

  // 5. Verify actuator positions didn't change (only offset changed)
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double pos_after = motor->getCurrentPosition();
    EXPECT_NEAR(pos_after, positions_before[id], 0.1)
        << "Actuator position should not change during offset adjustment for motor " << (int) id;
  }
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
