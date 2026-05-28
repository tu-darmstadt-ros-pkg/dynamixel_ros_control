// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Partial Torque (ignore_joints) Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, PartialTorque_IgnoreJointsKeepsSpecifiedJointsTorqued)
{
  // Verify that disabling torque with ignore_joints keeps the ignored joint torqued.
  std::this_thread::sleep_for(200ms);

  // 1. Verify initial state: all arm + gripper motors have torque ON
  auto gripper = MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  ASSERT_NE(gripper, nullptr);
  uint16_t torque_addr = gripper->getAddress("torque_enable");
  EXPECT_EQ(gripper->read1Byte(torque_addr), 1) << "Gripper should start with torque ON";

  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 1)
        << "Arm motor " << static_cast<int>(id) << " should start with torque ON";
  }

  // 2. Disable torque with ignore_joints=["gripper_servo_joint"]
  auto torque_client = createTorqueClient();
  ASSERT_TRUE(setTorque(torque_client, false, {"gripper_servo_joint"}));
  std::this_thread::sleep_for(300ms);

  // 3. Verify arm motors have torque OFF
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 0)
        << "Arm motor " << static_cast<int>(id) << " torque should be OFF";
  }

  // 4. Verify gripper still has torque ON
  EXPECT_EQ(gripper->read1Byte(torque_addr), 1) << "Gripper torque should still be ON (ignored)";
}

TEST_F(HardwareInterfaceTest, PartialTorque_IgnoreJointsDoesNotResetIgnoredGoals)
{
  // Verify that re-enabling torque with ignore_joints does not reset the ignored joint's goal.
  // The gripper's goal position was set to current position during on_activate.
  // A torque cycle with ignore_joints should NOT overwrite it via resetGoalStateAndVerify.
  std::this_thread::sleep_for(200ms);

  // 1. Record gripper goal position from the register
  auto gripper = MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  ASSERT_NE(gripper, nullptr);
  uint16_t goal_pos_addr = gripper->getAddress("goal_position");
  ASSERT_GT(goal_pos_addr, 0u) << "Goal position register should exist";
  int32_t gripper_goal_before = gripper->read4ByteSigned(goal_pos_addr);

  // 2. Disable torque ignoring the gripper
  auto torque_client = createTorqueClient();
  ASSERT_TRUE(setTorque(torque_client, false, {"gripper_servo_joint"}));
  std::this_thread::sleep_for(300ms);

  // 3. Re-enable torque ignoring the gripper
  ASSERT_TRUE(setTorque(torque_client, true, {"gripper_servo_joint"}));
  std::this_thread::sleep_for(300ms);

  // 4. Verify gripper goal was NOT reset
  int32_t gripper_goal_after = gripper->read4ByteSigned(goal_pos_addr);
  EXPECT_EQ(gripper_goal_after, gripper_goal_before)
      << "Gripper goal should be preserved when using ignore_joints. "
      << "Before: " << gripper_goal_before << ", After: " << gripper_goal_after;
}

TEST_F(HardwareInterfaceTest, PartialTorque_DesiredStateNotUpdatedWithIgnoreJoints)
{
  // Verify that desired_torque_state_ is not updated when ignore_joints is used.
  // After a partial torque-off (with ignore_joints), calling setTorque(true) without
  // ignore_joints should succeed and re-enable all joints — proving the internal
  // desired state was not corrupted to "off".

  // 1. Initial state: torque ON for all
  auto torque_client = createTorqueClient();
  std::this_thread::sleep_for(200ms);

  // 2. Disable torque with ignore_joints (should NOT update desired_torque_state_)
  ASSERT_TRUE(setTorque(torque_client, false, {"gripper_servo_joint"}));
  std::this_thread::sleep_for(300ms);

  // Verify arm motors are off, gripper still on
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 0)
        << "Arm motor " << static_cast<int>(id) << " should be OFF";
  }
  auto gripper = MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  ASSERT_NE(gripper, nullptr);
  EXPECT_EQ(gripper->read1Byte(gripper->getAddress("torque_enable")), 1) << "Gripper should still be ON";

  // 3. Re-enable torque for ALL joints (no ignore_joints).
  // This should succeed because desired_torque_state_ was not changed by the partial call.
  ASSERT_TRUE(setTorque(torque_client, true));
  std::this_thread::sleep_for(300ms);

  // 4. Verify all motors are torqued on
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 1)
        << "Arm motor " << static_cast<int>(id) << " should be ON after full re-enable";
  }
  EXPECT_EQ(gripper->read1Byte(gripper->getAddress("torque_enable")), 1) << "Gripper should be ON after full re-enable";
}

// ============================================================================
// Goal Joint State Publisher Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, GoalStatePublisher_PublishesOnTopic)
{
  // Verify that the goal_joint_states topic is published when publish_goal_joint_states is enabled.
  sensor_msgs::msg::JointState::SharedPtr received_msg;
  auto sub = tester_node_->create_subscription<sensor_msgs::msg::JointState>(
      "/athena_arm_interface_node/goal_joint_states", rclcpp::SystemDefaultsQoS(),
      [&received_msg](const sensor_msgs::msg::JointState::SharedPtr msg) { received_msg = msg; });

  // Wait for a message
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && !received_msg) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  ASSERT_NE(received_msg, nullptr) << "Should receive goal_joint_states message";
  EXPECT_FALSE(received_msg->name.empty()) << "Message should contain joint names";
  EXPECT_EQ(received_msg->name.size(), received_msg->position.size()) << "Names and positions should have same size";
}

TEST_F(HardwareInterfaceTest, GoalStatePublisher_ReflectsCommandedGoals)
{
  // Verify that published goal values reflect what a controller commands.

  // 1. Subscribe to goal joint states
  sensor_msgs::msg::JointState::SharedPtr received_msg;
  std::mutex msg_mutex;
  auto sub = tester_node_->create_subscription<sensor_msgs::msg::JointState>(
      "/athena_arm_interface_node/goal_joint_states", rclcpp::SystemDefaultsQoS(),
      [&received_msg, &msg_mutex](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(msg_mutex);
        received_msg = msg;
      });

  // 2. Activate position controller and send command
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0u);

  constexpr double TARGET_POSITION = 0.7;
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {TARGET_POSITION, TARGET_POSITION, TARGET_POSITION, TARGET_POSITION,
              TARGET_POSITION, TARGET_POSITION, TARGET_POSITION};

  // 3. Publish repeatedly and wait for the goal to appear in the subscribed message.
  // The command needs to be picked up by the controller update cycle and written through
  // the hardware interface before it appears in goal_joint_states.
  bool goal_reflected = false;
  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && !goal_reflected) {
    pub->publish(cmd);
    std::this_thread::sleep_for(100ms);
    executor_->spin_some();

    std::lock_guard<std::mutex> lock(msg_mutex);
    if (received_msg) {
      auto it = std::find(received_msg->name.begin(), received_msg->name.end(), "arm_joint_1");
      if (it != received_msg->name.end()) {
        size_t idx = std::distance(received_msg->name.begin(), it);
        if (std::abs(received_msg->position[idx] - TARGET_POSITION) < 0.1) {
          goal_reflected = true;
        }
      }
    }
  }

  EXPECT_TRUE(goal_reflected) << "Published goal position should reflect the commanded value within timeout";
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
