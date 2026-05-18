// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause
//
// Tests for the ~/goal_joint_states publisher. This is the realtime joint-state-style topic
// that mirrors commanded goals back to consumers (separate from ~/health and ~/manifest, which
// publish DiagnosticArray).

#include "test_hardware_interface_common.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

namespace dynamixel_ros_control::test {

TEST_F(HardwareInterfaceTest, GoalJointStates_Published)
{
  sensor_msgs::msg::JointState::SharedPtr msg;
  auto sub = tester_node_->create_subscription<sensor_msgs::msg::JointState>(
      "/athena_arm_interface/goal_joint_states", 10, [&msg](sensor_msgs::msg::JointState::SharedPtr m) { msg = m; });

  auto deadline = std::chrono::steady_clock::now() + 10s;
  while (!msg && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }

  ASSERT_NE(msg, nullptr) << "No goal joint state message received";
  EXPECT_GE(msg->name.size(), 7u) << "Should have at least 7 arm joint names";
  EXPECT_EQ(msg->position.size(), msg->name.size());
  EXPECT_EQ(msg->velocity.size(), msg->name.size());
  EXPECT_EQ(msg->effort.size(), msg->name.size());

  std::set<std::string> names(msg->name.begin(), msg->name.end());
  EXPECT_TRUE(names.count("arm_joint_1")) << "Missing arm_joint_1";
  EXPECT_TRUE(names.count("arm_joint_7")) << "Missing arm_joint_7";
}

TEST_F(HardwareInterfaceTest, GoalJointStates_ReflectsCommandedPosition)
{
  sensor_msgs::msg::JointState::SharedPtr goal_msg;
  auto goal_sub = tester_node_->create_subscription<sensor_msgs::msg::JointState>(
      "/athena_arm_interface/goal_joint_states", 10,
      [&goal_msg](sensor_msgs::msg::JointState::SharedPtr m) { goal_msg = m; });

  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0u);

  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  for (int i = 0; i < 20; ++i) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }

  ASSERT_NE(goal_msg, nullptr);
  for (size_t i = 0; i < goal_msg->name.size(); ++i) {
    if (goal_msg->name[i].find("arm_joint") != std::string::npos) {
      EXPECT_NEAR(goal_msg->position[i], 0.5, 0.15)
          << "Joint " << goal_msg->name[i] << " goal position should be near target";
    }
  }
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
