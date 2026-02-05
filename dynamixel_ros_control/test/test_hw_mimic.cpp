// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Mimic Joint Tests - Verify gripper mimic joints follow the main joint
// ============================================================================

// Mimic joint multipliers from URDF:
// gripper_joint_r1: multiplier=1.0, offset=0
// gripper_joint_r2: multiplier=0.909, offset=0
// gripper_joint_l1: multiplier=1.0, offset=0
// gripper_joint_l2: multiplier=0.909, offset=0
constexpr double MIMIC_R1_MULTIPLIER = 1.0;
constexpr double MIMIC_R2_MULTIPLIER = 0.909;
constexpr double MIMIC_L1_MULTIPLIER = 1.0;
constexpr double MIMIC_L2_MULTIPLIER = 0.909;

TEST_F(HardwareInterfaceTest, MimicJoint_GripperMimicJointsFollowServo)
{
  // 1. First activate joint_state_broadcaster to publish joint states
  ASSERT_TRUE(loadAndActivateController("joint_state_broadcaster"));

  // 2. Activate gripper position controller
  ASSERT_TRUE(loadAndActivateController("gripper_position_controller"));

  // 3. Verify gripper motor exists
  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  ASSERT_NE(motor, nullptr) << "Gripper motor not found";

  // 4. Create publisher for gripper commands
  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/gripper_position_controller/commands", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0);

  // 5. Subscribe to joint_states to read mimic joint positions
  sensor_msgs::msg::JointState::SharedPtr last_joint_state;
  std::mutex js_mutex;
  auto joint_state_sub = tester_node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(js_mutex);
        last_joint_state = msg;
      });

  // Wait for first joint state message
  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(js_mutex);
      if (last_joint_state) {
        break;
      }
    }
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_NE(last_joint_state, nullptr) << "Did not receive joint_states";

  // 6. Send position command to gripper
  std_msgs::msg::Float64MultiArray cmd;
  double target_position = 0.5;  // radians
  cmd.data = {target_position};
  pub->publish(cmd);

  // 7. Wait for motor to reach target and joint states to update, spinning to process callbacks
  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  // 8. Get final joint state
  sensor_msgs::msg::JointState::SharedPtr final_state;
  {
    std::lock_guard<std::mutex> lock(js_mutex);
    final_state = last_joint_state;
  }
  ASSERT_NE(final_state, nullptr);

  // 9. Find joint positions in joint_state message
  auto findJointPosition = [&final_state](const std::string& joint_name) -> double {
    for (size_t i = 0; i < final_state->name.size(); ++i) {
      if (final_state->name[i] == joint_name) {
        return final_state->position[i];
      }
    }
    return std::numeric_limits<double>::quiet_NaN();
  };

  double servo_pos = findJointPosition("gripper_servo_joint");
  double r1_pos = findJointPosition("gripper_joint_r1");
  double r2_pos = findJointPosition("gripper_joint_r2");
  double l1_pos = findJointPosition("gripper_joint_l1");
  double l2_pos = findJointPosition("gripper_joint_l2");

  // 10. Verify main servo joint reached target
  EXPECT_FALSE(std::isnan(servo_pos)) << "gripper_servo_joint not found in joint_states";
  EXPECT_NEAR(servo_pos, target_position, 0.2) << "Gripper servo did not reach target position";

  // 11. Verify mimic joints follow with correct multipliers
  EXPECT_FALSE(std::isnan(r1_pos)) << "gripper_joint_r1 not found in joint_states";
  EXPECT_NEAR(r1_pos, servo_pos * MIMIC_R1_MULTIPLIER, 0.1)
      << "gripper_joint_r1 should be servo_pos * " << MIMIC_R1_MULTIPLIER;

  EXPECT_FALSE(std::isnan(r2_pos)) << "gripper_joint_r2 not found in joint_states";
  EXPECT_NEAR(r2_pos, servo_pos * MIMIC_R2_MULTIPLIER, 0.1)
      << "gripper_joint_r2 should be servo_pos * " << MIMIC_R2_MULTIPLIER;

  EXPECT_FALSE(std::isnan(l1_pos)) << "gripper_joint_l1 not found in joint_states";
  EXPECT_NEAR(l1_pos, servo_pos * MIMIC_L1_MULTIPLIER, 0.1)
      << "gripper_joint_l1 should be servo_pos * " << MIMIC_L1_MULTIPLIER;

  EXPECT_FALSE(std::isnan(l2_pos)) << "gripper_joint_l2 not found in joint_states";
  EXPECT_NEAR(l2_pos, servo_pos * MIMIC_L2_MULTIPLIER, 0.1)
      << "gripper_joint_l2 should be servo_pos * " << MIMIC_L2_MULTIPLIER;
}

TEST_F(HardwareInterfaceTest, MimicJoint_GripperMimicJointsUpdateDuringMovement)
{
  // Test that mimic joints update continuously as servo moves, not just at final position

  // First activate joint_state_broadcaster to publish joint states
  ASSERT_TRUE(loadAndActivateController("joint_state_broadcaster"));
  ASSERT_TRUE(loadAndActivateController("gripper_position_controller"));

  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  ASSERT_NE(motor, nullptr);

  // Subscribe to joint_states
  std::vector<sensor_msgs::msg::JointState::SharedPtr> joint_state_history;
  std::mutex js_mutex;
  auto joint_state_sub = tester_node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(js_mutex);
        joint_state_history.push_back(msg);
      });

  // Create publisher
  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/gripper_position_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(pub, 5s));

  // Clear history and send command to a far position
  {
    std::lock_guard<std::mutex> lock(js_mutex);
    joint_state_history.clear();
  }

  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {1.0};  // Move to 1 radian
  pub->publish(cmd);

  // Wait for movement to complete, spinning to process callbacks and collect joint states
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  // Analyze joint state history
  std::lock_guard<std::mutex> lock(js_mutex);
  ASSERT_GT(joint_state_history.size(), 10) << "Should have collected multiple joint states during movement";

  auto findJointPosition = [](const sensor_msgs::msg::JointState::SharedPtr& state,
                              const std::string& joint_name) -> double {
    for (size_t i = 0; i < state->name.size(); ++i) {
      if (state->name[i] == joint_name) {
        return state->position[i];
      }
    }
    return std::numeric_limits<double>::quiet_NaN();
  };

  // Check that mimic joints updated throughout the movement
  int valid_samples = 0;
  for (const auto& state : joint_state_history) {
    double servo_pos = findJointPosition(state, "gripper_servo_joint");
    double r1_pos = findJointPosition(state, "gripper_joint_r1");

    if (!std::isnan(servo_pos) && !std::isnan(r1_pos)) {
      // The mimic joint should always be within tolerance of expected value
      double expected_r1 = servo_pos * MIMIC_R1_MULTIPLIER;
      EXPECT_NEAR(r1_pos, expected_r1, 0.15) << "Mimic joint r1 out of sync at servo_pos=" << servo_pos;
      valid_samples++;
    }
  }

  EXPECT_GT(valid_samples, 5) << "Should have validated multiple samples";
}

TEST_F(HardwareInterfaceTest, MimicJoint_GripperStateInterfacesExist)
{
  // Verify that mimic joints have state interfaces exported

  // Wait a bit for hardware interfaces to be fully available
  std::this_thread::sleep_for(500ms);

  // List hardware interfaces
  auto request = std::make_shared<ListHardwareInterfaces::Request>();
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto resp =
      hector_testing_utils::call_service<ListHardwareInterfaces>(list_hw_client_->get(), request, *executor_, options);

  ASSERT_NE(resp, nullptr);

  // Check for mimic joint state interfaces
  std::vector<std::string> expected_mimic_joints = {"gripper_joint_r1", "gripper_joint_r2", "gripper_joint_l1",
                                                    "gripper_joint_l2"};

  for (const auto& mimic_joint : expected_mimic_joints) {
    bool found_position = false;

    for (const auto& iface : resp->state_interfaces) {
      if (iface.name.find(mimic_joint + "/position") != std::string::npos) {
        found_position = true;
        break;
      }
    }

    EXPECT_TRUE(found_position) << "State interface " << mimic_joint << "/position should exist";
  }
}

TEST_F(HardwareInterfaceTest, MimicJoint_GripperVelocityFollowsServo)
{
  // Test that mimic joint velocities also follow the servo velocity with multiplier

  // First activate joint_state_broadcaster to publish joint states
  ASSERT_TRUE(loadAndActivateController("joint_state_broadcaster"));
  ASSERT_TRUE(loadAndActivateController("gripper_position_controller"));

  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  ASSERT_NE(motor, nullptr);

  // Subscribe to joint_states
  sensor_msgs::msg::JointState::SharedPtr last_joint_state;
  std::mutex js_mutex;
  auto joint_state_sub = tester_node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(js_mutex);
        last_joint_state = msg;
      });

  // Create publisher
  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/gripper_position_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(pub, 5s));

  // Wait for first joint state to arrive
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(js_mutex);
      if (last_joint_state) {
        break;
      }
    }
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  // Send command to initiate movement
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.8};
  pub->publish(cmd);

  // Wait for movement to start, spinning to receive joint state updates
  deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  sensor_msgs::msg::JointState::SharedPtr state;
  {
    std::lock_guard<std::mutex> lock(js_mutex);
    state = last_joint_state;
  }
  ASSERT_NE(state, nullptr);

  // Skip velocity checks if joint_state_broadcaster doesn't publish velocities
  if (state->velocity.empty()) {
    GTEST_SKIP() << "Joint state broadcaster does not publish velocities, skipping velocity test";
  }

  auto findJointVelocity = [&state](const std::string& joint_name) -> double {
    for (size_t i = 0; i < state->name.size(); ++i) {
      if (state->name[i] == joint_name && i < state->velocity.size()) {
        return state->velocity[i];
      }
    }
    return std::numeric_limits<double>::quiet_NaN();
  };

  double servo_vel = findJointVelocity("gripper_servo_joint");
  double r1_vel = findJointVelocity("gripper_joint_r1");
  double r2_vel = findJointVelocity("gripper_joint_r2");

  // If servo is moving, mimic joints should also report velocity
  if (!std::isnan(servo_vel) && std::abs(servo_vel) > 0.01) {
    if (!std::isnan(r1_vel)) {
      EXPECT_NEAR(r1_vel, servo_vel * MIMIC_R1_MULTIPLIER, 0.5)
          << "Mimic joint r1 velocity should follow servo with multiplier";
    }
    if (!std::isnan(r2_vel)) {
      EXPECT_NEAR(r2_vel, servo_vel * MIMIC_R2_MULTIPLIER, 0.5)
          << "Mimic joint r2 velocity should follow servo with multiplier";
    }
  }
}

TEST_F(HardwareInterfaceTest, MimicJoint_NoCommandInterfacesForMimicJoints)
{
  // Verify that mimic joints do NOT have command interfaces (they follow the main joint)

  std::this_thread::sleep_for(500ms);

  auto request = std::make_shared<ListHardwareInterfaces::Request>();
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  auto resp =
      hector_testing_utils::call_service<ListHardwareInterfaces>(list_hw_client_->get(), request, *executor_, options);

  ASSERT_NE(resp, nullptr);

  std::vector<std::string> mimic_joints = {"gripper_joint_r1", "gripper_joint_r2", "gripper_joint_l1",
                                           "gripper_joint_l2"};

  for (const auto& mimic_joint : mimic_joints) {
    for (const auto& iface : resp->command_interfaces) {
      EXPECT_TRUE(iface.name.find(mimic_joint) == std::string::npos)
          << "Mimic joint " << mimic_joint << " should NOT have command interface: " << iface.name;
    }
  }

  // But main servo joint SHOULD have command interface
  bool servo_has_command = false;
  for (const auto& iface : resp->command_interfaces) {
    if (iface.name.find("gripper_servo_joint/position") != std::string::npos) {
      servo_has_command = true;
      break;
    }
  }
  EXPECT_TRUE(servo_has_command) << "gripper_servo_joint should have position command interface";
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
