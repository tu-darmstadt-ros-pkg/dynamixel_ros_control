// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "direct_hardware_fixture.hpp"

#include <cstdint>
#include <limits>
#include <optional>

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

  ASSERT_TRUE(waitForSubscribers(pub, 5s));

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

  ASSERT_TRUE(waitForSubscribers(pub, 5s));

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
  // 1. Is refused while a controller commands the flippers, and leaves the controller running
  // 2. Adjusts offset so position jumps to external measurement once it is stopped
  // 3. Position should not change during service call

  using AdjustOffsets = hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets;

  // 1. First activate flipper controller and move to a position
  ASSERT_TRUE(loadAndActivateController("flipper_position_controller"));

  auto pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_position_controller/commands", 10);

  ASSERT_TRUE(waitForSubscribers(pub, 5s));

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
      tester_node_->create_test_client<AdjustOffsets>("/athena_flipper_interface_node/adjust_transmission_offsets");
  ASSERT_TRUE(offset_client->wait_for_service(*executor_, 5s)) << "adjust_transmission_offsets service not available";

  auto request = std::make_shared<AdjustOffsets::Request>();
  // Set external measurement. This is the "true" joint position from external sensor
  request->external_joint_measurements.name = {"flipper_fl_joint", "flipper_fr_joint", "flipper_bl_joint",
                                               "flipper_br_joint"};
  double external_position = 1.5708;  // ~90 degrees
  request->external_joint_measurements.position = {external_position, external_position, external_position,
                                                   external_position};

  auto resp = callService<AdjustOffsets>(offset_client, request, 10s);
  ASSERT_NE(resp, nullptr) << "Service call failed";
  EXPECT_FALSE(resp->success) << "Offset adjustment went through while a controller commands the flippers";
  EXPECT_TRUE(waitForControllerState("flipper_position_controller", "active", 1s))
      << "The offset service stopped the controller behind the controller manager's back";

  // 4. Stopped, the adjustment goes through
  ASSERT_TRUE(switch_controllers({}, {"flipper_position_controller"}));
  resp = callService<AdjustOffsets>(offset_client, request, 10s);
  ASSERT_NE(resp, nullptr) << "Service call failed";
  EXPECT_TRUE(resp->success) << "Offset adjustment should succeed: " << resp->message;

  // 5. Verify actuator positions didn't change (only offset changed)
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double pos_after = motor->getCurrentPosition();
    EXPECT_NEAR(pos_after, positions_before[id], 0.1)
        << "Actuator position should not change during offset adjustment for motor " << (int) id;
  }
}

TEST_F(HardwareInterfaceTest, TransmissionOffset_JointPositionMatchesExternalMeasurement)
{
  // CRITICAL TEST: After calling adjust_transmission_offsets, the joint position
  // (as read from state interface / joint_states topic) should equal the external measurement value.
  // This verifies the offset was correctly computed and applied.

  using AdjustOffsets = hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets;

  // 1. First activate joint_state_broadcaster to publish joint states
  ASSERT_TRUE(loadAndActivateController("joint_state_broadcaster"));

  // 2. Create subscriber to read joint states
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state;
  std::mutex joint_state_mutex;
  auto joint_state_sub = tester_node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [&](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_state_mutex);
        latest_joint_state = msg;
      });

  // Wait for first joint state message
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex);
      if (latest_joint_state) {
        break;
      }
    }
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex);
    ASSERT_NE(latest_joint_state, nullptr) << "Failed to receive joint states";
  }

  // 3. Call adjust_transmission_offsets service with specific external measurements
  auto offset_client =
      tester_node_->create_test_client<AdjustOffsets>("/athena_flipper_interface_node/adjust_transmission_offsets");
  ASSERT_TRUE(offset_client->wait_for_service(*executor_, 5s)) << "adjust_transmission_offsets service not available";

  auto request = std::make_shared<AdjustOffsets::Request>();
  // Set external measurement. This is the "true" joint position from external sensor
  std::vector<std::string> flipper_joints = {"flipper_fl_joint", "flipper_fr_joint", "flipper_bl_joint",
                                             "flipper_br_joint"};
  std::vector<double> external_positions = {0.7854, -0.5236, 1.0472, -0.3491};  // Different values for each
  request->external_joint_measurements.name = flipper_joints;
  request->external_joint_measurements.position = external_positions;

  auto resp = callService<AdjustOffsets>(offset_client, request, 10s);

  ASSERT_NE(resp, nullptr) << "Service call failed";
  EXPECT_TRUE(resp->success) << "Offset adjustment should succeed: " << resp->message;

  // 4. Wait for joint states to update with new offset
  std::this_thread::sleep_for(500ms);

  // Read fresh joint state
  latest_joint_state = nullptr;
  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex);
      if (latest_joint_state) {
        break;
      }
    }
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  // 5. CRITICAL VERIFICATION: Joint positions should match the external measurements
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex);
    ASSERT_NE(latest_joint_state, nullptr) << "Failed to receive updated joint states";

    for (size_t i = 0; i < flipper_joints.size(); ++i) {
      const std::string& joint_name = flipper_joints[i];
      double expected_position = external_positions[i];

      // Find this joint in the joint state message
      auto it = std::find(latest_joint_state->name.begin(), latest_joint_state->name.end(), joint_name);
      ASSERT_NE(it, latest_joint_state->name.end()) << "Joint " << joint_name << " not found in joint_states";

      size_t idx = std::distance(latest_joint_state->name.begin(), it);
      ASSERT_LT(idx, latest_joint_state->position.size()) << "Position index out of bounds for " << joint_name;

      double actual_position = latest_joint_state->position[idx];

      EXPECT_NEAR(actual_position, expected_position, 0.05)
          << "Joint " << joint_name << " position should match external measurement after offset adjustment. "
          << "Expected: " << expected_position << " rad, Got: " << actual_position << " rad. "
          << "This indicates the transmission offset was not correctly applied!";
    }
  }
}

// The flipper's state transmission absorbs 2pi jumps of the actuator so a lost multi-turn count
// after a power cut does not move the joint. It must not absorb a real turn, and whatever it
// decides, the command side has to agree with it. These tests share the gap setup: the flipper
// interface is deactivated, something happens to the motor, it is reactivated.
class FlipperGapTest : public HardwareInterfaceTest
{
protected:
  const std::vector<std::string> flipper_joints = {"flipper_fl_joint", "flipper_fr_joint", "flipper_bl_joint",
                                                   "flipper_br_joint"};  // the position controller's order
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  std::mutex joint_state_mutex_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  void startListening()
  {
    ASSERT_TRUE(loadAndActivateController("joint_state_broadcaster"));
    joint_state_sub_ = tester_node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, [this](sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(joint_state_mutex_);
          latest_joint_state_ = msg;
        });
  }

  /// The flipper joint positions from a joint_states message published after this call.
  std::optional<std::vector<double>> freshFlipperPositions(std::chrono::seconds timeout = 5s)
  {
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      latest_joint_state_.reset();
    }
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        if (latest_joint_state_) {
          std::vector<double> positions;
          for (const auto& joint : flipper_joints) {
            const auto it = std::find(latest_joint_state_->name.begin(), latest_joint_state_->name.end(), joint);
            if (it == latest_joint_state_->name.end()) {
              break;
            }
            positions.push_back(latest_joint_state_->position[std::distance(latest_joint_state_->name.begin(), it)]);
          }
          if (positions.size() == flipper_joints.size()) {
            return positions;
          }
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    return std::nullopt;
  }

  void deactivateFlippers()
  {
    ASSERT_TRUE(setHardwareComponentState("athena_flipper_interface",
                                          lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive"));
    std::this_thread::sleep_for(300ms);
  }

  void reactivateFlippers()
  {
    ASSERT_TRUE(setHardwareComponentState("athena_flipper_interface", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
                                          "active"));
    ASSERT_TRUE(waitForHardwareInterfacesActive(10s));
  }

  /// Command every flipper to the position the joint state reports, as any teleop or controller
  /// seeding from the state would, and return how far the FL actuator moved as a result.
  double actuatorMoveOnHold(MockDynamixel& motor)
  {
    const auto held = freshFlipperPositions();
    EXPECT_TRUE(held.has_value());
    if (!held) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const double before = motor.getCurrentPosition();
    EXPECT_TRUE(loadAndActivateController("flipper_position_controller"));
    auto pub =
        tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_position_controller/commands", 10);
    EXPECT_TRUE(waitForSubscribers(pub, 5s));
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = *held;
    pub->publish(cmd);
    std::this_thread::sleep_for(1500ms);
    return motor.getCurrentPosition() - before;
  }
};

// A flipper turned one actuator turn by hand (~180 degrees on the flipper, as for transport) while
// the motor is powered but not driving is a move: the joint state must show it, and a hold command
// built from that state must not move it.
TEST_F(FlipperGapTest, HandTurnWhilePoweredIsReportedAndHoldDoesNotMove)
{
  startListening();
  const auto before = freshFlipperPositions();
  ASSERT_TRUE(before.has_value());
  auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_NE(motor, nullptr);

  deactivateFlippers();
  motor->powerCycle();  // the power cut turns torque off. The position, small so far, is unchanged by the wrap
  motor->moveByHand(2.0 * M_PI);
  reactivateFlippers();

  const auto after = freshFlipperPositions();
  ASSERT_TRUE(after.has_value());
  // FL's reduction is -2: one actuator turn is -pi on the joint.
  EXPECT_NEAR((*after)[0] - (*before)[0], -M_PI, 0.05)
      << "The flipper was turned by hand while the interface was inactive, but the joint state does not "
         "show the move: the jump detector took it for a power cycle.";

  EXPECT_NEAR(actuatorMoveOnHold(*motor), 0.0, 0.1)
      << "A hold-position command built from the joint state moved the actuator: state and command "
         "transmissions disagree on the joint's offset.";
}

// A power cut while the actuator sits three turns out brings it back reporting its single-turn
// absolute position. That is a lost count, not a move: the joint state must not change.
TEST_F(FlipperGapTest, PowerCutThreeTurnsOutKeepsTheJointWhereItWas)
{
  startListening();
  auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_NE(motor, nullptr);

  // Get three turns out the honest way: a hand move, powered, that the detector reports as one.
  deactivateFlippers();
  motor->powerCycle();
  motor->moveByHand(3 * 2.0 * M_PI + 0.3);
  reactivateFlippers();
  const auto before = freshFlipperPositions();
  ASSERT_TRUE(before.has_value());
  ASSERT_GT(std::abs(motor->getCurrentPosition()), 2 * M_PI) << "Test premise: the actuator is several turns out";

  deactivateFlippers();
  motor->powerCycle();
  ASSERT_LT(std::abs(motor->getCurrentPosition()), M_PI) << "Test premise: the mock came back within one turn";
  reactivateFlippers();

  const auto after = freshFlipperPositions();
  ASSERT_TRUE(after.has_value());
  EXPECT_NEAR((*after)[0], (*before)[0], 0.05)
      << "The actuator lost its turn count in a power cut and the joint state followed it instead of "
         "being corrected.";

  EXPECT_NEAR(actuatorMoveOnHold(*motor), 0.0, 0.1)
      << "A hold-position command built from the joint state moved the actuator after the correction.";
}

// Current is a motor quantity, so it passes a transmission unchanged whatever the reduction. A current
// limit lost in SimpleTransmission left arm_joint_1 and arm_joint_7 on athena at their full current.
class CurrentPassThroughTest : public DirectHardwareTest
{
protected:
  /// max_current in place of current, under a 5 A limit on arm_joint_1 and flipper_fl_joint, since a
  /// fresh mock holds 0 in its limits
  static std::string capped(std::string urdf)
  {
    const std::string current = "<command_interface name=\"current\"/>";
    for (auto pos = urdf.find(current); pos != std::string::npos; pos = urdf.find(current, pos)) {
      urdf.replace(pos, current.size(), "<command_interface name=\"max_current\"/>");
    }
    for (const std::string id : {"<param name=\"id\">11</param>", "<param name=\"id\">1</param>"}) {
      urdf.insert(urdf.find(id) + id.size(), "<param name=\"registers.current_limit\">5</param>");
    }
    return urdf;
  }

  void expectCurrentPassesUnchanged(const std::string& joint, const uint8_t id)
  {
    const auto motor = MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    ASSERT_EQ(hardware_->perform_command_mode_switch({joint + "/position", joint + "/max_current"}, {}),
              hardware_interface::return_type::OK);
    setCommand(joint + "/position", states_.at(joint + "/position")->get_optional().value());
    setCommand(joint + "/max_current", 0.5);
    cycles(3);
    EXPECT_EQ(static_cast<int16_t>(motor->read2Byte(motor->getAddress("goal_current"))), 500)
        << "the claimed current did not reach the servo unchanged";
    // The read of the cycle sees it before the mock's physics overwrites it
    motor->write2Byte(motor->getAddress("present_current"), static_cast<uint16_t>(-300));
    ASSERT_EQ(cycle(), hardware_interface::return_type::OK);
    const auto current = states_.at(joint + "/current")->get_optional();
    ASSERT_TRUE(current.has_value());
    EXPECT_NEAR(*current, -0.3, 1e-9) << "the present current did not reach the joint unchanged";
  }
};

TEST_F(CurrentPassThroughTest, ThroughASimpleTransmission)
{
  // Effort is a transmission interface, current is not, and both read the same register
  startComponent("athena_arm_interface", [](std::string urdf) {
    const std::string effort = "<state_interface name=\"effort\"/>";
    for (auto pos = urdf.find(effort); pos != std::string::npos; pos = urdf.find(effort, pos)) {
      urdf.replace(pos, effort.size(), "<state_interface name=\"current\"/>");
    }
    return capped(urdf);
  });
  expectCurrentPassesUnchanged("arm_joint_1", ARM_JOINT_1_ID);
}

// Reduction -2, so scaling or a sign flip shows
TEST_F(CurrentPassThroughTest, ThroughAnAdjustableOffsetTransmission)
{
  startComponent("athena_flipper_interface", capped);
  expectCurrentPassesUnchanged("flipper_fl_joint", FLIPPER_FL_ID);
}

// Unclaimed, the current goal is the limit the plugin reset it to, not the joint side's zero
TEST_F(CurrentPassThroughTest, AnUnclaimedCurrentKeepsItsLimitThroughASimpleTransmission)
{
  startComponent("athena_arm_interface", [](std::string urdf) {
    const std::string anchor = "<param name=\"id\">11</param>";
    urdf.insert(urdf.find(anchor) + anchor.size(), "<param name=\"registers.current_limit\">5</param>");
    return urdf;
  });
  const auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);
  startInterface("arm_joint_1/position", states_.at("arm_joint_1/position")->get_optional().value());
  cycles(3);
  EXPECT_EQ(static_cast<int16_t>(motor->read2Byte(motor->getAddress("goal_current"))), 5000);
}

}  // namespace dynamixel_ros_control::test
