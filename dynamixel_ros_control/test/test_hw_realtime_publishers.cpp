// Copyright (c) 2026 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

#include <algorithm>
#include <cmath>
#include <map>

namespace dynamixel_ros_control::test {

// Verifies that the URDF flags publish_read_joint_states and publish_write_joint_states
// cause the corresponding ~/read_joint_states and ~/write_joint_states topics to be
// published, and that the values follow the configured flipper transmission ratios:
//  - goal_joint_states is joint-space (pre-transmission)
//  - write_joint_states is actuator-space (post-transmission)
//  - read_joint_states is actuator-space (pre-transmission application on the read path)
TEST_F(HardwareInterfaceTest, RealtimePublishers_PublishOnReadAndWrite)
{
  ASSERT_TRUE(loadAndActivateController("joint_state_broadcaster"));
  ASSERT_TRUE(loadAndActivateController("flipper_position_controller"));

  std::mutex mtx;
  sensor_msgs::msg::JointState::SharedPtr last_read, last_write, last_goal;

  auto sub_read =
      tester_node_->create_subscription<sensor_msgs::msg::JointState>("/athena_flipper_interface/read_joint_states",
                                                                      rclcpp::SystemDefaultsQoS(),
                                                                      [&](sensor_msgs::msg::JointState::SharedPtr m) {
                                                                        std::lock_guard<std::mutex> l(mtx);
                                                                        last_read = m;
                                                                      });
  auto sub_write =
      tester_node_->create_subscription<sensor_msgs::msg::JointState>("/athena_flipper_interface/write_joint_states",
                                                                      rclcpp::SystemDefaultsQoS(),
                                                                      [&](sensor_msgs::msg::JointState::SharedPtr m) {
                                                                        std::lock_guard<std::mutex> l(mtx);
                                                                        last_write = m;
                                                                      });
  auto sub_goal =
      tester_node_->create_subscription<sensor_msgs::msg::JointState>("/athena_flipper_interface/goal_joint_states",
                                                                      rclcpp::SystemDefaultsQoS(),
                                                                      [&](sensor_msgs::msg::JointState::SharedPtr m) {
                                                                        std::lock_guard<std::mutex> l(mtx);
                                                                        last_goal = m;
                                                                      });

  auto cmd_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_position_controller/commands", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && cmd_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(cmd_pub->get_subscription_count(), 0u);

  const std::vector<std::string> joints = {"flipper_fl_joint", "flipper_fr_joint", "flipper_bl_joint",
                                           "flipper_br_joint"};
  auto idx_of = [](const sensor_msgs::msg::JointState& js, const std::string& name) {
    auto it = std::find(js.name.begin(), js.name.end(), name);
    return it == js.name.end() ? size_t(-1) : static_cast<size_t>(std::distance(js.name.begin(), it));
  };

  // Drive the joints to two different commanded positions and capture goal/write at each.
  // The flipper transmission is AdjustableOffsetTransmission: actuator = joint * ratio + offset.
  // The offset cancels when we look at the delta across the two commands, so we cross-check
  // against the ratio without depending on whatever offset the transmission currently holds.
  auto drive_to = [&](double pos, sensor_msgs::msg::JointState::SharedPtr& out_goal,
                      sensor_msgs::msg::JointState::SharedPtr& out_write,
                      sensor_msgs::msg::JointState::SharedPtr& out_read) {
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = {pos, pos, pos, pos};
    bool ok = false;
    auto local_deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < local_deadline && !ok) {
      cmd_pub->publish(cmd);
      std::this_thread::sleep_for(100ms);
      executor_->spin_some();
      std::lock_guard<std::mutex> l(mtx);
      if (last_goal && last_write && last_read) {
        bool all_match = true;
        for (const auto& j : joints) {
          size_t i_g = idx_of(*last_goal, j);
          if (i_g == size_t(-1) || std::abs(last_goal->position[i_g] - pos) > 1e-6) {
            all_match = false;
            break;
          }
        }
        if (all_match) {
          out_goal = last_goal;
          out_write = last_write;
          out_read = last_read;
          ok = true;
        }
      }
    }
    return ok;
  };

  sensor_msgs::msg::JointState::SharedPtr g1, w1, r1, g2, w2, r2;
  ASSERT_TRUE(drive_to(0.0, g1, w1, r1)) << "Failed to capture publishers' state at command 0.0";
  // Allow the motor model some time to track the commanded position before the second snapshot
  // so that read_joint_states reflects the new commanded state.
  std::this_thread::sleep_for(2s);
  ASSERT_TRUE(drive_to(0.5, g2, w2, r2)) << "Failed to capture publishers' state at command 0.5";
  std::this_thread::sleep_for(2s);
  // Refresh the read snapshot once the motor has settled at the second command.
  {
    std::lock_guard<std::mutex> l(mtx);
    ASSERT_NE(last_read, nullptr);
    r2 = last_read;
  }

  // Sanity: all three messages were delivered and have parallel, identically-named entries.
  ASSERT_NE(r1, nullptr);
  ASSERT_NE(w1, nullptr);
  ASSERT_NE(g1, nullptr);
  EXPECT_EQ(r1->name.size(), r1->position.size());
  EXPECT_EQ(w1->name.size(), w1->position.size());
  EXPECT_EQ(r1->name, w1->name);
  EXPECT_EQ(r1->name, g1->name);

  // The flipper joints declare `current` (not `effort`) state/command interfaces in URDF.
  // sensor_msgs/JointState only has `effort`, so the publisher set falls back to HW_IF_CURRENT;
  // verify the effort field is finite at every snapshot to catch a regression of that fallback.
  const std::vector<std::string> flippers = {"flipper_fl_joint", "flipper_fr_joint", "flipper_bl_joint",
                                             "flipper_br_joint"};
  for (const auto* snap : {&w1, &w2, &r1, &r2, &g1, &g2}) {
    for (const auto& jname : flippers) {
      size_t i = idx_of(**snap, jname);
      ASSERT_NE(i, size_t(-1)) << jname << " missing from a published JointState";
      EXPECT_TRUE(std::isfinite((*snap)->effort[i]))
          << jname << ": effort should be populated from HW_IF_CURRENT, got NaN";
    }
  }

  // write_joint_states is actuator-space (post-transmission), goal_joint_states is joint-space
  // (pre-transmission). Their delta across two commanded joint positions is exactly
  // (goal_delta) * mechanical_reduction (the constant offset cancels).
  const std::map<std::string, double> reductions = {
      {"flipper_fl_joint", -2.0}, {"flipper_fr_joint", 2.0}, {"flipper_bl_joint", 2.0}, {"flipper_br_joint", -2.0}};
  for (const auto& [jname, ratio] : reductions) {
    size_t i_w = idx_of(*w1, jname);
    size_t i_g = idx_of(*g1, jname);
    size_t i_r = idx_of(*r1, jname);
    size_t i_r2 = idx_of(*r2, jname);
    ASSERT_NE(i_w, size_t(-1)) << jname << " missing from write_joint_states";
    ASSERT_NE(i_g, size_t(-1)) << jname << " missing from goal_joint_states";
    ASSERT_NE(i_r, size_t(-1)) << jname << " missing from read_joint_states";

    const double goal_delta = g2->position[i_g] - g1->position[i_g];
    const double write_delta = w2->position[i_w] - w1->position[i_w];

    // Exact: write is computed from goal via the transmission, no motor dynamics involved.
    EXPECT_NEAR(write_delta, goal_delta * ratio, 1e-6)
        << jname << ": write delta (actuator) should equal goal delta (joint) * reduction (" << ratio << ")";

    // Sanity for read_joint_states: position is finite (not NaN) and the sample is live
    // (changes between snapshots taken several seconds apart while the motor is moving).
    // We don't assert numerical agreement with write because mock-motor tracking dynamics
    // are the responsibility of test_hw_transmission.
    EXPECT_TRUE(std::isfinite(r1->position[i_r])) << jname << ": read position (snapshot 1) should be finite";
    EXPECT_TRUE(std::isfinite(r2->position[i_r2])) << jname << ": read position (snapshot 2) should be finite";
    EXPECT_NE(r1->position[i_r], r2->position[i_r2])
        << jname << ": read position should change between snapshots while the motor is being driven";
  }
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
