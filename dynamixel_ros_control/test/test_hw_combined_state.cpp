// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Combined State Tests: behavior when multiple states are active
// ============================================================================

TEST_F(HardwareInterfaceTest, CombinedState_CalibrationWhileTorqueOff)
{
  // Test calibration (offset adjustment) when torque is off

  // 1. Activate position controller
  ASSERT_TRUE(loadAndActivateController("flipper_position_controller"));
  std::this_thread::sleep_for(300ms);

  // 2. Disable torque on the flippers
  ASSERT_TRUE(limp("flipper_torque_off_controller"));

  std::this_thread::sleep_for(300ms);

  // Record initial positions
  std::vector<uint8_t> flipper_ids = {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID};
  std::map<uint8_t, double> initial_positions;
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions[id] = motor->getCurrentPosition();
  }

  // 3. Attempt calibration service call
  auto calibration_client =
      tester_node_->create_test_client<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>(
          "/athena_flipper_interface_node/adjust_transmission_offsets");
  ASSERT_TRUE(calibration_client->wait_for_service(*executor_, 5s));

  auto cal_request = std::make_shared<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Request>();
  sensor_msgs::msg::JointState ext_measurement;
  ext_measurement.name = {"flipper_fl_joint"};
  ext_measurement.position = {0.5};
  cal_request->external_joint_measurements = ext_measurement;

  auto cal_resp =
      callService<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>(calibration_client, cal_request);
  // Calibration should succeed even with torque off (offset adjustment is software-only)
  ASSERT_NE(cal_resp, nullptr);
  EXPECT_TRUE(cal_resp->success) << "Calibration should succeed when torque is off: " << cal_resp->message;

  std::this_thread::sleep_for(500ms);

  // 4. Verify no sudden movements (torque is off, so motors shouldn't move)
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double position_change = std::abs(motor->getCurrentPosition() - initial_positions[id]);
    EXPECT_LT(position_change, 0.05) << "Motor " << (int) id
                                     << " should not move when torque is off during calibration";
  }

  // 5. Enable torque again and verify no sudden movement when torque re-enabled
  EXPECT_TRUE(releaseLimp("flipper_torque_off_controller"));

  // Give time for any potential movement
  std::this_thread::sleep_for(500ms);

  // Verify positions didn't jump
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    // Position change should be minimal (just due to physics simulation settling)
    double position_change = std::abs(motor->getCurrentPosition() - initial_positions[id]);
    EXPECT_LT(position_change, 0.2) << "Motor " << (int) id << " should not have jumped after torque re-enable";
  }
}

}  // namespace dynamixel_ros_control::test
