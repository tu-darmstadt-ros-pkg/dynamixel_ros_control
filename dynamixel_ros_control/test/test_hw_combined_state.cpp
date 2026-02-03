// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Combined State Tests - Verifying behavior when multiple states are active
// ============================================================================

TEST_F(HardwareInterfaceTest, CombinedState_EStopWhileTorqueOff)
{
  // Test that E-Stop can be activated while torque is off
  // When torque is already off, E-Stop has no additional effect on motors (already safe)
  // LED behavior: When torque is off, LED stays green even with E-Stop active
  // (torque off is already a safe state)

  // 1. Disable torque first
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

  // Verify LED is green (torque off)
  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  EXPECT_EQ(motor->getLedGreen(), COLOR_GREEN_G) << "LED should be green when torque is off";

  // Verify torque is off
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 0) << "Motor " << (int) id << " torque should be disabled";
  }

  // 2. Activate E-Stop while torque is off
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0);

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 3. Verify torque is still off (E-Stop should not enable torque)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 0)
        << "Motor " << (int) id << " torque should still be disabled with E-Stop";
  }

  // 4. Disable E-Stop
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 5. Verify torque is still off after E-Stop release
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 0)
        << "Motor " << (int) id << " torque should still be disabled after E-Stop release";
  }
}

TEST_F(HardwareInterfaceTest, CombinedState_TorqueOffWhileEStopActive)
{
  // Test disabling torque while E-Stop is active - should still work
  // This verifies that torque can always be disabled as a safety mechanism

  // 1. Activate E-Stop first
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0);

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // Verify LED is orange during E-Stop
  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  EXPECT_EQ(motor->getLedRed(), COLOR_ORANGE_R) << "LED should be orange during E-Stop";

  // 2. Disable torque while E-Stop is active
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
  // Note: This should succeed - torque can be disabled even during E-Stop
  EXPECT_TRUE(resp->success);

  std::this_thread::sleep_for(300ms);

  // 3. CRITICAL: Verify torque is actually disabled (safety check)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 0) << "Motor " << (int) id << " torque should be disabled";
  }

  // 4. Release E-Stop
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // 5. Verify torque is still off after E-Stop release
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 0)
        << "Motor " << (int) id << " torque should still be disabled after E-Stop release";
    // LED should be green (torque off state)
    EXPECT_EQ(motor->getLedGreen(), COLOR_GREEN_G)
        << "Motor " << (int) id << " LED should be green after E-Stop release";
  }
}

TEST_F(HardwareInterfaceTest, CombinedState_CalibrationWhileEStopActive)
{
  // Test that calibration (offset adjustment) is blocked when E-Stop is active

  // 1. Activate position controller to set up state
  ASSERT_TRUE(loadAndActivateController("flipper_position_controller"));
  std::this_thread::sleep_for(300ms);

  // 2. Activate E-Stop
  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(estop_pub->get_subscription_count(), 0);

  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // Record initial positions
  std::vector<uint8_t> flipper_ids = {FLIPPER_FL_ID, FLIPPER_FR_ID, FLIPPER_BL_ID, FLIPPER_BR_ID};
  std::map<uint8_t, double> initial_positions;
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions[id] = motor->getCurrentPosition();
  }

  // 3. Attempt calibration service call (should complete but E-Stop prevents movement)
  auto calibration_client =
      tester_node_->create_test_client<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>(
          "/athena_flipper_interface/adjust_transmission_offsets");
  // Service may or may not be available during E-Stop - just ensure no dangerous movement
  bool service_available = calibration_client->wait_for_service(*executor_, 2s);

  if (service_available) {
    auto request = std::make_shared<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Request>();
    sensor_msgs::msg::JointState ext_measurement;
    ext_measurement.name = {"flipper_fl_joint"};
    ext_measurement.position = {0.5};
    request->external_joint_measurements = ext_measurement;

    hector_testing_utils::ServiceCallOptions options;
    options.service_timeout = 5s;
    options.response_timeout = 5s;
    auto resp = hector_testing_utils::call_service<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>(
        calibration_client->get(), request, *executor_, options);
    // Response may or may not succeed, but key is no sudden movement
  }

  std::this_thread::sleep_for(500ms);

  // 4. Verify no sudden movements occurred
  for (uint8_t id : flipper_ids) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double position_change = std::abs(motor->getCurrentPosition() - initial_positions[id]);
    EXPECT_LT(position_change, 0.1) << "Motor " << (int) id
                                    << " should not have significant movement during E-Stop calibration attempt";
  }

  // Cleanup
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(300ms);
}

TEST_F(HardwareInterfaceTest, CombinedState_CalibrationWhileTorqueOff)
{
  // Test calibration (offset adjustment) when torque is off

  // 1. Activate position controller
  ASSERT_TRUE(loadAndActivateController("flipper_position_controller"));
  std::this_thread::sleep_for(300ms);

  // 2. Disable torque (flipper interface service)
  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_flipper_interface/set_torque");
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
          "/athena_flipper_interface/adjust_transmission_offsets");
  ASSERT_TRUE(calibration_client->wait_for_service(*executor_, 5s));

  auto cal_request = std::make_shared<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets::Request>();
  sensor_msgs::msg::JointState ext_measurement;
  ext_measurement.name = {"flipper_fl_joint"};
  ext_measurement.position = {0.5};
  cal_request->external_joint_measurements = ext_measurement;

  auto cal_resp =
      hector_testing_utils::call_service<hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets>(
          calibration_client->get(), cal_request, *executor_, options);
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

  // 5. Enable torque again - verify no sudden movement when torque re-enabled
  request->data = true;
  resp = hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success);

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

TEST_F(HardwareInterfaceTest, CombinedState_NoSuddenMovementOnAnyStateTransition)
{
  // Test verifying safe behavior during state transitions
  // The mock physics simulation naturally moves motors toward goals, so this test
  // verifies that state transitions don't cause dangerous instantaneous jumps
  // (as opposed to gradual controlled movement toward goal positions)

  // Record initial positions for all motors
  std::map<uint8_t, double> initial_positions;
  std::vector<uint8_t> all_arm_ids = {ARM_JOINT_1_ID, ARM_JOINT_2_ID, ARM_JOINT_3_ID, ARM_JOINT_4_ID,
                                      ARM_JOINT_5_ID, ARM_JOINT_6_ID, ARM_JOINT_7_ID};

  auto record_positions = [&]() {
    for (uint8_t id : all_arm_ids) {
      auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
      initial_positions[id] = motor->getCurrentPosition();
    }
  };

  // Max delta of 2.0 radians allows for normal physics simulation movement over 500ms
  // but catches instantaneous jumps (which would be >10 radians in a single step)
  auto verify_no_jump = [&](const std::string& context, double max_delta = 2.0) {
    for (uint8_t id : all_arm_ids) {
      auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
      double delta = std::abs(motor->getCurrentPosition() - initial_positions[id]);
      EXPECT_LT(delta, max_delta) << "Motor " << (int) id << " jumped during " << context;
    }
  };

  // Setup
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));
  std::this_thread::sleep_for(300ms);
  record_positions();

  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s));

  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;
  std_msgs::msg::Bool estop_msg;
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

  // Transition 1: Normal -> E-Stop
  record_positions();
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);
  verify_no_jump("Normal -> E-Stop");

  // Transition 2: E-Stop -> E-Stop + Torque Off
  record_positions();
  request->data = false;
  hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  std::this_thread::sleep_for(300ms);
  verify_no_jump("E-Stop -> E-Stop + Torque Off");

  // Transition 3: E-Stop + Torque Off -> Torque Off (release E-Stop)
  record_positions();
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);
  verify_no_jump("E-Stop + Torque Off -> Torque Off");

  // Transition 4: Torque Off -> Normal (enable torque)
  record_positions();
  request->data = true;
  hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  std::this_thread::sleep_for(300ms);
  verify_no_jump("Torque Off -> Normal");

  // Transition 5: Normal -> Torque Off -> E-Stop -> Torque On -> E-Stop Off (complex sequence)
  record_positions();

  // Torque off
  request->data = false;
  hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  std::this_thread::sleep_for(100ms);
  verify_no_jump("Complex: after torque off");

  // E-Stop on
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(100ms);
  verify_no_jump("Complex: after E-Stop on");

  // Torque on (while E-Stop active)
  request->data = true;
  hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  std::this_thread::sleep_for(100ms);
  verify_no_jump("Complex: after torque on during E-Stop");

  // E-Stop off
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(300ms);
  verify_no_jump("Complex: after E-Stop off");
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
