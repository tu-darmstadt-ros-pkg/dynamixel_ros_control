// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Hardware Interface Lifecycle Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, Lifecycle_TorqueServiceAvailableWhenActive)
{
  // Test that torque service is available and works when hardware interface is active

  auto torque_client = tester_node_->create_test_client<std_srvs::srv::SetBool>("/athena_arm_interface/set_torque");
  ASSERT_TRUE(torque_client->wait_for_service(*executor_, 5s)) << "Torque service should be available when active";

  // Toggle torque off and on
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = false;

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;

  auto resp =
      hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr) << "Torque service should respond";
  EXPECT_TRUE(resp->success) << "Torque disable should succeed";

  request->data = true;
  resp = hector_testing_utils::call_service<std_srvs::srv::SetBool>(torque_client->get(), request, *executor_, options);
  ASSERT_NE(resp, nullptr);
  EXPECT_TRUE(resp->success) << "Torque enable should succeed";
}

TEST_F(HardwareInterfaceTest, Lifecycle_CalibrationServiceAvailableWhenActive)
{
  // Test that calibration service is available when hardware interface is active

  using hector_transmission_interface_msgs::srv::AdjustTransmissionOffsets;

  auto calibration_client = tester_node_->create_test_client<AdjustTransmissionOffsets>(
      "/athena_flipper_interface/adjust_transmission_offsets");
  ASSERT_TRUE(calibration_client->wait_for_service(*executor_, 5s))
      << "Calibration service should be available when active";

  // Try a calibration request
  auto request = std::make_shared<AdjustTransmissionOffsets::Request>();
  request->external_joint_measurements.name = {"flipper_fl_joint"};
  request->external_joint_measurements.position = {0.0};

  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 5s;
  options.response_timeout = 5s;

  auto resp = hector_testing_utils::call_service<AdjustTransmissionOffsets>(calibration_client->get(), request,
                                                                            *executor_, options);
  ASSERT_NE(resp, nullptr) << "Calibration service should respond";
  // Note: success depends on implementation, but service should at least respond
}

TEST_F(HardwareInterfaceTest, Lifecycle_EStopTopicSubscribedWhenActive)
{
  // Test that e-stop topic is subscribed when hardware interface is active

  auto estop_pub = tester_node_->create_publisher<std_msgs::msg::Bool>("/soft_e_stop", 10);

  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && estop_pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }

  EXPECT_GT(estop_pub->get_subscription_count(), 0) << "E-stop topic should have a subscriber when HW interface active";
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
