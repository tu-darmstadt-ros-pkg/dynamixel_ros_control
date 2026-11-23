// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Communication Error / Hard E-Stop Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, CommunicationError_TemporaryErrorRecovery)
{
  // Test that the system can recover from temporary communication errors
  // This simulates brief motor disconnection or bus errors

  // 1. Activate controller and verify normal operation
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  ASSERT_TRUE(waitForSubscribers(pub, 5s));

  // Move to initial position
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // 2. Inject temporary communication errors on some motors
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  auto motor2 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_2_ID);
  ASSERT_NE(motor1, nullptr);
  ASSERT_NE(motor2, nullptr);

  motor1->setCommunicationFault(dynamixel_ros_control::CommunicationFault::DISCONNECTED);
  motor2->setCommunicationFault(dynamixel_ros_control::CommunicationFault::DISCONNECTED);

  // Brief glitch: stays below the consecutive-error threshold (25 cycles @ 50 Hz = 500 ms),
  // so the interface must ride it out without erroring.
  std::this_thread::sleep_for(200ms);

  // 3. Clear errors to simulate recovery
  motor1->setCommunicationFault(dynamixel_ros_control::CommunicationFault::NONE);
  motor2->setCommunicationFault(dynamixel_ros_control::CommunicationFault::NONE);

  // 4. Wait for recovery and then send command
  std::this_thread::sleep_for(500ms);

  // 5. Every joint reaches a new target, so the system recovered
  EXPECT_TRUE(armReaches(pub, 0.5)) << "System should recover from temporary communication errors";
}

TEST_F(HardwareInterfaceTest, PartialCommLoss_HoldsReachableMotorsAndStopsDeadOne)
{
  // When one motor stops answering, the cyclic Protocol 2.0 sync read yields nothing for ANY
  // motor, so a whole-group goal reset cannot secure the arm. The reachable motors would keep
  // driving toward their last goal. on_error therefore probes each motor individually: reachable
  // motors are held (torque restored after a torque-safe mode switch), the unreachable motor gets
  // a statusless torque-off.
  std::this_thread::sleep_for(300ms);  // let activation settle. All motors start torqued

  // Sanity: everything starts torqued.
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    ASSERT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 1) << "Motor " << static_cast<int>(id);
  }

  // One motor drops its status packets: it still receives writes but never answers, so every cyclic
  // read fails. After the consecutive-error threshold (25 @ 50 Hz) read() returns ERROR -> on_error.
  constexpr uint8_t DEAD_ID = ARM_JOINT_4_ID;
  auto dead = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(DEAD_ID);
  ASSERT_NE(dead, nullptr);
  dead->setCommunicationFault(dynamixel_ros_control::CommunicationFault::DROP_STATUS_PACKETS);

  // Wait past the error threshold plus the one-shot on_error handling.
  std::this_thread::sleep_for(1500ms);

  // Reachable motors must remain torqued. They were held at position, not left driving and not
  // de-energised. (The old code left them exactly as they were because the group reset aborted.)
  for (uint8_t id : ARM_MOTOR_IDS) {
    if (id == DEAD_ID) {
      continue;
    }
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 1)
        << "Reachable motor " << static_cast<int>(id) << " should be held (torqued) after the partial comm loss";
    // Held at its current position: goal position was reset to the present position.
    EXPECT_NEAR(motor->read4ByteSigned(motor->getAddress("goal_position")),
                motor->read4ByteSigned(motor->getAddress("present_position")), 5)
        << "Reachable motor " << static_cast<int>(id) << " was not held at its current position";
  }

  // The unresponsive motor received a best-effort torque-off (it still applies writes).
  EXPECT_EQ(dead->read1Byte(dead->getAddress("torque_enable")), 0)
      << "Unresponsive motor should have been sent a torque-off";

  dead->setCommunicationFault(dynamixel_ros_control::CommunicationFault::NONE);  // let teardown settle
}

TEST_F(HardwareInterfaceTest, CommunicationError_GlobalErrorBlocksOperation)
{
  // Test that global communication errors (all motors) block operation
  // This simulates hard E-Stop cutting power to all motors
  // Every wait is for observable state, so a loaded machine only makes the test slower. A command
  // is published again on every check, because the first one can precede the subscriber's discovery.
  const auto arm_motor = [](size_t i) {
    return dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
  };
  const auto goal_ticks = [&](size_t i) {
    return arm_motor(i)->read4ByteSigned(arm_motor(i)->getAddress("goal_position"));
  };
  const auto positions = [&]() {
    std::vector<double> result;
    for (size_t i = 0; i < 7; ++i) {
      result.push_back(arm_motor(i)->getCurrentPosition());
    }
    return result;
  };

  // 1. Activate controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);

  ASSERT_TRUE(waitForSubscribers(pub, 5s));

  // Move to initial position and wait until every motor took the new goal and came to rest there
  std::vector<int32_t> goals_before;
  for (size_t i = 0; i < 7; ++i) {
    goals_before.push_back(goal_ticks(i));
  }
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  ASSERT_TRUE(executor_->spin_until(
      [&] {
        pub->publish(cmd);
        for (size_t i = 0; i < 7; ++i) {
          const auto present = arm_motor(i)->read4ByteSigned(arm_motor(i)->getAddress("present_position"));
          if (goal_ticks(i) == goals_before[i] || std::abs(goal_ticks(i) - present) > 5 ||
              std::abs(arm_motor(i)->getCurrentVelocity()) > 1e-3) {
            return false;
          }
        }
        return true;
      },
      60s))
      << "The arm did not settle at its initial command";
  const std::vector<double> initial_positions = positions();

  // 2. Enable global communication error (simulates power cut)
  dynamixel_ros_control::MockDynamixelManager::instance().setGlobalCommunicationError(true);

  // 3. Try to send movement commands. They must have no effect until the component gave up on the bus
  cmd.data = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  ASSERT_TRUE(executor_->spin_until(
      [&] {
        pub->publish(cmd);
        return hardwareComponentState("athena_arm_interface") == "unconfigured";
      },
      60s))
      << "Sustained communication loss must take the component through the fault path";

  // Verify positions haven't changed (communication errors prevent read/write)
  for (size_t i = 0; i < 7; ++i) {
    EXPECT_NEAR(arm_motor(i)->getCurrentPosition(), initial_positions[i], 0.1)
        << "Motor " << (ARM_JOINT_1_ID + i) << " should not move during communication errors";
  }

  // 4. Clear errors (simulate power restored)
  dynamixel_ros_control::MockDynamixelManager::instance().setGlobalCommunicationError(false);

  // 5. Sustained comm loss errors out the hardware interface and deactivates its controllers
  // (matching real hardware, where a dead bus produces RX timeouts). Recovery therefore
  // requires re-activation — the same flow the spawner performs on the robot.
  ASSERT_TRUE(
      setHardwareComponentState("athena_arm_interface", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active"))
      << "Hardware reactivation should succeed after communication is restored";

  ASSERT_TRUE(switch_controllers({"arm_position_controller"}, {}))
      << "Controller reactivation should succeed after hardware recovery";
  ASSERT_TRUE(waitForControllerState("arm_position_controller", "active"));

  EXPECT_TRUE(armReaches(pub, 0.7)) << "System should recover after reactivation";
}

TEST_F(HardwareInterfaceTest, EdgeCase_ControllerActivationWithCommunicationErrors)
{
  // Test controller activation when communication errors prevent successful reads
  // Expected: Controller activation should fail gracefully

  // 1. Set communication errors on all arm motors BEFORE activating controller
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor)
      motor->setCommunicationFault(dynamixel_ros_control::CommunicationFault::DISCONNECTED);
  }
  // Also set error on gripper
  auto gripper_motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  if (gripper_motor)
    gripper_motor->setCommunicationFault(dynamixel_ros_control::CommunicationFault::DISCONNECTED);

  // Wait for errors to take effect in read cycle
  std::this_thread::sleep_for(500ms);

  // 2. Try to load and activate controller. This may fail
  auto load_req = std::make_shared<LoadController::Request>();
  load_req->name = "arm_position_controller";
  auto load_resp = callService<LoadController>(load_client_, load_req);

  // Loading should succeed (doesn't require communication)
  ASSERT_NE(load_resp, nullptr);
  EXPECT_TRUE(load_resp->ok) << "Controller loading should succeed";

  auto config_req = std::make_shared<ConfigureController::Request>();
  config_req->name = "arm_position_controller";
  auto config_resp = callService<ConfigureController>(config_client_, config_req);
  ASSERT_NE(config_resp, nullptr);
  EXPECT_TRUE(config_resp->ok) << "Controller configuration should succeed";

  // 3. No motor can be read, so there is no present position to hold and the activation must fail
  auto switch_req = switchRequest();
  switch_req->activate_controllers = {"arm_position_controller"};
  auto switch_resp = callService<SwitchController>(switch_client_, switch_req);
  ASSERT_NE(switch_resp, nullptr);
  EXPECT_FALSE(switch_resp->ok) << "A controller activated against motors that cannot be read";
  EXPECT_FALSE(waitForControllerState("arm_position_controller", "active", 1s));

  // 4. Clear errors for cleanup
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    if (motor)
      motor->setCommunicationFault(dynamixel_ros_control::CommunicationFault::NONE);
  }
  if (gripper_motor)
    gripper_motor->setCommunicationFault(dynamixel_ros_control::CommunicationFault::NONE);
}

TEST_F(HardwareInterfaceTest, RebootService_ResetsMotors)
{
  // Test the reboot service functionality
  // This is used to recover motors from error states

  // 1. Create reboot service client
  auto reboot_client = tester_node_->create_test_client<std_srvs::srv::Trigger>("/athena_arm_interface_node/reboot");
  ASSERT_TRUE(reboot_client->wait_for_service(*executor_, 5s)) << "Reboot service not available";

  // 2. Record initial motor states
  std::vector<double> initial_positions;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    initial_positions.push_back(motor->getCurrentPosition());
  }

  // 3. Call reboot service
  ASSERT_TRUE(setHardwareComponentState("athena_arm_interface", lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        "unconfigured"))
      << "the reboot service needs the component unconfigured";
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto resp = callService<std_srvs::srv::Trigger>(reboot_client, request, 10s);

  ASSERT_NE(resp, nullptr) << "Reboot service call failed";
  EXPECT_TRUE(resp->success) << "Reboot should succeed: " << resp->message;

  // 4. Verify motors still exist and are functional
  std::this_thread::sleep_for(500ms);

  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Motor " << (int) id << " should still exist after reboot";
  }

  // 5. Verify LED is correct (pink, the component is unconfigured)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    verifyLEDColor(id, COLOR_PINK_R, COLOR_PINK_G, COLOR_PINK_B);
  }
}

TEST_F(HardwareInterfaceTest, RebootService_OnlyRebootsFaultyMotors)
{
  // Test that reboot is only called for motors with hardware errors, not healthy ones

  // 1. Create reboot service client
  auto reboot_client = tester_node_->create_test_client<std_srvs::srv::Trigger>("/athena_arm_interface_node/reboot");
  ASSERT_TRUE(reboot_client->wait_for_service(*executor_, 5s)) << "Reboot service not available";

  // 2. Reset reboot counters for all motors
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    motor->resetRebootCount();
  }
  auto gripper_motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(GRIPPER_ID);
  gripper_motor->resetRebootCount();

  // 3. Set hardware error on only motor 1 and motor 3
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  auto motor3 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_3_ID);
  motor1->setHardwareError(dynamixel_ros_control::HardwareErrorBit::ERROR_OVERHEATING);
  motor3->setHardwareError(dynamixel_ros_control::HardwareErrorBit::ERROR_INPUT_VOLTAGE);

  // The reboot service requires the component that on_error left unconfigured
  ASSERT_TRUE(waitForComponentState("athena_arm_interface", "unconfigured"));

  // 4. Call reboot service
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto resp = callService<std_srvs::srv::Trigger>(reboot_client, request, 10s);

  ASSERT_NE(resp, nullptr) << "Reboot service call failed";
  EXPECT_TRUE(resp->success) << "Reboot should succeed: " << resp->message;

  std::this_thread::sleep_for(500ms);

  // 5. Verify ONLY faulty motors were rebooted
  EXPECT_EQ(motor1->getRebootCount(), 1) << "Motor 1 (with error) should have been rebooted once";
  EXPECT_EQ(motor3->getRebootCount(), 1) << "Motor 3 (with error) should have been rebooted once";

  // Other arm motors should NOT have been rebooted
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    if (id == ARM_JOINT_1_ID || id == ARM_JOINT_3_ID)
      continue;
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getRebootCount(), 0) << "Motor " << (int) id << " (healthy) should NOT have been rebooted";
  }

  // Gripper should not have been rebooted either
  EXPECT_EQ(gripper_motor->getRebootCount(), 0) << "Gripper (healthy) should NOT have been rebooted";

  // 6. Verify hardware errors were cleared
  EXPECT_EQ(motor1->getHardwareError(), 0) << "Motor 1 hardware error should be cleared after reboot";
  EXPECT_EQ(motor3->getHardwareError(), 0) << "Motor 3 hardware error should be cleared after reboot";
}

TEST_F(HardwareInterfaceTest, RebootService_RestoresTorqueOnAndBlueLED)
{
  // Test that reboot restores torque ON state after clearing hardware errors
  //
  // NOTE: When a hardware error is detected, read() returns ERROR which causes
  // the controller_manager to deactivate the HW interface. After reboot,
  // the HW interface remains in inactive state, so LEDs will be pink.

  // 1. Setup: torque is ON by default (torque_on_startup: true)
  auto reboot_client = tester_node_->create_test_client<std_srvs::srv::Trigger>("/athena_arm_interface_node/reboot");
  ASSERT_TRUE(reboot_client->wait_for_service(*executor_, 5s)) << "Reboot service not available";

  // 2. Verify torque is initially on
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    ASSERT_EQ(motor->read1Byte(torque_addr), 1) << "Motor " << (int) id << " torque should be ON initially";
  }

  // 3. Reset reboot counter and set hardware error on motor 1
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  motor1->resetRebootCount();
  motor1->setHardwareError(dynamixel_ros_control::HardwareErrorBit::ERROR_OVERHEATING);

  // The reboot service requires the component that on_error left unconfigured
  ASSERT_TRUE(waitForComponentState("athena_arm_interface", "unconfigured"));

  // 4. Call reboot service
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto resp = callService<std_srvs::srv::Trigger>(reboot_client, request, 10s);

  ASSERT_NE(resp, nullptr) << "Reboot service call failed";
  EXPECT_TRUE(resp->success) << "Reboot should succeed: " << resp->message;

  std::this_thread::sleep_for(500ms);

  // 5. Verify motor was rebooted and hardware error was cleared
  EXPECT_EQ(motor1->getRebootCount(), 1) << "Motor 1 should have been rebooted";
  EXPECT_EQ(motor1->getHardwareError(), 0) << "Motor 1 hardware error should be cleared after reboot";

  // 6. Verify torque is still ON (restored state) for all motors
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 1) << "Motor " << (int) id << " torque should be restored to ON";
  }

  // 7. LED will be pink because HW interface was deactivated after hardware error
  // (controller_manager deactivates HW when read() returns ERROR)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_PINK_R) << "Motor " << (int) id << " LED should be pink (R) - HW inactive";
    EXPECT_EQ(motor->getLedGreen(), COLOR_PINK_G) << "Motor " << (int) id << " LED should be pink (G) - HW inactive";
    EXPECT_EQ(motor->getLedBlue(), COLOR_PINK_B) << "Motor " << (int) id << " LED should be pink (B) - HW inactive";
  }
}

TEST_F(HardwareInterfaceTest, RebootService_RestoresTorqueOffAndGreenLED)
{
  // Test that reboot restores torque OFF state after clearing hardware errors
  //
  // NOTE: When a hardware error is detected, read() returns ERROR which causes
  // the controller_manager to deactivate the HW interface. After reboot,
  // the HW interface remains in inactive state, so LEDs will be pink.

  // 1. Setup clients
  auto reboot_client = tester_node_->create_test_client<std_srvs::srv::Trigger>("/athena_arm_interface_node/reboot");
  ASSERT_TRUE(reboot_client->wait_for_service(*executor_, 5s)) << "Reboot service not available";

  // 2. Disable torque first
  ASSERT_TRUE(limp()) << "Should be able to disable torque";

  std::this_thread::sleep_for(300ms);

  // 3. Verify torque is now off
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    ASSERT_EQ(motor->read1Byte(torque_addr), 0) << "Motor " << (int) id << " torque should be OFF";
  }

  // 4. Set hardware error on motor 1 and call reboot
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  motor1->resetRebootCount();
  motor1->setHardwareError(dynamixel_ros_control::HardwareErrorBit::ERROR_OVERLOAD);

  // The reboot service requires the component that on_error left unconfigured
  ASSERT_TRUE(waitForComponentState("athena_arm_interface", "unconfigured"));

  auto reboot_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto resp = callService<std_srvs::srv::Trigger>(reboot_client, reboot_request, 10s);

  ASSERT_NE(resp, nullptr) << "Reboot service call failed";
  EXPECT_TRUE(resp->success) << "Reboot should succeed: " << resp->message;

  std::this_thread::sleep_for(500ms);

  // 5. Verify motor was rebooted and hardware error was cleared
  EXPECT_EQ(motor1->getRebootCount(), 1) << "Motor 1 should have been rebooted";
  EXPECT_EQ(motor1->getHardwareError(), 0) << "Motor 1 hardware error should be cleared after reboot";

  // 6. Verify torque is still OFF (restored to user's desired state)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    uint16_t torque_addr = motor->getAddress("torque_enable");
    EXPECT_EQ(motor->read1Byte(torque_addr), 0) << "Motor " << (int) id << " torque should remain OFF after reboot";
  }

  // 7. LED will be pink because HW interface was deactivated after hardware error
  // (controller_manager deactivates HW when read() returns ERROR)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getLedRed(), COLOR_PINK_R) << "Motor " << (int) id << " LED should be pink (R) - HW inactive";
    EXPECT_EQ(motor->getLedGreen(), COLOR_PINK_G) << "Motor " << (int) id << " LED should be pink (G) - HW inactive";
    EXPECT_EQ(motor->getLedBlue(), COLOR_PINK_B) << "Motor " << (int) id << " LED should be pink (B) - HW inactive";
  }
}

TEST_F(HardwareInterfaceTest, RebootService_NoRebootWhenNoErrors)
{
  // Test that when no motors have hardware errors, no reboots occur but LEDs are still updated

  // 1. Create reboot service client
  auto reboot_client = tester_node_->create_test_client<std_srvs::srv::Trigger>("/athena_arm_interface_node/reboot");
  ASSERT_TRUE(reboot_client->wait_for_service(*executor_, 5s)) << "Reboot service not available";

  // 2. Reset reboot counters for all motors (no hardware errors set)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    motor->resetRebootCount();
    ASSERT_EQ(motor->getHardwareError(), 0) << "Motor " << (int) id << " should have no hardware error";
  }

  // 3. Call reboot service
  ASSERT_TRUE(setHardwareComponentState("athena_arm_interface", lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
                                        "unconfigured"))
      << "the reboot service needs the component unconfigured";
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto resp = callService<std_srvs::srv::Trigger>(reboot_client, request, 10s);

  ASSERT_NE(resp, nullptr) << "Reboot service call failed";
  EXPECT_TRUE(resp->success) << "Reboot should succeed (no errors to fix): " << resp->message;

  std::this_thread::sleep_for(500ms);

  // 4. Verify NO motors were rebooted
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    EXPECT_EQ(motor->getRebootCount(), 0) << "Motor " << (int) id << " should NOT have been rebooted (no error)";
  }

  // 5. Verify LED is still correct (pink, the component is unconfigured)
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    verifyLEDColor(id, COLOR_PINK_R, COLOR_PINK_G, COLOR_PINK_B);
  }
}

// ============================================================================
// Error Threshold / Robustness Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, Robustness_ToleratesOccasionalReadErrors)
{
  // Test that the system tolerates occasional read errors without triggering ERROR state
  // The error threshold is 25 consecutive errors before returning ERROR

  // 1. Activate controller and verify normal operation
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(pub, 5s));

  // Move to initial position
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // 2. Inject brief communication errors (less than threshold)
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor1, nullptr);

  // Cause a few errors then recover (well below the 25 error threshold)
  motor1->setCommunicationFault(dynamixel_ros_control::CommunicationFault::DISCONNECTED);
  std::this_thread::sleep_for(200ms);  // ~4 read cycles at 50Hz
  motor1->setCommunicationFault(dynamixel_ros_control::CommunicationFault::NONE);

  // 3. Wait for recovery
  std::this_thread::sleep_for(500ms);

  // 4. Verify controller is still active (system tolerated the brief errors)
  ASSERT_TRUE(waitForControllerState("arm_position_controller", "active", 5s))
      << "Controller should remain active after brief communication errors";

  // 5. Verify commands still work
  EXPECT_TRUE(armReaches(pub, 0.6)) << "System should continue operating after tolerating brief read errors";
}

TEST_F(HardwareInterfaceTest, Robustness_IntermittentErrorsWithRecovery)
{
  // Test that the system handles intermittent errors (error, recover, error, recover)
  // without accumulating errors across recovery periods

  // 1. Activate controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(pub, 5s));

  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor1, nullptr);

  // 2. Cause multiple intermittent error periods
  for (int i = 0; i < 5; ++i) {
    // Brief error
    motor1->setCommunicationFault(dynamixel_ros_control::CommunicationFault::DISCONNECTED);
    std::this_thread::sleep_for(100ms);  // ~2 read cycles

    // Recovery period, where errors should reset
    motor1->setCommunicationFault(dynamixel_ros_control::CommunicationFault::NONE);
    std::this_thread::sleep_for(200ms);  // Allow successful reads to reset error counter
  }

  // 3. Verify controller is still active
  ASSERT_TRUE(waitForControllerState("arm_position_controller", "active", 5s))
      << "Controller should remain active after intermittent errors with recovery";

  // 4. Verify system is still functional
  EXPECT_TRUE(armReaches(pub, 0.4)) << "System should function after intermittent errors with recovery periods";
}

TEST_F(HardwareInterfaceTest, Robustness_ControllerActivationWithoutPriorRead)
{
  // Test that controller activation works even when no read() has been called yet
  // The perform_command_mode_switch should perform an inline read

  // Note: The test fixture automatically activates hardware, but we can test
  // by ensuring controller activation succeeds on a fresh start

  // 1. Verify no controllers are active initially
  auto list_resp = list_controllers();
  ASSERT_NE(list_resp, nullptr);

  bool any_active = false;
  for (const auto& ctrl : list_resp->controller) {
    if (ctrl.state == "active") {
      any_active = true;
      break;
    }
  }

  // 2. If no controllers are active, activate one. This tests inline read
  if (!any_active) {
    ASSERT_TRUE(loadAndActivateController("arm_position_controller"))
        << "Controller activation should succeed with inline read in perform_command_mode_switch";
  }

  // 3. Verify controller is now active and functional
  ASSERT_TRUE(waitForControllerState("arm_position_controller", "active", 5s));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(pub, 5s));

  // 4. Send command and verify it works
  EXPECT_TRUE(armReaches(pub, 0.3)) << "Commands should work after controller activation with inline read";
}

TEST_F(HardwareInterfaceTest, Robustness_ControllerSwitchAfterCommunicationRecovery)
{
  // Test controller switching after communication errors have been resolved
  // This verifies the inline read allows mode switches after error recovery

  // 1. Activate first controller
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pos_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(pos_pub, 5s));

  // 2. Cause a brief communication glitch (below the 25-cycle error threshold, so the
  // interface and controllers stay active)
  dynamixel_ros_control::MockDynamixelManager::instance().setGlobalCommunicationError(true);
  std::this_thread::sleep_for(200ms);
  dynamixel_ros_control::MockDynamixelManager::instance().setGlobalCommunicationError(false);
  std::this_thread::sleep_for(500ms);

  // 3. Load velocity controller
  auto load_req = std::make_shared<LoadController::Request>();
  load_req->name = "arm_velocity_controller";
  auto load_resp = callService<LoadController>(load_client_, load_req);
  ASSERT_NE(load_resp, nullptr);
  ASSERT_TRUE(load_resp->ok);

  auto config_req = std::make_shared<ConfigureController::Request>();
  config_req->name = "arm_velocity_controller";
  callService<ConfigureController>(config_client_, config_req);

  // 4. Switch controllers. This should succeed with inline read
  ASSERT_TRUE(switch_controllers({"arm_velocity_controller"}, {"arm_position_controller"}))
      << "Controller switch should succeed after communication recovery (inline read)";

  // 5. Verify velocity controller is active
  ASSERT_TRUE(waitForControllerState("arm_velocity_controller", "active", 5s));
  ASSERT_TRUE(waitForControllerState("arm_position_controller", "inactive", 5s));

  // 6. Verify velocity commands work
  auto vel_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_velocity_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(vel_pub, 5s));

  std::vector<double> positions_before;
  for (uint8_t id = ARM_JOINT_1_ID; id <= ARM_JOINT_7_ID; ++id) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_before.push_back(motor->getCurrentPosition());
  }

  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  vel_pub->publish(cmd);
  const auto every_joint_moved = [&] {
    for (size_t i = 0; i < 7; ++i) {
      auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID + i);
      if (motor->getCurrentPosition() - positions_before[i] <= 0.1) {
        return false;
      }
    }
    return true;
  };
  EXPECT_TRUE(executor_->spin_until(every_joint_moved, scaled(10s)))
      << "Not every joint followed the velocity controller after the communication recovery";
}

}  // namespace dynamixel_ros_control::test
