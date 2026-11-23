// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "dynamixel_ros_control/common.hpp"
#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Hardware Error Handling Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, HardwareError_LEDTurnsRedOnError)
{
  // on_error leaves the component unconfigured: the faulted motor red, the others pink

  // 1. Verify initial LED state (blue, active with torque on)
  std::this_thread::sleep_for(500ms);
  for (uint8_t id : ARM_MOTOR_IDS) {
    verifyLEDColor(id, COLOR_BLUE_R, COLOR_BLUE_G, COLOR_BLUE_B);
  }

  // 2. Inject hardware error on one motor
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor1, nullptr);
  motor1->setHardwareError(dynamixel_ros_control::ERROR_OVERLOAD);

  // 3. on_error leaves the component unconfigured
  ASSERT_TRUE(waitForComponentState("athena_arm_interface", "unconfigured"));

  // 4. Verify motor with error has red LED
  verifyLEDColor(ARM_JOINT_1_ID, COLOR_RED_R, COLOR_RED_G, COLOR_RED_B);

  // 5. Other motors show the unconfigured component
  for (uint8_t id = ARM_JOINT_2_ID; id <= ARM_JOINT_7_ID; ++id) {
    verifyLEDColor(id, COLOR_PINK_R, COLOR_PINK_G, COLOR_PINK_B);
  }

  // Cleanup by clearing the error
  motor1->clearHardwareError();
}

TEST_F(HardwareInterfaceTest, HardwareError_JointsSecuredOnError)
{
  // A hardware error secures the component, so no actuator moves on

  // 1. Activate position controller and send a command
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(pub, 5s));

  // Move to a known position
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // Record positions before error
  std::map<uint8_t, double> positions_before;
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    positions_before[id] = motor->getCurrentPosition();
  }

  // 2. Inject hardware error on one motor
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor1, nullptr);
  motor1->setHardwareError(dynamixel_ros_control::ERROR_OVERLOAD);

  // 3. Wait for error to be detected
  ASSERT_TRUE(waitForComponentState("athena_arm_interface", "unconfigured"));

  // 4. Try to send a new command. The motors must not move
  cmd.data = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  pub->publish(cmd);
  std::this_thread::sleep_for(1s);

  // 5. Verify motors did NOT move significantly
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    double position_change = std::abs(motor->getCurrentPosition() - positions_before[id]);
    EXPECT_LT(position_change, 0.1) << "Motor " << (int) id << " should not move after a hardware error";
  }

  // Cleanup
  motor1->clearHardwareError();
}

TEST_F(HardwareInterfaceTest, HardwareError_RebootServiceClearsError)
{
  // Test that calling the reboot service clears the hardware error
  //
  // NOTE: After hardware error, the HW interface is deactivated by controller_manager.
  // The reboot service still works but the HW interface remains in inactive state.
  // LEDs will be pink (inactive) after reboot.

  // 1. Inject hardware error
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor1, nullptr);
  motor1->setHardwareError(dynamixel_ros_control::ERROR_OVERLOAD);

  // 2. The reboot service requires the component that on_error left unconfigured
  ASSERT_TRUE(waitForComponentState("athena_arm_interface", "unconfigured"));

  // 3. Call reboot service

  auto resp = rebootMotors();

  ASSERT_NE(resp, nullptr) << "Reboot service call failed";
  EXPECT_TRUE(resp->success) << "Reboot should succeed: " << resp->message;

  // 4. Verify motor was rebooted
  EXPECT_GT(motor1->getRebootCount(), 0) << "Motor should have been rebooted";

  // 5. Wait for LED update
  std::this_thread::sleep_for(500ms);

  // 6. Verify hardware error is cleared
  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  EXPECT_EQ(motor->getHardwareError(), 0) << "Hardware error should be cleared after reboot";

  // LED will be pink because HW interface was deactivated by controller_manager
  // after the initial hardware error was detected
  verifyLEDColor(ARM_JOINT_1_ID, COLOR_PINK_R, COLOR_PINK_G, COLOR_PINK_B);
}

// A hardware error leaves the component UNCONFIGURED, where neither read() nor write() runs. The
// reboot service restores each joint's torque from there, and a watchdog armed on that path has
// nothing feeding it: it trips within its window and latches, which shuts the goal registers of
// every capable motor until the next configure clears them, so a later securing pass cannot write the
// hold goals it needs to keep the arm up.
TEST_F(HardwareInterfaceTest, HardwareError_RebootDoesNotArmAWatchdogNothingFeeds)
{
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor1, nullptr);
  ASSERT_EQ(motor1->getBusWatchdog(), 4) << "Test premise: the watchdog should be armed while active";

  motor1->setHardwareError(dynamixel_ros_control::ERROR_OVERLOAD);
  ASSERT_TRUE(waitForComponentState("athena_arm_interface", "unconfigured"));

  auto resp = rebootMotors();
  ASSERT_NE(resp, nullptr) << "Reboot service call failed";
  EXPECT_TRUE(resp->success) << "Reboot should succeed: " << resp->message;
  std::this_thread::sleep_for(500ms);

  // Torque was restored on every joint that wanted it, so this is the state the arm is left in
  // until something reconfigures it.
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(motor->getBusWatchdog(), 0)
        << "Motor " << static_cast<int>(id)
        << " was left with an armed watchdog after a reboot that ran while the component was "
           "unconfigured. Nothing feeds it there, so it trips and latches the goal registers shut.";
  }
}

// A reboot must not report success while the motor still reports the fault it was rebooted for. A
// fault whose cause outlives the reboot (a motor still too hot) is the case where restoring torque
// would re-energise a joint that is not fit to hold. The mock's sticky error models that cause, and
// until it survived the reboot's own RAM reset this path was unreachable: the interface read the
// register the reset had zeroed and called the motor healthy.
TEST_F(HardwareInterfaceTest, HardwareError_RebootFailsWhileTheFaultPersists)
{
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor1, nullptr);

  motor1->setHardwareErrorSticky(true);
  motor1->setHardwareError(dynamixel_ros_control::ERROR_OVERHEATING);
  ASSERT_TRUE(waitForComponentState("athena_arm_interface", "unconfigured"));

  auto resp = rebootMotors();
  ASSERT_NE(resp, nullptr) << "Reboot service call failed";
  EXPECT_FALSE(resp->success)
      << "The reboot reported success although the motor still reports its fault, so torque was "
         "restored on a joint that is not fit to hold it.";
  EXPECT_NE(motor1->getHardwareError(), 0) << "Test premise: the sticky fault must outlive the reboot";
  EXPECT_EQ(motor1->read1Byte(motor1->getAddress("hardware_error_status")), dynamixel_ros_control::ERROR_OVERHEATING)
      << "The fault survived only in the mock's own flag, not in the register the interface reads.";

  motor1->setHardwareErrorSticky(false);
  motor1->setHardwareError(0);
}

TEST_F(HardwareInterfaceTest, HardwareError_MultipleMotorsWithErrors)
{
  // Test that multiple motors with hardware errors all get red LEDs
  // while the others show the unconfigured component

  // 1. Inject hardware errors on multiple motors
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  auto motor3 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_3_ID);
  auto motor5 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_5_ID);
  ASSERT_NE(motor1, nullptr);
  ASSERT_NE(motor3, nullptr);
  ASSERT_NE(motor5, nullptr);

  motor1->setHardwareError(dynamixel_ros_control::ERROR_OVERLOAD);
  motor3->setHardwareError(dynamixel_ros_control::ERROR_OVERHEATING);
  motor5->setHardwareError(dynamixel_ros_control::ERROR_INPUT_VOLTAGE);

  // 2. on_error leaves the component unconfigured
  ASSERT_TRUE(waitForComponentState("athena_arm_interface", "unconfigured"));

  // 3. Verify motors with errors have red LEDs
  verifyLEDColor(ARM_JOINT_1_ID, COLOR_RED_R, COLOR_RED_G, COLOR_RED_B);
  verifyLEDColor(ARM_JOINT_3_ID, COLOR_RED_R, COLOR_RED_G, COLOR_RED_B);
  verifyLEDColor(ARM_JOINT_5_ID, COLOR_RED_R, COLOR_RED_G, COLOR_RED_B);

  // 4. Motors without errors are pink
  verifyLEDColor(ARM_JOINT_2_ID, COLOR_PINK_R, COLOR_PINK_G, COLOR_PINK_B);
  verifyLEDColor(ARM_JOINT_4_ID, COLOR_PINK_R, COLOR_PINK_G, COLOR_PINK_B);
  verifyLEDColor(ARM_JOINT_6_ID, COLOR_PINK_R, COLOR_PINK_G, COLOR_PINK_B);
  verifyLEDColor(ARM_JOINT_7_ID, COLOR_PINK_R, COLOR_PINK_G, COLOR_PINK_B);

  // Cleanup
  motor1->clearHardwareError();
  motor3->clearHardwareError();
  motor5->clearHardwareError();
}

// on_error returns SUCCESS, which leaves the component unconfigured and recoverable. FAILURE would
// finalize it, and nothing short of a restart brings it back.
TEST_F(HardwareInterfaceTest, HardwareError_LeavesTheComponentRecoverable)
{
  auto motor1 = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor1, nullptr);
  motor1->setHardwareError(dynamixel_ros_control::ERROR_OVERLOAD);
  EXPECT_TRUE(waitForComponentState("athena_arm_interface", "unconfigured"))
      << "the component reached '" << hardwareComponentState("athena_arm_interface") << "' after a hardware error";
  EXPECT_EQ(hardwareComponentState("athena_flipper_interface"), "active");
  motor1->clearHardwareError();
}

// ============================================================================
// A motor that was already errored before this process started
// ============================================================================
//
// Protocol 2.0 sets the alert bit in the status packet of every instruction while a hardware error
// is latched. A motor that overloaded and then outlived the process that saw it answers the next
// process's very first read with that bit set. Nothing about the transaction failed, so a
// configuration that treats the alert as a failed transaction refuses to come up against a motor
// that is talking normally, and the reboot service is the only way out.
// A reboot cuts the motor's torque, so the reboot service must not decide on one dropped status
// packet. The component is unconfigured first, as the service requires, so the counted fault below
// is spent by the service's own reads. The arm stays torqued (torque_off_on_shutdown is false).
class RebootSelectionTest : public HardwareInterfaceTest
{
protected:
  void stopUpdateLoop()
  {
    cm_running_ = false;
    if (cm_update_thread_.joinable()) {
      cm_update_thread_.join();
    }
  }

  void unconfigureArm()
  {
    ASSERT_TRUE(setHardwareComponentState("athena_arm_interface",
                                          lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured"));
  }
};

// A reboot under a running read and write would cut torque and wipe RAM beneath them
TEST_F(RebootSelectionTest, ARebootWhileActiveIsRefused)
{
  std::this_thread::sleep_for(500ms);
  stopUpdateLoop();
  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_3_ID);
  ASSERT_NE(motor, nullptr);
  motor->setHardwareError(ERROR_OVERHEATING);

  auto response = rebootMotors("athena_arm_interface");
  ASSERT_NE(response, nullptr) << "The reboot service did not answer";
  EXPECT_FALSE(response->success);
  EXPECT_EQ(motor->getRebootCount(), 0) << "A motor was rebooted under an active component";
  motor->setHardwareError(0);
}

TEST_F(RebootSelectionTest, OneDroppedStatusPacketDoesNotRebootAHealthyMotor)
{
  std::this_thread::sleep_for(500ms);
  unconfigureArm();

  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_3_ID);
  ASSERT_NE(motor, nullptr);
  ASSERT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 1) << "Test premise: the arm is energised";
  // Two uses: the group read and the per-motor probe that follows it. The retry gets an answer.
  motor->setRegisterReadFaultCount("hardware_error_status", 2);

  auto response = rebootMotors("athena_arm_interface");
  ASSERT_NE(response, nullptr) << "The reboot service did not answer";

  EXPECT_EQ(motor->getRebootCount(), 0) << "A healthy motor was rebooted because one status packet was lost.";
  EXPECT_EQ(motor->read1Byte(motor->getAddress("torque_enable")), 1)
      << "The motor lost its torque to a reboot it did not need.";
}

TEST_F(RebootSelectionTest, AMotorThatStaysSilentIsStillRebooted)
{
  std::this_thread::sleep_for(500ms);
  unconfigureArm();

  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_3_ID);
  ASSERT_NE(motor, nullptr);
  motor->setRegisterReadFault("hardware_error_status");

  auto response = rebootMotors("athena_arm_interface");
  ASSERT_NE(response, nullptr) << "The reboot service did not answer";

  EXPECT_EQ(motor->getRebootCount(), 1)
      << "A motor that never answered was not rebooted. A motor that cannot be asked is not known to be healthy.";
}

class PreExistingHardwareErrorTest : public HardwareInterfaceTest
{
protected:
  void SetUp() override
  {
    // The error is latched from the first read on, so the interface reports it and the manager
    // deactivates the component. Coming up is what this fixture is about, not staying up.
    require_hardware_active_ = false;
    HardwareInterfaceTest::SetUp();
  }

  void beforeControllerManager() override
  {
    // Created ahead of the fixture's motors, which leave an existing id alone, so the interface meets a
    // motor that is already flagging a fault rather than one that develops it later.
    auto& manager = dynamixel_ros_control::MockDynamixelManager::instance();
    manager.addMotor(ARM_JOINT_1_ID, DEFAULT_MOCK_MODEL_NUMBER);
    auto motor = manager.getMotor(ARM_JOINT_1_ID);
    ASSERT_NE(motor, nullptr);
    motor->setHardwareError(dynamixel_ros_control::ERROR_OVERLOAD);
  }
};

// The interface has to come up far enough to offer its recovery service, and the operator has to
// be able to get from there back to a working arm. Refusing the configuration instead leaves the
// reboot service unadvertised, and a power cycle of the motors as the only remedy.
TEST_F(PreExistingHardwareErrorTest, RebootAndActivateRecoverAnArmThatStartedUpErrored)
{
  ASSERT_TRUE(waitForComponentState("athena_arm_interface", "unconfigured"));

  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);
  ASSERT_NE(motor->getHardwareError(), 0) << "Test premise: the motor must still be flagging a fault";

  // Refused while the fault is latched: energising a motor that is flagging one is what the error
  // state exists to prevent.
  EXPECT_NE(hardwareComponentState("athena_arm_interface"), "active")
      << "The arm activated with a motor reporting a hardware error";

  auto response = rebootMotors("athena_arm_interface");
  ASSERT_NE(response, nullptr) << "The reboot service did not answer";
  ASSERT_EQ(motor->getHardwareError(), 0) << "The reboot did not clear the fault: " << response->message;

  ASSERT_TRUE(setHardwareComponentState("athena_arm_interface", lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                        "inactive"));
  ASSERT_TRUE(
      setHardwareComponentState("athena_arm_interface", lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active"));
  std::this_thread::sleep_for(500ms);

  EXPECT_EQ(hardwareComponentState("athena_arm_interface"), "active")
      << "The arm could not be brought up after its fault was rebooted away, so a process that "
         "started against an errored motor never recovers without a power cycle.";
}

// The reboot service decides what to reboot from the hardware error each joint last reported. In a
// process that has never completed a state read, nothing has reported anything, so a service that
// trusts that cache finds no work, does none, and answers that it rebooted successfully.
TEST_F(PreExistingHardwareErrorTest, RebootServiceRecoversAMotorThisProcessNeverSawFail)
{
  ASSERT_TRUE(waitForComponentState("athena_arm_interface", "unconfigured"));

  auto motor = dynamixel_ros_control::MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);
  ASSERT_NE(motor->getHardwareError(), 0) << "Test premise: the motor must still be flagging a fault";

  auto response = rebootMotors("athena_arm_interface");
  ASSERT_NE(response, nullptr) << "The reboot service did not answer";

  EXPECT_EQ(motor->getHardwareError(), 0)
      << "The reboot service answered '" << response->message
      << "' without rebooting the motor that is flagging a fault. Whatever it reports, the fault "
         "is still latched and the arm cannot be brought back.";
}

}  // namespace dynamixel_ros_control::test
