// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

// ============================================================================
// Hardware Interface Lifecycle Tests
// ============================================================================

// Deactivation stops read()/write(), so a joint left torqued in velocity or current mode keeps
// executing whatever goal it last received. It is only stationary in position mode, where the goal
// *is* a standstill. Disarming the bus watchdog at the same time removes the one mechanism that
// would have stopped it, so the goals have to be zeroed first.
TEST_F(HardwareInterfaceTest, Lifecycle_DeactivationZeroesAFreeRunningVelocityJoint)
{
  std::this_thread::sleep_for(400ms);
  ASSERT_TRUE(loadAndActivateController("flipper_velocity_controller"));
  std::this_thread::sleep_for(300ms);

  auto cmd_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_velocity_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(cmd_pub, 5s));
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = std::vector<double>(FLIPPER_MOTOR_IDS.size(), 0.5);  // rad/s, well clear of zero
  cmd_pub->publish(cmd);
  std::this_thread::sleep_for(600ms);

  auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_NE(motor, nullptr);
  const auto goal_velocity_address = motor->getAddress("goal_velocity");
  ASSERT_NE(motor->read4ByteSigned(goal_velocity_address), 0)
      << "Test premise: the joint must be commanded to a non-zero velocity before deactivating";

  ASSERT_TRUE(setHardwareComponentState("athena_flipper_interface", lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
                                        "inactive"));
  std::this_thread::sleep_for(500ms);

  EXPECT_EQ(motor->read4ByteSigned(goal_velocity_address), 0)
      << "The joint was left with a non-zero goal velocity while the loop that commands it stopped "
         "and the watchdog that would stop it was disarmed.";
}

// torque_off_on_shutdown is off in the shared config, so the whole failure path below is otherwise
// never exercised.
class TorqueOffOnShutdownTest : public HardwareInterfaceTest
{
protected:
  std::string transformUrdf(std::string urdf) override
  {
    // The shared URDF already declares this param as false for both interfaces, so injecting a
    // second one next to it silently loses. Flip the arm's own declaration instead, located after
    // the arm interface's unique port_name so the flipper's is left alone.
    const std::string anchor = "<param name=\"port_name\">/dev/ttyUSB_manipulator_arm</param>";
    const auto anchor_pos = urdf.find(anchor);
    if (anchor_pos == std::string::npos) {
      ADD_FAILURE() << "Could not find the arm interface anchor";
      return urdf;
    }
    const std::string target = "<param name=\"torque_off_on_shutdown\">false</param>";
    const auto pos = urdf.find(target, anchor_pos);
    if (pos == std::string::npos) {
      ADD_FAILURE() << "Could not find the arm interface's torque_off_on_shutdown param";
      return urdf;
    }
    urdf.replace(pos, target.size(), "<param name=\"torque_off_on_shutdown\">true</param>");
    return urdf;
  }
};

// A torque-off that cannot be verified is the case that leaves a joint energised on the way out.
// The block this used to return past is the one that stops such a joint. Returning early skipped it
// and left the interface with the motor still torqued in velocity mode, the watchdog disarmed (the
// suspension inside setTorque re-arms nothing once the phase is Inactive) and read()/write() about
// to stop, with nothing left to halt it.
TEST_F(TorqueOffOnShutdownTest, DeactivationStopsAFreeRunningJointWhenTheTorqueOffFails)
{
  std::this_thread::sleep_for(400ms);

  ASSERT_TRUE(loadAndActivateController("arm_velocity_controller"));
  std::this_thread::sleep_for(300ms);

  auto motor = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(motor, nullptr);

  auto cmd_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_velocity_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(cmd_pub, 5s));
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = std::vector<double>(ARM_MOTOR_IDS.size(), 0.5);
  cmd_pub->publish(cmd);
  std::this_thread::sleep_for(600ms);
  ASSERT_GT(std::abs(motor->getCurrentVelocity()), 0.05) << "Test premise: the joint should be turning";

  // The motor acknowledges the torque-off and keeps torque on. The master sees a successful write
  // and a read-back that disagrees with it, so the change is unverifiable and the joint stays
  // energised and turning.
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto m = MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(m, nullptr);
    m->setRegisterWriteFault("torque_enable", RegisterWriteFault::DROP);
  }

  auto hw_state_client =
      tester_node_->create_test_client<SetHardwareComponentState>("/controller_manager/set_hardware_component_state");
  ASSERT_TRUE(hw_state_client->wait_for_service(*executor_, 5s));
  hector_testing_utils::ServiceCallOptions options;
  options.service_timeout = 20s;
  options.response_timeout = 20s;
  auto request = std::make_shared<SetHardwareComponentState::Request>();
  request->name = "athena_arm_interface";
  request->target_state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
  request->target_state.label = "inactive";
  (void) hector_testing_utils::call_service<SetHardwareComponentState>(hw_state_client->get(), request, *executor_,
                                                                       options);

  // The mock's physics keeps stepping regardless of the component's lifecycle state, so a joint
  // whose zero goal never reached it is still turning here.
  std::this_thread::sleep_for(1000ms);

  EXPECT_LT(std::abs(motor->getCurrentVelocity()), 0.05)
      << "The joint is still turning at " << motor->getCurrentVelocity()
      << " rad/s after deactivation. The failed torque-off returned before the block that commands a "
         "zero velocity, and nothing else will: the watchdog is disarmed and read()/write() have stopped.";

  for (uint8_t id : ARM_MOTOR_IDS) {
    auto m = MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(m, nullptr);
    m->clearRegisterWriteFaults();
  }
}

}  // namespace dynamixel_ros_control::test
