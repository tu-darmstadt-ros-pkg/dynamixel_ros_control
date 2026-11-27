#include "test_hardware_interface_common.hpp"

#include <cstring>

namespace dynamixel_ros_control::test {

// Securing must not energise a motor before its hold goal has been written and verified.
// Otherwise it briefly runs in position mode against a stale Goal Position and chases it.
//
// Safe and unsafe orderings leave identical registers behind, and the window between the two writes
// is microseconds in the mock, so watching for motion would pass or fail by luck. These tests use
// the mock's write trace to decide the ordering directly.

namespace {
bool motorTorqued(uint8_t id)
{
  auto motor = MockDynamixelManager::instance().getMotor(id);
  EXPECT_NE(motor, nullptr) << "Motor " << static_cast<int>(id) << " missing";
  return motor != nullptr && motor->read1Byte(motor->getAddress("torque_enable")) == 1;
}
}  // namespace

// Reaching the mode-switch branch needs the joint in a non-position mode when securing begins. A
// hardware error goes straight to on_error with the velocity controller still active.
TEST_F(HardwareInterfaceTest, SecuringNeverRestoresTorqueBeforeWritingHoldGoal)
{
  std::this_thread::sleep_for(400ms);

  // Put the arm into velocity mode and let it build up a goal that is not its current position.
  ASSERT_TRUE(loadAndActivateController("arm_velocity_controller"));
  std::this_thread::sleep_for(300ms);

  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr);
    ASSERT_EQ(motor->read1Byte(motor->getAddress("operating_mode")),
              static_cast<uint8_t>(OperatingMode::VELOCITY_CONTROL))
        << "Test premise: motor " << static_cast<int>(id) << " must be in velocity mode";
    ASSERT_TRUE(motorTorqued(id)) << "Test premise: motor " << static_cast<int>(id) << " must be torqued";
  }

  auto cmd_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_velocity_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(cmd_pub, 5s));
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = std::vector<double>(ARM_MOTOR_IDS.size(), 0.5);
  cmd_pub->publish(cmd);
  std::this_thread::sleep_for(600ms);

  // Record every applied write from here on.
  for (uint8_t id : ARM_MOTOR_IDS) {
    MockDynamixelManager::instance().getMotor(id)->startWriteTrace();
  }

  // A hardware error drives on_error with the joints still in velocity mode.
  auto faulted = MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID);
  ASSERT_NE(faulted, nullptr);
  faulted->setHardwareError(ERROR_OVERHEATING);
  ASSERT_TRUE(waitForComponentState("athena_arm_interface", "unconfigured"));

  size_t reenergised = 0;
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    motor->stopWriteTrace();

    const int torque_on_index = motor->lastWriteIndex("torque_enable", 1);
    const int goal_position_index = motor->lastWriteIndex("goal_position");

    if (torque_on_index < 0) {
      continue;  // never re-energised during securing: trivially safe
    }
    ++reenergised;
    ASSERT_GE(goal_position_index, 0)
        << "Motor " << static_cast<int>(id)
        << " had torque restored during securing but no hold goal was ever written to it.";
    EXPECT_LT(goal_position_index, torque_on_index)
        << "Motor " << static_cast<int>(id) << ": securing re-enabled torque (write #" << torque_on_index
        << ") BEFORE writing the hold goal (write #" << goal_position_index
        << "). The motor was energised in position mode with a stale Goal Position and would chase it.";
  }
  EXPECT_GT(reenergised, 0u) << "securing re-energised no motor, so the order under test never happened";

  faulted->setHardwareError(0);
}

// A velocity-only joint has no position interface. Dropping torque would let a gravity-loaded joint
// fall, so it is commanded to zero velocity in the mode it is actually in, and that command is
// verified before the joint counts as secured.
class VelocityOnlyJointTest : public HardwareInterfaceTest
{
protected:
  std::string transformUrdf(std::string urdf) override
  {
    for (const char* joint : {"flipper_fl_joint", "flipper_fr_joint", "flipper_bl_joint", "flipper_br_joint"}) {
      const std::string open_tag = "<joint name=\"" + std::string(joint) + "\">";
      const auto start = urdf.find(open_tag);
      if (start == std::string::npos) {
        ADD_FAILURE() << "Could not find ros2_control joint block for " << joint;
        return urdf;
      }
      const auto end = urdf.find("</joint>", start);
      if (end == std::string::npos) {
        ADD_FAILURE() << "Unterminated joint block for " << joint;
        return urdf;
      }
      std::string block = urdf.substr(start, end - start);
      // Remove the position *command* interface only. The state interface stays, since the
      // transmission and the mock physics both need it.
      const char* line = "<command_interface name=\"position\"/>";
      for (auto pos = block.find(line); pos != std::string::npos; pos = block.find(line)) {
        block.erase(pos, std::strlen(line));
      }
      urdf.replace(start, end - start, block);
    }
    return urdf;
  }
};

// Deactivating a controller leaves a joint with no active command interfaces, and
// Joint::updateControlMode() then switches it to its preferred *position* mode, even without a
// position command interface. So the zero-velocity branch is only reachable when that mode switch
// fails, which is also when it matters most: the joint is still in velocity mode executing its last
// goal, and dropping torque would let a loaded flipper fall.
TEST_F(VelocityOnlyJointTest, SecuringBrakesVelocityOnlyJointInsteadOfDroppingIt)
{
  std::this_thread::sleep_for(400ms);

  ASSERT_TRUE(loadAndActivateController("flipper_velocity_controller"));
  std::this_thread::sleep_for(300ms);

  auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_NE(motor, nullptr);
  ASSERT_EQ(motor->read1Byte(motor->getAddress("operating_mode")), static_cast<uint8_t>(OperatingMode::VELOCITY_CONTROL))
      << "Test premise: the flipper should be in velocity mode";

  auto cmd_pub =
      tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/flipper_velocity_controller/commands", 10);
  ASSERT_TRUE(waitForSubscribers(cmd_pub, 5s));
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = std::vector<double>(FLIPPER_MOTOR_IDS.size(), 0.8);
  cmd_pub->publish(cmd);
  std::this_thread::sleep_for(800ms);

  ASSERT_GT(std::abs(motor->getCurrentVelocity()), 0.05) << "Test premise: the flipper should be moving";
  ASSERT_TRUE(motorTorqued(FLIPPER_FL_ID)) << "Test premise: the flipper should be torqued";

  // Pin the joints in velocity mode: from here the mode cannot be changed.
  for (uint8_t id : FLIPPER_MOTOR_IDS) {
    MockDynamixelManager::instance().getMotor(id)->setRegisterWriteFault("operating_mode", RegisterWriteFault::TIMEOUT);
  }

  // Deactivating the controller now fails its mode switch, which faults the component, and on_error
  // secures it with the joints still in velocity mode.
  switch_controllers({}, {"flipper_velocity_controller"});
  std::this_thread::sleep_for(1200ms);

  ASSERT_EQ(motor->read1Byte(motor->getAddress("operating_mode")), static_cast<uint8_t>(OperatingMode::VELOCITY_CONTROL))
      << "Test premise: the flipper must still be in velocity mode for this branch to be exercised";

  EXPECT_EQ(motor->read4ByteSigned(motor->getAddress("goal_velocity")), 0)
      << "A velocity-only joint still in velocity mode was not commanded to zero velocity by securing.";
  EXPECT_NEAR(motor->getCurrentVelocity(), 0.0, 0.05) << "The flipper is still moving after securing";

  // Braked, not dropped: torque must survive.
  EXPECT_TRUE(motorTorqued(FLIPPER_FL_ID))
      << "Securing dropped torque on a velocity-only joint instead of braking it at zero velocity.";

  for (uint8_t id : FLIPPER_MOTOR_IDS) {
    MockDynamixelManager::instance().getMotor(id)->clearRegisterWriteFaults();
  }
}

}  // namespace dynamixel_ros_control::test
