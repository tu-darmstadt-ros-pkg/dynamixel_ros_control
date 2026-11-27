#include "test_hardware_interface_common.hpp"

#include <dynamixel_ros_control/common.hpp>

namespace dynamixel_ros_control::test {

// ============================================================================
// Reboot against motors that never connected
// ============================================================================
//
// The reboot service is callable while the component is UNCONFIGURED, which is exactly the state a
// failed configuration leaves behind. A motor whose ping never succeeded has no control table, so
// every register access on it has to fail rather than reach for a table that is not there.

// One joint of the arm is dead on the bus. The other seven connect, so the interface has motors it
// could reboot, and the probe has to step over the one it cannot ask.
class RebootWithOneDeadJointTest : public HardwareInterfaceTest
{
protected:
  void SetUp() override
  {
    require_hardware_active_ = false;  // configure fails: arm_joint_1 never answers
    HardwareInterfaceTest::SetUp();
  }

  void beforeControllerManager() override
  {
    auto& manager = MockDynamixelManager::instance();
    manager.addMotor(ARM_JOINT_1_ID, DEFAULT_MOCK_MODEL_NUMBER);
    manager.getMotor(ARM_JOINT_1_ID)->setCommunicationFault(CommunicationFault::DISCONNECTED);
  }
};

TEST_F(RebootWithOneDeadJointTest, RebootAnswersInsteadOfReachingForAMissingControlTable)
{
  ASSERT_NE(hardwareComponentState("athena_arm_interface"), "active") << "fixture did not break the configuration";

  auto response = rebootMotors();
  ASSERT_NE(response, nullptr) << "the reboot service did not answer";

  // The seven motors that connected all read back healthy, so there is nothing to reboot and the
  // service says so. What matters is that it got here at all.
  EXPECT_TRUE(response->success) << response->message;
}

// Every motor on the interface is dead. There is nothing a reboot instruction could reach, so the
// service refuses instead of reporting a reboot it never performed.
class RebootWithNoConnectedJointsTest : public HardwareInterfaceTest
{
protected:
  void SetUp() override
  {
    require_hardware_active_ = false;
    HardwareInterfaceTest::SetUp();
  }

  void beforeControllerManager() override
  {
    auto& manager = MockDynamixelManager::instance();
    for (const uint8_t id : {ARM_JOINT_1_ID, ARM_JOINT_2_ID, ARM_JOINT_3_ID, ARM_JOINT_4_ID, ARM_JOINT_5_ID,
                             ARM_JOINT_6_ID, ARM_JOINT_7_ID, GRIPPER_ID}) {
      manager.addMotor(id, DEFAULT_MOCK_MODEL_NUMBER);
      manager.getMotor(id)->setCommunicationFault(CommunicationFault::DISCONNECTED);
    }
  }
};

TEST_F(RebootWithNoConnectedJointsTest, RebootRefusesWhenNoMotorEverConnected)
{
  ASSERT_NE(hardwareComponentState("athena_arm_interface"), "active") << "fixture did not break the configuration";

  auto response = rebootMotors();
  ASSERT_NE(response, nullptr) << "the reboot service did not answer";
  EXPECT_FALSE(response->success) << "a reboot that reached no motor reported success: " << response->message;
}

}  // namespace dynamixel_ros_control::test
