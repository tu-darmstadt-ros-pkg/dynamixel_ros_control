// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause
//
// Guards the normal (fresh-boot) command-mode-switch path for joints whose URDF
// command-interface order differs from their siblings — e.g. the real arm declares
// joints 2/4/6 as `position, current, velocity` and 1/3/5/7 as `position, velocity,
// current`. On a fresh boot the indirect-address pointers are written correctly
// (torque is off during on_configure), so a position switch must succeed and reset
// every joint's goal velocity to 0 regardless of interface order.
//
// The actual field failure required STALE EEPROM indirect pointers surviving because
// the motors were already torqued at configure time (torque_off_on_shutdown:false +
// no power cycle). That precondition can't arise in this fixture, which always builds
// the controller manager from scratch with fresh, un-torqued motors — so the EEPROM
// write-lock mechanism is reproduced and pinned in the unit test
// StaleIndirectPointerSurvivesWhenTorquedAndMisalignsGoalRead (test_mock_dynamixel).
//
// The default test URDF uses one identical interface order for every arm joint, which
// is why the existing suite never exercised the mixed-order path. Here we mutate the
// URDF (via transformUrdf) to recreate the real robot's mixed order.

#include "test_hardware_interface_common.hpp"

namespace dynamixel_ros_control::test {

class MixedInterfaceOrderTest : public HardwareInterfaceTest
{
protected:
  // Swap the command-interface order of the even arm joints (2, 4, 6) from the
  // canonical `position, velocity, current` to `position, current, velocity`.
  // Everything else in the URDF is left untouched.
  std::string transformUrdf(std::string urdf) override
  {
    const std::string canonical = "      <command_interface name=\"position\"/>\n"
                                  "      <command_interface name=\"velocity\"/>\n"
                                  "      <command_interface name=\"current\"/>\n";
    const std::string swapped = "      <command_interface name=\"position\"/>\n"
                                "      <command_interface name=\"current\"/>\n"
                                "      <command_interface name=\"velocity\"/>\n";

    for (const std::string& joint : {"arm_joint_2", "arm_joint_4", "arm_joint_6"}) {
      swapInterfaceOrderForJoint(urdf, joint, canonical, swapped);
    }
    return urdf;
  }

private:
  // Replace the canonical command-interface block with the swapped one, but only
  // inside the <joint name="..."> element for the given joint.
  static void swapInterfaceOrderForJoint(std::string& urdf, const std::string& joint, const std::string& canonical,
                                         const std::string& swapped)
  {
    const std::string marker = "<joint name=\"" + joint + "\">";
    const size_t joint_pos = urdf.find(marker);
    ASSERT_NE(joint_pos, std::string::npos) << "Joint '" << joint << "' not found in test URDF";

    const size_t block_pos = urdf.find(canonical, joint_pos);
    ASSERT_NE(block_pos, std::string::npos)
        << "Canonical command-interface block not found for joint '" << joint << "'";

    // Guard against accidentally crossing into the next joint's block.
    const size_t next_joint = urdf.find("<joint name=\"", joint_pos + marker.size());
    ASSERT_TRUE(next_joint == std::string::npos || block_pos < next_joint)
        << "Command-interface block for joint '" << joint << "' bled into the next joint";

    urdf.replace(block_pos, canonical.size(), swapped);
  }
};

TEST_F(MixedInterfaceOrderTest, PositionSwitchSucceedsWithMixedInterfaceOrder)
{
  // Let the hardware settle and the initial goal reset complete.
  std::this_thread::sleep_for(300ms);

  // Activating the position controller forces a command-mode switch on all arm
  // joints, which runs resetGoalStateAndVerify. On a fresh boot the indirect
  // pointers are correct, so this must succeed for every joint regardless of the
  // (mixed) command-interface order.
  EXPECT_TRUE(loadAndActivateController("arm_position_controller"))
      << "Position controller failed to activate with mixed command-interface order.";
}

TEST_F(MixedInterfaceOrderTest, GoalVelocityReadsBackZeroAfterPositionSwitch)
{
  std::this_thread::sleep_for(300ms);

  ASSERT_TRUE(loadAndActivateController("arm_position_controller")) << "Position controller failed to activate";

  // Give the write/read cycle a moment to push the reset goals.
  std::this_thread::sleep_for(300ms);

  // In position mode the commanded goal velocity must be 0 on every joint,
  // regardless of interface order.
  for (uint8_t id : ARM_MOTOR_IDS) {
    auto motor = MockDynamixelManager::instance().getMotor(id);
    ASSERT_NE(motor, nullptr) << "Motor " << static_cast<int>(id) << " not found";
    uint16_t goal_vel_addr = motor->getAddress("goal_velocity");
    ASSERT_GT(goal_vel_addr, 0u);
    EXPECT_EQ(motor->read4ByteSigned(goal_vel_addr), 0)
        << "Joint with motor id " << static_cast<int>(id)
        << " has a non-zero goal velocity after switching to position mode";
  }
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
