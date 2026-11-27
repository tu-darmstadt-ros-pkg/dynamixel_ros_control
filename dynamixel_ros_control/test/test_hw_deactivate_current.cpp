// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

#include <dynamixel_ros_control/dynamixel.hpp>  // ControlMode, for the operating-mode premise

// Its own binary, and deliberately the only test in it: driving a joint in current mode needs
// forward_command_controller/ForwardCommandController, and class_loader can only register that
// class if its library enters the process through pluginlib. position_controllers and
// velocity_controllers link the same library, so once any test has loaded one of those, loading the
// base class as a plugin fails with "no factory exists for it".
namespace dynamixel_ros_control::test {

// Current mode is refused until a joint without a current command can be held (plan D1b). A
// controller that claims only current interfaces must not start, and no joint may end up in
// current mode.
//
// The current stop in on_deactivate and on_error is unreachable while the mode is refused. It stays
// as a defence and gets its test back together with current mode.
TEST_F(HardwareInterfaceTest, APureCurrentControllerIsRefusedAndNoJointEntersCurrentMode)
{
  std::this_thread::sleep_for(400ms);
  EXPECT_FALSE(loadAndActivateController("flipper_current_controller", 3s))
      << "the controller claims only current interfaces, which the hardware refuses";

  auto motor = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
  ASSERT_NE(motor, nullptr);
  EXPECT_NE(motor->read1Byte(motor->getAddress("operating_mode")), dynamixel_ros_control::CURRENT);
}

}  // namespace dynamixel_ros_control::test
