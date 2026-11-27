#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {

// A power blip shorter than the read error threshold reboots a motor without a single failed cycle
// too many. The motor comes back limp, with its RAM registers reset, and must not be driven on.
class PowerBlipTest : public DirectHardwareTest
{
protected:
  void SetUp() override
  {
    DirectHardwareTest::SetUp();
    startComponent("athena_arm_interface");
    motor_ = MockDynamixelManager::instance().getMotor(ARM_JOINT_2_ID);
    ASSERT_NE(motor_, nullptr);
  }

  /// The motor is gone for ten cycles and comes back rebooted
  void blip()
  {
    motor_->setCommunicationFault(CommunicationFault::DISCONNECTED);
    cycles(10);
    motor_->powerCycle();
    motor_->setCommunicationFault(CommunicationFault::NONE);
  }

  std::shared_ptr<MockDynamixel> motor_;
};

TEST_F(PowerBlipTest, AMotorThatRebootedFaultsTheComponent)
{
  blip();
  EXPECT_EQ(cycle(), hardware_interface::return_type::ERROR) << "a rebooted motor was driven on as if nothing happened";
}

// A joint held torque off reads torque off by intent, which is no reboot
TEST_F(PowerBlipTest, AJointHeldTorqueOffIsNoReboot)
{
  ASSERT_EQ(hardware_->perform_command_mode_switch({"arm_joint_2/torque_off"}, {}), hardware_interface::return_type::OK);
  cycles(5);
  blip();
  EXPECT_EQ(cycle(), hardware_interface::return_type::OK);
}

// Activation derives the torque afresh, so a motor that rebooted while inactive is no fault
TEST_F(PowerBlipTest, ARebootWhileInactiveIsNoFault)
{
  ASSERT_EQ(hardware_->on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  blip();
  EXPECT_EQ(cycle(), hardware_interface::return_type::OK);
}

// X-series motors keep their indirect address pointers in RAM, so a reboot also wipes the mapping
// that carries torque_enable in the cyclic read. The reboot must still fault the component.
class XSeriesPowerBlipTest : public PowerBlipTest
{
protected:
  void SetUp() override
  {
    DirectHardwareTest::SetUp();
    startComponent("athena_flipper_interface", [](std::string urdf) {
      for (const int id : {1, 2, 3, 4}) {
        const std::string anchor = "<param name=\"id\">" + std::to_string(id) + "</param>";
        const auto pos = urdf.find(anchor);
        EXPECT_NE(pos, std::string::npos);
        urdf.insert(pos + anchor.size(), "<param name=\"model_number\">1020</param>");  // XM430-W350
      }
      // XM has 28 indirect slots, too few for more than position
      const auto begin = urdf.find("<ros2_control name=\"athena_flipper_interface\"");
      const auto end = urdf.find("</ros2_control>", begin);
      std::string flippers = urdf.substr(begin, end - begin);
      for (const std::string interface :
           {"<command_interface name=\"current\"/>", "<state_interface name=\"current\"/>",
            "<command_interface name=\"velocity\"/>", "<state_interface name=\"velocity\"/>"}) {
        for (auto pos = flippers.find(interface); pos != std::string::npos; pos = flippers.find(interface)) {
          flippers.erase(pos, interface.size());
        }
      }
      return urdf.replace(begin, end - begin, flippers);
    });
    motor_ = MockDynamixelManager::instance().getMotor(FLIPPER_FL_ID);
    ASSERT_NE(motor_, nullptr);
  }
};

TEST_F(XSeriesPowerBlipTest, AMotorThatRebootedWithItsMappingWipedFaultsTheComponent)
{
  blip();
  EXPECT_EQ(cycle(), hardware_interface::return_type::ERROR) << "a rebooted motor was driven on as if nothing happened";
}

}  // namespace dynamixel_ros_control::test
