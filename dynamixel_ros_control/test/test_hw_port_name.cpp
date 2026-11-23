#include <string>

#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {

class PortNameTest : public DirectHardwareTest
{
protected:
  void adjustInfo(hardware_interface::HardwareInfo& info) override
  {
    info.hardware_parameters["use_dummy"] = "false";
    info.hardware_parameters["port_name"] = "/tmp/" + std::string(95, 'x');
  }
};

// The SDK copies the name into 100 bytes, so a name of 100 characters must fail init, not memory
TEST_F(PortNameTest, APortNameTooLongForTheSdkFailsInit)
{
  initComponent("athena_arm_interface");
  EXPECT_EQ(hardware_->init(params_), hardware_interface::CallbackReturn::ERROR);
}

}  // namespace dynamixel_ros_control::test
