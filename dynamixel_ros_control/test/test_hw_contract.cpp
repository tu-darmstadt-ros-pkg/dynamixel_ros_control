// The plugin contract of hector_controller_manager, checked on the Dynamixel plugin and its mock bus,
// once directly and once through the real SDK over a pty

#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>

#include <dynamixel_ros_control/dynamixel_hardware_interface.hpp>
#include <hardware_contract_tests/hardware_contract_tests.hpp>
#include <hardware_interface/component_parser.hpp>

#include "test_hardware_interface_common.hpp"
#include "wire_bus.hpp"

namespace dynamixel_ros_control::test {

class DynamixelPlant
{
public:
  ~DynamixelPlant()
  {
    MockDynamixelManager::instance().reset();
  }

  std::unique_ptr<hardware_interface::SystemInterface> make() const
  {
    return std::make_unique<DynamixelHardwareInterface>();
  }

  hardware_interface::HardwareInfo info() const
  {
    for (const auto& info : hardware_interface::parse_control_resources_from_urdf(
             load_file(std::string(TEST_CONFIG_DIR) + "/athena.urdf"))) {
      if (info.name == "athena_arm_interface") {
        addMotorsOf(info);
        return info;
      }
    }
    throw std::runtime_error("athena_arm_interface is missing from the test robot");
  }

  std::string joint() const
  {
    return "arm_joint_1";
  }

  void step(double period) const
  {
    MockDynamixelManager::instance().update(period);
  }

  void push(double delta) const
  {
    MockDynamixelManager::instance().getMotor(ARM_JOINT_1_ID)->moveByHand(delta);
  }

private:
  // The plugin keeps files in $HOME, which each test gets fresh
  TestHome home_;
};

class WireDynamixelPlant : public DynamixelPlant
{
public:
  WireDynamixelPlant()
      : info_(DynamixelPlant::info()), server_(serveOverWire(info_))
  {}

  hardware_interface::HardwareInfo info() const
  {
    return info_;
  }

  void step(double period) const
  {
    server_->waitUntilIdle();
    DynamixelPlant::step(period);
  }

private:
  hardware_interface::HardwareInfo info_;
  std::unique_ptr<MockBusServer> server_;
};

}  // namespace dynamixel_ros_control::test

namespace hardware_contract_tests {
INSTANTIATE_TYPED_TEST_SUITE_P(Dynamixel, HardwareContract, dynamixel_ros_control::test::DynamixelPlant);
INSTANTIATE_TYPED_TEST_SUITE_P(DynamixelOverWire, HardwareContract, dynamixel_ros_control::test::WireDynamixelPlant);
}  // namespace hardware_contract_tests
