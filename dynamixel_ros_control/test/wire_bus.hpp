#ifndef DYNAMIXEL_ROS_CONTROL_TEST_WIRE_BUS_HPP
#define DYNAMIXEL_ROS_CONTROL_TEST_WIRE_BUS_HPP

#include <memory>
#include <string>
#include <vector>

#include <dynamixel_ros_control/mock_bus_server.hpp>
#include <hardware_interface/hardware_info.hpp>

#include "direct_hardware_fixture.hpp"

namespace dynamixel_ros_control::test {

/// Moves a component from the dummy bus onto a MockBusServer. The plugin reaches its motors
/// through the real SDK and a pty, at @p link if one is given.
inline std::unique_ptr<MockBusServer> serveOverWire(hardware_interface::HardwareInfo& info, const std::string& link = {})
{
  const std::vector<uint8_t> ids = addMotorsOf(info);
  auto server = std::make_unique<MockBusServer>(ids, link);
  info.hardware_parameters["use_dummy"] = "false";
  info.hardware_parameters["port_name"] = server->portName();
  return server;
}

/// A component driven directly, whose motors the plugin reaches over the wire
class WireBusTest : public DirectHardwareTest
{
protected:
  void adjustInfo(hardware_interface::HardwareInfo& info) override
  {
    server_ = serveOverWire(info);
  }

  void step() override
  {
    if (server_) {
      server_->waitUntilIdle();
    }
    DirectHardwareTest::step();
  }

  double state(const std::string& name) const
  {
    return states_.at(name)->get_optional().value();
  }

  std::unique_ptr<MockBusServer> server_;
};

}  // namespace dynamixel_ros_control::test

#endif
