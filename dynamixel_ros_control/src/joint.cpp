#include "dynamixel_ros_control/common.hpp"
#include "dynamixel_ros_control/log.hpp"

#include <dynamixel_ros_control/joint.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace dynamixel_ros_control {

bool Joint::loadConfiguration(DynamixelDriver& driver, const hardware_interface::ComponentInfo& info)
{
  name = info.name;
  getParameter(info.parameters, "mounting_offset", mounting_offset, 0.0);
  getParameter(info.parameters, "offset", offset, 0.0);
  uint8_t id;
  if (!getParameter(info.parameters, "id", id)) {
    return false;
  }
  dynamixel = std::make_shared<Dynamixel>(id, driver);

  // Requested state interfaces
  for (const auto& state_interface : info.state_interfaces) {
    state_interfaces_.emplace_back(state_interface.name);
  }

  // Requested command interfaces
  for (const auto& command_interface : info.command_interfaces) {
    command_interfaces_.emplace_back(command_interface.name);
  }

  return true;
}

ControlMode Joint::getControlMode() const
{
  return control_mode_;
}

bool Joint::setControlMode(const ControlMode& value)
{
  control_mode_ = value;
  return true;
}

bool Joint::isPositionControlled() const
{
  return getControlMode() == POSITION || getControlMode() == EXTENDED_POSITION ||
         getControlMode() == CURRENT_BASED_POSITION;
}

bool Joint::isVelocityControlled() const
{
  return getControlMode() == VELOCITY;
}

bool Joint::isEffortControlled() const
{
  return getControlMode() == CURRENT;
}
const std::vector<std::string>& Joint::getStateInterfaces() const
{
  return state_interfaces_;
}
const std::vector<std::string>& Joint::getCommandInterfaces() const
{
  return command_interfaces_;
}

}  // namespace dynamixel_ros_control
