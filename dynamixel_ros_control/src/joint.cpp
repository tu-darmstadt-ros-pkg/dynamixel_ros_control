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
  if (!command_interfaces_.empty()) {
    control_mode_ = getControlModeFromInterfaces(std::vector{command_interfaces_.front()});
  }

  // Preferred position command mode
  std::string control_mode_str;
  if (getParameter(info.parameters, "position_control_mode", control_mode_str)) {
    preferred_position_control_mode_ = stringToControlMode(control_mode_str);
  } else {
    preferred_position_control_mode_ = POSITION;
  }

  return true;
}

ControlMode Joint::getControlMode() const
{
  return control_mode_;
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

const std::vector<std::string>& Joint::getAvailableStateInterfaces() const
{
  return state_interfaces_;
}

const std::vector<std::string>& Joint::getAvailableCommandInterfaces() const
{
  return command_interfaces_;
}

bool Joint::addActiveCommandInterface(const std::string& interface_name)
{
  const auto it = std::find(command_interfaces_.begin(), command_interfaces_.end(), interface_name);
  if (it == command_interfaces_.end()) {
    DXL_LOG_ERROR("No available command interface '" << interface_name << "' for joint '" << name << "'.");
    return false;
  }
  active_command_interfaces_.emplace_back(interface_name);
  return true;
}

bool Joint::removeActiveCommandInterface(const std::string& interface_name)
{
  const auto it = std::find(active_command_interfaces_.begin(), active_command_interfaces_.end(), interface_name);
  if (it == active_command_interfaces_.end()) {
    DXL_LOG_ERROR("Could not remove interface '" << interface_name << "' because it is not active for joint '" << name
                                                 << "'.");
    return false;
  }
  active_command_interfaces_.erase(it);
  return true;
}

bool Joint::updateControlMode()
{
  // determine required control mode
  const ControlMode new_control_mode = getControlModeFromInterfaces(active_command_interfaces_);
  if (new_control_mode == control_mode_) {
    return true;
  }
  DXL_LOG_DEBUG("Changing control mode of joint '" << name << "' from '" << control_mode_ << " to '" << new_control_mode << "'.");
  control_mode_ = new_control_mode;

  // write control mode
  return dynamixel->writeControlMode(control_mode_, true);
}

const std::vector<std::string>& Joint::getActiveCommandInterfaces() const
{
  return active_command_interfaces_;
}

ControlMode Joint::getControlModeFromInterfaces(const std::vector<std::string>& interfaces) const
{
  const auto position_it = std::find(interfaces.begin(), interfaces.end(), hardware_interface::HW_IF_POSITION);
  if (position_it != interfaces.end()) {
    return preferred_position_control_mode_;
  }

  const auto velocity_it = std::find(interfaces.begin(), interfaces.end(), hardware_interface::HW_IF_VELOCITY);
  if (velocity_it != interfaces.end()) {
    return VELOCITY;
  }

  const auto effort_it = std::find(interfaces.begin(), interfaces.end(), hardware_interface::HW_IF_EFFORT);
  if (effort_it != interfaces.end()) {
    return CURRENT;
  }
  DXL_LOG_WARN("None out of the command interfaces "
               << hardware_interface::HW_IF_POSITION << ", " << hardware_interface::HW_IF_VELOCITY << ", "
               << hardware_interface::HW_IF_EFFORT << " have been requested. Defaulting to "
               << hardware_interface::HW_IF_POSITION);
  return POSITION;
}

}  // namespace dynamixel_ros_control
