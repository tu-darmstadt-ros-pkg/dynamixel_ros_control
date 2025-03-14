#include "dynamixel_ros_control/common.hpp"
#include "dynamixel_ros_control/log.hpp"

#include <dynamixel_ros_control/joint.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace {

std::string interfaceToRegisterName(const std::string& interface_name,
                                    const std::unordered_map<std::string, std::string>& conversion_map)
{
  std::string register_name = interface_name;  // Default to interface name
  try {
    register_name = conversion_map.at(interface_name);  // Check if there is a specific configuration
  }
  catch (const std::out_of_range&) {
  }
  return register_name;
}

}  // namespace

namespace dynamixel_ros_control {

bool Joint::loadConfiguration(DynamixelDriver& driver, const hardware_interface::ComponentInfo& info,
                              const std::unordered_map<std::string, std::string>& state_interface_to_register,
                              const std::unordered_map<std::string, std::string>& command_interface_to_register)
{
  name = info.name;
  state_interface_to_register_ = state_interface_to_register;
  command_interface_to_register_ = command_interface_to_register;

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

  // Preferred position command mode
  std::string control_mode_str;
  if (getParameter(info.parameters, "position_control_mode", control_mode_str)) {
    preferred_position_control_mode_ = stringToControlMode(control_mode_str);
  }

  // Initial values
  std::unordered_map<std::string, std::string> initial_register_values;
  for (const auto& [param_name, param_value] : info.parameters) {
    if (param_name.rfind("registers.", 0) != 0) {
      continue;
    }
    std::string register_name = param_name.substr(10, param_name.size() - 10);
    initial_register_values.emplace(register_name, param_value);
  }
  dynamixel->setInitialRegisterValues(initial_register_values);

  return true;
}
bool Joint::connect()
{
  if (!dynamixel->connect()) {
    return false;
  }
  dynamixel->readRegister(DXL_REGISTER_CMD_TORQUE, torque);
  if (!initDefaultGoalValues()) {
    return false;
  }
  return true;
}

void Joint::reset()
{
  active_command_interfaces_.clear();
  control_mode_ = UNDEFINED;
  for (auto& [interface_name, value] : joint_state.current) {
    value = 0.0;
  }
  for (auto& [interface_name, value] : actuator_state.current) {
    value = 0.0;
  }
  for (auto& [interface_name, value] : joint_state.goal) {
    value = 0.0;
  }
  for (auto& [interface_name, value] : actuator_state.goal) {
    value = 0.0;
  }
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
  // Check if existing interface
  const auto it = std::find(command_interfaces_.begin(), command_interfaces_.end(), interface_name);
  if (it == command_interfaces_.end()) {
    DXL_LOG_ERROR("No available command interface '" << interface_name << "' for joint '" << name << "'.");
    return false;
  }

  // Check if already included
  const auto it2 = std::find(active_command_interfaces_.begin(), active_command_interfaces_.end(), interface_name);
  if (it2 != active_command_interfaces_.end()) {
    // Already included
    return true;
  }

  resetGoalState(interface_name);
  active_command_interfaces_.emplace_back(interface_name);
  return true;
}

bool Joint::removeActiveCommandInterface(const std::string& interface_name)
{
  // Check if it exists
  const auto it = std::find(active_command_interfaces_.begin(), active_command_interfaces_.end(), interface_name);
  if (it == active_command_interfaces_.end()) {
    DXL_LOG_ERROR("Could not remove interface '" << interface_name << "' because it is not active for joint '" << name
                                                 << "'.");
    return false;
  }

  resetGoalState(interface_name);
  active_command_interfaces_.erase(it);
  return true;
}

bool Joint::updateControlMode()
{
  if (active_command_interfaces_.empty()) {
    // Do nothing if no command interface is active
    return true;
  }
  // determine required control mode
  const ControlMode new_control_mode = getControlModeFromInterfaces(active_command_interfaces_);
  if (new_control_mode == control_mode_) {
    return true;
  }
  DXL_LOG_DEBUG("Changing control mode of joint '" << name << "' from '" << control_mode_ << " to '" << new_control_mode
                                                   << "'.");
  control_mode_ = new_control_mode;
  control_mode_changed_ = true;

  // write control mode
  return dynamixel->writeControlMode(control_mode_, torque);
}

bool Joint::controlModeChanged() const
{
  return control_mode_changed_;
}

void Joint::resetControlModeChanged()
{
  control_mode_changed_ = false;
}

std::string Joint::stateInterfaceToRegisterName(const std::string& interface_name) const
{
  return interfaceToRegisterName(interface_name, state_interface_to_register_);
}

std::string Joint::commandInterfaceToRegisterName(const std::string& interface_name) const
{
  return interfaceToRegisterName(interface_name, command_interface_to_register_);
}

void Joint::resetGoalState(const std::string& interface_name)
{
  double& value = getActuatorState().goal.at(interface_name);  // This should exist

  // Special handling for position
  if (interface_name == hardware_interface::HW_IF_POSITION) {
    try {
      value = getActuatorState().current.at(hardware_interface::HW_IF_POSITION);
    }
    catch (const std::out_of_range&) {
      DXL_LOG_WARN("Joint '"
                   << name
                   << "' is controlled in position mode but the current position is not read out. Cannot initialize "
                      "goal field. Add a position state interface to this joint to resolve this problem.");
      value = 0;
    }
  } else {
    // Default value
    value = default_goal_values_.at(interface_name); // This should exist
  }

  if (command_transmission) {
    command_transmission->actuator_to_joint(); // Unfortunately, there is no interface for single interface handles
  }
}

void Joint::resetGoalState()
{
  for (auto& interface_name : getAvailableCommandInterfaces()) {
    resetGoalState(interface_name);
  }
}

State& Joint::getActuatorState()
{
  return state_transmission ? actuator_state : joint_state;
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

  if (std::find(interfaces.begin(), interfaces.end(), hardware_interface::HW_IF_EFFORT) != interfaces.end() ||
      std::find(interfaces.begin(), interfaces.end(), hardware_interface::HW_IF_CURRENT) != interfaces.end()) {

    return CURRENT;
  }
  DXL_LOG_WARN("None out of the command interfaces "
               << hardware_interface::HW_IF_POSITION << ", " << hardware_interface::HW_IF_VELOCITY << ", "
               << hardware_interface::HW_IF_EFFORT << " have been requested. Defaulting to "
               << hardware_interface::HW_IF_POSITION << " mode");
  return POSITION;
}

bool Joint::initDefaultGoalValues()
{
  for (auto& interface_name : getAvailableCommandInterfaces()) {
    const std::string register_name = commandInterfaceToRegisterName(interface_name);
    double default_value;
    if (!dynamixel->readRegister(register_name, default_value)) {
      return false;
    }
    default_goal_values_[interface_name] = default_value;
  }
  return true;
}

}  // namespace dynamixel_ros_control
