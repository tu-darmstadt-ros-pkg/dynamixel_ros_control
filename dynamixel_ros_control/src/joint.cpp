#include "dynamixel_ros_control/common.hpp"
#include "dynamixel_ros_control/log.hpp"

#include <dynamixel_ros_control/joint.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hector_transmission_interface/adjustable_offset_transmission.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

/// The register the map names for @p interface_name, or the interface name itself, which then is a register
std::string interfaceToRegisterName(const std::string& interface_name,
                                    const std::unordered_map<std::string, std::string>& conversion_map)
{
  const auto it = conversion_map.find(interface_name);
  return it != conversion_map.end() ? it->second : interface_name;
}

}  // namespace

namespace dynamixel_ros_control {

bool Joint::loadConfiguration(DynamixelDriver& driver, const hardware_interface::ComponentInfo& info,
                              const std::unordered_map<std::string, std::string>& state_interface_to_register,
                              const std::unordered_map<std::string, std::string>& command_interface_to_register,
                              const std::unordered_map<std::string, std::string>& interface_to_register_limits)
{
  name = info.name;
  state_interface_to_register_ = state_interface_to_register;
  command_interface_to_register_ = command_interface_to_register;
  interface_to_register_limits_ = interface_to_register_limits;

  uint8_t id;
  if (!getParameter(info.parameters, "id", id)) {
    // Named here rather than left to the caller: a refused description takes the controller
    // manager with it, so this log line is all the operator gets.
    DXL_LOG_ERROR("Joint '" << name << "' has no usable 'id' parameter.");
    return false;
  }
  dynamixel = std::make_shared<Dynamixel>(id, driver);

  // Requested state interfaces
  for (const auto& state_interface : info.state_interfaces) {
    state_interfaces_.emplace_back(state_interface.name);
  }

  // Requested command interfaces. The mode switch handles torque_off, so it stays out of them.
  for (const auto& command_interface : info.command_interfaces) {
    if (command_interface.name == TORQUE_OFF_INTERFACE) {
      exports_torque_off_ = true;
      continue;
    }
    // Only these have a safe reset goal, any other register would be written with 0 on activation
    if (command_interface.name != hardware_interface::HW_IF_POSITION &&
        command_interface.name != hardware_interface::HW_IF_VELOCITY &&
        command_interface.name != hardware_interface::HW_IF_EFFORT &&
        command_interface.name != hardware_interface::HW_IF_CURRENT &&
        command_interface.name != MAX_CURRENT_INTERFACE && command_interface.name != MAX_VELOCITY_INTERFACE) {
      DXL_LOG_ERROR("Joint '" << name << "' declares the command interface '" << command_interface.name
                              << "'. Only position, velocity, effort, current, max_current, max_velocity and "
                                 "torque_off are supported.");
      return false;
    }
    command_interfaces_.emplace_back(command_interface.name);
  }

  // Preferred position command mode
  std::string control_mode_str;
  if (getParameter(info.parameters, "position_control_mode", control_mode_str)) {
    // stringToControlMode throws on an unknown name. Caught here because the only caller is
    // on_init: an exception escaping into the controller manager reports a plugin that crashed
    // rather than a joint whose configured mode does not exist.
    try {
      const ControlMode mode = stringToControlMode(control_mode_str);
      // This is the mode a claimed position command interface selects, and the mode the emergency
      // hold puts the joint into. A non-position one leaves the motor ignoring every goal position
      // it is sent, and turns the hold into a torque-off.
      if (!isPositionMode(mode)) {
        DXL_LOG_ERROR("Joint '" << name << "' requests position_control_mode '" << control_mode_str
                                << "', which is not a position mode. Use position, extended_position or "
                                   "current_based_position.");
        return false;
      }
      preferred_position_control_mode_ = mode;
    }
    catch (const std::invalid_argument& e) {
      DXL_LOG_ERROR("Joint '" << name << "' requests position_control_mode '" << control_mode_str
                              << "', which is not a control mode: " << e.what());
      return false;
    }
  }

  if (!getOptionalParameter(info.parameters, "do_not_reset_on_ctrl_change", keeps_goal_on_controller_change_, false)) {
    DXL_LOG_ERROR("Joint '" << name << "' has a do_not_reset_on_ctrl_change that is not a bool.");
    return false;
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
  // write() overwrites a cap's register every cycle, so a configured value there would never hold
  for (const char* cap : {MAX_CURRENT_INTERFACE, MAX_VELOCITY_INTERFACE}) {
    const std::string register_name = commandInterfaceToRegisterName(cap);
    if (std::find(command_interfaces_.begin(), command_interfaces_.end(), cap) != command_interfaces_.end() &&
        initial_register_values.count(register_name) != 0) {
      DXL_LOG_ERROR("Joint '" << name << "' declares " << cap << " and configures registers." << register_name
                              << ", which the cap overwrites.");
      return false;
    }
  }
  dynamixel->setInitialRegisterValues(initial_register_values);

  return true;
}
bool Joint::connect()
{
  if (!dynamixel->connect()) {
    return false;
  }
  if (!dynamixel->readRegister(DXL_REGISTER_CMD_TORQUE, torque)) {
    return false;
  }
  if (!initDefaultGoalValues() || !checkVelocityBasedProfile()) {
    return false;
  }
  return setupJumpResetWindow();
}

bool Joint::setupJumpResetWindow()
{
  const auto adjustable =
      std::dynamic_pointer_cast<hector_transmission_interface::AdjustableOffsetTransmission>(state_transmission);
  if (!adjustable) {
    return true;
  }
  // After connect(), so the configured homing offset is already on the motor.
  double homing_offset = 0.0;
  if (dynamixel->registerAvailable(DXL_REGISTER_HOMING_OFFSET) &&
      !dynamixel->readRegister(DXL_REGISTER_HOMING_OFFSET, homing_offset)) {
    DXL_LOG_ERROR("Joint '" << name << "': failed to read the homing offset.");
    return false;
  }
  // A motor coming back from a power cut reports its absolute position within one turn, homing
  // offset added. Which turn is a matter of representation: a signed one puts it in [-pi, pi), an
  // unsigned one in [0, 2pi), and the e-Manual states only "within one full rotation". Bounding the
  // union of the two needs no per-model claim and still admits at most two of the candidate
  // shifts, where an unbounded detector accepts every one of them.
  const double lower = homing_offset - M_PI;
  const double upper = homing_offset + 2 * M_PI;
  adjustable->jointOffset()->setResetWindow(lower, upper);
  DXL_LOG_DEBUG("Joint '" << name << "': a power cycle puts the actuator in [" << lower << ", " << upper << ") rad.");
  return true;
}

bool Joint::checkVelocityBasedProfile() const
{
  if (std::find(command_interfaces_.begin(), command_interfaces_.end(), MAX_VELOCITY_INTERFACE) ==
      command_interfaces_.end()) {
    return true;
  }
  if (!dynamixel->registerAvailable(commandInterfaceToRegisterName(MAX_VELOCITY_INTERFACE))) {
    DXL_LOG_ERROR("Joint '" << name << "' declares max_velocity, but its model has no Profile Velocity.");
    return false;
  }
  // Bit 2 makes Profile Velocity a duration, which a velocity cap cannot be written to
  int32_t drive_mode = 0;
  if (!dynamixel->readRegister(DXL_REGISTER_DRIVE_MODE, drive_mode)) {
    DXL_LOG_ERROR("Joint '" << name << "' declares max_velocity, but its drive_mode cannot be read.");
    return false;
  }
  if ((drive_mode & DXL_DRIVE_MODE_TIME_BASED_PROFILE) != 0) {
    DXL_LOG_ERROR("Joint '" << name << "' declares max_velocity, but its drive_mode " << drive_mode
                            << " selects a time-based profile, in which Profile Velocity is a duration.");
    return false;
  }
  return true;
}

void Joint::reset()
{
  active_command_interfaces_.clear();
  torque_off_claimed_ = false;
  control_mode_ = UNDEFINED;
  // NaN until the next successful read. These values are exported, and a zero here is a position
  // that a calibration request cannot tell from one the bus gave.
  constexpr double unread = std::numeric_limits<double>::quiet_NaN();
  for (auto& [interface_name, value] : joint_state.current) {
    value = unread;
  }
  for (auto& [interface_name, value] : actuator_state.current) {
    value = unread;
  }
  for (auto& [mimic_joint_name, mimic_state] : mimic_joint_states) {
    for (auto& [interface_name, value] : mimic_state.current) {
      value = unread;
    }
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

const std::vector<std::string>& Joint::getAvailableStateInterfaces() const
{
  return state_interfaces_;
}

const std::vector<std::string>& Joint::getAvailableCommandInterfaces() const
{
  return command_interfaces_;
}

const std::vector<std::string>& Joint::getActiveCommandInterfaces() const
{
  return active_command_interfaces_;
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

  if (!keeps_goal_on_controller_change_) {
    resetGoalState(interface_name);
  }
  active_command_interfaces_.emplace_back(interface_name);
  return true;
}

void Joint::removeActiveCommandInterface(const std::string& interface_name)
{
  const auto it = std::find(active_command_interfaces_.begin(), active_command_interfaces_.end(), interface_name);
  if (it == active_command_interfaces_.end()) {
    return;
  }
  if (!keeps_goal_on_controller_change_) {
    resetGoalState(interface_name);
  }
  active_command_interfaces_.erase(it);
}

void Joint::clearActiveCommandInterfaces()
{
  // One at a time over a copy, since each reset derives its mode from the interfaces still active.
  // Against the full set a position joint would keep its velocity limit as an armed profile.
  const std::vector<std::string> active = active_command_interfaces_;
  for (const auto& interface_name : active) {
    removeActiveCommandInterface(interface_name);
  }
  torque_off_claimed_ = false;
}

bool Joint::readControlMode()
{
  int32_t mode = UNDEFINED;
  if (!dynamixel->readRegister(DXL_REGISTER_CONTROL_MODE, mode)) {
    return false;
  }
  control_mode_ = static_cast<ControlMode>(mode);
  return true;
}

std::optional<ControlMode> Joint::getRequiredControlModeChange() const
{
  if (active_command_interfaces_.empty()) {
    // No controller is claiming this joint: any position-like mode holds station, so keep it.
    if (isPositionMode(control_mode_)) {
      return std::nullopt;
    }
    return preferred_position_control_mode_;
  }
  const ControlMode required = getControlModeFromInterfaces(active_command_interfaces_);
  if (required == control_mode_) {
    return std::nullopt;
  }
  return required;
}

bool Joint::updateControlMode(const bool leave_torque_off)
{
  const auto new_control_mode = getRequiredControlModeChange();
  if (!new_control_mode) {
    return true;
  }
  DXL_LOG_DEBUG("Changing control mode of joint '" << name << "' from " << control_mode_ << " to " << *new_control_mode
                                                   << (leave_torque_off ? ", leaving torque off." : "."));
  return writeControlMode(*new_control_mode, leave_torque_off);
}

bool Joint::writeControlMode(const ControlMode mode, const bool leave_torque_off, const bool torque_proven_off)
{
  // The live registers, not the cache or the `torque` flag, which is only what a statusless sync
  // write asked for. A motor already in the mode keeps its torque.
  bool restore_torque = false;
  if (!leave_torque_off) {
    int32_t live_mode = UNDEFINED;
    if (!dynamixel->readRegister(DXL_REGISTER_CONTROL_MODE, live_mode)) {
      return false;
    }
    if (live_mode == mode) {
      control_mode_ = mode;
      return true;
    }
    if (!dynamixel->readRegister(DXL_REGISTER_CMD_TORQUE, restore_torque)) {
      return false;
    }
  }
  if (leave_torque_off ? !torque_proven_off : restore_torque) {
    if (!dynamixel->writeRegister(DXL_REGISTER_CMD_TORQUE, false)) {
      return false;
    }
    torque = false;
    torque_observed = TorqueState::Off;
  }
  if (!dynamixel->writeRegister(DXL_REGISTER_CONTROL_MODE, static_cast<int32_t>(mode))) {
    return false;
  }
  control_mode_ = mode;
  if (restore_torque) {
    if (!dynamixel->writeRegister(DXL_REGISTER_CMD_TORQUE, true)) {
      return false;
    }
    torque = true;
    torque_observed = TorqueState::On;
  }
  return true;
}

std::string Joint::stateInterfaceToRegisterName(const std::string& interface_name) const
{
  return interfaceToRegisterName(interface_name, state_interface_to_register_);
}

std::string Joint::commandInterfaceToRegisterName(const std::string& interface_name) const
{
  return interfaceToRegisterName(interface_name, command_interface_to_register_);
}

std::string Joint::interfaceToLimitRegisterName(const std::string& interface_name) const
{
  return interfaceToRegisterName(interface_name, interface_to_register_limits_);
}

void Joint::resetGoalState(const std::string& interface_name)
{
  missing_command_hold_.reset();
  double& value = getActuatorState().goal.at(interface_name);  // This should exist
  ControlMode next_control_mode = getControlModeFromInterfaces(active_command_interfaces_);

  // Special handling for position
  if (interface_name == hardware_interface::HW_IF_POSITION) {
    // on_init refuses a description without a position state
    value = getActuatorState().current.at(hardware_interface::HW_IF_POSITION);
    // The reset is the newest goal, so a kept goal never reaches back past an activation
    last_position_goal_ = value;
  } else if (interface_name == hardware_interface::HW_IF_VELOCITY) {
    // In current-based position mode Goal Velocity is the profile limit, where a non-zero value
    // drives to the goal the moment torque comes on. The preferred mode counts too, because the
    // first reset runs before the mode is read.
    if (control_mode_ == VELOCITY || next_control_mode == VELOCITY || control_mode_ == CURRENT_BASED_POSITION ||
        next_control_mode == CURRENT_BASED_POSITION || preferred_position_control_mode_ == CURRENT_BASED_POSITION) {
      value = 0.0;
    } else {
      value = default_goal_values_.at(hardware_interface::HW_IF_VELOCITY);
    }
  } else if (interface_name == MAX_CURRENT_INTERFACE || interface_name == MAX_VELOCITY_INTERFACE) {
    value = default_goal_values_.at(interface_name);
  } else if (interface_name == hardware_interface::HW_IF_EFFORT || interface_name == hardware_interface::HW_IF_CURRENT) {
    if (control_mode_ == CURRENT || next_control_mode == CURRENT) {
      value = 0.0;  // the controller must write a gravity compensating current at once
    } else {
      value = default_goal_values_.at(interface_name);
    }
  }
  DXL_LOG_DEBUG("Resetting goal value of interface '" << interface_name << "' for joint '" << name << "' to " << value);
  DXL_LOG_DEBUG("Active command interfaces for joint '" << name << "': " << iterableToString(active_command_interfaces_)
                                                        << ", control mode: " << control_mode_);
  if (command_transmission) {
    goalToJoint();  // Unfortunately, there is no interface for single interface handles
  }
}

void Joint::resetGoalState()
{
  for (auto& interface_name : getAvailableCommandInterfaces()) {
    resetGoalState(interface_name);
  }
}

void Joint::holdOnMissingCommand(const bool command_taken)
{
  const auto is_active = [this](const char* interface_name) {
    return std::find(active_command_interfaces_.begin(), active_command_interfaces_.end(), interface_name) !=
           active_command_interfaces_.end();
  };
  State& actuator = getActuatorState();
  if (isPositionMode(control_mode_) && is_active(hardware_interface::HW_IF_POSITION)) {
    double& goal = actuator.goal.at(hardware_interface::HW_IF_POSITION);
    // A latched hold left in the goal by a busy handle is no command, and re-latching would creep
    if (!command_taken && missing_command_hold_) {
      goal = *missing_command_hold_;
      return;
    }
    if (!std::isnan(goal)) {
      missing_command_hold_.reset();
      last_position_goal_ = goal;
      return;
    }
    if (!missing_command_hold_) {
      // A gripper holding an object keeps squeezing only while its goal stays beyond the object
      missing_command_hold_ = keeps_goal_on_controller_change_ && last_position_goal_ ?
                                  *last_position_goal_ :
                                  actuator.current.at(hardware_interface::HW_IF_POSITION);
    }
    goal = *missing_command_hold_;
  } else if (control_mode_ == VELOCITY && is_active(hardware_interface::HW_IF_VELOCITY)) {
    double& goal = actuator.goal.at(hardware_interface::HW_IF_VELOCITY);
    if (std::isnan(goal)) {
      goal = 0.0;
    }
  }
}

void Joint::limitCaps()
{
  State& actuator = getActuatorState();
  for (const Cap& cap : caps_) {
    double& goal = actuator.goal.at(cap.interface_name);
    goal = std::isnan(goal) ? cap.ceiling : std::clamp(goal, cap.floor, cap.ceiling);
  }
}

double Joint::commandRatio(const std::string& interface_name) const
{
  // A velocity-based Profile Velocity counts in the unit of the velocity limit (e-Manual)
  if (interface_name == MAX_VELOCITY_INTERFACE) {
    return dynamixel->getItem(interfaceToLimitRegisterName(interface_name)).dxlValueToUnitRatio();
  }
  return 0.0;
}

void Joint::setupMimicJoint(const std::string& joint_name, const double offset, const double multiplier)
{
  mimic_joint_states[joint_name] = MimicState{};
  mimic_joint_states[joint_name].offset = offset;
  mimic_joint_states[joint_name].multiplier = multiplier;
}

void Joint::updateMimicJointStates()
{
  for (auto& [joint_name, mimic_state] : mimic_joint_states) {
    for (auto& [interface_name, value] : mimic_state.current) {
      if (interface_name == hardware_interface::HW_IF_POSITION) {
        value =
            joint_state.current.at(hardware_interface::HW_IF_POSITION) * mimic_state.multiplier + mimic_state.offset;
      } else {  // velocity, effort, current
        value = joint_state.current.at(interface_name) * mimic_state.multiplier;
      }
    }
  }
}

void Joint::stateToJoint()
{
  state_transmission->actuator_to_joint();
  for (const auto& [joint_value, actuator_value] : state_pass_through) {
    *joint_value = *actuator_value;
  }
}

void Joint::goalToActuator()
{
  command_transmission->joint_to_actuator();
  for (const auto& [joint_value, actuator_value] : command_pass_through) {
    *actuator_value = *joint_value;
  }
}

void Joint::goalToJoint()
{
  command_transmission->actuator_to_joint();
  for (const auto& [joint_value, actuator_value] : command_pass_through) {
    *joint_value = *actuator_value;
  }
}

State& Joint::getActuatorState()
{
  return state_transmission ? actuator_state : joint_state;
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
  return preferred_position_control_mode_;
}

// The goal of an inactive interface is its limit, e.g. the velocity limit in position mode
bool Joint::initDefaultGoalValues()
{
  caps_.clear();
  for (auto& interface_name : getAvailableCommandInterfaces()) {
    if (interface_name == hardware_interface::HW_IF_POSITION)
      continue;  // position does not need a default value
    std::string register_name = interfaceToLimitRegisterName(interface_name);
    const bool is_cap = interface_name == MAX_CURRENT_INTERFACE || interface_name == MAX_VELOCITY_INTERFACE;
    double default_value;
    // A cap's own register may still hold the cap of an earlier session, so only the EEPROM limit will do
    if (is_cap && !dynamixel->readRegister(register_name, default_value)) {
      DXL_LOG_ERROR("Could not read the limit '" << register_name << "' of the cap '" << interface_name
                                                 << "' for joint '" << name << "'.");
      return false;
    }
    if (!is_cap && !dynamixel->readRegister(register_name, default_value)) {
      // in case limit register is not available, try to read the value from the command register itself
      // after reboot (!) this should be the maximum value
      register_name = commandInterfaceToRegisterName(interface_name);
      if (!dynamixel->readRegister(register_name, default_value)) {
        DXL_LOG_ERROR("Could not read default value for interface '" << interface_name << "' for joint '" << name
                                                                     << "'.");
        return false;
      }
    }
    default_goal_values_[interface_name] = default_value;
    if (is_cap) {
      // At least one tick, since 0 lifts the cap on some series (Goal Torque of PRO, Profile Velocity)
      const double tick = dynamixel->getItem(register_name).dxlValueToUnitRatio();
      caps_.push_back(Cap{interface_name, std::min(tick, default_value), default_value});
    }
    DXL_LOG_DEBUG("Setting default goal value for interface '" << interface_name << "' for joint '" << name << "' to "
                                                               << default_value << " read from register '"
                                                               << register_name << "'.");
  }
  return true;
}

}  // namespace dynamixel_ros_control
