#include "dynamixel_ros_control/common.hpp"
#include "dynamixel_ros_control/log.hpp"

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <dynamixel_ros_control/dynamixel_hardware_interface.hpp>

namespace dynamixel_ros_control {

hardware_interface::CallbackReturn
DynamixelHardwareInterface::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
  // Load hardware configuration
  const auto result = SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  // Load parameters
  std::string port_name;
  if (!getParameter<std::string>(info_.hardware_parameters, "port_name", port_name)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  int baud_rate;
  if (!getParameter<int>(info_.hardware_parameters, "baud_rate", baud_rate)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  getParameter(info_.hardware_parameters, "debug", debug_, false);
  if (debug_) {
    rclcpp::get_logger(DXL_LOGGER_NAME).set_level(rclcpp::Logger::Level::Debug);
  }
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_init");
  getParameter(info_.hardware_parameters, "torque_on_startup", torque_on_startup_, false);
  getParameter(info_.hardware_parameters, "torque_off_on_shutdown", torque_off_on_shutdown_, false);

  // Initialize driver
  if (!driver_.init(port_name, baud_rate)) {
    DXL_LOG_ERROR("Failed to initialize driver");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Load joints
  joints_.reserve(info_.joints.size());
  for (const auto& joint_info : info_.joints) {
    Joint joint;
    if (!joint.loadConfiguration(driver_, joint_info)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::stringstream ss;
    ss << "Loaded dynamixel:" << std::endl;
    ss << "-- name: " << joint.name << std::endl;
    ss << "-- id: " << static_cast<int>(joint.dynamixel->getId()) << std::endl;
    ss << "-- mounting_offset: " << joint.mounting_offset << std::endl;
    ss << "-- offset: " << joint.offset << std::endl;
    DXL_LOG_DEBUG(ss.str());
    joints_.emplace(joint.name, std::move(joint));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DynamixelHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_configure from " << previous_state.label());
  // TODO temporarily disabled for testing without dynamixel
  // if (!driver_.connect()) {
  //   return hardware_interface::CallbackReturn::FAILURE;
  // }
  //
  // for (Joint& j : joints_) {
  //   if (!j.dynamixel->connect()) {
  //     return hardware_interface::CallbackReturn::FAILURE;
  //   }
  // }

  // Set up read manager (depends on requested interfaces)

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_cleanup from " << previous_state.label());
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_activate from " << previous_state.label());
  first_read_successful_ = false;
  // turn on torque here?
  return SystemInterface::on_activate(previous_state);
}

hardware_interface::CallbackReturn
DynamixelHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_deactivate from " << previous_state.label());
  // disable torque here?
  // if (torque_off_on_shutdown_)
  // {
  //   std::cout << "Disabling torque on shutdown!" << std::endl;
  //   if (connected_) setTorque(false);
  // }
  return SystemInterface::on_deactivate(previous_state);
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr> DynamixelHardwareInterface::on_export_state_interfaces()
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_export_state_interfaces");
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

  // Read all requested fields from all motors
  std::set<std::string> requested_state_interface_names;
  for (const auto& [name, joint] : joints_) {
    requested_state_interface_names.insert(joint.getAvailableStateInterfaces().begin(),
                                           joint.getAvailableStateInterfaces().end());
  }
  // Create the state interfaces
  for (auto& [name, joint] : joints_) {
    joint.current_state.reserve(requested_state_interface_names.size());
    for (const auto& interface_name : requested_state_interface_names) {
      joint.current_state[interface_name] = 0.0;
      const auto state_interface = std::make_shared<hardware_interface::StateInterface>(
          joint.name, interface_name, &joint.current_state[interface_name]);
      state_interfaces.emplace_back(state_interface);
    }
  }
  DXL_LOG_DEBUG("State interfaces: " << iterableToString(requested_state_interface_names));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> DynamixelHardwareInterface::on_export_command_interfaces()
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_export_command_interfaces");
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

  // Read all requested fields from all motors
  std::set<std::string> requested_command_interface_names;
  for (const auto& [name, joint] : joints_) {
    requested_command_interface_names.insert(joint.getAvailableCommandInterfaces().begin(),
                                             joint.getAvailableCommandInterfaces().end());
  }
  // Create the state interfaces
  for (auto& [name, joint] : joints_) {
    joint.goal_state.reserve(requested_command_interface_names.size());
    for (const auto& interface_name : requested_command_interface_names) {
      joint.goal_state[interface_name] = 0.0;
      const auto command_interface = std::make_shared<hardware_interface::CommandInterface>(
          joint.name, interface_name, &joint.goal_state[interface_name]);
      command_interfaces.emplace_back(command_interface);
    }
  }
  DXL_LOG_DEBUG("Command interfaces: " << iterableToString(requested_command_interface_names));
  return command_interfaces;
}

hardware_interface::return_type
DynamixelHardwareInterface::perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                        const std::vector<std::string>& stop_interfaces)
{
  // Set up write manager
  DXL_LOG_DEBUG("DynamixelHardwareInterface::perform_command_mode_switch");
  DXL_LOG_DEBUG("start_interfaces: " << iterableToString(start_interfaces));
  DXL_LOG_DEBUG("stop_interfaces: " << iterableToString(stop_interfaces));

  // Start interfaces
  for (const auto& full_interface_name : start_interfaces) {
    std::vector<std::string> parts;
    boost::split(parts, full_interface_name, boost::is_any_of("/"));
    if (parts.size() != 2) {
      DXL_LOG_ERROR("Invalid interface name: " << full_interface_name);
      return hardware_interface::return_type::ERROR;
    }
    const std::string joint_name = parts[0];
    const std::string interface_name = parts[1];
    try {
      if (joints_.at(joint_name).addActiveCommandInterface(interface_name)) {
        return hardware_interface::return_type::ERROR;
      }
    }
    catch (std::out_of_range& e) {
      DXL_LOG_ERROR("Unknown joint name: " << joint_name);
      return hardware_interface::return_type::ERROR;
    }
  }

  // write control mode
  return SystemInterface::perform_command_mode_switch(start_interfaces, stop_interfaces);
}

hardware_interface::CallbackReturn DynamixelHardwareInterface::on_error(const rclcpp_lifecycle::State& previous_state)
{
  DXL_LOG_DEBUG("DynamixelHardwareInterface::on_error from " << previous_state.label());
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DynamixelHardwareInterface::read(const rclcpp::Time& time,
                                                                 const rclcpp::Duration& period)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardwareInterface::write(const rclcpp::Time& time,
                                                                  const rclcpp::Duration& period)
{
  return hardware_interface::return_type::OK;
}

}  // namespace dynamixel_ros_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_ros_control::DynamixelHardwareInterface, hardware_interface::SystemInterface)
