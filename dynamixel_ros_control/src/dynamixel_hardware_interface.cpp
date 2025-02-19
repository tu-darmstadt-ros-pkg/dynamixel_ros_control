#include <dynamixel_ros_control/dynamixel_hardware_interface.hpp>

namespace dynamixel_ros_control {

hardware_interface::CallbackReturn
DynamixelHardwareInterface::on_init(const hardware_interface::HardwareInfo& hardware_info)
{
  // Load hardware configuration
  auto result = SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS) {
    return result;
  }

  // Load parameters
  std::string port_name;
  if (!getParameter<std::string>("port_name", port_name)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  int baud_rate;
  if (!getParameter<int>("baud_rate", baud_rate)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize driver
  if (!driver_.init(port_name, baud_rate)) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize driver");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Load joints

  // Connect

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DynamixelHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return SystemInterface::on_activate(previous_state);
}

hardware_interface::CallbackReturn
DynamixelHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return SystemInterface::on_deactivate(previous_state);
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr> DynamixelHardwareInterface::on_export_state_interfaces()
{
  return SystemInterface::on_export_state_interfaces();
}

std::vector<hardware_interface::CommandInterface::SharedPtr> DynamixelHardwareInterface::on_export_command_interfaces()
{
  return SystemInterface::on_export_command_interfaces();
}

hardware_interface::return_type
DynamixelHardwareInterface::perform_command_mode_switch(const std::vector<std::string>& basic_strings,
                                                        const std::vector<std::string>& vector)
{
  return SystemInterface::perform_command_mode_switch(basic_strings, vector);
}

hardware_interface::CallbackReturn DynamixelHardwareInterface::on_error(const rclcpp_lifecycle::State& previous_state)
{
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

bool DynamixelHardwareInterface::getParameterAsString(const std::string& param_name, std::string& value) const
{
  try {
    value = info_.hardware_parameters.at(param_name);
  }
  catch (const std::out_of_range& e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Parameter '" << param_name << "' does not exist");
    return false;
  }
  return true;
}

template <>
bool DynamixelHardwareInterface::getParameter<double>(const std::string& param_name, double& value)
{
  std::string parameter_string;
  if (!getParameterAsString(param_name, parameter_string)) {
    return false;
  }

  try {
    value = std::stod(parameter_string);
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Parameter '" << param_name << "' is not a double.");
    return false;
  }
  return true;
}

template <>
bool DynamixelHardwareInterface::getParameter<int>(const std::string& param_name, int& value)
{
  std::string parameter_string;
  if (!getParameterAsString(param_name, parameter_string)) {
    return false;
  }

  try {
    value = std::stoi(parameter_string);
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Parameter '" << param_name << "' is not an integer.");
    return false;
  }
  return true;
}

template <>
bool DynamixelHardwareInterface::getParameter<std::string>(const std::string& param_name, std::string& value)
{
  return getParameterAsString(param_name, value);
}

}  // namespace dynamixel_ros_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_ros_control::DynamixelHardwareInterface, hardware_interface::SystemInterface)
