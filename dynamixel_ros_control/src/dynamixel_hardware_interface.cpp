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

  // Load joints

  // Initialize driver
  std::string port_name = info_.hardware_parameters.at("port_name");

  int baud_rate;
  std::string baud_rate_string = info_.hardware_parameters.at("baud_rate"); //TODO wrapper for loading params
  try {
    baud_rate = std::stoi(baud_rate_string);
  } catch (const std::exception& e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error reading baud rate '" << baud_rate_string << '');
    return hardware_interface::CallbackReturn::FAILURE; //TODO check error levels
  }
  driver_.init(port_name, baud_rate);

  // Connect

  return hardware_interface::CallbackReturn::SUCCESS;
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
