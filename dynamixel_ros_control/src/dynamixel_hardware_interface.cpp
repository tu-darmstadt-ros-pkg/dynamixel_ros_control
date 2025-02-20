#include "dynamixel_ros_control/common.hpp"
#include "dynamixel_ros_control/log.hpp"

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
    joints_.emplace_back(std::move(joint));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DynamixelHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  if (!driver_.connect()) {
    return hardware_interface::CallbackReturn::FAILURE;
  }

  for (Joint& j: joints_) {
    if (!j.dynamixel->connect()) {
      return hardware_interface::CallbackReturn::FAILURE;
    }
  }

  // Set up read manager (depends on requested interfaces)

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  first_read_successful_ = false;
  // turn on torque here?
  return SystemInterface::on_activate(previous_state);
}

hardware_interface::CallbackReturn
DynamixelHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
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

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> DynamixelHardwareInterface::on_export_command_interfaces()
{
  return SystemInterface::on_export_command_interfaces();
}

hardware_interface::return_type
DynamixelHardwareInterface::perform_command_mode_switch(const std::vector<std::string>& basic_strings,
                                                        const std::vector<std::string>& vector)
{
  // Set up write manager

  // write control mode
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

}  // namespace dynamixel_ros_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_ros_control::DynamixelHardwareInterface, hardware_interface::SystemInterface)
