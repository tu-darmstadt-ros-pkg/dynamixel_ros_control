#include <cstdint>
#include <limits>
#include <stdexcept>
#include <dynamixel_ros_control/common.hpp>

namespace dynamixel_ros_control {

template <>
bool getParameter<double>(const ParameterMap& map, const std::string& param_name, double& value)
{
  std::string parameter_string;
  if (!getParameter<std::string>(map, param_name, parameter_string)) {
    return false;
  }

  try {
    value = std::stod(parameter_string);
  }
  catch (const std::exception& e) {
    // RCLCPP_ERROR_STREAM(get_logger(), "Failed to convert parameter '" << param_name << "' to a double: " << e.what());
    return false;
  }
  return true;
}

template <>
bool getParameter<int>(const ParameterMap& map, const std::string& param_name, int& value)
{
  std::string parameter_string;
  if (!getParameter<std::string>(map, param_name, parameter_string)) {
    return false;
  }

  try {
    value = std::stoi(parameter_string);
  }
  catch (const std::exception& e) {
    // RCLCPP_ERROR_STREAM(get_logger(), "Failed to convert parameter '" << param_name << "' to an integer: " << e.what());
    return false;
  }
  return true;
}

template <>
bool getParameter<uint8_t>(const ParameterMap& map, const std::string& param_name, uint8_t& value)
{
  int param_int;
  if (!getParameter<int>(map, param_name, param_int)) {
    return false;
  }

  if (param_int < std::numeric_limits<uint8_t>::min() || param_int > std::numeric_limits<uint8_t>::max()) {
    // RCLCPP_ERROR_STREAM(get_logger(), "Failed to convert parameter '" << param_name << "' to an uint8: Exceeded bounds.");
    return false;
  }
  value = param_int;
  return true;
}

template <>
bool getParameter<std::string>(const ParameterMap& map, const std::string& param_name, std::string& value)
{
  try {
    value = map.at(param_name);
  }
  catch (const std::out_of_range& e) {
    // RCLCPP_ERROR_STREAM(get_logger(), "Parameter '" << param_name << "' does not exist");
    return false;
  }
  return true;
}

}  // namespace dynamixel_ros_control