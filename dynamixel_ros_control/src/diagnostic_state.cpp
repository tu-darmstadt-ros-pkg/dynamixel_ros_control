#include "dynamixel_ros_control/diagnostic_state.hpp"

#include "dynamixel_ros_control/dynamixel.hpp"

namespace dynamixel_ros_control {

diagnostic_msgs::msg::DiagnosticStatus DiagnosticState::toMsg(const std::string& name) const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.hardware_id = name;

  if (!hw_ok) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "Hardware error detected";
  } else if (e_stop_active) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "E-Stop active";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "OK";
  }

  status.values.resize(4 + joint_hw_errors.size());
  status.values[0].key = "e_stop_active";
  status.values[0].value = e_stop_active ? "true" : "false";
  status.values[1].key = "torque_enabled";
  status.values[1].value = torque_enabled ? "true" : "false";
  status.values[2].key = "consecutive_read_errors";
  status.values[2].value = std::to_string(read_errors);
  status.values[3].key = "consecutive_write_errors";
  status.values[3].value = std::to_string(write_errors);

  size_t i = 4;
  for (const auto& [joint_name, error_status] : joint_hw_errors) {
    status.values[i].key = joint_name + "/hardware_error";
    status.values[i].value = hardwareErrorToString(error_status);
    ++i;
  }

  return status;
}

std::string DiagnosticState::hardwareErrorToString(int32_t error_status)
{
  if (error_status == OK) {
    return "ok";
  }
  std::string result;
  if (error_status & VOLTAGE_ERROR)
    result += "Voltage, ";
  if (error_status & HALL_SENSOR_ERROR)
    result += "Hall Sensor, ";
  if (error_status & OVERHEATING_ERROR)
    result += "Overheating, ";
  if (error_status & MOTOR_ENCODER_ERROR)
    result += "Motor Encoder, ";
  if (error_status & ELECTRICAL_SHOCK_ERROR)
    result += "Electrical Shock, ";
  if (error_status & OVERLOAD_ERROR)
    result += "Overload, ";
  if (result.size() >= 2) {
    result.erase(result.size() - 2);
  }
  return result;
}

}  // namespace dynamixel_ros_control
