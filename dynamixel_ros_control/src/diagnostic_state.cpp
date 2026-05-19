#include "dynamixel_ros_control/diagnostic_state.hpp"

#include "dynamixel_ros_control/dynamixel.hpp"

namespace dynamixel_ros_control {

std::string hardwareErrorToString(int32_t error_status)
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
