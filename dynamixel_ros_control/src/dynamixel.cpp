#include <dynamixel_ros_control/dynamixel.h>

#include <dynamixel_ros_control/common.h>

namespace dynamixel_ros_control {

Dynamixel::Dynamixel(uint8_t id, uint16_t model_number)
  : id_(id), model_number_(model_number){}

bool Dynamixel::loadControlTable(const ros::NodeHandle& nh)
{
  std::string series = getSeries(nh);

  ros::NodeHandle device_nh(nh, "devices/" + series);

  // Load info
  bool success = true;
  success &= loadRequiredParameter(device_nh, "velocity_to_value_ratio", velocity_to_value_ratio_);
  success &= loadRequiredParameter(device_nh, "torque_to_current_value_ratio", torque_to_value_ratio_);
  int ticks_per_rev_int;
  success &= loadRequiredParameter(device_nh, "ticks_per_revolution", ticks_per_rev_int);
  ticks_per_revolution_ = static_cast<double>(ticks_per_rev_int);

  // Load table
  std::vector<std::string> control_table_entries;
  device_nh.getParam("/control_table", control_table_entries);
  for (const std::string& entry_str: control_table_entries) {
    ControlTableItem entry;
    if (entry.loadFromString(entry_str)) {
      control_table_.emplace(entry.name(), entry); // TODO prevent copy?
    } else {
      ROS_ERROR_STREAM("Failed to load control table entry '" << entry_str << "'.");
      success = false;
    }
  }
  return success;
}

double Dynamixel::dxlValueToUnit(std::string register_name, int32_t value)
{
  std::string unit = getItem(register_name).unit();
  if (unit == "rad") {
    return dxlValuetoPosition(value);
  }
}

bool Dynamixel::dxlValueToBool(std::string register_name, int32_t value)
{
  std::string unit = getItem(register_name).unit();
  if (unit == "bool") {
    return static_cast<bool>(value);
  } else {
    ROS_ERROR_STREAM("Unknown bool type: '" << unit << "'");
    return false;
  }
}

double Dynamixel::dxlValuetoPosition(int32_t dxl_value)
{
  return (static_cast<double>(dxl_value) / ticks_per_revolution_ * 2 * M_PI);
}

int32_t Dynamixel::positionToDxlValue(double rad)
{

}

double Dynamixel::dxlValueToVelocity(int32_t dxl_value)
{
  return static_cast<double>(dxl_value) / velocity_to_value_ratio_;
}

int32_t Dynamixel::velocityToDxlValue(double velocity)
{
  return static_cast<int32_t>(velocity_to_value_ratio_ * velocity);
}

double Dynamixel::dxlValueToTorque(int32_t dxl_value)
{
  return static_cast<double>(dxl_value) / torque_to_value_ratio_;
}

int32_t Dynamixel::torqueToDxlValue(double torque)
{

}

const ControlTableItem& Dynamixel::getItem(std::string& name)
{
  try {
    return control_table_.at(name);
  } catch (const std::out_of_range& e) {
    ROS_ERROR_STREAM("Could not find register '" << name << "'.");
    throw;
  }

}

std::string Dynamixel::getSeries(const ros::NodeHandle& nh)
{
  std::string series;
  if (!nh.getParam("model_list/" + std::to_string(model_number_), series)) {
    ROS_ERROR_STREAM("Unknown model number " << model_number_ << ".");
    return "";
  } else {
    return series;
  }
}

uint16_t Dynamixel::getModelNumber() const
{
  return model_number_;
}

uint8_t Dynamixel::getId() const
{
  return id_;
}

}
