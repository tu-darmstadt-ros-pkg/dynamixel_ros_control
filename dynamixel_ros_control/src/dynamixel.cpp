#include <dynamixel_ros_control/dynamixel.h>

#include <dynamixel_ros_control/common.h>

namespace dynamixel_ros_control {

Dynamixel::Dynamixel(uint8_t id, uint16_t model_number)
  : id_(id), model_number_(model_number){}

bool Dynamixel::loadControlTable(const ros::NodeHandle& nh)
{
  std::string series = getSeries(nh);

  ros::NodeHandle device_nh(nh, "devices/" + series);

  bool success = true;
  // TODO load unit conversion ratios

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
  return static_cast<double>(value) * getItem(register_name).dxlValueToUnitRatio();
}

bool Dynamixel::dxlValueToBool(std::string register_name, int32_t value)
{
  std::string unit = getItem(register_name).unit();
  if (unit == "bool") {
    return value > 0;
  } else {
    ROS_ERROR_STREAM("The register '" << register_name << "' (type=" << unit << ") is not of type bool");
    return false;
  }
}

int32_t Dynamixel::unitToDxlValue(std::string register_name, double unit_value)
{
  return static_cast<int32_t>(unit_value / getItem(register_name).dxlValueToUnitRatio());
}

int32_t Dynamixel::boolToDxlValue(std::string register_name, bool b)
{
  std::string unit = getItem(register_name).unit();
  if (unit == "bool") {
    return static_cast<int32_t>(b);
  } else {
    ROS_ERROR_STREAM("The register '" << register_name << "' (type=" << unit << ") is not of type bool");
    return false;
  }
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

std::string Dynamixel::getSeries(const ros::NodeHandle& nh) const
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
