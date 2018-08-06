#include <dynamixel_ros_control/dynamixel.h>

#include <dynamixel_ros_control/common.h>

namespace dynamixel_ros_control {

Dynamixel::Dynamixel(uint8_t id, uint16_t model_number)
  : id_(id), model_number_(model_number){}

bool Dynamixel::loadControlTable(const ros::NodeHandle& nh)
{
  std::string series = getSeries(nh);

  ros::NodeHandle device_nh(nh, "devices/" + series);

  // Load table
  bool success = true;
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

  // Load unit conversion ratios
  if (!loadUnitConversionRatios(device_nh)) {
    ROS_ERROR_STREAM("Failed to load unit conversion ratios.");
    return false;
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
  } catch (const std::out_of_range&) {
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

bool Dynamixel::loadUnitConversionRatios(const ros::NodeHandle& nh)
{
  XmlRpc::XmlRpcValue units;
  nh.getParam("unit_conversions", units);
  if (units.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    return false;
  }
  std::map<std::string, double> unit_to_ratio;
  for(XmlRpc::XmlRpcValue::ValueStruct::iterator it = units.begin(); it != units.end(); ++it)
  {
    std::string unit_name = static_cast<std::string>(it->first);
    if (it->second.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
      ROS_ERROR_STREAM("Ratio of unit '" << unit_name << "_ is not of type double.");
      return false;
    }
    double ratio = static_cast<double>(it->second);
    unit_to_ratio.emplace(unit_name, ratio);
  }

  // Assign ratios to control table entries
  for (std::map<std::string, ControlTableItem>::value_type& kv: control_table_) {
    if (kv.second.unit() != "") {
      double ratio = 1.0;
      try {
        ratio = unit_to_ratio.at(kv.second.unit());
      } catch (const std::out_of_range&) {
        ROS_ERROR_STREAM("Undefined unit '" << kv.second.unit() << "' in control table entry '" << kv.first << "'.");
      }
      kv.second.setDxlValueToUnitRatio(ratio);
    }
  }
  return true;
}

uint16_t Dynamixel::getModelNumber() const
{
  return model_number_;
}

bool Dynamixel::setIndirectAddress(unsigned int indirect_address_start, std::string register_name)
{

}

uint8_t Dynamixel::getId() const
{
  return id_;
}

}
