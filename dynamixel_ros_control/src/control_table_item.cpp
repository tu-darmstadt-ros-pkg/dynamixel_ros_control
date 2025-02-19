#include <dynamixel_ros_control/control_table_item.hpp>

#include <boost/algorithm/string.hpp>
#include <dynamixel_ros_control/common.hpp>
#include <utility>

namespace dynamixel_ros_control {

ControlTableItem::ControlTableItem(std::string name, const uint16_t address, const uint8_t data_length,
                                   const AccessType access_type, const MemoryType memory_type)
  : name_(std::move(name)),
    address_(address),
    data_length_(data_length),
    access_type_(access_type),
    memory_type_(memory_type),
    dxl_value_to_unit_ratio_(1.0) {}

ControlTableItem::ControlTableItem()
  : ControlTableItem("unset", 0, 0, READ, RAM) {}

bool ControlTableItem::loadFromString(std::string control_table_string)
{
  removeWhitespace(control_table_string);
  std::vector<std::string> parts;
  boost::split(parts, control_table_string, boost::is_any_of("|"));
  if (parts.size() != 6) {
    // ROS_ERROR("Control table item has invalid size (%lu), expected (6)", parts.size());
    return false;
  }

  // load data
  try {
    address_ = static_cast<uint16_t>(std::stoi(parts[0]));
  } catch (const std::invalid_argument&) {
    // ROS_ERROR_STREAM("Failed to read address '" << parts[0] << "'.");
    return false;
  }
  name_ = parts[1];
  try {
    data_length_ = static_cast<uint8_t>(std::stoi(parts[2]));
  } catch (const std::invalid_argument&) {
    // ROS_ERROR_STREAM("Failed to read data length '" << parts[2] << "'.");
    return false;
  }
  if (!stringToAccessType(parts[3], access_type_)) {
    return false;
  }
  if (!stringToMemoryType(parts[4], memory_type_)) {
    return false;
  }
  unit_ = parts[5];
  return true;
}

std::string ControlTableItem::name() const
{
  return name_;
}

uint16_t ControlTableItem::address() const
{
  return address_;
}

AccessType ControlTableItem::access_type() const
{
  return access_type_;
}

MemoryType ControlTableItem::memory_type() const
{
  return memory_type_;
}

void ControlTableItem::setDxlValueToUnitRatio(double dxl_value_to_unit_ratio)
{
  dxl_value_to_unit_ratio_ = dxl_value_to_unit_ratio;
}

double ControlTableItem::dxlValueToUnitRatio() const
{
  return dxl_value_to_unit_ratio_;
}

std::string ControlTableItem::unit() const
{
    return unit_;
}

uint8_t ControlTableItem::data_length() const
{
  return data_length_;
}

bool stringToAccessType(const std::string& str, AccessType& access_type)
{
  if (str == "R") {
    access_type = READ;
    return true;
  }
  if (str == "RW") {
    access_type = READ_WRITE;
    return true;
  }
  // ROS_ERROR_STREAM("Unknown access type '" << str << "'.");
  return false;
}

bool stringToMemoryType(const std::string& str, MemoryType& memory_type)
{
  if (str == "RAM") {
    memory_type = RAM;
    return true;
  }
  if (str == "EEPROM") {
    memory_type = EEPROM;
    return true;
  }
  // ROS_ERROR_STREAM("Unknown memory type '" << str << "'.");
  return false;
}

}
