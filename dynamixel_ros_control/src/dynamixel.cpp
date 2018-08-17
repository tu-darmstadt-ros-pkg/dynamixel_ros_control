#include <dynamixel_ros_control/dynamixel.h>

#include <dynamixel_ros_control/common.h>
#include <boost/algorithm/string.hpp>

namespace dynamixel_ros_control {

Dynamixel::Dynamixel(uint8_t id, uint16_t model_number, DynamixelDriver& driver)
  :  driver_(driver), id_(id), model_number_(model_number) {}

bool Dynamixel::loadControlTable(const ros::NodeHandle& nh)
{
  std::string series = getSeries(nh);

  ros::NodeHandle device_nh(nh, "devices/" + series);

  // Load table
  bool success = true;
  std::vector<std::string> control_table_entries;
  if (!loadRequiredParameter(device_nh, "control_table", control_table_entries)) {
    return false;
  }
  for (const std::string& entry_str: control_table_entries) {
//    ROS_INFO_STREAM("Processing: " << entry_str);
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
    ROS_WARN_STREAM("Failed to load unit conversion ratios.");
  }

  // Load indirect addresses
  if (!loadIndirectAddresses(device_nh)) {
    ROS_WARN_STREAM("Failed to load indirect addresses.");
  }

  return success;
}

bool Dynamixel::writeRegister(std::string register_name, int32_t value) const
{
  const ControlTableItem* item;
  try {
    item = &getItem(register_name);
  } catch (const std::out_of_range&) {
    return false;
  }
  return writeRegister(item->address(), item->data_length(), value);
}

bool Dynamixel::writeRegister(uint16_t address, uint8_t data_length, int32_t value) const
{
  return driver_.writeRegister(getId(), address, data_length, value);
}

bool Dynamixel::readRegister(std::string register_name, int32_t& value_out)
{
  const ControlTableItem* item;
  try {
    item = &getItem(register_name);
  } catch (const std::out_of_range&) {
    return false;
  }
  return readRegister(item->address(), item->data_length(), value_out);
}

bool Dynamixel::readRegister(uint16_t address, uint8_t data_length, int32_t& value_out)
{
  return driver_.readRegister(getId(), address, data_length, value_out);
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

const ControlTableItem& Dynamixel::getItem(std::string& name) const
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
    ROS_ERROR_STREAM("Unknown model number " << model_number_ << ". nh: " << nh.getNamespace() << "/model_list/" << model_number_);
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
    if (kv.second.unit() != "" && kv.second.unit() != "bool") {
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

bool Dynamixel::loadIndirectAddresses(const ros::NodeHandle& nh)
{
  std::vector<std::string> lines;
  if (!loadRequiredParameter(nh, "indirect_addresses", lines)) {
    return false;
  }
  for (std::string line: lines) {
    removeWhitespace(line);
    std::vector<std::string> parts;
    boost::split(parts, line, boost::is_any_of("|"));
    if (parts.size() != 2) {
      ROS_ERROR_STREAM("Indirect address line has invalid size " << parts.size());
      continue;
    }
    IndirectAddressInfo info;
    try {
      info.indirect_address_start = static_cast<uint16_t>(std::stoi(parts[0]));
    } catch (const std::invalid_argument&) {
      ROS_ERROR_STREAM("Indirect address start '" << parts[0] << "' is not an integer.");
      continue;
    }
    try {
      info.count = static_cast<unsigned int>(std::stoi(parts[1]));
    } catch (const std::invalid_argument&) {
      ROS_ERROR_STREAM("Indirect address count '" << parts[1] << "' is not an integer.");
      continue;
    }
    info.indirect_data_start = info.indirect_address_start + static_cast<uint16_t>(2 * info.count);
    ROS_DEBUG_STREAM("Added indirect address: " << std::endl << info.toString());
    indirect_addresses_.push_back(info);
  }
  return true;
}

uint16_t Dynamixel::getModelNumber() const
{
  return model_number_;
}

bool Dynamixel::setIndirectAddress(unsigned int indirect_address_index, std::string register_name, uint16_t& indirect_data_address)
{
  uint16_t indirect_address;
  if (!indirectIndexToAddresses(indirect_address_index, indirect_address, indirect_data_address)) {
    return false;
  }
  const ControlTableItem* item;
  try {
    item = &getItem(register_name);
  } catch (const std::out_of_range&) {
    return false;
  }
  uint16_t register_address = item->address();
  uint8_t data_length = item->data_length();

  ROS_DEBUG_STREAM("[INDIRECT ADDRESS] Setting indirect address " << indirect_address << " to " << register_address << "(" << register_name <<
                   "), data_address: " << indirect_data_address);
  bool success = true;
  for (uint16_t i = 0; i < data_length; i++) {
    success &= writeRegister(indirect_address + (2*i), sizeof(register_address), register_address + i);
  }
  return success;
}

bool Dynamixel::indirectIndexToAddresses(unsigned int indirect_address_index, uint16_t& indirect_address, uint16_t& indirect_data_address)
{
  const IndirectAddressInfo* info;
  unsigned int total_count = 0;
  unsigned int local_index;
  for (const IndirectAddressInfo& i: indirect_addresses_) {
    total_count += i.count;
    if (indirect_address_index < total_count) {
      info = &i;
      local_index = indirect_address_index - (total_count - i.count);
      break;
    }
  }
  indirect_address = info->indirect_address_start + static_cast<uint16_t>(local_index * 2);
  indirect_data_address = info->indirect_data_start + static_cast<uint16_t>(local_index);
}

uint8_t Dynamixel::getId() const
{
  return id_;
}

}
