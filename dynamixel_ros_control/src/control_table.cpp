#include <dynamixel_ros_control/control_table.h>

#include <dynamixel_ros_control/common.h>
#include <boost/algorithm/string.hpp>

namespace dynamixel_ros_control {

bool ControlTable::loadFromYaml(const std::string& path)
{
  YAML::Node config = YAML::LoadFile(path); // TODO check if file exists
  if (!loadIndirectAddressInfo(config["indirect_addresses"])) {
    return false;
  }
  if (!loadControTable(config["control_table"])) {
    return false;
  }
  if (!loadUnitConversions(config["unit_conversions"])) {
    return false;
  }
  assignRatiosToControlTableEntries();
  return true;
}

const ControlTableItem& ControlTable::getItem(std::string& name) const
{
  try {
    return control_table_.at(name);
  } catch (const std::out_of_range&) {
    ROS_ERROR_STREAM("Could not find register '" << name << "'.");
    throw;
  }
}

const std::vector<IndirectAddressInfo>& ControlTable::getIndirectAddressInfo()
{
  return indirect_addresses_;
}

bool ControlTable::loadIndirectAddressInfo(const YAML::Node& node)
{
  if (!node.IsSequence()) {
    ROS_ERROR_STREAM("Indirect address info is not a sequence.");
    return false;
  }
  std::vector<std::string> lines = node.as<std::vector<std::string>>();
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

bool ControlTable::loadControTable(const YAML::Node& node)
{
  if (!node.IsSequence()) {
    ROS_ERROR_STREAM("Control table is not a sequence.");
    return false;
  }
  std::vector<std::string> control_table_entries = node.as<std::vector<std::string>>();
  bool success = true;
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

bool ControlTable::loadUnitConversions(const YAML::Node& node)
{
  if (!node.IsMap()) {
    ROS_ERROR_STREAM("Unit conversions is not a map.");
    return false;
  }
  for(YAML::const_iterator it = node.begin();it != node.end(); ++it) {
    // TODO check types
    std::string unit_name = it->first.as<std::string>();
    double ratio = it->second.as<double>();
    unit_to_ratio.emplace(unit_name, ratio);
  }
  return true;
}

void ControlTable::assignRatiosToControlTableEntries()
{
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
}

}
