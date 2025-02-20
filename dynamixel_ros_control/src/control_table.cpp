#include "dynamixel_ros_control/log.hpp"

#include <dynamixel_ros_control/control_table.hpp>

#include <dynamixel_ros_control/common.hpp>
#include <boost/algorithm/string.hpp>

namespace dynamixel_ros_control {

bool ControlTable::loadFromYaml(const std::string& path)
{
  YAML::Node config;
  try {
    config = YAML::LoadFile(path);
  }
  catch (YAML::BadFile&) {
    DXL_LOG_ERROR("Failed to read control table at '" << path << "'. Does the file exist?");
    return false;
  }

  if (!loadIndirectAddressInfo(config["indirect_addresses"])) {
    return false;
  }
  if (!loadControlTable(config["control_table"])) {
    return false;
  }
  if (!loadUnitConversions(config["unit_conversions"])) {
    return false;
  }
  assignRatiosToControlTableEntries();
  return true;
}

bool ControlTable::itemAvailable(const std::string& name) const
{
  return control_table_.find(name) != control_table_.end();
}

const ControlTableItem& ControlTable::getItem(const std::string& name) const
{
  try {
    return control_table_.at(name);
  }
  catch (const std::out_of_range&) {
    DXL_LOG_ERROR("Could not find register '" << name << "'.");
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
    DXL_LOG_ERROR("Indirect address info is not a sequence.");
    return false;
  }
  auto lines = node.as<std::vector<std::string>>();
  for (std::string line : lines) {
    removeWhitespace(line);
    std::vector<std::string> parts;
    boost::split(parts, line, boost::is_any_of("|"));
    if (parts.size() != 3) {
      DXL_LOG_ERROR("Indirect address line has invalid size " << parts.size());
      continue;
    }
    IndirectAddressInfo info{};
    // Load address start
    try {
      info.indirect_address_start = static_cast<uint16_t>(std::stoi(parts[0]));
    }
    catch (const std::invalid_argument&) {
      DXL_LOG_ERROR("Indirect address start '" << parts[0] << "' is not an integer.");
      continue;
    }
    // Load data start
    try {
      info.indirect_data_start = static_cast<uint16_t>(std::stoi(parts[1]));
    }
    catch (const std::invalid_argument&) {
      DXL_LOG_ERROR("Indirect data start '" << parts[1] << "' is not an integer.");
      continue;
    }
    // Load count
    try {
      info.count = static_cast<unsigned int>(std::stoi(parts[2]));
    }
    catch (const std::invalid_argument&) {
      DXL_LOG_ERROR("Indirect address count '" << parts[2] << "' is not an integer.");
      continue;
    }
    DXL_LOG_DEBUG("Added indirect address: " << std::endl << info.toString());
    indirect_addresses_.push_back(info);
  }
  return true;
}

bool ControlTable::loadControlTable(const YAML::Node& node)
{
  if (!node.IsSequence()) {
    DXL_LOG_ERROR("Control table is not a sequence.");
    return false;
  }
  auto control_table_entries = node.as<std::vector<std::string>>();
  bool success = true;
  for (const std::string& entry_str : control_table_entries) {
    ControlTableItem entry;
    if (entry.loadFromString(entry_str)) {
      control_table_.emplace(entry.name(), entry);  // TODO prevent copy?
    } else {
      DXL_LOG_ERROR("Failed to load control table entry '" << entry_str << "'.");
      success = false;
    }
  }
  return success;
}

bool ControlTable::loadUnitConversions(const YAML::Node& node)
{
  if (!node.IsMap()) {
    DXL_LOG_ERROR("Unit conversions is not a map.");
    return false;
  }
  for (auto it = node.begin(); it != node.end(); ++it) {
    auto unit_name = it->first.as<std::string>();
    if (it->second.IsScalar()) {
      auto ratio = it->second.as<double>();
      unit_to_ratio.emplace(unit_name, ratio);
    } else {
      DXL_LOG_ERROR("Bad type in unit conversion value. Expected scalar, received " << it->second.Type());
      return false;
    }
  }
  return true;
}

void ControlTable::assignRatiosToControlTableEntries()
{
  for (auto& [name, item] : control_table_) {
    if (!item.unit().empty() && item.unit() != "bool") {
      double ratio = 1.0;
      try {
        ratio = unit_to_ratio.at(item.unit());
      }
      catch (const std::out_of_range&) {
        DXL_LOG_ERROR("Undefined unit '" << item.unit() << "' in control table entry '" << name << "'.");
      }
      item.setDxlValueToUnitRatio(ratio);
    }
  }
}

}  // namespace dynamixel_ros_control
