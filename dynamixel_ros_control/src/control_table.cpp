#include "dynamixel_ros_control/log.hpp"

#include <dynamixel_ros_control/control_table.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <dynamixel_ros_control/common.hpp>
#include <dynamixel_ros_control/dynamixel.hpp>
#include <boost/algorithm/string.hpp>

namespace dynamixel_ros_control {

bool ControlTable::loadFromYaml(const std::string& path)
{
  // Every YAML conversion below throws on a bad type, and an escaped one would read as a plugin that
  // would not load rather than as the file it is
  try {
    return load(path);
  }
  catch (const YAML::Exception& e) {
    DXL_LOG_ERROR("Failed to read control table at '" << path << "': " << e.what());
    return false;
  }
}

bool ControlTable::load(const std::string& path)
{
  const YAML::Node config = YAML::LoadFile(path);

  if (!loadIndirectAddressInfo(config["indirect_addresses"])) {
    return false;
  }
  if (!loadControlTable(config["control_table"])) {
    return false;
  }
  if (!loadUnitConversions(config["unit_conversions"])) {
    return false;
  }
  if (!assignRatiosToControlTableEntries() || !loadCurrentCapModes(config["current_cap_modes"])) {
    return false;
  }
  return validateAddressRanges();
}

bool ControlTable::itemAvailable(const std::string& name) const
{
  return control_table_.find(name) != control_table_.end();
}

const ControlTableItem* ControlTable::findItem(const std::string& name) const
{
  const auto it = control_table_.find(name);
  return it != control_table_.end() ? &it->second : nullptr;
}

const ControlTableItem& ControlTable::getItem(const std::string& name) const
{
  const auto it = control_table_.find(name);
  if (it == control_table_.end()) {
    // Named, because map::at's own out_of_range says only "map::at". Nothing in this package
    // catches this: it means a model's YAML lacks a register the code requires.
    throw std::out_of_range("No register '" + name + "' in this model's control table.");
  }
  return it->second;
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
    if (parts.size() != 4) {
      DXL_LOG_ERROR("Indirect address line has invalid size " << parts.size() << " (expected 4: address start | "
                                                              << "data start | count | memory).");
      return false;
    }
    IndirectAddressInfo info{};
    if (!parseIntegerInto(parts[0], info.indirect_address_start)) {
      DXL_LOG_ERROR("Indirect address start '" << parts[0] << "' is not an integer in range.");
      return false;
    }
    if (!parseIntegerInto(parts[1], info.indirect_data_start)) {
      DXL_LOG_ERROR("Indirect data start '" << parts[1] << "' is not an integer in range.");
      return false;
    }
    if (!parseIntegerInto(parts[2], info.count)) {
      DXL_LOG_ERROR("Indirect address count '" << parts[2] << "' is not an integer in range.");
      return false;
    }
    // Load memory area of the pointer registers (EEPROM survives reboot, RAM does not).
    // Whitespace was already stripped from the whole line above, so parts[3] is clean.
    if (!stringToMemoryType(parts[3], info.memory_type)) {
      DXL_LOG_ERROR("Indirect address memory type '" << parts[3] << "' is invalid (expected RAM or EEPROM).");
      return false;
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
      const std::string name = entry.name();
      // emplace keeps the first of a repeated key, so a second row under the same name would be
      // dropped without a word and the table would silently describe a different register.
      if (!control_table_.emplace(name, entry).second) {  // TODO: prevent copy
        DXL_LOG_ERROR("Control table lists the register '" << name << "' more than once.");
        success = false;
      }
    } else {
      DXL_LOG_ERROR("Failed to load control table entry '" << entry_str << "'.");
      success = false;
    }
  }
  return success;
}

bool ControlTable::loadCurrentCapModes(const YAML::Node& node)
{
  // Absent for a model nobody checked, which then refuses max_current in every mode
  if (!node) {
    return true;
  }
  for (const auto& mode : node.as<std::vector<std::string>>()) {
    try {
      current_cap_modes_.insert(controlModeToString(stringToControlMode(mode)));
    }
    catch (const std::invalid_argument& e) {
      DXL_LOG_ERROR("current_cap_modes lists '" << mode << "': " << e.what());
      return false;
    }
  }
  return true;
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
      const auto ratio = it->second.as<double>();
      // A ratio divides on the way to the motor and multiplies on the way back. Zero or non-finite
      // makes every value of that unit convert to a raw 0, and a negative one silently reverses the
      // register's direction. Neither shows up as a failure anywhere later.
      if (!std::isfinite(ratio) || ratio <= 0.0) {
        DXL_LOG_ERROR("Unit '" << unit_name << "' has conversion ratio " << ratio
                               << ". A ratio must be finite and greater than zero.");
        return false;
      }
      unit_to_ratio.emplace(unit_name, ratio);
    } else {
      DXL_LOG_ERROR("Bad type in unit conversion value. Expected scalar, received " << it->second.Type());
      return false;
    }
  }
  return true;
}

bool ControlTable::assignRatiosToControlTableEntries()
{
  bool success = true;
  for (auto& [name, item] : control_table_) {
    if (!item.unit().empty() && item.unit() != "bool") {
      const auto it = unit_to_ratio.find(item.unit());
      if (it == unit_to_ratio.end()) {
        // Falling back to 1.0 would send the register's value through unconverted, which for a
        // position or current limit is a plausible number in the wrong scale.
        DXL_LOG_ERROR("Undefined unit '" << item.unit() << "' in control table entry '" << name
                                         << "'. Declare it under unit_conversions.");
        success = false;
        continue;
      }
      item.setDxlValueToUnitRatio(it->second);
    }
  }
  return success;
}

bool ControlTable::validateAddressRanges() const
{
  // Registers are keyed by name, so two rows pointing at one address are not a duplicate key and
  // load cleanly. The overlap is the only sign that an address was mistyped.
  std::vector<std::pair<uint16_t, const ControlTableItem*>> by_address;
  by_address.reserve(control_table_.size());
  for (const auto& [name, item] : control_table_) {
    by_address.emplace_back(item.address(), &item);
  }
  std::sort(by_address.begin(), by_address.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

  bool success = true;
  for (size_t i = 1; i < by_address.size(); ++i) {
    const ControlTableItem& previous = *by_address[i - 1].second;
    const ControlTableItem& current = *by_address[i].second;
    const unsigned int previous_end = previous.address() + previous.data_length();
    if (current.address() < previous_end) {
      DXL_LOG_ERROR("Control table registers overlap: '"
                    << previous.name() << "' covers [" << previous.address() << ", " << previous_end << ") and '"
                    << current.name() << "' starts at " << current.address() << ". One of the two addresses is wrong.");
      success = false;
    }
  }
  return success;
}

}  // namespace dynamixel_ros_control
