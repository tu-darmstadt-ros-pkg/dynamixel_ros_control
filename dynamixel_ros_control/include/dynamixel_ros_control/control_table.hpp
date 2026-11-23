#ifndef DYNAMIXEL_ROS_CONTROL_CONTROL_TABLE_H
#define DYNAMIXEL_ROS_CONTROL_CONTROL_TABLE_H

#include <optional>
#include <set>
#include <yaml-cpp/yaml.h>
#include <dynamixel_ros_control/control_table_item.hpp>

namespace dynamixel_ros_control {

struct IndirectAddressInfo
{
  uint16_t indirect_address_start;
  unsigned int count;
  uint16_t indirect_data_start;
  MemoryType memory_type;

  [[nodiscard]] std::string toString() const
  {
    return " --- Indirect address start: " + std::to_string(indirect_address_start) + "\n" +
           " --- Count: " + std::to_string(count) + "\n" +
           " --- Indirect data start: " + std::to_string(indirect_data_start) + "\n" +
           " --- Memory: " + (memory_type == RAM ? "RAM" : "EEPROM");
  }
};

class ControlTable
{
public:
  bool loadFromYaml(const std::string& path);
  [[nodiscard]] bool itemAvailable(const std::string& name) const;
  /// @brief The item, or nullptr if this model has no such register. For callers that treat an
  /// absent register as an ordinary outcome and want to say so in their own words.
  [[nodiscard]] const ControlTableItem* findItem(const std::string& name) const;
  /// @brief The item, throwing std::out_of_range if this model has no such register.
  [[nodiscard]] const ControlTableItem& getItem(const std::string& name) const;
  const std::vector<IndirectAddressInfo>& getIndirectAddressInfo();
  /// The operating modes, by name, in which Goal Current limits the current
  [[nodiscard]] const std::set<std::string>& currentCapModes() const
  {
    return current_cap_modes_;
  }

private:
  /// @brief loadFromYaml's body, run inside its guard against YAML's decoding exceptions.
  bool load(const std::string& path);

  bool loadIndirectAddressInfo(const YAML::Node& node);
  bool loadControlTable(const YAML::Node& node);
  bool loadUnitConversions(const YAML::Node& node);
  bool loadCurrentCapModes(const YAML::Node& node);
  bool assignRatiosToControlTableEntries();

  /// @brief Check that no two registers cover the same byte. Registers are keyed by name, so a
  /// mistyped address collides with another register rather than producing a duplicate key.
  [[nodiscard]] bool validateAddressRanges() const;

  std::vector<IndirectAddressInfo> indirect_addresses_;
  std::map<std::string, ControlTableItem> control_table_;
  std::map<std::string, double> unit_to_ratio;
  std::set<std::string> current_cap_modes_;
};

}  // namespace dynamixel_ros_control

#endif
