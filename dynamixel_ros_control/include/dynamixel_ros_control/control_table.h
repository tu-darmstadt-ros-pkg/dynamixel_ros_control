#ifndef DYNAMIXEL_ROS_CONTROL_CONTROL_TABLE_H
#define DYNAMIXEL_ROS_CONTROL_CONTROL_TABLE_H

#include <yaml-cpp/yaml.h>
#include <dynamixel_ros_control/control_table_item.h>

namespace dynamixel_ros_control {

struct IndirectAddressInfo {
  uint16_t indirect_address_start;
  unsigned int count;
  uint16_t indirect_data_start;

  std::string toString() const {
    return " --- Indirect address start: " + std::to_string(indirect_address_start) + "\n" +
           " --- Count: " + std::to_string(count) + "\n" +
           " --- Indirect data start: " + std::to_string(indirect_data_start);
  }
};

class ControlTable {
public:
  bool loadFromYaml(const std::string& path);
  bool itemAvailable(std::string name) const;
  const ControlTableItem& getItem(std::string& name) const;
  const std::vector<IndirectAddressInfo>& getIndirectAddressInfo();
private:
  bool loadIndirectAddressInfo(const YAML::Node& node);
  bool loadControTable(const YAML::Node& node);
  bool loadUnitConversions(const YAML::Node& node);
  void assignRatiosToControlTableEntries();

  std::vector<IndirectAddressInfo> indirect_addresses_;
  std::map<std::string, ControlTableItem> control_table_;
  std::map<std::string, double> unit_to_ratio;
};

}

#endif
