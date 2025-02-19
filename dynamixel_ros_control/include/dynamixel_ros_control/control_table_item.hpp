#ifndef DYNAMIXEL_ROS_CONTROL_CONTROL_TABLE_ITEM_H
#define DYNAMIXEL_ROS_CONTROL_CONTROL_TABLE_ITEM_H

#include <cstdint>
#include <string>

namespace dynamixel_ros_control {

enum AccessType {
  READ,
  READ_WRITE
};

enum MemoryType {
  EEPROM,
  RAM
};

bool stringToAccessType(const std::string& str, AccessType& access_type);
bool stringToMemoryType(const std::string& str, MemoryType& memory_type);

class ControlTableItem {
public:
  ControlTableItem(std::string name, uint16_t address, uint8_t data_length,
                   AccessType access_type, MemoryType memory_type);
  ControlTableItem();
  bool loadFromString(std::string control_table_string);

  [[nodiscard]] std::string name() const;
  [[nodiscard]] uint16_t address() const;
  [[nodiscard]] uint8_t data_length() const;
  [[nodiscard]] AccessType access_type() const;
  [[nodiscard]] MemoryType memory_type() const;
  [[nodiscard]] std::string unit() const;
  [[nodiscard]] double dxlValueToUnitRatio() const;

  void setDxlValueToUnitRatio(double dxl_value_to_unit_ratio);

private:


  std::string name_;
  uint16_t address_;
  uint8_t data_length_;
  AccessType access_type_;
  MemoryType memory_type_;
  std::string unit_;
  double dxl_value_to_unit_ratio_;
};

}

#endif
