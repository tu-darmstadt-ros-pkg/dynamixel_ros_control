#ifndef DYNAMIXEL_ROS_CONTROL__CONTROL_TABLE_ITEM_H
#define DYNAMIXEL_ROS_CONTROL__CONTROL_TABLE_ITEM_H

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

class ControlTableItem {
public:
  ControlTableItem(std::string name, uint16_t address, uint8_t data_length,
                   AccessType access_type, MemoryType memory_type);
  std::string name() const;
  uint16_t address() const;
  uint8_t data_length() const;
  AccessType access_type() const;
  MemoryType memory_type() const;
private:
  std::string name_;
  uint16_t address_;
  uint8_t data_length_;
  AccessType access_type_;
  MemoryType memory_type_;
};

}

#endif
