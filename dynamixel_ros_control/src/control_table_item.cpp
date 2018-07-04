#include <dynamixel_ros_control/control_table_item.h>

namespace dynamixel_ros_control {

ControlTableItem::ControlTableItem(std::string name, uint16_t address, uint8_t data_length, AccessType access_type, MemoryType memory_type)
  : name_(name), address_(address), data_length_(data_length), access_type_(access_type), memory_type_(memory_type) {}

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

uint8_t ControlTableItem::data_length() const
{
  return data_length_;
}

}
