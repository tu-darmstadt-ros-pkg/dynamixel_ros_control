#include <dynamixel_ros_control/dynamixel.h>

#include <dynamixel_ros_control/common.h>
#include <boost/algorithm/string.hpp>

namespace dynamixel_ros_control {

Dynamixel::Dynamixel(uint8_t id, uint16_t model_number, DynamixelDriver& driver)
  :  driver_(driver), id_(id), model_number_(model_number) {}

bool Dynamixel::loadControlTable()
{
  control_table_ = driver_.loadControlTable(getModelNumber());
  if (!control_table_) {
    return false;
  }

  return true;
}

bool Dynamixel::writeRegister(std::string register_name, bool value) const
{
  int32_t dxl_value = boolToDxlValue(register_name, value);
  return writeRegister(register_name, dxl_value);
}

bool Dynamixel::writeRegister(std::string register_name, double value) const
{
  int32_t dxl_value = unitToDxlValue(register_name, value);
  return writeRegister(register_name, dxl_value);
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

bool Dynamixel::writeControlMode(ControlMode mode)
{
  return writeRegister("operating_mode", static_cast<int32_t>(mode));
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

int32_t Dynamixel::unitToDxlValue(std::string register_name, double unit_value) const
{
  return static_cast<int32_t>(unit_value / getItem(register_name).dxlValueToUnitRatio());
}

int32_t Dynamixel::boolToDxlValue(std::string register_name, bool b) const
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
  return control_table_->getItem(name);
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
  for (const IndirectAddressInfo& i: control_table_->getIndirectAddressInfo()) {
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

ControlMode stringToControlMode(const std::string& str) {
  if (str == "effort" || str == "current") {
    return CURRENT;
  }
  if (str == "velocity") {
    return VELOCITY;
  }
  if (str == "position") {
    return POSITION;
  }
  if (str == "extended_position") {
    return EXTENDED_POSITION;
  }
  if (str == "current_based_position") {
    return CURRENT_BASED_POSITION;
  }
  if (str == "pwm") {
    return PWM;
  }
  throw std::invalid_argument("Control Mode '" + str + "' is unknown.");
}

}
