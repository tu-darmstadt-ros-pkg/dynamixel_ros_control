#include <dynamixel_ros_control/dynamixel.h>

#include <dynamixel_ros_control/common.h>
#include <boost/algorithm/string.hpp>

#include <memory>

namespace dynamixel_ros_control {

Dynamixel::Dynamixel(DynamixelDriver& driver)
  : driver_(driver)
{}

Dynamixel::Dynamixel(uint8_t id, DynamixelDriver& driver)
  :  driver_(driver), id_(id)

{}

bool Dynamixel::loadControlTable()
{
  int model_number_config;
  bool model_number_set = nh_.param("model_number", model_number_config, -1);
  if (!driver_.ping(getId(), model_number_)) {
    ROS_ERROR_STREAM("Failed to ping ID " << static_cast<int>(getId()));
    return false;
  } else {
    // Ping successful, add to list
    if (model_number_set && model_number_config != model_number_) {
      ROS_WARN_STREAM("Model number in config [" << model_number_config
                      << "] does not match servo model number [" << model_number_ << "] for ID: " << static_cast<int>(getId()));
    }
    control_table_ = driver_.loadControlTable(getModelNumber());
    if (!control_table_) {
      return false;
    }
  }

  return true;
}

bool Dynamixel::initFromNh(const ros::NodeHandle& nh)
{
  nh_ = nh;
  int id_int;
  if (!loadRequiredParameter(nh, "id", id_int)) {
    return false;
  }
  if (id_int >= 0 && id_int < 256) {
    id_ = static_cast<uint8_t>(id_int);
  } else {
    ROS_ERROR_STREAM("ID " << id_int << " is not in the valid range [0;255]");
    return false;
  }
  return true;
}

bool Dynamixel::ping()
{
  return driver_.ping(getId());
}

bool Dynamixel::reboot()
{
  return driver_.reboot(getId());
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

bool Dynamixel::readRegister(std::string register_name, double& value_out) const
{
  int32_t dxl_value;
  if (!readRegister(register_name, dxl_value)) {
    return false;
  }
  value_out = dxlValueToUnit(register_name, dxl_value);
  return true;
}

bool Dynamixel::readRegister(std::string register_name, int32_t& value_out) const
{
  const ControlTableItem* item;
  try {
    item = &getItem(register_name);
  } catch (const std::out_of_range&) {
    return false;
  }
  return readRegister(item->address(), item->data_length(), value_out);
}

bool Dynamixel::readRegister(uint16_t address, uint8_t data_length, int32_t& value_out) const
{
  return driver_.readRegister(getId(), address, data_length, value_out);
}

bool Dynamixel::writeControlMode(ControlMode mode) const
{
  return writeRegister("operating_mode", static_cast<int32_t>(mode));
}

double Dynamixel::dxlValueToUnit(std::string register_name, int32_t value) const
{
  return static_cast<double>(value) * getItem(register_name).dxlValueToUnitRatio();
}

bool Dynamixel::dxlValueToBool(std::string register_name, int32_t value) const
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

bool Dynamixel::registerAvailable(std::string register_name) const
{
  return control_table_->itemAvailable(register_name);
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
  indirectIndexToAddresses(indirect_address_index, indirect_address, indirect_data_address);
  const ControlTableItem* item;
  try {
    item = &getItem(register_name);
  } catch (const std::out_of_range&) {
    return false;
  }
  uint16_t register_address = item->address();
  uint8_t data_length = item->data_length();

  ROS_DEBUG_STREAM("[INDIRECT ADDRESS] Setting indirect address " << indirect_address << " to " << register_address << " (" << register_name <<
                   "), data_address: " << indirect_data_address);
  bool success = true;
  for (uint16_t i = 0; i < data_length; i++) {
    success &= writeRegister(indirect_address + (2*i), sizeof(register_address), register_address + i);
  }
  return success;
}

void Dynamixel::addTimeTranslator(const ros::NodeHandle& nh)
{
  device_time_translator_ = std::unique_ptr<cuckoo_time_translator::DefaultDeviceTimeUnwrapperAndTranslator>(
                               new cuckoo_time_translator::DefaultDeviceTimeUnwrapperAndTranslator(
                                 cuckoo_time_translator::WrappingClockParameters(std::numeric_limits<int16_t>::max(), 1000.0),
                                 nh.getNamespace()));
}

void Dynamixel::indirectIndexToAddresses(unsigned int indirect_address_index, uint16_t& indirect_address, uint16_t& indirect_data_address)
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

bool Dynamixel::translateTime(const ros::Time& receive_time)
{
  if (!device_time_translator_) {
    ROS_ERROR_STREAM("Device time translator has not been initialized yet");
    return false;
  }
  // Offset between device and host time, we guess 5ms for now
  double offset = -0.005;
  
  // @TODO Make sure we really only use strightly monotonic increasing device stamps
  stamp_ = device_time_translator_->update(realtime_tick_ms_, receive_time, offset);

  return true;
}

ros::Time Dynamixel::getStamp() const
{
  if (!device_time_translator_) {
    ROS_ERROR_STREAM("Device time translator has not been initialized yet");
    return ros::Time::now();
  }
  return stamp_;
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
