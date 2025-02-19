#include <dynamixel_ros_control/dynamixel.hpp>

namespace dynamixel_ros_control {

Dynamixel::Dynamixel(const uint8_t id, DynamixelDriver& driver)
    : driver_(driver), id_(id)
{}

bool Dynamixel::connect()
{
  if (!driver_.ping(id_, model_number_)) {
    // ROS_ERROR_STREAM("Failed to ping ID " << static_cast<int>(getId()));
    return false;
  }

  // Ping successful, add to list
  control_table_ = driver_.loadControlTable(getModelNumber());
  if (!control_table_) {
    return false;
  }

  // ROS_DEBUG_STREAM("ID: " << getId() << ": Loaded control table for model " << getModelNumber());
  return true;
}

bool Dynamixel::ping() const
{
  return driver_.ping(getId());
}

bool Dynamixel::reboot() const
{
  return driver_.reboot(getId());
}

bool Dynamixel::readWriteRegister(const uint16_t address, const uint8_t data_length, const int32_t value) const
{
  int32_t register_value;
  if (!readRegister(address, data_length, register_value)) {
    return false;
  }
  if (register_value != value) {
    if (!writeRegister(address, data_length, value)) {
      return false;
    }
  }
  return true;
}

bool Dynamixel::writeRegister(const std::string& register_name, const bool value) const
{
  int32_t dxl_value = boolToDxlValue(register_name, value);
  return writeRegister(register_name, dxl_value);
}

bool Dynamixel::writeRegister(const std::string& register_name, const double value) const
{
  int32_t dxl_value = unitToDxlValue(register_name, value);
  return writeRegister(register_name, dxl_value);
}

bool Dynamixel::writeRegister(const std::string& register_name, const int32_t value) const
{
  const ControlTableItem* item;
  try {
    item = &getItem(register_name);
  }
  catch (const std::out_of_range&) {
    return false;
  }
  return writeRegister(item->address(), item->data_length(), value);
}

bool Dynamixel::writeRegister(const uint16_t address, const uint8_t data_length, const int32_t value) const
{
  return driver_.writeRegister(getId(), address, data_length, value);
}

bool Dynamixel::readRegister(const std::string& register_name, double& value_out) const
{
  int32_t dxl_value;
  if (!readRegister(register_name, dxl_value)) {
    return false;
  }
  value_out = dxlValueToUnit(register_name, dxl_value);
  return true;
}

bool Dynamixel::readRegister(const std::string& register_name, bool& value_out) const
{
  int32_t dxl_value;
  if (!readRegister(register_name, dxl_value)) {
    return false;
  }
  value_out = dxlValueToBool(register_name, dxl_value);
  return true;
}

bool Dynamixel::readRegister(const std::string& register_name, int32_t& value_out) const
{
  const ControlTableItem* item;
  try {
    item = &getItem(register_name);  // TODO why is this different?
  }
  catch (const std::out_of_range&) {
    return false;
  }
  return readRegister(item->address(), item->data_length(), value_out);
}

bool Dynamixel::readRegister(const uint16_t address, const uint8_t data_length, int32_t& value_out) const
{
  return driver_.readRegister(getId(), address, data_length, value_out);
}

bool Dynamixel::writeControlMode(const ControlMode mode) const
{
  return readWriteRegister("operating_mode", static_cast<int32_t>(mode));
}

double Dynamixel::dxlValueToUnit(const std::string& register_name, const int32_t value) const
{
  return static_cast<double>(value) * getItem(register_name).dxlValueToUnitRatio();
}

bool Dynamixel::dxlValueToBool(const std::string& register_name, const int32_t value) const
{
  std::string unit = getItem(register_name).unit();
  if (unit != "bool") {
    // ROS_ERROR_STREAM("The register '" << register_name << "' (type=" << unit << ") is not of type bool");
    return false;
  }
  return value > 0;
}

int32_t Dynamixel::unitToDxlValue(const std::string& register_name, const double unit_value) const
{
  return static_cast<int32_t>(unit_value / getItem(register_name).dxlValueToUnitRatio());
}

int32_t Dynamixel::boolToDxlValue(const std::string& register_name, const bool b) const
{
  std::string unit = getItem(register_name).unit();
  if (unit != "bool") {
    // ROS_ERROR_STREAM("The register '" << register_name << "' (type=" << unit << ") is not of type bool");
    return false;
  }
  return b;
}

bool Dynamixel::registerAvailable(const std::string& register_name) const
{
  return control_table_->itemAvailable(register_name);
}

const ControlTableItem& Dynamixel::getItem(const std::string& name) const
{
  return control_table_->getItem(name);
}

uint16_t Dynamixel::getModelNumber() const
{
  return model_number_;
}

bool Dynamixel::setIndirectAddress(const unsigned int indirect_address_index, const std::string& register_name,
                                   uint16_t& indirect_data_address) const
{
  uint16_t indirect_address;
  indirectIndexToAddresses(indirect_address_index, indirect_address, indirect_data_address);
  const ControlTableItem* item;
  try {
    item = &getItem(register_name);
  }
  catch (const std::out_of_range&) {
    return false;
  }
  uint16_t register_address = item->address();
  uint8_t data_length = item->data_length();

  // ROS_DEBUG_STREAM("[INDIRECT ADDRESS] Setting indirect address " << indirect_address << " to " << register_address
  // << " (" << register_name <<
  //                  "), data_address: " << indirect_data_address);
  bool success = true;
  for (uint16_t i = 0; i < data_length; i++) {
    success &= readWriteRegister(indirect_address + (2 * i), sizeof(register_address), register_address + i);
  }
  return success;
}

void Dynamixel::indirectIndexToAddresses(const unsigned int indirect_address_index, uint16_t& indirect_address,
                                         uint16_t& indirect_data_address) const
{
  const IndirectAddressInfo* info;
  unsigned int total_count = 0;
  unsigned int local_index;  // TODO ensure initialization
  for (const IndirectAddressInfo& i : control_table_->getIndirectAddressInfo()) {
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

std::string Dynamixel::getHardwareErrorStatusString() const
{
  std::stringstream ss;
  if (hardware_error_status & VOLTAGE_ERROR) {
    ss << "Voltage Error, ";
  }
  if (hardware_error_status & HALL_SENSOR_ERROR) {
    ss << "Hall Sensor Error, ";
  }
  if (hardware_error_status & OVERHEATING_ERROR) {
    ss << "Overheating Error, ";
  }
  if (hardware_error_status & MOTOR_ENCODER_ERROR) {
    ss << "Motor Encoder Error, ";
  }
  if (hardware_error_status & ELECTRICAL_SHOCK_ERROR) {
    ss << "Electrical Shock Error, ";
  }
  if (hardware_error_status & OVERLOAD_ERROR) {
    ss << "Overload Error, ";
  }
  ss << std::endl;
  return ss.str();
}

ControlMode stringToControlMode(const std::string& str)
{
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

}  // namespace dynamixel_ros_control
