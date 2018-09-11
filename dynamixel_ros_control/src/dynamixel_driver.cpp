#include <dynamixel_ros_control/dynamixel_driver.h>

#include <dynamixel_ros_control/common.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

namespace dynamixel_ros_control {

DynamixelDriver::DynamixelDriver()
  : next_indirect_address_(0) {}

bool DynamixelDriver::init(const ros::NodeHandle& nh)
{
  // Get package path
  package_path_ = ros::package::getPath(PACKAGE_NAME);
  if (package_path_ == "") {
    ROS_FATAL_STREAM("Could not find own package path.");
    return false;
  }
  // Get model number to series mapping
  if (!loadSeriesMapping()) {
    ROS_FATAL_STREAM("Failed to load model number to series mapping.");
    return false;
  }

  // Get port info
  std::string port_name;
  if (!loadRequiredParameter(nh, "port_info/port_name", port_name) || !setPortHandler(port_name)) {
    return false;
  }

  int baudrate;
  if (!loadRequiredParameter(nh, "port_info/baud_rate", baudrate) || !setBaudRate(baudrate)) {
    return false;
  }

  float protocol_version;
  if (!loadRequiredParameter(nh, "port_info/protocol_version", protocol_version)) {
    return false;
  }
  return setPacketHandler(protocol_version);
}

bool DynamixelDriver::loadSeriesMapping()
{
  std::string path = package_path_ + "/devices/model_list.yaml";
  YAML::Node config = YAML::LoadFile(path); // TODO check if file exists
  if (!config.IsMap()) {
    ROS_ERROR_STREAM("model_list.yaml is not a map (wrong format).");
    return false;
  }
  for(YAML::const_iterator it = config.begin();it != config.end(); ++it) {
    // TODO check if types are valid
    uint16_t model_number = it->first.as<uint16_t>();
    std::string series = it->second.as<std::string>();
    model_number_to_series_.emplace(model_number, series);
  }
  return true;
}

ControlTable* DynamixelDriver::readControlTable(std::string series)
{
  ControlTable table;
  std::pair<std::map<std::string, ControlTable>::iterator, bool> result = series_to_control_table_.emplace(series, table);
  ControlTable* table_ptr = &result.first->second;
  std::string path = package_path_ + "/devices/models/" + series + ".yaml";
  if (!table_ptr->loadFromYaml(path)) {
    ROS_ERROR_STREAM("Failed to read control table for '" << series << "'");
    return nullptr;
  }
  return table_ptr;
}

ControlTable* DynamixelDriver::loadControlTable(uint16_t model_number)
{
  std::string series;
  try {
    series = model_number_to_series_.at(model_number);
  } catch (const std::out_of_range&) {
    ROS_FATAL_STREAM("Could not find series of model number " << model_number);
    return nullptr;
  }

  // Check if control table was loaded already
  ControlTable* control_table;
  try {
    control_table = &series_to_control_table_.at(series);
  } catch (const std::out_of_range&) {
    control_table = nullptr;
  }
  if (!control_table) {
    // Read it
    control_table = readControlTable(series);
  }
  return control_table;
}

bool DynamixelDriver::ping(uint8_t id)
{
  uint16_t model_number;
  return ping(id, model_number);
}

bool DynamixelDriver::ping(uint8_t id, uint16_t& model_number)
{
  uint8_t error = 0;
  return packet_handler_->ping(port_handler_, id, &model_number, &error) == COMM_SUCCESS;
}

std::vector<std::pair<uint8_t, uint16_t>> DynamixelDriver::scan()
{
  std::vector<std::pair<uint8_t, uint16_t>> dxl_list;
  for (int id = 0; id <= std::numeric_limits<uint8_t>::max(); id++) {
    uint8_t idc = static_cast<uint8_t>(id);
    uint16_t model_number;
    if (ping(idc, model_number)) {
      dxl_list.push_back(std::make_pair(idc, model_number));
    }
  }
  return dxl_list;
}

bool DynamixelDriver::writeRegister(uint8_t id, uint16_t address, uint8_t data_length, int32_t value)
{
  ROS_DEBUG_STREAM("[Register Write] id " << static_cast<unsigned int>(id) << ", address: " << address << ", length: " << static_cast<unsigned int>(data_length)
                   << ", value: " << value);
  uint8_t error = 0;
  int comm_result = COMM_TX_FAIL;

  if (data_length == 1) {
    comm_result = packet_handler_->write1ByteTxRx(port_handler_, id, address, static_cast<uint8_t>(value), &error);
  }
  else if (data_length == 2) {
    comm_result = packet_handler_->write2ByteTxRx(port_handler_, id, address, static_cast<uint16_t>(value), &error);
  }
  else if (data_length == 4) {
    comm_result = packet_handler_->write4ByteTxRx(port_handler_, id, address, static_cast<uint32_t>(value), &error);
  }

  if (comm_result == COMM_SUCCESS) {
    if (error != 0) {
      const char* error_cstr = packet_handler_->getRxPacketError(error);
      ROS_ERROR_STREAM("[ID " << static_cast<int>(id) << "] Failed to write: " << error_cstr);
      return false;
    }
    return true;
  } else {
    const char* error_cstr = packet_handler_->getTxRxResult(comm_result);
    ROS_ERROR_STREAM("[ID " << static_cast<int>(id) << "] Communication error while writing: " << error_cstr);
    return false;
  }
}

bool DynamixelDriver::readRegister(uint8_t id, uint16_t address, uint8_t data_length, int32_t& value_out)
{
  uint8_t error = 0;
  int comm_result = COMM_RX_FAIL;

  uint32_t* value_ptr = reinterpret_cast<uint32_t*>(&value_out);

  if (data_length == 1) {
    comm_result = packet_handler_->read1ByteTxRx(port_handler_, id, address, reinterpret_cast<uint8_t*>(value_ptr), &error);
  }
  else if (data_length == 2)
  {
    comm_result = packet_handler_->read2ByteTxRx(port_handler_, id, address, reinterpret_cast<uint16_t*>(value_ptr), &error);
  }
  else if (data_length == 4)
  {
    comm_result = packet_handler_->read4ByteTxRx(port_handler_, id, address, value_ptr, &error);
  }
  ROS_DEBUG_STREAM("[Register Read] id " << static_cast<unsigned int>(id) << ", address: " << address << ", length: " << static_cast<unsigned int>(data_length)
                   << ", value: " << value_out);
  if (comm_result == COMM_SUCCESS) {
    if (error != 0) {
      const char* error_cstr = packet_handler_->getRxPacketError(error);
      ROS_ERROR_STREAM("[ID " << static_cast<int>(id) << "] Read error: " << error_cstr);
      return false;
    }
    return true;
  } else {
    const char* error_cstr = packet_handler_->getTxRxResult(comm_result);
    ROS_ERROR_STREAM("[ID " << static_cast<int>(id) << "] Read communication error: " << error_cstr);
    return false;
  }
}

dynamixel::GroupSyncWrite*DynamixelDriver::setSyncWrite(uint16_t address, uint8_t data_length)
{
  return new dynamixel::GroupSyncWrite(port_handler_, packet_handler_, address, data_length);
}

dynamixel::GroupSyncRead* DynamixelDriver::setSyncRead(uint16_t address, uint8_t data_length)
{
  return new dynamixel::GroupSyncRead(port_handler_, packet_handler_, address, data_length);
}

bool DynamixelDriver::requestIndirectAddresses(unsigned int data_length, unsigned int& address_start)
{
  address_start = next_indirect_address_;
  next_indirect_address_ += data_length;
  return true;
}

bool DynamixelDriver::setPacketHandler(float protocol_version)
{
  packet_handler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);
  if (!packet_handler_) {
    ROS_ERROR_STREAM("Unsupported protocol version: " << protocol_version);
    return false;
  }
  return true;
}

bool DynamixelDriver::setPortHandler(std::string port_name)
{
  port_handler_ = dynamixel::PortHandler::getPortHandler(port_name.c_str());

  if (port_handler_->openPort())
  {
    ROS_INFO_STREAM("Succeeded to open port " << port_name);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to open port " << port_name);
    return false;
  }
}

bool DynamixelDriver::setBaudRate(int baud_rate)
{
  if (port_handler_->setBaudRate(baud_rate))
  {
    ROS_INFO_STREAM("Succeeded to change the baudrate to " << port_handler_->getBaudRate());
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to change the baudrate to " << port_handler_->getBaudRate());
    return false;
  }
}

}
