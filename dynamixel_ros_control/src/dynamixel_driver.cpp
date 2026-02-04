#include "dynamixel_ros_control/log.hpp"

#include <cstring>
#include <dynamixel_ros_control/dynamixel_driver.hpp>
#include <dynamixel_ros_control/mock_dynamixel.hpp>

#include <dynamixel_ros_control/common.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <yaml-cpp/yaml.h>

namespace dynamixel_ros_control {

DynamixelDriver::DynamixelDriver()
    : next_indirect_address_index_(0), use_dummy_(false)
{}

bool DynamixelDriver::init(const std::string& port_name, const int baud_rate, bool use_dummy)
{
  use_dummy_ = use_dummy;
  port_name_ = port_name;
  setPortHandler(port_name_);

  baud_rate_ = baud_rate;

  // Get package path
  package_path_ = ament_index_cpp::get_package_share_directory("dynamixel_ros_control");
  if (package_path_.empty()) {
    DXL_LOG_FATAL("Could not find own package path.");
    return false;
  }

  // Get model number to series mapping
  if (!loadSeriesMapping()) {
    DXL_LOG_FATAL("Failed to load model number to series mapping.");
    return false;
  }

  // Set Packet handler
  if (!setPacketHandler()) {
    return false;
  }

  return true;
}

bool DynamixelDriver::connect()
{
  if (!connectPort()) {
    return false;
  }
  return setBaudRate(baud_rate_);
}

bool DynamixelDriver::loadSeriesMapping()
{
  const std::string path = package_path_ + "/devices/model_list.yaml";
  YAML::Node config;
  try {
    config = YAML::LoadFile(path);
  }
  catch (YAML::BadFile&) {
    DXL_LOG_ERROR("Failed to read series mapping at '" << path << "'. Does the file exist?");
    return false;
  }
  if (!config.IsMap()) {
    DXL_LOG_ERROR("model_list.yaml is not a map (wrong format).");
    return false;
  }
  for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
    // TODO: check if types are valid
    auto model_number = it->first.as<uint16_t>();
    auto series = it->second.as<std::string>();
    model_number_to_series_.emplace(model_number, series);
  }
  return true;
}

ControlTable* DynamixelDriver::readControlTable(std::string series)
{
  // Emplace empty table first, then load into it (avoids copy)
  auto [entry, inserted] = series_to_control_table_.try_emplace(series);
  if (!inserted) {
    // Already existed - return cached pointer
    return &entry->second;
  }

  const std::string path = package_path_ + "/devices/models/" + series + ".yaml";
  if (!entry->second.loadFromYaml(path)) {
    DXL_LOG_ERROR("Failed to read control table for '" << series << "'");
    series_to_control_table_.erase(entry);
    return nullptr;
  }
  return &entry->second;
}

ControlTable* DynamixelDriver::loadControlTable(const uint16_t model_number)
{
  // Look up series name for model number
  if (auto it = model_number_to_series_.find(model_number); it != model_number_to_series_.end()) {
    const auto& series = it->second;

    // Check if control table is already cached
    if (auto table_it = series_to_control_table_.find(series); table_it != series_to_control_table_.end()) {
      return &table_it->second;
    }

    // Load and cache the control table
    return readControlTable(series);
  }

  DXL_LOG_FATAL("Could not find series of model number " << model_number);
  return nullptr;
}

bool DynamixelDriver::ping(const uint8_t id) const
{
  uint16_t model_number;
  return ping(id, model_number);
}

bool DynamixelDriver::ping(const uint8_t id, uint16_t& model_number) const
{
  uint8_t error = 0;
  return packet_handler_->ping(port_handler_, id, &model_number, &error) == COMM_SUCCESS;
}

std::vector<std::pair<uint8_t, uint16_t>> DynamixelDriver::scan() const
{
  std::vector<std::pair<uint8_t, uint16_t>> dxl_list;
  for (int id = 0; id <= std::numeric_limits<uint8_t>::max(); id++) {
    auto idc = static_cast<uint8_t>(id);
    uint16_t model_number;
    if (ping(idc, model_number)) {
      dxl_list.emplace_back(idc, model_number);
    }
  }
  return dxl_list;
}

void DynamixelDriver::addDummyMotor(uint8_t id, uint16_t model_number)
{
  if (use_dummy_) {
    MockDynamixelManager::instance().addMotor(id, model_number);
  }
}

bool DynamixelDriver::reboot(const uint8_t id) const
{
  uint8_t error = 0;
  DXL_LOG_DEBUG("[REBOOT] id " << static_cast<unsigned int>(id));
  int comm_result = packet_handler_->reboot(port_handler_, id, &error);
  if (comm_result != COMM_SUCCESS) {
    DXL_LOG_ERROR("[ID " << static_cast<int>(id)
                         << "] Communication error while rebooting: " << communicationErrorToString(comm_result));
  }
  if (error != 0) {
    DXL_LOG_ERROR("[ID " << static_cast<int>(id) << "] Failed to reboot: " << packetErrorToString(error)
                         << " error code: " << error);
  }
  return comm_result == COMM_SUCCESS;
}

bool DynamixelDriver::writeRegister(const uint8_t id, const uint16_t address, const uint8_t data_length,
                                    int32_t value) const
{
  DXL_LOG_DEBUG("[Register Write] id " << static_cast<unsigned int>(id) << ", address: " << address << ", length: "
                                       << static_cast<unsigned int>(data_length) << ", value: " << value);
  uint8_t error = 0;
  int comm_result = COMM_TX_FAIL;

  if (data_length == 1) {
    auto value_8bit = static_cast<uint8_t>(value & 0xFF);
    comm_result = packet_handler_->write1ByteTxRx(port_handler_, id, address, value_8bit, &error);
  } else if (data_length == 2) {
    auto value_16bit = static_cast<uint16_t>(value & 0xFFFF);
    comm_result = packet_handler_->write2ByteTxRx(port_handler_, id, address, value_16bit, &error);
  } else if (data_length == 4) {
    uint32_t unsigned_value;
    std::memcpy(&unsigned_value, &value, sizeof(uint32_t));
    comm_result = packet_handler_->write4ByteTxRx(port_handler_, id, address, unsigned_value, &error);
  }

  if (comm_result != COMM_SUCCESS) {
    DXL_LOG_ERROR("[ID " << static_cast<int>(id)
                         << "] Communication error while writing: " << communicationErrorToString(comm_result));
    return false;
  }

  if (error != 0) {
    DXL_LOG_ERROR("[ID " << static_cast<int>(id) << "] Failed to write: " << packetErrorToString(error));
    return false;
  }

  return true;
}

bool DynamixelDriver::readRegister(const uint8_t id, const uint16_t address, const uint8_t data_length,
                                   int32_t& value_out) const
{
  uint8_t error = 0;
  int comm_result = COMM_RX_FAIL;

  value_out = 0;

  if (data_length == 1) {
    uint8_t data;
    comm_result = packet_handler_->read1ByteTxRx(port_handler_, id, address, &data, &error);
    value_out = static_cast<int8_t>(data);
  } else if (data_length == 2) {
    uint16_t data;
    comm_result = packet_handler_->read2ByteTxRx(port_handler_, id, address, &data, &error);
    value_out = static_cast<int16_t>(data);  // Sign extension for signed 16-bit values
  } else if (data_length == 4) {
    uint32_t data;
    comm_result = packet_handler_->read4ByteTxRx(port_handler_, id, address, &data, &error);
    std::memcpy(&value_out, &data, sizeof(int32_t));
  } else {
    DXL_LOG_ERROR("Unsupported data length: " << data_length);
    return false;
  }
  DXL_LOG_DEBUG("[Register Read] id " << static_cast<unsigned int>(id) << ", address: " << address << ", length: "
                                      << static_cast<unsigned int>(data_length) << ", value: " << value_out);

  if (comm_result != COMM_SUCCESS) {
    DXL_LOG_ERROR("[ID " << static_cast<int>(id)
                         << "] Read communication error: " << communicationErrorToString(comm_result));
    return false;
  }

  if (error != 0) {
    DXL_LOG_ERROR("[ID " << static_cast<int>(id) << "] Read error: " << packetErrorToString(error));
    return false;
  }

  return true;
}

std::shared_ptr<GroupSyncWrite> DynamixelDriver::setSyncWrite(uint16_t address, uint8_t data_length) const
{
  if (use_dummy_) {
    return std::make_shared<MockGroupSyncWrite>(port_handler_, packet_handler_, address, data_length);
  }
  return std::make_shared<RealGroupSyncWrite>(port_handler_, packet_handler_, address, data_length);
}

std::shared_ptr<GroupSyncRead> DynamixelDriver::setSyncRead(uint16_t address, uint8_t data_length) const
{
  if (use_dummy_) {
    return std::make_shared<MockGroupSyncRead>(port_handler_, packet_handler_, address, data_length);
  }
  return std::make_shared<RealGroupSyncRead>(port_handler_, packet_handler_, address, data_length);
}

bool DynamixelDriver::requestIndirectAddresses(const unsigned int data_length, unsigned int& address_start_index)
{
  if (use_dummy_) {
    // Mock indirect addresses simply by incrementing.
    // The MockDynamixel/MockPacketHandler logic doesn't strictly enforce indirect address checking
    // unless we implemented complex indirect address logic in the mock.
    // For now, let's just pretend it works.
    address_start_index = next_indirect_address_index_;
    next_indirect_address_index_ += data_length;
    DXL_LOG_DEBUG("[INDIRECT ADDRESS MOCK] Reserving " << address_start_index);
    return true;
  }
  DXL_LOG_DEBUG("[INDIRECT ADDRESS] Reserving indirect address index " << address_start_index << " with length "
                                                                       << data_length);
  address_start_index = next_indirect_address_index_;
  next_indirect_address_index_ += data_length;
  return true;
}

bool DynamixelDriver::releaseIndirectAddresses(const unsigned int data_length, const unsigned int address_start_index)
{
  const unsigned int new_address_start = next_indirect_address_index_ - data_length;

  // Check if this is the last address
  if (new_address_start == address_start_index) {
    next_indirect_address_index_ = address_start_index;
    DXL_LOG_DEBUG("[INDIRECT ADDRESS] Realising indirect address index " << address_start_index << " with length "
                                                                         << data_length);
    return true;
  }

  // Cannot release the address (for now)
  return false;
}

std::string DynamixelDriver::communicationErrorToString(const int comm_result) const
{
  const char* error_cstr = packet_handler_->getTxRxResult(comm_result);
  return {error_cstr};
}

std::string DynamixelDriver::packetErrorToString(const uint8_t error) const
{
  const char* error_cstr = packet_handler_->getRxPacketError(error);
  return {error_cstr};
}

bool DynamixelDriver::setPacketHandler()
{
  constexpr float protocol_version = 2.0;
  if (use_dummy_) {
    packet_handler_ = std::make_shared<MockPacketHandler>(protocol_version);
  } else {
    packet_handler_ = std::make_shared<RealPacketHandler>(protocol_version);
  }
  if (!packet_handler_) {
    DXL_LOG_ERROR("Unsupported protocol version: " << protocol_version);
    return false;
  }
  return true;
}

bool DynamixelDriver::setPortHandler(const std::string& port_name)
{
  if (use_dummy_) {
    port_handler_ = std::make_shared<MockPortHandler>(port_name.c_str());
  } else {
    port_handler_ = std::make_shared<RealPortHandler>(port_name.c_str());
  }
  return true;
}

bool DynamixelDriver::connectPort()
{
  if (!port_handler_->openPort()) {
    DXL_LOG_ERROR("Failed to open port '" << port_handler_->getPortName() << "'.");
    return false;
  }

  DXL_LOG_INFO("Succeeded to open port '" << port_handler_->getPortName() << "'.");
  next_indirect_address_index_ = 0;
  return true;
}

bool DynamixelDriver::setBaudRate(const int baud_rate) const
{
  if (!port_handler_->setBaudRate(baud_rate)) {
    DXL_LOG_ERROR("Failed to change the baudrate to " << port_handler_->getBaudRate());
    return false;
  }

  DXL_LOG_INFO("Succeeded to change the baudrate to " << port_handler_->getBaudRate());
  return true;
}

}  // namespace dynamixel_ros_control
