#include "dynamixel_ros_control/log.hpp"

#include <dynamixel_ros_control/sync_read_manager.hpp>

namespace dynamixel_ros_control {

void SyncReadManager::addDynamixel(Dynamixel* dxl)
{
  std::pair<std::set<Dynamixel*>::iterator, bool> result = dynamixels_.emplace(dxl);
  if (!result.second) {
    DXL_LOG_WARN("Already contained dynamixel with id " << dxl->getId());
  }
}

bool SyncReadManager::addRegister(std::string register_name, const DxlValueMappingList& dxl_value_pairs,
                                  std::vector<double> offsets)
{
  if (read_entries_.find(register_name) != read_entries_.end()) {
    DXL_LOG_ERROR("Register '" << register_name << "' has already been added.");
    return false;
  }

  // Size checks
  if (offsets.empty()) {
    offsets.resize(dynamixels_.size(), 0.0);
  } else {
    if (dxl_value_pairs.size() != offsets.size()) {
      DXL_LOG_ERROR("Size of offsets (" << dxl_value_pairs.size()
                                        << ") does not match number of Dynamixel-Value pairs (" << offsets.size()
                                        << ")");
      return false;
    }
  }

  if (dxl_value_pairs.size() != dynamixels_.size()) {
    DXL_LOG_ERROR("Number of Dynamixel-Value pairs ("
                  << dxl_value_pairs.size() << ") does not match number of dynamixels (" << dynamixels_.size()
                  << "). Make sure to add all dynamixels before adding registers to the SyncReadManager.");
    return false;
  }

  // Add entry
  ReadEntry entry;
  entry.register_name = register_name;

  for (const auto& [dynamixel, value] : dxl_value_pairs) {
    if (dynamixels_.find(dynamixel) == dynamixels_.end()) {
      DXL_LOG_ERROR("Dynamixel with id " << dynamixel->getId() << " is unknown to the SyncReadManager.");
      return false;
    }
  }
  entry.dxl_value_pairs = dxl_value_pairs;
  entry.offsets = offsets;

  // Check if data length of register is the same for each servo
  uint8_t register_data_length = 0;
  for (const auto& [dxl, value] : dxl_value_pairs) {
    const uint8_t length = dxl->getItem(register_name).data_length();
    if (register_data_length == 0) {
      register_data_length = length;
    } else {
      if (register_data_length != length) {
        DXL_LOG_ERROR("Length mismatch for register '" << register_name << "'. ID " << dxl->getId() << " wants size "
                                                       << length << ", current is " << register_data_length << ".");
        subsequent_error_count_++;
        return false;
      }
    }
  }
  entry.data_length = register_data_length;

  read_entries_.emplace(register_name, entry);
  return true;
}

bool SyncReadManager::init(DynamixelDriver& driver)
{
  driver_ = &driver;
  // Compute total data length by going through all read entries
  total_data_length_ = 0;
  for (const std::map<std::string, ReadEntry>::value_type& read_kv : read_entries_) {
    total_data_length_ += read_kv.second.data_length;
  }

  // Request indirect address space from dynamixel_driver
  if (!driver.requestIndirectAddresses(total_data_length_, indirect_address_index_)) {
    DXL_LOG_ERROR("Failed to aquire indirect addresses for register with data length of " << total_data_length_ << ".");
    return false;
  }

  // Write register addresses to indirect addresses
  unsigned int current_indirect_address_index = indirect_address_index_;
  bool first_register = true;
  for (std::map<std::string, ReadEntry>::value_type& read_kv : read_entries_) {
    std::string register_name = read_kv.first;
    read_kv.second.indirect_index = current_indirect_address_index;

    bool first_dxl = true;
    for (auto& [dxl, value] : read_kv.second.dxl_value_pairs) {
      uint16_t indirect_data_address;
      if (!dxl->setIndirectAddress(current_indirect_address_index, register_name, indirect_data_address)) {
        DXL_LOG_ERROR("Failed to set indirect address mapping");
        return false;
      }
      if (first_register) {
        indirect_data_address_ = indirect_data_address;
        first_register = false;
      }
      if (first_dxl) {
        read_kv.second.indirect_data_address = indirect_data_address;
        first_dxl = false;
      }
    }
    current_indirect_address_index += read_kv.second.data_length;
  }

  // Create sync read group
  sync_read_ = driver.setSyncRead(indirect_data_address_, total_data_length_);
  if (!sync_read_) {
    DXL_LOG_ERROR("Failed to initialize GroupSyncRead.");
    return false;
  }

  for (const auto& dxl : dynamixels_) {
    if (!sync_read_->addParam(dxl->getId()))
      return false;
  }
  return true;
}

bool SyncReadManager::read()
{
  rclcpp::Time time;
  return read(time);
}

bool SyncReadManager::read(rclcpp::Time& packet_receive_time)
{
  const int dxl_comm_result = sync_read_->txRxPacket();
  packet_receive_time = rclcpp::Clock().now();  // We do not care about simulated time

  if (dxl_comm_result != COMM_SUCCESS) {
    DXL_LOG_ERROR("Sync Read failed with error code: " << driver_->communicationErrorToString(dxl_comm_result));
    subsequent_error_count_++;
    return false;
  }
  subsequent_error_count_ = 0;

  for (auto& [register_name, entry] : read_entries_) {
    const uint16_t register_data_address = entry.indirect_data_address;
    const uint8_t register_data_length = entry.data_length;
    for (unsigned int i = 0; i < entry.dxl_value_pairs.size(); i++) {
      auto& [dxl, value] = entry.dxl_value_pairs[i];
      double offset = entry.offsets[i];
      if (sync_read_->isAvailable(dxl->getId(), register_data_address, register_data_length)) {
        uint32_t data = sync_read_->getData(dxl->getId(), register_data_address, register_data_length);
        int32_t dxl_value;
        if (register_data_length == 4) {
          dxl_value = static_cast<int32_t>(data);
        } else if (register_data_length == 2) {
          dxl_value = *reinterpret_cast<int16_t*>(&data);
        } else if (register_data_length == 1) {
          dxl_value = *reinterpret_cast<int8_t*>(&data);
        } else {
          DXL_LOG_ERROR("Unsupported data length: " << register_data_length);
          dxl_value = static_cast<int32_t>(data);
        }
        if (value.dvalue) {
          const double unit_value = dxl->dxlValueToUnit(register_name, dxl_value) + offset;
          // ROS_DEBUG_STREAM_THROTTLE(0.25, "[READING " << register_name << "] id " << dxl->getId() << ", value: " <<
          // dxl_value << ", converted: " << unit_value);
          *value.dvalue = unit_value;
        }
        if (value.bvalue) {
          const bool bool_value = dxl->dxlValueToBool(register_name, dxl_value);
          // ROS_DEBUG_STREAM_THROTTLE(0.25, "[READING " << register_name << "] id " << dxl->getId() << ", value: " <<
          // dxl_value << ", converted: " << (bool_value ? "true" : "false"));
          *value.bvalue = bool_value;
        }
        if (value.ivalue) {
          // ROS_DEBUG_STREAM_THROTTLE(0.25, "[READING " << register_name << "] id " << dxl->getId() << ", value: " << dxl_value);
          *value.ivalue = dxl_value;
        }
      }
    }
  }

  return true;
}

bool SyncReadManager::isOk() const
{
  return subsequent_error_count_ < error_threshold_;
}

void SyncReadManager::setErrorThreshold(const unsigned int threshold)
{
  error_threshold_ = threshold;
}

}  // namespace dynamixel_ros_control
