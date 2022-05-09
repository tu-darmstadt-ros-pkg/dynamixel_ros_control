#include <dynamixel_ros_control/sync_read_manager.h>

namespace dynamixel_ros_control {

SyncReadManager::SyncReadManager()
  : subsequent_error_count_(0), error_threshold_(25) {

}

void SyncReadManager::addDynamixel(Dynamixel* dxl)
{
  std::pair<std::set<Dynamixel*>::iterator, bool> result = dynamixels_.emplace(dxl);
  if (!result.second) {
    ROS_WARN_STREAM("Already contained dynamixel with id " << dxl->getId());
  }
}

//void SyncReadManager::addRegister(Dynamixel& dxl, std::string register_name, uint32_t* value)
//{

//}

bool SyncReadManager::addRegister(std::string register_name, const DxlValueMappingList& dxl_value_pairs, std::vector<double> offsets)
{
  if (read_entries_.find(register_name) != read_entries_.end()) {
    ROS_ERROR_STREAM("Register '" << register_name << "' has already been added.");
    return false;
  }

  // Size checks
  if (offsets.empty()) {
    offsets.resize(dynamixels_.size(), 0.0);
  } else {
    if (dxl_value_pairs.size() != offsets.size()) {
      ROS_ERROR("Size of offsets (%lu) does not match number of Dynamixel-Value pairs (%lu)", dxl_value_pairs.size(), offsets.size());
      return false;
    }
  }

  if (dxl_value_pairs.size() != dynamixels_.size()) {
    ROS_ERROR("Number of Dynamixel-Value pairs (%lu) does not match number of dynamixels (%lu). "
              "Make sure to add all dynamixels before adding registers to the SyncReadManager.", dxl_value_pairs.size(), dynamixels_.size());
    return false;
  }

  // Add entry
  ReadEntry entry;
  entry.register_name = register_name;

  for (const std::vector<std::pair<Dynamixel*, DxlValue>>::value_type& dxl_value_pair: dxl_value_pairs)
  {
    if (dynamixels_.find(dxl_value_pair.first) == dynamixels_.end()) {
      ROS_ERROR_STREAM("Dynamixel with id " << dxl_value_pair.first->getId() << " is unknown to the SyncReadManager.");
      return false;
    }
  }
  entry.dxl_value_pairs = dxl_value_pairs;
  entry.offsets = offsets;

  // Check if data length of register is the same for each servo
  uint8_t register_data_length = 0;
  for (const std::vector<std::pair<Dynamixel*, DxlValue>>::value_type& dxl_value_pair: dxl_value_pairs) {
    const Dynamixel* dxl = dxl_value_pair.first;
    uint8_t length = dxl->getItem(register_name).data_length();
    if (register_data_length == 0) {
      register_data_length = length;
    } else {
      if (register_data_length != length) {
        ROS_ERROR_STREAM("Length mismatch for register '" << register_name << "'. "
                         "ID " << dxl->getId() << " wants size " << length << ", current is " << register_data_length << ".");
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
  for (std::map<std::string, ReadEntry>::value_type& read_kv: read_entries_) {
    total_data_length_ += read_kv.second.data_length;
  }

  // Request indirect address space from dynamixel_driver
  if (!driver.requestIndirectAddresses(total_data_length_, indirect_address_index_)) {
    ROS_ERROR_STREAM("Failed to aquire indirect addresses for register with data length of " << total_data_length_ << ".");
    return false;
  }

  // Write register addresses to indirect addresses
  unsigned int current_indirect_address_index = indirect_address_index_;
  bool first_register = true;
  for (std::map<std::string, ReadEntry>::value_type& read_kv: read_entries_) {
    std::string register_name = read_kv.first;
    read_kv.second.indirect_index = current_indirect_address_index;

    bool first_dxl = true;
    for (std::vector<std::pair<Dynamixel*, DxlValue>>::value_type& dxl_value_pair: read_kv.second.dxl_value_pairs) {
      Dynamixel* dxl = dxl_value_pair.first;
      uint16_t indirect_data_address;
      if(!dxl->setIndirectAddress(current_indirect_address_index, register_name, indirect_data_address)) {
        ROS_ERROR_STREAM("Failed to set indirect address mapping");
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
    ROS_ERROR_STREAM("Failed to initialize GroupSyncRead.");
    return false;
  }

  for (const std::set<Dynamixel*>::value_type& dxl: dynamixels_)
  {
    if (!sync_read_->addParam(dxl->getId()))
      return false;
  }
  return sync_read_ != nullptr;
}

bool SyncReadManager::read()
{
  ros::Time time;
  return read(time);
}

bool SyncReadManager::read(ros::Time& packet_receive_time)
{
  int dxl_comm_result = sync_read_->txRxPacket();
  packet_receive_time = ros::Time::now();

  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR_STREAM("Sync Read failed with error code: " << driver_->communicationErrorToString(dxl_comm_result));
    subsequent_error_count_++;
    return false;
  }
  subsequent_error_count_ = 0;

  for (std::map<std::string, ReadEntry>::value_type& read_kv: read_entries_) {
    std::string register_name = read_kv.first;

    uint16_t register_data_address = read_kv.second.indirect_data_address;
    uint8_t register_data_length = read_kv.second.data_length;
    for (unsigned int i = 0; i < read_kv.second.dxl_value_pairs.size(); i++) {
      std::pair<Dynamixel*, DxlValue>& dxl_value_pair = read_kv.second.dxl_value_pairs[i];
      double offset = read_kv.second.offsets[i];
      Dynamixel* dxl = dxl_value_pair.first;
      if (sync_read_->isAvailable(dxl->getId(), register_data_address, register_data_length)) {
        int32_t dxl_value = static_cast<int32_t>(sync_read_->getData(dxl->getId(), register_data_address, register_data_length));
        if (dxl_value_pair.second.dvalue) {
          double unit_value = dxl->dxlValueToUnit(register_name, dxl_value) + offset;
          ROS_DEBUG_STREAM_THROTTLE(0.25, "[READING " << register_name << "] Value: " << dxl_value << ", Converted: " << unit_value);
          *dxl_value_pair.second.dvalue = unit_value;
        }
        if (dxl_value_pair.second.bvalue) {
          bool bool_value = dxl->dxlValueToBool(register_name, dxl_value);
          ROS_DEBUG_STREAM_THROTTLE(0.25, "[READING " << register_name << "] Value: " << dxl_value << ", Converted: " << (bool_value ? "True" : "False"));
          *dxl_value_pair.second.bvalue = bool_value;
        }
        if (dxl_value_pair.second.ivalue) {
          ROS_DEBUG_STREAM_THROTTLE(0.25, "[READING " << register_name << "] Value: " << dxl_value);
          *dxl_value_pair.second.ivalue = dxl_value;
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

void SyncReadManager::setErrorThreshold(unsigned int threshold)
{
  error_threshold_ = threshold;
}

}
