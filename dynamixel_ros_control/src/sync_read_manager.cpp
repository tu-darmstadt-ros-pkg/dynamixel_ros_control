#include <dynamixel_ros_control/sync_read_manager.h>

namespace dynamixel_ros_control {

SyncReadManager::SyncReadManager() {

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

void SyncReadManager::addRegister(std::string register_name, std::vector<std::pair<Dynamixel*, double*>> dxl_value_pairs)
{
  if (read_entries_.find(register_name) != read_entries_.end()) {
    ROS_ERROR_STREAM("Register '" << register_name << "' has already been added.");
    return;
  }

  if (dxl_value_pairs.size() != dynamixels_.size()) {
    ROS_ERROR("Number of Dynamixel-Value pairs (%lu) does not match number of dynamixels (%lu). "
              "Make sure to add all dynamixels befor adding registers to the SyncReadManager.", dxl_value_pairs.size(), dynamixels_.size());
    return;
  }

  ReadEntry entry;
  entry.register_name = register_name;
//  entry.indirect_data_index = -1; // Unset

  std::vector<ReadEntry> entry_list;
  for (const std::vector<std::pair<Dynamixel*, double*>>::value_type& dxl_value_pair: dxl_value_pairs)
  {
    if (dynamixels_.find(dxl_value_pair.first) == dynamixels_.end()) {
      ROS_ERROR_STREAM("Dynamixel with id " << dxl_value_pair.first->getId() << " is unknown to the SyncReadManager.");
      return;
    }
    entry.dxl = dxl_value_pair.first;
    entry.value = dxl_value_pair.second;
    entry_list.push_back(entry);
  }

  read_entries_.emplace(register_name, entry_list);
}

bool SyncReadManager::init(DynamixelDriver& driver)
{
  // Compute total data length by going through all read entries
  total_data_length_ = 0;
  for (std::map<std::string, std::vector<ReadEntry>>::value_type& read_kv: read_entries_) {
    std::string register_name = read_kv.first;

    // Check if data length of register is the same for each servo
    uint8_t register_data_length = 0;
    for (std::vector<ReadEntry>::value_type& read_entry: read_kv.second) {
      uint8_t length = read_entry.dxl->getItem(register_name).data_length();
      if (register_data_length == 0) {
        register_data_length = length;
      } else {
        if (register_data_length != length) {
          ROS_ERROR_STREAM("Length mismatch for register '" << register_name << "'. "
                           "ID " << read_entry.dxl->getId() << " wants size " << length << ", current is " << register_data_length << ".");
          return false;
        }
      }
    }
    data_length_.emplace(register_name, register_data_length);
    total_data_length_ += register_data_length;
  }

  // Request indirect address space from dynamixel_driver
  if (!driver.requestIndirectAddresses(total_data_length_, indirect_address_index_)) {
    ROS_ERROR_STREAM("Failed to aquire indirect addresses for register with data length of " << total_data_length_ << ".");
    return false;
  }

  // Write register addresses to indirect addresses
  unsigned int current_indirect_address_index = indirect_address_index_;
  for (std::map<std::string, std::vector<ReadEntry>>::value_type& read_kv: read_entries_) {
    std::string register_name = read_kv.first;
    register_indirect_index_map_.emplace(register_name, current_indirect_address_index);

    bool first = true;
    for (const std::vector<ReadEntry>::value_type& read_entry: read_kv.second) {
      uint16_t indirect_address_start, indirect_data_start;
      read_entry.dxl->setIndirectAddress(current_indirect_address_index, register_name, indirect_address_start, indirect_data_start);
      if (first) {
        indirect_data_start_ = indirect_data_start;
      }
    }
    current_indirect_address_index += data_length_.at(register_name);
  }

  // Create sync read group
  sync_read_ = driver.setSyncRead(indirect_data_start_, total_data_length_);
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
  int dxl_comm_result = sync_read_->txRxPacket();

  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR_STREAM("Sync Read failed with error code: " << dxl_comm_result);
    return false;
  }

  for (std::map<std::string, std::vector<ReadEntry>>::value_type& read_kv: read_entries_) {
    std::string register_name = read_kv.first;

    // TODO
    uint16_t register_data_address;
    uint8_t register_data_length;
    for (std::vector<ReadEntry>::value_type& read_entry: read_kv.second) {
      if (sync_read_->isAvailable(read_entry.dxl->getId(), register_data_address, register_data_length)) {
        int32_t value = static_cast<int32_t>(sync_read_->getData(read_entry.dxl->getId(), register_data_address, register_data_length));
        *read_entry.value = read_entry.dxl->dxlValueToUnit(register_name, value);
      }
    }
  }

  return true;
}

}
