#include <dynamixel_ros_control/sync_write_manager.h>

namespace dynamixel_ros_control {

SyncWriteManager::SyncWriteManager()
  : indirect_data_address_(0), data_length_(0) {}

void SyncWriteManager::addRegister(Dynamixel& dxl, std::string register_name, double& value)
{
  // Check if entry for dynamixel exists already
  for (const std::vector<WriteEntry>::value_type& entry: write_entries_) {
    if (dxl.getId() == entry.dxl->getId()) {
      ROS_ERROR_STREAM("A write entry for dynamixel ID " << dxl.getId() << " exists already.");
      return;
    }
  }

  // Check if data length matches
  if (data_length_ == 0) {
    data_length_ = dxl.getItem(register_name).data_length();
  } else {
    if (data_length_ != dxl.getItem(register_name).data_length()) {
      ROS_ERROR_STREAM("Data length of register '" << register_name << "' does not match the data length of previous write entries.");
      return;
    }
  }

  WriteEntry entry;
  entry.dxl = &dxl;
  entry.register_name = register_name;
  entry.value = &value;
  write_entries_.push_back(entry);
}

void SyncWriteManager::addRegister(const Dynamixel& dxl, std::string register_name, bool& value)
{

}

bool SyncWriteManager::init(DynamixelDriver& driver)
{
  // Request indirect address space from dynamixel_driver
  if (!driver.requestIndirectAddresses(data_length_, indirect_address_index_)) {
    ROS_ERROR_STREAM("Failed to aquire indirect addresses for register with data length of " << data_length_ << ".");
    return false;
  }

  bool first = true;
  for (std::vector<WriteEntry>::value_type& entry: write_entries_) {
    uint16_t indirect_data_address;
    entry.dxl->setIndirectAddress(indirect_address_index_, entry.register_name, indirect_data_address);
    if (first) { // Save address of first dynamixel, should be the same for every one
      indirect_data_address_ = indirect_data_address;
      first = false;
    }
  }

  // Create sync write group
  sync_write_ = driver.setSyncWrite(indirect_data_address_, data_length_);
  if (!sync_write_) {
    ROS_ERROR_STREAM("Failed to initialize GroupSyncWrite.");
    return false;
  }

  std::vector<unsigned char> tmp(data_length_, 0); // Will not be used
  for (std::vector<WriteEntry>::value_type& entry: write_entries_)
  {
    if (!sync_write_->addParam(entry.dxl->getId(), &tmp[0]))
      return false;
  }
  return true;
}

bool SyncWriteManager::write()
{
  // Convert values and update params
  for (std::vector<WriteEntry>::value_type& entry: write_entries_) {
    int32_t dxl_value = entry.dxl->unitToDxlValue(entry.register_name, *entry.value);
    unsigned char* value_ptr = reinterpret_cast<unsigned char*>(&dxl_value);
    sync_write_->changeParam(entry.dxl->getId(), value_ptr);
  }
  int result = sync_write_->txPacket();
  if (result != COMM_SUCCESS) {
    ROS_ERROR_STREAM("Sync Write failed.");
    return false;
  }
  return true;
}

}
