#include <dynamixel_ros_control/sync_write_manager.h>

namespace dynamixel_ros_control {

SyncWriteManager::SyncWriteManager()
  : indirect_data_address_(0), data_length_(0), subsequent_error_count_(0), error_threshold_(25) {}

void SyncWriteManager::addRegister(Dynamixel& dxl, std::string register_name, double& value, double offset)
{
  std::vector<WriteEntry>::iterator it = addEntry(dxl, register_name);
  if (it != write_entries_.end()) {
    it->d_value = &value;
    it->offset = offset;
  }
}

void SyncWriteManager::addRegister(Dynamixel& dxl, std::string register_name, bool& value)
{
  std::vector<WriteEntry>::iterator it = addEntry(dxl, register_name);
  if (it != write_entries_.end()) {
    it->b_value = &value;
  }
}

bool SyncWriteManager::init(DynamixelDriver& driver)
{
  driver_ = &driver;
  // Request indirect address space from dynamixel_driver
  if (!driver.requestIndirectAddresses(data_length_, indirect_address_index_)) {
    ROS_ERROR_STREAM("Failed to aquire indirect addresses for register with data length of " << data_length_ << ".");
    return false;
  }

  bool first = true;
  for (std::vector<WriteEntry>::value_type& entry: write_entries_) {
    uint16_t indirect_data_address;
    if (!entry.dxl->setIndirectAddress(indirect_address_index_, entry.register_name, indirect_data_address)) {
      ROS_ERROR_STREAM("Failed to set indirect address mapping");
      return false;
    }
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
  for (const std::vector<WriteEntry>::value_type& entry: write_entries_) {
    int32_t dxl_value;
    if (entry.d_value != nullptr) {
      double unit_value = *entry.d_value + entry.offset;
      dxl_value = entry.dxl->unitToDxlValue(entry.register_name, unit_value);
      ROS_DEBUG_STREAM_THROTTLE(0.25, "[WRITING " << entry.register_name << "] id " << entry.dxl->getId() << "value: " << dxl_value << ", converted: " << *entry.d_value);
    } else if (entry.b_value != nullptr) {
      dxl_value = entry.dxl->boolToDxlValue(entry.register_name, *entry.b_value);
      ROS_DEBUG_STREAM_THROTTLE(0.25, "[WRITING " << entry.register_name << "] id " << entry.dxl->getId() << "value: " << dxl_value << ", converted: " << *entry.b_value);
    }
    unsigned char* value_ptr;
    int16_t value_16bit;
    int8_t value_8bit;
    if (data_length_ == 4) {
      value_ptr = reinterpret_cast<unsigned char*>(&dxl_value);
    } else if (data_length_ == 2) {
      value_16bit = static_cast<int16_t>(dxl_value);
      value_ptr = reinterpret_cast<unsigned char*>(&value_16bit);
    } else if (data_length_ == 1) {
      value_8bit = static_cast<int8_t>(dxl_value);
      value_ptr = reinterpret_cast<unsigned char*>(&value_8bit);
    } else {
      ROS_ERROR_STREAM("Unsupported data length: " << data_length_);
      value_ptr = reinterpret_cast<unsigned char*>(&dxl_value);
    }
    sync_write_->changeParam(entry.dxl->getId(), value_ptr);
  }
  
  int result = sync_write_->txPacket();
  if (result != COMM_SUCCESS) {
    ROS_ERROR_STREAM("Sync Write failed with error: " << driver_->communicationErrorToString(result));
    subsequent_error_count_++;
    return false;
  }
  subsequent_error_count_ = 0;
  return true;
}

bool SyncWriteManager::isOk() const
{
  return subsequent_error_count_ < error_threshold_;
}

void SyncWriteManager::setErrorThreshold(unsigned int threshold)
{
  error_threshold_ = threshold;
}

std::vector<WriteEntry>::iterator SyncWriteManager::addEntry(Dynamixel& dxl, std::string register_name)
{
  // Check if entry for dynamixel exists already
  for (const std::vector<WriteEntry>::value_type& entry: write_entries_) {
    if (dxl.getId() == entry.dxl->getId()) {
      ROS_ERROR_STREAM("A write entry for dynamixel ID " << static_cast<int>(dxl.getId()) << " exists already.");
      return write_entries_.end();
    }
  }

  // Check if data length matches
  uint8_t register_data_length;
  try {
    register_data_length = dxl.getItem(register_name).data_length();
  } catch(const std::out_of_range&) {
    ROS_ERROR_STREAM("Failed to add write entry");
    return write_entries_.end();
  }

  if (data_length_ == 0) {
    data_length_ = register_data_length;
  } else {
    if (data_length_ != register_data_length) {
      ROS_ERROR_STREAM("Data length of register '" << register_name << "' does not match the data length of previous write entries.");
      return write_entries_.end();
    }
  }

  WriteEntry entry;
  entry.dxl = &dxl;
  entry.register_name = register_name;
  write_entries_.push_back(entry);
  return std::prev(write_entries_.end());
}

}
