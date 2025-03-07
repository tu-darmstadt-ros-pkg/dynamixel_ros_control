#include "dynamixel_ros_control/log.hpp"

#include <dynamixel_ros_control/sync_write_manager.hpp>

namespace dynamixel_ros_control {

void SyncWriteManager::addRegister(Dynamixel& dxl, const std::string& register_name, double& value, const double offset)
{
  if (const auto entry = addEntry(dxl, register_name)) {
    entry->get().d_value = &value;
    entry->get().offset = offset;
  }
}

void SyncWriteManager::addRegister(Dynamixel& dxl, const std::string& register_name, bool& value)
{
  if (const auto entry = addEntry(dxl, register_name)) {
    entry->get().b_value = &value;
  }
}

bool SyncWriteManager::init(DynamixelDriver& driver)
{
  if (write_entries_.empty()) {
    return true;
  }
  driver_ = &driver;
  total_data_length_ = 0;
  // Determine total data length
  for (const auto& [dxl, write_entries]: write_entries_) {
    unsigned int dxl_data_length = 0;
    for (const auto& entry: write_entries) {
      dxl_data_length += entry.data_length;
    }
    if (dxl_data_length > total_data_length_) {
      total_data_length_ = dxl_data_length;
    }
  }

  // Request indirect address space from dynamixel_driver
  if (!driver.requestIndirectAddresses(total_data_length_, indirect_address_index_)) {
    DXL_LOG_ERROR("Failed to acquire indirect addresses for register with data length of " << total_data_length_ << ".");
    return false;
  }

  // Register indirect addresses
  uint16_t total_indirect_data_address = 0;
  bool first = true;
  for (auto& [dxl, write_entries]: write_entries_) {
    unsigned int indirect_address_index = indirect_address_index_;
    for (auto& entry: write_entries) {
      if (!dxl->setIndirectAddress(indirect_address_index, entry.register_name, entry.indirect_data_address)) {
        DXL_LOG_ERROR("Failed to set indirect address mapping");
        return false;
      }
      if (first) {
        first = false;
        total_indirect_data_address = entry.indirect_data_address; // Should be the same for every servo
      }
      indirect_address_index += entry.data_length;
    }
  }

  // Create sync write group
  sync_write_ = driver.setSyncWrite(total_indirect_data_address, total_data_length_);
  if (!sync_write_) {
    DXL_LOG_ERROR("Failed to initialize GroupSyncWrite.");
    return false;
  }

  std::vector<unsigned char> tmp(total_data_length_, 0);  // Will not be used
  for (auto& [dxl, write_entries]: write_entries_) {
    if (!sync_write_->addParam(dxl->getId(), &tmp[0]))
      return false;
  }
  return true;
}

bool SyncWriteManager::release() const
{
  return total_data_length_ == 0 || driver_->releaseIndirectAddresses(total_data_length_, indirect_address_index_);
}

bool SyncWriteManager::write()
{
  if (write_entries_.empty()) {
    return true;
  }
  // Convert values and update params
  for (auto& [dxl, write_entries]: write_entries_) {
    std::vector<unsigned char> write_value(total_data_length_, 0);
    auto *buffer = write_value.data();
    for (auto& entry: write_entries) {
      int32_t dxl_value;
      if (entry.d_value) {
        const double unit_value = *entry.d_value + entry.offset;
        dxl_value = dxl->unitToDxlValue(entry.register_name, unit_value);
        DXL_LOG_DEBUG("[WRITING " << entry.register_name << "] id " << dxl->getIdInt() << ", value: " << dxl_value
                                  << ", converted: " << *entry.d_value);
      } else if (entry.b_value) {
        dxl_value = dxl->boolToDxlValue(entry.register_name, *entry.b_value);
        DXL_LOG_DEBUG("[WRITING " << entry.register_name << "] id " << dxl->getIdInt() << ", value: " << dxl_value
                                  << ", converted: " << *entry.b_value);
      } else {
        DXL_LOG_ERROR("No value set");
        dxl_value = 0;
      }
      std::memcpy(buffer, &dxl_value, entry.data_length);
      buffer += entry.data_length;
    }
    sync_write_->changeParam(dxl->getId(), write_value.data());
  }

  const int result = sync_write_->txPacket();
  if (result != COMM_SUCCESS) {
    DXL_LOG_ERROR("Sync Write failed with error: " << driver_->communicationErrorToString(result));
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

void SyncWriteManager::setErrorThreshold(const unsigned int threshold)
{
  error_threshold_ = threshold;
}

std::optional<std::reference_wrapper<WriteEntry>> SyncWriteManager::addEntry(Dynamixel& dxl, const std::string& register_name)
{
  WriteEntry entry;
  entry.register_name = register_name;
  try {
    entry.data_length = dxl.getItem(register_name).data_length();
  } catch (const std::out_of_range&) {
    DXL_LOG_ERROR("Unknown register '" << register_name << "'. Failed to add write entry");
    return {};
  }
  if (entry.data_length > sizeof(int32_t)) {
    DXL_LOG_ERROR("Data length must not exceed " << sizeof(int32_t));
    return {};
  }
  std::vector<WriteEntry>& dxl_write_entries = write_entries_[&dxl];
  dxl_write_entries.push_back(std::move(entry));
  return dxl_write_entries.back();
}

}  // namespace dynamixel_ros_control
