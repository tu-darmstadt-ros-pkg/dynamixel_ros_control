#include "dynamixel_ros_control/log.hpp"

#include <dynamixel_ros_control/sync_write_manager.hpp>
#include <sstream>

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
  for (const auto& [dxl, write_entries] : write_entries_) {
    unsigned int dxl_data_length = 0;
    for (const auto& entry : write_entries) {
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
  for (auto& [dxl, write_entries] : write_entries_) {
    unsigned int indirect_address_index = indirect_address_index_;
    for (auto& entry : write_entries) {
      entry.indirect_index = indirect_address_index;
      if (!dxl->setIndirectAddress(indirect_address_index, entry.register_name, entry.indirect_data_address)) {
        DXL_LOG_ERROR("Failed to set indirect address mapping");
        return false;
      }
      if (first) {
        first = false;
        total_indirect_data_address = entry.indirect_data_address;  // Should be the same for every servo
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
  for (auto& [dxl, write_entries] : write_entries_) {
    if (!sync_write_->addParam(dxl->getId(), &tmp[0]))
      return false;
  }
  return true;
}

bool SyncWriteManager::rewriteIndirectAddresses(const std::set<Dynamixel*>& motors)
{
  if (motors.empty() || write_entries_.empty()) {
    return true;
  }
  for (auto& [dxl, dxl_write_entries] : write_entries_) {
    if (motors.find(dxl) == motors.end()) {
      continue;
    }
    unsigned int indirect_address_index = indirect_address_index_;
    for (auto& entry : dxl_write_entries) {
      if (!dxl->setIndirectAddress(indirect_address_index, entry.register_name, entry.indirect_data_address)) {
        DXL_LOG_ERROR("Failed to rewrite indirect address mapping for register '"
                      << entry.register_name << "' on motor ID " << dxl->getIdInt() << ".");
        return false;
      }
      indirect_address_index += entry.data_length;
    }
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
  for (auto& [dxl, write_entries] : write_entries_) {
    std::vector<unsigned char> write_value(total_data_length_, 0);
    auto* buffer = write_value.data();
    for (auto& entry : write_entries) {
      int32_t dxl_value;
      if (entry.d_value) {
        const double unit_value = *entry.d_value + entry.offset;
        dxl_value = dxl->unitToDxlValue(entry.register_name, unit_value);
        DXL_LOG_DEBUG("[WRITING " << entry.register_name << "] id " << dxl->getIdInt() << ", value: " << dxl_value
                                  << ", converted: " << *entry.d_value
                                  << ", indirect_data_address: " << entry.indirect_data_address);
      } else if (entry.b_value) {
        dxl_value = dxl->boolToDxlValue(entry.register_name, *entry.b_value);
        DXL_LOG_DEBUG("[WRITING " << entry.register_name << "] id " << dxl->getIdInt() << ", value: " << dxl_value
                                  << ", converted: " << *entry.b_value
                                  << ", indirect_data_address: " << entry.indirect_data_address);
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
    std::ostringstream ids;
    for (const auto& [dxl, write_entries] : write_entries_) {
      ids << " id" << dxl->getIdInt() << "[";
      for (const auto& entry : write_entries) {
        ids << entry.register_name << "@" << entry.indirect_data_address << ",";
      }
      ids << "]";
    }
    DXL_LOG_ERROR("Sync Write failed with error: " << driver_->communicationErrorToString(result)
                                                   << ". Affected motors/registers:" << ids.str());
    subsequent_error_count_++;
    return false;
  }
  subsequent_error_count_ = 0;
  return true;
}

std::vector<IndirectWriteDebugEntry>
SyncWriteManager::getIndirectDebugEntries(const Dynamixel& dxl, const std::vector<std::string>& register_names) const
{
  std::vector<IndirectWriteDebugEntry> debug_entries;
  debug_entries.reserve(register_names.size());
  const auto it = write_entries_.find(const_cast<Dynamixel*>(&dxl));
  if (it == write_entries_.end()) {
    return debug_entries;
  }
  const auto& entries = it->second;
  for (const auto& register_name : register_names) {
    const auto entry_it = std::find_if(entries.begin(), entries.end(), [&register_name](const auto& entry) {
      return entry.register_name == register_name;
    });
    if (entry_it == entries.end()) {
      continue;
    }
    debug_entries.push_back(
        {entry_it->register_name, entry_it->indirect_index, entry_it->indirect_data_address, entry_it->data_length});
  }
  return debug_entries;
}

bool SyncWriteManager::isOk() const
{
  return subsequent_error_count_ < error_threshold_;
}

void SyncWriteManager::setErrorThreshold(const unsigned int threshold)
{
  error_threshold_ = threshold;
}

std::optional<std::reference_wrapper<WriteEntry>> SyncWriteManager::addEntry(Dynamixel& dxl,
                                                                             const std::string& register_name)
{
  WriteEntry entry;
  entry.register_name = register_name;
  try {
    entry.data_length = dxl.getItem(register_name).data_length();
  }
  catch (const std::out_of_range&) {
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
