#include "dynamixel_ros_control/log.hpp"

#include <cmath>
#include <optional>
#include <dynamixel_ros_control/sync_write_manager.hpp>

namespace dynamixel_ros_control {

bool SyncWriteManager::addRegister(Dynamixel& dxl, const std::string& register_name, double& value, const double ratio)
{
  const auto entry = addEntry(dxl, register_name);
  if (!entry) {
    binding_rejected_ = true;
    return false;
  }
  entry->get().d_value = &value;
  if (ratio != 0.0) {
    entry->get().ratio = ratio;
  }
  return true;
}

bool SyncWriteManager::addRegister(Dynamixel& dxl, const std::string& register_name, bool& value)
{
  // Checked before the entry exists, so a rejection leaves nothing behind: a motor carrying an
  // entry with no bound value would encode zeroes through its indirect slots every cycle.
  const ControlTableItem* item = dxl.findItem(register_name);
  if (item != nullptr && item->unit() != "bool") {
    DXL_LOG_ERROR("Register '" << register_name << "' is not of type bool. Not adding write entry.");
    binding_rejected_ = true;
    return false;
  }
  const auto entry = addEntry(dxl, register_name);
  if (!entry) {
    binding_rejected_ = true;
    return false;
  }
  entry->get().b_value = &value;
  return true;
}

bool SyncWriteManager::addRegister(Dynamixel& dxl, const std::string& register_name, int32_t& value)
{
  const auto entry = addEntry(dxl, register_name);
  if (!entry) {
    binding_rejected_ = true;
    return false;
  }
  entry->get().i_value = &value;
  return true;
}

bool SyncWriteManager::init(DynamixelDriver& driver)
{
  if (binding_rejected_) {
    DXL_LOG_ERROR("Refusing to build a sync write group: a register could not be bound. The group would write "
                  "without complaint while the rejected register never reaches the motor.");
    return false;
  }
  if (write_entries_.empty()) {
    return true;
  }
  driver_ = &driver;
  total_data_length_ = 0;
  // All motors must register the same byte layout: the sync write sends total_data_length_ bytes
  // to every ID, so a shorter motor would get padding bytes written through unmapped indirect
  // slots (hitting whatever register a stale pointer still points to).
  bool first_motor = true;
  for (const auto& [dxl, write_entries] : write_entries_) {
    unsigned int dxl_data_length = 0;
    for (const auto& entry : write_entries) {
      dxl_data_length += entry.data_length;
    }
    if (first_motor) {
      total_data_length_ = dxl_data_length;
      first_motor = false;
    } else if (dxl_data_length != total_data_length_) {
      DXL_LOG_ERROR("Sync write layout mismatch: motor ID "
                    << dxl->getIdInt() << " registers " << dxl_data_length << " bytes, other motors "
                    << total_data_length_ << " bytes. All joints in one sync write group need the same "
                    << "command interface byte layout.");
      return false;
    }
  }

  indirect_address_index_ = driver.requestIndirectAddresses(total_data_length_);

  // Register indirect addresses. Every motor must lay its registers out as one contiguous range
  // starting at the same address, because that single range is what the sync write sends to all of
  // them: a motor that starts elsewhere, or leaves a gap, silently receives the wrong registers.
  std::optional<uint16_t> group_start;
  for (auto& [dxl, write_entries] : write_entries_) {
    unsigned int indirect_address_index = indirect_address_index_;
    std::optional<uint16_t> expected_address;
    for (auto& entry : write_entries) {
      if (!dxl->setIndirectAddress(indirect_address_index, entry.register_name, entry.indirect_data_address)) {
        DXL_LOG_ERROR("Failed to set indirect address mapping");
        return false;
      }
      if (!group_start) {
        group_start = entry.indirect_data_address;
      }
      if (!expected_address) {
        expected_address = *group_start;  // every motor starts where the first one did
      }
      if (entry.indirect_data_address != *expected_address) {
        DXL_LOG_ERROR("Indirect data range is not contiguous at register '"
                      << entry.register_name << "' on ID " << dxl->getIdInt() << " (address "
                      << entry.indirect_data_address << ", expected " << *expected_address
                      << "). Either the group straddles two indirect address blocks, or this motor's "
                      << "layout differs from the others - mixed layouts are not supported in one group.");
        return false;
      }
      expected_address = static_cast<uint16_t>(*expected_address + entry.data_length);
      indirect_address_index += entry.data_length;
    }
  }

  // Create sync write group
  indirect_data_address_ = group_start.value_or(0);  // retained so writeSelected can rebuild a group
  sync_write_ = driver.setSyncWrite(indirect_data_address_, total_data_length_);

  std::vector<unsigned char> tmp(total_data_length_, 0);  // Will not be used
  for (auto& [dxl, write_entries] : write_entries_) {
    if (!sync_write_->addParam(dxl->getId(), &tmp[0])) {
      // The group rejects an id it already holds. Every motor here is a distinct Dynamixel, and a
      // mimic joint never gets one, so a rejection means two joints carry the same 'id' parameter.
      DXL_LOG_ERROR("Motor ID " << dxl->getIdInt()
                                << " is already part of this sync write group. Two joints are configured with the "
                                   "same 'id' parameter. Give each joint the id of the motor it drives.");
      // The group holds only the motors added so far. Drop it so write() refuses it instead of
      // commanding that subset.
      sync_write_.reset();
      return false;
    }
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

void SyncWriteManager::encodeMotor(Dynamixel* dxl, std::vector<WriteEntry>& entries, std::vector<unsigned char>& buffer)
{
  buffer.assign(total_data_length_, 0);
  auto* out = buffer.data();
  for (auto& entry : entries) {
    std::optional<int32_t> dxl_value;
    if (entry.d_value) {
      const double unit_value = *entry.d_value;
      if (!std::isfinite(unit_value)) {
        // NaN is the ordinary "no command" of a released interface, infinity a broken command.
        // Either keeps the last written value. Reported once per entry, this runs at the control rate.
        if (!entry.non_finite_reported) {
          if (std::isnan(unit_value)) {
            DXL_LOG_DEBUG("No command for register '" << entry.register_name << "' on ID " << dxl->getIdInt()
                                                      << ". Keeping last written value.");
          } else {
            DXL_LOG_ERROR("Infinite command for register '" << entry.register_name << "' on ID " << dxl->getIdInt()
                                                            << ". Keeping last written value.");
          }
          entry.non_finite_reported = true;
        }
        dxl_value = entry.last_written_value;
      } else {
        entry.non_finite_reported = false;
        dxl_value = unitToDxlValue(unit_value, entry.ratio);
      }
    } else if (entry.i_value) {
      dxl_value = *entry.i_value;
    } else if (entry.b_value) {
      dxl_value = *entry.b_value ? 1 : 0;  // type validated in addRegister
    }
    // The bus would send a wider value reduced modulo the register width, a setpoint somewhere
    // else entirely. Keep the last value that did fit, reported once per entry.
    if (dxl_value && fitsInRegister(*dxl_value, entry.data_length, entry.encoding)) {
      entry.out_of_range_reported = false;
      entry.last_written_value = *dxl_value;
    } else if (!entry.out_of_range_reported) {
      DXL_LOG_ERROR("Command " << (entry.d_value ? *entry.d_value : static_cast<double>(dxl_value.value_or(0)))
                               << " does not fit register '" << entry.register_name << "' ("
                               << static_cast<unsigned int>(entry.data_length) << " bytes) on ID " << dxl->getIdInt()
                               << ". Keeping the last value that fitted.");
      entry.out_of_range_reported = true;
    }
    packRegisterValue(out, entry.last_written_value, entry.data_length);
    out += entry.data_length;
  }
}

bool SyncWriteManager::writeSelected(const std::set<Dynamixel*>& motors)
{
  if (write_entries_.empty() || motors.empty()) {
    return true;
  }
  if (!sync_write_) {
    DXL_LOG_ERROR("Sync write manager is not initialized. Dropping selective write.");
    return false;
  }

  // Temporary group over the same indirect layout. The cyclic group and the consecutive-error
  // counter stay untouched, so emergency writes cannot skew the control loop's error accounting.
  const auto group = driver_->setSyncWrite(indirect_data_address_, total_data_length_);

  bool any_added = false;
  for (Dynamixel* dxl : motors) {
    const auto it = write_entries_.find(dxl);
    if (it == write_entries_.end()) {
      continue;  // not registered with this manager
    }
    encodeMotor(dxl, it->second, encode_buffer_);
    if (!group->addParam(dxl->getId(), encode_buffer_.data())) {
      DXL_LOG_ERROR("Failed to add ID " << dxl->getIdInt() << " to a temporary sync write group.");
      return false;
    }
    any_added = true;
  }
  if (!any_added) {
    return true;
  }

  driver_->countGroupWrite();
  const int result = group->txPacket();
  if (result != COMM_SUCCESS) {
    DXL_LOG_ERROR("Selective sync write failed: " << driver_->communicationErrorToString(result));
    return false;
  }
  return true;
}

bool SyncWriteManager::write()
{
  if (write_entries_.empty()) {
    return true;
  }
  if (!sync_write_) {
    DXL_LOG_ERROR("Sync write manager is not initialized. Dropping write.");
    return false;
  }
  for (auto& [dxl, write_entries] : write_entries_) {
    encodeMotor(dxl, write_entries, encode_buffer_);
    if (!sync_write_->changeParam(dxl->getId(), encode_buffer_.data())) {
      // init() puts every motor of write_entries_ into the group, so the id is present here. The
      // group keeps the previous cycle's payload for a motor it cannot update, so dropping the
      // packet is the only way this does not become a stale goal sent at the control rate.
      DXL_LOG_ERROR("Motor ID " << dxl->getIdInt()
                                << " is not part of the sync write group. Dropping the write instead of sending the "
                                   "previous cycle's values.");
      return false;
    }
  }

  driver_->countGroupWrite();
  last_comm_result_ = sync_write_->txPacket();
  if (last_comm_result_ != COMM_SUCCESS) {
    subsequent_error_count_++;
    return false;
  }
  subsequent_error_count_ = 0;
  return true;
}

std::string SyncWriteManager::lastError() const
{
  return driver_->communicationErrorToString(last_comm_result_);
}

bool SyncWriteManager::isOk() const
{
  return subsequent_error_count_ < COMM_ERROR_THRESHOLD;
}

std::optional<std::reference_wrapper<WriteEntry>> SyncWriteManager::addEntry(Dynamixel& dxl,
                                                                             const std::string& register_name)
{
  WriteEntry entry;
  entry.register_name = register_name;
  const ControlTableItem* item = dxl.findItem(register_name);
  if (item == nullptr) {
    DXL_LOG_ERROR("Unknown register '" << register_name << "'. Failed to add write entry");
    return {};
  }
  if (!item->writable()) {
    // A sync write carries no status, so a read-only register in the group is dropped by every
    // motor without a word. The joint would look commanded and never move.
    DXL_LOG_ERROR("Register '" << register_name << "' is read-only and cannot be written. Failed to add write entry "
                               << "for ID " << dxl.getIdInt() << ".");
    return {};
  }
  entry.data_length = item->data_length();
  entry.ratio = item->dxlValueToUnitRatio();
  entry.encoding = item->encoding();
  std::vector<WriteEntry>& dxl_write_entries = write_entries_[&dxl];
  dxl_write_entries.push_back(std::move(entry));
  return dxl_write_entries.back();
}

}  // namespace dynamixel_ros_control
