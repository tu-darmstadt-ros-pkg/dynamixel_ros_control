#include "dynamixel_ros_control/log.hpp"

#include <cstring>
#include <optional>
#include <set>
#include <dynamixel_ros_control/common.hpp>
#include <dynamixel_ros_control/sync_read_manager.hpp>

namespace dynamixel_ros_control {

void SyncReadManager::addDynamixel(Dynamixel* dxl)
{
  std::pair<std::set<Dynamixel*>::iterator, bool> result = dynamixels_.emplace(dxl);
  if (!result.second) {
    DXL_LOG_WARN("Already contained dynamixel with id " << dxl->getIdInt());
  }
}

bool SyncReadManager::addRegister(std::string register_name, const DxlValueMappingList& dxl_value_pairs,
                                  const std::string& padding_register)
{
  if (read_entries_.find(register_name) != read_entries_.end()) {
    DXL_LOG_ERROR("Register '" << register_name << "' has already been added.");
    return false;
  }
  if (dxl_value_pairs.size() != dynamixels_.size()) {
    DXL_LOG_ERROR("Number of Dynamixel-Value pairs ("
                  << dxl_value_pairs.size() << ") does not match number of dynamixels (" << dynamixels_.size()
                  << "). Make sure to add all dynamixels before adding registers to the SyncReadManager.");
    return false;
  }

  ReadEntry entry;
  entry.register_name = register_name;
  // Taken from the control table item below rather than classified here a second time, so both
  // decode paths read the same source.
  std::optional<bool> entry_is_signed;
  entry.dxl_value_pairs = dxl_value_pairs;
  entry.motor_registers.reserve(dxl_value_pairs.size());
  entry.is_padding.reserve(dxl_value_pairs.size());
  entry.ratios.reserve(dxl_value_pairs.size());

  uint8_t register_data_length = 0;
  for (const auto& [dxl, value] : dxl_value_pairs) {
    if (dynamixels_.find(dxl) == dynamixels_.end()) {
      DXL_LOG_ERROR("Dynamixel with id " << dxl->getIdInt() << " is unknown to the SyncReadManager.");
      return false;
    }
    // Motors without the register still have to contribute the same bytes to the group's range.
    std::string mapped = register_name;
    if (!dxl->registerAvailable(register_name)) {
      if (padding_register.empty() || !dxl->registerAvailable(padding_register)) {
        DXL_LOG_ERROR("Register '" << register_name << "' does not exist on ID " << dxl->getIdInt() << " (model "
                                   << dxl->getModelNumber()
                                   << "), and no padding register is available to fill its slot.");
        return false;
      }
      mapped = padding_register;
    }
    const ControlTableItem& item = dxl->getItem(mapped);
    const uint8_t length = item.data_length();
    if (register_data_length == 0) {
      register_data_length = length;
    } else if (register_data_length != length) {
      DXL_LOG_ERROR("Length mismatch for register '"
                    << register_name << "'. ID " << dxl->getIdInt() << " maps '" << mapped << "' with size "
                    << static_cast<int>(length) << ", current is " << static_cast<int>(register_data_length) << ".");
      return false;
    }
    const bool is_padding = mapped != register_name;
    // A bool destination demands a bool register, checked here rather than per cycle: the cyclic
    // read cannot refuse a binding, it can only log the same complaint at the control rate.
    if (!is_padding && value.bvalue && item.unit() != "bool") {
      DXL_LOG_ERROR("Register '" << register_name << "' (type=" << item.unit() << ") on ID " << dxl->getIdInt()
                                 << " is bound to a bool value but is not of type bool.");
      return false;
    }
    if (!is_padding) {
      // Every motor must agree. They do by construction while the encoding is keyed by register
      // name, but that is the assumption worth failing on rather than the one worth relying on.
      if (entry_is_signed && *entry_is_signed != item.is_signed()) {
        DXL_LOG_ERROR("Models disagree on whether register '" << register_name << "' is signed. ID " << dxl->getIdInt()
                                                              << " reads it the other way.");
        return false;
      }
      entry_is_signed = item.is_signed();
    }
    entry.motor_registers.push_back(std::move(mapped));
    entry.is_padding.push_back(is_padding);
    entry.ratios.push_back(!is_padding && value.ratio != 0.0 ? value.ratio : item.dxlValueToUnitRatio());
  }

  // Only padding slots would remain, and every one of those is discarded before it is decoded.
  entry.is_signed = entry_is_signed.value_or(false);
  entry.data_length = register_data_length;
  read_entries_.emplace(register_name, entry);
  return true;
}

bool SyncReadManager::init(DynamixelDriver& driver)
{
  driver_ = &driver;
  // Compute total data length by going through all read entries
  total_data_length_ = 0;
  for (const auto& [register_name, read_entry] : read_entries_) {
    total_data_length_ += read_entry.data_length;
  }

  indirect_address_index_ = driver.requestIndirectAddresses(total_data_length_);

  // Write register addresses to indirect addresses
  unsigned int current_indirect_address_index = indirect_address_index_;
  bool first_register = true;
  for (auto& [register_name, read_entry] : read_entries_) {
    read_entry.indirect_index = current_indirect_address_index;

    bool first_dxl = true;
    for (unsigned int i = 0; i < read_entry.dxl_value_pairs.size(); ++i) {
      Dynamixel* dxl = read_entry.dxl_value_pairs[i].first;
      uint16_t indirect_data_address;
      if (!dxl->setIndirectAddress(current_indirect_address_index, read_entry.motor_registers[i],
                                   indirect_data_address)) {
        DXL_LOG_ERROR("Failed to set indirect address mapping");
        return false;
      }
      if (first_register) {
        indirect_data_address_ = indirect_data_address;
        first_register = false;
      }
      if (first_dxl) {
        read_entry.indirect_data_address = indirect_data_address;
        first_dxl = false;
      } else if (indirect_data_address != read_entry.indirect_data_address) {
        // The sync read uses ONE start address for all IDs. Differing per-motor layouts
        // (e.g. mixed series in one group) would silently read the wrong registers.
        DXL_LOG_ERROR("Register '" << register_name << "' maps to indirect data address " << indirect_data_address
                                   << " on ID " << dxl->getIdInt() << " but to " << read_entry.indirect_data_address
                                   << " on other motors. Mixed indirect layouts are not supported in one group.");
        return false;
      }
    }
    current_indirect_address_index += read_entry.data_length;
  }

  // The registers must form one contiguous data range — that range is what the sync read fetches.
  uint16_t expected_address = indirect_data_address_;
  for (const auto& [register_name, read_entry] : read_entries_) {
    if (read_entry.indirect_data_address != expected_address) {
      DXL_LOG_ERROR("Indirect data range is not contiguous at register '"
                    << register_name << "' (address " << read_entry.indirect_data_address << ", expected "
                    << expected_address << "). The group likely straddles two indirect address blocks.");
      return false;
    }
    expected_address += read_entry.data_length;
  }

  sync_read_ = driver.setSyncRead(indirect_data_address_, total_data_length_);

  for (const auto& dxl : dynamixels_) {
    if (!sync_read_->addParam(dxl->getId()))
      return false;
  }
  return true;
}

bool SyncReadManager::rewriteIndirectAddresses(const std::set<Dynamixel*>& motors)
{
  if (motors.empty() || read_entries_.empty()) {
    return true;
  }
  for (auto& [register_name, read_entry] : read_entries_) {
    const unsigned int indirect_address_index = read_entry.indirect_index;
    for (unsigned int i = 0; i < read_entry.dxl_value_pairs.size(); ++i) {
      Dynamixel* dxl = read_entry.dxl_value_pairs[i].first;
      if (motors.find(dxl) == motors.end()) {
        continue;
      }
      uint16_t indirect_data_address;
      if (!dxl->setIndirectAddress(indirect_address_index, read_entry.motor_registers[i], indirect_data_address)) {
        DXL_LOG_ERROR("Failed to rewrite indirect address mapping for register '" << register_name << "' on motor ID "
                                                                                  << dxl->getIdInt() << ".");
        return false;
      }
    }
  }
  return true;
}

bool SyncReadManager::read()
{
  if (read_entries_.empty()) {
    return true;
  }
  missing_ids_.clear();
  if (!sync_read_) {
    DXL_LOG_ERROR("Sync read manager is not initialized. Dropping read.");
    return false;
  }
  driver_->countGroupRead();
  last_comm_result_ = sync_read_->txRxPacket();
  if (last_comm_result_ != COMM_SUCCESS) {
    subsequent_error_count_++;
    return false;
  }

  // Motors whose status packet was lost: their values stay stale and the cycle counts as failed.
  decodeGroup(*sync_read_, nullptr, missing_ids_);
  if (!missing_ids_.empty()) {
    subsequent_error_count_++;
    return false;
  }

  subsequent_error_count_ = 0;
  return true;
}

std::string SyncReadManager::lastError() const
{
  if (last_comm_result_ != COMM_SUCCESS) {
    return driver_->communicationErrorToString(last_comm_result_);
  }
  return "no response from IDs " + iterableToString(missing_ids_);
}

void SyncReadManager::decodeGroup(GroupSyncRead& group, const std::set<Dynamixel*>* only,
                                  std::set<unsigned int>& missing_ids)
{
  for (auto& [register_name, entry] : read_entries_) {
    const uint16_t register_data_address = entry.indirect_data_address;
    const uint8_t register_data_length = entry.data_length;
    for (unsigned int i = 0; i < entry.dxl_value_pairs.size(); i++) {
      auto& [dxl, value] = entry.dxl_value_pairs[i];
      if (only && only->find(dxl) == only->end()) {
        continue;  // not part of this selective read
      }
      if (group.isAvailable(dxl->getId(), register_data_address, register_data_length)) {
        // Padding slot: this motor lacks the register, so its bytes are not this value.
        if (entry.is_padding[i]) {
          continue;
        }
        const uint32_t data = group.getData(dxl->getId(), register_data_address, register_data_length);
        // Sign-extended only where the register is classified two's complement, as in
        // DynamixelDriver::readRegister. Extending an unsigned register turns a raw 255 into -1.
        const int32_t dxl_value = unpackRegisterValue(data, register_data_length, entry.is_signed);
        if (value.dvalue) {
          *value.dvalue = static_cast<double>(dxl_value) * entry.ratios[i];
        }
        if (value.bvalue) {
          // Type checked in addRegister(). A bool register is true for any value above zero.
          *value.bvalue = dxl_value > 0;
        }
        if (value.ivalue) {
          *value.ivalue = dxl_value;
        }
      } else {
        missing_ids.insert(dxl->getIdInt());
      }
    }
  }
}

bool SyncReadManager::readSelected(const std::set<Dynamixel*>& motors)
{
  if (read_entries_.empty() || motors.empty()) {
    return true;
  }
  if (!sync_read_) {
    DXL_LOG_ERROR("Sync read manager is not initialized. Dropping selective read.");
    return false;
  }

  // Temporary group over the same indirect layout. The cyclic group and its error counter are left
  // untouched: this runs during emergency handling, where they decide when the interface gives up.
  const auto group = driver_->setSyncRead(indirect_data_address_, total_data_length_);

  std::set<Dynamixel*> requested;
  for (Dynamixel* dxl : motors) {
    if (dynamixels_.find(dxl) == dynamixels_.end()) {
      continue;  // not registered with this manager
    }
    if (!group->addParam(dxl->getId())) {
      DXL_LOG_ERROR("Failed to add ID " << dxl->getIdInt() << " to a temporary sync read group.");
      return false;
    }
    requested.insert(dxl);
  }
  if (requested.empty()) {
    return true;
  }

  driver_->countGroupRead();
  const int comm_result = group->txRxPacket();
  if (comm_result != COMM_SUCCESS) {
    // Expected while probing an unresponsive motor, so this is not an error-level event.
    DXL_LOG_DEBUG("Selective sync read failed: " << driver_->communicationErrorToString(comm_result));
    return false;
  }

  std::set<unsigned int> missing_ids;
  decodeGroup(*group, &requested, missing_ids);
  if (!missing_ids.empty()) {
    DXL_LOG_DEBUG("Selective sync read incomplete: no response data from IDs " << iterableToString(missing_ids));
    return false;
  }
  return true;
}

std::set<Dynamixel*> SyncReadManager::readWhoeverAnswers(const std::set<Dynamixel*>& motors)
{
  if (motors.empty()) {
    return {};
  }
  // Callers turn this set into a safety claim, and readSelected() reports success for a group with
  // nothing to read. Claiming everyone answered would commit whatever the bound values hold.
  if (read_entries_.empty() || !driver_) {
    DXL_LOG_ERROR("Sync read manager has no registers to read; no motor can be said to have answered.");
    return {};
  }
  std::set<Dynamixel*> registered;
  for (Dynamixel* dxl : motors) {
    if (dynamixels_.find(dxl) != dynamixels_.end()) {
      registered.insert(dxl);
    }
  }
  if (registered.size() != motors.size()) {
    DXL_LOG_ERROR("Selective read asked for " << motors.size() << " motor(s) of which only " << registered.size()
                                              << " are registered with this manager; the rest cannot answer it.");
  }
  if (registered.empty()) {
    return {};
  }
  if (readSelected(registered)) {
    return registered;  // healthy bus: one transaction for the whole set
  }
  // Something did not answer, and the group read cannot say which. Probe individually. That is
  // expensive, but it only runs while the bus is already degraded, and nothing else finds out.
  std::set<Dynamixel*> answered;
  for (Dynamixel* dxl : registered) {
    if (readSelected({dxl})) {
      answered.insert(dxl);
    }
  }
  return answered;
}

bool SyncReadManager::isOk() const
{
  return subsequent_error_count_ < COMM_ERROR_THRESHOLD;
}

}  // namespace dynamixel_ros_control
