#include "dynamixel_ros_control/log.hpp"

#include <dynamixel_ros_control/dynamixel.hpp>
#include <dynamixel_ros_control/common.hpp>

#include <cmath>
#include <limits>
#include <thread>

namespace dynamixel_ros_control {

namespace {
constexpr int PING_MAX_ATTEMPTS = 3;
constexpr std::chrono::milliseconds PING_RETRY_DELAY{50};
}  // namespace

Dynamixel::Dynamixel(const uint8_t id, DynamixelDriver& driver)
    : driver_(driver), id_(id)
{}

bool Dynamixel::connect()
{
  bool ping_ok = false;
  for (int attempt = 1; attempt <= PING_MAX_ATTEMPTS && !ping_ok; ++attempt) {
    int comm_result;
    ping_ok = driver_.ping(id_, model_number_, comm_result);
    if (!ping_ok) {
      DXL_LOG_WARN("[ID " << getIdInt() << "] Ping attempt " << attempt << "/" << PING_MAX_ATTEMPTS
                          << " failed: " << driver_.communicationErrorToString(comm_result));
      if (attempt < PING_MAX_ATTEMPTS) {
        std::this_thread::sleep_for(PING_RETRY_DELAY);
      }
    }
  }
  if (!ping_ok) {
    DXL_LOG_ERROR("Failed to ping ID " << static_cast<int>(getId()) << " after " << PING_MAX_ATTEMPTS << " attempts");
    return false;
  }

  // Only overwritten once there is a table to overwrite it with. A reconnect whose table fails to
  // load would otherwise demote a motor that is already known, and register access on it stops.
  ControlTable* control_table = driver_.loadControlTable(getModelNumber());
  if (!control_table) {
    return false;
  }
  control_table_ = control_table;
  DXL_LOG_DEBUG("ID " << getIdInt() << ": Loaded control table for model " << getModelNumber());

  // A latched watchdog (-1) refuses the goal registers with a range error, so clear it before the
  // initial writes. UNSUPPORTED is fine, no register, no latch.
  bool watchdog_was_tripped = false;
  if (clearBusWatchdog(&watchdog_was_tripped) == WatchdogResult::FAILED) {
    DXL_LOG_ERROR("[ID " << getIdInt()
                         << "] Failed to clear bus watchdog. A latched watchdog refuses the goal registers, so "
                            "this motor cannot be brought up safely.");
    return false;
  }
  if (watchdog_was_tripped) {
    DXL_LOG_WARN("[ID " << getIdInt() << "] Bus watchdog had TRIPPED. Cleared the latched error.");
  }

  // Fatal as well: these are the configured safety-relevant registers (current limits, drive mode,
  // profiles, homing offsets). Coming up with unintended values and then energising is worse than
  // refusing to configure. Note writeInitialValues() already returns true for a register that this
  // model simply does not have, so false means a bad config value or a failed write.
  if (!writeInitialValues()) {
    DXL_LOG_ERROR("[ID " << getIdInt() << "] Failed to write the configured initial register values.");
    return false;
  }
  return true;
}

bool Dynamixel::reboot() const
{
  return driver_.reboot(getId());
}

bool Dynamixel::stringToDxlValue(const std::string& register_name, const std::string& value,
                                 int32_t& dxl_value_out) const
{
  // Get control table entry
  const ControlTableItem* item = findItem(register_name);
  if (item == nullptr) {
    DXL_LOG_ERROR("Unknown register '" << register_name << "'");
    return false;
  }
  // Check if bool
  if (item->unit() == "bool") {
    std::string value_lower_case = value;
    std::transform(value_lower_case.begin(), value_lower_case.end(), value_lower_case.begin(),
                   [](const unsigned char c) { return std::tolower(c); });

    if (value_lower_case == "true" || value_lower_case == "1") {
      return boolToDxlValue(register_name, true, dxl_value_out);
    }
    if (value_lower_case == "false" || value_lower_case == "0") {
      return boolToDxlValue(register_name, false, dxl_value_out);
    }
    DXL_LOG_ERROR("Cannot convert '" << value << "' to bool");
    return false;
  }

  // Not a bool
  double double_value;
  try {
    size_t parsed = 0;
    double_value = std::stod(value, &parsed);
    // stod stops at the first character it cannot use and reports success for what it did read, so
    // a remainder means the value was only partly understood. "2junk" must not become 2.
    if (value.find_first_not_of(" \t\n\r\f\v", parsed) != std::string::npos) {
      DXL_LOG_ERROR("Value '" << value << "' for register '" << register_name
                              << "' has trailing characters after the number.");
      return false;
    }
  }
  catch (const std::exception& e) {
    DXL_LOG_ERROR("Failed to convert value '" << value << "' to a double: " << e.what());
    return false;
  }
  // Covers a non-finite value and one whose tick count no register can carry. Substituting a
  // number for either would make it a real setting rather than an obvious refusal.
  const std::optional<int32_t> converted = unitToDxlValue(register_name, double_value);
  if (!converted) {
    DXL_LOG_ERROR("Value '" << value << "' for register '" << register_name
                            << "' has no raw equivalent. Refusing to convert it.");
    return false;
  }
  dxl_value_out = *converted;
  return true;
}

bool Dynamixel::writeRegister(const std::string& register_name, const std::string& value) const
{
  int32_t dxl_value;
  if (!stringToDxlValue(register_name, value, dxl_value)) {
    return false;
  }
  return writeRegister(register_name, dxl_value);
}

bool Dynamixel::writeRegister(const std::string& register_name, const bool value) const
{
  int32_t dxl_value;
  if (!boolToDxlValue(register_name, value, dxl_value)) {
    return false;
  }
  return writeRegister(register_name, dxl_value);
}

bool Dynamixel::writeRegister(const std::string& register_name, const double value) const
{
  // Covers a non-finite value and one whose tick count no register can carry. Both would otherwise
  // become a number that is a real setting for a limit or an offset rather than an obvious failure.
  const std::optional<int32_t> dxl_value = unitToDxlValue(register_name, value);
  if (!dxl_value) {
    DXL_LOG_ERROR("[ID " << getIdInt() << "] Refusing to write " << value << " to register '" << register_name << "'.");
    return false;
  }
  return writeRegister(register_name, *dxl_value);
}

bool Dynamixel::writeRegister(const std::string& register_name, const int32_t value) const
{
  const ControlTableItem* item = findItem(register_name);
  if (item == nullptr) {
    DXL_LOG_ERROR("Unknown register '" << register_name << "'");
    return false;
  }
  if (!item->writable()) {
    DXL_LOG_ERROR("[ID " << getIdInt() << "] Register '" << register_name
                         << "' is read-only. The motor reports the write as an instruction error and keeps its own "
                            "value, so the number written never takes effect.");
    return false;
  }
  // A wider value is not refused by the bus, it is sent reduced modulo the register width. The
  // number that arrives is in range, so the motor accepts it and the write reports success.
  if (!fitsInRegister(value, item->data_length(), item->encoding())) {
    DXL_LOG_ERROR("[ID " << getIdInt() << "] Value " << value << " does not fit register '" << register_name << "' ("
                         << static_cast<unsigned int>(item->data_length())
                         << " bytes). Refusing rather than writing it reduced to its width.");
    return false;
  }
  const auto result = driver_.writeRegister(getId(), item->address(), item->data_length(), value);
  if (!result) {
    DXL_LOG_ERROR("[ID " << getIdInt() << "] Failed to write register '" << register_name << "' (addr="
                         << item->address() << ", value=" << value << "): " << driver_.describeError(result));
  }
  return result.success();
}

bool Dynamixel::writeRegister(const uint16_t address, const uint8_t data_length, const int32_t value) const
{
  const auto result = driver_.writeRegister(getId(), address, data_length, value);
  if (!result) {
    DXL_LOG_ERROR("[ID " << getIdInt() << "] Failed to write addr=" << address
                         << " (len=" << static_cast<int>(data_length) << ", value=" << value
                         << "): " << driver_.describeError(result));
  }
  return result.success();
}

bool Dynamixel::readRegister(const std::string& register_name, double& value_out) const
{
  int32_t dxl_value;
  if (!readRegister(register_name, dxl_value)) {
    return false;
  }
  value_out = dxlValueToUnit(register_name, dxl_value);
  return true;
}

bool Dynamixel::readRegister(const std::string& register_name, bool& value_out) const
{
  int32_t dxl_value;
  if (!readRegister(register_name, dxl_value)) {
    return false;
  }
  const std::optional<bool> as_bool = dxlValueToBool(register_name, dxl_value);
  if (!as_bool) {
    return false;  // false is a legitimate reading, so it cannot double as the failure
  }
  value_out = *as_bool;
  return true;
}

bool Dynamixel::readRegister(const std::string& register_name, int32_t& value_out) const
{
  // Silent: callers probe optional registers through this, and an absent one is not an error.
  const ControlTableItem* item = findItem(register_name);
  if (item == nullptr) {
    return false;
  }
  const auto result = driver_.readRegister(getId(), item->address(), item->data_length(), value_out, item->is_signed());
  if (!result) {
    DXL_LOG_ERROR("[ID " << getIdInt() << "] Failed to read register '" << register_name
                         << "' (addr=" << item->address() << "): " << driver_.describeError(result));
  }
  return result.success();
}

bool Dynamixel::readRegister(const uint16_t address, const uint8_t data_length, int32_t& value_out) const
{
  // Signed, because this overload names no register to look the answer up for. Its callers read
  // indirect address pointers and compare them for equality, where the interpretation does not
  // change the comparison.
  const auto result = driver_.readRegister(getId(), address, data_length, value_out, true);
  if (!result) {
    DXL_LOG_ERROR("[ID " << getIdInt() << "] Failed to read addr=" << address
                         << " (len=" << static_cast<int>(data_length) << "): " << driver_.describeError(result));
  }
  return result.success();
}

WatchdogResult Dynamixel::clearBusWatchdog(bool* was_tripped) const
{
  if (was_tripped) {
    *was_tripped = false;
  }
  if (!registerAvailable(DXL_REGISTER_BUS_WATCHDOG)) {
    return WatchdogResult::UNSUPPORTED;
  }
  // A tripped watchdog latches at -1. Report it so callers can attribute a failure to a trip. A
  // failed read is not fatal here, because the clearing write below is what actually matters.
  int32_t current_value;
  if (was_tripped && readRegister(DXL_REGISTER_BUS_WATCHDOG, current_value) && current_value < 0) {
    *was_tripped = true;
  }
  if (!writeRegister(DXL_REGISTER_BUS_WATCHDOG, static_cast<int32_t>(0))) {
    return WatchdogResult::FAILED;
  }
  return WatchdogResult::SUCCESS;
}

double Dynamixel::dxlValueToUnit(const std::string& register_name, const int32_t value) const
{
  const ControlTableItem* item = findItem(register_name);
  if (item == nullptr) {
    DXL_LOG_ERROR("Unknown register '" << register_name << "'. Cannot convert raw value " << value << ".");
    return 0.0;
  }
  return static_cast<double>(value) * item->dxlValueToUnitRatio();
}

std::optional<bool> Dynamixel::dxlValueToBool(const std::string& register_name, const int32_t value) const
{
  const ControlTableItem* item = findItem(register_name);
  if (item == nullptr) {
    DXL_LOG_ERROR("Unknown register '" << register_name << "'. Cannot read it as a bool.");
    return std::nullopt;
  }
  if (item->unit() != "bool") {
    DXL_LOG_ERROR("The register '" << register_name << "' (type=" << item->unit() << ") is not of type bool");
    return std::nullopt;
  }
  return value > 0;
}

std::optional<int32_t> unitToDxlValue(const double unit_value, const double ratio)
{
  // A clamped position would be the far end of the joint's travel
  const double raw = std::round(unit_value / ratio);
  if (!std::isfinite(raw) || raw < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
      raw > static_cast<double>(std::numeric_limits<int32_t>::max())) {
    return std::nullopt;
  }
  return static_cast<int32_t>(raw);
}

std::optional<int32_t> Dynamixel::unitToDxlValue(const std::string& register_name, const double unit_value) const
{
  // Named rather than left to map::at, whose out_of_range says only "map::at": this is reachable
  // from writeRegister(name, double), where the register name is the whole diagnosis.
  const ControlTableItem* item = findItem(register_name);
  if (item == nullptr) {
    DXL_LOG_ERROR("Unknown register '" << register_name << "'. Cannot convert " << unit_value << " to a raw value.");
    return std::nullopt;
  }
  return dynamixel_ros_control::unitToDxlValue(unit_value, item->dxlValueToUnitRatio());
}

bool Dynamixel::boolToDxlValue(const std::string& register_name, const bool value, int32_t& dxl_value_out) const
{
  const ControlTableItem* item = findItem(register_name);
  if (item == nullptr) {
    DXL_LOG_ERROR("Unknown register '" << register_name << "'");
    return false;
  }
  if (item->unit() != "bool") {
    DXL_LOG_ERROR("The register '" << register_name << "' (type=" << item->unit() << ") is not of type bool");
    return false;
  }
  dxl_value_out = value ? 1 : 0;
  return true;
}

bool Dynamixel::limitsCurrentIn(const ControlMode mode) const
{
  return control_table_ != nullptr && control_table_->currentCapModes().count(controlModeToString(mode)) != 0;
}

bool Dynamixel::registerAvailable(const std::string& register_name) const
{
  return control_table_ != nullptr && control_table_->itemAvailable(register_name);
}

const ControlTableItem* Dynamixel::findItem(const std::string& name) const
{
  if (control_table_ == nullptr) {
    return nullptr;
  }
  return control_table_->findItem(name);
}

const ControlTableItem& Dynamixel::getItem(const std::string& name) const
{
  if (control_table_ == nullptr) {
    throw std::runtime_error("Motor ID " + std::to_string(getIdInt()) + " has no control table: it never connected.");
  }
  return control_table_->getItem(name);
}

uint16_t Dynamixel::getModelNumber() const
{
  return model_number_;
}

bool Dynamixel::setIndirectAddress(const unsigned int indirect_address_index, const std::string& register_name,
                                   uint16_t& indirect_data_address) const
{
  uint16_t indirect_address;
  if (!indirectIndexToAddresses(indirect_address_index, indirect_address, indirect_data_address)) {
    return false;
  }
  // Silent: callers probe optional registers through this, and an absent one is not an error.
  const ControlTableItem* item = findItem(register_name);
  if (item == nullptr) {
    return false;
  }
  const uint16_t register_address = item->address();
  const uint8_t data_length = item->data_length();

  // The register's slots must be contiguous: sync read/write treats [data_address, +len) as one
  // range, which breaks silently if the slots straddle two indirect address blocks.
  if (data_length > 1) {
    uint16_t last_indirect_address;
    uint16_t last_data_address;
    if (!indirectIndexToAddresses(indirect_address_index + data_length - 1, last_indirect_address, last_data_address)) {
      return false;
    }
    if (last_data_address != indirect_data_address + data_length - 1) {
      DXL_LOG_ERROR("[ID " << getIdInt() << "] Register '" << register_name
                           << "' would straddle two non-contiguous indirect address blocks.");
      return false;
    }
  }

  DXL_LOG_DEBUG("[INDIRECT ADDRESS] Setting indirect address " << indirect_address << " to " << register_address << " ("
                                                               << register_name
                                                               << "), data_address: " << indirect_data_address);
  // Read every pointer before writing any. Only a differing pointer needs a write, and the pointer
  // block is torque-locked: an energised motor answers the write with an Access Error. Writing the
  // ones that differ would leave a half-changed layout behind, so this checks first and refuses as
  // a whole.
  bool layout_matches = true;
  for (uint16_t i = 0; i < data_length; i++) {
    int32_t pointer = 0;
    if (!readRegister(indirect_address + (2 * i), sizeof(register_address), pointer)) {
      return false;
    }
    layout_matches &= pointer == register_address + i;
  }
  if (layout_matches) {
    return true;
  }
  bool torqued = true;
  if (!readRegister(DXL_REGISTER_CMD_TORQUE, torqued)) {
    return false;
  }
  if (torqued) {
    // No automatic torque-off: an energised arm joint without a brake falls while it is off.
    DXL_LOG_ERROR("[ID " << getIdInt() << "] The indirect address layout stored on the motor does not match this "
                         << "configuration (register '" << register_name << "'), and the motor is energised, which "
                         << "locks the pointer registers. Power-cycle the actuator or turn its torque off, then "
                         << "configure again.");
    return false;
  }
  for (uint16_t i = 0; i < data_length; i++) {
    if (!writeRegister(indirect_address + (2 * i), sizeof(register_address), register_address + i)) {
      return false;
    }
  }
  return true;
}

bool Dynamixel::indirectAddressesInRam() const
{
  if (control_table_ == nullptr) {
    return false;
  }
  // Conservative: if any indirect-address block lives in RAM, the motor needs a
  // post-reboot rewrite. All current models use a single, uniform block.
  for (const IndirectAddressInfo& info : control_table_->getIndirectAddressInfo()) {
    if (info.memory_type == RAM) {
      return true;
    }
  }
  return false;
}

void Dynamixel::setInitialRegisterValues(const std::unordered_map<std::string, std::string>& values)
{
  DXL_LOG_DEBUG("[ID " << getIdInt() << "] Initial register values: " << mapToString(values));
  initial_values_ = values;
}

bool Dynamixel::indirectIndexToAddresses(const unsigned int indirect_address_index, uint16_t& indirect_address,
                                         uint16_t& indirect_data_address) const
{
  if (control_table_ == nullptr) {
    return false;
  }
  const IndirectAddressInfo* info = nullptr;
  unsigned int total_count = 0;
  unsigned int local_index = 0;
  for (const IndirectAddressInfo& i : control_table_->getIndirectAddressInfo()) {
    total_count += i.count;
    if (indirect_address_index < total_count) {
      info = &i;
      local_index = indirect_address_index - (total_count - i.count);
      break;
    }
  }
  if (!info) {
    DXL_LOG_ERROR("[ID " << getIdInt() << "] Indirect address index " << indirect_address_index
                         << " exceeds the motor's capacity of " << total_count
                         << " slots. Reduce the number of state/command interfaces.");
    return false;
  }
  indirect_address = info->indirect_address_start + static_cast<uint16_t>(local_index * 2);
  indirect_data_address = info->indirect_data_start + static_cast<uint16_t>(local_index);
  return true;
}

bool Dynamixel::writeInitialValues()
{
  bool success = true;
  const Stopwatch total_timer;
  std::size_t written_count = 0;
  for (const auto& [register_name, register_value] : initial_values_) {
    if (!registerAvailable(register_name)) {
      // Not a hardware failure: the configured register simply doesn't exist on this model.
      // Already warned here, so callers that log on `false` would just double-warn.
      DXL_LOG_WARN("Register '" << register_name << "' is not available on motor ID " << getIdInt()
                                << ". Check model compatibility.");
      continue;
    }
    // Only write when the value differs (compared in raw register space): rewriting unchanged
    // EEPROM values wears the flash, stalls the motor and fails while torque is on.
    int32_t target_value;
    if (!stringToDxlValue(register_name, register_value, target_value)) {
      DXL_LOG_ERROR("Invalid initial value '" << register_value << "' for register '" << register_name << "' on ID "
                                              << getIdInt() << ".");
      success = false;
      continue;
    }
    // Checked before the comparison below skips the write, so whether a URDF is valid does not
    // depend on what the motor happens to hold
    const ControlTableItem& item = getItem(register_name);
    if (!item.writable()) {
      DXL_LOG_ERROR("[ID " << getIdInt() << "] Initial value configured for register '" << register_name
                           << "', which is read-only.");
      success = false;
      continue;
    }
    if (!fitsInRegister(target_value, item.data_length(), item.encoding())) {
      DXL_LOG_ERROR("[ID " << getIdInt() << "] Initial value " << target_value << " for register '" << register_name
                           << "' does not fit its " << static_cast<unsigned int>(item.data_length()) << " bytes.");
      success = false;
      continue;
    }
    // Compared directly. The target was just bounded to the register's own range and the read
    // decodes into that same range, so both are the same number for the same raw bytes.
    int32_t current_value;
    const bool read_ok = readRegister(register_name, current_value);
    if (read_ok && current_value == target_value) {
      DXL_LOG_DEBUG("[ID " << getIdInt() << "] Initial value of '" << register_name << "' already " << target_value
                           << ", skipping write.");
      continue;
    }
    const Stopwatch write_timer;
    if (!writeRegister(register_name, target_value)) {
      // The firmware refuses EEPROM writes while the motor is energised, and a warm start finds the
      // motors still torqued from the previous session. Failing outright there would refuse to
      // configure the bus, which leaves the arm energised with nobody managing it. So the write is
      // retried de-energised and the previous torque state put back.
      if (!retryEepromWriteDeEnergised(register_name, target_value)) {
        DXL_LOG_ERROR("Failed to write initial value '" << register_value << "' to register '" << register_name
                                                        << "' on ID " << static_cast<int>(getId()) << ".");
        success = false;
        continue;
      }
    }
    written_count++;
    if (write_timer.elapsedMs() > 10.0) {
      // Likely an EEPROM register: flash writes stall the motor.
      DXL_LOG_INFO("[timing] [ID " << getIdInt() << "] Initial write of '" << register_name << "' took "
                                   << static_cast<int>(write_timer.elapsedMs()) << " ms (EEPROM write?)");
    }
  }
  if (total_timer.elapsedMs() > 1.0) {
    DXL_LOG_INFO("[timing] [ID " << getIdInt() << "] writeInitialValues took "
                                 << static_cast<int>(total_timer.elapsedMs()) << " ms (" << written_count << " of "
                                 << initial_values_.size() << " registers written, rest unchanged)");
  }
  return success;
}

namespace {

/// Attempts at reading back the present position before torque is restored. One dropped status
/// packet affects a single transaction, so a retry distinguishes that from a bus that is really
/// gone without waiting on a timeout budget.
constexpr unsigned int GOAL_ANCHOR_READ_ATTEMPTS = 3;

/// Registers whose value changes what the goal position register means. Writing one leaves the
/// goal pointing at a different physical place than it did a moment ago.
bool isFrameChangingRegister(const std::string& register_name)
{
  return register_name == DXL_REGISTER_HOMING_OFFSET || register_name == DXL_REGISTER_DRIVE_MODE;
}

}  // namespace

bool Dynamixel::retryEepromWriteDeEnergised(const std::string& register_name, const int32_t target_value) const
{
  if (getItem(register_name).memory_type() != EEPROM) {
    return false;  // a RAM write that failed did not fail for this reason
  }
  bool torque_was_enabled = false;
  if (!readRegister(DXL_REGISTER_CMD_TORQUE, torque_was_enabled) || !torque_was_enabled) {
    return false;  // already de-energised, so torque is not what refused the write
  }
  DXL_LOG_WARN("[ID " << getIdInt() << "] EEPROM register '" << register_name
                      << "' was refused while the motor is energised. De-energising to write it, then "
                         "restoring the previous torque state.");
  if (!writeRegister(DXL_REGISTER_CMD_TORQUE, false)) {
    return false;
  }
  const bool written = writeRegister(register_name, target_value);

  // homing_offset and drive_mode redefine the frame of the goal register, so the old goal can now
  // mean another place. Torque comes back only once the goal is anchored on the present position,
  // otherwise the joint stays de-energised.
  if (isFrameChangingRegister(register_name) && registerAvailable(DXL_REGISTER_POSITION) &&
      registerAvailable(DXL_REGISTER_CMD_POSITION)) {
    bool anchored = false;
    for (unsigned int attempt = 0; attempt < GOAL_ANCHOR_READ_ATTEMPTS && !anchored; ++attempt) {
      int32_t present_position = 0;
      if (readRegister(DXL_REGISTER_POSITION, present_position)) {
        anchored = writeRegister(DXL_REGISTER_CMD_POSITION, present_position);
      }
    }
    if (!anchored) {
      DXL_LOG_ERROR("[ID " << getIdInt() << "] Could not anchor the goal position after writing '" << register_name
                           << "', which redefines the goal frame. Leaving the joint de-energised rather than "
                           << "energising it against a goal that now means a different place.");
      return false;
    }
  }

  // Restore what was found, never more: the motor was holding a moment ago, and leaving it limp
  // would be a change nobody asked for.
  if (!writeRegister(DXL_REGISTER_CMD_TORQUE, true)) {
    DXL_LOG_ERROR("[ID " << getIdInt() << "] Could not restore torque after writing '" << register_name
                         << "'. The joint is now de-energised.");
    return false;
  }
  return written;
}

uint8_t Dynamixel::getId() const
{
  return id_;
}

unsigned int Dynamixel::getIdInt() const
{
  return id_;
}

std::string hardwareErrorToString(const int32_t error_status)
{
  if (error_status == OK) {
    return "ok";
  }
  std::string result;
  if (error_status & VOLTAGE_ERROR)
    result += "Voltage, ";
  if (error_status & HALL_SENSOR_ERROR)
    result += "Hall Sensor, ";
  if (error_status & OVERHEATING_ERROR)
    result += "Overheating, ";
  if (error_status & MOTOR_ENCODER_ERROR)
    result += "Motor Encoder, ";
  if (error_status & ELECTRICAL_SHOCK_ERROR)
    result += "Electrical Shock, ";
  if (error_status & OVERLOAD_ERROR)
    result += "Overload, ";
  if (result.size() >= 2) {
    result.erase(result.size() - 2);
  }
  return result;
}

ControlMode stringToControlMode(const std::string& str)
{
  if (str == "effort" || str == "current") {
    return CURRENT;
  }
  if (str == "velocity") {
    return VELOCITY;
  }
  if (str == "position") {
    return POSITION;
  }
  if (str == "extended_position") {
    return EXTENDED_POSITION;
  }
  if (str == "current_based_position") {
    return CURRENT_BASED_POSITION;
  }
  if (str == "pwm") {
    return PWM;
  }
  throw std::invalid_argument("Control Mode '" + str + "' is unknown.");
}

std::string controlModeToString(const ControlMode mode)
{
  switch (mode) {
    case CURRENT:
      return "current";
    case VELOCITY:
      return "velocity";
    case POSITION:
      return "position";
    case EXTENDED_POSITION:
      return "extended_position";
    case CURRENT_BASED_POSITION:
      return "current_based_position";
    case PWM:
      return "pwm";
    case UNDEFINED:
      return "undefined";
  }
  return "unknown";
}

}  // namespace dynamixel_ros_control
