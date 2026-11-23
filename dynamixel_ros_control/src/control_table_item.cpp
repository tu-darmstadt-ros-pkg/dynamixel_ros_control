#include "dynamixel_ros_control/log.hpp"

#include <dynamixel_ros_control/control_table_item.hpp>

#include <boost/algorithm/string.hpp>
#include <dynamixel_ros_control/common.hpp>
#include <array>
#include <map>
#include <utility>

namespace dynamixel_ros_control {

ControlTableItem::ControlTableItem(std::string name, const uint16_t address, const uint8_t data_length,
                                   const MemoryType memory_type, const AccessType access)
    : name_(std::move(name)),
      address_(address),
      data_length_(data_length),
      memory_type_(memory_type),
      access_(access),
      encoding_(registerEncoding(name_)),
      dxl_value_to_unit_ratio_(1.0)
{}

/// An item that has not been loaded describes no register, so it grants nothing. READ_ONLY keeps it
/// from being written if it ever escapes a failed load.
ControlTableItem::ControlTableItem()
    : ControlTableItem("unset", 0, 0, RAM, READ_ONLY)
{}

bool ControlTableItem::loadFromString(std::string control_table_string)
{
  removeWhitespace(control_table_string);
  std::vector<std::string> parts;
  boost::split(parts, control_table_string, boost::is_any_of("|"));
  if (parts.size() != 6) {
    DXL_LOG_ERROR("Control table item has invalid size (" << parts.size() << "), expected (6).");
    return false;
  }

  // load data
  if (!parseIntegerInto(parts[0], address_)) {
    DXL_LOG_ERROR("Failed to read address '" << parts[0] << "'.");
    return false;
  }
  name_ = parts[1];
  encoding_ = registerEncoding(name_);
  if (!parseIntegerInto(parts[2], data_length_)) {
    DXL_LOG_ERROR("Failed to read data length '" << parts[2] << "'.");
    return false;
  }
  if (!isSupportedDataLength(data_length_)) {
    DXL_LOG_ERROR("Register '" << name_ << "' has data length " << static_cast<unsigned int>(data_length_)
                               << ". Only 1, 2 and 4 byte registers can be transferred.");
    return false;
  }
  // Only the narrow widths consult the encoding: a 4-byte value is copied into an int32_t bit for
  // bit, which is already right for the signed ones and in range for the rest. A narrow register
  // nobody has classified would read unsigned by omission, so it is refused instead.
  if ((data_length_ == 1 || data_length_ == 2) && encoding_ == RegisterEncoding::unknown) {
    DXL_LOG_ERROR("Register '" << name_ << "' has no signedness classification. Add it to "
                               << "TWOS_COMPLEMENT_REGISTERS or UNSIGNED_REGISTERS in "
                               << "control_table_item.cpp, taking the answer from the model's "
                               << "e-Manual control table rather than assuming.");
    return false;
  }
  if (!stringToAccessType(parts[3], access_)) {
    return false;
  }
  if (!stringToMemoryType(parts[4], memory_type_)) {
    return false;
  }
  unit_ = parts[5];
  return true;
}

const std::string& ControlTableItem::name() const
{
  return name_;
}

uint16_t ControlTableItem::address() const
{
  return address_;
}

MemoryType ControlTableItem::memory_type() const
{
  return memory_type_;
}

void ControlTableItem::setDxlValueToUnitRatio(double dxl_value_to_unit_ratio)
{
  dxl_value_to_unit_ratio_ = dxl_value_to_unit_ratio;
}

double ControlTableItem::dxlValueToUnitRatio() const
{
  return dxl_value_to_unit_ratio_;
}

const std::string& ControlTableItem::unit() const
{
  return unit_;
}

uint8_t ControlTableItem::data_length() const
{
  return data_length_;
}

bool ControlTableItem::writable() const
{
  return access_ == READ_WRITE;
}

bool ControlTableItem::is_signed() const
{
  return encoding_ == RegisterEncoding::twos_complement;
}

RegisterEncoding ControlTableItem::encoding() const
{
  return encoding_;
}

bool stringToMemoryType(const std::string& str, MemoryType& memory_type)
{
  if (str == "RAM") {
    memory_type = RAM;
    return true;
  }
  if (str == "EEPROM") {
    memory_type = EEPROM;
    return true;
  }
  DXL_LOG_ERROR("Unknown memory type '" << str << "'.");
  return false;
}

namespace {

/// Registers whose raw value carries a sign. Ranges quoted from the ROBOTIS e-Manual control
/// tables, which print a negative lower bound only for these.
constexpr std::array TWOS_COMPLEMENT_REGISTERS{
    // A trip sets it to -1 (XM430-W350 prose, PH54 Range "0 ~ 127 and -1"). The Range column on
    // XM430-W350 says "1 ~ 127" and omits the sentinel, so the table alone is not enough here.
    // Dynamixel::clearBusWatchdog() tests for the negative value.
    "bus_watchdog",
    "goal_current",  // Range "-Current Limit(38) ~ Current Limit(38)"
    "goal_pwm",      // Range "-PWM Limit(36) ~ PWM Limit(36)"
    // The only entry not printed as a signed range anywhere. getPresentCurrent() is documented as
    // "-100 ~ 100 %" in the ROBOTIS Arduino API, it is the measured counterpart of the signed
    // goal_current in the same unit, and MX-28(2.0) puts the signed present_load at its address.
    // Inferred, but from three directions.
    "present_current",
    "present_load",  // MX-28(2.0) Range "-1,000 ~ 1,000", "Positive(CCW Load), Negative(CW Load)"
    "present_pwm",   // MX-28(2.0) Range "-[PWM Limit(36)] ~ [PWM Limit(36)]"
};

/// Everything else the shipped tables declare at one or two bytes. Listed rather than defaulted:
/// a register missing from both lists is a decision nobody made, not an unsigned register.
constexpr std::array UNSIGNED_REGISTERS{
    // Printed with a non-negative Range in the e-Manual control table.
    "baud_rate",               // "0 ~ 7" (XM430-W350), "0 ~ 8" (H54-200-S500-R)
    "current_limit",           // "0 ~ 1,193" - a magnitude, unlike goal_current
    "drive_mode",              // "0 ~ 13"
    "external_port_data_1",    // "0 ~ 4,095"
    "external_port_data_2",    //
    "external_port_data_3",    //
    "external_port_data_4",    //
    "external_port_mod_1",     // "0 ~ 3", a mode selector
    "external_port_mod_2",     //
    "external_port_mod_3",     //
    "external_port_mod_4",     //
    "feedforward_1st_gain",    // "0 ~ 16,383"
    "feedforward_2nd_gain",    // "0 ~ 16,383"
    "id",                      // "0 ~ 252" - 252 would read as -4 sign-extended
    "led",                     // "0 ~ 1"
    "led_blue",                // "0 ~ 255" (H54-200-S500-R) - 255 would read as -1
    "led_green",               //
    "led_red",                 //
    "max_voltage_limit",       // "95 ~ 160" (X), "0 ~ 400" (PRO)
    "min_voltage_limit",       //
    "moving",                  // "0 ~ 1"
    "operating_mode",          // "0 ~ 16" (X), "0, 1, 3, 4" (PRO)
    "position_d_gain",         // "0 ~ 16,383"
    "position_i_gain",         //
    "position_p_gain",         //
    "present_temperature",     // "0 ~ 100"
    "protocol_version",        // "1 ~ 2"
    "pwm_limit",               // "0 ~ 885" - a magnitude, unlike goal_pwm
    "realtime_tick",           // "0 ~ 32,767" - stops one short of the sign bit
    "registered_instruction",  // "0 ~ 1"
    "return_delay_time",       // "0 ~ 254" (X), "0 ~ 255" (PRO). Ships at 250, so every motor
                               // out of the box read as -6 while this was sign-extended.
    "secondary_id",            // "0 ~ 252", ships at 255
    "status_return_level",     // "0 ~ 2"
    "temperature_limit",       // "0 ~ 100"
    "torque_enable",           // "0 ~ 1"
    "velocity_i_gain",         // "0 ~ 16,383"
    "velocity_p_gain",         //

    // Bit fields. The e-Manual prints no Range for these and describes them bit by bit; a bit
    // field has no sign to extend.
    "hardware_error_status",
    "moving_status",
    "shutdown",

    // Identifiers and measured magnitudes the e-Manual leaves without a Range.
    "model_number",           // H54-200-S500-R is 54,024, which does not fit a signed 16-bit read
    "present_input_voltage",  // supply voltage in 0.1 V, never negative
    "version_of_firmware",
};

}  // namespace

const std::map<std::string, RegisterEncoding>& registerEncodings()
{
  static const std::map<std::string, RegisterEncoding> encodings = [] {
    std::map<std::string, RegisterEncoding> map;
    for (const char* name : TWOS_COMPLEMENT_REGISTERS) {
      map.emplace(name, RegisterEncoding::twos_complement);
    }
    for (const char* name : UNSIGNED_REGISTERS) {
      map.emplace(name, RegisterEncoding::unsigned_value);
    }
    return map;
  }();
  return encodings;
}

RegisterEncoding registerEncoding(const std::string& register_name)
{
  const auto& encodings = registerEncodings();
  const auto it = encodings.find(register_name);
  return it != encodings.end() ? it->second : RegisterEncoding::unknown;
}

bool stringToAccessType(const std::string& str, AccessType& access)
{
  if (str == "R") {
    access = READ_ONLY;
    return true;
  }
  if (str == "RW") {
    access = READ_WRITE;
    return true;
  }
  DXL_LOG_ERROR("Unknown access type '" << str << "'.");
  return false;
}

}  // namespace dynamixel_ros_control
