#ifndef DYNAMIXEL_ROS_CONTROL_CONTROL_TABLE_ITEM_H
#define DYNAMIXEL_ROS_CONTROL_CONTROL_TABLE_ITEM_H

#include <cstdint>
#include <map>
#include <string>

namespace dynamixel_ros_control {

enum MemoryType
{
  EEPROM,
  RAM
};

bool stringToMemoryType(const std::string& str, MemoryType& memory_type);

/// Whether the motor accepts a write to a register. Dynamixel has no write-only registers, so the
/// distinction only ever restricts writing.
enum AccessType
{
  READ_ONLY,
  READ_WRITE
};

bool stringToAccessType(const std::string& str, AccessType& access);

/// @brief How a register's raw bytes are to be read.
enum class RegisterEncoding
{
  unsigned_value,   ///< A magnitude, count, identifier or bit field.
  twos_complement,  ///< Carries its sign in the top bit of the register.
  unknown,          ///< Not classified. Refused for the widths that consult this.
};

/// @brief Every register name this package classifies, with its encoding.
///
/// The control tables record no signedness, so it lives here, keyed by the canonical names the
/// tables normalise to. That holds for the Protocol 2.0 families only. Protocol 1.0 encodes load
/// and speed as a magnitude with a direction bit. A narrow register missing here fails the table
/// load rather than read unsigned by omission.
[[nodiscard]] const std::map<std::string, RegisterEncoding>& registerEncodings();

/// @brief Encoding of one register, or unknown if nothing classifies that name.
[[nodiscard]] RegisterEncoding registerEncoding(const std::string& register_name);

/// Register widths the driver can transfer, the only ones the SDK has accessors for
[[nodiscard]] constexpr bool isSupportedDataLength(const uint8_t data_length)
{
  return data_length == 1 || data_length == 2 || data_length == 4;
}

/// @brief Write @p value into @p out as @p data_length little-endian bytes, the order the wire uses
/// whatever the host's.
inline void packRegisterValue(unsigned char* out, const int32_t value, const uint8_t data_length)
{
  const auto raw = static_cast<uint32_t>(value);
  for (uint8_t byte = 0; byte < data_length; ++byte) {
    out[byte] = static_cast<unsigned char>((raw >> (8u * byte)) & 0xFFu);
  }
}

/// @brief Read the low @p data_length bytes of a value the SDK already assembled, in its own
/// signedness. @p data holds the register's numeric value, not its wire bytes: GroupSyncRead
/// assembles it with shifts, so only the narrowing back down is done here.
[[nodiscard]] constexpr int32_t unpackRegisterValue(const uint32_t data, const uint8_t data_length, const bool is_signed)
{
  if (data_length >= sizeof(int32_t)) {
    return static_cast<int32_t>(data);
  }
  const auto bits = static_cast<unsigned int>(8u * data_length);
  const uint32_t truncated = data & ((static_cast<uint32_t>(1) << bits) - 1);
  if (!is_signed || (truncated & (static_cast<uint32_t>(1) << (bits - 1))) == 0) {
    return static_cast<int32_t>(truncated);
  }
  return static_cast<int32_t>(truncated) - static_cast<int32_t>(static_cast<uint32_t>(1) << bits);
}

/// @brief Whether a raw value reaches a register of this width and encoding as the same number.
/// A value above a signed register's maximum arrives as a negative one, goal_current = 65535 as -1.
/// Every int32_t fits a four-byte register, whose range is the motor's own check.
[[nodiscard]] constexpr bool fitsInRegister(const int32_t value, const uint8_t data_length,
                                            const RegisterEncoding encoding)
{
  if (!isSupportedDataLength(data_length)) {
    return false;  // no width this driver can write into
  }
  if (data_length >= sizeof(int32_t)) {
    return true;
  }
  if (encoding == RegisterEncoding::unknown) {
    // Unreachable: a narrow register with no classification fails the table load. Refusing rather
    // than picking a reading keeps that true if the load check is ever relaxed.
    return false;
  }
  const int64_t bits = 8 * static_cast<int64_t>(data_length);
  const bool is_signed = encoding == RegisterEncoding::twos_complement;
  const int64_t lowest = is_signed ? -(static_cast<int64_t>(1) << (bits - 1)) : 0;
  const int64_t highest = (static_cast<int64_t>(1) << (is_signed ? bits - 1 : bits)) - 1;
  return value >= lowest && value <= highest;
}

class ControlTableItem
{
public:
  ControlTableItem(std::string name, uint16_t address, uint8_t data_length, MemoryType memory_type, AccessType access);
  ControlTableItem();
  bool loadFromString(std::string control_table_string);

  [[nodiscard]] const std::string& name() const;
  [[nodiscard]] uint16_t address() const;
  [[nodiscard]] uint8_t data_length() const;
  [[nodiscard]] MemoryType memory_type() const;
  /// @brief Whether the motor accepts a write to this register. False for the telemetry registers,
  /// which report a Dynamixel instruction error when written directly and are silently ignored
  /// inside a sync write.
  [[nodiscard]] bool writable() const;
  /// @brief Whether this register's raw value is two's complement. See registerEncodings().
  [[nodiscard]] bool is_signed() const;
  [[nodiscard]] RegisterEncoding encoding() const;
  [[nodiscard]] const std::string& unit() const;
  [[nodiscard]] double dxlValueToUnitRatio() const;

  void setDxlValueToUnitRatio(double dxl_value_to_unit_ratio);

private:
  std::string name_;
  uint16_t address_;
  uint8_t data_length_;
  MemoryType memory_type_;
  AccessType access_;
  RegisterEncoding encoding_;
  std::string unit_;
  double dxl_value_to_unit_ratio_;
};

}  // namespace dynamixel_ros_control

#endif
