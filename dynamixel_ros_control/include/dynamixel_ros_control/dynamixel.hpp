#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_H

#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <dynamixel_ros_control/control_table.hpp>
#include <dynamixel_ros_control/dynamixel_driver.hpp>

namespace dynamixel_ros_control {

/// @brief Convert a physical value to the nearest raw tick, or nothing if it has no raw equivalent.
///
/// Empty for a non-finite value and for one whose tick count leaves the int32 the wire carries,
/// where a number would substitute a real setpoint for an impossible one. Silent, because the
/// cyclic write calls it, so the caller reports.
[[nodiscard]] std::optional<int32_t> unitToDxlValue(double unit_value, double ratio);

/// @brief Decode a hardware-error-status bitfield (Dynamixel error byte) into a comma-separated
/// human-readable string. Returns "ok" when no bits are set.
[[nodiscard]] std::string hardwareErrorToString(int32_t error_status);

/// @brief Dynamixel operating modes (protocol 2.0).
enum ControlMode
{
  CURRENT = 0,
  VELOCITY = 1,
  POSITION = 3,
  EXTENDED_POSITION = 4,
  CURRENT_BASED_POSITION = 5,
  PWM = 16,
  UNDEFINED = 255
};

[[nodiscard]] inline bool isPositionMode(const ControlMode mode)
{
  return mode == POSITION || mode == EXTENDED_POSITION || mode == CURRENT_BASED_POSITION;
}

/// @brief Hardware error status flags (can be combined).
enum HardwareErrorStatus
{
  OK = 0,
  VOLTAGE_ERROR = 1,
  HALL_SENSOR_ERROR = 2,
  OVERHEATING_ERROR = 4,
  MOTOR_ENCODER_ERROR = 8,
  ELECTRICAL_SHOCK_ERROR = 16,
  OVERLOAD_ERROR = 32,
};

// Common register names
constexpr char DXL_REGISTER_CMD_TORQUE[] = "torque_enable";
constexpr char DXL_REGISTER_CMD_POSITION[] = "goal_position";
constexpr char DXL_REGISTER_CMD_VELOCITY[] = "goal_velocity";
constexpr char DXL_REGISTER_CMD_EFFORT[] = "goal_current";  // All models use current (A), never real torque
constexpr char DXL_REGISTER_CONTROL_MODE[] = "operating_mode";
constexpr char DXL_REGISTER_HOMING_OFFSET[] = "homing_offset";
constexpr char DXL_REGISTER_DRIVE_MODE[] = "drive_mode";
constexpr int32_t DXL_DRIVE_MODE_TIME_BASED_PROFILE = 0x04;

constexpr char DXL_REGISTER_POSITION[] = "present_position";
constexpr char DXL_REGISTER_VELOCITY[] = "present_velocity";
constexpr char DXL_REGISTER_EFFORT[] = "present_current";
constexpr char DXL_REGISTER_HARDWARE_ERROR[] = "hardware_error_status";

constexpr char DXL_REGISTER_LED_RED[] = "led_red";
constexpr char DXL_REGISTER_LED_GREEN[] = "led_green";
constexpr char DXL_REGISTER_LED_BLUE[] = "led_blue";

constexpr char DXL_REGISTER_BUS_WATCHDOG[] = "bus_watchdog";
// Valid range for the bus_watchdog register across all supported Dynamixel models.
// The register is 1 byte but interpreted as signed: 0 disables the watchdog, and
// values 1..127 select a timeout in units defined per-model (see `ms_20` in the
// model YAMLs). Values >127 are reserved/invalid per the Dynamixel control table spec.
constexpr int DXL_BUS_WATCHDOG_MIN_TICKS = 1;
constexpr int DXL_BUS_WATCHDOG_MAX_TICKS = 127;

/**
 * @brief Outcome of a bus watchdog operation.
 *
 * UNSUPPORTED is distinct from FAILED on purpose: a model without the register is a valid
 * configuration (mixed buses), so callers must not treat it as an error.
 */
enum class WatchdogResult : uint8_t
{
  SUCCESS,
  UNSUPPORTED,
  FAILED
};

/// @brief Convert string to ControlMode enum.
ControlMode stringToControlMode(const std::string& str);

/// @brief Convert ControlMode enum to its canonical string ("position", "extended_position", ...).
std::string controlModeToString(ControlMode mode);

/**
 * @brief Represents a single Dynamixel motor.
 *
 * Provides high-level access to motor registers with automatic unit conversion.
 * Values are converted between Dynamixel counts and SI units (radians, rad/s, etc.)
 * based on the motor's control table.
 */
class Dynamixel
{
public:
  Dynamixel(uint8_t id, DynamixelDriver& driver);

  [[nodiscard]] const rclcpp::Logger& dxlLogger() const
  {
    return driver_.dxlLogger();
  }

  /// @brief Ping the motor and load its control table based on model number.
  [[nodiscard]] bool connect();

  /// @brief Whether connect() has loaded this motor's control table. Register access on a motor
  /// that has not fails rather than reaching for a table that is not there.
  [[nodiscard]] bool isConnected() const
  {
    return control_table_ != nullptr;
  }

  /// @brief Reboot the motor (clears hardware errors, resets torque to off).
  [[nodiscard]] bool reboot() const;

  // Register access (with automatic unit conversion)
  [[nodiscard]] bool writeRegister(const std::string& register_name, const std::string& value) const;
  [[nodiscard]] bool writeRegister(const std::string& register_name, double value) const;
  [[nodiscard]] bool writeRegister(const std::string& register_name, bool value) const;
  [[nodiscard]] bool writeRegister(const std::string& register_name, int32_t value) const;
  [[nodiscard]] bool writeRegister(uint16_t address, uint8_t data_length, int32_t value) const;

  [[nodiscard]] bool readRegister(const std::string& register_name, double& value_out) const;
  [[nodiscard]] bool readRegister(const std::string& register_name, bool& value_out) const;
  [[nodiscard]] bool readRegister(const std::string& register_name, int32_t& value_out) const;
  [[nodiscard]] bool readRegister(uint16_t address, uint8_t data_length, int32_t& value_out) const;

  /// @brief Clear a latched trip by writing 0, which also restores goal-register write access.
  ///
  /// The one addressed watchdog operation, for connect(): it runs before any sync group exists, and
  /// a latch left in place would silently swallow every initial register write. Arming belongs to
  /// BusWatchdog, which does it for the whole bus in one transaction per step.
  /// @param was_tripped Optional out: true if the register read back negative (latched) before clearing.
  [[nodiscard]] WatchdogResult clearBusWatchdog(bool* was_tripped = nullptr) const;

  /// @brief Convert a configured string value ("true", "3.14", ...) to the raw register value
  /// using the register's unit (bool or scaled double).
  [[nodiscard]] bool stringToDxlValue(const std::string& register_name, const std::string& value,
                                      int32_t& dxl_value_out) const;

  // Unit conversion
  [[nodiscard]] double dxlValueToUnit(const std::string& register_name, int32_t value) const;
  /// @brief Read a raw value as a bool, or nothing if the register is not of type bool. Empty
  /// rather than false, which is a legitimate reading and would hide the mistake.
  [[nodiscard]] std::optional<bool> dxlValueToBool(const std::string& register_name, int32_t value) const;
  [[nodiscard]] std::optional<int32_t> unitToDxlValue(const std::string& register_name, double unit_value) const;
  /// @brief Convert a bool to the raw register value. Fails (without touching dxl_value_out)
  /// if the register is unknown or not of type bool.
  [[nodiscard]] bool boolToDxlValue(const std::string& register_name, bool value, int32_t& dxl_value_out) const;

  /// @brief Check if a register exists in this motor's control table.
  [[nodiscard]] bool registerAvailable(const std::string& register_name) const;
  /// @brief Whether Goal Current limits the current in @p mode, per the model's current_cap_modes
  [[nodiscard]] bool limitsCurrentIn(ControlMode mode) const;

  /// @brief Get control table item for a register, or nullptr if this model has no such register.
  [[nodiscard]] const ControlTableItem* findItem(const std::string& name) const;

  /// @brief Get control table item for a register. Throws if this model has no such register.
  [[nodiscard]] const ControlTableItem& getItem(const std::string& name) const;

  [[nodiscard]] uint8_t getId() const;
  [[nodiscard]] unsigned int getIdInt() const;
  [[nodiscard]] uint16_t getModelNumber() const;

  /// @brief Configure indirect addressing for a register.
  [[nodiscard]] bool setIndirectAddress(unsigned int indirect_address_index, const std::string& register_name,
                                        uint16_t& indirect_data_address) const;

  /// @brief Whether this motor's indirect-address pointer registers live in RAM.
  /// True for X-series (pointers are wiped by a reboot and must be re-written),
  /// false for P-/PRO-series (pointers live in EEPROM and survive a reboot).
  [[nodiscard]] bool indirectAddressesInRam() const;

  int32_t hardware_error_status{OK};  ///< Last read hardware error status.
  int32_t bus_watchdog_value{0};      ///< Last read bus watchdog register value (-1 = tripped/latched).

  /// @brief Set initial register values to write on connect.
  void setInitialRegisterValues(const std::unordered_map<std::string, std::string>& values);

  /// @brief Write all configured initial register values to hardware.
  bool writeInitialValues();

private:
  /// @brief Retry an EEPROM write that a torqued motor refused, with torque off and restored.
  ///
  /// The goal is re-anchored on the present position before re-energising: the registers worth
  /// retrying this way (homing_offset, drive_mode) are exactly the ones that change what the goal
  /// already in the register means.
  /// @return false if the register is not EEPROM, the motor was not energised, or the retry failed -
  ///         i.e. whenever the original failure still stands.
  [[nodiscard]] bool retryEepromWriteDeEnergised(const std::string& register_name, int32_t target_value) const;

  /// @brief Map an indirect slot index to its pointer/data addresses.
  /// @return false if the index exceeds the motor's indirect address capacity.
  [[nodiscard]] bool indirectIndexToAddresses(unsigned int indirect_address_index, uint16_t& indirect_address,
                                              uint16_t& indirect_data_address) const;

  DynamixelDriver& driver_;
  ControlTable* control_table_{nullptr};

  uint8_t id_;
  uint16_t model_number_{0};

  std::unordered_map<std::string, std::string> initial_values_;
};

}  // namespace dynamixel_ros_control

#endif
