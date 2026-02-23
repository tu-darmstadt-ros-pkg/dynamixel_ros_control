#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_H

#include <rclcpp/rclcpp.hpp>
#include <dynamixel_ros_control/control_table.hpp>
#include <dynamixel_ros_control/dynamixel_driver.hpp>

namespace dynamixel_ros_control {

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

constexpr char DXL_REGISTER_POSITION[] = "present_position";
constexpr char DXL_REGISTER_VELOCITY[] = "present_velocity";
constexpr char DXL_REGISTER_EFFORT[] = "present_current";
constexpr char DXL_REGISTER_HARDWARE_ERROR[] = "hardware_error_status";

constexpr char DXL_REGISTER_LED_RED[] = "led_red";
constexpr char DXL_REGISTER_LED_GREEN[] = "led_green";
constexpr char DXL_REGISTER_LED_BLUE[] = "led_blue";

/// @brief Convert string to ControlMode enum.
ControlMode stringToControlMode(const std::string& str);

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

  /// @brief Ping the motor and load its control table based on model number.
  [[nodiscard]] bool connect();

  /// @brief Check if the motor responds to ping.
  [[nodiscard]] bool ping() const;

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

  /// @brief Read register first; only write if value differs. Reduces bus traffic.
  [[nodiscard]] bool readWriteRegister(uint16_t address, uint8_t data_length, int32_t value) const;

  /// @brief Template version of readWriteRegister for named registers.
  template <typename T>
  [[nodiscard]] bool readWriteRegister(std::string register_name, T value) const
  {
    T register_value;
    if (!readRegister(register_name, register_value)) {
      return false;
    }
    if (register_value != value) {
      if (!writeRegister(register_name, value)) {
        return false;
      }
    }
    return true;
  }

  /// @brief Write control mode register (optionally disabling torque first).
  [[nodiscard]] bool writeControlMode(ControlMode mode, bool disable_torque = false) const;

  // Unit conversion
  [[nodiscard]] double dxlValueToUnit(const std::string& register_name, int32_t value) const;
  [[nodiscard]] bool dxlValueToBool(const std::string& register_name, int32_t value) const;
  [[nodiscard]] int32_t unitToDxlValue(const std::string& register_name, double unit_value) const;
  [[nodiscard]] int32_t boolToDxlValue(const std::string& register_name, bool b) const;

  /// @brief Check if a register exists in this motor's control table.
  [[nodiscard]] bool registerAvailable(const std::string& register_name) const;

  /// @brief Get control table item for a register.
  [[nodiscard]] const ControlTableItem& getItem(const std::string& name) const;

  [[nodiscard]] uint8_t getId() const;
  [[nodiscard]] unsigned int getIdInt() const;
  [[nodiscard]] uint16_t getModelNumber() const;

  /// @brief Configure indirect addressing for a register.
  [[nodiscard]] bool setIndirectAddress(unsigned int indirect_address_index, const std::string& register_name,
                                        uint16_t& indirect_data_address) const;

  /// @brief Get human-readable description of current hardware error.
  [[nodiscard]] std::string getHardwareErrorStatusString() const;

  int32_t hardware_error_status{OK};  ///< Last read hardware error status.

  /// @brief Set initial register values to write on connect.
  void setInitialRegisterValues(const std::unordered_map<std::string, std::string>& values);

  /// @brief Get configured initial register values.
  const std::unordered_map<std::string, std::string>& getInitialRegisterValues() const;

private:
  void indirectIndexToAddresses(unsigned int indirect_address_index, uint16_t& indirect_address,
                                uint16_t& indirect_data_address) const;
  bool writeInitialValues();

  DynamixelDriver& driver_;
  ControlTable* control_table_{nullptr};

  uint8_t id_;
  uint16_t model_number_{0};

  std::unordered_map<std::string, std::string> initial_values_;
};

}  // namespace dynamixel_ros_control

#endif
