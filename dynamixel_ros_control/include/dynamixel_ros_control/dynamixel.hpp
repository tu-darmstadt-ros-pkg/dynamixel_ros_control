#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_H

#include <rclcpp/rclcpp.hpp>
#include <dynamixel_ros_control/control_table.hpp>
#include <dynamixel_ros_control/dynamixel_driver.hpp>

namespace dynamixel_ros_control {

enum ControlMode
{
  CURRENT = 0,
  VELOCITY = 1,
  POSITION = 3,
  EXTENDED_POSITION = 4,
  CURRENT_BASED_POSITION = 5,
  PWM = 16
};

enum HardwareErrorStatus
{
  OK = 0,
  VOLTAGE_ERROR = 1,            // 2^0
  HALL_SENSOR_ERROR = 2,        // 2^1
  OVERHEATING_ERROR = 4,        // 2^2
  MOTOR_ENCODER_ERROR = 8,      // 2^3
  ELECTRICAL_SHOCK_ERROR = 16,  // 2^4
  OVERLOAD_ERROR = 32,          // 2^5
};

ControlMode stringToControlMode(const std::string& str);

class Dynamixel
{
public:
  Dynamixel(uint8_t id, DynamixelDriver& driver);

  /**
   * Ping motor and load control table
   * @return
   */
  bool connect();

  [[nodiscard]] bool ping() const;
  bool reboot() const;

  // Register access
  bool writeRegister(const std::string& register_name, double value) const;
  bool writeRegister(const std::string& register_name, bool value) const;
  bool writeRegister(const std::string& register_name, int32_t value) const;
  bool writeRegister(uint16_t address, uint8_t data_length, int32_t value) const;

  bool readRegister(const std::string& register_name, double& value_out) const;
  bool readRegister(const std::string& register_name, bool& value_out) const;
  bool readRegister(const std::string& register_name, int32_t& value_out) const;
  bool readRegister(uint16_t address, uint8_t data_length, int32_t& value_out) const;

  /**
   * @brief Reads a register first. If the read value matches the desired value,
   * no write operation is performed. Otherwise, the desired value is written.
   * @param address Target address
   * @param data_length Data length to write
   * @param value Data to write
   * @return True, if the address contains the desired value
   */
  bool readWriteRegister(uint16_t address, uint8_t data_length, int32_t value) const;

  template <typename T>
  bool readWriteRegister(std::string register_name, T value) const
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

  bool writeControlMode(ControlMode mode) const;

  // Value conversion functions
  [[nodiscard]] double dxlValueToUnit(const std::string& register_name, int32_t value) const;
  [[nodiscard]] bool dxlValueToBool(const std::string& register_name, int32_t value) const;
  [[nodiscard]] int32_t unitToDxlValue(const std::string& register_name, double unit_value) const;
  [[nodiscard]] int32_t boolToDxlValue(const std::string& register_name, bool b) const;

  [[nodiscard]] bool registerAvailable(const std::string& register_name) const;
  [[nodiscard]] const ControlTableItem& getItem(const std::string& name) const;
  [[nodiscard]] uint8_t getId() const;
  [[nodiscard]] uint16_t getModelNumber() const;

  bool setIndirectAddress(unsigned int indirect_address_index, const std::string& register_name,
                          uint16_t& indirect_data_address) const;

  [[nodiscard]] std::string getHardwareErrorStatusString() const;
  int32_t hardware_error_status;

private:
  void indirectIndexToAddresses(unsigned int indirect_address_index, uint16_t& indirect_address,
                                uint16_t& indirect_data_address) const;

  DynamixelDriver& driver_;
  ControlTable* control_table_;

  uint8_t id_;
  uint16_t model_number_;
  std::string model_name_;
};

}  // namespace dynamixel_ros_control

#endif
