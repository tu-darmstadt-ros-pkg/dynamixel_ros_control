#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_H

#include <ros/ros.h>
#include <dynamixel_ros_control/control_table.h>
#include <dynamixel_ros_control/dynamixel_driver.h>

#include <cuckoo_time_translator/DeviceTimeTranslator.h>

namespace dynamixel_ros_control {

enum ControlMode {
  CURRENT = 0,
  VELOCITY = 1,
  POSITION = 3,
  EXTENDED_POSITION = 4,
  CURRENT_BASED_POSITION = 5,
  PWM = 16
};

enum HardwareErrorStatus {
  OK = 0,
  VOLTAGE_ERROR = 1,           // 2^0
  HALL_SENSOR_ERROR = 2,       // 2^1
  OVERHEATING_ERROR = 4,       // 2^2
  MOTOR_ENCODER_ERROR = 8,     // 2^3
  ELECTRICAL_SHOCK_ERROR = 16, // 2^4
  OVERLOAD_ERROR = 32,         // 2^5
};

ControlMode stringToControlMode(const std::string& str);

class Dynamixel {
public:
  Dynamixel(dynamixel_ros_control::DynamixelDriver& driver);
  Dynamixel(uint8_t id, dynamixel_ros_control::DynamixelDriver& driver);

  bool initFromNh(const ros::NodeHandle& nh);
  bool loadControlTable();

  bool ping();
  bool reboot();

  // Register access
  /**
   * @brief Reads a register first. If the read value matches the desired value,
   * no write operation is performed. Otherwise, the desired value is written.
   * @param address Target address
   * @param data_length Data length to write
   * @param value Data to write
   * @return True, if the address contains the desired value
   */
  bool readWriteRegister(uint16_t address, uint8_t data_length, int32_t value);

  bool writeRegister(std::string register_name, bool value) const;
  bool writeRegister(std::string register_name, double value) const;
  bool writeRegister(std::string register_name, int32_t value) const;
  bool writeRegister(uint16_t address, uint8_t data_length, int32_t value) const;

  bool readRegister(std::string register_name, double& value_out) const;
  bool readRegister(std::string register_name, int32_t& value_out) const;
  bool readRegister(uint16_t address, uint8_t data_length, int32_t& value_out) const;

  bool writeControlMode(ControlMode mode) const;

  // Value conversion functions
  double dxlValueToUnit(std::string register_name, int32_t value) const;
  bool dxlValueToBool(std::string register_name, int32_t value) const;
  int32_t unitToDxlValue(std::string register_name, double unit_value) const;
  int32_t boolToDxlValue(std::string register_name, bool b) const;

  bool registerAvailable(std::string register_name) const;
  const ControlTableItem& getItem(std::string& name) const;
  uint8_t getId() const;
  uint16_t getModelNumber() const;

  bool setIndirectAddress(unsigned int indirect_address_index, std::string register_name, uint16_t& indirect_data_address);
  
  // Time translator
  void addTimeTranslator(const ros::NodeHandle& nh);
  bool translateTime(const ros::Time& receive_time);
  ros::Time getStamp() const;
  
  double realtime_tick_ms_;

  std::string getHardwareErrorStatusString() const;
  int32_t hardware_error_status;
private:
  void indirectIndexToAddresses(unsigned int indirect_address_index, uint16_t& indirect_address, uint16_t& indirect_data_address);

  ros::NodeHandle nh_;
  dynamixel_ros_control::DynamixelDriver& driver_;
  ControlTable* control_table_;

  std::unique_ptr<cuckoo_time_translator::DefaultDeviceTimeUnwrapperAndTranslator> device_time_translator_;

  uint8_t id_;
  uint16_t model_number_;
  std::string model_name_;

  ros::Time stamp_;
};

}

#endif
