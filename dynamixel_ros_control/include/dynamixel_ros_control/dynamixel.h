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

ControlMode stringToControlMode(const std::string& str);

class Dynamixel {
public:
  Dynamixel(uint8_t id, uint16_t model_number, dynamixel_ros_control::DynamixelDriver& driver);
  bool loadControlTable();

  bool ping();
  bool reboot();

  // Register access
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

  const ControlTableItem& getItem(std::string& name) const;
  uint8_t getId() const;
  uint16_t getModelNumber() const;

  bool setIndirectAddress(unsigned int indirect_address_index, std::string register_name, uint16_t& indirect_data_address);
  
  // Time translator
  void addTimeTranslator(const ros::NodeHandle& nh);
  bool translateTime(const ros::Time& receive_time);
  ros::Time getStamp() const;
  
  double realtime_tick_ms_;
  
  std::unique_ptr<cuckoo_time_translator::DefaultDeviceTimeUnwrapperAndTranslator> device_time_translator_;
private:
  void indirectIndexToAddresses(unsigned int indirect_address_index, uint16_t& indirect_address, uint16_t& indirect_data_address);

  dynamixel_ros_control::DynamixelDriver& driver_;
  ControlTable* control_table_;


  uint8_t id_;
  uint16_t model_number_;
  std::string model_name_;

  ros::Time stamp_;
};

}

#endif
