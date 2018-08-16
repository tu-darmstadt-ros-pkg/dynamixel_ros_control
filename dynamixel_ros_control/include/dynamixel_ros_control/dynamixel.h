#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_H

#include <ros/ros.h>
#include <dynamixel_ros_control/control_table_item.h>
#include <dynamixel_ros_control/dynamixel_driver.h>

namespace dynamixel_ros_control {

struct IndirectAddressInfo {
  uint16_t indirect_address_start;
  unsigned int count;
  uint16_t indirect_data_start;
};

class Dynamixel {
public:
  Dynamixel(uint8_t id, uint16_t model_number, dynamixel_ros_control::DynamixelDriver& driver);
  bool loadControlTable(const ros::NodeHandle& nh);

  // Register access
  bool writeRegister(std::string register_name, int32_t value) const;
  bool writeRegister(uint16_t address, uint8_t data_length, int32_t value) const;

  bool readRegister(std::string register_name, int32_t& value_out);
  bool readRegister(uint16_t address, uint8_t data_length, int32_t& value_out);

  // Value conversion functions
  double dxlValueToUnit(std::string register_name, int32_t value);
  bool dxlValueToBool(std::string register_name, int32_t value);
  int32_t unitToDxlValue(std::string register_name, double unit_value);
  int32_t boolToDxlValue(std::string register_name, bool b);

  const ControlTableItem& getItem(std::string& name) const;
  uint8_t getId() const;
  uint16_t getModelNumber() const;

  bool setIndirectAddress(unsigned int indirect_address_index, std::string register_name, uint16_t& indirect_data_address);

private:
  bool indirectIndexToAddresses(unsigned int indirect_address_index, uint16_t& indirect_address, uint16_t& indirect_data_address);
  std::string getSeries(const ros::NodeHandle& nh) const;
  bool loadUnitConversionRatios(const ros::NodeHandle& nh);
  bool loadIndirectAddresses(const ros::NodeHandle& nh);

  dynamixel_ros_control::DynamixelDriver& driver_;

  uint8_t id_;
  uint16_t model_number_;
  std::string model_name_;

  std::map<std::string, ControlTableItem> control_table_;
  std::vector<IndirectAddressInfo> indirect_addresses_;
};

}

#endif
