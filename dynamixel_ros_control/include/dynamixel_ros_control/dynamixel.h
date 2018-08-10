#ifndef DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_H
#define DYNAMIXEL_ROS_CONTROL_DYNAMIXEL_H

#include <ros/ros.h>
#include <dynamixel_ros_control/control_table_item.h>

namespace dynamixel_ros_control {



class Dynamixel {
public:
  Dynamixel(uint8_t id, uint16_t model_number);
  bool loadControlTable(const ros::NodeHandle& nh);

  double dxlValueToUnit(std::string register_name, int32_t value);
  bool dxlValueToBool(std::string register_name, int32_t value);

  int32_t unitToDxlValue(std::string register_name, double unit_value);
  int32_t boolToDxlValue(std::string register_name, bool b);

  const ControlTableItem& getItem(std::string& name) const;
  uint8_t getId() const;
  uint16_t getModelNumber() const;

  bool setIndirectAddress(unsigned int indirect_address_index, std::string register_name, uint16_t& indirect_data_address);

private:
  std::string getSeries(const ros::NodeHandle& nh) const;
  bool loadUnitConversionRatios(const ros::NodeHandle& nh);
  uint8_t id_;
  uint16_t model_number_;
  std::string model_name_;

  std::map<std::string, ControlTableItem> control_table_;
};

}

#endif
