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

  double dxlValuetoPosition(int32_t dxl_value);
  int32_t positionToDxlValue(double rad);

  double dxlValueToVelocity(int32_t dxl_value);
  int32_t velocityToDxlValue(double velocity);

  double dxlValueToTorque(int32_t dxl_value);
  int32_t torqueToDxlValue(double torque);

  const ControlTableItem& getItem(std::string& name);
  uint8_t getId() const;
  uint16_t getModelNumber() const;

private:
  std::string getSeries(const ros::NodeHandle& nh);
  uint8_t id_;
  uint16_t model_number_;
  std::string model_name_;

  double ticks_per_revolution_;
  double position_to_value_ratio_;
  double velocity_to_value_ratio_;
  double torque_to_value_ratio_;

  std::map<std::string, ControlTableItem> control_table_;
};

}

#endif
