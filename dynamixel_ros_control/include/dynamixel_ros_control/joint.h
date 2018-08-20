#ifndef DYNAMIXEL_ROS_CONTROL_JOINT_H
#define DYNAMIXEL_ROS_CONTROL_JOINT_H

#include <dynamixel_ros_control/dynamixel.h>

#include <dynamixel_ros_control/dynamixel_driver.h>

namespace dynamixel_ros_control {

struct State
{
  State() : position(0), velocity(0), effort(0), torque(false) {}
  double position;
  double velocity;
  double effort;
  bool torque;
};

class Joint {
public:
  Joint(std::string _name, uint8_t id, uint16_t model_number, dynamixel_ros_control::DynamixelDriver& driver);

  std::string name;
  Dynamixel dynamixel;

  State current_state;
  State goal_state;

  double mounting_offset;
  double offset;

  ControlMode getControlMode() const;
  bool setControlMode(const ControlMode& value);

private:
  ControlMode control_mode;

};

}

#endif
