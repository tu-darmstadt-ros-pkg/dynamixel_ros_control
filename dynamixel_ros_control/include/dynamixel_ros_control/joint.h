#ifndef DYNAMIXEL_ROS_CONTROL__JOINT_H
#define DYNAMIXEL_ROS_CONTROL__JOINT_H

#include <dynamixel_ros_control/dynamixel.h>

namespace dynamixel_ros_control {

struct State
{
  State() : position(0), velocity(0), effort(0) {}
  double position;
  double velocity;
  double effort;
};

struct Joint {
  Joint(std::string _name, uint8_t id, uint16_t model_number);
  std::string name;
  Dynamixel dynamixel;
  State current_state;
  State goal_state;

  double mounting_offset;
  double offset;
};

}

#endif
