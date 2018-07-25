#ifndef DYNAMIXEL_ROS_CONTROL_JOINT_H
#define DYNAMIXEL_ROS_CONTROL_JOINT_H

#include <dynamixel_ros_control/dynamixel.h>

namespace dynamixel_ros_control {

struct State
{
  State() : position(0), velocity(0), effort(0), torque(false) {}
  double position;
  double velocity;
  double effort;
  bool torque;
};

enum ControlMode {
  CURRENT,
  VELOCITY,
  POSITION,
  EXTENDED_POSITION,
  CURRENT_BASED_POSITION,
  PWM
};

struct Joint {
  Joint(std::string _name, uint8_t id, uint16_t model_number);
  std::string name;
  Dynamixel dynamixel;
  State current_state;
  State goal_state;

  ControlMode control_mode;
  double mounting_offset;
  double offset;

  bool read_position;
  bool read_velocity;
  bool read_effort;
};

}

#endif
